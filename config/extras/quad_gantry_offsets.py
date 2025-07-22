import logging
from typing import List, Tuple, Dict, Optional
import random, numpy as np
import math

def _dbg(gcmd, label, **vals):
    """Pretty-print any number of name=value pairs via RESPOND_INFO."""
    if gcmd:
        gcmd.respond_info(f"[{label}] " + "  ".join(f"{k}={v:+.6f}" for k, v in vals.items()))

PROBE_PATTERNS = {
    1: [(0, 0)],
    2: [(-1, 0), (1, 0)],
    3: [(0, 1), (-0.866, -0.5), (0.866, -0.5)],
    4: [(-1, -1), (1, -1), (1, 1), (-1, 1)], 
    5: [(0, 0), (-1, -1), (1, -1), (1, 1), (-1, 1)],
}

_param_help = """\noptionally to overwrite config defined:\n[POINTS] [SAFE_Z] [TILT] [JITTER] [F]"""

def fit_plane(points: List[Tuple[float, float, float]]):
    if not points: return 0.0, 0.0, 0.0
    if len(points) < 3:
        return 0., 0., np.mean([p[2] for p in points])
    A = np.array([[x, y, 1] for x, y, _ in points]); b = np.array([z for _, _, z in points])
    try:
        mx, my, c = np.linalg.lstsq(A, b, rcond=None)[0]
        return float(mx), float(my), float(c)
    except np.linalg.LinAlgError: return 0.0, 0.0, 0.0

class QuadGantryOffsets:
    def __init__(self, config):
        self.config   = config
        self.printer  = config.get_printer()
        self.gcode    = self.printer.lookup_object('gcode')
        # User-configurable knobs
        self.jitter_mm  = config.getfloat('jitter',   0.1,  minval=0.0)
        self.def_pts    = config.getint ('points',    2,    minval=1, maxval=5)
        self.cfg_tilt   = config.getfloat('tilt',     5.0,  above=0.0)
        self.length_mm  = config.getfloat('length',   None, above=0.0)
        self.safe_z_mm  = config.getfloat('safe_z',   None, above=0.0)
        self.speed_xy   = config.getfloat('speed',    None, above=0.0)
        self.center_x   = config.getfloat("center_x", None)
        self.center_y   = config.getfloat("center_y", None)
        self.tilt_mm    = self.cfg_tilt
        # Runtime state
        self.current_delta = [0.0] * 4
        self._baseline: Dict[str, float] = {}
        self._last_off = (0.0, 0.0, 0.0)
        # Event + command registration
        self.printer.register_event_handler('klippy:connect', self._on_connect)
        self.gcode.register_command('QGO_SLOPED_G0', 
                                    self.cmd_QGO_SLOPED_G0, 
                                    desc=self.cmd_QGO_SLOPED_G0_help)
        self.gcode.register_command('QGO_TILT_GANTRY', 
                                    self.cmd_QGO_TILT_GANTRY, 
                                    desc=self.cmd_QGO_TILT_GANTRY_help)
        self.gcode.register_command('QGO_CALIBRATE_BASELINE', 
                                    self.cmd_QGO_CALIBRATE_BASELINE, 
                                    desc=self.cmd_QGO_CALIBRATE_BASELINE_help)
        self.gcode.register_command('QGO_CALIBRATE_OFFSET', 
                                    self.cmd_QGO_CALIBRATE_OFFSET, 
                                    desc=self.cmd_QGO_CALIBRATE_OFFSET_help)
        self.gcode.register_command('QGO_CALIBRATE_CORNERS', 
                                    self.cmd_QGO_CALIBRATE_CORNERS, 
                                    desc=self.cmd_QGO_CALIBRATE_CORNERS_help)

    def _on_connect(self):
        self.toolhead   = self.printer.lookup_object('toolhead')
        self.gcode_move = self.printer.lookup_object('gcode_move')
        self.qgl = self.printer.lookup_object('quad_gantry_level', default=None)
        if self.qgl is None:
            raise self.config.error("[quad_gantry_offsets] requires [quad_gantry_level] to be set up")

        self.probe_h = self.qgl.probe_helper
        self.zhelper = self.qgl.z_helper

        if (self.center_x is None) ^ (self.center_y is None):
            missing, req = (("center_x", "center_y") if self.center_x is None
                            else ("center_y", "center_x"))
            raise self.config.error(f"'{missing}' requires '{req}' to be set as well")

        xs, ys = zip(*self.qgl.probe_helper.probe_points)
        mid_x, mid_y = (min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0
        self.sign_x = [-1 if x < mid_x else +1 for x in xs]
        self.sign_y = [-1 if y < mid_y else +1 for y in ys]
        
        self._update_spans_and_slopes()
        (x0, y0), (x2, y2) = self.qgl.gantry_corners
        gxs, gys = [x0, x0, x2, x2], [y0, y2, y2, y0]
        self.rot_mid_x, self.rot_mid_y = (min(gxs) + max(gxs)) / 2.0, (min(gys) + max(gys)) / 2.0

        if abs(self.slope_x) < 1e-6 or abs(self.slope_y) < 1e-6:
            raise self.config.error("Gantry slopes are zero, cannot calculate. Check config.")

        if (self.tilt_mm > self.qgl.max_adjust):
            raise self.config.error(
                f"Requested ±{self.tilt_mm} mm tilt exceeds "
                f"QGL max_adjust: {self.qgl.max_adjust} mm")

        if self.center_x is None:
            self.center_x, self.center_y = mid_x, mid_y
        if self.length_mm is None:
            self.length_mm = min(self.span_x, self.span_y) / 4.0
        if self.speed_xy is None:
            self.speed_xy = self.probe_h.speed
        if self.safe_z_mm is None:
            self.safe_z_mm = self.qgl.horizontal_move_z
        self.lift_speed = self.probe_h.get_lift_speed()

    #============= HELPERS =====================================================================
    def _update_spans_and_slopes(self):
        (x0, y0), (x2, y2) = self.qgl.gantry_corners
        self._gantry_xy = [(x0, y0), (x0, y2), (x2, y2), (x2, y0)]
        self.span_x, self.span_y = abs(x2 - x0), abs(y2 - y0)  
        self.slope_x = self.tilt_mm / self.span_x if self.span_x > 1e-6 else 0
        self.slope_y = self.tilt_mm / self.span_y if self.span_y > 1e-6 else 0

    def _jit(self, x: float, y: float, r: float) -> Tuple[float, float]:
        return (x + random.uniform(-r, r), y + random.uniform(-r, r)) if r > 0 else (x, y)

    def _current_plane_offset(self, x: float, y: float) -> float:
        pts = [(gx, gy, h) for (gx, gy), h in zip(self._gantry_xy, self.current_delta)]
        mx, my, c = fit_plane(pts)
        return float(mx * x + my * y + c)
    
    def _calculate_corners_from_deltas(self, gcmd, axis: str, p1_coord: float, p2_coord: float, dZ1: float, dZ2: float) -> Tuple[float, float]:
        """Helper to perform the corner calculation math."""
        probe_span = p2_coord - p1_coord
        if abs(probe_span) < 1e-6:
            raise gcmd.error(f"Probe points for {axis}-corner measurement have same {axis} coordinate.")
        m = (dZ2 - dZ1) / probe_span
        c = dZ1 - m * p1_coord
        if abs(m) < 1e-6:
            raise gcmd.error(f"Measured zero slope for {axis}-corners. Cannot calculate.")   
        return sorted([(-self.tilt_mm / 2.0 - c) / m, (self.tilt_mm / 2.0 - c) / m])
    #============= PROBING =====================================================================
    def _measure_z_at_point(self, gcmd, x: float, y: float, safe_z: float, jitter: float) -> float:
        probe_obj = self.printer.lookup_object('probe', None)

        xj, yj = self._jit(x, y, jitter)
        self.tilted_move(x=xj, y=yj, z=safe_z, speed=self.speed_xy)
        p_sess = probe_obj.start_probe_session(gcmd)
        p_sess.run_probe(gcmd)
        epos = p_sess.pull_probed_results()[0]
        p_sess.end_probe_session()
        self.toolhead.wait_moves()
        #status = probe_obj.get_status(self.printer.get_reactor().monotonic())
        #measured_z = status.get('last_z_result', 0.0)
        self.tilted_move(z=safe_z, speed=self.lift_speed)
        return float(epos[2])
        #return float(measured_z)

    def _measure_z_by_line_fit(self, gcmd, probe_axis: str, num_points: int, length: float, safe_z: float, jitter: float, center_x: float, center_y: float) -> float:
        """Probes a line and returns avreage"""
        if num_points == 1: 
            return self._measure_z_at_point(gcmd, center_x, center_y, safe_z, jitter)
        dir_x, dir_y = (1, 0) if probe_axis == 'X' else (0, 1)
        offsets = np.linspace(-length / 2.0, length / 2.0, num_points)
        probed_points = []
        for offset in offsets:
            px = center_x + offset * dir_x
            py = center_y + offset * dir_y
            pz = self._measure_z_at_point(gcmd, px, py, safe_z, jitter)
            probed_points.append((offset, pz))
        p_coords = [p[0] for p in probed_points]
        z_coords = [p[1] for p in probed_points]
        _, intercept = np.polyfit(p_coords, z_coords, 1)
        return float(intercept)

    def _measure_z_by_plane_fit(self, gcmd, axis, pts, length, safe_z, jitter, cx, cy) -> float:
        """Probes a 2D pattern and returns the Z-height at the center, corrected for bed plane."""
        if pts < 3:
            return self._measure_z_by_line_fit(gcmd, axis, pts, length, safe_z, jitter, cx, cy)
        gcmd.respond_info(f"Probing {pts}-point pattern with length {length:.2f}mm to fit a plane...")
        pattern = PROBE_PATTERNS.get(pts, PROBE_PATTERNS[3])
        probed_points = []
        for p_offset in pattern:
            px = cx + p_offset[0] * length / 2.0
            py = cy + p_offset[1] * length / 2.0
            pz = self._measure_z_at_point(gcmd, px, py, safe_z, jitter)
            probed_points.append((px, py, pz))
        mx, my, c = fit_plane(probed_points)
        return float(mx * cx + my * cy + c)

    def _tilt(self, X: float = 0.0, Y: float = 0.0, restore_z: bool = False):
        if restore_z:
            start_pos = self.toolhead.get_position()
            start_offset = self._current_plane_offset(start_pos[0], start_pos[1])
            z_nozzle_before_tilt = start_pos[2] + start_offset
        raw_delta = [0.5 * (self.sign_y[i] * X + self.sign_x[i] * Y) for i in range(4)]
        adjustments = [float(raw_i - current_i) for raw_i, current_i in zip(raw_delta, self.current_delta)]
        if not any(abs(v) > 1e-6 for v in adjustments): 
            return
        self.zhelper.adjust_steppers(adjustments, self.lift_speed)
        self.current_delta = raw_delta
        if restore_z:
            self.tilted_move(z=z_nozzle_before_tilt, speed=self.lift_speed)

    def tilted_move(self, *, x: Optional[float] = None, y: Optional[float] = None, z: Optional[float] = None, speed: Optional[float] = None):
        cx, cy, cz_actuator = self.toolhead.get_position()[:3]   
        final_speed = speed if speed is not None else self.gcode_move.get_status(self.printer.get_reactor().monotonic())['speed'] 
        tx = x if x is not None else cx
        ty = y if y is not None else cy  
        offset_cur = self._current_plane_offset(cx, cy)
        cz_nozzle = cz_actuator + offset_cur   
        tz_nozzle = z if z is not None else cz_nozzle  
        offset_tgt = self._current_plane_offset(tx, ty)
        tz_actuator = tz_nozzle - offset_tgt  
        self.toolhead.manual_move([float(tx), float(ty), float(tz_actuator)], float(final_speed))

    #============= GCODE COMMANDS =====================================================================
    def _params(self, gcmd, baseline: bool = False) -> tuple[int, float, float, float, float, float]:
        pts    = gcmd.get_int  ('POINTS',   self.def_pts,       minval=1, maxval=5)
        length = gcmd.get_float('LENGTH',   self.length_mm,     above=0.0)
        safe_z = gcmd.get_float('SAFE_Z',   self.safe_z_mm,     above=0.0)
        tilt   = gcmd.get_float('TILT',     self.cfg_tilt,      above=0.0, maxval=self.qgl.max_adjust)
        jitter = gcmd.get_float('JITTER',   self.jitter_mm,     minval=0.0)
        speed  = gcmd.get_float('F',        self.speed_xy * 60, above=0.0)
        cx     = gcmd.get_float('CENTER_X', self.center_x)
        cy     = gcmd.get_float('CENTER_Y', self.center_y)
        if gcmd.get_int('SPAN', 0):
            gcmd.respond_info(f"SPAN parameter set, probing at gantry center of rotation ({self.rot_mid_x:.2f}, {self.rot_mid_y:.2f})")
            cx, cy = self.rot_mid_x, self.rot_mid_y
        self.speed_xy = speed / 60.0
        if baseline and self.tilt_mm != tilt:
            self.tilt_mm = tilt
            self._update_spans_and_slopes()
        elif not baseline and abs(self.tilt_mm - tilt) > 1e-6:
            raise gcmd.error(f"TILT mismatch. Baseline was recorded with {self.tilt_mm}mm tilt, but command specified {tilt}mm.")
        return pts, length, safe_z, jitter, cx, cy
    
    cmd_QGO_SLOPED_G0_help = ("Move with gantry-tilt compensation.\n"
                              "Syntax: QGO_SLOPED_G0 [X=<mm>] [Y=<mm>] [Z=<mm>] [F=<mm/min>]")
    def cmd_QGO_SLOPED_G0(self, gcmd):
        feed = gcmd.get_float('F', None, above=0.0)
        self.tilted_move(
            x=gcmd.get_float('X', None),
            y=gcmd.get_float('Y', None),
            z=gcmd.get_float('Z', None),
            speed=feed / 60.0 if feed is not None else None
        )

    cmd_QGO_TILT_GANTRY_help = ("Tilt the gantry: [X=<mm>] [Y=<mm>] [RESTORE_Z=0/1]\n"
                                "Rotates around provided axis, RESTORE_Z to restore the nozzle's Z position.")
    def cmd_QGO_TILT_GANTRY(self, gcmd):
        self._tilt(
            X=gcmd.get_float('X', 0.0),
            Y=gcmd.get_float('Y', 0.0),
            restore_z=bool(gcmd.get_int('RESTORE_Z', 0))
        )

    cmd_QGO_CALIBRATE_BASELINE_help = ("Collects baseline Z-height data." + _param_help)
    def cmd_QGO_CALIBRATE_BASELINE(self, gcmd):
        z_flat, z_x_slope, z_y_slope = self._perform_calibration_sequence(gcmd, baseline=True)
        self._baseline = {'f': z_flat, 'x': z_x_slope, 'y': z_y_slope}
        _dbg(gcmd, "BASELINE", Z_flat=z_flat, Z_x_slope=z_x_slope, Z_y_slope=z_y_slope)

    cmd_QGO_CALIBRATE_OFFSET_help = ("Calculates and reports the tool's offset against the baseline." + _param_help)
    def cmd_QGO_CALIBRATE_OFFSET(self, gcmd):
        if not self._baseline:
            raise gcmd.error("No baseline data. Run QGO_CALIBRATE_BASELINE first.")
        
        z_flat_new, z_x_slope_new, z_y_slope_new = self._perform_calibration_sequence(gcmd, baseline=False)
        
        dZ_x_base = self._baseline['x'] - self._baseline['f']
        dZ_y_base = self._baseline['y'] - self._baseline['f']
        dZ_x_new = z_x_slope_new - z_flat_new
        dZ_y_new = z_y_slope_new - z_flat_new
        
        err_delta_x = dZ_x_new - dZ_x_base
        err_delta_y = dZ_y_new - dZ_y_base
            
        dx = err_delta_x / self.slope_x
        dy = err_delta_y / self.slope_y
        dz = z_flat_new - self._baseline['f']
        
        self._last_off = (dx, dy, dz)
        _dbg(gcmd, "CALCULATION_INPUTS", dZ_x_base=dZ_x_base, dZ_y_base=dZ_y_base,
             dZ_x_new=dZ_x_new, dZ_y_new=dZ_y_new, slope_x=self.slope_x, slope_y=self.slope_y, 
             Z_flat=z_flat_new, Z_x_slope=z_x_slope_new, Z_y_slope=z_y_slope_new)
        gcmd.respond_info(f"Calculated Offsets: X={dx:+.4f} Y={dy:+.4f} Z={dz:+.4f}")

    def _perform_calibration_sequence(self, gcmd, baseline: bool) -> Tuple[float, float, float]:
        """Probes in three states (flat, x-slope, y-slope) using line fits."""
        pts, length, safe_z, jitter, cx, cy = self._params(gcmd, baseline=baseline)

        mode = gcmd.get('MODE', 'PLANE')
        if mode == 'PLANE':
            measurement = self._measure_z_by_plane_fit
        elif mode == 'LINE':
            measurement = self._measure_z_by_line_fit
        else:
            raise gcmd.error(f"Mode: '{mode}' not in ['PLANE', 'LINE'] ")
        
        self._tilt()
        z_flat =    measurement(gcmd, 'Y', pts, length, safe_z, jitter, cx, cy)
        self._tilt(Y=self.tilt_mm)
        z_x_slope = measurement(gcmd, 'Y', pts, length, safe_z, jitter, cx, cy)
        self._tilt(X=self.tilt_mm)
        z_y_slope = measurement(gcmd, 'X', pts, length, safe_z, jitter, cx, cy)
        self._tilt()
        return z_flat, z_x_slope, z_y_slope

    cmd_QGO_CALIBRATE_CORNERS_help = "Measures the effective gantry corner locations."
    def cmd_QGO_CALIBRATE_CORNERS(self, gcmd):
        gcmd.respond_info(f"--- STARTING GANTRY CORNER CALIBRATION (TILT={float(self.tilt_mm)}mm) ---")
        pts, length, safe_z, jitter, cx, cy = self._params(gcmd, baseline=True)
        
        p_arr = self.qgl.probe_helper.probe_points
        p0, p1, p2, p3 = p_arr[0], p_arr[1], p_arr[2], p_arr[3]
        left_mid  = ((p0[0] + p1[0]) / 2.0, (p0[1] + p1[1]) / 2.0)
        right_mid = ((p2[0] + p3[0]) / 2.0, (p2[1] + p3[1]) / 2.0)
        front_mid = ((p0[0] + p3[0]) / 2.0, (p0[1] + p3[1]) / 2.0)
        back_mid  = ((p1[0] + p2[0]) / 2.0, (p1[1] + p2[1]) / 2.0)

        self._tilt()
        z_lm_flat = self._measure_z_by_line_fit(gcmd, 'Y', pts, length, safe_z, jitter, left_mid[0],  left_mid[1])
        z_rm_flat = self._measure_z_by_line_fit(gcmd, 'Y', pts, length, safe_z, jitter, right_mid[0], right_mid[1])
        z_fm_flat = self._measure_z_by_line_fit(gcmd, 'X', pts, length, safe_z, jitter, front_mid[0], front_mid[1])
        z_bm_flat = self._measure_z_by_line_fit(gcmd, 'X', pts, length, safe_z, jitter, back_mid[0],  back_mid[1])
 
        gcmd.respond_info("Measuring X-Corners (tilted around Y)")
        self._tilt(Y=self.tilt_mm)
        z_lm_tilted = self._measure_z_by_line_fit(gcmd, 'Y', pts, length, safe_z, jitter, left_mid[0],  left_mid[1])
        z_rm_tilted = self._measure_z_by_line_fit(gcmd, 'Y', pts, length, safe_z, jitter, right_mid[0], right_mid[1])
        
        gcmd.respond_info("Measuring Y-Corners (tilted around X)")
        self._tilt(X=self.tilt_mm)
        z_fm_tilted = self._measure_z_by_line_fit(gcmd, 'X', pts, length, safe_z, jitter, front_mid[0], front_mid[1])
        z_bm_tilted = self._measure_z_by_line_fit(gcmd, 'X', pts, length, safe_z, jitter, back_mid[0],  back_mid[1])

        dz_l, dz_r = z_lm_tilted - z_lm_flat, z_rm_tilted - z_rm_flat
        dz_f, dz_b = z_fm_tilted - z_fm_flat, z_bm_tilted - z_bm_flat
        calc_x = self._calculate_corners_from_deltas(gcmd, 'X', left_mid[0],  right_mid[0], dz_l, dz_r)
        calc_y = self._calculate_corners_from_deltas(gcmd, 'Y', front_mid[1], back_mid[1],  dz_f, dz_b)

        self._tilt(restore_z=True)
        gcmd.respond_info("--- GANTRY CORNER CALIBRATION COMPLETE ---\n" \
                        "Update your [quad_gantry_level] config with these values if they differ significantly:\n"\
                        f"gantry_corners:\n   {float(calc_x[0]):.2f}, {float(calc_y[0]):.2f}\n   {float(calc_x[1]):.2f}, {float(calc_y[1]):.2f}")
        
    def get_status(self, eventtime):
        dx, dy, dz = self._last_off
        return {
            'baseline_set': bool(self._baseline),
            'last_x_result': float(dx),
            'last_y_result': float(dy),
            'last_z_result': float(dz)
        }

def load_config(config):
    return QuadGantryOffsets(config)