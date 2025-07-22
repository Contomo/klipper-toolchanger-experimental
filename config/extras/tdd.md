
# Accellerometer polling/tool drop detection

## objects/stats avalible inside of printer.tool_drop_detection object

- **rotation**
rotation{'pitch':0.000,'yaw':0.000}
- **magnitude** -> total acceleration, absolute in g. avalible as:
'magnitude': 0.000
- **vector** -> the data retrieved from our accel. avalible as:
'vector':{'x':0.000,'y':0.000,'z':0.000}
- **session** -> data from our start stop session, peak, norm and current in g abs.
'session':{'peak': 0.000, 'norm': 0.000, 'current': 0.000}

## config

### general
- sample_results: [ median | average ] (default: median) -> *norm* (the way to calculate our current accel.)
- decimals: [ 0 - 10 ] (default: 3) -> *the amount of decimals to pack into our objects*
- accelerometer: [comma seperated names] (default: none)

### crash/drop detection
- peak_g_threshold: [ 0.1 - 25 ] (default: 5) -> *if defined enables also triggering crash gcode when a peak higher than this is detected (instant)*
- rotational thresholding *(the threshold in rotation at which to trigger the crash gcode)*
 - either:  rotation_threshold: [0.0 - 180.0] (±abs, vector to vector)
 - or:      pitch_threshold and/or roll_threshold (±abs, actual rotation angles)
- crash_mintime: [ 0.0 - 100.0 ] (default 1.0) -> *how long the angle has to be exceeded to be considered dropped* (high g events remain instant)
- crash_gcode: gcode template to be executed when THRESHOLD exceeded. provided with extra context: [ie: 'ACCEL':T1]

### unrelated/toys *(will always use pitch_threshold, roll_threshold)*
- angle_exceed: gcode template to be ran when the angle gets exceeded
- angle_return: gcode template to be ran when the angle returns to normal.
- hysterisis: [0.0-180.0] (default: 5.0) Hysterisis between those two templates executing.

(unsure if this works?)
- polling_freq: [1-max] (default: 1) -> *the frequency at which at ask the mcu for values*
- polling_rate: [see adxl345] (default: 50) -> *the frequency the accelerometer is spitting out values to mcu*

## commands

# ---[ testing ]
 - TDD_QUERY_ASYNC/TDD_QUERY [/ACCEL=NAME/]
-> single query, either querying all or just those provided. comma seperated list.
action: responds with the rotation, magnitude and vector in the console,
all rounded to two decimals without updating the tool_drop_detection object.

 - TDD_DUMP_ROTATIONS [/ACCEL=NAME/]
-> single query, either querying all or just those provided. comma seperated list replied to copy into config.
default_$NAME$: [g:?.??, p:?.??, r:?.??]


# ---[ drop detection ]
 - TDD_STOP [/ACCEL=NAME/]
-> stops tool drop detection for that tool or all

 - TDD_START [ACCEL=NAME] ([LIMIT_PITCH=0.0-180.0] [LIMIT_ROLL=0.0-180.0]) or [LIMIT_ANGLE=0.0-180.0] optional: [/CRASH_MINTIME=0.0-100.0/] [/LIMIT_G=0.0-25/]
where LIMIT_ANGLE is the angle between the two vectors. limits again relative to our config objects previously set by obtaining an offset to our 0.0.0 angles with TDD_DUMP_ROTATIONS
-> starts tool drop detection for that tool or all, everything provided optional to config.

# ---[ polling ]
 - TDD_POLLING_START [/ACCEL/] [/FREQ/] [/RATE/]
-> starts the high speed polling for that tool or all, freq, interval provided to overwrite internal settings.
will update the info in session accordingly. 
will also update the acceleration vector, the magnitutde, and the rotation that can be retrieved.

 - TDD_POLLING_RESET [/ACCEL/]
-> resets the info in session for that tool or all.

 - TDD_POLLING_STOP [/ACCEL/]
-> stops the polling for that tool or all.












    # ─── SINGLE QUERIES  ─────────────────────────────────────────────────
    def _cmd_query(self, gcmd):
        lines: List[str] = []
        for n in self._targets(gcmd):
            vec = self.readers[n].grab()
            if vec is None:
                lines.append(f"ACCEL_{n}: no-data"); continue
            gx, gy, gz = (_mm_to_g(v) for v in vec)
            mag = math.sqrt(gx*gx + gy*gy + gz*gz)
            p, r = _euler(vec)
            lines.append(f"ACCEL_{n}: x{gx:7.2f} g y{gy:7.2f} g z{gz:7.2f} g |a|={mag:.2f} g pitch={p:.2f}° roll={r:.2f}°")
        gcmd.respond_info("\n".join(lines))

    def _cmd_query_async(self, gcmd):
        tgt = self._targets(gcmd)
        reactor = self.printer.get_reactor()
        def work(ev):
            buf: List[str] = []
            for n in tgt:
                vec = self.readers[n].grab()
                if vec is None:
                    buf.append(f"ACCEL_{n}: no-data"); continue
                gx, gy, gz = (_mm_to_g(v) for v in vec)
                mag = math.sqrt(gx*gx + gy*gy + gz*gz)
                p, r = _euler(vec)
                buf.append(f"ACCEL_{n}: x{gx:7.2f} g y{gy:7.2f} g z{gz:7.2f} g |a|={mag:.2f} g pitch={p:.2f}° roll={r:.2f}°")
            self.gcode.respond_info("\n".join(buf))
            return reactor.NEVER
        reactor.register_timer(work, reactor.monotonic())
        gcmd.respond_info(f"TDD_QUERY_ASYNC queued ({len(tgt)})")

   
    def _cmd_dump_rot(self, gcmd):
        results = []
        for n in self._targets(gcmd):
            p = self.pollers.get(n)

            # Use poller history only if we already have >= 1 s of data
            if p and len(p.rot_win) >= p.rot_win.maxlen:
                data = list(p.rot_win)
                if self.sample_results == 'median':
                    gm = statistics.median(p.win)
                else:
                    gm = statistics.fmean(p.win)
            else:
                data, mags = [], []
                # one shot fallback: sample DWELL_GRAB repeatedly for 1s
                start = time.monotonic()
                while time.monotonic() - start < _WINDOW_SEC:
                    vec = self.readers[n].grab(_DWELL_GRAB)
                    if not vec: 
                        continue
                    data.append(_euler(vec))
                    gx, gy, gz = (_mm_to_g(v) for v in vec)
                    mags.append(math.sqrt(gx*gx + gy*gy + gz*gz))

                gm = (statistics.median(mags) if mags and self.sample_results == 'median'
                      else statistics.fmean(mags) if mags else 0.0)
            if not data:
                results.append(f"default_{n}: no data")
                continue
            # separate lists
            ps = [pr for pr, rr in data]
            rs = [rr for pr, rr in data]
            pm = statistics.median(ps) if self.sample_results == 'median' else statistics.fmean(ps)
            rm = statistics.median(rs) if self.sample_results == 'median' else statistics.fmean(rs)
            results.append(f"default_{n}: [g:{gm:.2f}, p:{pm:.2f}, r:{rm:.2f}]")
        gcmd.respond_info("\n".join(results))
