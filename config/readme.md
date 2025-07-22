# my config beyond this point

---

already working features:
- `T<n>` commands supplied with extra X Y Z parameters to restore to.
- `error_gcode` toolchange failure will automatically raise an error, undo the toolchange, save temps and pause.<br>
  Is able to recover with UI and adjusting docking position from that said UI (and retest or just do manually)
- `offset calibration` Everything is done via the SVF, tool offsets,
   probe offsets and the calibration probes trigger to bottom z have thus been moved there aswell.<br>
  what that allows for? calibration at runtime, babystepping and it sticking around immediately. even after restarts.
- `lots more` there is of course a lot more, from offset history to overbuilt clean nozzle macro, take a look around, youll certainly find macros to steal.
- *(untested)* `FILAMENT_RUNOUT` using `gcode_button` to allow the selection of backup tools (or just pause on runout)
- *(untested)* `LAZY_PAUSE` allows you to pause, but lazily. (when encountering layer change, or infill)


Most adjustable settings have been moved to the `settings.cfg` and are thus easy to toggle/adjust by just editing it.
The whole config was initially intended to be compatible with the original toolchanger branch, but because of issues that dont allow to restoring to a different position, that didnt work out as planned *(atleast wouldnt have in a clean, readable manner)*

---

If you plan on testing/running this, it is of course best to make a backup of your old config. this setup requires a basic setup voron to run, or even better, previously set up tool to be present.
youll get a message prompting/informing you that stuff has been moved to the save variables file.

---
*a small note on the macros contained here*
to many it may not be too common to see things split like this, most macros rely on the "macro_helpers" to run, they will be added to the globals and are thus importable in other macros. this is mainly done as to not write the same thing 20x and also make things easier to work with.
