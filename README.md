# klipper-toolchanger-experimental

Aimed to be fully compatible with the original `klipper-toolchanger` while offering more features, extras, and options to expand upon.
Basically, the "assortment of Klipper extensions" from viesturz, with the "assortment of Klipper extensions" from me.

# Installation

To install this plugin, run the installation script using the following command over SSH. This script will download this GitHub repository to your RaspberryPi home directory, and symlink the files in the Klipper extra folder.

```
wget -O - https://raw.githubusercontent.com/Contomo/klipper-toolchanger-experimental/main/install.sh | bash
```


## Updates that add new files

Note that if an update has new klipper files, they **will not** be automatically installed into Klipper.
You will need to run the intall script manualy to add them:
```commandline
bash ~/klipper-toolchanger-experimental/install.sh
```

# Components

* [Toolchanger](/toolchanger.md) - tool management support.
* [Tool probe](/tool_probe.md) - per tool Z probe.
* [Rounded path](/rounded_path.md) - rounds the travel path corners for fast non-print moves.
* [Tools calibrate](/tools_calibrate.md) - support for contact based XYZ offset calibration probes.
* [Heater_power_distributor](/heater_power_distributor.md) - dynamic heater group power limiting
