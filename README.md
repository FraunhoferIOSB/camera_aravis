# camera_aravis

Actively maintained repository for the ROS1 camara_aravis driver. It is open source under the LGPL (like Aravis itself).

The [Aravis](http://live.gnome.org/Aravis) library is a glib/gobject based library for video acquisition using Genicam cameras. It currently implements the gigabit ethernet and USB3 protocols used by industrial cameras.

The camera_aravis driver has long history of multiple forks and now abandoned GitHub repositories. This repository is based on https://github.com/florisvb/camera_aravis.git, which in turn was forked from a deleted github repo (https://github.com/CaeruleusAqua/camera_aravis), which was itself forked from https://github.com/ssafarik/camera_aravis.

------------------------

Tested with Aravis version 0.6.X. Since Ubuntu 20.04 the library can be installed from the official Ubuntu package repository. Install with:

```
sudo apt install libaravis-dev
```

The basic command to run camera_aravis:

	$ rosrun camera_aravis cam_aravis

To run it in a given namespace:

	$ ROS_NAMESPACE=cam1 rosrun camera_aravis cam_aravis

------------------------
## Continuous Integration

| Service    | Noetic  | Master |
| ---------- | ------- | ------ |
| GitHub     | [![build](https://github.com/FraunhoferIOSB/camera_aravis/actions/workflows/industrial_ci_action.yml/badge.svg?branch=noetic-devel)](https://github.com/FraunhoferIOSB/camera_aravis/actions/workflows/industrial_ci_action.yml/badge.svg?branch=noetic-devel)    | [![build](https://github.com/FraunhoferIOSB/camera_aravis/actions/workflows/industrial_ci_action.yml/badge.svg?branch=master)](https://github.com/FraunhoferIOSB/camera_aravis/actions/workflows/industrial_ci_action.yml/badge.svg?branch=master) |
| ROS Build Farm | [![build](https://build.ros.org/job/Ndev__camera_aravis__ubuntu_focal_amd64/6/badge/icon?style=plastic&subject=build)](https://build.ros.org/job/Ndev__camera_aravis__ubuntu_focal_amd64/6/)   | N/A |

## Configuration

This ROS node publishes messages image_raw and camera_info for a specified camera.  It supports
a variety of camera features via the ROS reconfigure_gui, including the following:
* ExposureAuto         (string: Off, Once, Continuous)
* GainAuto             (string: Off, Once, Continuous)
* ExposureTimeAbs      (float)
* Gain                 (float)
* AcquisitionMode      (string: Continuous, SingleFrame, MultiFrame)
* AcquisitionFrameRate (float)
* TriggerMode          (string: Off, On)
* TriggerSource        (string: Any, Software, Line0, Line1, Line2)
* softwaretriggerrate  (float)
* frame_id             (string)
* FocusPos             (integer)
* mtu                  (integer)

Note that the above are also the ROS parameter names of their respective feature.  You may
set initial values for the camera by setting ROS parameters in the camera's namespace.

In addition to the above features, this driver now supports (almost) every feature of every camera,
you just have to know how the feature is specified; each GenICam-based camera contains
an XML file onboard, and by viewing this file you can determine which ROS parameters to set
for camera_aravis to write to the camera.  You can use arv-tool-0.2 to see the feature list
and the XML file (e.g. "arv-tool-0.2 --name=Basler-21285878 features")

Note that for this special feature access, the ROS parameter type must match the feature type.
For example, a Basler ac640 has a boolean feature called "GammaEnable", an integer feature
called "BlackLevelRaw", and a string enum feature called "PixelFormat" that takes values
(Mono8, Mono12, Mono12Packed, YUV422Packed, etc).  The ROS params that you set for these
must be, respectively, a bool, an integer and a string.  Also note that boolean features must
be specified as ROS params false/true, not as integer 0/1.

	$ rosparam set /camera_aravis/GammaEnable false
	$ rosparam set /camera_aravis/BlackLevelRaw 5
	$ rosparam set /camera_aravis/PixelFormat Mono12
	$ rosrun camera_aravis cam_aravis


------------------------
camera_aravis supports multiple cameras, each of which may be specified on the
command-line, or via parameter.  Runs one camera per node.

To specify which camera to open, via the command-line:

	$ rosrun camera_aravis cam_aravis _guid:=Basler-21237813


To specify which camera to open, via a parameter:

	$ rosparam set /camera_aravis/guid Basler-21237813
	$ rosrun camera_aravis cam_aravis


------------------------
It supports the dynamic_reconfigure protocol, and once the node is running, you may adjust
its parameters by running the following and then manipulating the GUI:

	$ rosrun dynamic_reconfigure reconfigure_gui


------------------------
There is an additional nice feature related to timestamps that unifies ROS time with camera time.
We want a stable timestamp on the images that the camera delivers, giving a nice smooth time
delta from frame to frame.  If we were to use the ROS clock on the PC, by the time we get the
image packets from the camera a variable amount of time has passed on the PC's clock due to
variable network and system delays.  The camera's onboard clock is stable but it doesn't match
with the ROS clock on the PC, and furthermore since it comes from a different piece of hardware,
the two clock's rates are slightly different.

The solution is to start with a base of ROS time, and to accumulate the dt's from the camera clock.
To accommodate the difference in clock rates, a PID controller gently pulls the result toward
ROS time.

### Activating PTP Timestamp

Some cameras support the use of the Precision Time Protocol (PTP) to set the timestamps of the 
captured images. To activate it using camera_aravis a couple of launch parameters are available:

- ```use_ptp_timestamp```: General switch to activate the use of the PTP timestamp within
camera_aravis. Set to ```true``` to activate.
	- Type: ```bool```
	- Default: ```false```
- ```ptp_enable_feature_name```: Feature name on the camera device to enable the use of PTP.
	- Type: ```string```
	- Default: ```"GevIEEE1588"```
- ```ptp_status_feature_name```: Feature name on the camera device to access the status of the PTP.
This is needed to monitor, whether camera_aravis needs to reset the PTP clock.
	- Type: ```string```
	- Default: ```"GevIEEE1588Status"```
- ```ptp_set_cmd_feature_name```: Feature name of the 'Set-Command' on the camera device for PTP. 
On some cameras a 'set' or 'synchronization' command needs to be executed after setting the features
above for the PTP to be activated. If this launch parameter is set, the corresponding command will
be executed after the parameters above are set.
	- Type: ```string```
	- Default: ```""```

## Publishing camera diagnostics / status

Camera_aravis allows to periodically monitor custom camera features and publish them in a designated
topic named ```~/diagnostics``` in a message type as specified in 
[CameraDiagnostics.msg](msg/CameraDiagnostics.msg). In order to configure and customize this 
status monitoring, two launch parameters are provided:

- ```diagnostic_publish_rate```: Rate at which to read and publish the diagnostic data.
	- Type: ```double```
	- Default: ```0.1``` (10 seconds)
- ```diagnostic_yaml_url```: URL to yaml file specifying the camera features which are to be 
monitored. If left empty (as default) no diagnostic features will be read and published.
	- Type: ```string```
	- Default: ```""```

An example of such a diagnostic yaml file is given in 
[camera_diagnostics.yaml](launch/camera_diagnostics.yaml). This file should hold a list of 
```FeatureName``` together with a corresponding ```Type``` (bool, float, int, or string) for each
feature which is to be monitored. If a feature is associated with a feature selector, one can 
additionally specify a list of ```Selectors```. Each entry in this list should again have a 
```FeatureName``` and ```Type```, as well as a ```Value``` to set.

For each feature a key-value pair is constructed and published in the ```data``` field of the 
message stated above. If a feature as a list of selectors, one key-value pair is constructed for
each Feature-Selector pair.

## Known Issues

### Slow read of white balance and black level values

From [PR#22](https://github.com/FraunhoferIOSB/camera_aravis/pull/22): The white balance and black 
level values of some cameras (e.g. Basler acA2440-20gc & JAI FS-3200D-10GE ) can be read more 
efficiently by reading from the exact memory locations of the camera instead of using the Selector 
features.

However, since we are not sure on how stable these optimizations based on exact memory locations are 
in terms of firmware updates and how well they generalize to other camera models, we have refrained 
from merging the pull request into the main repository.

For more information and details on the implementation, please look into the changes and the 
comments inside the pull request.

