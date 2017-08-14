camera_aravis

This is a [ROS](http://ros.org) package for the [Aravis GigEVision
library](http://live.gnome.org/Aravis). It is open source, under the
LGPL (like Aravis itself).


------------------------
The basic command to run camera_aravis:

	$ rosrun camera_aravis camnode

To run it in a given namespace, which is the better way to do it:

	$ ROS_NAMESPACE=cam1 rosrun camera_aravis camnode


------------------------
This ROS node publishes messages image_raw and camera_info for a specified camera.  It supports 
a variety of camera features via the ROS reconfigure_gui, including the following:
* ExposureAuto         (string: Off, Once, Continuous)
* GainAuto             (string: Off, Once, Continuous)
* ExposureTimeAbs      (float)
* Gain                 (float)
* AcquisitionMode      (string: Continuous, SingleFrame, MultiFrame)
* AcquisitionFrameRate (float)
* TriggerMode          (string: Off, On)
* TriggerSource        (string: Software, Line1, Line2)
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

	$ rosparam set cam1/GammaEnable false
	$ rosparam set cam1/BlackLevelRaw 5
	$ rosparam set cam1/PixelFormat Mono12
	$ ROS_NAMESPACE=cam1 rosrun camera_aravis camnode


------------------------
camera_aravis supports multiple cameras, each of which may be specified on the 
command-line, or via parameter.  Runs one camera per node.

To specify which camera to open, via the command-line:

	$ ROS_NAMESPACE=cam1 rosrun camera_aravis camnode Basler-21237813


To specify which camera to open, via a parameter:

	$ rosparam set cam1/guid Basler-21237813
	$ ROS_NAMESPACE=cam1 rosrun camera_aravis camnode


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
To accomodate the difference in clock rates, a PID controller gently pulls the result toward 
ROS time.


