^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package camera_aravis
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.0.3 (2022-07-08)
------------------
* Refactor image conversion (`#20 <https://github.com/FraunhoferIOSB/camera_aravis/issues/20>`_)
* Use plain file names for includes (`#17 <https://github.com/FraunhoferIOSB/camera_aravis/issues/17>`_)
* Add verbose flag for feature detection (default = false) (`#19 <https://github.com/FraunhoferIOSB/camera_aravis/issues/19>`_)
* Assume num_streams\_ = 1 if DeviceStreamChannelCount and GevStreamChannelCount unavailable (`#18 <https://github.com/FraunhoferIOSB/camera_aravis/issues/18>`_)
* Add Line0 to Line5 to TriggerSource Enum
* Fix: nodelet namespace
* Fix: onInit deadlock
* Contributors: Dominik Kleiser, Boitumelo Ruf, Thomas Emter, Peter Mortimer, tas, Geoff McIver

4.0.2 (2022-05-04)
------------------
* Add optional ExtendedCameraInfo message to publish additional camera acquisition parameters
* Fix: Set reasonable height and width when not given in the CameraInfo
* Contributors: Peter Mortimer

4.0.1 (2022-03-25)
------------------
* Add ROS getter/setter services for camera features
* Add support for multistream encoding conversion
* Fix: Pass on the correct encoding for the additional streams of multisource cameras
* Fix: Continuously check the spawning\_ flag
* Fix: Check spawning\_ flag only once during spawnStream
* Contributors: Peter Mortimer, Thomas Emter, Dominik Kleiser

4.0.0 (2021-10-27)
------------------
* Major refactoring
* Add support for ROS Noetic and aravis-0.6
* Fix several bugs (see git history)
* Add new features:

  * Support for multisource cameras
  * Zero-copy transport with ROS nodelets
  * Camera time synchronization
  * Example launch files

* Update package author and maintainer
* Contributors: Dominik Klein, Floris van Breugel, Gaël Écorchard, Thomas Emter, Peter Mortimer, Dominik Kleiser
