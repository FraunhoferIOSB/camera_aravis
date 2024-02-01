^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package camera_aravis
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.0.5 (2024-02-01)
------------------
* Added launch parameters allowing to manually set white balance
* Launch configuration, scripts and adjustments to allow raspawning
* Moved execution of set latch command for ptp
* Added parameter init_params_from_dyn_reconfigure and minor bugfix
* Read and publish camera diagnostics 
* Improved support for Precision Time Protocol (PTP)
* Minor bugfixes
  * Opened device output
  * Resetting ptp timestamp
* Updated documentation in README.md
* Contributors: Boitumelo Ruf

4.0.4 (2022-12-23)
------------------
* Update package maintainer
* Refactor node params (`#21 <https://github.com/FraunhoferIOSB/camera_aravis/issues/21>`_)
  * Refactor node params
  * Rename extended_camera_info\_ -> pub_ext_camera_info\_
  * Move stream parameters to the top of onInit()
* fix: only reset PTP clock when in "Faulty" or "Disabled" state (`#23 <https://github.com/FraunhoferIOSB/camera_aravis/issues/23>`_)
* Update industrial_ci default branch to main
* Contributors: Dominik Kleiser, Peter Mortimer, Ruf, Boitumelo

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
