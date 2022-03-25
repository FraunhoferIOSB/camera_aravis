^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package camera_aravis
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
