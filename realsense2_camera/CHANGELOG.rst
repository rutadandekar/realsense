^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package realsense2_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

100.223.4 (2020-09-16)
----------------------
* Support for topic monitoring and cleanly shutting down or resetting driver without killing nodelet manager. `#28 <https://github.com/plusone-robotics/realsense/issues/28>`_
* Contributors: Isaac I.Y. Saito, Marc Alban

100.223.3 (2019-05-02)
----------------------
* [fix] reset feature didn't function when spawning multiple realsense cameras `#26 <https://github.com/plusone-robotics/realsense/issues/26>`_
* Contributors: Abhijit Majumdar

100.223.2 (2019-04-29)
----------------------
* [capability] add decimation filter at the front of the filter list, before the start of disparity filter `#24 <https://github.com/plusone-robotics/realsense/issues/24>`_  
* Contributors: Abhijit Majumdar, Isaac I.Y. Saito

100.223.1 (2019-04-09)
----------------------
* Plus One Robotics branch synced with the upstream v2.2.3 https://github.com/plusone-robotics/realsense/pull/23

2.2.3 (2019-04-xx)
------------------

2.2.2 (2019-04-xx)
------------------

100.1.0 (2019-04-03)
--------------------
* Changes to add ir-rgb frame `#21 <https://github.com/plusone-robotics/realsense/issues/21>`_ 
* Contributors: Abhijit Majumdar, Isaac I.Y. Saito

2.2.1 (2019-03-07)
------------------
* update version - 2.2.1
* Merge branch 'ibaranov-cp-development' into development
* Merge branch 'development' of https://github.com/ibaranov-cp/realsense into ibaranov-cp-development
* Add handling t265 coordinate system (`#657 <https://github.com/plusone-robotics/realsense/issues/657>`_)
  * fixed launch files (fisheye1,2)
  * renamed spatial_frame to odom_frame
  fixed dependency of librealsense to version 2.19.0
  Add t265_realsense_node.h, t265_realsense_node.cpp to handle the different coordinate system.
  Add demo_t265.launch file and t265.rviz
  send odom_frame tf even without someone registered to odom topic.
* renamed spatial_frame to odom_frame
  fixed dependency of librealsense to version 2.19.0
  Add t265_realsense_node.h, t265_realsense_node.cpp to handle the different coordinate system.
  Add demo_t265.launch file and t265.rviz
  send odom_frame tf even without someone registered to odom topic.
* :
  [Problem]
  [Solution]
  [Test]
  [Links]
  https://issues.labcollab.net/browse/
* fixed launch files (fisheye1,2)
* Merge branch 'bfulkers-i-update-readme' into development
* add Notice to README.md and rs_t265.launch
* Merge branch 'doronhi-reconnect2' into development
* Merge branch 'reconnect2' of https://github.com/doronhi/realsense into doronhi-reconnect2
  # Conflicts:
  #	README.md
  #	realsense2_camera/launch/includes/nodelet.launch.xml
  #	realsense2_camera/launch/rs_camera.launch
  #	realsense2_camera/launch/rs_d400_and_t265.launch
  #	realsense2_camera/launch/rs_t265.launch
  #	realsense2_camera/scripts/rs2_test.py
  #	realsense2_camera/src/realsense_node_factory.cpp
* Fix version in package.xml (`#625 <https://github.com/plusone-robotics/realsense/issues/625>`_)
* Modified the CMake file so that URDF and mesh files will be installed (`#615 <https://github.com/plusone-robotics/realsense/issues/615>`_)
* Fix `#628 <https://github.com/plusone-robotics/realsense/issues/628>`_ - added guards around clang-specific pragmas (`#630 <https://github.com/plusone-robotics/realsense/issues/630>`_)
  Also added a guard around an OpenMP pragma
* fix rs_aligned_depth.launch
* increase rs2_test.py robustness for node failing to load.
* fix README.md and launch files.
* auto reset if need to.
* fix README.md and launch files.
* restore initial_reset option.
  Fix bug of locking tracking module (t265) by nodes that don't use it.
* modify behavior: if reconnect if camera disconnected.
  package.xml: upgrade package format
  removed initial_reset option - need to return.
* rename tm2 to t265
* fixed static_tf test in rs2_test and changed the name of vis_avg_1 to non_existent_file to reflect it's true purpose.
* delete topics of aligned depth to index 2 of other sensors. (`#644 <https://github.com/plusone-robotics/realsense/issues/644>`_)
  It is not implemented in librealsense and the topics that were published so far do not provide useful information were actually aligned to index 1.
* delete topics of aligned depth to index 2 of other sensors.
  It is not implemented in librealsense and the topics that were published so far do not provide useful information were actually aligned to index 1.
* rs_t265.launch: Add a disclaimer about wheel odometry
* renames and readme (`#629 <https://github.com/plusone-robotics/realsense/issues/629>`_)
  * fixed static_tf test in rs2_test and changed the name of vis_avg_1 to non_existent_file to reflect it's true purpose.
  * rename tm2 to t265
  * fix README.md
* Contributors: Brian Fulkerson, CameronDevine, Jarvis Schultz, Stephan, doronhi, iliabara

2.2.0 (2019-02-17)
------------------
* check build with librealsense v2.18.1
* update version to 2.2.0
* Merge remote-tracking branch 'doronhi/tm2' into development
  # Conflicts:
  #	realsense2_camera/src/base_realsense_node.cpp
* use tf2 instead of tf for pose static transformation
* Fix pending messages variable name typo (`#608 <https://github.com/plusone-robotics/realsense/issues/608>`_)
* Replace spaces and hyphens in parameter names (`#617 <https://github.com/plusone-robotics/realsense/issues/617>`_)
* fix dependency between covariance values and confidence value.
  Added to README.md
* fix test. remove some log messages.
* fix frame_id for odom topic.
* TM265 - add odometry topic
  interface change: add parameter: enable_tm2 - cause the wrapper to wait on initialization while tm2 device sets its Unique USB ID
  use enable_gyro and enable_accel instead of enable_imu
  use infra_width, infra_fps instead of infra1_width, infra1_fps and infra2_width, infra2_fps
* add basic support for TM265. Fisheye, Gyro, Accel.
* code reorganization.
  fix bug of reinitializing align operator.
* add support for TM1 fisheye comes in RAW8 and Tm2's in Y8.
  moved enabling HID sensors to enable_devices()
* clean parameters reading.
* set base time on first message (image or imu originated)
  clean code.
* Remove gencfg dependency (`#581 <https://github.com/plusone-robotics/realsense/issues/581>`_)
  Now with ddynamic_reconfigure being the backend for dynamic reconfigurability, the ${PROJECT_NAME}_gencfg target doesn't exist anymore and this dependency can be removed.
* fix bug: "No stream match for pointcloud chosen texture" warning was meant to appear when unavailable texture is chosen. As it was, it appears every time a frame was dropped. (`#591 <https://github.com/plusone-robotics/realsense/issues/591>`_)
* Remove REQUIRED from find_package to show the correct error message (`#592 <https://github.com/plusone-robotics/realsense/issues/592>`_)
* Add filters argument to rs_rgbd.launch (`#593 <https://github.com/plusone-robotics/realsense/issues/593>`_)
* No depth required (`#601 <https://github.com/plusone-robotics/realsense/issues/601>`_)
  * add benchmark test for static_tf
  * enable running with depth disabled.
  rs2_test.py: Add message to results summery.
* fix bug: no default covariance for separate gyro and accel imu messages. (`#600 <https://github.com/plusone-robotics/realsense/issues/600>`_)
* Contributors: Ian Zhang, Stephan, doronhi, socrob, vatbrain

2.1.4 (2019-01-24)
------------------
* update version to 2.1.4
* fix bug: update camera_info if image size changes. (`#587 <https://github.com/plusone-robotics/realsense/issues/587>`_)
* changed the default gyro_fps and accel_fps to match actual values (`#560 <https://github.com/plusone-robotics/realsense/issues/560>`_)
* Merge branch 'RhysMcK-development' into development
* Merge branch 'development' of https://github.com/RhysMcK/realsense into RhysMcK-development
* add initial_reset to camera2 in rs_multiple_devices.launch
* fixed urdf.rviz to look nicer.
* Merge branch 'atyshka-development' into development
* fix transform between urdf and driver
* correctred .stl filename
* added realsense D415 urdf
* Fixed d435 collision position
* add bottom_screw joint to _d435.urdf.xacro
* Merge branch 'development' of https://github.com/atyshka/realsense into atyshka-development
* add initial_reset option to rs_multiple_devices.launch
* Merge remote-tracking branch 'origin/development' into development
* fix bug in align depth to image. (`#572 <https://github.com/plusone-robotics/realsense/issues/572>`_)
  When publishFrame is called from publishAlignedDepthToOthers the format of the images is already set and is different from what is defined in _image_format for that stream type.
* close sensors when Ctrl-C signal is received. (`#571 <https://github.com/plusone-robotics/realsense/issues/571>`_)
  add test in makefile for librealsense version
* Fixed different transforms between xacro and driver
* update version number
* Contributors: Harsh Pandya, RhysMcK, Unknown, doronhi

2.1.3 (2019-01-01)
------------------
* add linear interpolation union method for IMU (`#558 <https://github.com/plusone-robotics/realsense/issues/558>`_)
  Add linear interpolation method for union of IMU sensors. Thanks to Marius Fehr (@mfehr) for the idea.
  Set the initial behavior to sending IMU sensors separately, since this is the raw data. Enabling union with option unite_imu_method as demonstrated in the file opensource_tracking.launch.
  fix bug if initializing with unavailable imu profile.
* fix to work with librealsense v2.17.0 (`#555 <https://github.com/plusone-robotics/realsense/issues/555>`_)
  fixed to work with librealsense v2.17.0
* fix: wrong reference for the gmock dependency (`#546 <https://github.com/plusone-robotics/realsense/issues/546>`_)
  fix: typo on ddynamic_reconfigure
* Add notifications for hardware errors.
* add parameter "initial_reset" to reset the device on start up. Default is set to false.
* Merge branch 'yycho0108-development' into development
* Merge branch 'development' of https://github.com/yycho0108/realsense into yycho0108-development
* Fixed: invalid module name format for ROS (`#537 <https://github.com/plusone-robotics/realsense/issues/537>`_)
* use ddynamic_reconfigure and support D435i (`#535 <https://github.com/plusone-robotics/realsense/issues/535>`_)
  Add dynamic dynamic reconfigure. That means there are no longer differences in the code between D415, D430, SR300.
  Add dynamic options for filters
  Add support for camera D435i.
  Add clipping_disance option. enabled with parameter: clip_distance. units: meters. Default: no clipping.
  Add linear accel covariance - Default: 0.01
  Add option: unite_imu - send linear acceleration and radial velocity in the same Imu message. Default: True
  Add parameter: hold_back_imu_for_frames. If set to true, hold imu messages that arrived while manipulating frames, until frames are actually sent.
  Comply with librealsense v2.17.0
  Add opensource_tracking.launch - demo that runs realsense2_camera, imu_filter_madgwick, rtabmap and robot_localization to demonstrate Slam with realsense D435i
  Set accel_fps to 250 as this is the new maximal rate in librealsense v2.17.0
  * Add NOTICE file, to emphasize the contribution of the ddynamic_reconfigure project.
  Known Issue: Option for toggling sensor on and off while running is missing.
* Merge remote-tracking branch 'intel/development' into development
* removed unnecessary device query (artifact from merge)
* fixed merge conflict while retaining hardware reset during initialization; added exec_depends to rgbd_launch
* possible fix
* Merge remote-tracking branch 'reset_dev/reset_dev' into development
* Contributors: Jamie Cho, Thiago de Freitas, carlos, doronhi

2.1.2 (2018-12-06)
------------------
* Update constants.h
  update version to 2.1.2
* Potential Fix for librealsense2 v2.17.0 Compatilbility (`#523 <https://github.com/plusone-robotics/realsense/issues/523>`_)
  Fix to comply with librealsense v2.17.0.
  Thanks @m-price-softwearinc
* Contributors: Miles Price, doronhi

2.1.1 (2018-11-01)
------------------
* add log info - when dynamic reconfiguration is done.
* revert PR `#490 <https://github.com/plusone-robotics/realsense/issues/490>`_: rgbd_launch file is a running example for using the rgbd module. No need to add elements to installation for all users.
* add disparity processing.
  move colorizer to the back of the filters pipeline.
* add disparity processing
  moved colorizer filter to the end of filters pipeline.
* add decimation filter (`#504 <https://github.com/plusone-robotics/realsense/issues/504>`_)
  * add decimation filter. enable with filters:=decimation
  * fix tests to check number of holes in depth image.
  add tests to check decimation filter.
* fix tests to check number of holes in depth image.
  add tests to check decimation filter.
* add decimation filter. enable with filters:=decimation
* update version to 2.1.1
* start working on decimation filter
* Merge branch 'development' of https://github.com/intel-ros/realsense into development
* add filters option to rs_aligned_depth.launch
* fix all sensors.
* fix bug: depth_auto_exposure was override in initialization by depth_exposure.
  fix bug: error in setting a parameter stop setting all other parameters.
* added missing dependencies: rgbd_launch (`#490 <https://github.com/plusone-robotics/realsense/issues/490>`_)
* Merge branch 'fork_development' into development
* fix bug: Initial dynamic configuration was stopped by starting an already started sensors. While this may not be the best practice, it's not doing any wrong and setting parameters to their default values should continue.
* fix issue: depth is being sent incorrectly if pointcloud is being sent. (`#498 <https://github.com/plusone-robotics/realsense/issues/498>`_)
  * add test for depth and aligned_depth_to_infra1.
  * fix bug: _aligned_depth_images initialized incorrectly if width, height not specified in launch parameters.
  * use librealsense2 align filter to align the depth image. Also fix bug that was on the previous projection.
  add test: align_depth_color_1
  * add test depth_w_cloud_1 according to issue `#491 <https://github.com/plusone-robotics/realsense/issues/491>`_.
  * fix bug: depth_frame is not sent if pointcloud is on.
* fix bug: depth_frame is not sent if pointcloud is on.
* add test depth_w_cloud_1 according to issue `#491 <https://github.com/plusone-robotics/realsense/issues/491>`_. Fails.
* use librealsense2 align filter to align the depth image. Also fix bug that was on the previous projection.
  add test: align_depth_color_1
* fix bug: _aligned_depth_images initialized incorrectly if width, height not specified in launch parameters.
* Merge branch 'development' with fix for aligned depth bug into fork_development with matching test.
* add test for depth and aligned_depth_to_infra1. The last one is knowingly fails.
* fix bug aligning depth to images
* Merge pull request `#483 <https://github.com/plusone-robotics/realsense/issues/483>`_ from shuntaraw/fix_tf_prefix
  Set tf_prefix in demo_pointcloud.launch
* Merge branch 'AndyZe-development' into development
* Merge branch 'development' of https://github.com/AndyZe/realsense into AndyZe-development
* base_realsense_node.cpp: fix typo.
* set_cams_transforms.py: fix bugs.
* add set_cams_transforms.py to add transformation between cameras.
* Pausing sensors with sens.stop(). Saves about 9% CPU load on useless processing.
* Adding a dynamic_reconfigure option to toggle ROS publication (issue `#477 <https://github.com/plusone-robotics/realsense/issues/477>`_).
* Set tf_prefix in demo_pointcloud.launch
* Contributors: AndyZe, Florenz Graf, Shuntaro Yamazaki, doronhi

2.1.0 (2018-09-27)
------------------
* Merge pull request `#482 <https://github.com/plusone-robotics/realsense/issues/482>`_ from doronhi/development
  Add support for post processing filters
* Merge branch 'development' into development
* filters applied in given order.
  add spatial and temporal filters.
  pointcloud can be activated as a type of filter (also, still, with flag enable_pointcloud)
* fix build warning.
* modify test for pointcloud because of known bug in setting texture for pointcloud of 1st frame.
  New pointcloud does not put background color so values of test have changed.
* fix image size in pointcloud test.
* Merge branch 'baumanta-multi_cam' into development
* Change default names for frames to the same name specified for the camera topics
* new launch parameter for frame distinction in multi camera use
* enable filter colorizer.
  Issue: Can not send both pointcloud and colorized depth image at the same time.
* working pointcloud by filter. need to clean.
* Start adding filters.
  pointcloud is now implemented with filter.
  BUG: Not transmitting texture.
* add test for PointCloud2 in topic /camera/depth/color/points
* Start working on version 2.1.0 - enabling filters.
* Start working on version 2.1.0 - enabling filters.
* Contributors: baumanta, doronhi

2.0.4 (2018-08-29)
------------------
* Merge pull request `#452 <https://github.com/plusone-robotics/realsense/issues/452>`_ from doronhi/development
  build with librealsense 2.16
* create wrapper class PipelineSyncer to work around librealsense 2.16 feature, removing operator() from class asynchronous_syncer.
* Merge pull request `#440 <https://github.com/plusone-robotics/realsense/issues/440>`_ from doronhi/development
  merge PR regarding CMakefile and package.xml
* remove librealsense2 from catkin dependencies.
* Use find_package() variables.
* Merge pull request `#439 <https://github.com/plusone-robotics/realsense/issues/439>`_ from doronhi/development
  namespace argument renamed "camera".
* namespace argument renamed "camera".
* Merge branch 'MisoRobotics-fix-rotationMatrixToQuaternion-declaration' into development
* fix input for realsense2_camera::rotationMatrixToQuaternion from float[3] to float[9]
* line up <group ns> parameter in all launch files. (`#438 <https://github.com/plusone-robotics/realsense/issues/438>`_)
  fixed parameter name for <group ns> to be "namespace", as defined previously in other launch files.
* fixed parameter name for <group ns> to be "namespace", as defined previously in other launch files.
* Merge branch 'development' of https://github.com/intel-ros/realsense into development
* Merge branch 'SteveMacenski-launch_name_configuration' into development
* Merge branch 'launch_name_configuration' of https://github.com/SteveMacenski/realsense into SteveMacenski-launch_name_configuration
* Travis CI build and test (`#437 <https://github.com/plusone-robotics/realsense/issues/437>`_)
  * fix issue `#335 <https://github.com/plusone-robotics/realsense/issues/335>`_ according to solution lsolanka as suggested in pull request `#336 <https://github.com/plusone-robotics/realsense/issues/336>`_.
  * moving all the properties and material definitions inside the macro as suggested by @felixvd
  * add compilation flag SET_USER_BREAK_AT_STARTUP to create user waiting point for debugging purposes.
  add reading from bagfile option by using <rosbag_filename> parameter in launch file.
  base_realsense_node.cpp: add option - by specifying width, height or fps as 0, pick up on the first sensor profile available.
  scripts/rs2_listener.py, rs2_test.py - initial version for file based, standalone unitest.
  * add .travis.yml file
* remove parse_bag_file.py
* use locations of realsense2
* TravisCI.yml: fix and add data downloading.
  rs2_test.py: fix test to match new bag file: outdoors.bag
* update .travis.yml
  make test expected to fail to display SUCCESS.
* moved .travis.yml to root
* add .travis.yml file
* Merge branch 'read_bg_file' into development
* Merge branch 'Origin->development' 'fork->development'
* add compilation flag SET_USER_BREAK_AT_STARTUP to create user waiting point for debugging purposes.
  add reading from bagfile option by using <rosbag_filename> parameter in launch file.
  base_realsense_node.cpp: add option - by specifying width, height or fps as 0, pick up on the first sensor profile available.
  scripts/rs2_listener.py, rs2_test.py - initial version for file based, standalone unitest.
* making camera name configurable, necessity for launching multiple cameras
* Merge pull request `#418 <https://github.com/plusone-robotics/realsense/issues/418>`_ from yayaneath/alignment
  Fix the name of the alignment-related parameters when invoking the RealSenseNodeFactory.
* Fix the name of the alignment-related parameters when invoking the RealSenseNodeFactory.
* Merge pull request `#417 <https://github.com/plusone-robotics/realsense/issues/417>`_ from doronhi/fix_bug_pointer_out_of_bounds
  fix bug pointer out of bounds
* fix issue `#335 <https://github.com/plusone-robotics/realsense/issues/335>`_ according to solution lsolanka as suggested in pull request `#336 <https://github.com/plusone-robotics/realsense/issues/336>`_.
* Merge branch 'development' of https://github.com/doronhi/realsense into development
* moving all the properties and material definitions inside the macro as suggested by @felixvd
* Merge branch 'development' of https://github.com/intel-ros/realsense into development
* Merge branch 'Affonso-Gui-add_d435_urdf' including some modifications into development
* fixed coordinate system for sensors in camera.
  renamed fisheye to color camera
* Merge branch 'add_d435_urdf' of https://github.com/Affonso-Gui/realsense into Affonso-Gui-add_d435_urdf
* Merge pull request `#374 <https://github.com/plusone-robotics/realsense/issues/374>`_ from scythe-robotics/development
  Fixes librealsense CMake vars.
* Merge branch 'development' of https://github.com/intel-ros/realsense into development
* Merge pull request `#367 <https://github.com/plusone-robotics/realsense/issues/367>`_ from AlanBB277/development
  checked also with D415. Confirmed.
* fix issue `#335 <https://github.com/plusone-robotics/realsense/issues/335>`_ according to solution lsolanka as suggested in pull request `#336 <https://github.com/plusone-robotics/realsense/issues/336>`_.
* Merge pull request `#383 <https://github.com/plusone-robotics/realsense/issues/383>`_ from mikolajz/my-development
  Fix coordinate system transforms so that the pointcloud aligns with camera view
* Fixing the length of an array argument in rotationMatrixToQuaternion
* Add mesh and urdf for D435
* Also when align_depth is no, publish proper data on extrinsic topics.
  AFAIK there is no convention of what to publish on extrinsic topics, so you
  may choose to keep it as is, but I would say the current behavior can be
  surprising in a negative way.
* Fix the rotation quaternion in coordinate transforms.
  When going from one optical frame to another, the actual rotation we are
  performing is quaternion_optical.inverse() * Q * quaternion_optical, so we
  need to for the final rotation to be as specific in the extrinsics.
  The pointcloud is now properly aligned.
* Publish coordinate system transforms also when align depth is on.
  That fact that aligned_depth_to\_* is in color coordinates is already
  experessed by these cameras camera_info reporting the color frame. However,
  for the "depth", "infra1" etc. camera to be properly reported and for the
  pointcloud to have a change to align, we need to report the transformations.
* In coordinate system transforms, fix which extrincits we use and use matrix properly.
  Two bugs which cancel out each other for rotation, but not translation:
  - it seems that ROS and Realsense use different conventions of coordinate
  system transformations. In ROS, it is defined as a transformation of child
  fame coordinates to parent frame coordinates (see
  http://wiki.ros.org/tf/Overview/Transformations), while in RealSense
  it seems to be transformation of "from" frame coordinates to "to" frame
  coordinates. Thus, the order needs to be reversed.
  - the matrix in RealSense extrinsics is stored in column-major format, while
  Eigen::Matrix3f expects row-major, causing the matrix to be transposed.
  To see that this is a problem, one can open rviz and add the pointcloud and the
  color/image_raw camera. From the camera viewpoint, the images should align, but
  don't. This patch doesn't yet solve the whole problem, but makes it smaller.
* Fixes librealsense CMake vars.
* fix the aligned depth frame unit conversion issue
* Merge pull request `#364 <https://github.com/plusone-robotics/realsense/issues/364>`_ from lorenwel/fix/aligned_depth_cam_info
  aligned_depth_to\_... assign stream cam info instead of depth
* Assign stream cam info instead of depth
* Contributors: AlanBB277, Guilherme de Campos Affonso, Itay Carpis, Jack Morrison, Mikołaj Zalewski, Robert Haschke, Ryan Sinnet, brayan, doronhi, lorenwel, stevemacenski

2.0.3 (2018-03-29)
------------------
* Merge pull request `#352 <https://github.com/plusone-robotics/realsense/issues/352>`_ from ruvu/feature/diagnostics
  Feature/diagnostics
* Corrected diagnostics naming of aligned streams (comment @icarpis)
* correct pointer to expected frequency
* Revert "Use nodehandles from nodelet"
  This reverts commit 03b0114bdca04ac8752c760495981c349b7ae595.
* Use nodehandles from nodelet
* Some logging
* diagnostic updaters with frequency status for publishers
* Merge pull request `#351 <https://github.com/plusone-robotics/realsense/issues/351>`_ from icarpis/development
  Bump version
* Bump version
* Merge pull request `#350 <https://github.com/plusone-robotics/realsense/issues/350>`_ from icarpis/development
  Improve CPU utilization using rs_rgbd.launch
* Fixed SR300 depth scale issue
* Check for subscribers before publish aligned frames
* Merge pull request `#324 <https://github.com/plusone-robotics/realsense/issues/324>`_ from icarpis/development
  Renaming ROS package from realsense_ros_camera to realsense2_camera
* Fixed merge issue
* Renaming ROS package from realsense_ros_camera to realsense2_camera
* Contributors: Itay Carpis, Rein Appeldoorn, icarpis

2.0.2 (2018-01-31)
------------------

2.0.1 (2017-11-02)
------------------

2.0.0 (2017-09-17)
------------------
