^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package realsense2_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

100.0.8 (2019-02-07)
--------------------
* [capability] Ir-RGB integration  `#12 <https://github.com/plusone-robotics/realsense/issues/12>`_
* Filters for Plus One Robotics. `#3 <https://github.com/plusone-robotics/realsense/issues/3>`_
* Contributors: Isaac I.Y. Saito, Joshua Curtis, Shaun Edwards

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
