ay_3dvision
==================
3D computer vision tools, including pose estimation from RGB-D image, stereo, Sentis M100 controller, etc.

`ay_3dvision` is built on ROS, implemented with C++ using PCL and OpenCV.


Author
==================
Akihiko Yamaguchi, http://akihikoy.net/


Requirements
==================
See `manifest.xml`.  PCL and OpenCV are used.


Directories
==================

include/ay_3dvision
----------------------------
ROS-independent header files (C++).

src
----------------------------
ROS-independent source files (C++).

src_ros
----------------------------
ROS-dependent source files (C++).  Mainly, programs of ROS nodes are contained.


Build
==================
The repository directory should be in ROS workspace (e.g. ~/ros_ws/).
Build `ay_3dvision` with `rosmake`.

```
$ rosmake ay_3dvision
```

After `rosmake`, you will find some executables in `bin/` directory and shared objects in `lib/` directory.
There will be some directories made by `rosmake`.


Usage
==================

rt_pose_estimator
---------------------------
Estimates 3D pose from an RGB-D image.  Requires a model of object as a set of shape primitives, and an initial pose.

sentis_m100
---------------------------
Sentis M100 (depth sensor) controller.

usb_stereo
---------------------------
Stereo vision with USB cameras.


Troubles
==================
Send e-mails to the author.
