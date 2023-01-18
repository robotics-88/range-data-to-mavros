# depth-image-to-mavlink

This package extends thien94's [repo](https://github.com/thien94/vision_to_mavros). That repo was specific to the Intel Realsense, and used Python and no ROS. This package generalizes that work, enabling ArduPilot obstacle avoidance with any ROS depth camera.

To run:

`roslaunch depth_image_to_mavlink depth_to_mavlink.launch`