# range_data_to_mavros

This package is named "range_data_to_mavros". It converts range data (i.e. from a depth image or a point cloud) into the format MAVROS utilizes in its [obstacle_distance](https://github.com/mavlink/mavros/blob/master/mavros_extras/src/plugins/obstacle_distance.cpp) plugin. 

The point cloud handler extends paulbobvel's [repo](https://github.com/ros-perception/pointcloud_to_laserscan/blob/indigo-devel/src/pointcloud_to_laserscan_node.cpp), and adapts it to package the output data for MAVROS

The depth image handler extends thien94's [repo](https://github.com/thien94/vision_to_mavros). That repo was specific to the Intel Realsense, and used Python and no ROS. This package generalizes that work, enabling ArduPilot obstacle avoidance with any ROS depth camera.

To run, you should select the input data type in the 'data_type' parameter, whether it's point_cloud or depth_image. Default is depth_image. So, for example, to run the converter on a point cloud, you would do:

`roslaunch range_data_to_mavros range_data_to_mavros.launch data_type:=point_cloud`