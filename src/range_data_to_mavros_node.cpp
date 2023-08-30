/* 
© 2023 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "range_data_to_mavros/depth_image_handler.h"
#include "range_data_to_mavros/point_cloud_handler.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "range_data_to_mavros");
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::NodeHandle node("~");

  std::string data_type;
  if (!node.getParam("data_type", data_type)) {
    data_type = "depth_image";
    ROS_WARN("Couldn't find depth image parameter, defaulting to %s", data_type.c_str());
  }

  if (data_type == "point_cloud") {
    ROS_INFO("Running point cloud handler");
    PointCloudHandler point_cloud_handler(node);
    ros::spin();
  }
  else {
    ROS_INFO("Running depth image handler");
    DepthImageHandler depth_image_handler(node);
    ros::spin();
  }

  return 0;
}