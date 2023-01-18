/* 
© 2023 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "depth_image_to_mavlink/depth_image_to_mavlink.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_image_to_mavlink");
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::NodeHandle node;
  depth_image_to_mavlink::Depth_image_to_mavlink depth_image_to_mavlink(node);

  ros::spin();

  return 0;
}