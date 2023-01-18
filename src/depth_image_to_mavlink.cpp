/* 
© 2022 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "depth_image_to_mavlink/depth_image_to_mavlink.h"

namespace depth_image_to_mavlink
{
Depth_image_to_mavlink::Depth_image_to_mavlink(ros::NodeHandle& node)
  : private_nh_("~")
  , nh_(node)
{
    
}

Depth_image_to_mavlink::~Depth_image_to_mavlink(){}

}