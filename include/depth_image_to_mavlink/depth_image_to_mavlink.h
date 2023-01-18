/* 
© 2023 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#ifndef DEPTH_IMAGE_TO_MAVLINK_H_
#define DEPTH_IMAGE_TO_MAVLINK_H_

#include <ros/ros.h>

namespace depth_image_to_mavlink {
/**
 * @class Depth_image_to_mavlink
 * @brief Converts depth images to mavlink obstacle messages required for obstacle avoidance.
 */
class Depth_image_to_mavlink {

    public:
        Depth_image_to_mavlink(ros::NodeHandle& node);
        ~Depth_image_to_mavlink();

    private:
        ros::NodeHandle private_nh_;
        ros::NodeHandle nh_;
};

}

#endif