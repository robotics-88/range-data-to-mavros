/* 
© 2023 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#ifndef DEPTH_IMAGE_TO_MAVLINK_H_
#define DEPTH_IMAGE_TO_MAVLINK_H_

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

namespace depth_image_to_mavlink {
/**
 * @class Depth_image_to_mavlink
 * @brief Converts depth images to mavlink obstacle messages required for obstacle avoidance.
 */
class Depth_image_to_mavlink {

    public:
        Depth_image_to_mavlink(ros::NodeHandle& node);
        ~Depth_image_to_mavlink();

        void depthImageCallback(const sensor_msgs::ImageConstPtr &msg);

    private:
        ros::NodeHandle private_nh_;
        ros::NodeHandle nh_;

        std::string depth_topic_;
        ros::Subscriber depth_image_subscriber_;
        ros::Publisher depth_smoothed_publisher_;
};

}

#endif