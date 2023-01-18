/* 
© 2023 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "depth_image_to_mavlink/depth_image_to_mavlink.h"

#include <opencv2/photo.hpp>

namespace depth_image_to_mavlink
{
Depth_image_to_mavlink::Depth_image_to_mavlink(ros::NodeHandle& node)
  : private_nh_("~")
  , nh_(node)
  , depth_topic_("zed2i/zed_node/depth/depth_registered")
{   
    private_nh_.param<std::string>("depth_image", depth_topic_, depth_topic_);

    depth_image_subscriber_ = nh_.subscribe<sensor_msgs::Image>(depth_topic_, 10, &Depth_image_to_mavlink::depthImageCallback, this);

    depth_smoothed_publisher_ = nh_.advertise<sensor_msgs::Image>("depth_smoothed", 10);
}

Depth_image_to_mavlink::~Depth_image_to_mavlink(){}

void Depth_image_to_mavlink::depthImageCallback(const sensor_msgs::ImageConstPtr &msg) {
    // Get depth image as mat
    cv::Mat depth_mat;
    cv_bridge::CvImagePtr cv_ptr; 
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1); 
    depth_mat = cv_ptr->image;

    // Clean depth image
    cv::Mat depth_smoothed, inpaintMask;
    inpaintMask = depth_mat.clone();
    cv::patchNaNs(inpaintMask, -1.0);
    inpaintMask = (inpaintMask == -1.0);
    cv::inpaint(depth_mat, inpaintMask, depth_smoothed, 3, cv::INPAINT_NS);

    // Republish smoothed depth
    cv_bridge::CvImage smoothed_msg;
    smoothed_msg.header   = msg->header; // Same timestamp and tf frame as input image
    smoothed_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    smoothed_msg.image    = depth_smoothed;
    depth_smoothed_publisher_.publish(smoothed_msg.toImageMsg());
}

}