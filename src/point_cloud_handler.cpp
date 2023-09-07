/* 
© 2023 Robotics 88
Author: Gus Meyer <gus@robotics88.com> 
*/

#include "range_data_to_mavros/point_cloud_handler.h"
#include <math.h>

PointCloudHandler::PointCloudHandler(ros::NodeHandle& node)
    : private_nh_("~")
    , nh_(node)
    , point_cloud_topic_("/velodyne_points")
    , mavros_obstacle_topic_("/mavros/obstacle/send")
    , target_frame_("base_link_frd")
    , last_obstacle_distance_sent_ms(ros::Time(0).toSec())
    , distances_array_length_(72)
    , min_height_(std::numeric_limits<double>::min())
    , max_height_(std::numeric_limits<double>::max())
    , angle_min_(0)
    , angle_max_(2.0*M_PI)
    , range_min_(0.0)
    , range_max_(std::numeric_limits<double>::max())
    , vehicle_state_received_(false)
{   
    private_nh_.param<std::string>("point_cloud_topic", point_cloud_topic_, point_cloud_topic_);
    private_nh_.param<std::string>("mavros_obstacle_topic", mavros_obstacle_topic_, mavros_obstacle_topic_);
    private_nh_.param<std::string>("target_frame", target_frame_, target_frame_);
    private_nh_.param<double>("min_height", min_height_, min_height_);
    private_nh_.param<double>("max_height", max_height_, max_height_);
    private_nh_.param<double>("angle_min", angle_min_, angle_min_);
    private_nh_.param<double>("angle_max", angle_max_, angle_max_);
    private_nh_.param<double>("range_min", range_min_, range_min_);
    private_nh_.param<double>("range_max", range_max_, range_max_);

    angle_increment_ = (angle_max_ - angle_min_) / distances_array_length_;

    point_cloud_subscriber_ = nh_.subscribe<sensor_msgs::PointCloud2>(point_cloud_topic_, 10, &PointCloudHandler::pointCloudCallback, this);

    mavros_obstacle_publisher_ = nh_.advertise<sensor_msgs::LaserScan>(mavros_obstacle_topic_, 10);
}

PointCloudHandler::~PointCloudHandler(){}

// Adapted from https://github.com/ros-perception/pointcloud_to_laserscan/blob/indigo-devel/src/pointcloud_to_laserscan_node.cpp
void PointCloudHandler::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {

    // Build laserscan output
    auto scan_msg = std::make_unique<sensor_msgs::LaserScan>();
    scan_msg->header = msg->header;
    if (!target_frame_.empty()) {
        scan_msg->header.frame_id = target_frame_;
    }
    scan_msg->angle_min = angle_min_;
    scan_msg->angle_max = angle_max_;
    scan_msg->angle_increment = angle_increment_;
    scan_msg->time_increment = 0.0;
    scan_msg->scan_time = scan_time_;
    scan_msg->range_min = range_min_;
    scan_msg->range_max = range_max_;
    scan_msg->ranges.assign(distances_array_length_, UINT16_MAX);

    // Iterate through points
    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"),
        iter_y(*msg, "y"), iter_z(*msg, "z");
        iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)     
    { 
        // Check validity of point
        if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
            // ROS_DEBUG("rejected for nan in point(%f, %f, %f)\n", *iter_x, *iter_y, *iter_z);
            continue;
        }

        // Convert point to our reference frame, where points are relative to sensor's height in an absolute sense, based on vehicle IMU data
        tf2::Vector3 point(*iter_x, *iter_y, *iter_z);

        if (vehicle_state_received_) {
            tf2::Quaternion q;
            tf2::fromMsg(last_pose_.pose.orientation, q);
            point = tf2::quatRotate(q, point);
        }
        
        // Y axis swap from FLU to FRD, since mavros wants FRD. 
        // Sort of hacky, might be better to do with real ROS transforms using the base_link_frd frame
        point.setY(-point.getY());

        // Filter out points outside height range relative to drone
        if (point.getZ() > max_height_ || point.getZ() < min_height_) {
            // ROS_DEBUG("rejected for height %f not in range (%f, %f)\n", point.getZ(), min_height_, max_height_);
            continue;
        }

        double range = hypot(point.getX(), point.getY());
        if (range < range_min_) {
            //ROS_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range, range_min_, point.getX(), point.getY(), point.getZ());
            continue;
        }

        if (range > range_max_) {
            //ROS_DEBUG("rejected for range %f above maximum value %f. Point: (%f, %f, %f)", range, range_max_, point.getX(), point.getY(), point.getZ());
            continue;
        }

        double angle = atan2(point.getY(), point.getX());
        if (angle < scan_msg->angle_min || angle > scan_msg->angle_max) {
            //ROS_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, scan_msg->angle_min, scan_msg->angle_max);
            continue;
        }

        // overwrite range at laserscan ray if new range is smaller
        int index = (angle - scan_msg->angle_min) / scan_msg->angle_increment;
        if (range < scan_msg->ranges[index]) {
            scan_msg->ranges[index] = range;
        }

    }

    // Publish laserscan message
    if (ros::Time::now().toSec() == last_obstacle_distance_sent_ms) {
        // # no new frame
        ROS_WARN("no new frame");
        return;
    }
    last_obstacle_distance_sent_ms = ros::Time::now().toSec();

    mavros_obstacle_publisher_.publish(*scan_msg);
}

void PointCloudHandler::dronePoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    vehicle_state_received_ = true;
    last_pose_ = *msg;
}