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
    , target_frame_("")
    , last_obstacle_distance_sent_ms(ros::Time(0).toSec())
    , distances_array_length_(72)
    , min_height_(std::numeric_limits<double>::min())
    , max_height_(std::numeric_limits<double>::max())
    , angle_min_(-M_PI)
    , angle_max_(M_PI)
    , range_min_(0.0)
    , range_max_(std::numeric_limits<double>::max())
    , vehicle_state_received_(false)
{   
    private_nh_.param<std::string>("point_cloud_topic", point_cloud_topic_, point_cloud_topic_);
    private_nh_.param<std::string>("mavros_obstacle_topic", mavros_obstacle_topic_, mavros_obstacle_topic_);
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

    std::cout << "Getting point cloud" << std::endl;

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
        tf2::Quaternion q;
        tf2::fromMsg(last_pose_.pose.orientation, q);
        tf2::Vector3 rotated_point = tf2::quatRotate(q, point);

        // Filter out points we don't care about
        if (point.getZ() > max_height_ || point.getZ() < min_height_) {
            // ROS_DEBUG("rejected for height %f not in range (%f, %f)\n", *iter_z, min_height_, max_height_);
            continue;
        }

        double range = hypot(*iter_x, *iter_y);
        if (range < range_min_) {
            //ROS_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range, range_min_, *iter_x, *iter_y, *iter_z);
            continue;
        }

        if (range > range_max_) {
            //ROS_DEBUG("rejected for range %f above maximum value %f. Point: (%f, %f, %f)", range, range_max_, *iter_x, *iter_y, *iter_z);
            continue;
        }

        double angle = atan2(*iter_y, *iter_x);
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

    mavros_obstacle_publisher_.publish(*scan_msg);


    // TESTING
    // tf2::Vector3 point(5.0, 0.0, 0.0);
    // tf2::Quaternion q;
    // q.setRPY(0.0, -0.64, 0.0);
    // tf2::Vector3 rotated_point = tf2::quatRotate(q, point);
    // std::cout << "Rotated point: " << rotated_point.getX() << ", " << rotated_point.getY() << ", " << rotated_point.getZ() << std::endl;

}

void PointCloudHandler::dronePoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    last_pose_ = *msg;
}

// Mavlink Message:
// https://mavlink.io/en/messages/common.html#OBSTACLE_DISTANCE
// Mavros plugin:
// https://github.com/mavlink/mavros/blob/ros2/mavros_extras/src/plugins/obstacle_distance.cpp
void PointCloudHandler::publishObstacleDistances(const std_msgs::Header &header, const std::vector<float> &distances) {
    if (ros::Time::now().toSec() == last_obstacle_distance_sent_ms) {
        // # no new frame
        ROS_WARN("no new frame");
        return;
    }
    last_obstacle_distance_sent_ms = ros::Time::now().toSec();

    sensor_msgs::LaserScan obstacle_msg;
    obstacle_msg.header.frame_id = "base_link";
    obstacle_msg.header.stamp = header.stamp;
    obstacle_msg.range_max = range_max_;
    obstacle_msg.range_min = range_min_;
    obstacle_msg.ranges = distances;
    obstacle_msg.angle_increment = angle_increment_ * M_PI / 180;
    obstacle_msg.time_increment = 0;
    obstacle_msg.scan_time = 0;
    obstacle_msg.angle_max = angle_max_;
    obstacle_msg.angle_min = angle_min_;

    mavros_obstacle_publisher_.publish(obstacle_msg);

    // Prior code example below- uses Mavlink message directly
    // conn.mav.obstacle_distance_send(
    //     current_time_us,    // us Timestamp (UNIX time or time since system boot)
    //     0,                  // sensor_type, defined here: https://mavlink.io/en/messages/common.html#MAV_DISTANCE_SENSOR
    //     distances,          // distances,    uint16_t[72],   cm
    //     0,                  // increment,    uint8_t,        deg
    //     min_depth_cm,	    // min_distance, uint16_t,       cm
    //     max_depth_cm,       // max_distance, uint16_t,       cm
    //     increment_f,	    // increment_f,  float,          deg
    //     angle_offset,       // angle_offset, float,          deg
    //     12                  // MAV_FRAME, vehicle-front aligned: https://mavlink.io/en/messages/common.html#MAV_FRAME_BODY_FRD    
    // )
}