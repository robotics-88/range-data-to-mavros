/* 
© 2023 Robotics 88
Author: Gus Meyer <gus@robotics88.com> 
*/

#include "range_data_to_mavros/point_cloud_handler.h"

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <math.h>

PointCloudHandler::PointCloudHandler(ros::NodeHandle& node)
  : private_nh_("~")
  , nh_(node)
  , tf_listener_(tf_buffer_)
  , point_cloud_topic_("/velodyne_points")
  , mavros_obstacle_topic_("/mavros/obstacle/send")
  , last_obstacle_distance_sent_ms(ros::Time(0).toSec())
  , obstacle_params_set_(false)
  , depth_scale(1.0)
  , obstacle_line_height_ratio(0.18)
  , obstacle_line_thickness_pixel(10)
  , distances_array_length(72)
  , min_depth_m(0.25)
  , max_depth_m(10.0)
  , vehicle_state_received_(false)
{   
    private_nh_.param<std::string>("point_cloud_topic", point_cloud_topic_, point_cloud_topic_);
    private_nh_.param<std::string>("mavros_obstacle_topic", mavros_obstacle_topic_, mavros_obstacle_topic_);

    point_cloud_subscriber_ = nh_.subscribe<sensor_msgs::PointCloud2>(point_cloud_topic_, 10, &PointCloudHandler::pointCloudCallback, this);

    mavros_obstacle_publisher_ = nh_.advertise<sensor_msgs::LaserScan>(mavros_obstacle_topic_, 10);
}

PointCloudHandler::~PointCloudHandler(){}

void PointCloudHandler::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {

    std::cout << "Getting point cloud" << std::endl;

    // Convert to pcl::PointCloud
    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    pcl::fromROSMsg(*msg, point_cloud);

    // point_cloud.points contains xyz data of points.

    

}

// TODO 
void PointCloudHandler::setObstacleDistanceParams(const sensor_msgs::PointCloud2 &msg) {

    obstacle_params_set_ = true;
}

void PointCloudHandler::distancesFromPointCloud(const sensor_msgs::PointCloud2 &point_cloud, std::vector<float> &distances){




    float dist_m = 0;

    // # Note that dist_m is in meter, while distances[] is in cm.
    if (dist_m > min_depth_m and dist_m < max_depth_m) {
        distances.push_back(dist_m);
    }
    else {
        // # Default value, unless overwritten: 
        // #   A value of max_distance + 1 (cm) means no obstacle is present. 
        // #   A value of UINT16_MAX (65535) for unknown/not used.
        distances.push_back(65535);
    }

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
    if (!obstacle_params_set_){
        ROS_WARN("params not set");
        return;
    }
    else {
        sensor_msgs::LaserScan obstacle_msg;
        obstacle_msg.header.frame_id = "base_link";
        obstacle_msg.header.stamp = header.stamp;
        obstacle_msg.range_max = max_depth_m;
        obstacle_msg.range_min = min_depth_m;
        obstacle_msg.ranges = distances;
        obstacle_msg.angle_increment = increment_f * M_PI / 180;
        obstacle_msg.time_increment = 0;
        obstacle_msg.scan_time = 0;
        obstacle_msg.angle_max = max_angle_rad;
        obstacle_msg.angle_min = min_angle_rad;

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
}