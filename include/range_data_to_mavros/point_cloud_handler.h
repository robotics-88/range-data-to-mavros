/* 
© 2023 Robotics 88
Author: Gus Meyer <gus@robotics88.com> 
*/

#ifndef POINT_CLOUD_HANDLER_H_
#define POINT_CLOUD_HANDLER_H_

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_ros/point_cloud.h>
#include "pcl_ros/transforms.h"

/**
 * @class PointCloudHandler
 * @brief Converts point cloud (e.g. 3D Lidar) data to mavros obstacle messages required for obstacle avoidance.
 */
class PointCloudHandler {

    public:
        PointCloudHandler(ros::NodeHandle& node);
        ~PointCloudHandler();

        void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
        void dronePoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    private:
        ros::NodeHandle private_nh_;
        ros::NodeHandle nh_;

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        std::string point_cloud_topic_;
        std::string mavros_obstacle_topic_;
        ros::Subscriber point_cloud_subscriber_;

        std::vector<float> latest_distances_;
        double last_obstacle_distance_sent_ms;
        ros::Publisher mavros_obstacle_publisher_;

        geometry_msgs::PoseStamped last_pose_;

        // Params
        std::string target_frame_;
        double distances_array_length_;
        double min_height_;
        double max_height_;
        double angle_min_;
        double angle_max_;
        double angle_increment_;
        double scan_time_;
        double range_min_;
        double range_max_;
        double inf_epsilon_;
        double use_inf_;

        bool vehicle_state_received_;
};

#endif