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
#include "tf/transform_broadcaster.h"

/**
 * @class PointCloudHandler
 * @brief Converts point cloud (e.g. 3D Lidar) data to mavros obstacle messages required for obstacle avoidance.
 */
class PointCloudHandler {

    public:
        PointCloudHandler(ros::NodeHandle& node);
        ~PointCloudHandler();

        void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

    private:
        ros::NodeHandle private_nh_;
        ros::NodeHandle nh_;

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        tf2_ros::TransformBroadcaster tf_b_;

        ros::Subscriber point_cloud_subscriber_;
        ros::Publisher mavros_obstacle_publisher_;

        std::string point_cloud_topic_;
        std::string mavros_obstacle_topic_;

        double last_obstacle_distance_sent_ms;
        
        // Params
        std::string target_frame_; // This should be a 'stabilized' FRD frame
        std::string frd_frame_; // The 'unstabilized' FRD frame of the vehicle
        double distances_array_length_; // Number of fields in the distances array
        double min_height_; // Height relative to the vehilce below which points will be ignored
        double max_height_; // Height relative to the vehilce below which points will be ignored
        double angle_min_; // Minimum 'angle' of the Laserscan
        double angle_max_; // Maximum 'angle' of the Laserscan
        double angle_increment_; // Angle increment between distance array fields
        double scan_time_; // Not used
        double range_min_; // Minimum relevant range of the rangefinder
        double range_max_; // Maximum relevant range of the rangefinder
        double orientation_timeout_; // How long ago we will still use latest orientation data for pitch/roll correction
};

#endif