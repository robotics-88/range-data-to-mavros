/* 
© 2023 Robotics 88
Author: Gus Meyer <gus@robotics88.com> 
*/

#ifndef POINT_CLOUD_HANDLER_H_
#define POINT_CLOUD_HANDLER_H_

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>

/**
 * @class PointCloudHandler
 * @brief Converts point cloud (e.g. 3D Lidar) data to mavros obstacle messages required for obstacle avoidance.
 */
class PointCloudHandler {

    public:
        PointCloudHandler(ros::NodeHandle& node);
        ~PointCloudHandler();

        void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

    private:
        ros::NodeHandle private_nh_;
        ros::NodeHandle nh_;

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        std::string point_cloud_topic_;
        std::string mavros_obstacle_topic_;
        message_filters::Subscriber<sensor_msgs::PointCloud2> point_cloud_subscriber_;

        /* typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::CameraInfo> MySyncPolicy;
        typedef message_filters::Synchronizer<MySyncPolicy> Sync;
        boost::shared_ptr<Sync> sync_; */

        std::vector<float> latest_distances_;
        double last_obstacle_distance_sent_ms;
        ros::Publisher mavros_obstacle_publisher_;

        // Obstacle params based on librealsense example
        bool obstacle_params_set_;
        int depth_height;
        int depth_width;
        double angle_offset;
        double camera_facing_angle_degree = 0; // appears to be always 0 - verify later, but makes sense if this param is the rotation of camera, expect always facing forward
        double increment_f;
        double depth_scale;
        double depth_hfov_deg;
        double depth_vfov_deg;
        double obstacle_line_height_ratio; // [0-1]: 0-Top, 1-Bottom. The height of the horizontal line to find distance to obstacle.
        int obstacle_line_thickness_pixel; // [1-DEPTH_HEIGHT]: Number of pixel rows to use to generate the obstacle distance message. For each column, the scan will return the minimum value for those pixels centered vertically in the image.
        int distances_array_length;
        double min_depth_m;
        double max_depth_m;
        double min_angle_rad;
        double max_angle_rad;

        double vehicle_pitch_rad;
        bool vehicle_state_received_;

        void setObstacleDistanceParams(const sensor_msgs::CameraInfoConstPtr &info);
        void distancesFromPointCloud(const sensor_msgs::PointCloud2 &point_cloud, std::vector<float> &distances);
        int findObstacleLineHeight();
        void sendObstacleDistanceMessage(const std_msgs::Header &header, const std::vector<float> &distances);
};

#endif