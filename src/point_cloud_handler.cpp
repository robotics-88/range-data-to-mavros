/* 
© 2023 Robotics 88
Author: Gus Meyer <gus@robotics88.com> 
*/

#include "range_data_to_mavros/point_cloud_handler.h"
#include <math.h>

PointCloudHandler::PointCloudHandler(ros::NodeHandle& node)
    : private_nh_("~")
    , nh_(node)
    , tf_listener_(tf_buffer_)
    , point_cloud_topic_("/velodyne_points")
    , mavros_obstacle_topic_("/mavros/obstacle/send")
    , frd_frame_("")
    , target_frame_("")
    , last_obstacle_distance_sent_ms(ros::Time(0).toSec())
    , distances_array_length_(72)
    , min_height_(std::numeric_limits<double>::min())
    , max_height_(std::numeric_limits<double>::max())
    , angle_min_(0)
    , angle_max_(2.0*M_PI)
    , range_min_(0.0)
    , range_max_(std::numeric_limits<double>::max())
    , orientation_timeout_(0.25)
{   
    private_nh_.param<std::string>("point_cloud_topic", point_cloud_topic_, point_cloud_topic_);
    private_nh_.param<std::string>("mavros_obstacle_topic", mavros_obstacle_topic_, mavros_obstacle_topic_);
    private_nh_.param<std::string>("target_frame", target_frame_, target_frame_);
    private_nh_.param<std::string>("frd_frame", frd_frame_, frd_frame_);
    private_nh_.param<double>("min_height", min_height_, min_height_);
    private_nh_.param<double>("max_height", max_height_, max_height_);
    private_nh_.param<double>("angle_min", angle_min_, angle_min_);
    private_nh_.param<double>("angle_max", angle_max_, angle_max_);
    private_nh_.param<double>("range_min", range_min_, range_min_);
    private_nh_.param<double>("range_max", range_max_, range_max_);
    private_nh_.param<double>("imu_timeout", orientation_timeout_, orientation_timeout_);

    angle_increment_ = (angle_max_ - angle_min_) / distances_array_length_;

    point_cloud_subscriber_ = nh_.subscribe<sensor_msgs::PointCloud2>(point_cloud_topic_, 10, &PointCloudHandler::pointCloudCallback, this);

    mavros_obstacle_publisher_ = nh_.advertise<sensor_msgs::LaserScan>(mavros_obstacle_topic_, 10);

    stabilized_pointcloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("/stabilized_pointcloud", 10);
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

    // Handle point cloud and put it in PCL format
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_frd(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_frd_stabilized(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_processed(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);

    // Transform to FRD frame (this is what MAVROS expects)
    geometry_msgs::TransformStamped pcl_tf;
    try {
        pcl_tf = tf_buffer_.lookupTransform(frd_frame_, msg->header.frame_id, msg->header.stamp);
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        return;
    }
    pcl_ros::transformPointCloud(*cloud, *cloud_frd, pcl_tf.transform);

    geometry_msgs::TransformStamped stab_tf;
    try {
        stab_tf = tf_buffer_.lookupTransform(frd_frame_, "map_ned", ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        return;
    }

    // Apply drone orientation correction if vehicle orientation was received recently
    if ((stab_tf.header.stamp - ros::Time::now()) < ros::Duration(orientation_timeout_)) {

        // Get rotation in FRD
        tf2::Quaternion q_FRD;
        tf2::fromMsg(stab_tf.transform.rotation, q_FRD);

        // Get RPY from quaternion
        tf2Scalar roll, pitch, yaw;
        tf2::Matrix3x3 mat(q_FRD);
        mat.getRPY(roll, pitch, yaw);

        // 'Stabilize' the frame by applying pitch/roll rotation (but no yaw)
        q_FRD.setRPY(roll, pitch, 0.0);

        // Create transform
        geometry_msgs::TransformStamped frd_stabilized;
        frd_stabilized.header.stamp = ros::Time::now();
        frd_stabilized.header.frame_id = frd_frame_;
        frd_stabilized.child_frame_id = target_frame_;
        frd_stabilized.transform.translation.x = 0;
        frd_stabilized.transform.translation.y = 0;
        frd_stabilized.transform.translation.z = 0;
        frd_stabilized.transform.rotation.x = q_FRD.getX();
        frd_stabilized.transform.rotation.y = q_FRD.getY();
        frd_stabilized.transform.rotation.z = q_FRD.getZ();
        frd_stabilized.transform.rotation.w = q_FRD.getW();
        tf_b_.sendTransform(frd_stabilized);

        // Rotate point cloud points by inverse transform to apply pitch/roll correction for this frame
        q_FRD.setRPY(-roll, -pitch, 0.0);

        frd_stabilized.transform.rotation.x = q_FRD.getX();
        frd_stabilized.transform.rotation.y = q_FRD.getY();
        frd_stabilized.transform.rotation.z = q_FRD.getZ();
        frd_stabilized.transform.rotation.w = q_FRD.getW();

        // Transform point cloud
        pcl_ros::transformPointCloud(*cloud_frd, *cloud_frd_stabilized, frd_stabilized.transform);
        cloud_processed = cloud_frd_stabilized;

        cloud_frd_stabilized->header.frame_id = target_frame_;
        stabilized_pointcloud_publisher_.publish(cloud_frd_stabilized);
    }
    else {
        cloud_processed = cloud_frd;
        ROS_WARN("No recent vehicle orientation data, using unstabilized point cloud");
    }

    // Iterate through points
    for (auto &point : *cloud_processed)
    {

        // Filter out points outside height range relative to drone
        // Remember, positive 'z' is down, hence the negative sign
        if (-point.z > max_height_ || -point.z < min_height_) {
            // ROS_DEBUG("rejected for height %f not in range (%f, %f)\n", point.getZ(), min_height_, max_height_);
            continue;
        }

        double range = hypot(point.x, point.y);
        if (range < range_min_) {
            //ROS_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range, range_min_, point.getX(), point.getY(), point.getZ());
            continue;
        }

        if (range > range_max_) {
            //ROS_DEBUG("rejected for range %f above maximum value %f. Point: (%f, %f, %f)", range, range_max_, point.getX(), point.getY(), point.getZ());
            continue;
        }

        double angle = atan2(point.y, point.x);
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