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
    , point_cloud_topic_("")
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
    , orientation_timeout_(0.1)
{   
    private_nh_.param<std::string>("point_cloud_topic", point_cloud_topic_, point_cloud_topic_);
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

    drone_pose_subscriber_ = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &PointCloudHandler::dronePoseCallback, this);

    mavros_obstacle_publisher_ = nh_.advertise<sensor_msgs::LaserScan>("/mavros/obstacle/send", 10);
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
    geometry_msgs::TransformStamped frd_tf;
    try {
        frd_tf = tf_buffer_.lookupTransform(frd_frame_, msg->header.frame_id, msg->header.stamp);
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        return;
    }
    pcl_ros::transformPointCloud(*cloud, *cloud_frd, frd_tf.transform);

    // Apply drone orientation correction if vehicle orientation was received recently
    if ((ros::Time::now() - last_pose_.header.stamp) < ros::Duration(orientation_timeout_)) {

        // Get rotation in NWU
        tf2::Quaternion q_NWU;
        tf2::fromMsg(last_pose_.pose.orientation, q_NWU);

        // Convert to NED
        tf2::Quaternion q_data_frd, q_data_frd_inv;

        tf2::fromMsg(frd_tf.transform.rotation, q_data_frd);

        q_data_frd_inv = q_data_frd;
        q_data_frd_inv[3] = -q_data_frd[3];

        tf2::Quaternion q_NED = q_data_frd_inv * q_NWU * q_data_frd;

        // Get RPY from quaternion
        tf2Scalar roll, pitch, yaw;
        tf2::Matrix3x3 mat(q_NED);
        mat.getRPY(roll, pitch, yaw);

        // 'Stabilize' the frame by applying pitch/roll rotation (but no yaw)
        tf2::Quaternion q_NED_stab;
        q_NED_stab.setRPY(-roll, -pitch, 0.0);

        // Create transform
        geometry_msgs::TransformStamped frd_stabilized;
        frd_stabilized.header.stamp = ros::Time::now();
        frd_stabilized.header.frame_id = frd_frame_;
        frd_stabilized.child_frame_id = target_frame_;
        frd_stabilized.transform.translation.x = 0;
        frd_stabilized.transform.translation.y = 0;
        frd_stabilized.transform.translation.z = 0;
        frd_stabilized.transform.rotation.x = q_NED_stab.getX();
        frd_stabilized.transform.rotation.y = q_NED_stab.getY();
        frd_stabilized.transform.rotation.z = q_NED_stab.getZ();
        frd_stabilized.transform.rotation.w = q_NED_stab.getW();
        tf_b_.sendTransform(frd_stabilized);

        // Rotate point cloud points by inverse transform to apply pitch/roll correction for this frame
        tf2::Quaternion q_NED_stab_inv;
        q_NED_stab_inv.setRPY(roll, pitch, 0.0);

        frd_stabilized.transform.rotation.x = q_NED_stab_inv.getX();
        frd_stabilized.transform.rotation.y = q_NED_stab_inv.getY();
        frd_stabilized.transform.rotation.z = q_NED_stab_inv.getZ();
        frd_stabilized.transform.rotation.w = q_NED_stab_inv.getW();

        // Transform point cloud
        pcl_ros::transformPointCloud(*cloud_frd, *cloud_frd_stabilized, frd_stabilized.transform);
        cloud_frd_stabilized->header.frame_id = target_frame_;
        cloud_processed = cloud_frd_stabilized;
    }
    else {
        cloud_processed = cloud_frd;
        scan_msg->header.frame_id = frd_frame_;
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

void PointCloudHandler::dronePoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    last_pose_ = *msg;
}