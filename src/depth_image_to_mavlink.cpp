/* 
© 2023 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "depth_image_to_mavlink/depth_image_to_mavlink.h"

#include <math.h>
#include <opencv2/photo.hpp>

namespace depth_image_to_mavlink
{
Depth_image_to_mavlink::Depth_image_to_mavlink(ros::NodeHandle& node)
  : private_nh_("~")
  , nh_(node)
  , depth_topic_("zed2i/zed_node/depth/depth_registered")
  , depth_info_topic_("zed2i/zed_node/depth/camera_info")
  , last_obstacle_distance_sent_ms(ros::Time(0).toSec())
  , obstacle_params_set_(false)
  , depth_scale(1.0)
  , obstacle_line_height_ratio(0.18)
  , obstacle_line_thickness_pixel(10)
  , distances_array_length(72)
  , min_depth_m(0.2)
  , max_depth_m(10.0)
  , vehicle_state_received_(false)
{   
    private_nh_.param<std::string>("depth_image", depth_topic_, depth_topic_);
    private_nh_.param<std::string>("depth_info", depth_info_topic_, depth_info_topic_);

    depth_image_subscriber_.subscribe(nh_, depth_topic_, 10);
    depth_info_subscriber_.subscribe(nh_, depth_info_topic_, 10);
    sync_.reset(new Sync(MySyncPolicy(10), depth_image_subscriber_, depth_info_subscriber_));
    sync_->registerCallback(boost::bind(&Depth_image_to_mavlink::depthImageCallback, this, _1, _2));

    depth_smoothed_publisher_ = nh_.advertise<sensor_msgs::Image>("depth_smoothed", 10);
    mavros_obstacle_publisher_ = nh_.advertise<sensor_msgs::LaserScan>("/mavros/obstacle/send", 10);
}

Depth_image_to_mavlink::~Depth_image_to_mavlink(){}

void Depth_image_to_mavlink::depthImageCallback(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::CameraInfoConstPtr &info) {
    // Get depth image as mat
    cv::Mat depth_mat;
    cv_bridge::CvImagePtr cv_ptr; 
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1); 
    depth_mat = cv_ptr->image;

    if (!obstacle_params_set_) {
        setObstacleDistanceParams(info);
    }

    std::vector<float> distances;
    distancesFromDepthImage(depth_mat, distances);

    sendObstacleDistanceMessage(msg->header, distances);

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

void Depth_image_to_mavlink::setObstacleDistanceParams(const sensor_msgs::CameraInfoConstPtr &info) {
    depth_height = info->height;
    depth_width = info->width;

    // # For forward facing camera with a horizontal wide view:
    // #   HFOV=2*atan[w/(2.fx)],
    // #   VFOV=2*atan[h/(2.fy)],
    // #   DFOV=2*atan(Diag/2*f),
    // #   Diag=sqrt(w^2 + h^2)
    double fx = info->P[0];
    double fy = info->P[5];
    depth_hfov_deg = 180 * (2 * atan(info->width / (2 * fx))) / M_PI;
    depth_vfov_deg = 180 * (2 * atan(info->height / (2 * fy))) / M_PI;
    ROS_INFO("INFO: Depth camera HFOV: %0.2f degrees", depth_hfov_deg);
    ROS_INFO("INFO: Depth camera VFOV: %0.2f degrees", depth_vfov_deg);

    angle_offset = camera_facing_angle_degree - (depth_hfov_deg / 2);
    increment_f = depth_hfov_deg / distances_array_length;
    ROS_INFO("INFO: OBSTACLE_DISTANCE angle_offset: %f", angle_offset);
    ROS_INFO("INFO: OBSTACLE_DISTANCE increment_f: %f", increment_f);
    ROS_INFO("INFO: OBSTACLE_DISTANCE coverage: from %f to %f degrees",
        (angle_offset, angle_offset + increment_f * distances_array_length));

    min_angle_rad = M_PI * angle_offset / 180;
    max_angle_rad = M_PI * (angle_offset + increment_f * distances_array_length) / 180;

    // TODO decide if bring back checks or remove
    // # Sanity check for depth configuration
    // if obstacle_line_height_ratio < 0 or obstacle_line_height_ratio > 1:
    //     progress("Please make sure the horizontal position is within [0-1]: %s"  % obstacle_line_height_ratio)
    //     sys.exit()

    // if obstacle_line_thickness_pixel < 1 or obstacle_line_thickness_pixel > DEPTH_HEIGHT:
    //     progress("Please make sure the thickness is within [0-DEPTH_HEIGHT]: %s" % obstacle_line_thickness_pixel)
    //     sys.exit()

    obstacle_params_set_ = true;
}

void Depth_image_to_mavlink::distancesFromDepthImage(const cv::Mat &depth_mat, std::vector<float> &distances){
    // # Parameters for obstacle distance message
    int step = std::floor(((double) depth_width) / distances_array_length);

    for (int i = 0; i <distances_array_length; i++) {
        // # Each range (left to right) is found from a set of rows within a column
        // #  [ ] -> ignored
        // #  [x] -> center + obstacle_line_thickness_pixel / 2
        // #  [x] -> center = obstacle_line_height (moving up and down according to the vehicle's pitch angle)
        // #  [x] -> center - obstacle_line_thickness_pixel / 2
        // #  [ ] -> ignored
        // #   ^ One of [distances_array_length] number of columns, from left to right in the image
        int center_pixel = findObstacleLineHeight();
        int upper_pixel = center_pixel + obstacle_line_thickness_pixel / 2;
        int lower_pixel = center_pixel - obstacle_line_thickness_pixel / 2;

        // # Sanity checks
        if (upper_pixel > depth_height) {
            upper_pixel = depth_height;
        }
        else if (upper_pixel < 1) {
            upper_pixel = 1;
        }
        if (lower_pixel > depth_height) {
            lower_pixel = depth_height - 1;
        }
        else if (lower_pixel < 0) {
            lower_pixel = 0;
        }

        // # Converting depth from uint16_t unit to metric unit. depth_scale is usually 1mm following ROS convention.
        // # dist_m = depth_mat[int(obstacle_line_height), int(i * step)] * depth_scale
        cv::Mat submat = depth_mat.col(i * step).clone();
        submat = submat.rowRange(lower_pixel, upper_pixel);
        double min_point_in_scan; 
        double maxVal; 
        cv::Point minLoc; 
        cv::Point maxLoc;
        cv::minMaxLoc( submat, &min_point_in_scan, &maxVal, &minLoc, &maxLoc );
        // double min_point_in_scan = std::min(depth_mat[int(lower_pixel):int(upper_pixel), int(i * step)]);
        float dist_m = min_point_in_scan * depth_scale;

        // # Note that dist_m is in meter, while distances[] is in cm.
        if (dist_m > min_depth_m and dist_m < max_depth_m) {
            distances.push_back(dist_m * 100);
        }
        else {
            // # Default value, unless overwritten: 
            // #   A value of max_distance + 1 (cm) means no obstacle is present. 
            // #   A value of UINT16_MAX (65535) for unknown/not used.
            distances.push_back(65535);
        }
    }
}

// # Find the height of the horizontal line to calculate the obstacle distances
// #   - Basis: depth camera's vertical FOV, user's input
// #   - Compensation: vehicle's current pitch angle
int Depth_image_to_mavlink::findObstacleLineHeight() {
    // global vehicle_pitch_rad, depth_vfov_deg, DEPTH_HEIGHT

    // # Basic position
    int obstacle_line_height = depth_height * obstacle_line_height_ratio;

    // # Compensate for the vehicle's pitch angle if data is available
    if (vehicle_state_received_) {
        double depth_vfov_rad = M_PI * depth_vfov_deg / 180;
        double delta_height = sin(vehicle_pitch_rad / 2) / sin(depth_vfov_rad / 2) * depth_height;
        obstacle_line_height += delta_height;
    }

    // # Sanity check
    if (obstacle_line_height < 0) {
        obstacle_line_height = 0;
    }
    else if (obstacle_line_height > depth_height) {
        obstacle_line_height = depth_height;
    }
    
    return obstacle_line_height;
}

// # https://mavlink.io/en/messages/common.html#OBSTACLE_DISTANCE
void Depth_image_to_mavlink::sendObstacleDistanceMessage(const std_msgs::Header &header, const std::vector<float> &distances) {
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
        obstacle_msg.range_max = max_depth_m / 100;
        obstacle_msg.range_min = min_depth_m / 100;
        obstacle_msg.ranges = distances;
        obstacle_msg.angle_increment = increment_f;
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

// # https://mavlink.io/en/messages/common.html#DISTANCE_SENSOR
// void Depth_image_to_mavlink::sendSingleDistanceSensorMsg(distance, orientation) {
//     // # Average out a portion of the centermost part
//     conn.mav.distance_sensor_send(
//         0,                  # ms Timestamp (UNIX time or time since system boot) (ignored)
//         min_depth_cm,       # min_distance, uint16_t, cm
//         max_depth_cm,       # min_distance, uint16_t, cm
//         distance,           # current_distance,	uint16_t, cm	
//         0,	                # type : 0 (ignored)
//         0,                  # id : 0 (ignored)
//         orientation,        # orientation
//         0                   # covariance : 0 (ignored)
//     )
// }

// // # https://mavlink.io/en/messages/common.html#DISTANCE_SENSOR
// void Depth_image_to_mavlink::send_distance_sensor_message():
//     global distances
//     # Average out a portion of the centermost part
//     curr_dist = int(np.mean(distances[33:38]))
//     conn.mav.distance_sensor_send(
//         0,# ms Timestamp (UNIX time or time since system boot) (ignored)
//         min_depth_cm,   # min_distance, uint16_t, cm
//         max_depth_cm,   # min_distance, uint16_t, cm
//         curr_dist,      # current_distance,	uint16_t, cm	
//         0,	            # type : 0 (ignored)
//         0,              # id : 0 (ignored)
//         int(camera_facing_angle_degree / 45),              # orientation
//         0               # covariance : 0 (ignored)
//     )

}