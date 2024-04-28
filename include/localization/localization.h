#include "particle_filter.h"
#include "helper_structs.h"
#include "map.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int8.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <chrono>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <ctime>

class Localization : public rclcpp::Node {
public:
    Localization();
    ~Localization();

private:
    // ROS2 Msg Type
    sensor_msgs::msg::LaserScan::SharedPtr lane_msg;
    sensor_msgs::msg::Imu::SharedPtr imu_msg;
    nav_msgs::msg::Odometry::SharedPtr odom_msg;
    std_msgs::msg::Int8::SharedPtr decision_msg;

    // ROS2 Subscribe
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lane_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr decision_subscription_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_subscription_;
    void lane_callback(const sensor_msgs::msg::LaserScan::SharedPtr lane_msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    void decision_callback(const std_msgs::msg::Int8::SharedPtr decision_msg);
    void drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr drive_msg);

    void updateOdometry();
    void updateLane();

    Pose moveToCenter(Pose front_pose);

    // ROS2 Publish
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr particle_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;

    rclcpp::TimerBase::SharedPtr publisher_timer_;

    void publisher_timer_callback();
    void publish_pose(Pose center_pose);
    void publish_particle(std::vector<Particle> particles);
    geometry_msgs::msg::PoseStamped set_pose(geometry_msgs::msg::PoseStamped pose_msg, Pose pose);
    float quatToYaw(const geometry_msgs::msg::Quaternion quat);
    bool isSensorValid();
    
    // Set map
    Map map;

    // Set CrossWalk Position
    std::vector<Crosswalk> cross_walk_areas;

    // Sensor Data
    Odometry odometry;
    Lane front_lane;
    Lane rear_lane;
    Decision decision;

    float front_length;

    // Pose Vector
    Pose front_pose;
    Pose center_pose;

    double cos_initial_roll;
    double sin_initial_roll;

    // Particle Filter
    ParticleFilter pf;

    int imu_count;
    float imu_yawrate;
    int odom_count;
    float odom_velocity;

    int direction;
    float on_lane_threshold;

    // Publish Particle Condition
    bool isPublishParticle;
};