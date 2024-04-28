#include "localization.h"

Localization::Localization() : rclcpp::Node("localization") {
    this->declare_parameter("publish_particle", true);

    this->declare_parameter("init_sigma_x", 2.5);
    this->declare_parameter("init_sigma_y", 2.5);
    this->declare_parameter("init_sigma_theta", 0.01);

    this->declare_parameter("sigma_x", 4.0);
    this->declare_parameter("sigma_y", 3.0);
    this->declare_parameter("sigma_theta", 0.01);
    this->declare_parameter("sigma_diff_range", 3.0);

    this->declare_parameter("center_pose_x", 74.0);
    this->declare_parameter("center_pose_y", 50.0);
    this->declare_parameter("center_pose_theta", 0.0);

    this->declare_parameter("delta_t", 0.1);
    this->declare_parameter("node_threshold", 5);
    this->declare_parameter("on_lane_threshold", 1.0);
    this->declare_parameter("stop_velocity_threshold", 2.0);
    this->declare_parameter("num_particles", 130);

    this->declare_parameter("front_length", 21.0);
    this->declare_parameter("initial_roll", 18.0);

    this->declare_parameter("map_file_path", "/home/seame/seame_ws/src/localization/include/localization/test_map/test_map_horizontal_line.txt");
    this->declare_parameter("map_scale", 0.254);
    this->declare_parameter("map_origin_x", 0.0);
    this->declare_parameter("map_origin_y", 0.0);

    this->declare_parameter("cross_walk_file_path", "/home/seame/seame_ws/src/localization/include/localization/test_map/test_map_cross_walk.txt");

    isPublishParticle = this->get_parameter("publish_particle").as_bool();

    // Intitial sigma
    float init_sigma[3];
    init_sigma[0] = this->get_parameter("init_sigma_x").as_double();
    init_sigma[1] = this->get_parameter("init_sigma_y").as_double();
    init_sigma[2] = this->get_parameter("init_sigma_theta").as_double();

    // Sensor sigma
    float sigma[4];
    sigma[0] = this->get_parameter("sigma_x").as_double();
    sigma[1] = this->get_parameter("sigma_y").as_double();
    sigma[2] = this->get_parameter("sigma_theta").as_double();
    sigma[3] = this->get_parameter("sigma_diff_range").as_double();

    // Intitial Pose
    center_pose.x = this->get_parameter("center_pose_x").as_double();
    center_pose.y = this->get_parameter("center_pose_y").as_double();
    center_pose.theta = this->get_parameter("center_pose_theta").as_double();
    
    // Sensor uncertainty [x,y,theta]
    float delta_t = this->get_parameter("delta_t").as_double();
    int publish_interval = static_cast<int>(1000 * delta_t);
    int num_particles = this->get_parameter("num_particles").as_int();

    int node_threshold = this->get_parameter("node_threshold").as_int();
    float on_lane_threshold = this->get_parameter("on_lane_threshold").as_double();
    float stop_velocity_threshold = this->get_parameter("stop_velocity_threshold").as_double();


    // Set Length from Center to Front (cm)
    this->front_length = this->get_parameter("front_length").as_double();

    // Set Camera Roll Angle (deg)
    double initial_roll = this->get_parameter("initial_roll").as_double(); // deg
    initial_roll = deg_to_rad(initial_roll);
    this->cos_initial_roll = cos(initial_roll);
    this->sin_initial_roll = sin(initial_roll);
    
    // Original Map Parameter
    std::string map_file_path = this->get_parameter("map_file_path").as_string();
    float map_scale = this->get_parameter("map_scale").as_double();
    float map_origin_x = this->get_parameter("map_origin_x").as_double();
    float map_origin_y = this->get_parameter("map_origin_y").as_double();


    // Cross Walk Map Parameter
    std::string cross_walk_file_path = this->get_parameter("cross_walk_file_path").as_string();

    this->imu_count = 0;
    this->imu_yawrate = 0;
    this->odom_count = 0;
    this->odom_velocity = 0;

    this->direction = 1;
    
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/localization/pose", 10);
    particle_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/localization/particles", 10);

    lane_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/perception/lane", 10, std::bind(&Localization::lane_callback, this,  std::placeholders::_1));
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/camera/imu", 10, std::bind(&Localization::imu_callback, this,  std::placeholders::_1));
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/sensor/odom", 10, std::bind(&Localization::odom_callback, this,  std::placeholders::_1));
    // decision_subscription_ = this->create_subscription<std_msgs::msg::Int8>(
    // "/planner/state", 10, std::bind(&Localization::decision_callback, this,  std::placeholders::_1));
    drive_subscription_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
    "/controller/drive", 10, std::bind(&Localization::drive_callback, this,  std::placeholders::_1));

    publisher_timer_ = this->create_wall_timer(
        // Problem with OpenGL when timer set 100ms
        std::chrono::milliseconds(publish_interval),
        std::bind(&Localization::publisher_timer_callback, this)
    );
    
    map = map_load_occ(map_file_path, map_scale, map_origin_x, map_origin_y);
    cross_walk_areas = cross_walk_load_occ(cross_walk_file_path);

    pf.init(center_pose, init_sigma, sigma, delta_t, node_threshold, on_lane_threshold, stop_velocity_threshold, num_particles, front_length);
}

Localization::~Localization() {

}

void Localization::lane_callback(const sensor_msgs::msg::LaserScan::SharedPtr lane_msg){
    this->lane_msg = lane_msg;
}

void Localization::imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
    this->imu_count += 1;
    this->imu_yawrate -= (this->cos_initial_roll * imu_msg->angular_velocity.y) + (this->sin_initial_roll * imu_msg->angular_velocity.z);
}

void Localization::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    this->odom_count += 1;
    this->odom_velocity += odom_msg->twist.twist.linear.x;
}

// void Localization::decision_callback(const std_msgs::msg::Int8::SharedPtr decision_msg) {
//     this->decision.decision = decision_msg->data;
// }

void Localization::drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr drive_msg) {
    if (drive_msg->drive.speed < -0.5) {this->direction = -1;}
    else if (drive_msg->drive.speed > 0.5) {this->direction = 1;}
}

void Localization::publisher_timer_callback() {
    if (!isSensorValid()) {return;}

    // RCLCPP_INFO(this->get_logger(), "Processing");

    // Read Lane Data
    updateLane();

    // Read Odometry Data
    updateOdometry();

    // Start Localization
    center_pose = pf.updateLocalization(&map, cross_walk_areas, center_pose, odometry, front_lane, rear_lane, direction);

    // ROS2 Publish
    publish_pose(center_pose);

    if(isPublishParticle) {publish_particle(pf.get_particle());}
}

void Localization::updateOdometry() {
    // Read Data (Average)
    odometry.yawrate = this->imu_yawrate / this->imu_count;
    odometry.velocity = this->odom_velocity / this->odom_count;

    // Initialize Storage
    this->imu_count = 0;
    this->imu_yawrate = 0;
    this->odom_count = 0;
    this->odom_velocity = 0;
}

void Localization::updateLane() {
    // Initialize Lane Storage
    front_lane.thetas.clear();
    front_lane.ranges.clear();
    front_lane.in_range_count = 0;
    front_lane.range_average = 0.0;
    front_lane.range_max = lane_msg->range_max;

    float range_average = 0.0; 
    int count = static_cast<int> (std::round((lane_msg->angle_max - lane_msg->angle_min) / lane_msg->angle_increment));
    for (int i = 0; i < count; i += 1) {
        // ranges is -1.0 if max
        if(lane_msg->ranges[i] > 0) {
            front_lane.in_range_count++;
            range_average += lane_msg->ranges[i];
        }
        front_lane.thetas.push_back(lane_msg->angle_increment * i + lane_msg->angle_min);
        front_lane.ranges.push_back(lane_msg->ranges[i]);
    }
    range_average /= front_lane.in_range_count;
    front_lane.range_average = range_average;
}

Pose Localization::moveToCenter(Pose front_pose) {
    Pose center_pose;
    center_pose.x = front_pose.x - this->front_length * cos(front_pose.theta);
    center_pose.y = front_pose.y - this->front_length * sin(front_pose.theta);
    center_pose.theta = front_pose.theta;

    return center_pose;
}

void Localization::publish_pose(Pose center_pose) {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg = this->set_pose(pose_msg, center_pose);
    pose_publisher_->publish(pose_msg);

    // // // // // // // // // // // // // // // // // // // // // 
    // RCLCPP_INFO(this->get_logger(), "XYT");
    // RCLCPP_INFO(this->get_logger(), std::to_string(center_pose.x));
    // RCLCPP_INFO(this->get_logger(), std::to_string(center_pose.y));
    // RCLCPP_INFO(this->get_logger(), std::to_string(center_pose.theta));
    // // // // // // // // // // // // // // // // // // // // // 
}

void Localization::publish_particle(std::vector<Particle> particles) {
    // Generate Message
    sensor_msgs::msg::PointCloud2 particles_msg;
    sensor_msgs::PointCloud2Modifier modifier(particles_msg);

    modifier.setPointCloud2Fields(3,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32);

    particles_msg.header = std_msgs::msg::Header();
    particles_msg.header.stamp = rclcpp::Clock().now();
    particles_msg.header.frame_id = "map";

    particles_msg.height = 1;
    particles_msg.width = particles.size();
    particles_msg.is_dense = true;

    particles_msg.is_bigendian = false;
    particles_msg.point_step = 12;
    particles_msg.row_step = particles_msg.point_step * particles.size();
    particles_msg.data.resize(particles_msg.row_step);

    sensor_msgs::PointCloud2Iterator<float> pf_iter_x(particles_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> pf_iter_y(particles_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> pf_iter_z(particles_msg, "z");

    for (size_t i = 0; i < particles.size(); ++i) {
        *pf_iter_x = particles[i].x;
        *pf_iter_y = particles[i].y;
        *pf_iter_z = particles[i].weight;
        ++pf_iter_x;
        ++pf_iter_y;
        ++pf_iter_z;
        // RCLCPP_INFO(this->get_logger(), std::to_string(particles[i].weight));
    }

    particle_publisher_->publish(particles_msg);
}

geometry_msgs::msg::PoseStamped Localization::set_pose(geometry_msgs::msg::PoseStamped pose_msg, Pose pose) {
    pose_msg.header = std_msgs::msg::Header();
    pose_msg.header.frame_id = "map";
    pose_msg.header.stamp = rclcpp::Clock().now();

    pose_msg.pose.position.x = pose.x;
    pose_msg.pose.position.y = pose.y;
    
    tf2::Quaternion quat;
    quat.setRPY(0, 0, pose.theta);
    pose_msg.pose.orientation.x = quat.x();
    pose_msg.pose.orientation.y = quat.y();
    pose_msg.pose.orientation.z = quat.z();
    pose_msg.pose.orientation.w = quat.w();

    return pose_msg;
}

float Localization::quatToYaw(const geometry_msgs::msg::Quaternion quat) {
    tf2::Quaternion tf2_quat;
    tf2::fromMsg(quat, tf2_quat);
    double roll;
    double pitch;
    double yaw;
    tf2::Matrix3x3 matrix(tf2_quat);
    matrix.getRPY(roll, pitch, yaw);
    return yaw;
}

bool Localization::isSensorValid() {
    if (!this->lane_msg || !this->imu_count || !this->odom_count) {return false;}
    return true;
}
