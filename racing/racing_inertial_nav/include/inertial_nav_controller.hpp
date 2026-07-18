#ifndef INERTIAL_NAV_CONTROLLER_HPP
#define INERTIAL_NAV_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ai_msgs/msg/perception_targets.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/opencv.hpp>

#include <vector>
#include <queue>
#include <cmath>
#include <chrono>
#include <string>

namespace racing_inertial_nav
{

// Waypoint structure with tolerance parameters
struct Waypoint {
    double x;              // Target X position (meters, relative to origin)
    double y;              // Target Y position (meters, relative to origin)
    double yaw;            // Target orientation (radians), NaN if not specified
    
    // Tolerance parameters
    double distance_tolerance;  // 欧氏距离容差（米），到达判断的唯一依据
    double yaw_tolerance;  // Yaw tolerance (radians)
    
    // Timeout for this waypoint (seconds, 0 = no timeout)
    double timeout;
    
    // Pause duration at waypoint (seconds, 0 = no pause, pass through)
    // Used for QR code scanning, sensor readings, etc.
    double pause_duration;
    
    Waypoint(double x_, double y_, double yaw_ = NAN,
             double yaw_tol = 0.1, double timeout_ = 10.0,
             double dist_tol = 0.2, double pause_ = 0.0) 
        : x(x_), y(y_), yaw(yaw_),
          distance_tolerance(dist_tol > 0 ? dist_tol : 0.1),
          yaw_tolerance(yaw_tol), timeout(timeout_), pause_duration(pause_) {}
    
    bool has_yaw_target() const {
        return !std::isnan(yaw);
    }
    
    bool has_pause() const {
        return pause_duration > 0.0;
    }
    
    // 获取有效的距离容差（下限保护 0.1m）
    double get_distance_tolerance() const {
        return distance_tolerance > 0 ? distance_tolerance : 0.1;
    }
};

// PID Controller class
class PIDController {
public:
    PIDController(double kp, double ki, double kd, 
                  double output_min, double output_max);
    
    double compute(double error, double dt);
    void reset();
    
private:
    double kp_, ki_, kd_;
    double output_min_, output_max_;
    double integral_;
    double prev_error_;
    double prev_time_;
    bool first_run_;
};

// Main Inertial Navigation Controller
class InertialNavController : public rclcpp::Node
{
public:
    InertialNavController();
    ~InertialNavController() = default;
    
    // Public accessor for graceful shutdown
    void publish_stop_command();

private:
    // Callbacks
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void obstacle_callback(const ai_msgs::msg::PerceptionTargets::SharedPtr msg);
    void control_loop();
    
    // Navigation functions
    void initialize_origin();
    bool navigate_to_waypoint(const Waypoint& waypoint, double dt);  // 修复：添加 dt 参数
    void stop_robot();
    void emergency_stop();
    
    // Utility functions
    double normalize_angle(double angle);
    double calculate_distance(double x1, double y1, double x2, double y2);
    double calculate_angle_diff(double current_yaw, double target_yaw);
    void global_to_local(double global_x, double global_y, double global_yaw,
                        double& local_x, double& local_y, double& local_yaw);
    double decide_target_yaw(const Waypoint& waypoint, int current_idx);
    void load_waypoints_from_params();
    void load_default_waypoints();  // Load hardcoded default waypoints
    
    // Pure Pursuit functions
    bool find_lookahead_point(double lookahead_dist, double& target_x, double& target_y);
    double compute_pure_pursuit_control(double target_x, double target_y, double& linear_cmd, double dt);  // 修复：添加 dt 参数

    // Parking functions
    void execute_parking(double dt);
    double estimate_park_distance();

    // Obstacle avoidance functions
    void enter_detour();
    std::vector<Waypoint> compute_detour_waypoints();
    int calculate_actual_area(int max_barrel_area, int a_x);

    // Geofence (highest priority safety constraint)
    bool check_geofence();  // returns true if vehicle is outside fence
    
    // ROS2 interfaces
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr obstacle_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // State variables
    enum class NavState {
        IDLE,
        INITIALIZING,
        NAVIGATING,
        RETURNING,
        PARKING,
        COMPLETED
    };
    
    NavState current_state_;
    bool origin_initialized_;
    
    // Origin pose (global coordinates from odom_combined)
    double origin_global_x_;
    double origin_global_y_;
    double origin_global_yaw_;
    
    // Current pose in global coordinates
    double current_global_x_;
    double current_global_y_;
    double current_global_yaw_;
    
    // Current pose in local coordinates (relative to origin)
    double current_local_x_;
    double current_local_y_;
    double current_local_yaw_;
    
    // Waypoint navigation
    std::vector<Waypoint> waypoints_;
    int current_waypoint_idx_;
    rclcpp::Time waypoint_start_time_;
    
    // 航点暂停状态
    bool is_pausing_;                    // 是否正在暂停
    rclcpp::Time pause_start_time_;      // 暂停开始时间
    
    // PID controller (angular only, for orientation adjustment)
    std::unique_ptr<PIDController> angular_pid_;
    
    // Motion control parameters
    double max_acceleration_;      // m/s²
    double current_linear_speed_;  // For smooth acceleration
    double lookahead_distance_;    // For pure pursuit
    double min_lookahead_distance_; // Minimum lookahead distance
    double min_turning_radius_;    // Minimum turning radius (m) for curvature clamping

    // Dynamic lookahead parameters (距离自适应预瞄)
    double dynamic_lookahead_max_;       // 远距离预瞄上限 (m)
    double dynamic_lookahead_min_;       // 近距离预瞄下限 (m)
    double dist_far_threshold_;          // 远距离阈值, >=此值用最大预瞄 (m)
    double dist_near_threshold_;         // 近距离阈值, <=此值用最小预瞄 (m)
    double lookahead_lpf_alpha_;         // 低通滤波系数 (0~1, 越小越平滑)
    double lookahead_filtered_;          // 滤波后的预瞄距离

    // 接近减速参数
    double decel_distance_;              // m - 开始减速的距离（距航点多远开始减速）

    // Yaw 渐进式混合参数
    double yaw_blend_start_distance_;    // m - 开始混合姿态控制的距离（距航点多远开始偏航调整）

    // Reverse driving parameters
    bool enable_reverse_;          // Enable reverse driving
    double max_reverse_speed_;     // Maximum reverse speed (m/s)
    double reverse_angle_threshold_; // Angle threshold for reverse decision (radians)
    
    // Parameters
    double default_position_tolerance_;  // meters (用于到达判定)
    double default_angle_tolerance_;     // radians
    double default_waypoint_tolerance_;  // meters - 默认航点位置容差（航点未指定时使用）
    double min_waypoint_tolerance_;      // meters - 最小航点位置容差（下限）
    double max_linear_speed_;            // m/s
    double max_angular_speed_;           // rad/s
    double control_frequency_;           // Hz
    double default_waypoint_timeout_;    // seconds
    
    // Timing
    rclcpp::Time last_control_time_;
    rclcpp::Time last_odom_time_;

    // ==================== Obstacle Avoidance (detour-based) ====================
    // Detection data
    int max_barrel_area_ = 0;
    int actual_barrel_area_ = 0;
    int max_barrel_bottom_ = 0;
    int a_x_ = 0, a_y_ = 0;           // largest barrel center x / bottom y
    bool barrel_processed_ = false;

    // Detour state
    bool detour_active_ = false;
    bool is_left_ = false;            // true=barrel on left → dodge right
    std::vector<Waypoint> saved_waypoints_;
    int saved_waypoint_idx_ = 0;
    rclcpp::Time detour_cooldown_;

    // Avoidance parameters
    int avoid_area_ = 14000;
    int avoid_trigger_bottom_ = 300;
    double avoid_linear_speed_ = 0.7;
    double avoid_detour_lateral_offset_ = 0.5;  // m - sideways dodge distance
    double avoid_detour_forward_step_ = 0.8;    // m - forward step per waypoint
    double avoid_detour_cooldown_s_ = 2.0;      // s - cooldown between detours

    // ==================== Parking (YOLO-based) ====================
    bool park_detected_ = false;         // 是否检测到 park 目标
    int park_center_x_ = 0;              // park bbox 中心 x
    int park_area_ = 0;                  // park bbox 面积
    bool park_aligned_ = false;          // 是否完成对准
    double park_start_x_ = 0.0;          // 入库起始位置 x (local)
    double park_start_y_ = 0.0;          // 入库起始位置 y (local)
    double park_target_distance_ = 0.0;  // 目标入库距离 (m)
    bool park_distance_set_ = false;     // 是否已设定入库距离
    cv::Mat homography_M_;               // 逆透视变换矩阵

    // Parking tunable parameters (from YAML)
    double park_gain_far_ = 3.0;
    double park_gain_near_ = 2.0;
    double park_align_threshold_ = 2.0;  // degrees
    double park_entry_speed_ = 0.8;      // m/s
    double park_dist_multiplier_ = 1.05;

    // ==================== Geofence (坐标围栏 - 最高优先级安全约束) ====================
    double fence_x_min_ = -0.1;
    double fence_x_max_ = 4.3;
    double fence_y_min_ = -0.15;
    double fence_y_max_ = 4.3;
    bool fence_enabled_ = true;
};

} // namespace racing_inertial_nav

#endif // INERTIAL_NAV_CONTROLLER_HPP
