#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <cartographer_ros_msgs/srv/finish_trajectory.hpp>
#include <cartographer_ros_msgs/srv/start_trajectory.hpp>

namespace tdk_localization {

enum class InitState {
    IDLE,
    TRIGGER_SLAM,       // SLAM 執行 init
    VERIFYING,          // 比對 /robot_pose 是否到達重置點
    CLEARING_COSTMAP,   // clear Nav2 地圖
    SUCCESS             // 初始化成功
};

class LocalizationManager : public rclcpp::Node {
public:
    explicit LocalizationManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    virtual ~LocalizationManager() = default;

private:
    // ==== callback functions ====
    void onInitCmdReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void onRobotPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void triggerSlamReset();
    void verifyPose(double current_x, double current_y, double current_yaw);
    void clearNav2Costmaps();
    
    // ==== tool function ====
    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion & q);

    // ==== ros2 msg ====
    // publisher ans subscriber
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr init_cmd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr slam_toolbox_pub_;

    // service clients
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr global_costmap_cli_;
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr local_costmap_cli_;
    rclcpp::Client<cartographer_ros_msgs::srv::FinishTrajectory>::SharedPtr carto_finish_cli_;
    rclcpp::Client<cartographer_ros_msgs::srv::StartTrajectory>::SharedPtr carto_start_cli_;

    // other variables
    std::string slam_type_;
    InitState current_state_{InitState::IDLE};
    rclcpp::TimerBase::SharedPtr fsm_timer_;
    rclcpp::Time verification_start_time_;
    rclcpp::Time success_start_time_;
    int32_t current_active_trajectory_id_;
    
    double world_to_map_x_;
    double world_to_map_y_;

    double target_x_;
    double target_y_;
    double target_yaw_;

    double tolerance_dist_;
    double tolerance_yaw_;
};

} // namespace tdk_localization