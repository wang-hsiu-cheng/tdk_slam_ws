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
    IDLE,               // 靜態閒置
    TRIGGER_SLAM,       // 已向 SLAM 發出重置訊號
    VERIFYING,          // 正在比對 /robot_pose 是否達標
    CLEARING_COSTMAP,   // 正在清空 Nav2 地圖
    SUCCESS             // 初始化成功
};

class LocalizationManager : public rclcpp::Node {
public:
    explicit LocalizationManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    virtual ~LocalizationManager() = default;

private:
    // ==== 1. 回呼函式 (Callbacks) ====
    void onInitCmdReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void onRobotPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    
    // ==== 2. 核心邏輯功能 ====
    void triggerSlamReset();
    void verifyPose(double current_x, double current_y, double current_yaw);
    void clearNav2Costmaps();
    
    // ==== 3. 工具函式 ====
    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion & q);

    // ==== 4. ROS 2 通訊接口 ====
    // 訂閱與發布
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped::SharedPtr>::SharedPtr init_cmd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped::SharedPtr>::SharedPtr robot_pose_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr slam_toolbox_pub_;

    // 服務客戶端 (Clients)
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr global_costmap_cli_;
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr local_costmap_cli_;
    rclcpp::Client<cartographer_ros_msgs::srv::FinishTrajectory>::SharedPtr carto_finish_cli_;
    rclcpp::Client<cartographer_ros_msgs::srv::StartTrajectory>::SharedPtr carto_start_cli_;

    // ==== 5. 內部狀態變數 ====
    std::string slam_type_;
    InitState current_state_{InitState::IDLE};
    
    double world_to_map_x_{0.0};
    double world_to_map_y_{0.0};

    // 目標起點座標
    double target_x_{0.0};
    double target_y_{0.0};
    double target_yaw_{0.0};

    // 幾何容差參數
    double tolerance_dist_{0.05}; // 5 公分
    double tolerance_yaw_{0.05};  // 約 3 度 (弧度)
};

} // namespace tdk_localization