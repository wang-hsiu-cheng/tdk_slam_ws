#include "localization_manager.hpp"
#include <cmath>

namespace tdk_localization {

LocalizationManager::LocalizationManager(const rclcpp::NodeOptions & options)
: Node("localization_manager", options) {
    
    // 讀取 world -> map 的偏移量參數
    this->declare_parameter("world_to_map_x", 0.0);
    this->declare_parameter("world_to_map_y", 0.0);
    world_to_map_x_ = this->get_parameter("world_to_map_x").as_double();
    world_to_map_y_ = this->get_parameter("world_to_map_y").as_double();

    // 讀取參數，決定目前跟哪套 SLAM 對接
    this->declare_parameter("slam_type", "slam_toolbox"); // 或 "cartographer"
    slam_type_ = this->get_parameter("slam_type").as_string();

    // 讀取容差範圍參數
    this->declare_parameter("tolerance_dist", 0.05);
    this->declare_parameter("tolerance_yaw", 0.05);
    tolerance_dist_ = this->get_parameter("tolerance_dist").as_double();
    tolerance_yaw_ = this->get_parameter("tolerance_yaw").as_double();

    // ---- 初始化通訊接口 ----
    // 1. 接收主程式的抽象初始訊號 (此處採用 Topic 訂閱)
    init_cmd_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/initial_pose_cmd", 10, std::bind(&LocalizationManager::onInitCmdReceived, this, std::placeholders::_1));

    // 2. 接收來自你寫的專用 Node 的全域座標
    robot_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/robot_pose", 10, std::bind(&LocalizationManager::onRobotPoseReceived, this, std::placeholders::_1));

    // 3. 回傳給主程式的狀態發布器
    status_pub_ = this->create_publisher<std_msgs::msg::Bool>("/initialization_status", 10);

    // 4. 根據 SLAM 類型初始化對應接口
    if (slam_type_ == "slam_toolbox") {
        slam_toolbox_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
    } else if (slam_type_ == "cartographer") {
        carto_finish_cli_ = this->create_client<cartographer_ros_msgs::srv::FinishTrajectory>("/finish_trajectory");
        carto_start_cli_ = this->create_client<cartographer_ros_msgs::srv::StartTrajectory>("/start_trajectory");
    }

    // 5. Nav2 地圖清除服務客戶端
    global_costmap_cli_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>("/global_costmap/clear_entirely_global_costmap");
    local_costmap_cli_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>("/local_costmap/clear_entirely_local_costmap");

    RCLCPP_INFO(this->get_logger(), "Localization Manager 已啟動，目前對接模組: %s", slam_type_.c_str());
}

void LocalizationManager::onInitCmdReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // 當主程式持續發送抽象訊號時：
    // 如果目前已經在運作中，或已經成功，就忽略重複的觸發，直到主程式那邊關閉訊號
    if (current_state_ != InitState::IDLE) {
        if (current_state_ == InitState::SUCCESS) {
            std::msgs::msg::Bool status_msg;
            status_msg.data = true;
            status_pub_->publish(status_msg); // 持續回報成功，直到主程式閉環停止
        }
        return;
    }

    // 記錄主程式期望的目標初始化位置
    target_x_ = msg->pose.position.x;
    target_y_ = msg->pose.position.y;
    target_yaw_ = getYawFromQuaternion(msg->pose.orientation);

    RCLCPP_INFO(this->get_logger(), "收到主程式初始命令！目標位置: [%.2f, %.2f, %.2f]", target_x_, target_y_, target_yaw_);
    
    current_state_ = InitState::TRIGGER_SLAM;
    triggerSlamReset();
}

void LocalizationManager::triggerSlamReset() {
    // 💥 關鍵核心：計算從 world 轉到 map 坐標系下的具體座標
    double map_x = target_x_ - world_to_map_x_;
    double map_y = target_y_ - world_to_map_y_;

    if (slam_type_ == "slam_toolbox") {
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = this->get_clock()->now();
        pose_msg.header.frame_id = "map"; // 指定發送給 SLAM 的 Frame 是 map
        
        // 帶入轉換後的 map 座標
        pose_msg.pose.pose.position.x = map_x;
        pose_msg.pose.pose.position.y = map_y;
        pose_msg.pose.pose.orientation.z = sin(target_yaw_ / 2.0);
        pose_msg.pose.pose.orientation.w = cos(target_yaw_ / 2.0);
        
        slam_toolbox_pub_->publish(pose_msg);
        RCLCPP_INFO(this->get_logger(), "已將 World[%.2f, %.2f] 轉換為 Map[%.2f, %.2f] 發送至 slam_toolbox", target_x_, target_y_, map_x, map_y);
        current_state_ = InitState::VERIFYING;

    } else if (slam_type_ == "cartographer") {
        // Cartographer 流程：先清除軌跡 0，再用新座標啟動軌跡 1
        auto finish_req = std::make_shared<cartographer_ros_msgs::srv::FinishTrajectory::Request>();
        finish_req->trajectory_id = 0; 

        carto_finish_cli_->async_send_request(finish_req, [this, map_x, map_y](rclcpp::Client<cartographer_ros_msgs::srv::FinishTrajectory>::SharedFuture future) {
            try {
                auto res = future.get();
                auto start_req = std::make_shared<cartographer_ros_msgs::srv::StartTrajectory::Request>();
                start_req->configuration_directory = "/home/tdk/tdk_slam_ws/src/tdk_slam_manager/cartographer_config";
                start_req->configuration_basename = "localization.lua";
                start_req->use_initial_pose = true;
                
                // 帶入轉換後的 map 座標
                start_req->initial_pose.position.x = map_x;
                start_req->initial_pose.position.y = map_y;
                start_req->initial_pose.orientation.z = sin(target_yaw_ / 2.0);
                start_req->initial_pose.orientation.w = cos(target_yaw_ / 2.0);

                carto_start_cli_->async_send_request(start_req, [this, map_x, map_y](rclcpp::Client<cartographer_ros_msgs::srv::StartTrajectory>::SharedFuture start_future) {
                    RCLCPP_INFO(this->get_logger(), "已將 World 轉換為 Map[%.2f, %.2f] 重啟 Cartographer 軌跡", map_x, map_y);
                    this->current_state_ = InitState::VERIFYING;
                });
            } catch (const std::exception & e) {
                RCLCPP_ERROR(this->get_logger(), "Cartographer 重置失敗: %s", e.what());
                this->current_state_ = InitState::IDLE;
            }
        });
    }
}

void LocalizationManager::onRobotPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // 只有在驗證階段才去比對座標
    if (current_state_ != InitState::VERIFYING) return;

    double current_x = msg->pose.position.x;
    double current_y = msg->pose.position.y;
    double current_yaw = getYawFromQuaternion(msg->pose.orientation);

    verifyPose(current_x, current_y, current_yaw);
}

void LocalizationManager::verifyPose(double current_x, double current_y, double current_yaw) {
    // 計算距離
    double dist = std::sqrt(std::pow(current_x - target_x_, 2) + std::pow(current_y - target_y_, 2));
    // 計算角度差值 (經由 atan2 限制在 -PI ~ PI 之間)
    double yaw_diff = std::atan2(std::sin(current_yaw - target_yaw_), std::cos(current_yaw - target_yaw_));
    yaw_diff = std::abs(yaw_diff);

    if (dist < tolerance_dist_ && yaw_diff < tolerance_yaw_) {
        RCLCPP_INFO(this->get_logger(), "驗證通過！定位成功吸附目標點。距離誤差: %.3fm, 角度誤差: %.3f rad", dist, yaw_diff);
        current_state_ = InitState::CLEARING_COSTMAP;
        clearNav2Costmaps();
    }
    // 若不符合，則繼續維持 VERIFYING 狀態，等待更新的 /robot_pose 話題
}

void LocalizationManager::clearNav2Costmaps() {
    auto req = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
    
    // 同時清除全域與局部地圖的鬼影障礙物
    auto global_future = global_costmap_cli_->async_send_request(req);
    auto local_future = local_costmap_cli_->async_send_request(req);

    // 透過非同步監聽確認兩者皆完成
    RCLCPP_INFO(this->get_logger(), "正在清洗 Nav2 Costmap 殘留快取...");
    
    current_state_ = InitState::SUCCESS;

    // 回報成功給主程式
    std_msgs::msg::Bool status_msg;
    status_msg.data = true;
    status_pub_->publish(status_msg);
    RCLCPP_INFO(this->get_logger(), "======== 全流程初始化成功完成 ========");
}

double LocalizationManager::getYawFromQuaternion(const geometry_msgs::msg::Quaternion & q) {
    // 四元數轉 2D Yaw 角度
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

} // namespace tdk_localization

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<tdk_localization::LocalizationManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}