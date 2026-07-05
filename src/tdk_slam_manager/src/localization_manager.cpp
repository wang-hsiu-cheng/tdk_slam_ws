#include "tdk_slam_manager/localization_manager.hpp"
#include <cmath>

namespace tdk_localization {

LocalizationManager::LocalizationManager(const rclcpp::NodeOptions & options)
: Node("localization_manager", options) {
    
    // 讀取 world -> map 的 static tf，之後可以寫死在程式裡
    this->declare_parameter("world_to_map_x", 0.0);
    this->declare_parameter("world_to_map_y", 0.0);
    world_to_map_x_ = this->get_parameter("world_to_map_x").as_double();
    world_to_map_y_ = this->get_parameter("world_to_map_y").as_double();

    this->declare_parameter("slam_type", "slam_toolbox"); // 或 "cartographer"
    slam_type_ = this->get_parameter("slam_type").as_string();

    // setting tolerance for pose comparation with robot_pose topic
    this->declare_parameter("tolerance_dist", 0.05);
    this->declare_parameter("tolerance_yaw", 0.05);
    tolerance_dist_ = this->get_parameter("tolerance_dist").as_double();
    tolerance_yaw_ = this->get_parameter("tolerance_yaw").as_double();

    // 接收主程式的初始化訊號
    init_cmd_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/init_pose_cmd", 10, std::bind(&LocalizationManager::onInitCmdReceived, this, std::placeholders::_1));

    // 接收 robot_pose topics
    robot_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/robot_pose", 10, std::bind(&LocalizationManager::onRobotPoseReceived, this, std::placeholders::_1));

    // 回傳給主程式是否結束初始化
    status_pub_ = this->create_publisher<std_msgs::msg::Bool>("/init_pose_status", 10);

    // 根據 SLAM pkg 選擇初始化方式
    if (slam_type_ == "slam_toolbox") {
        slam_toolbox_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
    } else if (slam_type_ == "cartographer") {
        carto_finish_cli_ = this->create_client<cartographer_ros_msgs::srv::FinishTrajectory>("/finish_trajectory");
        carto_start_cli_ = this->create_client<cartographer_ros_msgs::srv::StartTrajectory>("/start_trajectory");
        this->current_active_trajectory_id_ = 1;
    }

    // client of Nav2 costmap clear service
    global_costmap_cli_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>("/global_costmap/clear_entirely_global_costmap");
    local_costmap_cli_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>("/local_costmap/clear_entirely_local_costmap");

    RCLCPP_INFO(this->get_logger(), "Localization Manager Start <localization mode>: %s", slam_type_.c_str());

    fsm_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        [this]() {
            // 只有在 SUCCESS 狀態才需要進行計時
            if (this->current_state_ == InitState::SUCCESS) {
                auto current_time = this->get_clock()->now();
                double elapsed_time = (current_time - this->success_start_time_).seconds();

                if (elapsed_time > 4.0) {
                    RCLCPP_INFO(this->get_logger(), "return to IDLE to get new cmd");
                    this->current_state_ = InitState::IDLE;
                }
            }
        }
    );
}

void LocalizationManager::onInitCmdReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // 當收到主程式發的指令
    // 如果目前已經成功，就忽略重複的觸發，直到超過成功狀態保持時間
    if (current_state_ == InitState::SUCCESS) {
        std_msgs::msg::Bool status_msg;
        status_msg.data = true;
        status_pub_->publish(status_msg); // 持續回報成功
        return;
    }

    // 記錄目標初始化位置
    this->target_x_ = msg->pose.position.x;
    this->target_y_ = msg->pose.position.y;
    this->target_yaw_ = getYawFromQuaternion(msg->pose.orientation);

    RCLCPP_INFO(this->get_logger(), "target from main: [%.2f, %.2f, %.2f]", this->target_x_, this->target_y_, this->target_yaw_);
    
    current_state_ = InitState::TRIGGER_SLAM;
    triggerSlamReset();
}

void LocalizationManager::triggerSlamReset() {
    // transform target pose from world frame to map frame
    double map_x = this->target_x_ - world_to_map_x_;
    double map_y = this->target_y_ - world_to_map_y_;

    if (slam_type_ == "slam_toolbox") {
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = this->get_clock()->now();
        pose_msg.header.frame_id = "map";
        
        pose_msg.pose.pose.position.x = map_x;
        pose_msg.pose.pose.position.y = map_y;
        pose_msg.pose.pose.orientation.z = sin(target_yaw_ / 2.0);
        pose_msg.pose.pose.orientation.w = cos(target_yaw_ / 2.0);
        
        slam_toolbox_pub_->publish(pose_msg);
        this->verification_start_time_ = this->get_clock()->now(); 
        RCLCPP_INFO(this->get_logger(), "transform world[%.2f, %.2f] to map[%.2f, %.2f] and send to slam_toolbox", this->target_x_, this->target_y_, map_x, map_y);
        current_state_ = InitState::VERIFYING;

    } else if (slam_type_ == "cartographer") {
        auto finish_req = std::make_shared<cartographer_ros_msgs::srv::FinishTrajectory::Request>();
        // 結束現在使用中的工作
        finish_req->trajectory_id = current_active_trajectory_id_;
        RCLCPP_INFO(this->get_logger(), "Sending request to finish active trajectory ID: %d", current_active_trajectory_id_);

        carto_finish_cli_->async_send_request(finish_req, [this, map_x, map_y](rclcpp::Client<cartographer_ros_msgs::srv::FinishTrajectory>::SharedFuture future) {
            // CLI command: ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 1}"
            try {
                auto res = future.get();
                if (res->status.code != 0) {  // 工作關閉失敗
                    RCLCPP_ERROR(this->get_logger(), "Finish trajectory %d fail: %s", current_active_trajectory_id_, res->status.message.c_str());
                    this->current_state_ = InitState::IDLE;
                    return;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(500));  // wait 500ms. let cartographer release data

                auto start_req = std::make_shared<cartographer_ros_msgs::srv::StartTrajectory::Request>();
                start_req->configuration_directory = "/home/tdk/tdk_slam_ws/src/tdk_slam_manager/cartographer_config";
                start_req->configuration_basename = "localization.lua";
                start_req->use_initial_pose = true;

                start_req->initial_pose.position.x = map_x;
                start_req->initial_pose.position.y = map_y;
                start_req->initial_pose.orientation.z = sin(this->target_yaw_ / 2.0);
                start_req->initial_pose.orientation.w = cos(this->target_yaw_ / 2.0);

                RCLCPP_INFO(this->get_logger(), "Requesting new trajectory with initial pose: [%.2f, %.2f]", map_x, map_y);

                carto_start_cli_->async_send_request(start_req, [this, map_x, map_y](rclcpp::Client<cartographer_ros_msgs::srv::StartTrajectory>::SharedFuture start_future) {
                    // CLI command: ros2 service call /start_trajectory cartographer_ros_msgs/srv/StartTrajectory "{configuration_directory: '/home/tdk/tdk_slam_ws/src/tdk_slam_manager/cartographer_config', configuration_basename: 'localization.lua', use_initial_pose: true, initial_pose: {position: {x: 1.0, y: 5.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
                    try {
                        auto start_res = start_future.get();
                        if (start_res->status.code == 0) { // 重啟工作成功
                            this->current_active_trajectory_id_ = start_res->trajectory_id; // 紀錄現在的工作代號，下次重啟會用到
                            RCLCPP_INFO(this->get_logger(), "pose [%.2f, %.2f] under map frame. reinit cartographer success. New Trajectory ID: %d", 
                                        map_x, map_y, this->current_active_trajectory_id_);
                            this->verification_start_time_ = this->get_clock()->now(); 
                            this->current_state_ = InitState::VERIFYING;
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "Start new trajectory logical fail: %s", start_res->status.message.c_str());
                            this->current_state_ = InitState::IDLE;
                        }
                    } catch (const std::exception & inner_e) {
                        RCLCPP_ERROR(this->get_logger(), "Cartographer start response exception: %s", inner_e.what());
                        this->current_state_ = InitState::IDLE;
                    }
                });
            } catch (const std::exception & e) {
                RCLCPP_ERROR(this->get_logger(), "reinit cartographer fail: %s", e.what());
                this->current_state_ = InitState::IDLE;
            }
        });
    }
}

void LocalizationManager::onRobotPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // 只有在 verify state 才會比對座標
    if (current_state_ != InitState::VERIFYING) return;

    double current_x = msg->pose.position.x;
    double current_y = msg->pose.position.y;
    double current_yaw = getYawFromQuaternion(msg->pose.orientation);

    verifyPose(current_x, current_y, current_yaw);
}

void LocalizationManager::verifyPose(double current_x, double current_y, double current_yaw) {
    double dist = std::sqrt(std::pow(current_x - this->target_x_, 2) + std::pow(current_y - this->target_y_, 2));
    double yaw_diff = std::atan2(std::sin(current_yaw - target_yaw_), std::cos(current_yaw - this->target_yaw_));
    yaw_diff = std::abs(yaw_diff);

    if (dist < tolerance_dist_ && yaw_diff < tolerance_yaw_) {
        RCLCPP_INFO(this->get_logger(), "pass, linear error is: %.3fm, angular error: %.3f rad", dist, yaw_diff);
        current_state_ = InitState::CLEARING_COSTMAP;
        clearNav2Costmaps();
        return;
    }
    // 若不符合，則繼續維持 VERIFYING 狀態，等待新的 /robot_pose topic
    RCLCPP_INFO(this->get_logger(), "fail, linear error is: %.3fm, angular error: %.3f rad", dist, yaw_diff);
    auto current_time = this->get_clock()->now();
    double elapsed_time = (current_time - verification_start_time_).seconds();

    if (elapsed_time > 5.0) {  // 太久就回到 IDLE
        RCLCPP_ERROR(this->get_logger(), "waitin over 5 second, init fail");
        std_msgs::msg::Bool status_msg;
        status_msg.data = false;
        status_pub_->publish(status_msg);
        current_state_ = InitState::IDLE;
    }
}

void LocalizationManager::clearNav2Costmaps() {
    auto req = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
    auto global_future = global_costmap_cli_->async_send_request(req);
    auto local_future = local_costmap_cli_->async_send_request(req);
    RCLCPP_INFO(this->get_logger(), "clearing Nav2 Costmap");
    current_state_ = InitState::SUCCESS;
    std_msgs::msg::Bool status_msg;
    status_msg.data = true;
    status_pub_->publish(status_msg);
    this->success_start_time_ = this->get_clock()->now();
}

double LocalizationManager::getYawFromQuaternion(const geometry_msgs::msg::Quaternion & q) {
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