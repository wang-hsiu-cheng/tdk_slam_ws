#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

enum class MainState {
    IDLE,
    SEND_INIT_REQUEST,
    WAITING_FOR_MANAGER,
    SYSTEM_READY,
    ALARM_FAILURE
};

class PseudoMain : public rclcpp::Node {
public:
    PseudoMain() : Node("pseudo_main"), current_state_(MainState::IDLE) {

        this->declare_parameter("init_x", 0.0);
        this->declare_parameter("init_y", 0.0);
        this->declare_parameter("init_yaw", 0.0);
        this->init_x_ = this->get_parameter("init_x").as_double();
        this->init_y_ = this->get_parameter("init_y").as_double();
        this->init_yaw_ = this->get_parameter("init_yaw").as_double();

        init_cmd_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/init_pose_cmd", 10);
        status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/init_pose_status", 10, std::bind(&PseudoMain::statusCallback, this, std::placeholders::_1));
        loop_timer_ = this->create_wall_timer(200ms, std::bind(&PseudoMain::controlLoop, this));
        current_state_ = MainState::SEND_INIT_REQUEST;
    }

private:
    void controlLoop() {
        switch (current_state_) {
            case MainState::IDLE:
                break;

            case MainState::SEND_INIT_REQUEST:
                RCLCPP_INFO(this->get_logger(), "send init request (retry count: %d)", retry_count_);
                publishInitCmd();
                current_state_ = MainState::WAITING_FOR_MANAGER;
                break;

            case MainState::WAITING_FOR_MANAGER:
                // do nothing, just wait
                break;

            case MainState::SYSTEM_READY:
                RCLCPP_INFO_ONCE(this->get_logger(), "init successfully. start main program");
                break;

            case MainState::ALARM_FAILURE:
                RCLCPP_ERROR_ONCE(this->get_logger(), "init fail and exceed max retry times");
                break;
        }
    }
    void statusCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        // 只有在等待 localization_manager 回應時才處理
        if (current_state_ != MainState::WAITING_FOR_MANAGER) {
            return;
        }
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "receive success");
            current_state_ = MainState::SYSTEM_READY;
        } 
        else {
            RCLCPP_WARN(this->get_logger(), "receive fail");
            if (retry_count_ < max_retries_) {
                retry_count_++;
                RCLCPP_WARN(this->get_logger(), "resend init request");
                current_state_ = MainState::SEND_INIT_REQUEST;
            } 
            else {
                // 超過 retry times
                current_state_ = MainState::ALARM_FAILURE;
            }
        }
    }

    void publishInitCmd() {
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, this->init_yaw_); 

        auto msg = geometry_msgs::msg::PoseStamped();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "world";
        msg.pose.position.x = this->init_x_; 
        msg.pose.position.y = this->init_y_;
        msg.pose.orientation = tf2::toMsg(q);
        init_cmd_pub_->publish(msg);
    }
    MainState current_state_;
    int retry_count_ = 0;
    const int max_retries_ = 5;

    double init_x_;
    double init_y_;
    double init_yaw_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr init_cmd_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr status_sub_;
    rclcpp::TimerBase::SharedPtr loop_timer_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PseudoMain>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}