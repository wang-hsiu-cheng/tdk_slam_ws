#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class RobotPosePub : public rclcpp::Node {
public:
    RobotPosePub() : Node("robot_pose_pub"), buffer_(this->get_clock()), listener_(buffer_) {
        this->declare_parameter<std::string>("parent_frame", "map");
        this->declare_parameter<std::string>("child_frame", "base_footprint");
        this->get_parameter("parent_frame", parent_frame_);
        this->get_parameter("child_frame", child_frame_);
        pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("robot_pose", rclcpp::QoS(10).reliable().durability_volatile());
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&RobotPosePub::lookupTransform, this)
        );
    }

private:
    void lookupTransform() {
        try {
            auto transformStamped = buffer_.lookupTransform(
                parent_frame_, 
                child_frame_,
                rclcpp::Time()
            );
            // RCLCPP_INFO(this->get_logger(), "Transform: x=%f, y=%f, z=%f", 
            //             transformStamped.transform.translation.x, 
            //             transformStamped.transform.translation.y, 
            //             transformStamped.transform.translation.z);
            robot_pose_.pose.position.x = transformStamped.transform.translation.x;
            robot_pose_.pose.position.y = transformStamped.transform.translation.y;
            robot_pose_.pose.position.z = 0;
            robot_pose_.pose.orientation = transformStamped.transform.rotation;
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Failed to lookup transform: %s", ex.what());
        }
        pub_->publish(robot_pose_);
    }
    std::string parent_frame_;
    std::string child_frame_;
    geometry_msgs::msg::PoseStamped robot_pose_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotPosePub>());
    rclcpp::shutdown();
    return 0;
}