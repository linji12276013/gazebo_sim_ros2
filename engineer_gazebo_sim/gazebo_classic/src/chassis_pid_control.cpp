#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_ros/buffer.h>
#include "../include/pid_controller.hpp"

void quaternion_to_euler(const geometry_msgs::msg::Quaternion& quaternion, double& yaw)
{
    double roll, pitch;
    tf2::Quaternion q(
        quaternion.x,
        quaternion.y, 
        quaternion.z,
        quaternion.w);

    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
}

class ChassisPidControl : public rclcpp::Node
{
public:
    ChassisPidControl(std::string name) : Node(name)
    {
        target_pose_sub_ = this->create_subscription<geometry_msgs::msg::Twist>\
        ("/engineer/cmd_pose", 100, 
            std::bind(&ChassisPidControl::target_pose_callback, this, std::placeholders::_1));
        current_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>\
        ("/engineer/odom", 10, 
            std::bind(&ChassisPidControl::current_pose_callback, this, std::placeholders::_1));
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>\
        ("/engineer/cmd_vel", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                         std::bind(&ChassisPidControl::timer_callback, 
                                         this));
    }

private:
    void target_pose_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (msg)
        {
            linear_x.setpoint += msg->linear.x * 0.01;
            linear_y.setpoint += msg->linear.y * 0.01;
            angular_z.setpoint += msg->angular.z * 0.01;
        }
    }

    void current_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (msg)
        {   
            quaternion_to_euler(msg->pose.pose.orientation, angular_z.measured_value);
            linear_x.measured_value = msg->pose.pose.position.x * cos(angular_z.measured_value) - msg->pose.pose.position.y * sin(angular_z.measured_value);
            linear_y.measured_value = msg->pose.pose.position.x * sin(angular_z.measured_value) + msg->pose.pose.position.y * cos(angular_z.measured_value);
        }
    }

    void timer_callback()
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = linear_x.compute(dt);
        cmd_vel.linear.y = linear_y.compute(dt);
        cmd_vel.angular.z = angular_z.compute(dt);
        cmd_vel_pub_->publish(cmd_vel);
    }

    double dt = 0.01;
    PidController linear_x{2.0, 0, 0.05}, linear_y{2.0, 0, 0.05}, angular_z{10.0, 0, 0.05};

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr target_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr current_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ChassisPidControl>("chassis_pid_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
