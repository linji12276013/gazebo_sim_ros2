#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "std_msgs/msg/string.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("exchanger_move_group_control");
static const std::string PLANNING_GROUP = "exchanger";

geometry_msgs::msg::Quaternion euler_to_quaternion(double roll, double pitch, double yaw)
{
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);

  geometry_msgs::msg::Quaternion quaternion;
  quaternion.x = q.x();
  quaternion.y = q.y();
  quaternion.z = q.z();
  quaternion.w = q.w();

  return quaternion;
};

// class ExchangerCtrl : public rclcpp::Node
// {
// public:
//     ExchangerCtrl(std::string name) : Node(name)
//     {
//         robot_description = this->create_subscription<std_msgs::msg::String>\
//         ("command", 10, std::bind(&ExchangerCtrl::command_callback, this, std::placeholders::_1));
//     }

// private:
//     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description;

//     void command_callback(const std_msgs::msg::String::SharedPtr msg)
//     {

//     }

// };


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__ns:=/exchanger"});
    auto node = std::make_shared<rclcpp::Node>("exchanger_move_group_control");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();
    
    // 规划组
    auto robot_description = node->get_parameter("/exchanger/robot_description").as_string();

    moveit::planning_interface::MoveGroupInterface::Options exchanger_options(
        PLANNING_GROUP, robot_description, "/exchanger");
    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, exchanger_options);
    // 获取基本信息
    RCLCPP_INFO(node->get_logger(), "Planning frame: %s", move_group->getPlanningFrame().c_str()); // 机器人参考系名称
    RCLCPP_INFO(node->get_logger(), "End effector link: %s", move_group->getEndEffectorLink().c_str()); // 末端执行器链接名称
    RCLCPP_INFO(node->get_logger(), "Available planning groups:");
    const std::vector<std::string>& groups = move_group->getJointModelGroupNames();
    for (const auto& group : groups)
    {
        RCLCPP_INFO(node->get_logger(), "  %s", group.c_str());
    }

    // 规划到一个姿态目标
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation = euler_to_quaternion(M_PI_2, 0.0, M_PI_2);
    target_pose.position.x = -0.1;
    target_pose.position.y = 0;
    target_pose.position.z = 0.6;
    move_group->setPoseTarget(target_pose);

    // 调用规划器计算路径并可视化
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(node->get_logger(), "Plan %s", success ? "" : "FAILED");

    // 移动到姿态目标
    if (success)
    {
        move_group->execute(my_plan);
        RCLCPP_INFO(node->get_logger(), "Executing...");
    }

    rclcpp::shutdown();
    return 0;

}
