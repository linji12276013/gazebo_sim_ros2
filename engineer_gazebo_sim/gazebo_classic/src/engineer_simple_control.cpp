#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <string>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("engineer_move_group_control");
static const std::string PLANNING_GROUP = "boom";

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

void quaternion_to_euler(const geometry_msgs::msg::Quaternion& quaternion, 
                        double& roll, double& pitch, double& yaw)
{
    tf2::Quaternion q(
        quaternion.x,
        quaternion.y, 
        quaternion.z,
        quaternion.w);

    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__ns:=/engineer"});
    auto node = std::make_shared<rclcpp::Node>("engineer_move_group_control", options);
    node->set_parameter(rclcpp::Parameter("use_sim_time", true));
    node->declare_parameter("target_pose.position", std::vector<double>{0.5, 0.0, 0.5});
    node->declare_parameter("target_pose.orientation", std::vector<double>{0, M_PI, M_PI_4});

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();
    
    // 规划组
    moveit::planning_interface::MoveGroupInterface::Options exchanger_options(
        PLANNING_GROUP, "robot_description", node->get_namespace());
    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, exchanger_options);
    // 获取基本信息
    RCLCPP_INFO(node->get_logger(), "Planning frame: %s", move_group->getPlanningFrame().c_str());
    RCLCPP_INFO(node->get_logger(), "End effector link: %s", move_group->getEndEffectorLink().c_str());
    RCLCPP_INFO(node->get_logger(), "Available planning groups:");
    const std::vector<std::string>& groups = move_group->getJointModelGroupNames();
    for (const auto& group : groups)
    {
        RCLCPP_INFO(node->get_logger(), "  %s", group.c_str());
    }

    // 规划到一个随机的姿态目标
    auto cp = move_group->getCurrentPose();
    double roll, pitch, yaw;
    quaternion_to_euler(cp.pose.orientation, roll, pitch, yaw);
    RCLCPP_INFO(node->get_logger(), "Current pose: %f, %f, %f", cp.pose.position.x, cp.pose.position.y, cp.pose.position.z);
    RCLCPP_INFO(node->get_logger(), "Current orientation: %f, %f, %f", roll, pitch, yaw);
    geometry_msgs::msg::Pose target_pose;
    std::vector<double> position_param, orientation_param;
    
    node->get_parameter("target_pose.position", position_param);
    node->get_parameter("target_pose.orientation", orientation_param);

    target_pose.position.x = position_param[0];
    target_pose.position.y = position_param[1];
    target_pose.position.z = position_param[2];
    target_pose.orientation = euler_to_quaternion(orientation_param[0], orientation_param[1], orientation_param[2]);
    move_group->setPoseTarget(target_pose);

    // 调用规划器计算路径并可视化
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(node->get_logger(), "Plan %s", success ? "" : "FAILED");

    // 移动到姿态目标
    if (success)
    {
        move_group->execute(my_plan);
    }

    rclcpp::shutdown();
    return 0;
}
