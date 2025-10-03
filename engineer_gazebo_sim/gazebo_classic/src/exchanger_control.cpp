#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <random>

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

geometry_msgs::msg::Pose generate_random_pose(
    double min_x = 0.8, double max_x = 1.0,
    double min_y = 0.0, double max_y = 0.0, 
    double min_z = 0.5, double max_z = 0.6,
    double min_roll = -M_PI_2, double max_roll = M_PI_2,
    double min_pitch = M_PI_4 / 2, double max_pitch = M_PI_2,
    double min_yaw = -M_PI_4, double max_yaw = M_PI_4)
{
    geometry_msgs::msg::Pose random_pose;

    // 生成随机位置
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dist_x(min_x, max_x);
    std::uniform_real_distribution<> dist_y(min_y, max_y);
    std::uniform_real_distribution<> dist_z(min_z, max_z);
    // 生成随机方向（随机欧拉角）
    std::uniform_real_distribution<> dist_roll(min_roll, max_roll);
    std::uniform_real_distribution<> dist_pitch(min_pitch, max_pitch);
    std::uniform_real_distribution<> dist_yaw(min_yaw, max_yaw);

    random_pose.position.x = dist_x(gen);
    random_pose.position.y = dist_y(gen);
    random_pose.position.z = dist_z(gen);

    random_pose.orientation = euler_to_quaternion(
        dist_roll(gen), 
        dist_pitch(gen), 
        dist_yaw(gen)
    );
    
    return random_pose;
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__ns:=/exchanger"});
    auto node = std::make_shared<rclcpp::Node>("exchanger_move_group_control", options);
    node->set_parameter(rclcpp::Parameter("use_sim_time", true));

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
    target_pose = generate_random_pose();
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
