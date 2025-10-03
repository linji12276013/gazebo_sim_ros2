#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>
#include <unistd.h>
#include <iostream>

static constexpr char KEYCODE_RIGHT = 0x43;
static constexpr char KEYCODE_LEFT = 0x44;
static constexpr char KEYCODE_UP = 0x41;
static constexpr char KEYCODE_DOWN = 0x42;
static constexpr char KEYCODE_SPACE = 0x20;
static constexpr char KEYCODE_Q = 0x71;
static constexpr char KEYCODE_E = 0x65;

class KeyboardControlNode : public rclcpp::Node
{
public:
    KeyboardControlNode() : Node("keyboard_control")
    {
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/engineer/cmd_vel", 10);
        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/engineer/cmd_twist", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/engineer/cmd_pose", 10);

        tcgetattr(STDIN_FILENO, &oldt);
        struct termios newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&KeyboardControlNode::timer_callback, this));
    }

    ~KeyboardControlNode()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }

private:
    void timer_callback()
    {
        char c;
        if (read(STDIN_FILENO, &c, 1) > 0)
        {  
            switch (c)
            {
                case KEYCODE_UP:
                    twist.linear.x += 0.05;
                    break;
                case KEYCODE_DOWN:
                    twist.linear.x -= 0.05;
                    break;
                case KEYCODE_LEFT:
                    twist.linear.y += 0.05;
                    break;
                case KEYCODE_RIGHT:
                    twist.linear.y -= 0.05;
                    break;
                case KEYCODE_Q:
                    twist.angular.z += 0.5;
                    break;
                case KEYCODE_E:
                    twist.angular.z -= 0.5;
                    break;
                case KEYCODE_SPACE:
                    twist.linear.x = 0.0;
                    twist.linear.y = 0.0;
                    twist.angular.z = 0.0;
                    break;
                default:
                    return;
            }

            vel_pub_->publish(twist);
            std::cout << "发布命令: linear.x=" << twist.linear.x << ", linear.y=" << twist.linear.y
                      << ", angular.z=" << twist.angular.z << std::endl;
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    struct termios oldt;
    geometry_msgs::msg::Twist twist;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
