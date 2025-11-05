/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*      Title     : joystick_servo_example.cpp
 *      Project   : moveit_servo
 *      Created   : 08/07/2020
 *      Author    : Adam Pettinger
 */

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include "boeing_gazebo_model_attachment_plugin_msgs/srv/attach.hpp"
#include "boeing_gazebo_model_attachment_plugin_msgs/srv/detach.hpp"
#include <rclcpp/client.hpp>
#include <rclcpp/experimental/buffers/intra_process_buffer.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/qos_event.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <thread>

// We'll just set up parameters here
const std::string JOY_TOPIC = "/joy";
const std::string TWIST_TOPIC = "/engineer/engineer_servo_node/delta_twist_cmds";
const std::string JOINT_TOPIC = "/engineer/engineer_servo_node/delta_joint_cmds";
const std::string EEF_FRAME_ID = "link7";
const std::string BASE_FRAME_ID = "base_link";

// Enums for button names -> axis/button array index
// For XBOX 1 controller
enum Axis
{
  LEFT_STICK_X = 0,
  LEFT_STICK_Y = 1,
  LEFT_TRIGGER = 2,
  RIGHT_STICK_X = 3,
  RIGHT_STICK_Y = 4,
  RIGHT_TRIGGER = 5,
  D_PAD_X = 6,
  D_PAD_Y = 7
};
enum Button
{
  A = 0,
  B = 1,
  X = 2,
  Y = 3,
  LEFT_BUMPER = 4,
  RIGHT_BUMPER = 5,
  CHANGE_VIEW = 6,
  MENU = 7,
  HOME = 8,
  LEFT_STICK_CLICK = 9,
  RIGHT_STICK_CLICK = 10
};
enum AttachCmd
{
  halt = 0,
  engineer_off = 1,
  engineer_on = 2,
  exchanger_off = 3,
  exchanger_on = 4
};

// Some axes have offsets (e.g. the default trigger position is 1.0 not 0)
// This will map the default values for the axes
std::map<Axis, double> AXIS_DEFAULTS = { { LEFT_TRIGGER, 1.0 }, { RIGHT_TRIGGER, 1.0 } };
std::map<Button, double> BUTTON_DEFAULTS;

// To change controls or setup a new controller, all you should to do is change the above enums and the follow 2
// functions
/** \brief // This converts a joystick axes and buttons array to a TwistStamped or JointJog message
 * @param axes The vector of continuous controller joystick axes
 * @param buttons The vector of discrete controller button values
 * @param twist A TwistStamped message to update in prep for publishing
 * @param joint A JointJog message to update in prep for publishing
 * @return return true if you want to publish a Twist, false if you want to publish a JointJog
 */
bool convertJoyToCmd(const std::vector<float>& axes, const std::vector<int>& buttons,
                     std::unique_ptr<geometry_msgs::msg::TwistStamped>& twist,
                     int &attachcmd, int &ctrlmode)
{
  static int last_button;
  if (buttons[HOME] && buttons[HOME] != last_button)
  {
    ctrlmode ++;
    ctrlmode %= 2;
  }
  last_button = buttons[HOME];

  // Attach and Detach commands
  if (buttons[A] || buttons[B] || buttons[X] || buttons[Y])
  {
    if (buttons[A] && !buttons[B] && !buttons[X] && !buttons[Y])
    {
      attachcmd = engineer_on;
    }
    else if (!buttons[A] && buttons[B] && !buttons[X] && !buttons[Y])
    {
      attachcmd = engineer_off;
    }
    else if (!buttons[A] && !buttons[B] && buttons[X] && !buttons[Y])
    {
      attachcmd = exchanger_on;
    }
    else if (!buttons[A] && !buttons[B] && !buttons[X] && buttons[Y])
    {
      attachcmd = exchanger_off;
    }
    return false;
  }

  // The bread and butter: map buttons to twist commands
  if (!ctrlmode)
  {
    twist->twist.linear.z = axes[RIGHT_STICK_Y];
    twist->twist.linear.y = axes[RIGHT_STICK_X];

    double lin_x_right = -0.5 * (axes[RIGHT_TRIGGER] - AXIS_DEFAULTS.at(RIGHT_TRIGGER));
    double lin_x_left = 0.5 * (axes[LEFT_TRIGGER] - AXIS_DEFAULTS.at(LEFT_TRIGGER));
    twist->twist.linear.x = lin_x_right + lin_x_left;

    twist->twist.angular.y = axes[LEFT_STICK_Y];
    twist->twist.angular.x = axes[LEFT_STICK_X];

    double roll_positive = buttons[RIGHT_BUMPER];
    double roll_negative = -1 * (buttons[LEFT_BUMPER]);
    twist->twist.angular.z = roll_positive + roll_negative;
  }
  else
  {
    twist->twist.linear.x = axes[RIGHT_STICK_Y];
    twist->twist.linear.y = axes[RIGHT_STICK_X];

    double ang_z_right = 0.5 * (axes[RIGHT_TRIGGER] - AXIS_DEFAULTS.at(RIGHT_TRIGGER));
    double ang_z_left = -0.5 * (axes[LEFT_TRIGGER] - AXIS_DEFAULTS.at(LEFT_TRIGGER));
    twist->twist.angular.z = ang_z_right + ang_z_left;
  }
  return true;
}

/** \brief // This should update the frame_to_publish_ as needed for changing command frame via controller
 * @param frame_name Set the command frame to this
 * @param buttons The vector of discrete controller button values
 */
void updateCmdFrame(std::string& frame_name, const std::vector<int>& buttons)
{
  if (buttons[CHANGE_VIEW] && frame_name == EEF_FRAME_ID)
    frame_name = BASE_FRAME_ID;
  else if (buttons[MENU] && frame_name == BASE_FRAME_ID)
    frame_name = EEF_FRAME_ID;
}

namespace engineer
{
class JoyToServoPub : public rclcpp::Node
{
public:
  JoyToServoPub(const rclcpp::NodeOptions& options)
    : Node("joy_to_twist_publisher", options), frame_to_publish_(BASE_FRAME_ID)
  {
    // Setup pub/sub
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        JOY_TOPIC, rclcpp::SystemDefaultsQoS(),
        [this](const sensor_msgs::msg::Joy::ConstSharedPtr& msg) { return joyCB(msg); });

    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, rclcpp::SystemDefaultsQoS());
    chassis_twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/engineer/cmd_pose", rclcpp::SystemDefaultsQoS());
    joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, rclcpp::SystemDefaultsQoS());

    // Create a service client to start the ServoNode
    servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
    servo_start_client_->wait_for_service(std::chrono::seconds(1));
    servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

    attach_client_ = this->create_client<boeing_gazebo_model_attachment_plugin_msgs::srv::Attach>("/gazebo/attach");
    detach_client_ = this->create_client<boeing_gazebo_model_attachment_plugin_msgs::srv::Detach>("/gazebo/detach");

    while (!attach_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(this->get_logger(), "等待attach服务端上线中");
    }
    while (!detach_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(this->get_logger(), "等待detach服务端上线中");
    }
  }

  void joyCB(const sensor_msgs::msg::Joy::ConstSharedPtr& msg)
  {
    // Create the messages we might publish
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();

    // This call updates the frame for twist commands
    updateCmdFrame(frame_to_publish_, msg->buttons);

    if (attachcmd != lastattachcmd)
      RCLCPP_INFO(this->get_logger(), "attachcmd = %d, last = %d", attachcmd, lastattachcmd);

    // Convert the joystick message to Twist or JointJog and publish
    if (convertJoyToCmd(msg->axes, msg->buttons, twist_msg, attachcmd, ctrlmode))
    {
      // publish the TwistStamped
      if (!ctrlmode)
      {

        twist_msg->header.frame_id = frame_to_publish_;
        twist_msg->header.stamp = this->now();
        twist_pub_->publish(std::move(twist_msg));
      }
      else
      {
        chassis_twist_pub_->publish(std::move(twist_msg->twist));
      }
    }
    else
    {
      if (attachcmd != lastattachcmd)
      {
        lastattachcmd = attachcmd;
        // Call Attach/Detach Service
        switch (attachcmd)
        {
          case engineer_on:
            attach("engineer", "link7_optical_link");
            break;
          case engineer_off:
            detach("engineer");
            break;
          case exchanger_on:
            attach("exchanger", "elink6");
            break;
          case exchanger_off:
            detach("exchanger");
            break;
          default:
            break;
        }
      }
    }
  }

  void attach(const std::string &model_name_1, const std::string &link_name_1)
  {
    auto req = std::make_shared<boeing_gazebo_model_attachment_plugin_msgs::srv::Attach::Request>();
    req->joint_name = "attach_joint";
    req->model_name_1 = model_name_1;
    req->link_name_1 = link_name_1;
    req->model_name_2 = "gold";
    req->link_name_2 = "gold_link";

    RCLCPP_INFO(this->get_logger(), "aaaaaaaaaa");
    auto future = attach_client_->async_send_request(req,
      [this, model_name_1](rclcpp::Client<boeing_gazebo_model_attachment_plugin_msgs::srv::Attach>::SharedFuture result)
    {
      try
      {
        RCLCPP_INFO(this->get_logger(), "附着成功: %s <-> gold", model_name_1.c_str());
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(this->get_logger(), "附着失败: %s", e.what());
      }
    });
  }

  void detach(const std::string &model_name_1)
  {
    auto req = std::make_shared<boeing_gazebo_model_attachment_plugin_msgs::srv::Detach::Request>();
    req->joint_name = "attach_joint";
    req->model_name_1 = model_name_1;
    req->model_name_2 = "gold";

    RCLCPP_INFO(this->get_logger(), "bbbbbbbb");
    auto future = detach_client_->async_send_request(req,
      [this, model_name_1](rclcpp::Client<boeing_gazebo_model_attachment_plugin_msgs::srv::Detach>::SharedFuture result)
    {
      try
      {
        RCLCPP_INFO(this->get_logger(), "分离成功: %s <-> gold", model_name_1.c_str());
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(this->get_logger(), "分离失败: %s", e.what());
      }
    });
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr chassis_twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;
  rclcpp::Client<boeing_gazebo_model_attachment_plugin_msgs::srv::Attach>::SharedPtr attach_client_;
  rclcpp::Client<boeing_gazebo_model_attachment_plugin_msgs::srv::Detach>::SharedPtr detach_client_;

  std::string frame_to_publish_;
  int attachcmd = halt, lastattachcmd = halt, ctrlmode = 0;
};  // class JoyToServoPub

}  // namespace engineer

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(engineer::JoyToServoPub)
