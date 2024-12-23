/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/JointState.h>


#define ARM_SLOW 0.1
#define ARM_FAST 0.3

class Jupiter2Teleop
{
public:
  Jupiter2Teleop();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr &joint_state);
  void publish();
  ros::NodeHandle ph_, nh_;

  int linear_, angular_, half_linear_, half_angular_, deadman_axis_, accelerate_button_, dir_left_right_, dir_up_down_, x_left_, b_right_, y_up_, a_down_, axis_down_l_, axis_down_r_, left_trigger_, right_trigger_, back_, start_;
  double l_scale_, a_scale_;

  ros::Publisher vel_pub_;
  ros::Publisher arm1_pub_;
  ros::Publisher arm2_pub_;
  ros::Publisher arm3_pub_;
  ros::Publisher arm4_pub_;
  ros::Publisher gripper_pub_;
  ros::Publisher head_pub_;
  ros::Publisher capture_pub_;

  ros::Subscriber joy_sub_;
  ros::Subscriber joint_state_sub_;
  ros::Publisher joint_state_pub_;

  geometry_msgs::Twist last_published_;

  std_msgs::Float64 arm_xb;
  std_msgs::Float64 arm_ya;
  double arm1, arm2, arm3, arm4;
  double gripper, head;

  std_msgs::String capture_hold;
  std_msgs::String capture_published_;
  
  boost::mutex publish_mutex_;
  bool deadman_pressed_, accelerate_pressed_;
  bool zero_twist_published_;
  ros::Timer timer_;

  double arm1_pos_, arm2_pos_, arm3_pos_, arm4_pos_;
  double gripper_pos_, head_pos_;
};

Jupiter2Teleop::Jupiter2Teleop() : 
                                    l_scale_(0.2),
                                    a_scale_(1.0),
                                    linear_(1),
                                    angular_(0),
                                    half_linear_(3),
                                    half_angular_(2),
                                    deadman_axis_(5),
                                    accelerate_button_(4),
                                    dir_left_right_(4),
                                    dir_up_down_(5),
                                    x_left_(0),
                                    b_right_(2),
                                    y_up_(3),
                                    a_down_(1),
                                    axis_down_l_(10),
                                    axis_down_r_(11),
                                    back_(8),
                                    start_(9),
                                    left_trigger_(6),
                                    right_trigger_(7)

{
  ph_.param("axis_linear", linear_, linear_);
  ph_.param("axis_angular", angular_, angular_);
  ph_.param("axis_half_linear", half_linear_, half_linear_);
  ph_.param("axis_half_angular", half_angular_, half_angular_);
  ph_.param("axis_deadman", deadman_axis_, deadman_axis_);
  ph_.param("button_accelerate", accelerate_button_, accelerate_button_);
  ph_.param("scale_linear", l_scale_, l_scale_);
  ph_.param("scale_angular", a_scale_, a_scale_);

  deadman_pressed_ = false;
  accelerate_pressed_ = false;
  zero_twist_published_ = false;

  vel_pub_ = ph_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  joint_state_pub_ = ph_.advertise<sensor_msgs::JointState>("/jupiter2/joint_states", 1, true);
  joint_state_sub_ = nh_.subscribe("jupiter2/joint_states", 10, &Jupiter2Teleop::jointStateCallback, this);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Jupiter2Teleop::joyCallback, this);
  

  timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&Jupiter2Teleop::publish, this));
}

void Jupiter2Teleop::jointStateCallback(const sensor_msgs::JointState::ConstPtr &joint_state)
{
  for (size_t i = 0; i < joint_state->name.size(); ++i)
  {
    if (joint_state->name[i] == "arm1_joint") arm1_pos_ = joint_state->position[i];
    else if (joint_state->name[i] == "arm2_joint") arm2_pos_ = joint_state->position[i];
    else if (joint_state->name[i] == "arm3_joint") arm3_pos_ = joint_state->position[i];
    else if (joint_state->name[i] == "arm4_joint") arm4_pos_ = joint_state->position[i];
    else if (joint_state->name[i] == "gripper_joint") gripper_pos_ = joint_state->position[i];
    else if (joint_state->name[i] == "head_joint") head_pos_ = joint_state->position[i];
  }
  ROS_INFO("Arm1: %.4f, Arm2: %.4f, Arm3: %.4f, Arm4: %.4f, Gripper: %.4f, Head: %.4f",
           arm1_pos_, arm2_pos_, arm3_pos_, arm4_pos_, gripper_pos_, head_pos_);
}

void Jupiter2Teleop::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
  geometry_msgs::Twist vel;
  deadman_pressed_ = joy->buttons[deadman_axis_];
  accelerate_pressed_ = joy->buttons[accelerate_button_];

  if (joy->axes[half_linear_] || joy->axes[half_angular_])
  {
    vel.linear.x = l_scale_*0.6*joy->axes[half_linear_];
    vel.angular.z = a_scale_*0.6*joy->axes[half_angular_];
  }
  else if (accelerate_pressed_)
  {
    vel.linear.x = l_scale_*1.5*joy->axes[linear_];
    vel.angular.z = a_scale_*2.0*joy->axes[angular_];
  }
  else if (joy->axes[linear_] || joy->axes[angular_])
  {
    vel.linear.x = l_scale_*joy->axes[linear_];
    vel.angular.z = a_scale_*joy->axes[angular_];
  }

  if (joy->buttons[x_left_] && accelerate_pressed_)
  {
    arm_xb.data = ARM_FAST;
  }
  else if (joy->buttons[b_right_] && accelerate_pressed_)
  {
    arm_xb.data = 0.0 - ARM_FAST;
  }
  else if (joy->buttons[x_left_] && !accelerate_pressed_)
  {
    arm_xb.data = ARM_SLOW;
  }
  else if (joy->buttons[b_right_] && !accelerate_pressed_)
  {
    arm_xb.data = 0.0 - ARM_SLOW;
  }
  else
  {
    arm_xb.data = 0.0;
  }

  if (accelerate_pressed_ && joy->buttons[a_down_])
  {
    arm_ya.data = ARM_FAST;
  }
  else if (accelerate_pressed_ && joy->buttons[y_up_])
  {
    arm_ya.data = 0.0 - ARM_FAST;
  }
  else if (!accelerate_pressed_ && joy->buttons[a_down_])
  {
    arm_ya.data = ARM_SLOW;
  }
  else if (!accelerate_pressed_ && joy->buttons[y_up_])
  {
    arm_ya.data = 0.0 - ARM_SLOW;
  }
  else
  {
    arm_ya.data = 0.0;
  }

  if (joy->axes[dir_up_down_] > 0)
  {
    gripper = gripper + arm_xb.data;
    if (gripper > 0.6)
    {
      gripper = 0.6;
    }
    if (gripper < -0.4)
    {
      gripper = -0.4;
    }
    head = head + arm_ya.data;
    if (head < -0.8)
    {
      head = -0.8;
    }
    if (head > 0.6)
    {
      head = 0.6;
    }
  }
  if (joy->axes[dir_up_down_] < 0)
  {
    arm1 = arm1 + arm_xb.data;
    if (arm1 < -2.6)
    {
      arm1 = -2.6;
    }
    if (arm1 > 2.6)
    {
      arm1 = 2.6;
    }
    arm3 = arm3 + arm_ya.data;
    if (arm3 < -2.5)
    {
      arm3 = -2.5;
    }
    if (arm3 > 2.6)
    {
      arm3 = 2.6;
    }
  }
  if (joy->axes[dir_left_right_] > 0)
  {
    arm2 = arm2 + arm_ya.data;
    if (arm2 < -2.1)
    {
      arm2 = -2.1;
    }
    if (arm2 > 2.2)
    {
      arm2 = 2.2;
    }
  }
  if (joy->axes[dir_left_right_] < 0)
  {
    arm4 = arm4 + arm_ya.data;
    if (arm4 < -1.8)
    {
      arm4 = -1.8;
    }
    if (arm4 > 1.8)
    {
      arm4 = 1.8;
    }
  }
  if (joy->buttons[back_])
  {
    head = -0.5;
  }
  if (joy->buttons[start_])
  {
    head = 0.5;
  }
  if (joy->buttons[axis_down_l_] && joy->buttons[axis_down_r_])
  {
    arm1 = 0.0;
    arm2 = 0.0;
    arm3 = 0.0;
    arm4 = 0.0;
    gripper = 0.0;
    head = 0.0;
  }
  if (joy->buttons[left_trigger_])
  {
    arm1 = 0.0;
    arm2 = -1.4;
    arm3 = 2.2;
    arm4 = 0.6;
  }
  if (joy->buttons[right_trigger_])
  {
    arm1 = 0.0;
    arm2 = 1.9;
    arm3 = 0.8;
    arm4 = -1.4;
  }
  last_published_ = vel;
}

void Jupiter2Teleop::publish()
{
  boost::mutex::scoped_lock lock(publish_mutex_);

  // 创建 JointState 消息
  sensor_msgs::JointState joint_state_msg;

  // 设置关节的名称 (确保顺序和你的机器人的关节顺序一致)
  joint_state_msg.name = {"arm1_joint", "arm2_joint", "arm3_joint", "arm4_joint", "gripper_joint", "head_joint"};

  // 设置关节的角度值（即 position）
  joint_state_msg.position = {arm1, arm2, arm3, arm4, gripper, head};

  // 你也可以发布速度和力矩，如果需要的话
  joint_state_msg.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // 假设你没有控制速度
  joint_state_msg.effort = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};    // 假设没有控制力矩

  // 发布关节状态
  joint_state_pub_.publish(joint_state_msg);

  // 控制机器人运动的部分
  if (deadman_pressed_)
  {
    vel_pub_.publish(last_published_);
  }
  else if (!deadman_pressed_ && !zero_twist_published_)
  {
    vel_pub_.publish(geometry_msgs::Twist());
    zero_twist_published_ = true;
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "jupiter2_teleop_joy");
  Jupiter2Teleop jupiter2_teleop;
  ros::spin();
  return 0;
}
