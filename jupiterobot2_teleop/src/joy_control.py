#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
import moveit_commander
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from geometry_msgs.msg import Twist
import time

class ArmController:
    def __init__(self):
        rospy.init_node('joy_controller')

        # initialize moveit commander
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.arm_group = MoveGroupCommander("arm_group")
        self.head_group = moveit_commander.MoveGroupCommander("head_group")
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper_group")

        self.current_arm_id = None
        self.current_gripper_id = None
        self.current_head_id = None
        self.is_moving_base = False

        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # suscriber
        self.joy_subscriber = rospy.Subscriber('/joy', Joy, self.joy_callback)

    def joy_callback(self, joy_msg):
        twist = Twist()

        twist.linear.x = joy_msg.axes[3] * 0.5
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = joy_msg.axes[2] * 0.5
        if joy_msg.buttons[0] == 0 and joy_msg.buttons[1] == 0 and joy_msg.buttons[2] == 0 and joy_msg.buttons[3] == 0 and joy_msg.buttons[8] == 0 and joy_msg.buttons[9] == 0:
            self.is_moving_base = True
            self.cmd_vel_publisher.publish(twist)
        else:
            self.is_moving_base = False

        # botton 1-4 control arm1-4
        if joy_msg.buttons[0] == 1:
            self.current_arm_id = 0
            self.current_gripper_id = None
            self.current_head_id = None
        elif joy_msg.buttons[1] == 1:
            self.current_arm_id = 1
            self.current_gripper_id = None
            self.current_head_id = None
        elif joy_msg.buttons[2] == 1:
            self.current_arm_id = 2
            self.current_gripper_id = None
            self.current_head_id = None
        elif joy_msg.buttons[3] == 1:
            self.current_arm_id = 3
            self.current_gripper_id = None
            self.current_head_id = None

        elif joy_msg.buttons[8] == 1:
            self.current_gripper_id = 0
            self.current_arm_id = None
            self.current_head_id = None
        elif joy_msg.buttons[9] == 1:
            self.current_head_id = 0
            self.current_arm_id = None
            self.current_gripper_id = None

        if joy_msg.buttons[4] == 1 and joy_msg.buttons[5] == 1:
            self.set_arm_angle([0, 1.9199, 1.3090, -1.6705])
            time.sleep(1)
            self.open_gripper(-0.2618)
        if joy_msg.buttons[6] == 1 and joy_msg.buttons[7] == 1:
            self.open_gripper(0.18) # close gripper
            time.sleep(1)
            self.set_arm_angle([0, 0.5236, 1.0472, 0])
        

        # axes[5] control all angles
        if not self.is_moving_base:
            if self.current_arm_id is not None:
                self.control_joint_angle(joy_msg.axes[5])
            elif self.current_gripper_id is not None:
                self.control_gripper_angle(joy_msg.axes[5])
            elif self.current_head_id is not None:
                self.control_head_angle(joy_msg.axes[5])

    def control_joint_angle(self, axis_value):
        current_joint_values = self.arm_group.get_current_joint_values()
        if axis_value == 1:
            current_joint_values[self.current_arm_id] += 0.02 # rad
        elif axis_value == -1:
            current_joint_values[self.current_arm_id] -= 0.02

        self.arm_group.set_joint_value_target(current_joint_values)

        self.arm_group.go(wait=True)

        self.arm_group.stop()

    def control_gripper_angle(self, axis_value):
        current_joint_values = self.gripper_group.get_current_joint_values()
        if axis_value == 1:
            current_joint_values[self.current_gripper_id] += 0.05 # rad
        elif axis_value == -1:
            current_joint_values[self.current_gripper_id] -= 0.05

        self.gripper_group.set_joint_value_target(current_joint_values)

        self.gripper_group.go(wait=True)

        self.gripper_group.stop()   

    def control_head_angle(self, axis_value):
        current_joint_values = self.head_group.get_current_joint_values()
        if axis_value == 1:            
            current_joint_values[self.current_head_id] += 0.02 # rad
        elif axis_value == -1:
            current_joint_values[self.current_head_id] -= 0.02

        self.head_group.set_joint_value_target(current_joint_values)

        self.head_group.go(wait=True)

        self.head_group.stop()   

    def set_arm_angle(self, angle_rad):
        # set target angle
        joint_goal = self.arm_group.get_current_joint_values()
        joint_goal[0] = angle_rad[0]
        joint_goal[1] = angle_rad[1]
        joint_goal[2] = angle_rad[2]
        joint_goal[3] = angle_rad[3]

        # plan and execute        
        self.arm_group.go(joint_goal, wait=True)
        self.arm_group.stop() 
    def open_gripper(self,angle):
        # set target angle
        joint_goal = self.gripper_group.get_current_joint_values()
        joint_goal[0] = angle

        # plan and execute
        self.gripper_group.go(joint_goal, wait=True)
        self.gripper_group.stop() 

if __name__ == '__main__':
    controller = ArmController()
    rospy.spin()
