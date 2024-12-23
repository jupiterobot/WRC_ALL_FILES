#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf.transformations as tft
import time
from sound_play.libsoundplay import SoundClient  # import SoundClient
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

import sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectoryPoint
from geometry_msgs.msg import Twist


class MoveToGoalNode:
    def __init__(self):
        # init node
        rospy.init_node('move_to_goal_node', anonymous=True)
        self.sound_client = SoundClient()  # init sound client

        moveit_commander.roscpp_initialize(sys.argv)
        self.head_group = moveit_commander.MoveGroupCommander("head_group")
        self.arm_group = moveit_commander.MoveGroupCommander("arm_group")
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper_group")

        self.bridge = CvBridge()
        self.image_subscriber = None
        self.found_red_flask = False
        self.found_green_flask = False
        self.found_blue_flask = False
        self.arm_in_grasp_positioin = False
        self.only_display_image = False

        self.detecting_red = False
        self.detecting_green = False
        self.detecting_blue = False

        self.subscriber = None
        self.listening = False
        self.awaiting_confirmation = False
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    def rotate_robot(self,angular_speed=0.5):
        twist = Twist()
        twist.angular.z = angular_speed
        self.cmd_vel_pub.publish(twist)
    def stop_robot(self):
        twist = Twist()
    
        # 设置线性速度为 0
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
    
        # 设置角速度为 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
    
        # 发布停止指令
        self.cmd_vel_pub.publish(twist)

    def start_image_stream(self):
        if self.image_subscriber is None:
            self.image_subscriber = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
    
    
    def image_callback(self, image_msg):
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        if self.arm_in_grasp_positioin:
            cv2.imshow("Camera View", cv_image)
            cv2.waitKey(3)
            return
        
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # red range
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv_image, lower_red, upper_red)

        # green range
        lower_green = np.array([35, 50, 50])
        upper_green = np.array([85, 255, 255])
        green_mask = cv2.inRange(hsv_image, lower_green, upper_green)

        # blue range
        lower_blue = np.array([100, 50, 20])
        upper_blue = np.array([140, 255, 60])
        blue_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

        # elimate noise
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        green_mask = cv2.erode(green_mask, None, iterations=2)
        green_mask = cv2.dilate(green_mask, None, iterations=2)

        blue_mask = cv2.erode(blue_mask, None, iterations=2)
        blue_mask = cv2.dilate(blue_mask, None, iterations=2)

        # find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if self.detecting_red:
            # draw rectangles around contours
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 500:  # filter out small contours
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    # Add text label
                    label = "Red Flask"
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    font_scale = 0.5
                    color = (255, 255, 255)  # White text
                    thickness = 1
                    cv2.putText(cv_image, label, (x, y - 10), font, font_scale, color, thickness)

                    flask_center_x = x + w / 2
                    flask_center_y = y + h / 2
                    image_center_x = cv_image.shape[1] / 2
                    image_center_y = cv_image.shape[0] / 2

                    x_deviation = flask_center_x - image_center_x
                    y_deviation = flask_center_y - image_center_y

                    tolerance_x = 2
                    tolerance_y = 2
                
                    # Adjust robot's position
                    if abs(x_deviation) > tolerance_x or abs(y_deviation) > tolerance_y:
                        twist = Twist()

                        # If flask is to the left or right, adjust angular velocity
                        if abs(x_deviation) > tolerance_x:
                            twist.angular.z = -0.005 * x_deviation  # Negative for left, positive for right

                        # If flask is too high or low in the image, adjust linear velocity
                        if abs(y_deviation) > tolerance_y:
                            twist.linear.x = -0.005 * y_deviation  # Negative to move forward, positive to move backward

                        self.cmd_vel_pub.publish(twist)
                    else:
                        self.stop_robot()
                        self.found_red_flask = True
                    break
        elif self.detecting_green:
            for contour in green_contours:
                area = cv2.contourArea(contour)
                if area > 200:
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    label = "Green Flask"
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    font_scale = 0.5
                    color = (255, 255, 255)
                    thickness = 1
                    cv2.putText(cv_image, label, (x, y - 10), font, font_scale, color, thickness)

                    flask_center_x = x + w / 2
                    flask_center_y = y + h / 2
                    image_center_x = cv_image.shape[1] / 2
                    image_center_y = cv_image.shape[0] / 2

                    x_deviation = flask_center_x - image_center_x
                    y_deviation = flask_center_y - image_center_y

                    tolerance_x = 2.5
                    tolerance_y = 2.5

                    if abs(x_deviation) > tolerance_x or abs(y_deviation) > tolerance_y:
                        twist = Twist()

                        if abs(x_deviation) > tolerance_x:
                            twist.angular.z = -0.005 * x_deviation 

                        if abs(y_deviation) > tolerance_y:
                            twist.linear.x = -0.005 * y_deviation 

                        self.cmd_vel_pub.publish(twist)
                    else:
                        self.stop_robot()
                        self.found_green_flask = True
                    break

        elif self.detecting_blue:
            for contour in blue_contours:
                area = cv2.contourArea(contour)
                if area > 200: 
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    label = "Blue Flask"
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    font_scale = 0.5
                    color = (255, 255, 255)
                    thickness = 1
                    cv2.putText(cv_image, label, (x, y - 10), font, font_scale, color, thickness)

                    flask_center_x = x + w / 2
                    flask_center_y = y + h / 2
                    image_center_x = cv_image.shape[1] / 2
                    image_center_y = cv_image.shape[0] / 2

                    x_deviation = flask_center_x - image_center_x
                    y_deviation = flask_center_y - image_center_y

                    tolerance_x = 2.5
                    tolerance_y = 2.5

                    if abs(x_deviation) > tolerance_x or abs(y_deviation) > tolerance_y:
                        twist = Twist()

                        if abs(x_deviation) > tolerance_x:
                            twist.angular.z = -0.005 * x_deviation 

                        if abs(y_deviation) > tolerance_y:
                            twist.linear.x = -0.005 * y_deviation 

                        self.cmd_vel_pub.publish(twist)
                    else:
                        self.stop_robot()
                        self.found_blue_flask = True
                    break

        cv2.imshow("Camera View", cv_image)
        cv2.waitKey(3)


    def start_listening(self):
        if not self.listening:
            self.subscriber = rospy.Subscriber('/recognizer/output', String, self.confirmation_callback)
            self.listening = True 

    def confirmation_callback(self, msg):
        if self.awaiting_confirmation:
            if msg.data.lower() == "yes":
                if self.current_task == "red-flask":
                    # self.awaiting_confirmation = False
                    self.sound_client.say("Great! I will take the red flask.")
                    rospy.loginfo("Moving to the Search Area...")
                    self.cmd_vel_control()
                    time.sleep(0.5)
                    self.move_to_goal(1.392033, 0.061360, 0.000100, 0.000000, 0.000000, -1.684001)
                    time.sleep(2)
                    rospy.loginfo("Reached Search Area, beginning to search for red flask...")
                
                    rospy.loginfo("Adjusting head angle and robot_arm position...")
                    self.set_head_angle(0.5)  # set head angle to look down
                    self.set_arm_angle([0, -1.5708, 1.5708, 0])

                    rospy.loginfo("camera looking down, truning on camera...")
                    self.start_image_stream()

                    while not rospy.is_shutdown() and not self.found_red_flask:
                        self.rotate_robot(angular_speed=0.5)
                        time.sleep(0.1)

                    self.stop_robot()
                    rospy.loginfo("Red flask found!")
                    time.sleep(2)
                    rospy.loginfo("Start adjusting pose and position...")
                    if self.found_red_flask:
                        rospy.loginfo("Red flask centered!")
                        self.arm_in_grasp_positioin = True
                        time.sleep(2)
                        self.stop_robot()
                        self.stop_robot()
                        rospy.loginfo("Adjusting arm pose and open gripper for picking up the red flask...")
                        self.set_arm_angle([0, 1.9199, 1.3090, -1.6705])
                        time.sleep(1)
                        self.stop_robot()
                        self.stop_robot()
                        self.open_gripper(-0.2618)
                        time.sleep(0.5)
                        self.vel_control()
                        time.sleep(0.5)
                        self.open_gripper(0.18) # close gripper
                        rospy.loginfo("Red flask picked up!")
                        self.set_arm_angle([0, 0.5236, 1.0472, 0])
                        time.sleep(0.5)
                        self.move_to_goal(-1.580803, -0.527339, 0.001000, 0.000000, 0.000000, 1.579850)
                        rospy.loginfo("Reached the chamber, waiting for dropping the red flask...")
                        time.sleep(2)
                        self.set_arm_angle([0, 1.0472, 0.5236, 0])
                        time.sleep(0.5)
                        self.open_gripper(-0.15)
                        rospy.loginfo("Red flask dropped! Tache done!")
                        rospy.loginfo("Resetting robot's pose and position...")
                        time.sleep(2)
                        self.open_gripper(0)
                        self.set_arm_angle([0, 0, 0, 0])
                        time.sleep(0.5)
                        self.set_head_angle(0)

                elif self.current_task == "green-flask":
                    if self.current_task == "green-flask":
                        # self.awaiting_confirmation = False
                        self.sound_client.say("Great! I will take the green flask.")
                        rospy.loginfo("Moving to the Search Area...")
                        self.cmd_vel_control()
                        time.sleep(0.5)
                        self.move_to_goal(1.392033, 0.061360, 0.000100, 0.000000, 0.000000, -1.684001)
                        time.sleep(2)
                        rospy.loginfo("Reached Search Area, beginning to search for green flask...")
                
                        rospy.loginfo("Adjusting head angle and robot_arm position...")
                        self.set_head_angle(0.5)  # set head angle to look down
                        self.set_arm_angle([0, -1.5708, 1.5708, 0])

                        rospy.loginfo("camera looking down, truning on camera...")
                        self.start_image_stream()

                        while not rospy.is_shutdown() and not self.found_green_flask:
                            self.rotate_robot(angular_speed=0.5)
                            time.sleep(0.1)

                        self.stop_robot()
                        rospy.loginfo("Green flask found!")
                        time.sleep(2)
                        rospy.loginfo("Start adjusting pose and position...")
                        if self.found_green_flask:
                            rospy.loginfo("Green flask centered!")
                            self.arm_in_grasp_positioin = True
                            time.sleep(2)
                            self.stop_robot()
                            self.stop_robot()
                            rospy.loginfo("Adjusting arm pose and open gripper for picking up the green flask...")
                            self.set_arm_angle([0, 1.9199, 1.3090, -1.6705])
                            time.sleep(1)
                            self.stop_robot()
                            self.stop_robot()
                            self.open_gripper(-0.2618)
                            time.sleep(0.5)
                            self.vel_control() ####### change......
                            time.sleep(0.5)
                            self.open_gripper(0.18) # close gripper
                            rospy.loginfo("Green flask picked up!")
                            self.set_arm_angle([0, 0.5236, 1.0472, 0])
                            time.sleep(0.5)
                            self.move_to_goal(-1.580803, -0.527339, 0.001000, 0.000000, 0.000000, 1.579850)
                            rospy.loginfo("Reached the chamber, waiting for dropping the green flask...")
                            time.sleep(2)
                            self.set_arm_angle([0, 1.0472, 0.5236, 0])
                            time.sleep(0.5)
                            self.open_gripper(-0.15)
                            rospy.loginfo("Green flask dropped! Tache done!")
                            rospy.loginfo("Resetting robot's pose and position...")
                            time.sleep(2)
                            self.open_gripper(0)
                            self.set_arm_angle([0, 0, 0, 0])
                            time.sleep(0.5)
                            self.set_head_angle(0)

                elif self.current_task == "blue-flask":
                    # self.awaiting_confirmation = False
                    self.sound_client.say("Great! I will take the blue flask.")
                    rospy.loginfo("Moving to the Search Area...")
                    self.cmd_vel_control()
                    time.sleep(0.5)
                    self.move_to_goal(1.392033, 0.061360, 0.000100, 0.000000, 0.000000, -1.684001)
                    time.sleep(2)
                    rospy.loginfo("Reached Search Area, beginning to search for blue flask...")
                
                    rospy.loginfo("Adjusting head angle and robot_arm position...")
                    self.set_head_angle(0.5)  # set head angle to look downs
                    self.set_arm_angle([0, -1.5708, 1.5708, 0])

                    rospy.loginfo("camera looking down, truning on camera...")
                    self.start_image_stream()

                    while not rospy.is_shutdown() and not self.found_blue_flask:
                        self.rotate_robot(angular_speed=0.5)
                        time.sleep(0.1)

                    self.stop_robot()
                    rospy.loginfo("Blue flask found!")
                    time.sleep(2)
                    rospy.loginfo("Start adjusting pose and position...")
                    if self.found_blue_flask:
                        rospy.loginfo("Blue flask centered!")
                        self.arm_in_grasp_positioin = True
                        time.sleep(2)
                        self.stop_robot()
                        self.stop_robot()
                        rospy.loginfo("Adjusting arm pose and open gripper for picking up the blue flask...")
                        self.set_arm_angle([0, 1.9199, 1.3090, -1.6705])
                        time.sleep(1)
                        self.stop_robot()
                        self.stop_robot()
                        self.open_gripper(-0.2618)
                        time.sleep(0.5)
                        self.vel_control() ####### change......
                        time.sleep(0.5)
                        self.open_gripper(0.18) # close gripper
                        rospy.loginfo("Blue flask picked up!")
                        self.set_arm_angle([0, 0.5236, 1.0472, 0])
                        time.sleep(0.5)
                        self.cmd_vel_control()
                        time.sleep(0.5)
                        self.move_to_goal(-1.580803, -0.527339, 0.001000, 0.000000, 0.000000, 1.579850)
                        rospy.loginfo("Reached the chamber, waiting for dropping the blue flask...")
                        time.sleep(2)
                        self.set_arm_angle([0, 1.0472, 0.5236, 0])
                        time.sleep(0.5)
                        self.open_gripper(-0.15)
                        rospy.loginfo("Blue flask dropped! Tache done!")
                        rospy.loginfo("Resetting robot's pose and position...")
                        time.sleep(2)
                        self.open_gripper(0)
                        self.set_arm_angle([0, 0, 0, 0])
                        time.sleep(0.5)
                        self.set_head_angle(0)

            elif msg.data.lower() == "no":
                self.awaiting_confirmation = False
                rospy.loginfo("Cancelled moving to red flask.")
                self.start_listening()

        else:
            if "red-flask" in msg.data:
                print("I heard RED FLASK")
                self.sound_client.say("Do you confirm going to take the red flask?")
                time.sleep(0.5)
                self.awaiting_confirmation = True
                self.detecting_red = True
                self.current_task = "red-flask"  # current task is red-flask
                self.start_listening() 
            elif "green-flask" in msg.data:
                print("I heard GREEN FLASK")
                self.sound_client.say("Do you confirm going to take the green flask?")
                time.sleep(0.5)
                self.awaiting_confirmation = True
                self.detecting_green = True
                self.current_task = "green-flask"  # current task is green-flask
                self.start_listening()
            elif "blue-flask" in msg.data:
                print("I heard BLUE FLASK")
                self.sound_client.say("Do you confirm going to take the blue flask?")
                time.sleep(0.5)                
                self.awaiting_confirmation = True
                self.detecting_blue = True
                self.current_task = "blue-flask"  # current task is blue-flask
                self.start_listening()


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

    def set_head_angle(self, angle_rad):

        # set target angle
        joint_goal = self.head_group.get_current_joint_values()
        joint_goal[0] = angle_rad

        # plan and execute
        self.head_group.go(joint_goal, wait=True)
        self.head_group.stop() 

    def open_gripper(self,angle):
        # set target angle
        joint_goal = self.gripper_group.get_current_joint_values()
        joint_goal[0] = angle

        # plan and execute
        self.gripper_group.go(joint_goal, wait=True)
        self.gripper_group.stop() 

    def vel_control(self):
        twist = Twist()
        twist.linear.x = 0.05
        twist.angular.z = 0.0
        rate = rospy.Rate(10)  # 10 Hz

        start_time = rospy.get_time()
        while rospy.get_time() - start_time < 13.8:
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        rospy.loginfo("ready to catch the red flask")
        stop_cmd_vel = Twist()
        self.cmd_vel_pub.publish(stop_cmd_vel)

    def cmd_vel_control(self):
        twist = Twist()
        twist.linear.x = -0.1
        twist.angular.z = 0.0
        rate = rospy.Rate(10)  # 10 Hz

        start_time = rospy.get_time()
        while rospy.get_time() - start_time < 4.0:
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        rospy.loginfo("ready to catch the red flask")
        stop_cmd_vel = Twist()
        self.cmd_vel_pub.publish(stop_cmd_vel)

    def move_to_goal(self, x, y, z, roll, pitch, yaw):
        # action client
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        client.wait_for_server()

        # set goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # target position
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = z

        quaternion = tft.quaternion_from_euler(roll, pitch, yaw)
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

        # send goal
        rospy.loginfo("Moving to goal...")
        client.send_goal(goal)
        client.wait_for_result()
        rospy.loginfo("Goal reached.")

    def move_to_chamber(self):
        # ask user if start collaborative task
        start_task = input("Do you wanna to start an automatic task? (yes/no): ")
        if start_task.lower() == 'yes':
            rospy.loginfo("Automatic task will start in 2 seconds...")
            time.sleep(2)

            # set target position
            x = -1.580803
            y = -0.527339
            z = 0.001000
            roll = 0.000000
            pitch = 0.000000
            yaw = 1.579850

            time.sleep(0.1)
            self.move_to_goal(-1.553708, -1.582060, 0.001000, 0.000000, 0.000000, 1.526966)
            time.sleep(0.1)
            self.move_to_goal(x, y, z, roll, pitch, yaw)

            time.sleep(2)
            # play sound
            self.sound_client.say("Reached the chamber.")
            time.sleep(2)
            self.sound_client.say("Hello sir, what can I do for you?")
            
            self.start_listening()


            rate = rospy.Rate(10)  # 10 Hz
            while not rospy.is_shutdown() and self.listening:
                rate.sleep()  

        else:
            rospy.loginfo("Task not started.")

if __name__ == '__main__':
    try:
        node = MoveToGoalNode()
        node.move_to_chamber()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Program interrupted before completion.")
