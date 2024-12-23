#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf.transformations as tft
import time
from sound_play.libsoundplay import SoundClient  # import SoundClient
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class MoveToGoalNode:
    def __init__(self):
        # init node
        rospy.init_node('move_to_goal_node', anonymous=True)
        self.sound_client = SoundClient()  # init sound client
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # subscribe to voice recognition output
        # self.subscriber = rospy.Subscriber('/recognizer/output', String, self.callback)
        self.subscriber = None
        self.listening = False
        self.awaiting_confirmation_1 = False
        self.awaiting_confirmation_2 = False
        self.awaiting_confirmation_3 = False
        self.awaiting_confirmation_4 = False 
        self.awaiting_confirmation_5 = False 
        self.awaiting_confirmation_6 = False   

    
    def start_listening(self):
        if not self.listening:
            self.subscriber = rospy.Subscriber('/recognizer/output', String, self.confirmation_callback)
            self.listening = True   

    def confirmation_callback(self, msg):
        if self.awaiting_confirmation_1:
            if msg.data.lower() == "yes":
                self.awaiting_confirmation_1 = False
                rospy.loginfo("Moving to point one...")
                self.cmd_vel_control()
                self.move_to_goal(-0.443407, 0.356996, 0.000019, -0.000003, 0.005434, -0.000442)
                rospy.loginfo("Reached the point one.")
                # time.sleep(2)
                # self.choose_action()
            elif msg.data.lower() == "no":
                self.awaiting_confirmation_1 = False
                rospy.loginfo("Cancelled moving to point one.")
                self.start_listening()

        elif self.awaiting_confirmation_2:
            if msg.data.lower() == "yes":
                self.awaiting_confirmation_2 = False
                rospy.loginfo("Moving to point two...")
                self.cmd_vel_control()
                self.move_to_goal(-0.443910, -0.420651, 0.000017, -0.000001, 0.005448, -0.000442)
                rospy.loginfo("Reached the point two.")
                # time.sleep(2)
                # self.choose_action()
            elif msg.data.lower() == "no":
                self.awaiting_confirmation_2 = False
                rospy.loginfo("Cancelled moving to point two.")
                self.start_listening()

        elif self.awaiting_confirmation_3:
            if msg.data.lower() == "yes":
                self.awaiting_confirmation_3 = False
                rospy.loginfo("Moving to point three...")
                self.cmd_vel_control()
                self.move_to_goal(-0.447450, -1.191253, 0.000094, 0.000001, 0.005012, -0.000558)
                rospy.loginfo("Reached the point three.")
                # time.sleep(2)
                # self.choose_action()
            elif msg.data.lower() == "no":
                self.awaiting_confirmation_3 = False
                rospy.loginfo("Cancelled moving to point three.")
                self.start_listening()

        elif self.awaiting_confirmation_4:
            if msg.data.lower() == "yes":
                self.awaiting_confirmation_4 = False
                rospy.loginfo("Moving to point four...")
                self.cmd_vel_control()
                self.move_to_goal(-0.448613, -2.205272, 0.001000, 0.000000, 0.000000, -0.000558)
                rospy.loginfo("Reached the point four.")
                # time.sleep(2)
                # self.choose_action()
            elif msg.data.lower() == "no":
                self.awaiting_confirmation_4 = False
                rospy.loginfo("Cancelled moving to point four.")
                self.start_listening()

        elif self.awaiting_confirmation_5:
            if msg.data.lower() == "yes":
                self.awaiting_confirmation_5 = False
                rospy.loginfo("Moving to point five...")
                self.cmd_vel_control()
                self.move_to_goal(-0.460933, -3.000043, 0.000095, 0.000005, 0.005008, -0.000522)
                rospy.loginfo("Reached the point five.")
                # time.sleep(2)
                # self.choose_action()
            elif msg.data.lower() == "no":
                self.awaiting_confirmation_5 = False
                rospy.loginfo("Cancelled moving to point five.")
                self.start_listening()

        elif self.awaiting_confirmation_6:
            if msg.data.lower() == "yes":
                self.awaiting_confirmation_6 = False
                rospy.loginfo("Moving to point six...")
                self.cmd_vel_control()
                self.move_to_goal(-0.459640, -3.763645, 0.000096, 0.000003, 0.005002, 0.145558)
                rospy.loginfo("Reached the point six.")
                # time.sleep(2)
                # self.choose_action()
            elif msg.data.lower() == "no":
                self.awaiting_confirmation_6 = False
                rospy.loginfo("Cancelled moving to point six.")
                self.start_listening()

        else:
            if msg.data == "1":
                print("I heard 1")
                self.sound_client.say("Do you confirm moving to the point one?")
                time.sleep(0.5)
                self.awaiting_confirmation_1 = True
                self.awaiting_confirmation_2 = False
                self.awaiting_confirmation_3 = False
                self.awaiting_confirmation_4 = False
                self.awaiting_confirmation_5 = False
                self.awaiting_confirmation_6 = False
                
                self.start_listening()

            elif msg.data == "2":
                print("I heard 2")
                self.sound_client.say("Do you confirm moving to the point two?")
                time.sleep(0.5)
                self.awaiting_confirmation_2 = True
                self.awaiting_confirmation_1 = False
                self.awaiting_confirmation_3 = False
                self.awaiting_confirmation_4 = False
                self.awaiting_confirmation_5 = False
                self.awaiting_confirmation_6 = False
                self.start_listening()

            elif msg.data == "3":
                print("I heard 3")
                self.sound_client.say("Do you confirm moving to the point three?")
                time.sleep(0.5)
                self.awaiting_confirmation_3 = True
                self.awaiting_confirmation_2 = False
                self.awaiting_confirmation_1 = False
                self.awaiting_confirmation_4 = False
                self.awaiting_confirmation_5 = False
                self.awaiting_confirmation_6 = False
                self.start_listening()

            elif msg.data == "4":
                print("I heard 4")
                self.sound_client.say("Do you confirm moving to the point four?")
                time.sleep(0.5)
                self.awaiting_confirmation_4 = True
                self.awaiting_confirmation_3 = False
                self.awaiting_confirmation_2 = False
                self.awaiting_confirmation_1 = False
                self.awaiting_confirmation_5 = False
                self.awaiting_confirmation_6 = False
                self.start_listening()

            elif msg.data == "5":
                print("I heard 5")
                self.sound_client.say("Do you confirm moving to the point five?")
                time.sleep(0.5)
                self.awaiting_confirmation_5 = True
                self.awaiting_confirmation_3 = False
                self.awaiting_confirmation_2 = False
                self.awaiting_confirmation_1 = False
                self.awaiting_confirmation_4 = False
                self.awaiting_confirmation_6 = False
                self.start_listening() 

            elif msg.data == "6":
                print("I heard 6")
                self.sound_client.say("Do you confirm moving to the point six")
                time.sleep(0.5)
                self.awaiting_confirmation_6 = True
                self.awaiting_confirmation_3 = False
                self.awaiting_confirmation_2 = False
                self.awaiting_confirmation_1 = False
                self.awaiting_confirmation_4 = False
                self.awaiting_confirmation_5 = False
                self.start_listening() 

    # def choose_action(self):
    #     while True:
    #         action = input("Type 'stay' to stay at actuel position or 'return' to go back to the room: ")
    #         if action.lower() == "stay":
    #             rospy.loginfo("staying...")
    #             break
                    
    #         elif action.lower() == "return":
    #             self.move_to_goal(-1.583318, -2.954982, 0.001000, 0.0, 0.0, -1.556272)
    #             rospy.loginfo("Robot returned to the room.")
    #             break
    #         else:
    #             rospy.loginfo("Invalid input. Please try again.")

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
        start_task = input("Do you wanna to start a collaborative task? (yes/no): ")
        if start_task.lower() == 'yes':
            rospy.loginfo("Collaborative task will start in 2 seconds...")
            time.sleep(2)
            self.vel_control()
            time.sleep(0.5)
            self.move_to_goal(-1.583318, -2.954982, 0.001000, 0.0, 0.0, -1.556272)
            time.sleep(2)
            # play sound
            self.sound_client.say("Reached the chamber.")
            time.sleep(2)
            self.sound_client.say("Nice to meet you, Sir! Please tell me my goal point.")
            
            self.start_listening()


            rate = rospy.Rate(10)  # 10 Hz
            while not rospy.is_shutdown() and self.listening:
                rate.sleep()  

        else:
            rospy.loginfo("Task not started.")

    def cmd_vel_control(self):
        twist = Twist()
        twist.linear.x = -0.3
        twist.angular.z = 0.0
        rate = rospy.Rate(10)  # 10 Hz

        start_time = rospy.get_time()
        while rospy.get_time() - start_time < 2.0:
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        #rospy.loginfo("ready to catch the red flask")
        stop_cmd_vel = Twist()
        self.cmd_vel_pub.publish(stop_cmd_vel)

    def vel_control(self):
        print("  ")
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.5
        rate = rospy.Rate(10)  # 10 Hz

        start_time = rospy.get_time()
        while rospy.get_time() - start_time < 5:
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        #rospy.loginfo("ready to catch the red flask")
        stop_cmd_vel = Twist()
        self.cmd_vel_pub.publish(stop_cmd_vel)

if __name__ == '__main__':
    try:
        node = MoveToGoalNode()
        node.move_to_chamber()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Program interrupted before completion.")
