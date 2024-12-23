#!/usr/bin/env python
#coding:UTF-8

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def main():
    # 初始化 Twist 控制消息
    global twist, enable, pub
    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    enable = 0

    # 初始化 ros 节点
    rospy.init_node("base_control", anonymous=False)

    # 初始化控制命令订阅者
    rospy.Subscriber("base_cmd", String, cmd_vel_callback)

    # 初始化控制命令发布者
    pub = rospy.Publisher("cmd_vel_mux/input/teleop", Twist, queue_size=1)

    # 初始化 ros主循环
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if enable == 1:
            pub.publish(twist)
        rate.sleep()

def cmd_vel_callback(msg):
    global enable
    if msg.data.find("whirl") > -1:
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0.2
        enable = 1
    elif msg.data.find("forward") > -1:
        twist.linear.x = 0.1
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        enable = 1
    elif msg.data.find("back") > -1:
        twist.linear.x = -0.1
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        enable = 1
    else:
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        enable = 0
        pub.publish(twist)

if __name__ == "__main__":
    main()
