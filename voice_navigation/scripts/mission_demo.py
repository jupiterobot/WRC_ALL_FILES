#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Basic Modules
import rospy
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient


class MissionDemo(object):

    def __init__(self):
        # Flags
        self._FLAG_EXECUTE = 0
        self._FLAG_NAVI = 1
        self._FLAG_GOAL = 0
        # Soundplay parameter
        self.voice = rospy.get_param("~voice", "voice_kal_diphone")
        self.speaker = SoundClient(blocking=True)
        rospy.sleep(1)
        rospy.init_node("mission_demo", disable_signals=True)

    def main_loop(self):
        # Initial
        self.pub_nav_cmd = rospy.Publisher("nav_cmd", String, queue_size=1)
        self.pub_base_cmd = rospy.Publisher("base_cmd", String, queue_size=1)
        rospy.Subscriber("nav_feedback", String, self._navi_callback)
        rospy.Subscriber("lm_data", String, self._voice_callback)
        rospy.sleep(1)
        # Navigating to master-point
        while (True):
            if self._FLAG_EXECUTE == 1:
                self.pub_nav_cmd.publish("G")
                rospy.loginfo("Going to the other chamber.")
                self.speaker.say("Going to the other chamber.", self.voice)
                rospy.sleep(2)
                break
        # Reached the point and ask
        while (True):
            if self._FLAG_NAVI == 0:
                rospy.loginfo("Reached the chamber.")
                self.speaker.say("Reached the chamber.", self.voice)
                rospy.sleep(2)
                rospy.loginfo("Nice to meet you, Sir! Please tell me my goal point.")
                self.speaker.say("Nice to meet you, Sir! Please tell me my goal point.", self.voice)
                rospy.sleep(2)
                break
        self._FLAG_NAVI = 2
        # Navigating to the goal point
        while (True):
            if self._FLAG_NAVI == 1:
                if(self._FLAG_GOAL==1):
                    nav_cmd = "A"
                    goal_pos = "1"
                elif(self._FLAG_GOAL==2):
                    nav_cmd = "B"
                    goal_pos = "2"
                elif(self._FLAG_GOAL==3):
                    nav_cmd = "C"
                    goal_pos = "3"
                elif(self._FLAG_GOAL==4):
                    nav_cmd = "D"
                    goal_pos = "4"
                elif(self._FLAG_GOAL==5):
                    nav_cmd = "E"
                    goal_pos = "5"
                elif(self._FLAG_GOAL==6):
                    nav_cmd = "F"
                    goal_pos = "6"
                rospy.sleep(1)
                try:
                    rospy.loginfo("Got it. I'm going to Point {}".format(goal_pos))
                    self.speaker.say("Got it. I'm going to Point {}".format(goal_pos), self.voice)
                    rospy.sleep(2)
                    self.pub_nav_cmd.publish(nav_cmd)
                except:
                    pass
                break
        # Reached the point
        while (True):
            if self._FLAG_NAVI == 0:
                rospy.loginfo("Got the point.")
                self.speaker.say("Got the point.", self.voice)
                rospy.sleep(2)
                break
        #rospy.spin()

    def _navi_callback(self, msg):
        if msg.data.find("Done") > -1:
            self._FLAG_NAVI = 0

    def _voice_callback(self, msg):
        global target
        if msg.data.find("OVER") > -1 and msg.data.find("HELLO") > -1:
            self._FLAG_EXECUTE = 1
        if self._FLAG_NAVI == 2:
            if msg.data.find("1") > -1:
                self._FLAG_GOAL = 1
                self._FLAG_NAVI = 1
            elif msg.data.find("2") > -1:
                self._FLAG_GOAL = 2
                self._FLAG_NAVI = 1
            elif msg.data.find("3") > -1:
                self._FLAG_GOAL = 3
                self._FLAG_NAVI = 1
            elif msg.data.find("4") > -1:
                self._FLAG_GOAL = 4
                self._FLAG_NAVI = 1
            elif msg.data.find("5") > -1:
                self._FLAG_GOAL = 5
                self._FLAG_NAVI = 1
            elif msg.data.find("6") > -1:
                self._FLAG_GOAL = 6
                self._FLAG_NAVI = 1
            


if __name__ == "__main__":
    controller = MissionDemo()
    controller.main_loop()
