#!/usr/bin/env python2
import time
import os
import sys
import signal
import threading

import rospy

from mycobot_communication.msg import (
    MycobotAngles,
    MycobotSetAngles,
    MycobotCoords,
    MycobotGripperStatus,
    MycobotPumpStatus,
)

from pymycobot.mycobot import MyCobot




class MycobotTopics(object):


    def __init__(self):
        super(MycobotTopics, self).__init__()

        rospy.init_node("mycobot_topics_sending")
        rospy.loginfo("start ...")
        port = rospy.get_param("~port", "/dev/ttyAMA0")
        baud = rospy.get_param("~baud", 1000000)
        rospy.loginfo("%s,%s" % (port, baud))
        self.mc = MyCobot(port, baud)
        self.real_angle_pub = rospy.Publisher("mycobot/angles_real", MycobotAngles, queue_size=5)
        self.real_coords_pub = rospy.Publisher("mycobot/coords_real", MycobotCoords, queue_size=5)
        self.cmd_angle_sub = rospy.Subscriber(
            "mycobot/angles_goal", MycobotSetAngles, callback=self.cmd_angle_callback
        )
        self.cur_cmd_angles = []
        self.cur_cmd_speed = 0

    def cmd_angle_callback(self, msg):
        self.cur_cmd_angles = [
                msg.joint_1,
                msg.joint_2,
                msg.joint_3,
                msg.joint_4,
                msg.joint_5,
                msg.joint_6,
            ]
        self.cur_cmd_speed = int(msg.speed)

    def get_and_publish_real_angles(self):
        msg = MycobotAngles()
        rospy.loginfo("reading angles")
        angles = self.mc.get_angles()
        rospy.loginfo("read angles")
        msg.joint_1 = angles[0]
        msg.joint_2 = angles[1]
        msg.joint_3 = angles[2]
        msg.joint_4 = angles[3]
        msg.joint_5 = angles[4]
        msg.joint_6 = angles[5]
        self.real_angle_pub.publish(msg)
        rospy.loginfo("published angles")

    def get_and_publish_real_coords(self):
        msg = MycobotCoords()
        rospy.loginfo("reading coords")
        coords = self.mc.get_coords()
        rospy.loginfo("read coords")
        if not coords:
            rospy.logerror("coords did not come back")
        else:
            msg.x = coords[0]
            msg.y = coords[1]
            msg.z = coords[2]
            msg.rx = coords[3]
            msg.ry = coords[4]
            msg.rz = coords[5]
            self.real_coords_pub.publish(msg)
            rospy.loginfo("published coords")
    
    def set_cur_cmd_angles(self):
        rospy.loginfo("sending cmd angles")
        self.mc.send_angles(self.cur_cmd_angles, self.cur_cmd_speed)
        rospy.loginfo("sent cmd angles")

    def main(self):
        while not rospy.is_shutdown():
            self.get_and_publish_real_angles()
            self.get_and_publish_real_coords()
            if self.cur_cmd_angles:
                self.set_cur_cmd_angles()
        

    def sub_gripper_status(self):
        def callback(data):
            if data.Status:
                self.mc.set_gripper_state(0, 80)
            else:
                self.mc.set_gripper_state(1, 80)

        sub = rospy.Subscriber(
            "mycobot/gripper_status", MycobotGripperStatus, callback=callback
        )
        rospy.spin()

    def sub_pump_status(self):
        def callback(data):
            if data.Status:
                self.mc.set_basic_output(2, 0)
                self.mc.set_basic_output(5, 0)
            else:
                self.mc.set_basic_output(2, 1)
                self.mc.set_basic_output(5, 1)

        sub = rospy.Subscriber(
            "mycobot/pump_status", MycobotPumpStatus, callback=callback
        )
        rospy.spin()


if __name__ == "__main__":
    mc_topics = MycobotTopics()
    mc_topics.main()
    # while True:
    #     mc_topics.pub_real_coords()
    # mc_topics.sub_set_angles()
