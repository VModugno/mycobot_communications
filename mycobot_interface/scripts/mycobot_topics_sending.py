#!/usr/bin/env python2
import time
import os
import sys
import signal
import threading

import rospy

from mycobot_communication.msg import (
    MycobotAngles,
    MycobotCoords,
    MycobotGripperStatus,
    MycobotPumpStatus,
)

from pymycobot.mycobot import MyCobot
from pymycobot import PI_PORT, PI_BAUD




class MycobotTopics(object):
    def __init__(self):
        super(MycobotTopics, self).__init__()

        rospy.init_node("mycobot_topics_sending")
        rospy.loginfo("start ...")
        port = rospy.get_param("~port", PI_PORT)
        baud = rospy.get_param("~baud", PI_BAUD)
        rospy.loginfo("%s,%s" % (port, baud))
        self.mc = MyCobot(port, baud)
        self.real_angle_pub = rospy.Publisher("mycobot/angles_real", MycobotAngles, queue_size=5)

    def main(self):
        ma = MycobotAngles()
        while not rospy.is_shutdown():
            start_time = time.time()
            angles = self.mc.get_angles()
            elapsed_time = time.time() - start_time 
            print(angles)
            print('elapsed time:', elapsed_time)

    def start(self):
        pa = threading.Thread(target=self.pub_real_angles)
        #pb = threading.Thread(target=self.pub_real_coords)
        #sg = threading.Thread(target=self.sub_gripper_status)
        #sp = threading.Thread(target=self.sub_pump_status)

        pa.setDaemon(True)
        pa.start()
        #pb.setDaemon(True)
        #pb.start()
        #sg.setDaemon(True)
        #sg.start()
        #sp.setDaemon(True)
        #sp.start()

        pa.join()
        #pb.join()
        #sg.join()
        #sp.join()

    def pub_real_angles(self):
        pub = rospy.Publisher("mycobot/angles_real", MycobotAngles, queue_size=5)
        ma = MycobotAngles()
        while not rospy.is_shutdown():
            #self.lock.acquire()
            start_time = time.time()
            angles = self.mc.get_angles()
            elapsed_time = time.time() - start_time 
            print(angles)
            print('elapsed time:', elapsed_time)
            #self.lock.release()
            if angles:
                pass
                #ma.joint_1 = angles[0]
                #ma.joint_2 = angles[1]
                #ma.joint_3 = angles[2]
                #ma.joint_4 = angles[3]
                #ma.joint_5 = angles[4]
                #ma.joint_6 = angles[5]
                #pub.publish(ma)
            #time.sleep(0.25)

    def pub_real_coords(self):
        pub = rospy.Publisher("mycobot/coords_real", MycobotCoords, queue_size=5)
        ma = MycobotCoords()

        while not rospy.is_shutdown():
            self.lock.acquire()
            coords = self.mc.get_coords()
            self.lock.release()
            if coords:
                ma.x = coords[0]
                ma.y = coords[1]
                ma.z = coords[2]
                ma.rx = coords[3]
                ma.ry = coords[4]
                ma.rz = coords[5]
                pub.publish(ma)
            #time.sleep(0.25)


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
