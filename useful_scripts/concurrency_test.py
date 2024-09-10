#!/usr/bin/env python3

"""
the goal of this script is to test if we can read/write from the serial port at the same time.
We want to do this so we can have the actual joint angles continuously published at 100HZ or more,
even as we command the robot. We need to check loop rate in HZ, as well as accuracy of joint metrics.
"""

from functools import wraps
import math
import pprint
import signal
import sys
import threading
import time

import pymycobot
from pymycobot.mycobot import MyCobot
from pymycobot.error import MyCobotDataException

EXIT_FLAG = False
NUM_JOINTS = 6

def log_msg(msg):
    print(msg)


def signal_handler(sig, frame):
    global EXIT_FLAG
    EXIT_FLAG = True
    log_msg('You pressed Ctrl+C!')

class CurRealAngles:
    def __init__(self, angles, query_time):
        self.angles = angles
        self.query_time = query_time

class CmdAngles:
    def __init__(self, angles, speed, cmd_time):
        self.angles = angles
        self.speed = speed
        self.query_time = cmd_time

class MycobotTopics(object):

    def __init__(self):
        super(MycobotTopics, self).__init__()

        port = "/dev/ttyAMA0"
        baud = 1000000

        self.mc = MyCobot(port, baud, thread_lock=True)
    
        self.angle_queries = []

        self.get_angles_target_hz = 100
        self.get_angles_target_seconds = 1 / self.get_angles_target_hz
        self.last_get_angles_time = time.time()
        self.get_angle_thread = threading.Thread(target=self.get_angles)

        self.command_arm_target_hz = 20
        self.command_arm_target_seconds = 1 / self.command_arm_target_hz
        self.last_command_arm_time = time.time()
        self.command_speed = 80
        self.max_angle = 50
        self.cur_counter = 0
        self.counter_incr = 1
        self.cmds_sent = []
        self.cmd_thread = threading.Thread(target=self.command_arm)

        if not self.mc.is_controller_connected():
            raise RuntimeError("not connected with atom")
        
    def get_angles(self):
        while not EXIT_FLAG:
            time_since_loop = time.time() - self.last_get_angles_time
            if time_since_loop < self.get_angles_target_seconds:
                time.sleep(self.get_angles_target_seconds - time_since_loop)
            self.last_get_angles_time = time.time()
            angles = self.mc.get_angles()
            cur_angles = CurRealAngles(angles, self.last_get_angles_time)
            self.angle_queries.append(cur_angles)
    
    def command_arm(self):
        while not EXIT_FLAG:
            time_since_loop = time.time() - self.last_command_arm_time
            if time_since_loop < self.command_arm_target_seconds:
                time.sleep(self.command_arm_target_seconds - time_since_loop)
            self.last_command_arm_time = time.time()

            cur_angle = math.sin(self.cur_counter * math.pi / 180) * self.max_angle
            self.cur_counter += self.counter_incr

            cmd = CmdAngles([cur_angle for i in range(NUM_JOINTS)], self.command_speed, self.last_command_arm_time)
            self.mc.send_angles(cmd.angles, cmd.speed)
            self.cmds_sent.append(cmd)
    
    def main(self):
        signal.signal(signal.SIGINT, signal_handler)
        self.mc.send_angles([0] * NUM_JOINTS, 60)
        time.sleep(4)
        self.cmd_thread.start()
        self.get_angle_thread.start()
        self.cmd_thread.join()
        self.get_angle_thread.join()

        # get loop rate of publishing actual angles and how many matched prior
        query_times = []
        matched_priors = []
        for i in range(1, len(self.angle_queries)):
            time_to_query = self.angle_queries[i].query_time - self.angle_queries[i - 1].query_time
            matched_prior = self.angle_queries[i].angles == self.angle_queries[i - 1].angles
            query_times.append(time_to_query)
            matched_priors.append(matched_prior)
        loop_rate = 1 / (sum(query_times) / len(query_times))
        log_msg(f"{len(self.angle_queries)} joint angle queries, avg loop rate: {loop_rate}")
        log_msg(f"{sum(matched_priors)} matched the prior joint angle, {sum(matched_priors) / len(self.angle_queries):.2f}%")

        query_times = []
        matched_priors = []
        for i in range(1, len(self.cmds_sent)):
            time_to_query = self.cmds_sent[i].query_time - self.cmds_sent[i - 1].query_time
            matched_prior = self.cmds_sent[i].angles == self.cmds_sent[i - 1].angles
            query_times.append(time_to_query)
            matched_priors.append(matched_prior)
        loop_rate = 1 / (sum(query_times) / len(query_times))
        log_msg(f"{len(self.cmds_sent)} commands sent, avg loop rate: {loop_rate}")
        log_msg(f"{sum(matched_priors)} matched the command sent, {sum(matched_priors) / len(self.cmds_sent):.2f}%")

def main():
    arm = MycobotTopics()
    arm.main()

if __name__ == "__main__":
    main()