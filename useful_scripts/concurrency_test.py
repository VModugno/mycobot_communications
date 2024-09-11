#!/usr/bin/env python3

"""
the goal of this script is to test if we can read/write from the serial port at the same time.
We want to do this so we can have the actual joint angles continuously published at 100HZ or more,
even as we command the robot. We need to check loop rate in HZ, as well as accuracy of joint metrics.
"""

from functools import wraps
import math
import pprint
import sys
import multiprocessing
import threading
import time

import pymycobot
from pymycobot.mycobot import MyCobot
from pymycobot.error import MyCobotDataException

EXIT_FLAG = False
NUM_JOINTS = 6

def log_msg(msg):
    print(msg)

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


        self.use_threading = False

        self.angle_queries = multiprocessing.Queue()

        self.exit = multiprocessing.Event()
        self.get_angles_target_hz = 100
        self.get_angles_target_seconds = 1 / self.get_angles_target_hz
        self.last_get_angles_time = time.time()
        if self.use_threading:
            self.get_angle_worker = threading.Thread(target=self.get_angles, args=(self.exit, self.angle_queries))
            self.mc = MyCobot(port, baud, thread_lock=True) # we can probably try thread lock true/false here
        else:
            self.get_angle_worker = multiprocessing.Process(target=self.command_arm, args=(self.exit, self.angle_queries))
            self.mc = MyCobot(port, baud, thread_lock=False) # we probably cannot try the same here

        self.command_arm_target_hz = 100
        self.command_arm_target_seconds = 1 / self.command_arm_target_hz
        self.last_command_arm_time = time.time()
        self.command_speed = 80
        self.max_angle = 50
        self.cur_counter = 0
        self.counter_incr = 1
        self.cmds_sent = multiprocessing.Queue()
        if self.use_threading:
            self.cmd_worker = threading.Thread(target=self.command_arm, args=(self.exit, self.cmds_sent))
        else:
            self.cmd_worker = multiprocessing.Process(target=self.command_arm, args=(self.exit, self.cmds_sent))


        if not self.mc.is_controller_connected():
            raise RuntimeError("not connected with atom")
        
    def get_angles(self, exit_event, my_q):
        print("getting angles")
        while not exit_event.is_set():
            print(exit_event.is_set(), flush=True)
            time_since_loop = time.time() - self.last_get_angles_time
            if time_since_loop < self.get_angles_target_seconds:
                time.sleep(self.get_angles_target_seconds - time_since_loop)
            self.last_get_angles_time = time.time()
            angles = self.mc.get_angles()
            cur_angles = CurRealAngles(angles, self.last_get_angles_time)
            my_q.put(cur_angles)
        print("done getting angles")
    
    def command_arm(self, exit_event, my_q):
        """Note this process won't exit until the q is drained.

        Args:
            exit_event (_type_): _description_
            my_q (_type_): _description_
        """
        while not exit_event.is_set():
            time_since_loop = time.time() - self.last_command_arm_time
            if time_since_loop < self.command_arm_target_seconds:
                time.sleep(self.command_arm_target_seconds - time_since_loop)
            self.last_command_arm_time = time.time()

            cur_angle = math.sin(self.cur_counter * math.pi / 180) * self.max_angle
            self.cur_counter += self.counter_incr

            cmd = CmdAngles([cur_angle for i in range(NUM_JOINTS)], self.command_speed, self.last_command_arm_time)
            self.mc.send_angles(cmd.angles, cmd.speed)
            my_q.put(cmd)
    
    def set_exit(self):
        log_msg("setting exit")
        self.exit.set()

    def main(self):
        self.mc.send_angles([0] * NUM_JOINTS, 60)
        time.sleep(4)
        self.cmd_worker.start()
        # self.get_angle_worker.start()
        time.sleep(10)
        self.set_exit()

        log_msg("doing metrics")

        # get loop rate of publishing actual angles and how many matched prior
        # query_times = []
        # matched_priors = []
        # last_angles = self.angle_queries.get()
        # while self.angle_queries.qsize() > 0:
        #     new_angles = self.angle_queries.get()
        #     log_msg(new_angles.query_time)
        #     time_to_query = new_angles.query_time - last_angles.query_time
        #     matched_prior = new_angles.angles == last_angles.angles
        #     query_times.append(time_to_query)
        #     matched_priors.append(matched_prior)
        #     last_angles = new_angles
        # loop_rate = 1 / (sum(query_times) / len(query_times))
        # log_msg(f"{len(query_times)} joint angle queries, avg loop rate: {loop_rate}")
        # log_msg(f"{sum(matched_priors)} matched the prior joint angle, {sum(matched_priors) / len(query_times):.2f}%")

        query_times = []
        matched_priors = []
        last_cmd = self.cmds_sent.get()
        while self.cmds_sent.qsize() > 0:
            new_cmd = self.cmds_sent.get()
            log_msg(new_cmd.query_time)
            time_to_query = new_cmd.query_time - last_cmd.query_time
            matched_prior = new_cmd.angles == last_cmd.angles
            query_times.append(time_to_query)
            matched_priors.append(matched_prior)
            last_cmd = new_cmd
        loop_rate = 1 / (sum(query_times) / len(query_times))
        log_msg(f"{len(query_times)} commands sent, avg loop rate: {loop_rate}")
        log_msg(f"{sum(matched_priors)} matched the command sent, {sum(matched_priors) / len(query_times):.2f}%")
    
    
        log_msg("waiting on workers to join")
        self.cmd_worker.join()
        # self.get_angle_worker.join()

def main():
    arm = MycobotTopics()
    arm.main()

if __name__ == "__main__":
    main()