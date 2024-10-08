#!/usr/bin/env python2
from functools import wraps
import pprint
import sys
from time import time

import pymycobot
from pymycobot.mycobot import MyCobot
from pymycobot.error import MyCobotDataException

EXIT_FLAG = False
TIMINGS_DICT = {}

def log_msg(msg):
    print(msg)

def timing(f):
    global TIMINGS_DICT
    @wraps(f)
    def wrap(*args, **kw):
        ts = time()
        result = f(*args, **kw)
        te = time()
        if TIMINGS_DICT.get(f.__name__, None) is None:
            TIMINGS_DICT[f.__name__] = []
        TIMINGS_DICT[f.__name__].append(te-ts)
        log_msg('func:%r args:[%r, %r] took: %2.4f sec' % \
          (f.__name__, args, kw, te-ts))
        return result
    return wrap

class CurRealAngles:
    def __init__(self, angles):
        self.angles = angles

    def __eq__(self, other):
        if isinstance(other, CurRealAngles):
            return self.angles == other.angles
        return False

    def __ne__(self, other):
        # needed in python 2
        return not self.__eq__(other)
    
    def __str__(self):
        joint_str = "joint_"
        debug_str = ""
        for i in range(len(self.angles)):
            debug_str += joint_str + str(i + 1) + ": " + str(self.angles[i]) + "\n"
        debug_str = debug_str[:-1]
        return debug_str


class MycobotTopics(object):

    def __init__(self):
        super(MycobotTopics, self).__init__()

        port = "/dev/ttyAMA0"
        baud = 1000000

        self.mc = MyCobot(port, baud)

        self.cur_angles = CurRealAngles([])
        self.encoders = []
        self.time_to_run = 10

        if not self.mc.is_controller_connected():
            raise RuntimeError("not connected with atom")

    @timing
    def get_angles(self):
        angles = self.mc.get_angles()
        self.cur_angles = CurRealAngles(angles)

    @timing
    def get_encoders(self):
        encoders = self.mc.get_encoders()
        self.encoders = encoders
    
    @timing
    def send_cmds(self):
        self.mc.send_angles([0] * 6, 30)

    def main(self):
        start_time = time()
        while time() - start_time < self.time_to_run:
            self.get_angles()
            self.get_encoders()
            self.send_cmds()


        py_version = sys.version
        log_msg('python_version %r ' % py_version)


        pymycobot_version = pymycobot.__version__
        log_msg('pymycobot_version %r ' % pymycobot_version)

        if int(pymycobot_version[0]) > 2:
            robo_version = self.mc.get_robot_version()
            log_msg('robot_version: %r ' % \
            (robo_version))

            
            sys_version = self.mc.get_system_version()
            log_msg('sys_version: %r ' % \
            (sys_version))

            robo_id = self.mc.get_robot_id()
            log_msg('robo_id: %r ' % \
            (robo_id))


            basic_version = self.mc.get_basic_version()
            log_msg('basic_firmware_version: %r ' % \
            (basic_version))

            atom_version = self.mc.get_atom_version()
            log_msg('atom_version: %r ' % \
            (atom_version))
        
        AVG_TIMING = {}
        NUM_CALLS = {}
        for func in TIMINGS_DICT.keys():
            AVG_TIMING[func] = sum(TIMINGS_DICT[func]) / len(TIMINGS_DICT[func])
            NUM_CALLS[func] = len(TIMINGS_DICT[func])
        log_msg("average timings")
        avg_str = pprint.pformat(AVG_TIMING)
        log_msg(avg_str)
        log_msg("number of calls")
        num_str = pprint.pformat(NUM_CALLS)
        log_msg(num_str)

if __name__ == "__main__":
    mc_topics = MycobotTopics()
    mc_topics.main()
