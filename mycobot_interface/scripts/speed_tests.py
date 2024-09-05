#!/usr/bin/env python2
from functools import wraps
import pprint
import signal
import sys
from time import time

from pymycobot.mycobot import MyCobot
from pymycobot.error import MyCobotDataException

EXIT_FLAG = False
TIMINGS_DICT = {}

def log_msg(msg):
    print(msg)

def signal_handler(sig, frame):
    global EXIT_FLAG
    EXIT_FLAG = True
    log_msg('You pressed Ctrl+C!')

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

    @timing
    def get_angles(self):
        angles = self.mc.get_angles()
        self.cur_angles = CurRealAngles(angles)
    
    def get_encoders(self):
        encoders = self.mc.get_encoders()

        return [round(angle * (math.pi / 180), 3) for angle in angles]

    def main(self):
        signal.signal(signal.SIGINT, signal_handler)
        while not EXIT_FLAG:
            self.get_angles()
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
