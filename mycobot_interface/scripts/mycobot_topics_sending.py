#!/usr/bin/env python2

import rospy

from mycobot_msgs.msg import (
    MycobotAngles,
    MycobotSetAngles,
    MycobotCoords,
    MycobotGripperStatus,
    MycobotPumpStatus,
)

from pymycobot.mycobot import MyCobot
from pymycobot.error import MyCobotDataException

COBOT_JOINT_GOAL_TOPIC = "mycobot/angles_goal"
COBOT_JOIN_REAL_TOPIC = "mycobot/angles_real"
COBOT_GRIPPER_STATUS_TOPIC = "mycobot/gripper_status"
COBOT_PUMP_STATUS_TOPIC = "mycobot/pump_status"
COBOT_END_EFFECTOR_COORDS_TOPIC = "mycobot/coords_real"


class CurAngles:
    def __init__(self, angles, speed):
        self.angles = angles
        self.speed = speed

    def __eq__(self, other):
        if isinstance(other, CurAngles):
            return self.angles == other.angles and self.speed == other.speed
        return False

    def __ne__(self, other):
        # needed in python 2
        return not self.__eq__(other)


class CurGripperState:
    def __init__(self, state, speed):
        self.state = state
        self.speed = speed

    def __eq__(self, other):
        if isinstance(other, CurGripperState):
            return self.state == other.state and self.speed == other.speed
        return False

    def __ne__(self, other):
        # needed in python 2
        return not self.__eq__(other)


class CurPumpStatus:
    def __init__(self, state, pin_1, pin_2):
        self.state = state
        self.pin_1 = pin_1
        self.pin_2 = pin_2

    def __eq__(self, other):
        if isinstance(other, CurPumpStatus):
            return self.state == other.state and self.pin_1 == other.pin_1 and self.pin_2 == other.pin_2
        return False

    def __ne__(self, other):
        # needed in python 2
        return not self.__eq__(other)


class MycobotTopics(object):

    def __init__(self):
        super(MycobotTopics, self).__init__()

        rospy.init_node("mycobot_topics_sending")
        rospy.loginfo("start ...")
        port = rospy.get_param("~port", "/dev/ttyAMA0")
        baud = rospy.get_param("~baud", 1000000)
        self.publish_real_coords = rospy.get_param("~pub_real_coords", False)
        rospy.loginfo("%s,%s" % (port, baud))
        self.mc = MyCobot(port, baud)
        self.real_angle_pub = rospy.Publisher(
            COBOT_JOIN_REAL_TOPIC, MycobotAngles, queue_size=5)
        self.cmd_angle_sub = rospy.Subscriber(
            COBOT_JOINT_GOAL_TOPIC, MycobotSetAngles, callback=self.cmd_angle_callback
        )
        self.gripper_status_sub = rospy.Subscriber(
            COBOT_GRIPPER_STATUS_TOPIC, MycobotGripperStatus, self.gripper_status_callback)
        self.pump_status_sub = rospy.Subscriber(
            COBOT_PUMP_STATUS_TOPIC, MycobotPumpStatus, callback=self.pump_status_callback)

        if self.publish_real_coords:
            self.real_coords_pub = rospy.Publisher(
                COBOT_END_EFFECTOR_COORDS_TOPIC, MycobotCoords, queue_size=5)

        self.cur_angles = CurAngles([], 0)
        self.prev_angles = CurAngles([], 0)

        self.cur_gripper_state = CurGripperState(0, 0)
        self.prev_gripper_state = CurGripperState(0, 0)

        self.cur_pump_status = CurPumpStatus(0, 0, 0)
        self.prev_pump_status = CurPumpStatus(0, 0, 0)

    def cmd_angle_callback(self, msg):
        cur_cmd_angles = [
            msg.joint_1,
            msg.joint_2,
            msg.joint_3,
            msg.joint_4,
            msg.joint_5,
            msg.joint_6,
        ]
        cur_cmd_speed = int(msg.speed)

        self.cur_angles = CurAngles(cur_cmd_angles, cur_cmd_speed)

    def gripper_status_callback(self, msg):
        state = msg.state
        speed = msg.speed
        self.cur_gripper_state = CurGripperState(state, speed)

    def pump_status_callback(self, msg):
        state = msg.state
        pin_1 = msg.pin_1
        pin_2 = msg.pin_2
        self.cur_pump_status = CurPumpStatus(state, pin_1, pin_2)

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
        try:
            self.mc.send_angles(self.cur_angles.angles, self.cur_angles.speed)
            self.prev_angles = self.cur_angles
            rospy.loginfo("sent cmd angles")
        except MyCobotDataException as err:
            rospy.logerror("invalid joint command. Command was {}, error was {}".format(self.cur_angles.angles, err))
            self.cur_angles = self.prev_angles

    def set_cur_gripper_state(self):
        rospy.loginfo("sending gripper state")
        self.mc.set_gripper_state(
            self.cur_gripper_state.state, self.cur_gripper_state.speed)
        self.prev_gripper_state = self.cur_gripper_state
        rospy.loginfo("sent gripper state")

    def set_cur_pump_status(self):
        rospy.loginfo("sending pump status")
        self.mc.set_basic_output(
            self.cur_pump_status.pin_1, self.cur_pump_status.state)
        self.mc.set_basic_output(
            self.cur_pump_status.pin_2, self.cur_pump_status.state)
        self.prev_pump_status = self.cur_pump_status
        rospy.loginfo("sent pump status")

    def main(self):
        while not rospy.is_shutdown():
            self.get_and_publish_real_angles()
            if self.publish_real_coords:
                self.get_and_publish_real_coords()
            if self.cur_angles != self.prev_angles:
                self.set_cur_cmd_angles()
            if self.cur_gripper_state != self.prev_gripper_state:
                self.set_cur_gripper_state()
            if self.cur_pump_status != self.prev_pump_status:
                self.set_cur_pump_status()


if __name__ == "__main__":
    mc_topics = MycobotTopics()
    mc_topics.main()
