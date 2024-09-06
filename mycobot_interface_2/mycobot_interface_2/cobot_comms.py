import rclpy
from rclpy.node import Node

from mycobot_msgs_2.msg import (
    MycobotAngles,
    MycobotSetAngles,
    MycobotCoords,
    MycobotGripperStatus,
    MycobotPumpStatus,
)

import pymycobot
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

class MycobotController(Node):

    def __init__(self):
        super().__init__('mycobot_controller')

        self.declare_parameter('port', '/dev/ttyAMA0')
        self.declare_parameter('baud', 1000000)
        self.declare_parameter('pub_angle_timer', 0.01)
        self.declare_parameter('pub_real_coords', False)
        self.declare_parameter('pub_real_coords_timer', 0.05)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        
        self.publish_real_coords = self.get_parameter('pub_real_coords').get_parameter_value().bool_value

        self.get_logger().info("start ...")
        self.get_logger().info("Params: %s,%s" % (port, baud))

        pymycobot_version = pymycobot.__version__
        self.get_logger().info("pymycobot version: %s" % (pymycobot_version))

        self.mc = MyCobot(port, baud)

        is_connected = self.mc.is_controller_connected()
        if not is_connected:
            raise RuntimeError("Not connected to the mycobot. Check if the robot is on and the serial port is correct?")

        self.real_angle_pub = self.create_publisher(MycobotAngles, COBOT_JOIN_REAL_TOPIC, 5)

        self.cmd_angle_sub = self.create_subscription(
            MycobotSetAngles,
            COBOT_JOINT_GOAL_TOPIC,
            self.cmd_angle_callback,
            5)
        self.cmd_angle_sub  # prevent unused variable warning


        self.gripper_status_sub = self.create_subscription(
            MycobotGripperStatus,
            COBOT_GRIPPER_STATUS_TOPIC,
            self.gripper_status_callback,
            5)
        self.gripper_status_sub  # prevent unused variable warning
        
        
        self.pump_status_sub = self.create_subscription(
            MycobotPumpStatus,
            COBOT_PUMP_STATUS_TOPIC,
            self.pump_status_callback,
            5)
        self.gripper_status_sub  # prevent unused variable warning
  
        if self.publish_real_coords:
            self.get_logger().info("setting up real coordinate publisher ...")
            self.real_coords_pub = self.create_publisher(MycobotCoords, COBOT_END_EFFECTOR_COORDS_TOPIC, 5)
            self.timer_real_coords = self.create_timer(self.get_parameter('pub_real_coords_timer').value,
                 self.get_and_publish_real_coords)

        self.cur_angles = CurAngles([], 0)
        self.prev_angles = CurAngles([], 0)

        self.cur_gripper_state = CurGripperState(0, 0)
        self.prev_gripper_state = CurGripperState(0, 0)

        self.cur_pump_status = CurPumpStatus(0, 0, 0)
        self.prev_pump_status = CurPumpStatus(0, 0, 0)

        self.timer_real_angles = self.create_timer(self.get_parameter('pub_angle_timer').value,
                 self.get_and_publish_real_angles)


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
        if self.cur_angles != self.prev_angles:
            self.set_cur_cmd_angles()

    def gripper_status_callback(self, msg):
        state = msg.state
        speed = msg.speed
        self.cur_gripper_state = CurGripperState(state, speed)
        if self.cur_gripper_state != self.prev_gripper_state:
            self.set_cur_gripper_state()

    def pump_status_callback(self, msg):
        state = msg.state
        pin_1 = msg.pin_1
        pin_2 = msg.pin_2
        self.cur_pump_status = CurPumpStatus(state, pin_1, pin_2)
        if self.cur_pump_status != self.prev_pump_status:
            self.set_cur_pump_status()

    def get_and_publish_real_angles(self):
        msg = MycobotAngles()
        self.get_logger().debug("reading angles")
        angles = self.mc.get_angles()
        if angles is None or angles[0] == angles[1] == angles[2] == 0.0:
            self.get_logger().error("angles came back None or 0.0, {}".format(angles))
            return
        self.get_logger().debug("read angles")
        msg.joint_1 = angles[0]
        msg.joint_2 = angles[1]
        msg.joint_3 = angles[2]
        msg.joint_4 = angles[3]
        msg.joint_5 = angles[4]
        msg.joint_6 = angles[5]
        self.real_angle_pub.publish(msg)
        self.get_logger().debug("published angles")

    def get_and_publish_real_coords(self):
        msg = MycobotCoords()
        self.get_logger().debug("reading coords")
        coords = self.mc.get_coords()
        self.get_logger().debug("read coords")
        if not coords:
            self.get_logger().error("coords did not come back")
        else:
            msg.x = coords[0]
            msg.y = coords[1]
            msg.z = coords[2]
            msg.rx = coords[3]
            msg.ry = coords[4]
            msg.rz = coords[5]
            self.real_coords_pub.publish(msg)
            self.get_logger().debug("published coords")

    def set_cur_cmd_angles(self):
        self.get_logger().debug("sending cmd angles")
        try:
            self.mc.send_angles(self.cur_angles.angles, self.cur_angles.speed)
            self.prev_angles = self.cur_angles
            self.get_logger().debug("sent cmd angles")
        except MyCobotDataException as err:
            self.get_logger().error("invalid joint command. Command was {}, error was {}".format(self.cur_angles.angles, err))
            self.cur_angles = self.prev_angles

    def set_cur_gripper_state(self):
        self.get_logger().debug("sending gripper state")
        self.mc.set_gripper_state(
            int(self.cur_gripper_state.state), self.cur_gripper_state.speed)
        self.prev_gripper_state = self.cur_gripper_state
        self.get_logger().debug("sent gripper state")

    def set_cur_pump_status(self):
        self.get_logger().debug("sending pump status")
        self.mc.set_basic_output(
            self.cur_pump_status.pin_1, self.cur_pump_status.state)
        self.mc.set_basic_output(
            self.cur_pump_status.pin_2, self.cur_pump_status.state)
        self.prev_pump_status = self.cur_pump_status
        self.get_logger().debug("sent pump status")


def main(args=None):
    rclpy.init(args=args)

    mycobot_controller = MycobotController()

    rclpy.spin(mycobot_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mycobot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
