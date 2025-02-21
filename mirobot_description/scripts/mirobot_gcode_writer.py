#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import serial
import wlkatapython  # Assuming usage of this module for Mirobot
from time import sleep
from enum import Enum, auto

# Define Motion and Position Types (from previous example)
class MotionType(Enum):
    FAST_MOTION = 0
    LINEAR_MOTION = 1
    JUMP_TRAJECTORY_MOVEMENT = 2

class PositionType(Enum):
    ABSOLUTE_VALUE_MOTION = 0


class MirobotWriteNode(Node):
    def __init__(self):
        super().__init__('mirobot_write_node')
        
        # Declaring and getting parameters
        self.declare_parameter('joint_states_topic_name', 'joint_states')
        self.joint_states_topic_name = self.get_parameter('joint_states_topic_name').get_parameter_value().string_value
        
        self.declare_parameter('port_name', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.port_name = self.get_parameter('port_name').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        self.get_logger().info(f'Joint States Topic Name: {self.joint_states_topic_name}')
        self.get_logger().info(f'Port Name: {self.port_name}')
        self.get_logger().info(f'Baudrate: {self.baud_rate}')
        
        self.js_sub = self.create_subscription(
            JointState,
            self.joint_states_topic_name,
            self.joint_state_callback,
            10)
        
        # Setup serial connection
        self.setup_serial()
        
    def setup_serial(self):
        try:
            self.serial_connection = serial.Serial(
                port=self.port_name, 
                baudrate=self.baud_rate, 
                timeout=1
            )
            self.mirobot = wlkatapython.Mirobot_UART()
            self.mirobot.init(self.serial_connection, -1)

            self.get_logger().info("Serial port opened and Mirobot initialized.")
            self.home_mirobot()
        except serial.SerialException as e:
            self.get_logger().fatal(f"Unable to open port: {e}")
            raise

    def home_mirobot(self):
        self.get_logger().info("Starting homing sequence...")
        self.mirobot.restart()
        sleep(5)
        self.mirobot.homing()
        sleep(15)  # Time for homing—adjust as needed
        self.mirobot.zero()
        self.get_logger().warn("Homing done!")

    def joint_state_callback(self, msg: JointState):
        if not self.serial_connection or not self.serial_connection.is_open:
            self.get_logger().error("Serial connection is not open!")
            return
        
        # check message length. We should recieve 6 joint states 
        if len(msg.position) < 6:
            self.get_logger().error("Not enough joint states received!")
            return
        # convert to radians
        angles = [round(pos * 57.296, 2) for pos in msg.position[:6]]

        # Send angles to Mirobot
        self.mirobot.writeangle(0, angles[0], angles[1], angles[2], angles[3], angles[4], angles[5])



def main(args=None):
    rclpy.init(args=args)
    mirobot_gcode_write_node = MirobotWriteNode()

    rclpy.spin(mirobot_gcode_write_node)
    mirobot_gcode_write_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()