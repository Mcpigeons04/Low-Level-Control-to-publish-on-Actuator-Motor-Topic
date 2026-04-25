#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import ActuatorMotors
from px4_msgs.msg import VehicleCommand, OffboardControlMode

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from drone_lowlevel_ctrl.controllers.position_controller import PositionController
from drone_lowlevel_ctrl.mixer.mixer import Mixer
from drone_lowlevel_ctrl.utils.math_utils import quaternion_to_euler
from drone_lowlevel_ctrl.controllers.attitude_controller import AttitudeController
from drone_lowlevel_ctrl.controllers.velocity_controller import VelocityController
from drone_lowlevel_ctrl.controllers.acceleration_controller import AccelerationController
from drone_lowlevel_ctrl.controllers.rate_controller import RateController

class ControllerNode(Node):
    def __init__(self):
        super().__init__('lowlevel_controller')
        self.roll=0.0
        self.pitch=0.0
        self.yaw=0.0
        self.vel_controller= VelocityController()
        self.acc_controller= AccelerationController()
        self.att_controller = AttitudeController()
        self.rate_controller = RateController()
        self.angular_velocity= [0.0,0.0,0.0]

        qos= QoSProfile(
            reliability= ReliabilityPolicy.BEST_EFFORT,
            history= HistoryPolicy.KEEP_LAST,
            depth =10
        )

        self.current_z=0.0
        self.current_velocity=[0.0,0.0,0.0]
        
        self.controller=PositionController()
        self.mixer= Mixer()

        self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            qos
        )

        
        self.motor_pub= self.create_publisher(
            ActuatorMotors,
            '/fmu/in/actuator_motors',
            10
        )

        self.cmd_pub=self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )
        self.offboard_pub= self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )

        
        self.create_timer(0.02,self.publish_offboard)
        self.create_timer(3.0, self.arm)
        self.create_timer(0.01, self.control_loop)

    def odom_callback(self,msg):
        self.current_z= msg.position[2]
        self.current_velocity= msg.velocity
        self.roll, self.pitch, self.yaw = quaternion_to_euler(msg.q)
        self.angular_velocity = msg.angular_velocity

    def publish_offboard(self):
        msg= OffboardControlMode()

        msg.position= False
        msg.velocity=False
        msg.acceleration=False
        msg.attitude=False
        msg.direct_actuator= True

        msg.timestamp= int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    def arm(self):
        msg= VehicleCommand()
        msg.command= VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM 
        msg.param1= 1.0
        msg.target_system= 1
        msg.target_component= 1
        msg.source_system= 1
        msg.source_component= 1
        msg.from_external= True

        msg.timestamp= int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(msg)

    def control_loop(self):
        msg=ActuatorMotors()
        target_z= -1.0

        #convreting position to velocity parameter
        v_des= self.controller.compute(target_z, self.current_z)

        #Velocity control
        a_des= self.vel_controller.compute(v_des, self.current_velocity[2])
        
        #Acceleration control
        roll_des, pitch_des, thrust= self.acc_controller.compute(0.0, 0.0, a_des)

        #Attitude control
        roll_cmd, pitch_cmd, yaw_cmd= self.att_controller.compute(roll_des, pitch_des, self.roll, self.pitch)


        #Rate controller
        roll_cmd, pitch_cmd, yaw_cmd= self.rate_controller.compute(roll_cmd,pitch_cmd, yaw_cmd, self.angular_velocity)

        #Mixer 
        motors = self.mixer.mix(thrust, roll_cmd, pitch_cmd, yaw_cmd)
        
        msg.control= motors

        msg.timestamp= int(self.get_clock().now().nanoseconds / 1000)
        self.motor_pub.publish(msg)
        self.get_logger().info(f"thrust: {thrust}, z:{self.current_z}")

def main(args=None):
    rclpy.init(args=args)
    node=ControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()






