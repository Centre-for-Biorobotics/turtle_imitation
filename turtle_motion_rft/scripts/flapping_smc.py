#!/usr/bin/python3
import numpy as np

import rclpy
import time
from rclpy.node import Node

from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import EntityState
from eeuv_sim_interfaces.msg import Flippers, Flipper, FlippersFeedback
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Float32, Float32MultiArray

from tf_transformations import quaternion_from_euler, euler_from_quaternion


THETA_MIN = 3.14 -np.pi / 4.5
THETA_MAX = 3.14 + np.pi / 4.5

class AUVKickController(Node):
    def __init__(self):
        super().__init__('auv_kick_controller')
        self.dt = 0.10
        self.get_logger().info('AUVKickController has been initialized')
        self.freq = 10
        self.pose = Pose()
        self.twist = Twist()
        self.fin_angles = Float32MultiArray()
        self.fuild_pressure = FluidPressure()
        self.depth = Float32()
        self.left_flippers_msg = Flipper()
        self.right_flippers_msg = Flipper()
        self.mode = "crawl" # "crawl" or "butterfly'
        self.euler_state = [0, 0, 0]

        self.kick_frequency = 1.5
        
        self.current_phase = 0
        self.reversed_rotation = [-1, 1, -1, 1]

        self.phase_offset_rad = [0.0, 3.14, -3.14, 3.1415]
        self.phase_offset_rad = [0.0, 3.14, 3.1415, 0.0]


        self.speed_ratio = 4.0
        self.drift_fixing_param = 0.040 # rad FUCKING SHIT PARAMS SHOULD BE FIXED

        self.state_sub = self.create_subscription(EntityState, "/ucat/state", self.odom_update_callback, 10)
        #self.pressure_sub = self.create_subscription(FluidPressure, "hw/pressure", self.pressure_update_callback, 10)

        self.left_fin_pub = self.create_publisher(Flipper, "/hw/left_flipper", 10)
        self.right_fin_pub = self.create_publisher(Flipper, "/hw/right_flipper", 10)
        
        # Sliding mode control params
        self.previous_depth = 0
        self.desired_depth = -5.0
        self.updating_threshold = 20 
        self.lambda_param = 1.5
        self.K_u = 1.0
        self.K_theta = 0.3245 # MAX rad / s
        self.K1 = 2.1
        self.K2 = 0.985
        self.K3 = 0.3217

        self.u = 0.0
        self.d_theta = 0
        self.desired_pitch = 0
        self.theta = 3.14
        self.epsilon = 0.07
        self.offset_angle = 0.03334412 + 3.14

        # PID params
        self.pid_params_depth = [2.0, 0.1, 0.1]
        self.pid_params_pitch = [1.0, 0.1, 0.1]
        self.e_depth = 0
        self.e_pitch = 0
        self.offset_x = 0
        self.offset_y = 0
        self.offset_z = 0
        self.offset_w = 0
        
        self.create_timer(1/self.freq, self.kick_timer_callback)

        self.use_SM = True

        print("----------------------")
        print(self.lambda_param)
        print(self.K_theta)
        print(self.K_u)
        print(self.K1)
        print(self.K2)
        print(self.K3)
        print(self.offset_angle) 
        print("---------------------")

        self.debug_depth_offset = 0 # Should be zero!
    
    
    def odom_update_callback(self, msg):
        self.state = msg.pose
        orientation_euler = euler_from_quaternion([self.state.orientation.x, self.state.orientation.y, self.state.orientation.z, self.state.orientation.w])
        #orientation_euler = [self.normalize_angle((orientation_euler[0] + np.pi)), orientation_euler[1] * (-1), orientation_euler[2]]  # New Y
        self.euler_state = orientation_euler
        quaternion = quaternion_from_euler(orientation_euler[0], orientation_euler[1], orientation_euler[2])
        self.pose.orientation.x = quaternion[0] # - self.offset_x 
        self.pose.orientation.y = quaternion[1] #- self.offset_y
        self.pose.orientation.z = quaternion[2] #- self.offset_z
        self.pose.orientation.w = quaternion[3] #- self.offset_w 

        self.depth.data = self.state.position.z #+ self.debug_depth_offset
        print(f"Depth: {self.depth.data}")
        self.depth_speed = msg.twist.linear.z


    def kick_timer_callback(self):
        orientation_euler = self.euler_state
        depth = self.depth.data
        pitch = orientation_euler[1]
        if self.use_SM:
            self.u, self.d_theta = self.sliding_mode_controller_kicking(
                0, depth, self.desired_depth, self.theta, 0, pitch, self.desired_pitch, 1/self.freq)
        else:
            self.u, self.d_theta = self.pid_controller_kicking(depth, self.desired_depth, pitch, self.desired_pitch, 1/self.freq)

        for i in range(2):
            if i == 0:
                self.left_flippers_msg.zero_direction = (self.theta + self.d_theta) * self.reversed_rotation[1] 
                self.left_flippers_msg.amplitude = self.u
                self.left_flippers_msg.frequency = self.kick_frequency
                self.left_flippers_msg.phase_offset = float(self.phase_offset_rad[1])
            else:
                self.right_flippers_msg.zero_direction = (self.theta + self.d_theta) * self.reversed_rotation[2] + 6.28
                self.right_flippers_msg.amplitude = self.u
                self.right_flippers_msg.frequency = self.kick_frequency
                self.right_flippers_msg.phase_offset = float(self.phase_offset_rad[2])
        self.left_fin_pub.publish(self.left_flippers_msg)
        self.right_fin_pub.publish(self.right_flippers_msg)


    def sliding_mode_controller_kicking(self, V, x, x_desired, angle, angle_desired, pitch, pitch_desired, dt):
        s = -(x - x_desired)*self.K1 + (pitch - pitch_desired)* self.K2 + (angle - angle_desired - self.offset_angle)* self.K3  # Sliding surface
        # z error
        print(f"Error: {x - x_desired}")
        theta_vel = np.tanh(self.lambda_param * s) * self.K_theta #* (-1)  
        u = abs(np.tanh(self.lambda_param * s)) * self.K_u  
        d_theta = theta_vel * dt * (-1)
        self.theta = self.theta + d_theta
        if self.theta > THETA_MAX:
            self.theta = THETA_MAX
            d_theta = 0
        elif self.theta < THETA_MIN:
            self.theta = THETA_MIN
            d_theta = 0
        if u < 0.25:
            u = 0.25
        return u, d_theta
    

def main(args=None):
    rclpy.init(args=args)
    auv_kick_controller = AUVKickController()
    rclpy.spin(auv_kick_controller)
    auv_kick_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()