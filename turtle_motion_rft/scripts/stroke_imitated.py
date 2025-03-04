#!/usr/bin/python3
"""
Node to simulate IL based motion with eeUV_sim
@author: Yuya Hamamatsu 
@contact: yuya.hamamatsu@taltech.ee
"""

import numpy as np

import rclpy
import time
from rclpy.node import Node

from eeuv_sim_interfaces.msg import Flippers, Flipper, FlippersFeedbacks, FlippersFeedback
from std_msgs.msg import Float32, Float32MultiArray


class CrawlImitated(Node):
    def __init__(self):
        super().__init__('crawl_imitated')
        self.get_logger().info('CrawlImitated node has been started')

        self.declare_parameter('with_kick', True)
        self.declare_parameter('model_path', 'dummy')

        self.flippers_msg = Flippers() # msg containing array of flippers
        self.flippers_msg.flippers = [Flipper(), Flipper(), Flipper(), Flipper()]

        self.create_subscription(FlippersFeedback, '/hw/flippers_feedback', self.flippers_feedback_callback, 10)

        self.with_kick = self.get_parameter('with_kick').value
        self.model_path = self.get_parameter('model_path').value

        self.get_logger().info('#####Parameters#####')
        self.get_logger().info('with_kick: {}'.format(self.with_kick))
        self.get_logger().info('model_path: {}'.format(self.model_path))
        self.get_logger().info('####################')

        self.buffer = 0.4

        self.reversed_rotation = [-1, 1, -1, 1]
        self.phase_offset_rad = [3.1415 + self.buffer, 3.1415, 3.1415, 3.1415 + self.buffer]
        self.PIDF_gains = [2.5, 0.02, 0.1, 6.1]
        self.F_gain_offset = 5.0
        self.integral = [[0.0], [0.0], [0.0], [0.0]]
        self.prev_error = [0.0, 0.0, 0.0, 0.0]

        self.bata = 2

        self.flippers_encoder_offset = [0, 0, 0, 0]
        self.flippers_o = [0, 0, 0, 0]
        self.feedback_pos_list = [[], [], [], []]
        self.feedback_time_list = [[], [], [], []]
        self.vel_cal_window = 4

        self.stop_time = 0.8
        self.cmd_hz = 20
        self.stop_datapoint = int(self.stop_time * self.cmd_hz)

        self.base_left_cycle_time = 2.5
        self.base_right_cycle_time = 2.5
        self.cycle_time = [self.base_left_cycle_time, 5.0, 5.0, self.base_right_cycle_time]
        self.cycle_time_latest = [self.base_left_cycle_time, 5.0, 5.0, self.base_right_cycle_time]

        self.angular_vel_list = [[0.0], [0.0], [0.0], [0.0]]
        self.predicted_phases_list = [[0.0], [0.0], [0.0], [0.0]]
        self.current_phase = [0, 0, 0, 0]
        self.cycles = [0, 0, 0, 0]
        
        self.flippers_pub = self.create_publisher(Flippers,'/hw/flippers_cmd', 10)
        self.left_flipper_error_pub = self.create_publisher(Float32, '/hw/left_flipper_error', 10)
        self.right_flipper_error_pub = self.create_publisher(Float32, '/hw/right_flipper_error', 10)
        
        self.reset_angle()

        # Initiate tf after reset the angle
        import tensorflow as tf
        print("TensorFlow version:", tf.__version__)

        self.model = tf.keras.models.load_model(self.model_path)
        self.get_logger().info('Model has been loaded')

        for i in range(4):
            if i == 0 or i == 3:
                angular_vel_list, predicted_phases_list = self.predict_angularvelocity_trajectory(self.cycle_time[i], self.cmd_hz)
                self.cycle_time_latest[i] = self.cycle_time[i]
                self.angular_vel_list[i] = angular_vel_list
                self.predicted_phases_list[i] = predicted_phases_list
        time.sleep(1)

        self.left_flipper_kick = Flipper()
        self.right_flipper_kick = Flipper()
        self.create_subscription(Flipper, "hw/left_flipper", self.left_flipper_callback, 10)
        self.create_subscription(Flipper, "hw/right_flipper", self.right_flipper_callback, 10)
        self.create_timer(1/self.cmd_hz, self.fin_timer_callback)


    def normalize_angle(self, a):
        """
        Function to normalize an angle between -Pi and Pi
        @param: self
        @param: a [radians]- angle in
        @result: returns normalized angle
        """
        return (a + np.pi) % (2 * np.pi) - np.pi
    

    def left_flipper_callback(self, msg):
        self.left_flipper_kick.amplitude = msg.amplitude
        self.left_flipper_kick.frequency = msg.frequency
        self.left_flipper_kick.zero_direction = -msg.zero_direction

    
    def right_flipper_callback(self, msg):
        self.right_flipper_kick.amplitude = msg.amplitude
        self.right_flipper_kick.frequency = msg.frequency
        self.right_flipper_kick.zero_direction = msg.zero_direction


    def left_crawl_callback(self, msg):
        self.cycle_time[0] = self.base_left_cycle_time - msg.data
    

    def right_crawl_callback(self, msg):
        self.cycle_time[3] = self.base_right_cycle_time - msg.data
        

    def flippers_feedback_callback(self, msg):
        self.flippers_feedback = msg.flippers_feedback        
        for i in range(4):
            self.feedback_pos_list[i].append(self.flippers_feedback[i].position)
            self.feedback_time_list[i].append(self.flippers_feedback[i].header.stamp.sec + self.flippers_feedback[i].header.stamp.nanosec * 1e-9)
            if len(self.feedback_pos_list[i]) > self.vel_cal_window:
                self.feedback_pos_list[i].pop(0)
                self.feedback_time_list[i].pop(0)


    def reset_angle(self):
        for i in range(4):
            if i == 0 or i == 3:
                self.flippers_msg.flippers[i].zero_direction = self.phase_offset_rad[i]
                self.flippers_msg.flippers[i].amplitude = 0.0
                self.flippers_msg.flippers[i].frequency = 5.0
                print("initial angle")
                print(self.phase_offset_rad[i])
            else:
                self.flippers_msg.flippers[i].zero_direction = self.phase_offset_rad[i]  
                self.flippers_msg.flippers[i].amplitude = 0.0
                self.flippers_msg.flippers[i].frequency = 5.0
        
        self.flippers_pub.publish(self.flippers_msg) 
        time.sleep(1)



    def predict_angularvelocity_trajectory(self, cycle_time, cmd_hz):
        normalized_times = np.linspace(0, 1, int(cycle_time * cmd_hz))
        # Load the model
        predicted_sin_cos = self.model.predict(normalized_times)

        predicted_phases = np.arctan2(predicted_sin_cos[:, 0], predicted_sin_cos[:, 1])
        predicted_phases = np.mod(predicted_phases, 2.0 * np.pi)

        zero_array = np.zeros(self.stop_datapoint)
        predicted_phases = np.concatenate([zero_array, predicted_phases])

        for i in range(1, len(predicted_phases)):
            if i > len(predicted_phases) //1.5:
                if predicted_phases[i] < 1.0:
                    predicted_phases[i] = 2 * np.pi

        predicted_phases = np.concatenate([[0], predicted_phases, [2 * np.pi]])

        phase_differences = np.diff(predicted_phases)
        phase_differences = np.mod(phase_differences + np.pi, 2 * np.pi) - np.pi

        time_intervals = cycle_time / (len(predicted_phases) - 1)

        angular_velocity = phase_differences / time_intervals
        angular_velocity = np.convolve(angular_velocity, np.ones(5) / 5, mode='same')
  
        print(predicted_phases)
        print(angular_velocity)
        predicted_phases_list = predicted_phases

        angular_vel_list = angular_velocity

        return angular_vel_list, predicted_phases_list


    def flippers_feedback_callback(self, msg):
        self.flippers_feedback = msg.flippers_feedback
        self.calc_current_velocity()


    def calc_current_velocity(self):
        current_vel_list = []
        for i in range(4):
            if len(self.feedback_pos_list[i]) > 1:
                current_vel = (self.feedback_pos_list[i][-1] - self.feedback_pos_list[i][-2]) / (self.feedback_time_list[i][-1] - self.feedback_time_list[i][-2])
                current_vel_list.append(current_vel)
            else:
                current_vel_list.append(0.0)
        

    def feedback_and_feedforward(self, p, current_p, v, i):
        P = self.PIDF_gains[0]
        I = self.PIDF_gains[1]
        D = self.PIDF_gains[2]
        F = self.PIDF_gains[3]
        #F = F /self.cycle_time_latest[i]
        current_p_rad = abs(current_p) 
        if i == 3:
            cycle = self.cycles[i]
        else:
            cycle = self.cycles[i] - 1
        p = p + 2 * np.pi * cycle + abs(self.flippers_encoder_offset[i]) * 1.00021
        error = p - current_p_rad + 1.4 #
        print("cycle: " , self.cycles)
        print("motor number: ", i)
        print("current_p_rad: ", current_p_rad)
        print("next_p: ", p)
        print("error: ", error)
        integral = sum(self.integral[i])
        pid_command = P * error + I * integral + D * (error - self.prev_error[i])
        pidf_command_rad = pid_command + F * v
        self.integral[i].append(error)
        self.prev_error[i] = error
        if len(self.integral[i]) > 50:
            self.integral[i].pop(0)

        step_vel = pidf_command_rad / (2 * np.pi) * 513.36

        print("pidf_command_rad: ", pidf_command_rad)

        if i == 0:
            self.left_flipper_error_pub.publish(Float32(data=error))
        elif i == 3:
            self.right_flipper_error_pub.publish(Float32(data=error))

        return step_vel


    def fin_timer_callback(self):
        print(self.current_phase)
        self.flippers_msg.header.stamp = self.get_clock().now().to_msg()
        for i in range(4):
            if i == 0 or i == 3:
                p, _v, change_next_cycle = self.generate_cmd(self.angular_vel_list[i], self.current_phase[i], self.predicted_phases_list[i], i)
                p = self.normalize_angle(p)
                self.flippers_msg.flippers[i].amplitude = 0.0 # practical value
                self.flippers_msg.flippers[i].zero_direction = p + self.phase_offset_rad[i]
                if change_next_cycle:
                    self.cycles[i] += 1
            elif i == 1 and self.with_kick:
                self.flippers_msg.flippers[i].amplitude = self.left_flipper_kick.amplitude
                self.flippers_msg.flippers[i].frequency = self.left_flipper_kick.frequency
                self.flippers_msg.flippers[i].zero_direction = self.left_flipper_kick.zero_direction
            elif i == 2 and self.with_kick:
                self.flippers_msg.flippers[i].amplitude = self.right_flipper_kick.amplitude
                self.flippers_msg.flippers[i].frequency = self.right_flipper_kick.frequency
                self.flippers_msg.flippers[i].zero_direction = self.right_flipper_kick.zero_direction
         
        self.flippers_pub.publish(self.flippers_msg)

        
    def generate_cmd(self, angular_vel_list, current_phase, predicted_phases_list, motor_number):
        #print(phase)
        if len(angular_vel_list) > current_phase:
            v = angular_vel_list[current_phase]
            p = predicted_phases_list[current_phase]
            current_phase += 1
            change_next_cycle = False
        else:
            v = angular_vel_list[-1]
            p = 2* np.pi
            self.get_logger().info("Over the angular velocity list")
            angular_vel_list, predicted_phases_list = self.predict_angularvelocity_trajectory(self.cycle_time[motor_number], self.cmd_hz)
            self.angular_vel_list[motor_number] = angular_vel_list
            self.predicted_phases_list[motor_number] = predicted_phases_list
            current_phase = 0
            change_next_cycle = True
            
        next_angle = p
        next_velocity = v

        self.current_phase[motor_number] = current_phase

        return next_angle, next_velocity, change_next_cycle

def main(args=None):
    rclpy.init(args=args)

    crawl_imitated = CrawlImitated()

    rclpy.spin(crawl_imitated)

    crawl_imitated.destroy_node()
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()