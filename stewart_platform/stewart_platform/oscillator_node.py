import rclpy
import time
from math import pi
import numpy as np

from rclpy.node import Node
from stewart_interfaces.msg import DOF


class SineWave():
    def __init__(self):

        self.trans_freq = 0.25
        self.rot_freq = 0.2

        self.surge_A = 0
        self.surge_bias = 0
        self.surge_offset = 0

        self.sway_A = 0
        self.sway_bias = 0
        self.sway_offset = -pi/2

        self.heave_A = 0.02
        self.heave_bias = 0.04853
        self.heave_offset = 0

        self.roll_A = 6
        self.roll_bias = 0
        self.roll_offset = 0

        self.pitch_A = 6
        self.pitch_bias = 0
        self.pitch_offset = -pi/2

        self.yaw_A = 0
        self.yaw_bias = 0
        self.yaw_offset = 0

    
    def calc_ouput(self, t):

        surge_out = self.surge_A * np.sin(2*pi * self.trans_freq * t + self.surge_offset) + self.surge_bias
        sway_out = self.sway_A * np.sin(2*pi * self.trans_freq * t + self.sway_offset) + self.sway_bias
        heave_out = self.heave_A * np.sin(2*pi * self.trans_freq * t + self.heave_offset) + self.heave_bias
        
        roll_out = self.roll_A * (pi/180) * np.sin(2*pi * self.rot_freq * t + self.roll_offset) + self.roll_bias
        pitch_out = self.pitch_A * (pi/180) * np.sin(2*pi * self.rot_freq * t + self.pitch_offset) + self.pitch_bias
        yaw_out = self.yaw_A * (pi/180) * np.sin(2*pi * self.rot_freq * t + self.yaw_offset) + self.yaw_bias

        return np.array([surge_out, sway_out, heave_out, roll_out, pitch_out, yaw_out])



class Oscillator(Node):
    def __init__(self):
        super().__init__('oscillator')

        self.sine_wave = SineWave()

        self.publisher_ = self.create_publisher(
            DOF,
            'dof_setpoint',
            10)
        
        timer_period = 0.01 #seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.t = 0.0


    def timer_callback(self):
        self.t += 0.01
        data = self.sine_wave.calc_ouput(self.t)
        
        msg = DOF()
        msg.x       = data[0]
        msg.y       = data[1]
        msg.z       = data[2]
        msg.roll    = data[3]
        msg.pitch   = data[4]
        msg.yaw     = data[5]

        self.publisher_.publish(msg)



def main(args=None):
    rclpy.init(args=args)

    oscillator = Oscillator()

    rclpy.spin(oscillator)

    oscillator.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()