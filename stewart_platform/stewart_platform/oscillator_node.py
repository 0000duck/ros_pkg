import rclpy
import time
from stewart_platform import calc_kinematics as ck

from rclpy.node import Node
from stewart_interfaces.msg import DOF


class Oscillator(Node):
    def __init__(self):
        super().__init__('oscillator')
        
        self.publisher_ = self.create_publisher(
            DOF,
            'dof_setpoint',
            10)

        self.data = ck.oscillator(0.2, 0.25, 0, 0, 0.025, 5, 5, 0)

        timer_period = 0.01 #seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count = 0

    
    def timer_callback(self):
        msg = DOF()

        msg.x       = self.data[0][self.count]
        msg.y       = self.data[1][self.count]
        msg.z       = self.data[2][self.count]
        msg.roll    = self.data[3][self.count]
        msg.pitch   = self.data[4][self.count]
        msg.yaw     = self.data[5][self.count]

        self.publisher_.publish(msg)
        self.get_logger().info('Send DOF points')
        self.count += 1

        if self.count == ck.no_sample:
            self.count = 0



def main(args=None):
    rclpy.init(args=args)

    oscillator = Oscillator()

    rclpy.spin(oscillator)

    oscillator.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()