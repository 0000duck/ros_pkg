import rclpy
from stewart_platform import calc_kinematics as ck
import numpy as np

from rclpy.node import Node
from stewart_interfaces.msg import PosVel, DOF


class IK(Node):
    def __init__(self):
        super().__init__('ik')
    
        self.subscription = self.create_subscription(
            DOF,
            'dof',
            self.listener_callback,
            10)
        
        self.publisher_ = self.create_publisher(
            PosVel,
            'setpoint',
            10)
        
        self.prev_stroke_length = np.zeros(6)


    def listener_callback(self, msg):
        self.get_logger().info('DOF received')

        stroke_length = ck.calc_kinematics(msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw)

        pub_msg = PosVel()
        pub_msg.position_1 = stroke_length[0]
        pub_msg.position_2 = stroke_length[1]
        pub_msg.position_3 = stroke_length[2]
        pub_msg.position_4 = stroke_length[3]
        pub_msg.position_5 = stroke_length[4]
        pub_msg.position_6 = stroke_length[5]

        if self.count == 0:
            pub_msg.velocity_1 = 0.0
            pub_msg.velocity_2 = 0.0
            pub_msg.velocity_3 = 0.0
            pub_msg.velocity_4 = 0.0
            pub_msg.velocity_5 = 0.0
            pub_msg.velocity_6 = 0.0
        else:
            pub_msg.velocity_1 = (stroke_length[0] - prev_stroke_length[0]) / ck.sample_rate
            pub_msg.velocity_2 = (stroke_length[1] - prev_stroke_length[1]) / ck.sample_rate
            pub_msg.velocity_3 = (stroke_length[2] - prev_stroke_length[2]) / ck.sample_rate
            pub_msg.velocity_4 = (stroke_length[3] - prev_stroke_length[3]) / ck.sample_rate
            pub_msg.velocity_5 = (stroke_length[4] - prev_stroke_length[4]) / ck.sample_rate
            pub_msg.velocity_6 = (stroke_length[5] - prev_stroke_length[5]) / ck.sample_rate

        self.publisher_.publish(pub_msg)

        self.prev_stroke_length = stroke_length

        self.get_logger().info('IK calculated')



def main(args=None):
    rclpy.init(args=args)

    ik = IK()

    rclpy.spin(ik)

    ik.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()