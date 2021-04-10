import rclpy
from stewart_platform import calc_kinematics as ck

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


    def listener_callback(self, msg):
        self.get_logger().info('DOF received')

        stroke_length = ck.calc_kinematics(msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw)

        pub_msg = PosVel()
        pub_msg.position[0] = stroke_length[0]
        pub_msg.position[1] = stroke_length[1]
        pub_msg.position[2] = stroke_length[2]
        pub_msg.position[3] = stroke_length[3]
        pub_msg.position[4] = stroke_length[4]
        pub_msg.position[5] = stroke_length[5]

        pub_msg.velocity[0] = 0
        pub_msg.velocity[1] = 0
        pub_msg.velocity[2] = 0
        pub_msg.velocity[3] = 0
        pub_msg.velocity[4] = 0
        pub_msg.velocity[5] = 0

        self.get_logger().info('IK calculated')

        self.publisher_.publish(pub_msg)



def main(args=None):
    rclpy.init(args=args)

    ik = IK()

    rclpy.spin(ik)

    ik.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()