import rclpy
import can
import os
import time


from rclpy.node import Node

from stewart_interfaces.msg import PosVel


class CAN_Tx_Rx(Node):

    def __init__(self):
        super().__init__('can_tx_rx')

        os.system("sudo /sbin/ip link set can0 up type can bitrate 500000")
        self.get_logger().info('CAN-bus up')
        time.sleep(0.1)

        try:
            bus = can.interface.Bus(channel='can0', bustype='socketcan_native') 
        except OSError:
            self.get_logger().error('Cannot find PiCAN board!')
            exit()

        start = can.Message(arbitration_id=0xB0,data=[4],extended_id=False)
        bus.send(start)
        
        self.subscription = self.create_subscription(
            PosVel,
            'setpoint',
            self.listener_callback,
            10)
        
        self.publisher_ = self.create_publisher(
            PosVel,
            'feedback',
            10)


    def __del__(self):
        os.system("sudo /sbin/ip link set can0 down")
        self.get_logger().info('CAN-bus down')


    def conv_pos_out( input ):
        out_max = 255.0
        in_min = 0.0
        in_max = 0.09
        out_pos = (input - in_min) * (out_max / (in_max - in_min))

        return int(out_pos + 0.5)


    def conv_vel_out( input ):
            out_max = 255.0
            in_min = -0.06
            in_max = 0.06
            out_pos = (input - in_min) * (out_max / (in_max - in_min))

            return int(out_pos + 0.5)


    def listener_callback(self, msg):
        self.get_logger().info('Setpoints received')

        pos_tx = can.Message(arbitration_id=0xA0,data=[
            self.conv_pos_out(msg.position[0]),
            self.conv_pos_out(msg.position[1]),
            self.conv_pos_out(msg.position[2]),
            self.conv_pos_out(msg.position[3]),
            self.conv_pos_out(msg.position[4]),
            self.conv_pos_out(msg.position[5]),0], extended_id=False)
        
        vel_tx = can.Message(arbitration_id=0xA1,data=[
            self.conv_vel_out(msg.velocity[0]),
            self.conv_vel_out(msg.velocity[1]),
            self.conv_vel_out(msg.velocity[2]),
            self.conv_vel_out(msg.velocity[3]),
            self.conv_vel_out(msg.velocity[4]),
            self.conv_vel_out(msg.velocity[5]),0],extended_id=False)
        
        bus.send(pos_tx)
        bus.send(vel_tx)

        self.get_logger().info('CAN msgs transmitted')

        pub_msg = PosVel()

        rx_msg = bus.recv(0.002)
        
        if rx_msg is None:
            self.get_logger().error('No feedback msg on bus received')
        
        elif rx_msg.arbitration_id == 0xF0:
            pub_msg.position[0] = rx_msg.data[0]
            pub_msg.position[1] = rx_msg.data[1]
            pub_msg.position[2] = rx_msg.data[2]
            pub_msg.position[3] = rx_msg.data[3]
            pub_msg.position[4] = rx_msg.data[4]
            pub_msg.position[5] = rx_msg.data[5]
            self.get_logger().info('Position feedback received')
        
        elif rx_msg.arbitration_id == 0xF1:
            pub_msg.velocity[0] = rx_msg.data[0]
            pub_msg.velocity[1] = rx_msg.data[1]
            pub_msg.velocity[2] = rx_msg.data[2]
            pub_msg.velocity[3] = rx_msg.data[3]
            pub_msg.velocity[4] = rx_msg.data[4]
            pub_msg.velocity[5] = rx_msg.data[5]
        
        self.publisher_.publish(pub_msg)


def main(args=None):
    rclpy.init(args=args)

    can_tx_rx = CAN_Tx_Rx()

    rclpy.spin(can_tx_rx)

    can_tx_rx.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()