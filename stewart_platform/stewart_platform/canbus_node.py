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
            self.bus = can.interface.Bus(channel='can0', bustype='socketcan_native') 
        except OSError:
            self.get_logger().error('Cannot find PiCAN board!')
            exit()
        
        self.subscription = self.create_subscription(
            PosVel,
            'setpoint',
            self.listener_callback,
            10)
        
        self.publisher_ = self.create_publisher(
            PosVel,
            'feedback',
            10)

        self.pos = can.Message(arbitration_id=0xA0, data=[127, 127, 127, 127, 127, 127], extended_id=False)
        self.vel = can.Message(arbitration_id=0xA1, data=[127, 127, 127, 127, 127, 127], extended_id=False)

        self.task_pos = can.send_periodic(self.bus, self.pos, 0.01)
        self.task_vel = can.send_periodic(self.bus, self.vel, 0.01)


    def __del__(self):
        os.system("sudo /sbin/ip link set can0 down")
        self.get_logger().info('CAN-bus down')


    def conv_pos_out(self, input):
        out_max = 255.0
        in_min = 0.0
        in_max = 0.09
        out_pos = (input - in_min) * (out_max / (in_max - in_min))

        return int(out_pos + 0.5)


    def conv_vel_out(self, input):
            out_max = 255.0
            in_min = -0.06
            in_max = 0.06
            out_pos = (input - in_min) * (out_max / (in_max - in_min))

            return int(out_pos + 0.5)


    def listener_callback(self, msg):
        self.get_logger().info('Setpoints received')

        self.pos.data=[
            self.conv_pos_out(msg.position_1),
            self.conv_pos_out(msg.position_2),
            self.conv_pos_out(msg.position_3),
            self.conv_pos_out(msg.position_4),
            self.conv_pos_out(msg.position_5),
            self.conv_pos_out(msg.position_6)]
        
        self.vel.data=[
            self.conv_vel_out(msg.velocity_1),
            self.conv_vel_out(msg.velocity_2),
            self.conv_vel_out(msg.velocity_3),
            self.conv_vel_out(msg.velocity_4),
            self.conv_vel_out(msg.velocity_5),
            self.conv_vel_out(msg.velocity_6)]
        
        self.task_pos.modify_data(pos)
        self.task_vel.modify_data(vel)

        self.get_logger().info('CAN msgs transmitted')

        pub_msg = PosVel()

        rx_msg = self.bus.recv(0.002)
        
        if rx_msg is None:
            self.get_logger().error('No feedback msg on bus received')
        
        elif rx_msg.arbitration_id == 0xF0:
            pub_msg.position_1 = float(rx_msg.data[0])
            pub_msg.position_2 = float(rx_msg.data[1])
            pub_msg.position_3 = float(rx_msg.data[2])
            pub_msg.position_4 = float(rx_msg.data[3])
            pub_msg.position_5 = float(rx_msg.data[4])
            pub_msg.position_6 = float(rx_msg.data[5])
            self.get_logger().info('Position feedback received')
        
        elif rx_msg.arbitration_id == 0xF1:
            pub_msg.velocity_1 = float(rx_msg.data[0])
            pub_msg.velocity_2 = float(rx_msg.data[1])
            pub_msg.velocity_3 = float(rx_msg.data[2])
            pub_msg.velocity_4 = float(rx_msg.data[3])
            pub_msg.velocity_5 = float(rx_msg.data[4])
            pub_msg.velocity_6 = float(rx_msg.data[5])
        
        self.publisher_.publish(pub_msg)


def main(args=None):
    rclpy.init(args=args)

    can_tx_rx = CAN_Tx_Rx()

    rclpy.spin(can_tx_rx)

    can_tx_rx.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()