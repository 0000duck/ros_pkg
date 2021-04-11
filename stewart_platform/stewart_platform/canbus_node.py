import rclpy
import can
import os
import time
import numpy as np

from rclpy.node import Node

from stewart_interfaces.msg import PosVel, StartStop
from stewart_platform import inverse_kinematics as ik


class MessageHandler(self):
    def __init__(self):
        try:
            self.bus = can.interface.Bus(channel='can0', bustype='socketcan_native') 
        except OSError:
            self.get_logger().error('Cannot find PiCAN board!')
            exit()
        
        self.pos_frame = can.Message(arbitration_id=0xA0, data=[128, 128, 128, 128, 128, 128], extended_id=False)
        self.vel_frame = can.Message(arbitration_id=0xA1, data=[128, 128, 128, 128, 128, 128], extended_id=False)
        self.start_frame = can.Message(arbitration_id=0xB0, data=[4], extended_id=False)
        self.stop_frame = can.Message(arbitration_id=0xB0, data=[3], extended_id=False)

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



class CANbus(Node):

    def __init__(self):
        super().__init__('can_tx_rx')

        os.system("sudo /sbin/ip link set can0 up type can bitrate 500000")
        self.get_logger().info('CAN-bus up: baudrate 500k')
        time.sleep(0.1)
        
        self.feedback_pub = self.create_publisher(
            PosVel,
            'leg_feedback',
            10)

        self.start_stop_sub = self.create_subscription(
            StartStop,
            'start_stop',
            self.start_stop_callback,
            10)

        self.msg_handler = MessageHandler()

        self.prev_length = np.zeros(6)
        

    def __del__(self):
        os.system("sudo /sbin/ip link set can0 down")
        self.get_logger().info('CAN-bus down')


    def start_stop_callback(self, msg):
        if msg.run == True:
            self.dof_sub = self.create_subscription(
                DOF,
                'dof_setpoint',
                self.setpoint_callback,
                10)
            
            self.leg_pub = self.create_publisher(
                PosVel,
                'leg_setpoint',
                10)

            self.get_logger().info('Starting CAN communication with Arduino at 100Hz')

            self.msg_handler.bus.send(self.msg_handler.start_frame)
            self.task_pos = can.send_periodic(self.msg_handler.bus, self.msg_handler.pos_frame, 0.01)
            self.task_vel = can.send_periodic(self.msg_handler.bus, self.msg_handler.vel_frame, 0.01)

            self.get_logger().info('Ready for setpoints')

        else:
            self.destroy_subscription(self.dof_sub)
            self.get_logger().info('Unsubscribed to setpoint topic')

            self.task_pos.stop()
            self.task_vel.stop()
            self.get_logger().info('Stopped CAN communication with Arduino')


    def setpoint_callback(self, msg):

        leg_lenght = ik.calc_kinematics(msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw)

        self.pos_frame.data=[
            self.msg_handler.conv_pos_out(leg_lenght[0]),
            self.msg_handler.conv_pos_out(leg_lenght[1]),
            self.msg_handler.conv_pos_out(leg_lenght[2]),
            self.msg_handler.conv_pos_out(leg_lenght[3]),
            self.msg_handler.conv_pos_out(leg_lenght[4]),
            self.msg_handler.conv_pos_out(leg_lenght[5])]
        
        self.vel_frame.data=[
            self.msg_handler.conv_vel_out((leg_lenght[0] - self.prev_length[0]) / 0.01),
            self.msg_handler.conv_vel_out((leg_lenght[1] - self.prev_length[1]) / 0.01),
            self.msg_handler.conv_vel_out((leg_lenght[2] - self.prev_length[2]) / 0.01),
            self.msg_handler.conv_vel_out((leg_lenght[3] - self.prev_length[3]) / 0.01),
            self.msg_handler.conv_vel_out((leg_lenght[4] - self.prev_length[4]) / 0.01),
            self.msg_handler.conv_vel_out((leg_lenght[5] - self.prev_length[5]) / 0.01)]
        
        self.task_pos.modify_data(self.pos_frame)
        self.task_vel.modify_data(self.vel_frame)

        self.prev_length = leg_lenght
        
        self.leg_pub.publish(msg)


        leg_msg = PosVel()

        rx_msg = self.bus.recv(0.002)
        
        if rx_msg is None:
            self.get_logger().error('No leg_msg msg on bus received')
        
        if rx_msg.arbitration_id == 0xF0:
            leg_msg.position_1 = float(rx_msg.data[0])
            leg_msg.position_2 = float(rx_msg.data[1])
            leg_msg.position_3 = float(rx_msg.data[2])
            leg_msg.position_4 = float(rx_msg.data[3])
            leg_msg.position_5 = float(rx_msg.data[4])
            leg_msg.position_6 = float(rx_msg.data[5])
            self.get_logger().info('Position leg_msg received')
        
        rx_msg = self.bus.recv(0.002)

        if rx_msg.arbitration_id == 0xF1:
            leg_msg.velocity_1 = float(rx_msg.data[0])
            leg_msg.velocity_2 = float(rx_msg.data[1])
            leg_msg.velocity_3 = float(rx_msg.data[2])
            leg_msg.velocity_4 = float(rx_msg.data[3])
            leg_msg.velocity_5 = float(rx_msg.data[4])
            leg_msg.velocity_6 = float(rx_msg.data[5])
        
        self.feedback_pub.publish(leg_msg)


def main(args=None):
    rclpy.init(args=args)

    can_tx_rx = CANbus()

    rclpy.spin(can_tx_rx)

    can_tx_rx.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()