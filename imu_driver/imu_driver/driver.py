#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64
import serial
from imu_custom_message.msg import IMUmsg
import math
import time


def get_imu_data(imu_data):
    imu_data_str = imu_data.decode('ascii', errors='replace').strip()
    imu_data_array = imu_data_str.strip().split(',')
    
    if any('\x00' in field for field in imu_data_array):
        print(f"Ignoring IMU data due to unwanted characters: {imu_data_str}")
        return None  # Skip this entry
    
    if imu_data_array[0] == "$VNYMR" and len(imu_data_array) >= 13:
        yaw_value = float(imu_data_array[1])
        pitch_value = float(imu_data_array[2])
        roll_value = float(imu_data_array[3])
        magx_value = float(imu_data_array[4])
        magy_value = float(imu_data_array[5])
        magz_value = float(imu_data_array[6])
        accelx_value = float(imu_data_array[7])
        accely_value = float(imu_data_array[8])
        accelz_value = float(imu_data_array[9])
        gyrox_value = float(imu_data_array[10])
        gyroy_value = float(imu_data_array[11])
        raw_gyroz_value = imu_data_array[12].split('*')
        gyroz_value = float(raw_gyroz_value[0])
        separated_imu_data_array = [yaw_value, pitch_value, roll_value, magx_value, magy_value, magz_value, accelx_value, accely_value, accelz_value, gyrox_value, gyroy_value, gyroz_value]
        
        return(separated_imu_data_array)
    
    
def convert_euler_to_quaternion(yaw, pitch, roll):
    
    yaw_in_radians = math.radians(yaw)
    pitch_in_radians = math.radians(pitch)
    roll_in_radians = math.radians(roll)
    
    cos_yaw_over_2= math.cos(yaw_in_radians/2)
    sin_yaw_over_2 = math.sin(yaw_in_radians/2)
    
    cos_pitch_over_2= math.cos(pitch_in_radians/2)
    sin_pitch_over_2 = math.sin(pitch_in_radians/2)
    
    cos_roll_over_2= math.cos(roll_in_radians/2)
    sin_roll_over_2 = math.sin(roll_in_radians/2)
    
    qx = sin_roll_over_2 * cos_pitch_over_2 * cos_yaw_over_2 - cos_roll_over_2 * sin_pitch_over_2 * sin_yaw_over_2
    qy = cos_roll_over_2 * sin_pitch_over_2 * cos_yaw_over_2 + sin_roll_over_2 * cos_pitch_over_2 * sin_yaw_over_2
    qz = cos_roll_over_2 * cos_pitch_over_2 * sin_yaw_over_2 - sin_roll_over_2 * sin_pitch_over_2 * cos_yaw_over_2
    qw = cos_roll_over_2 * cos_pitch_over_2 * cos_yaw_over_2 + sin_roll_over_2 * sin_pitch_over_2 * sin_yaw_over_2
    
    return [qx,qy,qz,qw]
        
        
class MessageDriver(Node):

    def __init__(self):
        super().__init__('message_driver')
        self.declare_parameter('port', '/dev/tty/USB0')
        port = self.get_parameter('port').get_parameter_value().string_value
        self.publisher_ = self.create_publisher(IMUmsg, '/imu', 10)
        self.serial_port = serial.Serial(port, baudrate=115200, timeout=0.95)
        self.configure_imu_frequency()
        timer_period = 0.025  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    
    def configure_imu_frequency(self):
        time.sleep(2)
        
        command = "$VNWRG,07,40*59\r\n"  
        self.serial_port.write(command.encode())
        
        
    def timer_callback(self):
        imu_data = self.serial_port.readline()
        
        if imu_data:
            self.get_logger().info(imu_data)
            separated_imu_data_array = get_imu_data(imu_data)
            
            if separated_imu_data_array != None:
                qx,qy,qz,qw = convert_euler_to_quaternion(separated_imu_data_array[0],separated_imu_data_array[1],separated_imu_data_array[2])
                
                msg = IMUmsg()
                
                msg.header.frame_id = "IMU1_Frame"
                msg.header.stamp = self.get_clock().now().to_msg()
             
                msg.imu.orientation.x = qx
                msg.imu.orientation.y = qy
                msg.imu.orientation.z = qz
                msg.imu.orientation.w = qw
                
                msg.imu.linear_acceleration.x = separated_imu_data_array[6]
                msg.imu.linear_acceleration.y = separated_imu_data_array[7]
                msg.imu.linear_acceleration.z = separated_imu_data_array[8]
                
                msg.imu.angular_velocity.x = separated_imu_data_array[9]
                msg.imu.angular_velocity.y = separated_imu_data_array[10]
                msg.imu.angular_velocity.z = separated_imu_data_array[11]
                
                msg.mag_field.magnetic_field.x = separated_imu_data_array[3] * 1e-4
                msg.mag_field.magnetic_field.y = separated_imu_data_array[4] * 1e-4
                msg.mag_field.magnetic_field.z = separated_imu_data_array[5] * 1e-4
                
                msg.raw_data = str(imu_data)
   
                self.publisher_.publish(msg)
                msg_str = str(msg)
                self.get_logger().info(msg_str)                
                

def main(args=None):
    rclpy.init(args=args)

    message_publisher = MessageDriver()

    rclpy.spin(message_publisher)
    message_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
