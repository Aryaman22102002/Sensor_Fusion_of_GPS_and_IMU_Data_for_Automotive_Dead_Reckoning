#!/usr/bin/env python3
import os
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
import pandas as pd
import numpy as np

def read_ros2_bag_to_csv(bag_file, csv_file):
    rclpy.init()

    storage_options = rosbag2_py.StorageOptions(uri=bag_file, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_dict = {topic.name: topic.type for topic in topic_types}

    data = {
        'timestamp_sec_gps': [],
        'timestamp_nanosec_gps': [],
        'timestamp_sec_imu': [],
        'timestamp_nanosec_imu': [],
        'latitude': [],
        'longitude': [],
        'altitude': [],
        'utm_easting': [],
        'utm_northing': [],
        'zone': [],
        'letter': [],
        'orientation_x': [],
        'orientation_y': [],
        'orientation_z': [],
        'orientation_w': [],
        'linear_acceleration_x': [],
        'linear_acceleration_y': [],
        'linear_acceleration_z': [],
        'angular_velocity_x': [],
        'angular_velocity_y': [],
        'angular_velocity_z': [],
        'magnetic_field_x': [],
        'magnetic_field_y': [],
        'magnetic_field_z': []
    }

    last_gps_timestamp_sec = None
    last_imu_timestamp_sec = None

    while reader.has_next():
        (topic, data_bytes, t) = reader.read_next()
        
        if topic == '/gps':  
            msg_type = get_message(type_dict[topic])
            msg = deserialize_message(data_bytes, msg_type)

            data['timestamp_sec_gps'].append(msg.header.stamp.sec)
            data['timestamp_nanosec_gps'].append(msg.header.stamp.nanosec)
            data['latitude'].append(msg.latitude)
            data['longitude'].append(msg.longitude)
            data['altitude'].append(msg.altitude)
            data['utm_easting'].append(msg.utm_easting)
            data['utm_northing'].append(msg.utm_northing)
            data['zone'].append(msg.zone)
            data['letter'].append(msg.letter)

            # Fill None for IMU data when GPS is present but IMU isn't
            if last_imu_timestamp_sec != msg.header.stamp.sec:
                data['timestamp_sec_imu'].append(None)
                data['timestamp_nanosec_imu'].append(None)
                data['orientation_x'].append(None)
                data['orientation_y'].append(None)
                data['orientation_z'].append(None)
                data['orientation_w'].append(None)
                data['linear_acceleration_x'].append(None)
                data['linear_acceleration_y'].append(None)
                data['linear_acceleration_z'].append(None)
                data['angular_velocity_x'].append(None)
                data['angular_velocity_y'].append(None)
                data['angular_velocity_z'].append(None)
                data['magnetic_field_x'].append(None)
                data['magnetic_field_y'].append(None)
                data['magnetic_field_z'].append(None)

        if topic == '/imu':  
            msg_type = get_message(type_dict[topic])
            msg = deserialize_message(data_bytes, msg_type)

            data['timestamp_sec_imu'].append(msg.header.stamp.sec)
            data['timestamp_nanosec_imu'].append(msg.header.stamp.nanosec)
            data['orientation_x'].append(msg.imu.orientation.x)
            data['orientation_y'].append(msg.imu.orientation.y)
            data['orientation_z'].append(msg.imu.orientation.z)
            data['orientation_w'].append(msg.imu.orientation.w)
            data['linear_acceleration_x'].append(msg.imu.linear_acceleration.x)
            data['linear_acceleration_y'].append(msg.imu.linear_acceleration.y)
            data['linear_acceleration_z'].append(msg.imu.linear_acceleration.z)
            data['angular_velocity_x'].append(msg.imu.angular_velocity.x)
            data['angular_velocity_y'].append(msg.imu.angular_velocity.y)
            data['angular_velocity_z'].append(msg.imu.angular_velocity.z)
            data['magnetic_field_x'].append(msg.mag_field.magnetic_field.x)
            data['magnetic_field_y'].append(msg.mag_field.magnetic_field.y)
            data['magnetic_field_z'].append(msg.mag_field.magnetic_field.z)

            # Fill None for GPS data when IMU is present but GPS isn't
            if last_gps_timestamp_sec != msg.header.stamp.sec:
                data['timestamp_sec_gps'].append(None)
                data['timestamp_nanosec_gps'].append(None)
                data['latitude'].append(None)
                data['longitude'].append(None)
                data['altitude'].append(None)
                data['utm_easting'].append(None)
                data['utm_northing'].append(None)
                data['zone'].append(None)
                data['letter'].append(None)

    rclpy.shutdown()

    df = pd.DataFrame(data)

    df.to_csv(csv_file, index=False)
    print(f"Data successfully written to {csv_file}")

if __name__ == '__main__':
    absolute_bag_file_path = input("Please enter the absolute path of the db3 file: ")
    absolute_csv_file_path = input("Please enter the absolute path where you want to store the CSV file: ")

    read_ros2_bag_to_csv(absolute_bag_file_path, absolute_csv_file_path)


