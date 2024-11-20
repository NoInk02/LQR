#!/usr/bin/env python3
''' File launches a node that takes trigger values to log sensor data of the drone into separate csv files'''


import rclpy
from rclpy.node import Node
from px4_msgs.msg import SensorCombined
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import csv
import os

class SensorDataLogger(Node):
    """Node for logging sensor data into CSV files based on a trigger signal."""

    def __init__(self):
        super().__init__('sensor_data_logger')
        self.logging_enabled = False  # Initially, logging is disabled
        self.iteration = 0

        # Set up QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber for the SensorCombined topic
        self.sensor_subscription = self.create_subscription(
            SensorCombined,
            '/fmu/out/sensor_combined',
            self.sensor_callback,
            qos_profile)
        
        
        
        # Subscriber for the Int32 trigger topic
        self.trigger_subscription = self.create_subscription(
            Int32,
            '/custom_controller/data_logging_trigger',
            self.trigger_callback,
            10)

        # Setup initial CSV file for logging
        self.csv_file_path = 'sensor_data_log.csv'
        self._open_csv_file()

    def _open_csv_file(self):
        """Helper method to open a new CSV file."""
        file_exists = os.path.isfile(self.csv_file_path)
        self.csv_file = open(self.csv_file_path, mode='a', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # Write header only if the file does not already exist
        if not file_exists:
            self.csv_writer.writerow([
                'timestamp', 
                'gyro_rad', 
                'gyro_integral_dt', 
                'accelerometer_timestamp_relative', 
                'accelerometer_m_s2', 
                'accelerometer_integral_dt'
            ])
        self.get_logger().info(f'Opened file: {self.csv_file_path}')

    def sensor_callback(self, msg):
        """Callback to log sensor data when logging is enabled."""
        # Only log data if logging is enabled
        if self.logging_enabled:
            # Extract data from the SensorCombined message
            timestamp = msg.timestamp
            gyro_rad = msg.gyro_rad
            gyro_integral_dt = msg.gyro_integral_dt
            accelerometer_timestamp_relative = msg.accelerometer_timestamp_relative
            accelerometer_m_s2 = msg.accelerometer_m_s2
            accelerometer_integral_dt = msg.accelerometer_integral_dt

            # Log data to CSV
            self.csv_writer.writerow([
                timestamp,
                gyro_rad,
                gyro_integral_dt,
                accelerometer_timestamp_relative,
                accelerometer_m_s2,
                accelerometer_integral_dt
            ])
            
            self.get_logger().info(f'Logged sensor data: {timestamp}, {gyro_rad}, {accelerometer_m_s2}')

    def trigger_callback(self, msg):
        """Callback to enable or disable logging based on the trigger value."""
        # Enable logging if trigger value is 1
        if msg.data == 1:
            self.logging_enabled = True
            self.get_logger().info('Data logging enabled.')
        # Disable logging if trigger value is 0
        elif msg.data == 0:
            self.logging_enabled = False
            self.get_logger().info('Data logging disabled.')

            # Close current file and increment iteration
            self.csv_file.close()
            self.iteration += 1
            self.csv_file_path = f"sensor_data_log_{self.iteration}.csv"

            # Open new file for logging
            self._open_csv_file()

    def destroy_node(self):
        super().destroy_node()
        self.csv_file.close()


def main(args=None):
    """Main function to initialize the sensor data logger node and start it."""
    rclpy.init(args=args)
    sensor_data_logger = SensorDataLogger()

    try:
        rclpy.spin(sensor_data_logger)
    except KeyboardInterrupt:
        pass

    # Cleanup
    sensor_data_logger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()