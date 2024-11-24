#!/usr/bin/env python3

# Import necessary modules from ROS2 and standard Python libraries
import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy 
from px4_msgs.msg import SensorCombined 
from std_msgs.msg import Int32
from px4_motor_error.motor_number_detector import detect_rotor_failure # Function to detect rotor failure based on gyro data

# Parameters for vibrational analysis
SAMPLES = 64  # Number of samples in the sliding window
SAMPLING_FREQUENCY = 240  # Sampling frequency in Hz
VIBRATION_THRESHOLD = 4  # RMS vibration threshold for abnormal detection
FFT_MAG_THRESHOLD = 1.2  # FFT magnitude threshold for abnormal frequency detection

# Sliding window buffer for vibration data
vibration_buffer = []

class MotorFailureDetectionNode(Node):
    """
    A ROS2 Node to detect potential motor failures based on vibration and gyroscope data.

    This node subscribes to IMU sensor data, performs vibration analysis using RMS and FFT,
    and publishes suspected motor failure messages if abnormal conditions are detected.
    """

    def __init__(self):
        super().__init__('motor_error_detector')
        self.get_logger().info('Initializing Error detector Code')

        # Initialize motor number to 0 (no motor failed initially)
        self.motor_number = 0

        # Store previous gyroscope readings for future comparisons
        self.prev_gyro = [0.0,0.0,0.0]

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Initialize subscriber
        self.subscription = self.create_subscription(
            SensorCombined,
            '/fmu/out/sensor_combined',
            self.imu_callback,
            qos_profile
        )

        # Publisher to publish motor failure status based on analysis
        self.error_publisher = self.create_publisher(
            Int32,
            '/custom_controller/drone_motor_status',
            qos_profile
        )

    def imu_callback(self, data):

        """
        Callback function to process IMU sensor data.

        Extracts accelerometer data from the incoming message, computes RMS vibration,
        and checks for abnormal vibration levels. If abnormal vibration is detected, 
        it triggers FFT analysis to confirm motor failure.

        Parameters:
        - data (SensorCombined): Incoming message with accelerometer and gyroscope data.
        """

        global vibration_buffer

        # Extract acceleration data
        ax = data.accelerometer_m_s2.data[0]
        ay = data.accelerometer_m_s2.data[1]
        az = data.accelerometer_m_s2.data[2]

        # Calculate RMS vibration for current sample
        rms_vibration = np.sqrt((ax**2 + ay**2 + az**2)/3.0)

        # Append to sliding window buffer
        vibration_buffer.append(rms_vibration)
        if len(vibration_buffer) > SAMPLES:
            vibration_buffer.pop(0)  # Keep buffer size constant

        # Only proceed if buffer is full
        if len(vibration_buffer) == SAMPLES:
            # Calculate overall RMS vibration
            overall_rms = np.sqrt(np.mean(np.square(vibration_buffer)))

            if self.perform_fft_analysis(vibration_buffer):
                self.get_logger().error(f"Motor failure suspected based on FFT analysis! (RMS: {overall_rms:.2f})")
                
                # Determine failed motor number based on gyroscope data
                self.get_motor_failure_number(data)
                self.get_logger().error(f"Motor failure number is {self.motor_number}")

                # Publish the motor failure number
                msg = Int32()
                msg.data = self.motor_number
                self.error_publisher.publish(msg)


                """ ---- ---- ---- ---- """
                # Attach LQR here...
                """ ---- ---- ---- ---- """




    def get_motor_failure_number(self,data):

        """
        Determines the motor that has likely failed by analyzing gyroscope data.

        This function uses the `detect_rotor_failure` function to identify a specific 
        motor failure pattern and updates the `motor_number` based on the result.

        Parameters:
        - data (SensorCombined): Incoming message with gyroscope data.
        """

        # Extract gyroscope data
        gyroX = data.gyro_rad.data[0]
        gyroY = data.gyro_rad.data[1]
        gyroZ = data.gyro_rad.data[2]

        # Detect motor failure pattern based on gyroscope data
        value = detect_rotor_failure(gyroX,gyroY,gyroZ)
        for index, i in enumerate(value):
            if(i == 1):
                self.motor_number = index+1 # Set motor number based on failure pattern
        
    def perform_fft_analysis(self, data):
        """
        Perform FFT analysis on vibration data to check for abnormal frequency magnitudes.

        Parameters:
        - data (list): List of RMS vibration samples.

        Returns:
        - bool: True if any FFT magnitude exceeds the threshold (excluding DC component), otherwise False.
        """

        # Perform FFT on the vibration data
        fft_result = np.fft.fft(data)
        fft_magnitudes = np.abs(fft_result[:SAMPLES // 2])  # Use real part of the FFT result

        print(f"{fft_magnitudes}")

        # Check for abnormal frequency magnitudes
        if any(magnitude > FFT_MAG_THRESHOLD for magnitude in fft_magnitudes[1:]):  # Ignore DC component
            return True
        return False

def main(args=None):

    """
    Main function to initialize and run the MotorFailureDetectionNode.

    Initializes ROS2, creates an instance of MotorFailureDetectionNode, and keeps the node running.
    """

    rclpy.init(args=args)
    motor_failure_detection_node = MotorFailureDetectionNode()
    rclpy.spin(motor_failure_detection_node)
    motor_failure_detection_node.destroy_node()
    rclpy.shutdown()

#Entry Point
if __name__ == '__main__':
    main()
