#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import SensorCombined
from std_msgs.msg import Int32, Float32MultiArray
from px4_motor_error.motor_number_detector import detect_rotor_failure

# Parameters for vibrational analysis
SAMPLES = 64
SAMPLING_FREQUENCY = 240
VIBRATION_THRESHOLD = 4
FFT_MAG_THRESHOLD = 1.2

vibration_buffer = []

class MotorFailureDetectionNode(Node):
    """
    A ROS2 Node to detect motor failures and apply LQR control for stabilization.
    """

    def __init__(self):
        super().__init__('motor_error_detector_with_lqr')
        self.get_logger().info('Initializing Error Detector with LQR Control')

        # Initialize motor failure status
        self.motor_number = 0
        self.prev_gyro = [0.0, 0.0, 0.0]

        # Initialize state vector
        self.state = np.zeros(12)  # Assuming a 12-state system (position, velocity, attitude, angular rates)

        # LQR controller gain (to be computed)
        self.K = self.calculate_lqr_gain()

        # Configure QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.subscription = self.create_subscription(
            SensorCombined,
            '/fmu/out/sensor_combined',
            self.imu_callback,
            qos_profile
        )

        # Publishers
        self.error_publisher = self.create_publisher(
            Int32,
            '/custom_controller/drone_motor_status',
            qos_profile
        )
        self.control_publisher = self.create_publisher(
            Float32MultiArray,
            '/custom_controller/control_commands',
            qos_profile
        )

    def calculate_lqr_gain(self):
        """
        Compute the LQR gain matrix for stabilization.
        """
        # State-space matrices (example: simplified dynamics)
        A = np.zeros((12, 12))  # Replace with actual linearized dynamics
        B = np.zeros((12, 4))   # Replace with actual control input matrix
        Q = np.eye(12) * 10     # State cost
        R = np.eye(4) * 0.1     # Input cost

        # Solve the continuous-time algebraic Riccati equation
        X = np.linalg.solve_continuous_are(A, B, Q, R)
        K = np.linalg.inv(R) @ B.T @ X
        return K

    def lqr_control(self, state):
        """
        Compute LQR control commands based on the current state.
        """
        desired_state = np.zeros(12)  # Reference state (hovering, for example)
        control_input = -self.K @ (state - desired_state)
        return control_input

    def imu_callback(self, data):
        global vibration_buffer

        # Extract IMU data
        ax, ay, az = data.accelerometer_m_s2.data
        gyroX, gyroY, gyroZ = data.gyro_rad.data

        # Update state vector (example: position, velocity, angular rates)
        self.state[0:3] = [ax, ay, az]
        self.state[3:6] = [gyroX, gyroY, gyroZ]

        # RMS Vibration Analysis
        rms_vibration = np.sqrt((ax**2 + ay**2 + az**2) / 3.0)
        vibration_buffer.append(rms_vibration)
        if len(vibration_buffer) > SAMPLES:
            vibration_buffer.pop(0)

        if len(vibration_buffer) == SAMPLES:
            if self.perform_fft_analysis(vibration_buffer):
                self.get_logger().error("Motor failure suspected based on FFT analysis!")
                self.get_motor_failure_number(data)

                # Publish motor failure number
                msg = Int32()
                msg.data = self.motor_number
                self.error_publisher.publish(msg)

        # Compute LQR control commands
        control_commands = self.lqr_control(self.state)

        # Publish control commands
        control_msg = Float32MultiArray()
        control_msg.data = control_commands.tolist()
        self.control_publisher.publish(control_msg)

    def get_motor_failure_number(self, data):
        """
        Determines the motor that has likely failed by analyzing gyroscope data.
        """
        gyroX, gyroY, gyroZ = data.gyro_rad.data
        value = detect_rotor_failure(gyroX, gyroY, gyroZ)
        for index, i in enumerate(value):
            if i == 1:
                self.motor_number = index + 1

    def perform_fft_analysis(self, data):
        """
        Perform FFT analysis on vibration data.
        """
        fft_result = np.fft.fft(data)
        fft_magnitudes = np.abs(fft_result[:SAMPLES // 2])
        return any(mag > FFT_MAG_THRESHOLD for mag in fft_magnitudes[1:])


def main(args=None):
    rclpy.init(args=args)
    motor_failure_detection_node = MotorFailureDetectionNode()
    rclpy.spin(motor_failure_detection_node)
    motor_failure_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
