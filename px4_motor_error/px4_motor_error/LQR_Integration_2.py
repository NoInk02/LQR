#!/usr/bin/env python3

# Import necessary modules
import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy 
from px4_msgs.msg import SensorCombined, ActuatorControls  # Publish control commands here
from std_msgs.msg import Int32
from px4_motor_error.motor_number_detector import detect_rotor_failure

# Parameters for vibrational analysis
SAMPLES = 64
SAMPLING_FREQUENCY = 240
VIBRATION_THRESHOLD = 4
FFT_MAG_THRESHOLD = 1.2

# Quadrotor Physical Constants
g = 9.81
m = 1.0
Ix = 8.1 * 1e1
Iy = 8.1 * 1e1
Iz = 14.2 * 1e1

# Sliding window buffer for vibration data
vibration_buffer = []

class MotorFailureDetectionNode(Node):
    """
    ROS2 Node to detect potential motor failures and stabilize the quadrotor using LQR control.
    """

    def __init__(self):
        super().__init__('motor_error_detector')
        self.get_logger().info('Initializing Motor Error Detector and LQR Controller')

        self.motor_number = 0  # Initialize motor failure flag
        self.prev_gyro = [0.0, 0.0, 0.0]

        # Configure QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber for IMU data
        self.subscription = self.create_subscription(
            SensorCombined,
            '/fmu/out/sensor_combined',
            self.imu_callback,
            qos_profile
        )

        # Publisher for motor failure status
        self.error_publisher = self.create_publisher(
            Int32,
            '/custom_controller/drone_motor_status',
            qos_profile
        )

        # Publisher for actuator control (LQR control commands)
        self.control_publisher = self.create_publisher(
            ActuatorControls,
            '/fmu/in/actuator_controls_0',
            qos_profile
        )

        # Define LQR Parameters
        self.Ks = self.calculate_lqr_gains()

    def calculate_lqr_gains(self):
        """
        Calculate the LQR gains for each subsystem.
        """
        Ks = []
        # X-subsystem
        Ax = np.array([[0.0, 1.0, 0.0, 0.0],
                       [0.0, 0.0, g, 0.0],
                       [0.0, 0.0, 0.0, 1.0],
                       [0.0, 0.0, 0.0, 0.0]])
        Bx = np.array([[0.0], [0.0], [0.0], [1 / Ix]])

        # Y-subsystem
        Ay = np.array([[0.0, 1.0, 0.0, 0.0],
                       [0.0, 0.0, -g, 0.0],
                       [0.0, 0.0, 0.0, 1.0],
                       [0.0, 0.0, 0.0, 0.0]])
        By = np.array([[0.0], [0.0], [0.0], [1 / Iy]])

        # Z-subsystem
        Az = np.array([[0.0, 1.0],
                       [0.0, 0.0]])
        Bz = np.array([[0.0], [1 / m]])

        # Yaw-subsystem
        Ayaw = np.array([[0.0, 1.0],
                         [0.0, 0.0]])
        Byaw = np.array([[0.0], [1 / Iz]])

        # Define LQR cost matrices
        subsystems = [(Ax, Bx), (Ay, By), (Az, Bz), (Ayaw, Byaw)]
        for A, B in subsystems:
            n = A.shape[0]
            Q = np.eye(n)
            Q[0, 0] = 10.0  # Prioritize the position states
            R = np.eye(B.shape[1])  # Penalize control effort
            K = self.lqr(A, B, Q, R)
            Ks.append(K)
        return Ks

    def lqr(self, A, B, Q, R):
        """
        Solve the continuous time LQR controller.
        """
        X = np.matrix(np.linalg.solve_continuous_are(A, B, Q, R))
        K = np.matrix(np.linalg.inv(R) @ (B.T @ X))
        return np.asarray(K)

    def imu_callback(self, data):
        """
        Process IMU data and apply LQR control if motor failure is detected.
        """
        global vibration_buffer

        # Extract accelerometer data
        ax = data.accelerometer_m_s2.data[0]
        ay = data.accelerometer_m_s2.data[1]
        az = data.accelerometer_m_s2.data[2]

        # Calculate RMS vibration
        rms_vibration = np.sqrt((ax**2 + ay**2 + az**2) / 3.0)
        vibration_buffer.append(rms_vibration)
        if len(vibration_buffer) > SAMPLES:
            vibration_buffer.pop(0)

        if len(vibration_buffer) == SAMPLES:
            overall_rms = np.sqrt(np.mean(np.square(vibration_buffer)))
            if self.perform_fft_analysis(vibration_buffer):
                self.get_logger().error(f"Motor failure detected! RMS: {overall_rms:.2f}")
                self.get_motor_failure_number(data)
                self.publish_control(data)

    def get_motor_failure_number(self, data):
        """
        Analyze gyroscope data to detect failed motor.
        """
        gyroX, gyroY, gyroZ = data.gyro_rad.data[:3]
        failure_pattern = detect_rotor_failure(gyroX, gyroY, gyroZ)
        self.motor_number = next((i + 1 for i, failed in enumerate(failure_pattern) if failed), 0)

    def publish_control(self, data):
        """
        Apply LQR control and publish actuator commands.
        """
        state = np.zeros(12)  # Placeholder for current state vector
        # Extract current state from IMU data
        # Example: state[0] = position_x, state[1] = velocity_x, etc.

        # Calculate control inputs
        UZ = -self.Ks[2].dot(state[[4, 5]])
        UX = -self.Ks[0].dot(state[[0, 1, 8, 9]])
        UY = -self.Ks[1].dot(state[[2, 3, 6, 7]])
        UYaw = -self.Ks[3].dot(state[[10, 11]])

        # Publish control commands
        control_msg = ActuatorControls()
        control_msg.control = [UX[0], UY[0], UZ[0], UYaw[0], 0.0, 0.0, 0.0, 0.0]
        self.control_publisher.publish(control_msg)

    def perform_fft_analysis(self, data):
        """
        Perform FFT analysis on vibration data.
        """
        fft_result = np.fft.fft(data)
        fft_magnitudes = np.abs(fft_result[:SAMPLES // 2])
        return any(magnitude > FFT_MAG_THRESHOLD for magnitude in fft_magnitudes[1:])

def main(args=None):
    rclpy.init(args=args)
    node = MotorFailureDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
