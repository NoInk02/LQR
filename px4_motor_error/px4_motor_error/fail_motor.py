# Import necessary modules from ROS2
from std_msgs.msg import Int32
import rclpy
from rclpy.node import Node

class MotorFailNode(Node):
    """
    A ROS2 Node to monitor and trigger motor failure messages.
    
    This node subscribes to a topic to listen for motor failure signals.
    Upon receiving a valid motor index, it publishes a message to trigger
    motor failure for the specified motor. If the index is invalid, it logs
    an error message and resets the index to 0.
    """

    def __init__(self):
        super().__init__('motor_fail_trigger_node')

        # Subscriber to listen for motor failure trigger messages
        # Listens on the '/custom_controller/motor_fail_trigger' topic, expecting Int32 messages

        self.motor_fail_listener = self.create_subscription(Int32, '/custom_controller/motor_fail_trigger',self.failMotor,10)
        
        # Publisher to send motor failure messages to a specific topic
        # Publishes to the '/motor_failure/motor_number' topic with an Int32 message type
        self.fail_motor = self.create_publisher(Int32,'/motor_failure/motor_number', 10)


    def failMotor(self,msg):
        """
        Callback function to handle received messages for motor failure.

        This function checks if the motor index in the received message is valid.
        If the motor index is greater than 4 (invalid), it logs an error message 
        and sets the motor index to 0. Otherwise, it publishes the motor failure
        message and logs the motor number.

        Parameters:
        - msg (Int32): The message containing the motor index to fail.
        """


        # Check if the motor index is greater than 4 (invalid motor index)
        if msg.data > 4:
            # Log an error if the motor index is out of range
            self.get_logger().error(f"Unable to process motor fail! No motor exists at index {msg.data}")
            # Set the motor index to 0 if invalid
            msg.data = 0
        else:
            # Log information on valid motor index and publish the message
            self.get_logger().info(f"Sent message to fail motor {msg.data}")
            self.fail_motor.publish(msg) # Publish the message to the motor failure topic


def main():
    '''
    Main function to initialize and run the MotorFailNode.
    '''


    rclpy.init()
    failNode = MotorFailNode()
    rclpy.spin(failNode)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
