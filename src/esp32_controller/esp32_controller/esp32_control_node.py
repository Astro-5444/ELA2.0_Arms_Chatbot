import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
from std_msgs.msg import String

class ESP32ControlNode(Node):
    def __init__(self):
        super().__init__('esp32_control_node')
        try:
            self.ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        except Exception as e:
            self.get_logger().error(f"Error opening serial port: {e}")
            raise e

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        self.publisher_ = self.create_publisher(String, '/esp32_commands', 10)
        self.get_logger().info('ESP32 Control Node Started')

        # Calibration for each joint.
        self.calibration = {
            'left_upper_arm_x_joint': {
                'servo_center': 266,
                'scale': -160
            },
            'left_upper_arm_y_joint': {
                'servo_center': 500,
                'scale': -180
            },
            'left_forearm_z_joint': {
                'servo_center': 149,
                'scale': 70
            },
            'left_forearm_x_joint': {
                'servo_center': 330,
                'scale': -160
            },
            'left_finger_joint': {
                'servo_center': 266,
                'scale': 60
            },
            'right_upper_arm_x_joint': {
                'servo_center': 400,
                'scale': 160
            },
            'right_upper_arm_y_joint': {
                'servo_center': 90,
                'scale': -180
            },
            'right_forearm_z_joint': {
                'servo_center': 149,
                'scale': 70
            },
            'right_forearm_x_joint': {
                'servo_center': 400,
                'scale': 160
            },
            'right_finger_joint': {
                'servo_center': 300,  # Placeholder value; adjust for your hardware
                'scale': 60
            }
        }
        
        # Dictionary to keep the last PWM value sent for each channel.
        self.last_pwm = {}
        self.command_threshold = 5  # Only send command if difference exceeds this threshold

    def joint_callback(self, msg):
        # Process left arm joints.
        if 'left_upper_arm_x_joint' in msg.name:
            index = msg.name.index('left_upper_arm_x_joint')
            position = msg.position[index]
            pwm_value = self.position_to_pwm(position, 'left_upper_arm_x_joint')
            self.send_to_esp32(0, pwm_value)  # Channel 0

        if 'left_upper_arm_y_joint' in msg.name:
            index = msg.name.index('left_upper_arm_y_joint')
            position = msg.position[index]
            pwm_value = self.position_to_pwm(position, 'left_upper_arm_y_joint')
            self.send_to_esp32(1, pwm_value)  # Channel 1

        if 'left_forearm_z_joint' in msg.name:
            index = msg.name.index('left_forearm_z_joint')
            position = msg.position[index]
            pwm_value = self.position_to_pwm(position, 'left_forearm_z_joint')
            self.send_to_esp32(2, pwm_value)  # Channel 2

        if 'left_forearm_x_joint' in msg.name:
            index = msg.name.index('left_forearm_x_joint')
            position = msg.position[index]
            pwm_value = self.position_to_pwm(position, 'left_forearm_x_joint')
            self.send_to_esp32(3, pwm_value)  # Channel 3
            
        if 'left_finger_joint' in msg.name:
            index = msg.name.index('left_finger_joint')
            position = msg.position[index]
            pwm_value = self.position_to_pwm(position, 'left_finger_joint')
            self.send_to_esp32(4, pwm_value)  # Channel 4

        # Process right arm joints.
        if 'right_upper_arm_x_joint' in msg.name:
            index = msg.name.index('right_upper_arm_x_joint')
            position = msg.position[index]
            pwm_value = self.position_to_pwm(position, 'right_upper_arm_x_joint')
            self.send_to_esp32(8, pwm_value)   # Channel 8

        if 'right_upper_arm_y_joint' in msg.name:
            index = msg.name.index('right_upper_arm_y_joint')
            position = msg.position[index]
            pwm_value = self.position_to_pwm(position, 'right_upper_arm_y_joint')
            self.send_to_esp32(9, pwm_value)  # Channel 9

        if 'right_forearm_z_joint' in msg.name:
            index = msg.name.index('right_forearm_z_joint')
            position = msg.position[index]
            pwm_value = self.position_to_pwm(position, 'right_forearm_z_joint')
            self.send_to_esp32(10, pwm_value)  # Channel 10

        if 'right_forearm_x_joint' in msg.name:
            index = msg.name.index('right_forearm_x_joint')
            position = msg.position[index]
            pwm_value = self.position_to_pwm(position, 'right_forearm_x_joint')
            self.send_to_esp32(11, pwm_value)  # Channel 11

        if 'right_finger_joint' in msg.name:
            index = msg.name.index('right_finger_joint')
            position = msg.position[index]
            pwm_value = self.position_to_pwm(position, 'right_finger_joint')
            self.send_to_esp32(12, pwm_value)  # Channel 12

    def position_to_pwm(self, position, joint):
        """
        Converts a simulation position (radians) to a PWM command.
        PWM = servo_center + (scale * simulation_offset)
        """
        params = self.calibration.get(joint)
        if params is None:
            min_pwm = 100
            max_pwm = 500
            min_position = -1.57
            max_position = 1.57
            pwm_value = int((position - min_position) / (max_position - min_position) * (max_pwm - min_pwm) + min_pwm)
            return max(min_pwm, min(max_pwm, pwm_value))
        pwm_value = int(round(params['servo_center'] + params['scale'] * position))
        min_pwm = 100
        max_pwm = 500
        return max(min_pwm, min(max_pwm, pwm_value))

    def send_to_esp32(self, channel, pwm_value):
        # Check against the last value; only send if difference exceeds the threshold.
        last_value = self.last_pwm.get(channel, None)
        if last_value is not None and abs(pwm_value - last_value) < self.command_threshold:
            return  # Skip sending if change is too small.
        
        self.last_pwm[channel] = pwm_value  # Update last PWM for the channel.
        command = f"{channel},{pwm_value}\n"
        try:
            self.ser.write(command.encode('utf-8'))
            self.get_logger().info(f"Sent to ESP32: {command.strip()}")
        except Exception as e:
            self.get_logger().error(f"Error sending command to ESP32: {e}")

        msg = String()
        msg.data = command.strip()
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published to /esp32_commands: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = ESP32ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

