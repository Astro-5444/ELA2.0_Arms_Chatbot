import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
from std_msgs.msg import String

class ESP32ControlNode(Node):
    def __init__(self):
        super().__init__('esp32_control_node')
        self.ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        self.publisher_ = self.create_publisher(String, '/esp32_commands', 10)
        self.get_logger().info('ESP32 Control Node Started')

        # Calibration dictionary for each joint.
        # For each joint, we define:
        #   servo_center: the PWM value that centers the joint in real life.
        #   scale: the conversion factor (PWM per radian) that adjusts for a nonzero simulation offset.
        #
        # Here, the simulation's center is 0.0, but the physical arm's center (when measured in radians)
        # would be (for example) 1.571 for joint 1, -0.298 for joint 2, etc.
        # When simulation sends 0.0 the desired PWM should be:
        #   Joint 1: 266
        #   Joint 2: 500
        #   Joint 3: 149
        #   Joint 4: 333
        self.calibration = {
            'left_upper_arm_x_joint': {
                'servo_center': 266,  # When simulation offset=0, use 266 (physical center PWM)
                'scale': -160           # Placeholder: PWM change per radian offset
            },
            'left_upper_arm_y_joint': {
                'servo_center': 500,
                'scale': -180          # Placeholder value; note the negative if increasing radians decreases PWM
            },
            'left_forearm_z_joint': {
                'servo_center': 149,
                'scale': 70           # Placeholder value
            },
            'left_forearm_x_joint': {
                'servo_center': 330,
                'scale': -160           # Placeholder value
            },
            'left_finger_joint': {
                'servo_center': 266,
                'scale': 60           # Placeholder value
            },



            'right_upper_arm_x_joint': {
                'servo_center': 400,  # When simulation offset=0, use 266 (physical center PWM)
                'scale': 160           # Placeholder: PWM change per radian offset
            },
            'right_upper_arm_y_joint': {
                'servo_center': 90,
                'scale': -180          # Placeholder value; note the negative if increasing radians decreases PWM
            },
            'right_forearm_z_joint': {
                'servo_center': 149,
                'scale': 70           # Placeholder value
            },
            'right_forearm_x_joint': {
                'servo_center': 400,
                'scale': 160           # Placeholder value
            },
            

        }

    def joint_callback(self, msg):
        # For each joint we care about, get the simulation offset (in radians)
        # and convert it to a PWM value using our calibration.
        if 'left_upper_arm_x_joint' in msg.name:
            index = msg.name.index('left_upper_arm_x_joint')
            # Simulation value (offset from center)
            position = msg.position[index]
            pwm_value = self.position_to_pwm(position, 'left_upper_arm_x_joint')
            self.send_to_esp32(0, pwm_value)  # Send on channel 0

        if 'left_upper_arm_y_joint' in msg.name:
            index = msg.name.index('left_upper_arm_y_joint')
            position = msg.position[index]
            pwm_value = self.position_to_pwm(position, 'left_upper_arm_y_joint')
            self.send_to_esp32(1, pwm_value)  # Send on channel 1

        if 'left_forearm_z_joint' in msg.name:
            index = msg.name.index('left_forearm_z_joint')
            position = msg.position[index]
            pwm_value = self.position_to_pwm(position, 'left_forearm_z_joint')
            self.send_to_esp32(2, pwm_value)  # Send on channel 2

        if 'left_forearm_x_joint' in msg.name:
            index = msg.name.index('left_forearm_x_joint')
            position = msg.position[index]
            pwm_value = self.position_to_pwm(position, 'left_forearm_x_joint')
            self.send_to_esp32(3, pwm_value)  # Send on channel 3
            
        if 'left_finger_joint' in msg.name:
            index = msg.name.index('left_finger_joint')
            position = msg.position[index]
            pwm_value = self.position_to_pwm(position, 'left_finger_joint')
            self.send_to_esp32(4, pwm_value)  # Send on channel 3

        if 'right_upper_arm_x_joint' in msg.name:
            index = msg.name.index('right_upper_arm_x_joint')
            # Simulation value (offset from center)
            position = msg.position[index]
            pwm_value = self.position_to_pwm(position, 'right_upper_arm_x_joint')
            self.send_to_esp32(8, pwm_value)  


        if 'right_upper_arm_y_joint' in msg.name:
            index = msg.name.index('right_upper_arm_y_joint')
            position = msg.position[index]
            pwm_value = self.position_to_pwm(position, 'right_upper_arm_y_joint')
            self.send_to_esp32(9, pwm_value)  # Send on channel 1

        
        if 'right_forearm_z_joint' in msg.name:
            index = msg.name.index('right_forearm_z_joint')
            position = msg.position[index]
            pwm_value = self.position_to_pwm(position, 'right_forearm_z_joint')
            self.send_to_esp32(10, pwm_value)  # Send on channel 2

        if 'right_forearm_x_joint' in msg.name:
            index = msg.name.index('right_forearm_x_joint')
            position = msg.position[index]
            pwm_value = self.position_to_pwm(position, 'right_forearm_x_joint')
            self.send_to_esp32(11, pwm_value)  # Send on channel 3





    def position_to_pwm(self, position, joint):
        """
        Converts a simulation position (in radians) to a PWM command.
        Since the simulation's center is 0.0, we want:
          PWM = servo_center + (scale * simulation_offset)
        """
        params = self.calibration.get(joint)
        if params is None:
            # Fallback linear mapping if no calibration data is available
            min_pwm = 100
            max_pwm = 500
            min_position = -1.57
            max_position = 1.57
            pwm_value = int((position - min_position) / (max_position - min_position) * (max_pwm - min_pwm) + min_pwm)
            return max(min_pwm, min(max_pwm, pwm_value))

        pwm_value = int(round(params['servo_center'] + params['scale'] * position))
        # Clamp PWM value within allowed range (if needed)
        min_pwm = 100
        max_pwm = 500
        return max(min_pwm, min(max_pwm, pwm_value))

    def send_to_esp32(self, channel, pwm_value):
        command = f"{channel},{pwm_value}\n"
        self.ser.write(command.encode('utf-8'))
        self.get_logger().info(f'Sent to ESP32: {command.strip()}')

        # Also publish the command on a ROS topic for debugging.
        msg = String()
        msg.data = command.strip()
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published to /esp32_commands: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = ESP32ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
