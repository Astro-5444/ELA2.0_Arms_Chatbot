import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubs(Node):
    def __init__(self):
        super().__init__("simple_subs")
        self.sub_ = self.create_subscription(
            String, 
            "chatter", 
            self.msg_callback, 
            10
        )
        self.sub_  # prevent unused variable warning

    def msg_callback(self, msg):
        self.get_logger().info("I heard: %s" % msg.data)

def main():
    rclpy.init()
    simple_subs = SimpleSubs()
    rclpy.spin(simple_subs)
    simple_subs.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
