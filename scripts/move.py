import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math


class move(Node):

    def __init__(self):
        super().__init__('move')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_ = self.create_timer(1.0, self.timer_callback)
        self.linear_speed_ = 0.2
        self.angular_speed_ = 0.5
        self.direction_ = 1
        self.count_ = 0

    def timer_callback(self):
        msg = Twist()

        if self.count_ < 5:
            msg.linear.x = self.linear_speed_
            msg.angular.z = 0.0
        elif self.count_ >= 5 and self.count_ < 10:
            msg.linear.x = 0.0
            msg.angular.z = self.direction_ * self.angular_speed_
        else:
            self.count_ = 0
            self.direction_ = -self.direction_

        self.publisher_.publish(msg)
        self.count_ += 1


def main(args=None):
    rclpy.init(args=args)
    move_node = move()
    rclpy.spin(zigzag_node)
    move_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()