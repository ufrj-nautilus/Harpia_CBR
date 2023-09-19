import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Empty
import math


class move(Node):

   def __init__(self):
        super().__init__('drone_controller')
        self.publisher_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.publisher_rc_override = self.create_publisher(
            OverrideRCIn, '/mavros/rc/override', 10)
        self.arm_service = rospy.ServiceProxy(
            '/mavros/cmd/arming', CommandBool)
        self.set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.takeoff_publisher = self.create_publisher(Empty, 'takeoff', 10)
        self.linear_speed = 0.2
        self.angular_speed = 0.5
        self.direction = 1
        self.count = 0

    def arm_drone(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            self.arm_service(True)
        except rospy.ServiceException as e:
            rospy.logerr("Failed to arm the drone: %s" % e)

    def heigth():

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