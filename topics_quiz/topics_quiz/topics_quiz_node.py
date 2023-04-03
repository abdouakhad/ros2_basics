import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class Topics_quiz(Node):

    def __init__(self):
        super().__init__('topics_quiz')
        # self.publisher_ = self.create_publisher(Age, 'age', 10)
        self.subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10) 
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom = Odometry()
        self.cmd = Twist()
        self.cmd.linear.x = 0.3
        
        self.current_orientation = 0.0
        self.target_orientation = math.pi / 2.0  # 90 degrees in radians
        self.angular_vel = 0.5  # radians per second
    

        # self.timer = self.create_timer(self.timer_period, self.odom_callback)

    def odom_callback(self,msg):

       
        self.publisher_.publish(self.cmd)
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation
        
        # Convert quaternion to Euler angles (in degrees)
        x = self.orientation.x
        y = self.orientation.y
        z = self.orientation.z
        w = self.orientation.w
        
        roll, pitch, yaw = self.quaternion_to_euler(x, y, z, w)
        self.current_orientation = yaw
        yaw_deg = math.degrees(yaw)

        print(f"Current Position: ({self.position.x}, {self.position.y}, {self.position.z})")
        print(f"Current Orientation: ({yaw_deg:.2f} degrees)")

        if self.position.x >= 1.02:
            print('Time to turn around')
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            
            if abs(self.current_orientation - self.target_orientation) > 0.1:  # threshold angle error
                self.cmd.angular.z = self.angular_vel * math.copysign(1.0, self.target_orientation - self.current_orientation)
            else:
                self.cmd.angular.z = 0.0  # stop turning
        
            self.publisher_.publish(self.cmd)

            # Keep moving forward
            if abs(self.current_orientation - self.target_orientation) < 0.1:   # threshold angle error
                self.cmd.linear.x = 0.2  # set linear velocity to move forward
            else:
                self.cmd.linear.x = 0.0  # stop moving forward

            self.publisher_.publish(self.cmd)

        if self.position.y > 0.7:
                self.cmd.linear.x = 0.0
                self.publisher_.publish(self.cmd)
        

    def quaternion_to_euler(self, x, y, z, w):
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x**2 + y**2)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y**2 + z**2)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    topics_quiz = Topics_quiz()
    rclpy.spin(topics_quiz)
    topics_quiz.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()