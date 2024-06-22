#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import time
from std_msgs.msg import Int32, Bool, String

class MovementController(Node):

    def __init__(self):
        super().__init__('movement_controller')
        self.subscription = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.state_sub = self.create_subscription(Int32, 'state', self.state_callback, 1)
        # self.state_pub2 = self.create_publisher(Int32, 'state2', 10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.initial_position = None
        self.initial_orientation = None
        self.state = 'move_forward'
        self.distance_to_travel = 2.5
        self.turn_angle = math.radians(90)
        self.linear_speed = 0.08
        self.angular_speed = 0.1
        self.tolerance = 0.01  # Tolerance for movement
        self.timer = self.create_timer(0.1, self.control_loop)
        self.active = False
        self.yaw = 0
        self.yaw_precision_ = math.pi / 90 
        self.desired_orientation = 0
        self.start_pub = self.create_publisher(String, 'customtopic', 10)


    def state_callback(self, msg):
        if msg.data == 1:
            self.active = True
            
        
    def odom_callback(self, msg):


        if self.initial_position is None:
            self.initial_position = msg.pose.pose.position
            self.initial_orientation = self.get_yaw_from_orientation(msg.pose.pose.orientation)
        self.current_position = msg.pose.pose.position
        self.current_orientation = self.get_yaw_from_orientation(msg.pose.pose.orientation)

    def get_yaw_from_orientation(self, orientation):
        quaternion = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )
        _, _, self.yaw = self.euler_from_quaternion(quaternion)
        return self.yaw

    def euler_from_quaternion(self, quaternion):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        quaternion = (x, y, z, w)
        """
        x, y, z, w = quaternion
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    def control_loop(self):
        if self.active:
            # self.get_logger().info('Sequence started')
            if self.initial_position is None or self.initial_orientation is None:
                return
            
            # if self.state == 'set_desired_orientation':
            #     self.set_desired_orientation(3, self.desired_orientation)
            if self.state == 'move_forward':
                self.move_forward()
            elif self.state == 'turn':
                self.turn()
            elif self.state == 'move_forward_again':
                self.move_forward_again()
            elif self.state == 'turn_again':
                self.turn_again()
            elif self.state == 'move_forward_final':
                self.move_forward_final()
            elif self.state == 'done':
                self.active = False
                self.state = 'move_forward'
                self.get_logger().info(f'self.active={self.active}')
                self.get_logger().info(f'self.state={self.state}')

                
    # def move_forward(self, duration=0.5):
        # distance_moved = self.get_distance_moved()
        # self.get_logger().info('Sequence1 started')
        # self.duration = duration
        # self.start_time = None
        # self.start_time = time.time() 
        # while time.time() - self.start_time < self.duration:
        #     twist = Twist()
        #     twist.linear.x = self.linear_speed
        #     self.publisher_.publish(twist)
        # self.get_logger().info('Pudhe gela')
            
            
        # twist = Twist()
        # twist.linear.x = 0.0
        # self.publisher_.publish(twist)
        # self.state = 'turn'
        # self.initial_orientation = self.current_orientation

    def set_desired_orientation(self, speed_in_degree, desired_angle_degree):
        self.get_logger().info('set desired orientation')
        relative_angle_radians = math.radians(desired_angle_degree) - self.yaw
        clockwise = 0
        if relative_angle_radians < 0:
            clockwise = 1
        else:
            clockwise = 0
        print("relative_angle_degree: ", math.degrees(relative_angle_radians))
        print("desired_angle_degree: ", desired_angle_degree)
        self.rotate(speed_in_degree, math.degrees(abs(relative_angle_radians)), clockwise)

        self.state = "move_forward"

    def rotate(self, angular_speed_degree, relative_angle_degree, clockwise):
        self.get_logger().info('rotating')

        velocity_message = Twist()
        angular_speed = math.radians(abs(angular_speed_degree))

        if clockwise:
            velocity_message.angular.z = -abs(angular_speed)
        else:
            velocity_message.angular.z = abs(angular_speed)

        angle_moved = 0.0
        loop_rate = self.create_rate(10)

        t0 = self.get_clock().now().seconds_nanoseconds()[0]

        while True:
            self.get_logger().info("Turtlebot rotates")
            self.publisher_.publish(velocity_message)

            t1 = self.get_clock().now().seconds_nanoseconds()[0]
            current_angle_degree = (t1 - t0) * angular_speed_degree
            loop_rate.sleep()

            if current_angle_degree > relative_angle_degree:
                self.get_logger().info("reached")
                break

        velocity_message.angular.z = 0
        self.publisher_.publish(velocity_message)
     

    def move_forward(self):
        distance_moved = self.get_distance_moved()
        self.get_logger().info('move_forward')
        if distance_moved < 0.45 - self.tolerance:
            twist = Twist()
            twist.linear.x = self.linear_speed
            self.publisher_.publish(twist)
        else:
            twist = Twist()
            twist.linear.x = 0.0
            self.publisher_.publish(twist)
            self.state = 'turn'
            self.initial_orientation = self.current_orientation


    def turn(self):
        angle_turned = self.get_angle_turned()
        self.get_logger().info('turn')
        if abs(angle_turned) < abs(self.turn_angle) - self.tolerance:
            twist = Twist()
            twist.angular.z = self.angular_speed if self.turn_angle > 0 else -self.angular_speed
            self.publisher_.publish(twist)
        else:
            twist = Twist()
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            self.state = 'move_forward_again'
            self.initial_position = self.current_position

    def move_forward_again(self):
        distance_moved = self.get_distance_moved()
        self.get_logger().info('move forward again')
        if distance_moved < 0.45 - self.tolerance:
            twist = Twist()
            twist.linear.x = self.linear_speed
            self.publisher_.publish(twist)
        else:
            twist = Twist()
            twist.linear.x = 0.0
            self.publisher_.publish(twist)
            self.state = 'turn_again'
            self.initial_orientation = self.current_orientation

    def turn_again(self):
        angle_turned = self.get_angle_turned()
        self.get_logger().info('turn again')
        if abs(angle_turned) < abs(self.turn_angle) - self.tolerance:
            twist = Twist()
            twist.angular.z = self.angular_speed if self.turn_angle > 0 else -self.angular_speed
            self.publisher_.publish(twist)
        else:
            twist = Twist()
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            self.state = 'move_forward_final'
            msg = String()
            msg.data='start'
            self.start_pub.publish(msg)
            self.initial_position = self.current_position

    def move_forward_final(self):
        distance_moved = self.get_distance_moved()
        self.get_logger().info('move forward final')
        if distance_moved < 0.45 - self.tolerance:
            twist = Twist()
            twist.linear.x = self.linear_speed
            self.publisher_.publish(twist)
        else:
            twist = Twist()
            twist.linear.x = 0.0
            self.publisher_.publish(twist)
            self.state = 'done'

    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def get_distance_moved(self):
        dx = self.current_position.x - self.initial_position.x
        dy = self.current_position.y - self.initial_position.y
        return math.sqrt(dx*dx + dy*dy)

    def get_angle_turned(self):
        return self.current_orientation - self.initial_orientation

def main(args=None):
    rclpy.init(args=args)
    movement_controller = MovementController()
    rclpy.spin(movement_controller)
    movement_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
