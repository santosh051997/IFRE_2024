#!/usr/bin/env python3

# import ros stuff
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion
import matplotlib
matplotlib.use('GTK3Agg')
from std_msgs.msg import Int32, String
import math
import matplotlib
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

# robot state variables
position_ = Point()
yaw_ = 0
# machine state
state_ = 0  # 0 means fix yaw, 1 means going to point, 2 means reached point
# goal point variable
goal = Point()
goal_received = False  # setting the flag variable
# parameters
yaw_precision_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.1
# publishers
pub = None
right = []
front_right = []
front = []
front_left = []
left = []
regions = {}
active_ = False
n = 0
yaw_case = 1
right_x_lower_limit = 13.5
left_x_higher_limit = 0.5
laser_ranges = []
laser_angles = []
Count = 1

bp = [[0.3, 0.100], [0.75, 0.31], [0.3, -0.31], [0.75, -0.100]]  # , [0.15, 0.1], [0.5, 0.2], [0.15, -0.2], [0.5, -0.1]]
width1 = bp[1][0] - bp[0][0]
height1 = bp[1][1] - bp[0][1]
width2 = bp[3][0] - bp[2][0]
height2 = bp[3][1] - bp[2][1]


class RowNavigation(Node):
    def __init__(self):
        super().__init__('row_navigation')
        global pub
        pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_laser = self.create_subscription(LaserScan, '/scan', self.clbk_laser, 10)
        self.sub = self.create_subscription(LaserScan, '/scan', self.front_boxes, 5)
        self.mid_point_goal_sub = self.create_subscription(Point, '/mid_point_goal', self.goal_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odometry/filtered', self.clbk_odom, 10)
        self.state_pub = self.create_publisher(Int32, 'state', 10)
        self.stop_pub = self.create_publisher(String, 'customtopic', 10)



        self.timer = self.create_timer(0.05, self.loop)

        time.sleep(1)

    def clbk_laser(self, msg):
        global regions, right_dist, right_dist_avg, left_dist, left_dist_avg, front_dist, front_dist_avg, Count
        ranges = msg.ranges
        ranges = [x for x in ranges if not math.isnan(x)]
        custom_range = ranges[270:360] + ranges[0:90]

        right_dist = [x for x in custom_range[0:70] if x < 1.1]
        left_dist = [x for x in custom_range[110:180] if x < 1.1]

        if len(right_dist) == 0 and len(left_dist) == 0:
            print('no data points for distance found')
            right_dist_avg = 0
            left_dist_avg = 0
        elif len(right_dist) == 0 and len(left_dist) > 0:
            left_dist_avg = sum(left_dist) / len(left_dist)
            right_dist_avg = 0

        elif len(right_dist) > 0 and len(left_dist) == 0:
            right_dist_avg = sum(right_dist) / len(right_dist)
            left_dist_avg = 0

        elif len(right_dist) > 0 and len(left_dist) > 0:
            right_dist_avg = sum(right_dist) / len(right_dist)
            left_dist_avg = sum(left_dist) / len(left_dist)

        regions = {
            'right': min(min(custom_range[0:70]), 1.0),
            'right-1': min(min(custom_range[30:70]), 0.75),
            'fright': min(min(custom_range[70:85]), 1),
            'front': min(min(custom_range[70:110]), 1),
            'forfront': min(min(custom_range[85:95]), 1),
            'fleft': min(min(custom_range[95:110]), 1),
            'left': min(min(custom_range[110:180]), 1.0),
            'left-1': min(min(custom_range[110:150]), 0.75),
        }

        # if right_dist_avg==0 and left_dist_avg==0 and (Count % 2) != 0:
        #     self.state_pub.publish(Int32(data=1))
        #     # print("sequence 1")
        #     Count+=1
        #     self.get_logger().info(f'Count: {Count}')

        # print("regions left max", regions['left'])
        # print("regions right max", regions['right'])
        

    def clbk_odom(self, msg):
        global position_
        global yaw_, active_, n, right_x_lower_limit, left_x_higher_limit, Count, regions

        # position
        position_ = msg.pose.pose.position
        if n == 0:
            if position_.x < right_x_lower_limit:
                active_ = True
            elif position_.x >= right_x_lower_limit:
                active_ = False
                n = None
            # elif (max(regions['left'])==1.0 and max(regions['right']==1.0)):
            #     active_ = False
            #     n = None

        if n is None:
            if left_x_higher_limit < position_.x < right_x_lower_limit:
                active_ = True
            elif position_.x <= left_x_higher_limit or position_.x >= right_x_lower_limit:
                active_ = False
            # elif (max(regions['left'])==1.0 and max(regions['right']==1.0)):
            #     active_ = False

        if position_.x >= (right_x_lower_limit - 0.1) and (Count % 2) != 0:
            self.state_pub.publish(Int32(data=1))
            # print("sequence 1")
            msg = String()
            msg.data='stop+1'
            self.stop_pub.publish(msg)
            Count+=1
            self.get_logger().info(f'Count: {Count}')

        # if (max(regions['left'])==1.0 and max(regions['right']==1.0)):
        #     if (Count % 2) != 0:
        #         self.state_pub.publish(Int32(data=1))
        #         # print("sequence 1")
        #         Count+=1
        #         self.get_logger().info(f'Count: {Count}')

        if position_.x <= (left_x_higher_limit + 0.1) and (Count % 2) == 0:
            self.state_pub.publish(Int32(data=2))
            # print("sequence 2")
            self.get_logger().info(f'Count: {Count}')

        # if (max(regions['left'])==1.0 and max(regions['right']==1.0)):
        #     if (Count % 2) == 0:
        #         self.state_pub.publish(Int32(data=2))
        #         # print("sequence 2")
        #         Count+=1
        #         self.get_logger().info(f'Count: {Count}')

        # yaw
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        yaw_ = euler[2]

    def front_boxes(self, msg):
        global laser_ranges, laser_angles, X_right, y_right, X_left, y_left, X_left_1, y_left_1, X_right_1, y_right_1
        laser_ranges = msg.ranges

        for i in range(0,len(laser_ranges)):
            laser_angles.append(msg.angle_min + i * msg.angle_increment)

        for j in range(0, len(bp), 2):
            X = []
            y = []

            for i in range(0, len(laser_ranges)):
                pn_x = laser_ranges[i] * math.cos(laser_angles[i])
                pn_y = laser_ranges[i] * math.sin(laser_angles[i])

                if (math.isinf(pn_x)==False and math.isinf(pn_y)==False and bp[j][0]<pn_x<bp[j+1][0] and bp[j][1]<pn_y<bp[j+1][1]):
                    X.append([pn_x])
                    y.append([pn_y])
                else:
                    pass

            X = np.array(X)
            print("number of points detected", X.shape[0])
            y = np.array(y)
            if j == 0:
                X_left = X
                y_left = y

            if j == 2:
                X_right = X
                y_right = y

        # print('number of points detected in left box', X_left.shape[0])
        # print('number of points detected in right box', X_right.shape[0])

        plt.clf()
        plt.ylim([-0.8, 0.8])
        plt.xlim([-0, 1.5])
        plt.grid()
        # plt.title("Ransac lines on the plot!")
        rectangle1 = Rectangle(bp[0], width1, height1, linewidth=1, edgecolor='r', facecolor='none')
        rectangle2 = Rectangle(bp[2], width2, height2, linewidth=1, edgecolor='b', facecolor='none')

        ax = plt.gca()
        ax.add_patch(rectangle1)
        ax.add_patch(rectangle2)

        plt.scatter(X_left, y_left)
        plt.scatter(X_right, y_right)

        plt.draw()
        plt.pause(0.001)

    def goal_callback(self, msg):
        global goal, goal_received, yaw_, yaw_case
        twist_msg = Twist()
        fixed_duration = Duration(seconds=1)
        if not goal_received:
            yaw_case = 1
            self.change_state(0)
            goal = msg
            goal_received = True
            self.get_logger().warn(f"New goal received -16<x<16: ({goal.x}, {goal.y})")
        else:
            self.get_logger().warn("Ignoring new goal, robot is already moving to a goal.")
            self.get_logger().warn(f"Current goal is ({goal.x}, {goal.y})")

    def rotate(self, angular_speed_degree, relative_angle_degree, clockwise):
        global pub

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
            pub.publish(velocity_message)

            t1 = self.get_clock().now().seconds_nanoseconds()[0]
            current_angle_degree = (t1 - t0) * angular_speed_degree
            loop_rate.sleep()

            if current_angle_degree > relative_angle_degree:
                self.get_logger().info("reached")
                break

        velocity_message.angular.z = 0
        pub.publish(velocity_message)

    def set_desired_orientation(self, speed_in_degree, desired_angle_degree):
        global yaw_
        relative_angle_radians = math.radians(desired_angle_degree) - yaw_
        clockwise = 0
        if relative_angle_radians < 0:
            clockwise = 1
        else:
            clockwise = 0
        print("relative_angle_degree: ", math.degrees(relative_angle_radians))
        print("desired_angle_degree: ", desired_angle_degree)
        self.rotate(speed_in_degree, math.degrees(abs(relative_angle_radians)), clockwise)

    def fix_yaw(self, des_pos):
        global yaw_, pub, yaw_precision_, state_, X_left, X_right, X_left_1, X_right_1
        twist_msg = Twist()
        desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
        err_yaw = self.normalize_angle(desired_yaw - yaw_)

        if math.fabs(err_yaw) > yaw_precision_:
            twist_msg.angular.z = 0.05 if err_yaw > 0 else -0.05

        pub.publish(twist_msg)

        # state change conditions
        if math.fabs(err_yaw) <= yaw_precision_:
            self.change_state(1)

    def forward_velocity(self):
        global pub, position_, X_left, X_right
        twist_msg = Twist()
        if position_.x>.5 and position_.x<5.5:
            twist_msg.linear.x = 0.1 #0.1
        else:
            twist_msg.linear.x = 0.07
            print('forward else')

        pub.publish(twist_msg)

        if X_left.shape[0] > 0:
            self.change_state(1)

        if X_right.shape[0] > 0:
            self.change_state(1)

    def avoid_obstacle(self):
        global pub
        global yaw_, pub, yaw_precision_, state_, regions, X_left, X_right, X_left_1, X_right_1
        twist_msg = Twist()
        # rate = self.create_rate(20)

        if X_left.shape[0] > 0:
            #while X_left.shape[0] > 0:
            twist_msg.linear.x = 0.055 #0.1
            twist_msg.angular.z = 0.06
            # self.get_logger().info("points found on Left" )

            pub.publish(twist_msg)
            # self.get_logger().info('spinning in left')
            # rate.sleep()
            time.sleep(0.1)
            self.change_state(0)

        elif X_right.shape[0] > 0:
            #while X_right.shape[0] > 0:
            twist_msg.linear.x = 0.055 #0.1
            twist_msg.angular.z = -0.06
            # self.get_logger().info('points found on Right')
            pub.publish(twist_msg)
            # self.get_logger().info('spining in right')
            # rate.sleep()
            time.sleep(0.1)
            self.change_state(0)
            
        # elif regions['right-1']< 0.28:
        #     while regions['right-1']<=0.3:
        #         twist_msg.angular.z = 0.3 
        #         self.get_logger().info('close to the right-1')
        #         pub.publish(twist_msg)
        #         if regions['front']>.8 and regions["left-1"]>0.5:
        #             break
        #         time.sleep(0.1)
        #         self.change_state(0)
        
        # elif regions['left-1']< 0.28: 
        #     while regions['left-1']<=0.3:
        #         twist_msg.angular.z = -0.3 
        #         self.get_logger().info('close to the left-1')
        #         pub.publish(twist_msg)
        #         if regions['front']>.8 and regions["right-1"]>0.5:
        #             break
        #         time.sleep(0.1)
        #         self.change_state(0)
     
    def done(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        pub.publish(twist_msg)

    def change_state(self, state):
        global state_
        state_ = state

    def normalize_angle(self, angle):
        if math.fabs(angle) > math.pi:
            angle = angle - (2 * math.pi * angle) / math.fabs(angle)
        return angle

    def loop(self):
        global pub, goal_received, active_, position_, yaw_case

        if position_.x > 5.5:
            self.state_pub.publish(Int32(data=1))
            # print("loop")


        if not active_:
            print('not active')
            goal_received = False
            return
        else:
            if state_ == 0:
                self.forward_velocity()
                print('state 0')
            elif state_ == 1:
                self.avoid_obstacle()
                print('state 1')
            else:
                self.get_logger().error('Unknown state!')


def main(args=None):
    rclpy.init(args=args)
    node = RowNavigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
