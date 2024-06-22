#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import math
import matplotlib
matplotlib.use('GTK3Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped, Point, PoseStamped, Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformListener, Buffer, LookupException, ConnectivityException, ExtrapolationException
from std_msgs.msg import Int32
import collections
import tf2_geometry_msgs
from std_msgs.msg import Int32, Bool, Float64
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty


bp = [[-0.2, 0.15], [1.90, 0.60], [-0.2, -0.60], [1.90, -0.15]]
width1 = bp[1][0] - bp[0][0]
height1 = bp[1][1] - bp[0][1]
width2 = bp[3][0] - bp[2][0]
height2 = bp[3][1] - bp[2][1]

class LidarRansacNode(Node):
    def __init__(self):
        super().__init__('lidar_ransac_transform')
        
        self.srv = self.create_service(Empty, 'trigger_ransac', self.ransac_callback)
        self.sub_laser_1 = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.sub_laser_2 = self.create_subscription(LaserScan, '/scan', self.range_filter, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odometry/filtered', self.clbk_odom, 10)
        # self.state_sub = self.create_subscription(Int32, 'state1', self.state_callback, 10)
        self.pub_slope_number = self.create_publisher(Int32, 'slope_number', 10)
        self.state_pub = self.create_publisher(Int32, 'state', 10)
        
        # self.pub_mid_point_goal = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.pub_mid_point_goal = self.create_publisher(Point, 'mid_point_goal', 10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.previous_midpoints = collections.deque(maxlen=10) 

        self.position_ = Point()
        self.pose_msg = Point()
        self.yaw_ = 0
        self.num_of_slope_ = 1

        self.right_dist_avg = 0
        self.left_dist_avg = 0

        self.laser_angles = []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.distance_pub = self.create_publisher(Float64, 'distance_update', 10)
        self.subscription = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
        self.laser_ranges = None
        self.angle_min = None
        self.transform_available = False
        self.timer_for_transform = self.create_timer(0.1, self.wait_for_transform)

        self.timer = self.create_timer(0.4, self.timer_callback)

        self.current_orientation = 0.0
        self.target_orientation = 0.0

    def normalize_angle(self, angle):
        # Normalize angle to be within the range [-pi, pi]
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    

    def imu_callback(self, msg):
        # Extract the yaw angle from the quaternion
        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        _, _, yaw = self.euler_from_quaternion(quaternion)
        self.current_orientation = yaw

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

    def stop_robot(self):
        twist = Twist()
        self.publisher_.publish(twist)

   
    def wait_for_transform(self):
        try:
            self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            self.transform_available = True
            self.timer_for_transform.cancel()
            self.get_logger().info('Transform between "odom" and "base_link" is now available.')
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn('Waiting for transform between "odom" and "base_link"...')
        
    def moving_average(self, new_point):
        self.previous_midpoints.append(new_point)
        if len(self.previous_midpoints) == 0:
            return new_point
        avg_x = sum(p[0] for p in self.previous_midpoints) / len(self.previous_midpoints)
        avg_y = sum(p[1] for p in self.previous_midpoints) / len(self.previous_midpoints)
        return (avg_x, avg_y)

    def clbk_odom(self, msg):
        self.position_ = msg.pose.pose.position
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.yaw_ = euler[2]

    def range_filter(self, msg):
        ranges = [x for x in msg.ranges if not math.isnan(x)]
        custom_range = ranges[270:360] + ranges[0:90]
        right_dist = [x for x in custom_range[0:70] if x < 0.85]
        left_dist = [x for x in custom_range[110:180] if x < 0.85]
        
        if len(right_dist) == 0 and len(left_dist) == 0:
            self.right_dist_avg = 0
            self.left_dist_avg = 0
        elif len(right_dist) == 0:
            self.right_dist_avg = 0
            self.left_dist_avg = sum(left_dist) / len(left_dist)
        elif len(right_dist) > 0 and len(left_dist) == 0:
            self.right_dist_avg = sum(right_dist) / len(right_dist)
            self.left_dist_avg = 0
        else:
            self.right_dist_avg = sum(right_dist) / len(right_dist)
            self.left_dist_avg = sum(left_dist) / len(left_dist)

    def fit_with_least_squares(self, X, y):
        b = np.ones((X.shape[0], 1))
        A = np.hstack((X, b))
        theta = np.linalg.lstsq(A, y, rcond=None)[0]
        return theta

    def evaluate_model(self, X, y, theta, inlier_threshold):
        b = np.ones((X.shape[0], 1))
        y = y.reshape((y.shape[0], 1))
        A = np.hstack((y, X, b))
        theta = np.insert(theta, 0, -1.)

        distances = np.abs(np.sum(A * theta, axis=1)) / np.sqrt(np.sum(np.power(theta[:-1], 2)))
        inliers = distances <= inlier_threshold
        num_inliers = np.count_nonzero(inliers == True)

        return num_inliers

    def ransac(self, X, y, max_iters=60, samples_to_fit=3, inlier_threshold=0.35, min_inliers=7):
        best_model = None
        best_model_performance = 0
        num_samples = X.shape[0]

        for i in range(max_iters):
            sample = np.random.choice(
                num_samples, size=samples_to_fit, replace=False)
            if len(X[sample]) == 0 or len(y[sample]) == 0:
                break
            model_params = self.fit_with_least_squares(X[sample], y[sample])
            model_performance = self.evaluate_model(
                X, y, model_params, inlier_threshold)

            if model_performance < min_inliers:
                continue

            if model_performance > best_model_performance:
                best_model = model_params
                best_model_performance = model_performance

        return best_model
    
    def ransac_callback(self, request, response):
        if self.transform_available:
            self.perform_ransac()
        else:
            self.get_logger().warn("Transform not yet available, skipping RANSAC.")
        return response
    
    def perform_ransac(self):

        laser_ranges = self.laser_ranges
        lines_m = []
        lines_b = []
        X_all = []
        y_all = []
        
        for i in range(len(laser_ranges)):
            self.laser_angles.append(self.angle_min + i * self.angle_increment)

        for j in range(0, len(bp), 2):
            X = []
            y = []
            for i in range(len(laser_ranges)):
                pn_x = laser_ranges[i] * math.cos(self.laser_angles[i])
                pn_y = laser_ranges[i] * math.sin(self.laser_angles[i])
                if (not math.isinf(pn_x) and not math.isinf(pn_y) and 
                    bp[j][0] < pn_x < bp[j+1][0] and bp[j][1] < pn_y < bp[j+1][1]):
                    X.append([pn_x])
                    y.append([pn_y])
            X = np.array(X)
            y = np.array(y)
            if (X.shape[0] <= 2) or (y.shape[0] <= 2):
                continue
            result = self.ransac(X, y)
            if result is None:
                continue
            X_all.append(X)
            y_all.append(y)
            m = result[0][0]
            b = result[1][0]
            lines_m.append(m)
            lines_b.append(b)

        plt.clf()
        plt.ylim([-1, 1])
        plt.xlim([-1, 3])
        plt.grid()
        plt.title("Ransac lines on the plot!")
        rectangle1 = Rectangle(bp[0], width1, height1, linewidth=1, edgecolor='r', facecolor='none')
        rectangle2 = Rectangle(bp[2], width2, height2, linewidth=1, edgecolor='b', facecolor='none')
        ax = plt.gca()
        ax.add_patch(rectangle1)
        ax.add_patch(rectangle2)
        for i in range(len(X_all)):
            plt.scatter(X_all[i], y_all[i])
            for m, b in zip(lines_m, lines_b):
                plt.plot(X_all[i], m * X_all[i] + b, 'r', linewidth=5)

        p = 1
        self.num_of_slope_ = len(lines_m)
        mid_point_y = 0.0
        mid_point_base = (0.0, 0.0)
        if len(lines_m) == 2:
            m1 = lines_m[0]
            b1 = lines_b[0]
            y1 = p * m1 + b1
            m2 = lines_m[1]
            b2 = lines_b[1]
            y2 = p * m2 + b2
            mid_point_y = (y1 + y2) / 2
            mid_point_base = [p, mid_point_y]
        elif len(lines_m) == 1:
            current_dist_1 = math.fabs(lines_b[0] / (math.sqrt((lines_m[0]**2) + 1)))
            m1 = lines_m[0]
            b1 = lines_b[0]
            if current_dist_1 > 0.3:
                mid_point_y = m1 * p
                mid_point_base = [p, mid_point_y]
            else:
                mid_point_y = m1 * p + (-b1)
        elif len(lines_m) == 0 and 0.5 < self.position_.x < 9.8:
            if self.right_dist_avg < self.left_dist_avg:
                mid_point_y = 0.2
                p = 0.75
            elif self.right_dist_avg > self.left_dist_avg:
                mid_point_y = -0.2
                p = 0.75

        if len(lines_m) > 0 or (len(lines_m) == 0 and 0.5 < self.position_.x < 9.5):
            smoothed_mid_point = self.moving_average(mid_point_base)
            plt.scatter(smoothed_mid_point[0], smoothed_mid_point[1])
            plt.draw()
            plt.pause(0.001)
            point = PointStamped()
            point.header.frame_id = 'base_link'
            point.point.x = float(smoothed_mid_point[0])
            point.point.y = float(smoothed_mid_point[1])
            point.point.z = 0.0
            try:
                point_odom = self.tf_buffer.transform(point, 'odom', rclpy.time.Duration(seconds=1.0))           
                self.pose_msg.x = point_odom.point.x
                self.pose_msg.y = point_odom.point.y
                self.pose_msg.z = 0.0
                
                self.pub_mid_point_goal.publish(self.pose_msg)
                self.distance_pub.publish(Float64(data=self.position_.x))
                self.get_logger().info("Following R")

                    
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().error(f"Could not transform point: {e}")
        

    def timer_callback(self):
        # self.pub_slope_number.publish(Int32(data=self.num_of_slope_))
        # self.get_logger().info(f'Number of slopes: {self.num_of_slope_}')
        # self.get_logger().info(f'Position x: {self.position_.x}')
        # self.get_logger().info(f'Position y: {self.position_.y}')
        # self.get_logger().info(f'Yaw: {self.yaw_ * 180 / math.pi}')
        pass
    
    def scan_callback(self, msg):
        self.laser_ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment
                     
def main(args=None):
    rclpy.init(args=args)
    node = LidarRansacNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()