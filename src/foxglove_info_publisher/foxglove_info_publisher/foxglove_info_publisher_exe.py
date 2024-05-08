import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped, Point32, Quaternion
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from a2rl_bs_msgs.msg import VectornavIns
from vectornav_msgs.msg import RtkObserver
import math
import numpy as np
import csv
from collections import deque

ENV = os.environ

class TrackPublisher(Node):
    def __init__(self):
        super().__init__('track_publisher')
        self.path_deque = deque(maxlen=2000)
        self.rtk_deque = deque(maxlen=2000)
        self.ekf_deque = deque(maxlen=2000)
        self.publisher_left = self.create_publisher(PolygonStamped, 'race_track_left', 10)
        self.publisher_right = self.create_publisher(PolygonStamped, 'race_track_right', 10)
        
        self.subscriber_rtk_observer = self.create_subscription(RtkObserver, "/bob_eye/rtk_observer", self.rtk_observer_callback, 1)
        self.publisher_rtk_path = self.create_publisher(Path, 'rtk_path', 10)
        self.publisher_rtk_pose = self.create_publisher(PoseStamped, 'rtk_pose', 10)
        
        self.subscriber_ekf_ins = self.create_subscription(VectornavIns, "/bob_eye/estimate_local", self.ekf_ins_callback, 1)
        self.publisher_ekf_path = self.create_publisher(Path, 'ekf_path', 10)
        self.publisher_ekf_pose = self.create_publisher(PoseStamped, 'ekf_pose', 10)
        
        self.subscriber_vn_ins = self.create_subscription(VectornavIns, "/a2rl/vn/ins", self.vn_ins_sub_callback, 1)
        self.publisher_ego_marker = self.create_publisher(Marker, 'ego_marker', 10)
        self.publisher_ego_pose = self.create_publisher(PoseStamped, 'ego_pose', 10)
        self.publisher_ego_path = self.create_publisher(Path, 'ego_path', 10)
        
        boundary_path = ENV.get("BOUNDARY_DIR")
        self.boundary_left_file = os.path.join(boundary_path, "yas_full_left.csv")
        self.boundary_right_file = os.path.join(boundary_path, "yas_full_right.csv")
        self.timer = self.create_timer(5, self.publish_track)
        
    def ekf_ins_callback(self, msg):
        self.ekf_deque.append((msg.position_enu_ins.x, msg.position_enu_ins.y))
        q = self.quaternion_from_euler(msg.orientation_ypr.z, msg.orientation_ypr.y, msg.orientation_ypr.x)
        path_data = [self.ekf_deque[i] for i in range(0, len(self.ekf_deque), 20)]
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        for (px, py) in path_data:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = float(px)
            pose_stamped.pose.position.y = float(py)
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = 0.0
            pose_stamped.pose.orientation.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            path_msg.poses.append(pose_stamped)
        self.publisher_ekf_path.publish(path_msg)
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = msg.position_enu_ins.x
        pose_msg.pose.position.y = msg.position_enu_ins.y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]
        self.publisher_ekf_pose.publish(pose_msg)
    
    def rtk_observer_callback(self, msg):
        self.rtk_deque.append((msg.position_enu_rtk.x, msg.position_enu_rtk.y))
        q = self.quaternion_from_euler(msg.orientation_ypr_ins.z, msg.orientation_ypr_ins.y, msg.orientation_ypr_ins.x)
        path_data = [self.rtk_deque[i] for i in range(0, len(self.rtk_deque), 20)]
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        for (px, py) in path_data:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = float(px)
            pose_stamped.pose.position.y = float(py)
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = 0.0
            pose_stamped.pose.orientation.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            path_msg.poses.append(pose_stamped)
        self.publisher_rtk_path.publish(path_msg)
        
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = msg.position_enu_rtk.x
        pose_msg.pose.position.y = msg.position_enu_rtk.y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]
        self.publisher_rtk_pose.publish(pose_msg)
        
    def vn_ins_sub_callback(self, msg):
        self.path_deque.append((msg.position_enu_ins.x, msg.position_enu_ins.y))
        marker_msg = Marker()
        marker_msg.header = Header()
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.header.frame_id = "map"
        marker_msg.type = Marker.CUBE
        marker_msg.action = Marker.ADD
        marker_msg.scale.x = 5.23
        marker_msg.scale.y = 1.92
        marker_msg.scale.z = 2.0
        marker_msg.color.a = 1.0
        marker_msg.color.r = 1.0
        marker_msg.color.g = 0.0
        marker_msg.color.b = 0.0
        
        # 设置车辆位置
        marker_msg.pose.position.x = msg.position_enu_ins.x
        marker_msg.pose.position.y = msg.position_enu_ins.y
        marker_msg.pose.position.z = 0.0
        
        # 设置车辆朝向
        q = self.quaternion_from_euler(msg.orientation_ypr.z, msg.orientation_ypr.y, msg.orientation_ypr.x)

        marker_msg.pose.orientation.x = q[0]
        marker_msg.pose.orientation.y = q[1]
        marker_msg.pose.orientation.z = q[2]
        marker_msg.pose.orientation.w = q[3]
        
        self.publisher_ego_marker.publish(marker_msg)
        
        # self.get_logger().info('Published vehicle marker')
        
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = msg.position_enu_ins.x
        pose_msg.pose.position.y = msg.position_enu_ins.y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]
        self.publisher_ego_pose.publish(pose_msg)
        # self.get_logger().info('Published vehicle pose')
        
        path_data = [self.path_deque[i] for i in range(0, len(self.path_deque), 20)]
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        for (px, py) in path_data:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = float(px)
            pose_stamped.pose.position.y = float(py)
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = 0.0
            pose_stamped.pose.orientation.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            path_msg.poses.append(pose_stamped)
        self.publisher_ego_path.publish(path_msg)
        
    
    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quat = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        return q
    def publish_track_spec(self, file_name, publisher):
        polygon_msg = PolygonStamped()
        polygon_msg.header.frame_id = "map"
        
        with open(file_name, 'r') as csvfile:
            csvreader = csv.reader(csvfile)
            for row in csvreader:
                point = Point32()
                point.x = float(row[0])
                point.y = float(row[1])
                point.z = 0.0
                polygon_msg.polygon.points.append(point)
        publisher.publish(polygon_msg)

    def publish_track(self):
        self.publish_track_spec(self.boundary_left_file, self.publisher_left)
        self.publish_track_spec(self.boundary_right_file, self.publisher_right)
        # self.get_logger().info('Published race track polygon')

def main(args=None):
    rclpy.init(args=args)
    track_publisher = TrackPublisher()
    rclpy.spin(track_publisher)
    track_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
