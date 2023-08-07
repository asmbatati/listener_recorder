import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import pandas as pd
from datetime import datetime
import csv

class PoseStampedCSVWriter(Node):
    def __init__(self):
        super().__init__('csv_writer')
        self.subscription = self.create_subscription(PoseStamped,'/target/px4_ros/pose',self.listener_callback,10)
        self.subscription  # prevent unused variable warning

        # Prepare header for csv
        self.header = ['timestamp', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw']
        # Open file in append mode as csvfile
        self.csvfile = open('pose_data.csv', 'a', newline='')
        self.writer = csv.writer(self.csvfile)
        # Write header to csv file
        self.writer.writerow(self.header)

    def listener_callback(self, msg):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9  # convert to seconds
        tx = msg.pose.position.x
        ty = msg.pose.position.y
        tz = msg.pose.position.z
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        # Write data to csv
        self.writer.writerow([timestamp, tx, ty, tz, qx, qy, qz, qw])

def main(args=None):
    rclpy.init(args=args)

    csv_writer = PoseStampedCSVWriter()

    rclpy.spin(csv_writer)

    csv_writer.csvfile.close()  # Close csv file when done
    csv_writer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
