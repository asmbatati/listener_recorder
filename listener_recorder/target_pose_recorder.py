import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import pandas as pd
import csv

class PoseStampedCSVWriter(Node):
    def __init__(self):
        super().__init__('pose_to_csv')
        self.subscription = self.create_subscription(PoseStamped,'/target/px4_ros/pose',self.listener_callback,10)
        self.subscription  # prevent unused variable warning

        # Prepare header for csv
        self.header = ['timestamp', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw']

        self.data_list = []
        self.first_timestamp = None

    def listener_callback(self, msg):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9  # convert to seconds
        if self.first_timestamp is None:
            self.first_timestamp = timestamp
        timestamp -= self.first_timestamp  # starting from 0
        
        tx = msg.pose.position.x
        ty = msg.pose.position.y
        tz = msg.pose.position.z
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        
        self.data_list.append([timestamp, tx, ty, tz, qx, qy, qz, qw])

    def write_to_csv(self):
        df = pd.DataFrame(self.data_list, columns=self.header)

        # Convert timestamp column to TimedeltaIndex
        df['timestamp'] = pd.to_timedelta(df['timestamp'], unit='s')
        df.set_index('timestamp', inplace=True)

        # Resampling
        df_resampled = df.resample('100ms').first().interpolate()  # 0.1s = 100ms
        df_resampled.reset_index(inplace=True)

        # Convert TimedeltaIndex back to seconds for the CSV output
        df_resampled['timestamp'] = df_resampled['timestamp'].dt.total_seconds()

        # Write data to CSV
        df_resampled.to_csv('/home/user/shared_volume/ros2_ws/src/listener_recorder/recordings/R1_df_resampled.csv', index=False)

def main(args=None):
    rclpy.init(args=args)

    pose_to_csv = PoseStampedCSVWriter()

    try:
        rclpy.spin(pose_to_csv)
    except KeyboardInterrupt:
        pass
    finally:
        pose_to_csv.write_to_csv()  # Write to csv once node is terminated
        pose_to_csv.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
