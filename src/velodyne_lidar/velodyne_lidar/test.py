import sys 
import time
import rclpy
from rclpy.node import Node
from velodyne_msgs.msg import VelodyneScan
from sensor_msgs.msg import PointCloud2 
from laser_geometry import LaserProjection
import numpy as np


class LiDAR(Node):
    
    def __init__(self):
        
        super().__init__('PointCloud')
        
        #self.publishPointCloud = self.create_publisher(PointCloud2, '/velodyne_points', 10 )
        #self.subscribeRawData = self.create_subscription(VelodyneScan, '/velodyne_packets', self.raw_callback)
        t=0
        with open("PC.txt", mode="wt") as self.f:
            self.f.truncate(0)
            self.f.write(" %f times, %([t+1]")
            
        self.subscribePointCloud = self.create_subscription(PointCloud2, '/velodyne_points', self.pc_callback, 10)
        self.get_logger().info("Point Cloud Generation Initiated")
        
        self.subscribePointCloud # preventing unused variable
        
        self.laser_projector = LaserProjection()

    def raw_callback(self, data):
        i = 0
        while True:
            self.get_logger().info("Receiving Raw Data\n")
            if i==0:
                print(data)
                i+=1
    def pc_callback(self, pc):
        self.get_logger().info("Receiving point cloud points\n")
        time.sleep(2)
        with open("PC.txt", mode="wt") as self.f:
            self.f.write(str(pc))
            self.f.write("\n\n\n\ntimes000000000000000000000000000...........................................................\n\n\n\n")
        
        self.f.flush
        self.f.close
        
def main(args=None):
    rclpy.init(args=args)
    node = LiDAR()
    rclpy.spin(node)
    rclpy.shutdown()

    
if __name__=='__main__':
    main()
