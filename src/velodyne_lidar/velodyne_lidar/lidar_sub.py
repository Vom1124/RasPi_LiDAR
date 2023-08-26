import sys 
import time
import rclpy
from rclpy.node import Node
from velodyne_msgs.msg import VelodyneScan
from sensor_msgs.msg import PointCloud2 
from laser_geometry import LaserProjection
import numpy as np

fd = False

class LiDAR(Node):
    
    def __init__(self):
        
        super().__init__('PointCloud')
        self.counter=0
        self.subscribeRawData = self.create_subscription(VelodyneScan, '/velodyne_packets', self.raw_callback, 10)
        head_txt = "\nPoint Cloud Generation Initiated\n"
        fmt_head_txt = head_txt.center(150,'=')
        print("\033[36:4m" + fmt_head_txt)
        
        self.subscribePointCloud = self.create_subscription(PointCloud2, '/velodyne_points', self.pc_callback, 10)
                
        self.subscribePointCloud # preventing unused variable
        
        self.laser_projector = LaserProjection()

    def raw_callback(self, data):

        if self.counter==0:
            raw_data_txt = "Printing Raw Data Packets (This will be displayed only once every time the code is run).\n"
            raw_data_txt_conf = "Finished receiving the raw data packets\n\n"
            self.get_logger().info("\033[93:4m" + raw_data_txt)
            print(data)    
            self.get_logger().info("\033[92:4m" + raw_data_txt_conf)        
            
    def pc_callback(self, pc):
        pc_txt = "Receiving point cloud points and saving ...\n"
        time.sleep(2)
        self.counter+=1
        fd.write(str(pc))
        fd.write("\n\n\n\n ======================================End of one spin:{}===================================\n\n\n".format(self.counter))
        self.get_logger().info("\033[95m" + pc_txt)
        
def main(args=None):
    rclpy.init(args=args)
    node = LiDAR()
    global fd

    fd = open("pc_output.txt","w")
    
    try:
        rclpy.spin(node)
    
    finally:
        rclpy.shutdown()
        fd.close()
        
    

    
if __name__=='__main__':
    main()
