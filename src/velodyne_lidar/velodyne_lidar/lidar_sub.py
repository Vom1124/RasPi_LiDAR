import sys 
import time
import rclpy
from rclpy.node import  Node
from velodyne_msgs.msg import VelodyneScan
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection
import numpy as np
import pandas as pd
import ros2_numpy as rnp # apt install ros-humble-ros-numpy

# Importing the user-created packages from the ros2 workspace
from velodyne_lidar import pc2_to_array 
from velodyne_lidar import pc_to_range

import ctypes
import struct

fd = False


class LiDAR(Node):
    
    def __init__(self):
        
        super().__init__('PointCloud')
        
        self.counter=0 # Using this counter to subscribe to the rawpacket only once in the callback.
        self.subscribeRawData = self.create_subscription(VelodyneScan, '/velodyne_packets', self.raw_callback, 10)
        
        head_txt = "\nPoint Cloud Generation Initiated\n"
        fmt_head_txt = head_txt.center(150,'=')
        print("\033[36:4m" + fmt_head_txt)
        
        # Subscribing to the PointCloud2 data from the velodyne transform node
        self.subscribePointCloud = self.create_subscription(PointCloud2, '/velodyne_points', self.pc_callback, 10)    
        self.subscribePointCloud # preventing unused variable
        
        # Subscribing to the raw laser scan data from only one laser (possibly from the ring number 0 from 0-15 lasers)
        # See the Velodyne VLP16 manual to check for the laser's ring number.
        # self.subscribeLaserScan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        # self.subscribeLaserScan # Preserving the unused variable
        
        #self.laser_projector = LaserProjection()
        
    def raw_callback(self, data):
        #Setting the condition to print this raw data packet only once.
        if self.counter==0:
            raw_data_txt = "Printing Raw Data Packets (This will be displayed only once every time the code is run).\n"
            raw_data_txt_conf = "Finished receiving the raw data packets\n\n"
            self.get_logger().info("\033[93:4m" + raw_data_txt)
            print(data)    
            self.get_logger().info("\033[92:4m" + raw_data_txt_conf)    
            time.sleep(1)    
            
    def pc_callback(self, pc2_msg):
        pc_txt = "Receiving point cloud points and saving ...\n"
        self.counter+=1
        pc2 = pc2_to_array.pointcloud2_to_array(pc2_msg)
        xyz = pc2.get("xyz")        
        pc_to_range_data = pc_to_range.xyz_to_range_and_angle(xyz)
        column_names = list(pc_to_range_data)
        range_data = np.concatenate((pc_to_range_data.get("range"),
                                    pc_to_range_data.get("alpha"),
                                    pc_to_range_data.get("omega")), axis=1)
        range_data = pd.DataFrame(range_data, columns=column_names)
        pd.set_option('display.max_rows', None)# to display all the rows in the DataFrame
        # np.set_printoptions(threshold=sys.maxsize)# to display all the points in the array
        fd.write("\n\n" + str(range_data)+"\n")
        fd.write("\n\n\n\n ======================================End of one spin:{}===================================\n\n\n".format(self.counter))
        self.get_logger().info("\033[95m" + pc_txt)
        time.sleep(2)
        
    def scan_callback(self, scan_msg):
        self.counter+=1
        scan_txt = "Received the Laser Scan Message"
        cloud = self.laser_projector.projectLaser(scan_msg)
        pc2 = rnp.numpify(cloud)
        time.sleep(1)
        fd.write(" Formatted points {} \n\n". format(pc2))
        fd.write("\n" + str(cloud))
        fd.write("\n\n\n ++++++++++++++++++++++++++++++++++++ End of spin ++++++++++++++++++++++++++++++++++ \n\n\n")
        self.get_logger().info("\033[95m" + scan_txt)
        time. sleep(3)
     
     
     
def main(args=None):
    rclpy.init(args=args)
    node = LiDAR()
    global fd

    current_directory = os.getcwd()
    fd = open(current_directory + "/range_data.txt","w")
    
    rclpy.spin(node)
    
    rclpy.shutdown()
    fd.close()
        
    

    
if __name__=='__main__':
    main()
