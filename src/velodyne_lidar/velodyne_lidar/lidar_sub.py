import os
import sys 
import time
import subprocess 

import rclpy
from rclpy.node import  Node
from velodyne_msgs.msg import VelodyneScan
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection
import numpy as np
#import pandas as pd Make sure to install it in the system before using this
import ros2_numpy as rnp # apt install ros-humble-ros-numpy

# Importing the user-created packages from the ros2 workspace
from velodyne_lidar import laserscan_pc2_to_array 
from velodyne_lidar import velodyne_pc2_to_array
from velodyne_lidar import xyz_to_range_data
from velodyne_lidar import lasers_to_single_laserscan

from example_interfaces.msg import Float32

from std_msgs.msg import String

# Logging in as sudo user
os.system("sudo -k") # First exiting the sudo mode if already in sudo mode
sudoPassword = "123"
os.system("echo '\e[7m \e[91m Logging in as sudo user...\e[0m'")
os.system("echo %s | sudo -i --stdin" %(sudoPassword))
os.system("echo '\n \e[5m \e[32m*Successfully logged in as sudo user!*\e[0m'")
current_username = os.getlogin()

mountStatus = False

class LiDAR(Node): 
    def __init__(self):
        super().__init__('PointCloud')
        
        # Using a counter to subscribe to the rawpacket 
        # -------only once in the callback and counting the number of spin.
        self.counter = 0
        
        # Subscribing to the raw laser scan data from only one laser 
        #    (possibly from the ring number 0 from 0-15 lasers)
        # self.subscribeRawData = self.create_subscription(VelodyneScan, 
                                    # '/velodyne_packets', self.raw_callback, 10)
        
        head_txt = "\nPoint Cloud Generation Initiated\n"
        fmt_head_txt = head_txt.center(150,'=')
        print("\033[36:4m" + fmt_head_txt)

        # Subscribing to the PointCloud2 data from the velodyne transform node

        print("\033[33:4m" + "Waiting for the LiDAR to publish data, please check the connection\033[0m")
        # self.subscribePointCloud = self.create_subscription(PointCloud2, 
                                    # '/velodyne_points', self.pc_callback, 10)    
        # self.subscribePointCloud # preventing unused variable   
        
        
        # -- Creating Publisher to publish the calculated distance 
        self.distance_publisher = self.create_publisher(Float32, "ObstacleDistance", 10)
        
        '''
        Check the LaserScan message type and velodyne point cloud to verify the correct Laser number
        '''
        # self.subscribeLaserScan = self.create_subscription(LaserScan,
        #                             '/scan', self.scan_callback, 10)
        # self.subscribeLaserScan # Preserving the unused variable
        # self.laser_projector = LaserProjection() # Creating an object for laser_projector library  
        
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0      
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        fd.write("Test of {}\n\n".format(msg.data))
        self.i += 1
        
    def raw_callback(self, data):
        '''
        This subscriber gets the raw packets from the LiDAR which gives the information about 
        the field names and datatypes of the raw scan packets
        '''
        #Setting the condition to print this raw data packet only once.
        if self.counter==0:
            raw_data_txt = "Printing Raw Data Packets (This will be displayed only once every time the code is run).\n"
            raw_data_txt_conf = "Finished receiving the raw data packets\n\n"
            self.get_logger().info("\033[93:4m" + raw_data_txt)
            print(data)    
            self.get_logger().info("\033[92:4m" + raw_data_txt_conf)    
            time.sleep(1)    
            
    def pc_callback(self, pc2_msg):
        '''
        This subsccriber is used to get the point cloud data for all the 16 laser scan lines.
        Thos data is saved onto a .txt file to visualize and/or get the entire field of view 
        of the environment.
        '''
        if self.counter==0:
            print("\033[36;5m" + "LiDAR connection successful...\033[0m")

        pc_txt = "Receiving point cloud points from /velodyne_points and saving ...\n"
        self.counter+=1
        
        pc2_data = velodyne_pc2_to_array.pointcloud2_to_array(pc2_msg)
        
        # xyz = pc2.get("xyz")           
        
        #------- Converting point cloud data (x,y & z to range and angles: refer to the manual)
        # pc_to_range_data = xyz_to_range_data.xyz_to_range_and_angle(xyz)
        # column_names = list(pc_to_range_data)
        # range_data = np.concatenate((pc_to_range_data.get("range"),
        #                             pc_to_range_data.get("alpha"),
        #                             pc_to_range_data.get("omega")), axis=1)
        # range_data = pd.DataFrame(range_data, columns=column_names)
        # pd.set_option('display.max_rows', None)# to display all the rows in the DataFrame
        #------------------done converting the point cloud data to range data

        np.set_printoptions(threshold=sys.maxsize)# to display all the points in the array
        fd.write("\n\n XYZIR : {}".format(pc2_data))
        fd.write("\n\n==================================="+
        "===End of one spin:{}===================================\n\n\n".format(self.counter))
        self.get_logger().info("\033[95m" + pc_txt)
        time.sleep(0.1)
        #------------Extracting the depth of the obstacle from 
        # the Laser #14 at -1 angle of increment.
        xyz_nav =  lasers_to_single_laserscan.single_laserscan(pc2_data, 14) 
        distance_msg = Float32()
        distance_to_obstacle = np.mean(xyz_nav[:,0]).astype('float')
        distance_msg.data = distance_to_obstacle
        time.sleep(0.5)
        
        distance_txt = "Obstacle Distance calculated and publishing to example_interface topic"
        self.get_logger().info("\033[95m" + distance_txt)
        self.distance_publisher.publish(distance_msg)
        
                
    def scan_callback(self, scan_msg):
        '''
        This subscriber is used to get the Laser Scan from the origin at 0 degree omega angle.
        The X-coordinate from this laser scan will be used to navigate as the x points from this point cloud 
        will yield the distance of the obstacle in front of the LiDAR.
        '''
        scan_txt = "Received the Laser Scan Message from /scan and converting the scan to pointcloud"
        cloud = self.laser_projector.projectLaser(scan_msg)
        pc2 = laserscan_pc2_to_array.pointcloud2_to_array(cloud)
        time.sleep(1)
        fd.write(" Converted cloud points{} \n\n".format(pc2))
        fd.write("\n" + str(pc2))
        
        self.get_logger().info("\033[95m" + scan_txt)
        time. sleep(1)
        
def pc_writer():
    '''
    This section of code helps to create a file inside the USB drive inserted...
    Requires to create a mount point name and then unmount existing drives and re-mount the drive...
    '''

    isMountsda = os.path.exists("/dev/sda1")
    isMountsdb = os.path.exists("/dev/sdb1")
    isMountsdc = os.path.exists("/dev/sdc1")
    isMountsdd = os.path.exists("/dev/sdd1")

    print("sda status" + str(isMountsda) + "\nsdb status" + \
        str(isMountsdb) + "\nsdc status" + str(isMountsdc) + \
             "\nsdd status" + str(isMountsdd))
    
    if isMountsda==True or isMountsdb==True or isMountsdc==True:   
        mountStatus = True
        
        #Removing/Unmounting (clearing) already existing mountpoint to avoid overlap in the mount status        
        os.system("sudo umount -f /dev/sd* > /dev/null  2>&1") # the output will be null.       
        
        #Checking if mount point name already exists (Need to create only on the first run).
        isMountPointName = os.path.exists("/media/Velodyne_LiDAR")

        os.system("sudo chown %s:%s /media/"%(current_username,current_username))
        os.system("sudo chown %s:%s /dev/sd*"%(current_username,current_username))
            
        if isMountPointName==True:
            try:
                os.system("sudo rm -r /media/*")
                os.system("mkdir /media/Velodyne_LiDAR") # Creating a mount point name
            except:
                pass
        elif isMountPointName==False:      
            os.system("mkdir /media/Velodyne_LiDAR") # Creating a mount point name
        '''
        The order of checking the mount is reversed to ensure that there 
        is no problem mounting with already preserved mountpoints by the system.
        For example, if sda is already mounted by the system for some port address, then the access to 
        mount the sda for USB drive won't exist. So, the further options will be checked, by in the mean time, the sda in the 
        alphabetical order will throw an error and stop the code. Therefore, the mount check is initiated with sdc.
        Only three /dev/sd* are used, as atmost three ports will be used simultaneously. 
        '''
        if isMountsdd:
            mountCommand = "sudo mount /dev/sdd1 /media/Velodyne_LiDAR -o umask=022,rw,uid=1000,gid=1000"
        elif isMountsdc:
            mountCommand = "sudo mount /dev/sdc1 /media/Velodyne_LiDAR -o umask=022,rw,uid=1000,gid=1000"   
        elif isMountsdb:
            mountCommand = "sudo mount /dev/sdb1 /media/Velodyne_LiDAR -o umask=022,rw,uid=1000,gid=1000"
        
        elif isMountsda:
            mountCommand = "sudo mount /dev/sda1 /media/Velodyne_LiDAR -o umask=022,rw,uid=1000,gid=1000"
        os.system(mountCommand)    
        
    else:
        return
    
        
    

def main(args=None):
    rclpy.init(args=args)
    global fd # Assigning the output file data name to be global so that the node can access it.
    global mountStatus
    mountStatus=False
    node = LiDAR()
    
    pc_writer() # Running this method to mount the USB drive properly.
    
    if mountStatus==True:
        os.system("echo '\e[33mINFO: Mount status success: a USB drive is found."\
        "The point cloud data will be saved to the inserted USB.\e[0m'")
        fd = open("/media/Velodyne_LiDAR/pc_data.txt","wt") # Creating the actual file 
    else:
        os.system("echo '\e[33mINFO: Mount status FAILURE: no USB is inserted."\
        "The point cloud data will be saved to the home directory.\e[0m'")
        fd = open("pc_data.txt","wt") # Creating the actual file 
    
    rclpy.spin(node)    	
    rclpy.shutdown()
    fd.close()

if __name__=='__main__':
    main()

    	
