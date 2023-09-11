# RasPi_LiDAR
# This is an example ROS2 node to acquire pointcloud from Velodyne LiDAR and secondary goal to visualize it in RViz.


This example implements Velodyne VLP 16 LiDAR in  ROS2 Humble OS operating Ubuntu 22.04 LTS.
There are some pre-setup to connect the Velodyne LiDAR to the computer using Ethernet cable in ROS2 environment. 

  Pre-Requisites:

  1) Setup the Velodyne LiDAR VLP16 to communicate with the computer running in ROS2. Use the link below to setup the Ethernet connection, which is section 1. Ignore the rest of the sections and continue here to finish the setup. Remember to verify the connection configuration by accessing the LiDAR's network address in any browser.

          http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16
     
  Note: The above step can be replicated to other models of the Velodyne LiDARs with very similar setup (there are also similar libraries/resources available). 
  
  2) Installing ROS drivers for the Velodyne LiDAR using the following command. Assuming that RViz is already installed to the latest version.

          sudo apt-get install ros-humble-velodyne

  3) This is an optional step. Installing the ros2-numpy for viewing/parsing the output  point cloud data in the python node. This is used as a test library to extract and test, although a different custom code modified from the source code of the ros2_numpy is implemented. 

          pip3 install ros2-numpy

Clone this directory as a ros2_ws and build it. 

          git clone https://github.com/Vom1124/RasPi_LiDAR.git && \
          cd RasPi_LiDAR && \
          colcon build --symlink-install
If there exists a ros2 workspace, then simply clone only the package into the exisitng ros2 workspace. 


Once the pre-setup and cloning the workspace are done, open a new terminal and launch the Velodyne LiDAR using the launch file located under /opt/ros/humble/share/velodyne/launch/velodyne-all-nodes-VLP16-launch.py as

          ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py

The available ros topics can be seen using

          ros2 topic list -t
which should now show velodyne_packets for raw data packets and velodyne_points for displaying point cloud.
          
Now, open another terminal window and start the ros2 node  as shown below to acquire the point cloud points and write it to a .txt file. 

          ros2 run velodyne_lidar lidar_read

The Velodyne LiDAR should now successfully run continuously and subsribe to the pointcloud and save it in a text file named "pc_output.txt" under the home directory. 

The output data is printed as range and angles from the spherical coordinates. If one should require raw point cloud data it can be printed in the output text file by modifying the range_data from line 69 from the subscriber "lidar_sub" under the directory "src/velodyne_lidar/velodyne_lidar/lidar_sub". Three different data output is possible: pc2, xyz, and range data. 'pc2' acquires point cloud data including the ring and intensity, whereas 'xyz' simply outputs the x,y, and z coorindates. 'range_data' converts the 'xyz' data to give the range and the angles of projection (alpha and omega from y-axis and z-axis, respectively). So, the line "69" from the file can be set to the variable to achieve desired output data to be saved to a text file.
   


Visualizing the point cloud in RViz:

The point cloud can also be visualized using RViz software with the following steps.

While the velodyne launch file is still running, open a new terminal window and run

        ros2 run rviz2 rviz2 -f velodyne

to start RViz with the velodyne as fixed frame.

Then,
  1) In the "displays" panel, click "Add", then hover to the "By topic" under the pop-up window.
  2) /velodyne_points topic should be avilable (while the velodyne node  is launched and still running) and select PointCloud2 under this topic and hit "OK".
  3)  Kudos! the laser scan points of your environment should now be visible in the visualization map.

        






