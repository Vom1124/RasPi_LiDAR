# RasPi_LiDAR
# This is an example ROS2 node to acquire pointcloud from Velodyne LiDAR and publish the resulting distance to the obstacle with a secondary goal to visualize it in RViz.


This example implements Velodyne VLP 16 LiDAR in  ROS2 Humble OS operating Ubuntu 22.04 LTS.
There are some pre-setup to connect the Velodyne LiDAR to the computer using Ethernet cable in ROS2 environment. 

 ##### Pre-Requisites:

  1) Setup the Velodyne LiDAR VLP16 to communicate with the computer running in ROS2. Use the link below to setup the Ethernet connection, which is section 1. Ignore the rest of the sections and continue here to finish the setup. Remember to verify the connection configuration by accessing the LiDAR's network address in any browser.

          http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16
     Note: The above step can be replicated to other models of the Velodyne LiDARs with very similar setup (there are also similar libraries/resources available). 
  
  2) Installing ROS drivers for the Velodyne LiDAR using the following command. Assuming that RViz is already installed to the latest version.

          sudo apt-get install ros-humble-velodyne

  3) This is an optional step. Installing the ros2-numpy for viewing/parsing the output  point cloud data in the python node. This is used as a test library to extract and test, although a different custom code modified from the source code of the ros2_numpy is implemented. 

          pip3 install ros2-numpy

##### Setting Up:  
   Clone this directory as a ROS2 workspace and build it.
   
          
            git clone https://github.com/Vom1124/RasPi_LiDAR.git && \
            cd RasPi_LiDAR && \
            colcon build --symlink-install

          
If there exists a ros2 workspace, then simply clone only the package into the exisitng ros2 workspace. 

The LiDAR parameters can be varied as desired in the VLP16-velodyne_transform_node-params.yaml file located under "/opt/ros/humble/velodyne_pointcloud/config" folder. The current settings chosen during this test was

        velodyne_transform_node:
            ros__parameters:
                calibration: VLP16db.yaml
                model: VLP16
                min_range: 0.1
                max_range: 20.0
                view_direction: 0.0
                view_width: 0.18
                fixed_frame: ""
                target_frame: ""
                organize_cloud: true


Launching/Starting the nodes: 

Once the pre-setup and cloning the workspace are done, open a new terminal and launch the Velodyne LiDAR using the launch file located under /opt/ros/humble/share/velodyne/launch/velodyne-all-nodes-VLP16-launch.py as

          ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py

The available ros topics can be seen using

          ros2 topic list -t
which should now show velodyne_packets for raw data packets, velodyne_points for displaying point cloud, and ObstacleDetection for getting the distance data from the obstacle(s) in the front.
          
Now, open another terminal window and start the ros2 node as shown below to acquire the point cloud points and write it to a .txt file. 

          ros2 run velodyne_lidar lidar_read

The Velodyne LiDAR should now successfully run continuously and subsribe to the pointcloud and save it in a text file named "range_data.txt" under the current directory from where the node is run. Along with it, the obstacle distance from the front of the LiDAR should be published to the ObstacleDistance topic, which can be subsribed for navigation.

Note: The output data is printed as point cloud message with {xyz, intensity, ring}. If one should require raw range and angles data format it can be printed in the output text file by replacing the "pc2_data" from line #100 with "range_data" from the subscriber "lidar_sub" under the directory "src/velodyne_lidar/velodyne_lidar/" after uncommenting the range_data section from lines #89-#97. Three different data output is possible: pc2, xyz, and range data. The aforementioned step can be perfmed if ne needs the xyzdata instead pc2 data. 'pc2' acquires point cloud data including the ring and intensity, whereas 'xyz' simply outputs the x,y, and z coorindates. 'range_data' converts the 'xyz' data to give the range and the angles of projection (alpha and omega from y-axis and z-axis, respectively). So, the line #100 from the file can be set to the variable of interest to achieve desired output data to be saved to the text file.

Writing the output data to a text file in a USB drive requires sudo login to automatically mount the USB, and thus, the code in the lidar_sub under "src/velodyne_lidar/velodyne_lidar/" has the SudoPassword (line #135 under the pc_writer method) variable which needs to be changed according to the user's system password. 
   

Visualizing the point cloud in RViz:

The point cloud can also be visualized using RViz software with the following steps.

While the velodyne launch file is still running, open a new terminal window and run

        ros2 run rviz2 rviz2 -f velodyne

to start RViz with the velodyne as fixed frame.

Then,
  1) In the "displays" panel, click "Add", then hover to the "By topic" under the pop-up window.
  2) /velodyne_points topic should be avilable (while the velodyne node  is launched and still running) and select PointCloud2 under this topic and hit "OK".
  3)  Kudos! the laser scan points of your environment should now be visible in the visualization map.

        






