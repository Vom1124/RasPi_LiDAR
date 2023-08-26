# RasPi_LiDAR
# This is an example ROS2 node to acquire pointcloud from Velodyne LiDAR. 

This example implements Velodyne VLP 16 LiDAR in  ROS2 Humble OS operating Ubuntu 22.04 LTS.
There are some pre-setup to connect the Velodyne LiDAR to the computer using Ethernet cable in ROS2 environment. 

Pre-Requisites:

  1) Setup the Velodyne LiDAR VLP16 to communicate with the computer running in ROS2. Use the link below to setup the Ethernet connection, which is section 1. Ignore the rest of the sections and continue here to finish the setup. Remember to verify the connection configuration by accessing the LiDAR's network address in any browser.

          http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16
     
  Note: The above step can be replicated to other models of the Velodyne LiDARs with very similar setup (there are also similar libraries/resources available). 
  
  2) Installing ROS drivers for the Velodyne LiDAR using the following command. Assuming that RViz is already installed to the latest version.

          sudo apt-get install ros-humble-velodyne


