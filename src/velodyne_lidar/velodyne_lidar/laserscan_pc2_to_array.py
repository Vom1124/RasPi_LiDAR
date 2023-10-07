# This is a ros2 code to work with PointCloud2 data structure. 
# This is a modified source code from ros2_numpy package to work with the 
    # desired application using Velodyne VLP 16 LiDAR.
#The source code is the file name point_cloud2.py under the directory 
    # "~/.local/lib/python3.10/site-packages/ros2_numpy"

# Author: Vomsheendhur (Vom) Raju

from sensor_msgs.msg import PointCloud2, PointField
import numpy as np


def pointcloud2_to_array(cloud_msg):
    ''' Convert a sensor_msgs/PointCloud2 message to a NumPy array. 
    Using the datatypes information to identify the each field's datatype.
    
    datatypes = [(PointField.INT8, np.dtype('int8')),
                 (PointField.UINT8, np.dtype('uint8')),
                 (PointField.INT16, np.dtype('int16')),
                 (PointField.UINT16, np.dtype('uint16')),
                 (PointField.INT32, np.dtype('int32')),
                 (PointField.UINT32, np.dtype('uint32')),
                 (PointField.FLOAT32, np.dtype('float32')),
                 (PointField.FLOAT64, np.dtype('float64'))]
    '''
    n_len = cloud_msg.point_step # Length of the entire data (total columns of data to be extracted)
    # Convert the PointCloud2 message to a NumPy 2-D array 
    pc2_data = np.frombuffer(cloud_msg.data, 
                             dtype=np.uint8).reshape(-1, n_len)
    
    '''Checking the field data according to the LiDAR data. The fields
    in the PointCloud2 message are mapped to the fields in the NumPy array
    as follows:
    [sensor_msgs.msg.PointField(name='x', offset=0, datatype=7, count=1),
    sensor_msgs.msg.PointField(name='y', offset=4, datatype=7, count=1),
    sensor_msgs.msg.PointField(name='z', offset=8, datatype=7, count=1), 
    sensor_msgs.msg.PointField(name='intensity', offset=12, datatype=7, count=1), 
    sensor_msgs.msg.PointField(name='index', offset=16, datatype=5, count=1)]

    '''
    field_data = cloud_msg.fields
    # print(field_data)
    
    ''' Checking the field names. 
    Should be  ['x', 'y', 'z', 'intensity', 'index']
    '''
    
    field_names = [f.name for f in field_data]
    #print(field_names)
        
    # Using the offset values from the "field_data"...
        # to extract the respective arrays
    xyz = pc2_data[:, 0:12].view(dtype=np.float32).reshape(-1,3)
    intensity = pc2_data[:, 12:16].view(dtype=np.float32)
    index = pc2_data[:,16:cloud_msg.point_step].view(dtype=np.int32)

    return {"xyz" : xyz, "intensity": intensity, "index": index }

    