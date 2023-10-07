#   This code extracts the single laser scan from the 
#       16 laser scan data available.
#  The first input is the point cloud data with different ring counts, 
#       and the second input is the Laser number that is to be extracted.
#  Author : Vomsheendhur (Vom) Raju   

import numpy as np

def single_laserscan(pc2, n):

    xyzir = np.asarray(np.concatenate((pc2.get("xyz").view(dtype=np.float32),
                            pc2.get("intensity").view(dtype=np.float32),
                            pc2.get("ring").view(dtype=np.uint16)), axis=1))
    '''
    Masks to extract individual laser scan data.
    We make use of laser 1 (which has as angle of direction of 1 degree)
    to detect the obstacle in the front.
    '''
   
    m=16 # Total number of rings possible : 0-15 laser scan lines
    mask = np.empty([xyzir.shape[0],m], dtype=bool)
    for i in range(m):
        mask[:,i]= (xyzir[:,4]==i)
        
    xyz = xyzir[mask[:,n],0:3].view(dtype=np.float32)
    return xyz
    
    
    

    
    