#   This code implements the velodyne's spherical coordinate system
#   to convert the point cloud x,y, and z data to range and angle 
#   from the origin 

#    Author : Vomsheendhur (Vom) Raju   

import numpy as np
import sympy  
from math import sin, asin, cos, acos, tan, atan
import math

def xyz_to_range_and_angle(xyz_array):
    
    '''
    Convert the xyz array from point cloud data to 
    range and angles for each point from the origin.
    Equation used from spherical coordinates:
    X = R * cos(omega) * sin(alpha)
    Y = R * cos(omega) * cos(alpha)
    Z = R * sin(omega)
    '''
    [m,n] = np.shape(xyz_array)
    x = (xyz_array[:,0]).reshape(-1,1)
    y = (xyz_array[:,1]).reshape(-1,1)
    z = (xyz_array[:,2]).reshape(-1,1)

    # Ignoring all divide by zero errors to result in NaNs.
    np.seterr(divide='ignore', invalid='ignore') 
    alpha = (np.arctan(np.divide(x,y)))
    omega = (np.arctan( z/(y*np.cos(alpha)) ))
    R = np.divide(z, np.sin(omega))

    return { "range": R, "alpha": alpha, "omega": omega}

    
    