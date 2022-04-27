#!/usr/bin/env python3

""" 
    NODE TO GET THE POINT CLOUD FROM LIDAR
    AND IMPLEMENT THE KMEANS ALGORITHM WITH THE PURPOSE
    TO IDENTIFY OBSTACLES (CARS)
"""

# LIBRARIES
import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseArray, Pose
import sensor_msgs.point_cloud2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import numpy as np
from random import uniform
import math
import copy

# OBEJCT DETECT CALLBACK
def callback_object_detect(msg):

    
    if msg:
        points = sensor_msgs.point_cloud2.read_points(msg, skip_nans=True)          # GET EACH POINT IN THE POINT CLOUD
        dataset = []
        x = []
        y = []
        z = []
        
        for point in points:
            if not point.__contains__(np.inf) and not point.__contains__(-np.inf):
                if( (point[0] >  -5.0 and point[0] < 5.0) and (point[1] > -1.5) and (point[2] < 0.0) ):
                    dataset.append(list(point))                                     # DATASET TO APPLY KMEANS - (X, Y,  Z)
                    x.append(point[0])
                    y.append(point[1])
                    z.append(point[2])
        

        fig = plt.figure()
        ax1 = fig.add_subplot(111, projection='3d')
        ax1.scatter(x, y, z, c='g', marker='o')
        plt.show()
        
        # print('SIZE DATASET', len(dataset))
        # print('DATA ', dataset)


# MAIN FUNCTION
def main():

    print('Object Detect Node...')
    rospy.init_node('object_detect')
    rate = rospy.Rate(10)

    # SUBSCRIBERS
    rospy.Subscriber('/point_cloud', PointCloud2, callback_object_detect)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except:
        rospy.ROSInitException
        pass
