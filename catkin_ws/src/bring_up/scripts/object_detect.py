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

x = 0.0
z = 0.0
i = 0.0

# OBEJCT DETECT CALLBACK
def callback_object_detect(msg):

    points = sensor_msgs.point_cloud2.read_points(msg, skip_nans=True)          # GET EACH POINT IN THE POINT CLOUD

    global x, z, i
    
    x = 0.0
    z = 0.0
    i = 0.0
        
        
    for point in points:
        if not point.__contains__(np.inf) and not point.__contains__(-np.inf):
            if( ( (point[0] > 0.75 and point[0] < 2.0) or (point[0] < -0.75 and point[0] > -2.0) ) and  (point[1] > -1.5) and (point[2] < 0.0) ):
                # dataset.append(dataset, np.array([point[0], point[2]]))                
                x = x + point[0]
                z = z + point[2]
                i = i + 1

    x = x/i
    z = z/i
    print('MEAN DATA', [x, z])
        

# MAIN FUNCTION
def main():

    global x, z, i

    print('Object Detect Node...')
    rospy.init_node('object_detect_py')
    rate = rospy.Rate(10)

    # SUBSCRIBERS
    rospy.Subscriber('/point_cloud', PointCloud2, callback_object_detect)

    # PUBLISHERS
    pub_pose = rospy.Publisher('/mean_pose', Pose, queue_size=10)

    while not rospy.is_shutdown():
        pose = Pose()
        pose.position.x = x
        pose.position.y = 0.0
        pose.position.z = z

        pub_pose.publish(pose)

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except:
        rospy.ROSInitException
        pass
