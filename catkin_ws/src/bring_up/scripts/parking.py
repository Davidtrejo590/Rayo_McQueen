#!/usr/bin/env python3

"""
    
"""

# LIBRARIES
from tkinter.messagebox import NO
import rospy
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Bool, Float64

# STATES
SM_CRUISE   = 'CRUISE'
SM_PASS     = 'PASS'

# GLOBAL VARIABLES
car_detected = False
positions = []
position = [0.0, 0.0]

# CAR POSE CALLBACK
def callback_car_pose(msg):

    global car_detected, positions, position

    if msg.poses[0].position.z != 0.0:
        print('CAR DETECTED', [msg.poses[0].position.x, msg.poses[0].position.z])
        position = [msg.poses[0].position.x, msg.poses[0].position.z]
    else:
        print('NO CAR DETECTED', [0.0, 0.0])
        if position[0] != 0.0 and position[1] != 0.0:
            positions.append(position)
        position = [0.0, 0.0]

    print('POSITIONS', len(positions))
    

# MAIN FUNCTION
def main():
    global enable_LT, enable_PS, pass_finished, car_detected

    # INIT NODE
    print('Parking Node...')
    rospy.init_node('parking_node')
    rate = rospy.Rate(10)

    # SUBSCRIBERS
    rospy.Subscriber('/object_pose', PoseArray, callback_car_pose)

    # PUBLISHERS

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInterruptException
