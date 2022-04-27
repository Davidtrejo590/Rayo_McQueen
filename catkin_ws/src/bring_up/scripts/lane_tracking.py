#!/usr/bin/env python3

""" 
    NODE TO ENABLE CONTROL LAWS (CRUISE SPEED & STEERING ANGLE) FOR LANE TRACKING BEHAVIOR
"""

# LIBRARIES
import rospy
from std_msgs.msg import Float64, Bool, Float64MultiArray
from control_laws import Control


# GLOBAL VARIABLES
left_border   = [0.0, 0.0]
right_border  = [0.0, 0.0]
enable_LT   = None
goal_speed = 0.0

# LEFT LANE CALLBACK
def callback_left_border(msg):
    global left_border
    left_border = list(msg.data)          # TUPLE TO LIST

# RIGHT LANE CALLBACK
def callback_right_border(msg):
    global right_border
    right_border = list(msg.data)        # TUPLE TO LIST

# ENABLE LANE TRACKIG CALLBACK
def callback_enable_LT(msg):
    global enable_LT
    enable_LT = msg.data

# GOAL SPEED CALLBACK
def callback_goal_speed(msg):
    global goal_speed
    goal_speed = msg.data


# MAIN FUNCTION
def main():

    global left_border, right_border, enable_LT, goal_speed

    # CLASS FOR CONTROL LAWS
    control_LT = Control()

    # INIT NODE
    print('Lane Tracking Node...')
    rospy.init_node('lane_tracking')
    rate = rospy.Rate(10)

    # SUBSCRIBERS
    rospy.Subscriber('/left_border', Float64MultiArray, callback_left_border)
    rospy.Subscriber('/right_border', Float64MultiArray, callback_right_border)
    rospy.Subscriber('/enable_LT', Bool, callback_enable_LT)
    rospy.Subscriber('/goal_speed', Float64, callback_goal_speed)

    # PUBLISHERS
    pub_speed = rospy.Publisher('/speed', Float64, queue_size=10)
    pub_angle = rospy.Publisher('/steering', Float64, queue_size=10)


    while not rospy.is_shutdown():
        if enable_LT:                                               # LANE TRACKING STATE
            control_LT.control_law(left_border, right_border, goal_speed)           # COMPUTE CONTROL LAWS
            pub_speed.publish(control_LT.cruise_speed)              # PUBLISH CRUISE SPEED
            # pub_speed.publish(goal_speed)                           # PUBLISH CRUISE SPEED
            pub_angle.publish(control_LT.steering_angle)            # PUBLISH STEERING ANGLE
        
        rate.sleep()
    

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass


