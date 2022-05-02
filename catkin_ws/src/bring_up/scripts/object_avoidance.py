#!/usr/bin/env python3

"""
    NODE TO CREATE A STATES MACHINE AND CHOOSE THE CORRECT BEHAVIOR
    (PASS, CRUISE, KEEP DISTANCE), ENABLE THE CORRESPOND STATE
"""

# LIBRARIES
import rospy
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Bool, Float64

# STATES
SM_CRUISE   = 'CRUISE'
SM_PASS     = 'PASS'

# ENABLES
enable_LT = Bool()              # ENABLE LANE TRACKING
enable_PS = Bool()              # ENABLE PASSING

# GLOBAL VARIABLES
pass_finished = False
car_detected = False

# CAR POSE CALLBACK
def callback_car_pose(msg):

    global car_detected

    if msg.poses[0].position.z != 0.0 and msg.poses[0].position.z > -15.0 :
        car_detected = True
    else:
        car_detected = False
    

# PASS FINISHED CALLBACK
def callback_pass_finished(msg):
    global pass_finished
    pass_finished = msg.data

# MAIN FUNCTION
def main():
    global enable_LT, enable_PS, pass_finished, car_detected

    # INIT NODE
    print('Object Avoidance Node...')
    rospy.init_node('object_avoidance')
    rate = rospy.Rate(10)

    # SUBSCRIBERS
    rospy.Subscriber('/object_pose', PoseArray, callback_car_pose)
    rospy.Subscriber('/pass_finished', Bool, callback_pass_finished)

    # PUBLISHERS
    pub_enable_LT = rospy.Publisher('/enable_LT', Bool, queue_size=10)
    pub_enable_PS = rospy.Publisher('/enable_PS', Bool, queue_size=10)

    # STATE MACHINE
    state = SM_CRUISE

    while not rospy.is_shutdown():
        
        if state == SM_CRUISE:                      # CRUISE STATE
            # print('CRUISE')
            enable_LT.data = True
            enable_PS.data = False

            if car_detected:
                state = SM_PASS
        
        elif state == SM_PASS:                      # PASS STATE
            # print('PASS')
            enable_PS.data = True
            enable_LT.data = False
            if pass_finished:
                pass_finished = False               # FROM PASS NODE
                car_detected = False
                state = SM_CRUISE
        

        # PUBLISH ENABLES
        pub_enable_LT.publish(enable_LT)        # ENABLE LANE TRACKING
        pub_enable_PS.publish(enable_PS)        # ENABLE PASS

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInterruptException
