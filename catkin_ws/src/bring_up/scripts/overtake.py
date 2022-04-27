#!/usr/bin/env python3

""" 
    ENABLE AND COMPUTE CONTROL LAWS (CRUISE SPEED & STEERING ANGLE) FOR PASS STATE 
"""

import rospy
from std_msgs.msg import Bool, Float64
import time

# GLOBAL VARIABLES
pass_finished = Bool()
enable_PS = None
current_steering = 0.0

# CALLBACK ENABLE PASS
def callback_enable_PS(msg):
    global enable_PS
    enable_PS = msg.data

def callback_current_steering(msg):
    global current_steering
    current_steering = msg.data

# MAIN FUNCTION
def main():
    
    global enable_PS, pass_finished, current_steering
    
    print('Pass Node...')
    rospy.init_node('pass_node')
    rate = rospy.Rate(10)

    # SUBSCRIBERS
    rospy.Subscriber('/enable_PS', Bool, callback_enable_PS)
    rospy.Subscriber('/steering', Float64, callback_current_steering)

    # PUBLISHERS
    pub_angle = rospy.Publisher('/steering', Float64, queue_size=10)
    pub_pass_finished = rospy.Publisher('/pass_finished', Bool, queue_size=10)

    while not rospy.is_shutdown():
        # PASSING ACTION
        steering_angle = 0.0
        if enable_PS:

            steering_angle = current_steering -( 0.0174533 * 8 )            # TURN LEFT
            print('GIRANDO A LA IZQUIERDA')
            pub_angle.publish(steering_angle)
            time.sleep(2)

            steering_angle = current_steering + ( 0.0174533 * 8 )             # TURN RIGHT
            print('GIRANDO A LA DERECHA')
            pub_angle.publish(steering_angle)
            time.sleep(2)

            steering_angle = 0.0                           # STAY STRAIGHT
            print('ALINEANDO')
            pub_angle.publish(steering_angle)
            time.sleep(2)

            steering_angle = current_steering + ( 0.0174533 * 16 )            # TURN RIGHT
            print('GIRANDO A LA DERECHA')
            pub_angle.publish(steering_angle)
            time.sleep(2)

            steering_angle = current_steering - ( 0.0174533 * 16 )           # TURN LEFT
            print('GIRANDO A LA IZQUIERDA')
            pub_angle.publish(steering_angle)
            time.sleep(2)

            pass_finished.data = True                      # PASS FINISHED

            pub_pass_finished.publish(pass_finished)       # PUBLISH PASS FINISHED
        else:
            steering_angle = 0.0
            
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInterruptException
