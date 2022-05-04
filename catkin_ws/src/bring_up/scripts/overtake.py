#!/usr/bin/env python3

""" 
    ENABLE AND COMPUTE CONTROL LAWS (CRUISE SPEED & STEERING ANGLE) FOR PASS STATE 
"""

import rospy
from std_msgs.msg import Bool, Float64
import time

# STATES
SM_START        = 'SM_START'
SM_TURN_LEFT    = 'SM_TURN_LEFT'
SM_TURN_RIGHT   = 'SM_TURN_RIGHT'
SM_GO_STRAIGHT  = 'SM_GO_STRAIGHT'
SM_TURN_RIGHT_2 = 'SM_TURN_RIGHT_2'
SM_TURN_LEFT_2  = 'SM_TURN_LEFT_2'

SM_WAIT_NEW_OVERTAKE    = 'SM_WAIT_NEW_OVERTAKE'
SM_WAIT_TURN_LEFT       = 'SM_WAIT_TURN_LEFT'
SM_WAIT_TURN_RIGHT      = 'SM_WAIT_TURN_RIGHT'
SM_WAIT_GO_STRAIGHT     = 'SM_WAIT_STRAIGHT'
SM_WAIT_TURN_RIGHT_2    = 'SM_WAIT_TURN_RIGHT_2'
SM_WAIT_TURN_LEFT_2     = 'SM_WAIT_TURN_LEFT_2'
SM_FINISH_OVERTAKE      = 'SM_FINISH_OVERTAKE'

# GLOBAL VARIABLES
start_overtake = None
current_steering = None
TIME = 10


# CALLBACK ENABLE PASS
def callback_start_overtake(msg):
    global start_overtake
    start_overtake = msg.data

def callback_current_steering(msg):
    global current_steering
    current_steering = msg.data

# MAIN FUNCTION
def main():
    
    global start_overtake, current_steering
    
    print('Overtake Node...')
    rospy.init_node('overtake_node')
    rate = rospy.Rate(10)

    # SUBSCRIBERS
    rospy.Subscriber('/start_overtake', Bool, callback_start_overtake)
    rospy.Subscriber('/steering', Float64, callback_current_steering)

    # PUBLISHERS
    pub_steering = rospy.Publisher('/steering', Float64, queue_size=10)
    pub_overtake_finished = rospy.Publisher('/overtake_finished', Bool, queue_size=10)

    state = SM_START
    i = 0
    count = 0

    while not rospy.is_shutdown():

        if state == SM_START:                                   # STATE START
            print('STATE MACHINE TO OVERTAKE')
            state = SM_WAIT_NEW_OVERTAKE
        
        elif state == SM_WAIT_NEW_OVERTAKE:                     # STATE WAIT NEW OVERTAKE
            if start_overtake:
                i += 1
                print('ACCION DE REBASE:', i)
                # start_overtake = False
                pub_steering.publish(0.0)
                state = SM_TURN_LEFT
            else:
                state = SM_WAIT_NEW_OVERTAKE
        
        elif state == SM_TURN_LEFT:                             # STATE TURN LEFT
            print('GIRANDO A LA IZQUIERDA')
            pub_steering.publish(-0.35)
            count = 0
            state = SM_WAIT_TURN_LEFT

        elif state == SM_WAIT_TURN_LEFT:                        # STATE WAIT TURN LEFT
            count += 1
            if count > 6:
                state = SM_TURN_RIGHT
            else:
                state = SM_WAIT_TURN_LEFT

        elif state == SM_TURN_RIGHT:                            # STATE TURN RIGHT
            print('ALINEANDO')
            pub_steering.publish(0.35)
            count = 0
            state = SM_WAIT_TURN_RIGHT

        elif state == SM_WAIT_TURN_RIGHT:                       # STATE WAIT TURN RIGHT
            count += 1
            if count > 3:
                state = SM_TURN_RIGHT_2
            else:
                state = SM_WAIT_TURN_RIGHT

        elif state == SM_TURN_RIGHT_2:                          # STATE TURN RIGHT 2
            print('GIRANDO A LA DERECHA')
            pub_steering.publish(0.5)
            count = 0
            state = SM_WAIT_TURN_RIGHT_2
        
        elif state == SM_WAIT_TURN_RIGHT_2:                     # STATE WAIT TURN RIGHT 2
            count += 1
            if count > 6:
                state = SM_TURN_LEFT_2
            else:
                state = SM_WAIT_TURN_RIGHT_2
        
        elif state == SM_TURN_LEFT_2:                           # STATE TURN LEFT 2
            print('ALINEADO FINAL')
            pub_steering.publish(-0.5)
            count = 0
            state = SM_WAIT_TURN_LEFT_2

        elif state == SM_WAIT_TURN_LEFT_2:                      # STATE WAIT TURN LEFT 2
            count += 1
            if count > 3:
                state = SM_FINISH_OVERTAKE
            else:
                state = SM_WAIT_TURN_LEFT_2

        elif state == SM_FINISH_OVERTAKE:                       # STATE FINISH OVERTAKE
            pub_overtake_finished.publish(True)
            state = SM_WAIT_NEW_OVERTAKE

        
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInterruptException

