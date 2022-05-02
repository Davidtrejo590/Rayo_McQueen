#!/usr/bin/env python3

""" 
    ENABLE AND COMPUTE CONTROL LAWS (CRUISE SPEED & STEERING ANGLE) FOR PASS STATE 
"""

import rospy
from std_msgs.msg import Bool, Float64
import time

# STATES
SM_INIT = "SM_INIT"
SM_TURNING_RIGHT = "SM_TURNING_RIGHT"
SM_TURNING_LEFT = "SM_TURNING_LEFT"
SM_STRAIGHT = "SM_STRAIGHT"
SM_STRAIGHT_2 = "SM_STRAIGHT_2"
SM_TURNING_LEFT_2 = "SM_TURNING_LEFT_2"
SM_TURNING_RIGHT_2 = "SM_TURNING_RIGHT_2"
SM_FINISH = "SM_FINISH"
SM_NONE = "SM_NONE"

# GLOBAL VARIABLES
pass_finished = Bool()
enable_PS = False
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

    state = SM_INIT
    i = 0

    while not rospy.is_shutdown():

        if enable_PS:
            i += 1
            steering_angle = 0.0
            pub_angle.publish(steering_angle)

            if not i & 1 == 0:

                print('INICIA ACCIÓN DE REBASE: ', i)

                steering_angle = 0.0
                pub_angle.publish(steering_angle)

                steering_angle = current_steering -( 0.0174533 * 17 )           # TURN LEFT
                print('GIRANDO A LA IZQUIERDA', steering_angle)
                pub_angle.publish(steering_angle)
                rospy.sleep(1)

                # steering_angle = current_steering + ( 0.0174533 * 17 )          # TURN RIGHT
                steering_angle = 0.0
                print('GIRANDO A LA DERECHA', steering_angle)
                pub_angle.publish(steering_angle)
                rospy.sleep(0.5)
            
                steering_angle = current_steering + ( 0.0174533 * 32 )         # TURN RIGHT
                print('GIRANDO A LA DERECHA', steering_angle)
                pub_angle.publish(steering_angle)
                rospy.sleep(1)

                # steering_angle = current_steering - ( 0.0174533 * 34 )         # TURN LEFT
                steering_angle = 0.0
                print('GIRANDO A LA IZQUIERDA', steering_angle)
                pub_angle.publish(steering_angle)
                rospy.sleep(0.5)


                pass_finished.data = True                      # PASS FINISHED
                pub_pass_finished.publish(pass_finished)       # PUBLISH PASS FINISHED

            


        # if state == SM_INIT:
        #     if enable_PS:
        #         state = SM_TURNING_LEFT
        #     else:
        #         state = SM_INIT

        # elif state == SM_TURNING_LEFT:
        #     print('GIRANDO A LA IZQUIERDA')
        #     pub_angle.publish(-0.0174533 * 17)
        #     time.sleep(1)
        #     state = SM_TURNING_RIGHT
                    

        # elif state == SM_TURNING_RIGHT:
        #     print('GIRANDO A LA DERECHA')
        #     pub_angle.publish(0.0174533 * 17 )
        #     time.sleep(1)
        #     state = SM_STRAIGHT
                
        
        # elif state == SM_STRAIGHT:
        #     print('ALINEANDO')
        #     pub_angle.publish(0.0)
        #     time.sleep(1)
        #     state = SM_TURNING_RIGHT_2
                
        
        # elif state == SM_TURNING_RIGHT_2:
        #     print('GIRANDO A LA DERECHA 2')
        #     pub_angle.publish(0.0174533 * 34)
        #     time.sleep(1)
        #     state = SM_TURNING_LEFT_2

        # elif state == SM_TURNING_LEFT_2:
        #     print('GIRANDO A LA IZQUIERDA 2')
        #     pub_angle.publish(-0.0174533 * 34)
        #     time.sleep(1)
        #     state = SM_STRAIGHT_2

        # elif state == SM_STRAIGHT_2:
        #     print('ALINEANDO FINAL')
        #     pub_angle.publish(  0.0)
        #     time.sleep(1)
        #     pass_finished.data = True
        #     pub_pass_finished.publish(pass_finished)
        #     state = SM_INIT
        
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInterruptException






            