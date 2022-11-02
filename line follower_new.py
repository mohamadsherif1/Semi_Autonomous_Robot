#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
    
if _name_ == '_main_':
    rospy.init_node('LINE_FOLLOWER_node')
    pub = rospy.Publisher('line_follower', Float32, queue_size = 10)
    flag = 0
    ir_1 =5
    ir_2=6
    ir_3=13
    ir_4=19
    ir_5=26
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(ir_1, GPIO.IN)
    GPIO.setup(ir_2, GPIO.IN)
    GPIO.setup(ir_3, GPIO.IN)
    GPIO.setup(ir_4, GPIO.IN)
    GPIO.setup(ir_5, GPIO.IN)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if (GPIO.input(ir_1) == 0) :
            flag = 1
            #pub.publish(Float32(flag))
            
        elif (GPIO.input(ir_2) == 0 ):
            flag = 2
            #pub.publish(Float32(flag))
            
        elif (GPIO.input(ir_3) ==  0):
            flag = 3
            #pub.publish(Float32(flag))
        elif (GPIO.input(ir_4) == 0 ):
            flag = 4
            #pub.publish(Float32(flag))
        elif (GPIO.input(ir_5) == 0 ):
            flag = 5
        
           # pub.publish(Float32(flag))
        if (GPIO.input(ir_1) == 0 ) and (GPIO.input(ir_2) == 0 ):
            flag = 1
        if (GPIO.input(ir_2) == 0 ) and (GPIO.input(ir_3) == 0 ):
            flag = 2
        if (GPIO.input(ir_3) == 0 ) and (GPIO.input(ir_4) == 0 ):
            flag = 4
        if (GPIO.input(ir_4) == 0 ) and (GPIO.input(ir_5) == 0 ):
            flag = 5
     
        
        if((GPIO.input(ir_5) == 1)and (GPIO.input(ir_4) == 1) and (GPIO.input(ir_3) == 1) and (GPIO.input(ir_2) == 1 )and (GPIO.input(ir_1) == 1)):
            flag=0
        pub.publish(Float32(flag))
        rate.sleep()
    GPIO.cleanup()