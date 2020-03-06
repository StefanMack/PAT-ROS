#!/usr/bin/env python
# -*- coding: utf-8 -*-
## listener abonniert von SRF04_talker_int.py den gemessenen Abstandswert range_val
## als uint16.
## Je nach Abstandswert wird der GPIO-Pin auf High oder Low geschaltet
## S. Mack, 2.12.19

import rospy
import Adafruit_BBIO.GPIO as GPIO
from std_msgs.msg import UInt16

pin = 'P8_11'

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + 'Abstand = {}'.format(data.data))
    #rospy.loginfo('Listener: Empfangen = {}'.format(data.data))
    if data.data < 30:
        GPIO.output(pin, GPIO.HIGH)
    else:
        GPIO.output(pin, GPIO.LOW)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('gpio_listener_int', anonymous=False)

    rospy.Subscriber('range_val', UInt16, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        GPIO.setup(pin, GPIO.OUT)
        print('Pin enabled...')
        listener()
        GPIO.output(pin, GPIO.LOW)
        print('')
        #print('...Pin disabled.')
    	#GPIO.cleanup() # Bewirkt unexport
    	#print("Byebye...")
    finally:
        GPIO.cleanup() # Bewirkt unexport
        print('...Pin disabled.')
        print('Byebye...')

