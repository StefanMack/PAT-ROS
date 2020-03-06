#!/usr/bin/env python
# -*- coding: utf-8 -*-
## listener abonniert die Daten des SNES Gamepad-Nodes joy-node aus dem 
## Package joy.
## Die Taste "select" (buttons[8]) schaltet den GPIO-Pin auf High solange 
## sie gedrückt ist.
##
## S. Mack, 3.12.19

import rospy
import Adafruit_BBIO.GPIO as GPIO
from sensor_msgs.msg import Joy

pin = 'P8_11'

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + 'Abstand = {}'.format(data.data))
    rospy.loginfo('Listener: Selecttaste = {}'.format(data.buttons[8]))
    if data.buttons[8] == 1:
        GPIO.output(pin, GPIO.HIGH)
    else:
        GPIO.output(pin, GPIO.LOW)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('snes_listener', anonymous=True)

    rospy.Subscriber('joy', Joy, callback)

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

