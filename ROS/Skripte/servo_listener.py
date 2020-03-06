#!/usr/bin/env python
# -*- coding: utf-8 -*-
## listener abonniert von SRF04_talker_int.py den gemessenen Abstandswert range_val
## als UInt16.
## Im Bereich 0 bis 100 cm wird die Servoposition proportional zum Abstandswert eingestellt.
## S. Mack, 9.12.19

import rospy
import Adafruit_BBIO.PWM as PWM
from std_msgs.msg import UInt16

PWM_PIN = "P9_14"
#Parameters for Micro Servo BM-306
FREQ = 50
MINDUTYC = 0.03
MAXDUTYC = 0.12
POL = 0

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + 'Abstand = {}'.format(data.data))
    rospy.loginfo('Listener: Empfangen = {}'.format(data.data))
    if data.data < 100:
        PWM.set_duty_cycle(PWM_PIN, MINDUTYC*100+(MAXDUTYC-MINDUTYC)*data.data)
    else:
        PWM.set_duty_cycle(PWM_PIN, MINDUTYC*100)

def listener():

    # anonymous=False flag means that rospy will choose exaclty the
    # given name for the  'listener' node. If two nodes with the same
    # name are launched, the previous one is kicked off.
    rospy.init_node('servo_listener_int', anonymous=False)

    rospy.Subscriber('range_val', UInt16, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        PWM.start(PWM_PIN, (MINDUTYC*100), FREQ, POL)
        print('PWM outpunt enabled...')
        listener()
        PWM.set_duty_cycle(PWM_PIN, MINDUTYC*100)
        print('')
    finally:
        PWM.cleanup()
        print('...Pin disabled.')
        print('Byebye...')

