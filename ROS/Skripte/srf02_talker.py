#!/usr/bin/env python
#-*- coding: utf-8 -*-
## Einfacher talker, der den Ultraschallwandler SRF02 ausliest und
## dessen Messwerte als UInt16  unter dem topic 'range_val' veröffentlicht
## Objektorientierte Variante
## S. Mack, 13.3.24

import rospy
import time
import smbus
import sys
from usSens import UsSens

from std_msgs.msg import UInt16

def talker():
    pub = rospy.Publisher('range_val', UInt16, queue_size=10)
    rospy.init_node('SRF02Talker', anonymous=False)
    rate = rospy.Rate(10) # maximum rate 10 Hz due to sensor response time
    print('Python-Interpreter: {}\n'.format(sys.version))
    i2c = smbus.SMBus(2)
    print('i2c-bus opened...')
    my_us = UsSens(i2c, ic2_addr=0x70)
    while not rospy.is_shutdown():
        # Abstand in cm ausgeben und in mm umrechnen
        range_val = my_us.read_range()*10
        #rospy.loginfo('Talker: range value = ' + str(range_val))
        pub.publish(range_val)
        rate.sleep()
    i2c.close()
    print('')
    print('...i2c-bus closed.')

if __name__ == '__main__':
    try:
        print('SRF02-Talker startet...')
        talker()
    except rospy.ROSInterruptException:
        print('ROSInterruptException')
        pass
    finally:
        if 'i2c' in locals():
            i2c.close()
        print('Byebye...')
