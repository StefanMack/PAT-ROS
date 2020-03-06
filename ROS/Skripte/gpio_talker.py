#!/usr/bin/env python
#-*- coding: utf-8 -*-
## Einfacher talker, der den P9_11 (GPIO 30) ausliest und
## dessen Wert als UInt8 unter dem topic 'gpio_val' veröffentlicht
## S. Mack, 14.12.19

import rospy
import Adafruit_BBIO.GPIO as GPIO

from std_msgs.msg import UInt8

def talker():
    pub = rospy.Publisher('gpio_val', UInt8, queue_size=10)
    rospy.init_node('GPIOTalker', anonymous=False)
    rate = rospy.Rate(10)
    GPIO.setup("P9_11", GPIO.IN)
    print('GPIO setup done...')
    while not rospy.is_shutdown():
        gpio_val = GPIO.input("P9_11")
        #rospy.loginfo('Talker: GPIO value = ' + str(gpio_val))
        pub.publish(gpio_val)
        rate.sleep()

if __name__ == '__main__':
    try:
        print('GPIO-Talker starting...')
        talker()
    except rospy.ROSInterruptException:
        print('ROSInterruptException')
        pass
    finally:
        GPIO.cleanup() # Bewirkt unexport
        print('...Pin disabled.')
        print('Byebye...')
