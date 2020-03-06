#!/usr/bin/env python
#-*- coding: utf-8 -*-
## Einfacher talker, der den AD-Wander an Pin 9_39  ausliest und
## dessen Messwerte als UInt16 unter dem topic 'adc_val' veröffentlicht
## S. Mack, 19.2.20

import rospy
import Adafruit_BBIO.ADC as ADC

from std_msgs.msg import UInt16

def talker():
    pub = rospy.Publisher('adc_val', UInt16, queue_size=10)
    rospy.init_node('ADCIntTalker', anonymous=False)
    rate = rospy.Rate(10)
    ADC.setup()
    print('ADC setup...')
    while not rospy.is_shutdown():
        adc_val = int(ADC.read_raw("P9_39"))
        #rospy.loginfo('Talker: ADC value = ' + str(adc_val))
        pub.publish(adc_val)
        rate.sleep()

if __name__ == '__main__':
    try:
        print('ADC-Talker starting...')
        talker()
    except rospy.ROSInterruptException:
        print('ROSInterruptException')
        pass
    finally:
        # No cleanup required
        print('')  
        print('Byebye...')
