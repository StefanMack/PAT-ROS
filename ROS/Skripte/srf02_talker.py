#!/usr/bin/env python
#-*- coding: utf-8 -*-
## Einfacher talker, der den Ultraschallwandler SRF02 ausliest und
## dessen Messwerte als UInt16  unter dem topic 'range_val' veröffentlicht
## S. Mack, 3.6.20

import rospy
import time
import smbus
import sys

from std_msgs.msg import UInt16

def talker():
    pub = rospy.Publisher('range_val', UInt16, queue_size=10)
    rospy.init_node('SRF02Talker', anonymous=False)
    rate = rospy.Rate(10) # maximum rate 10 Hz due to sensor response time
    print('Python-Interpreter: {}\n'.format(sys.version))
    i2c = smbus.SMBus(2)
    i2c = smbus.SMBus(2)
    print('i2c-bus opened...')
    while not rospy.is_shutdown():
        i2c.write_byte_data(0x70, 0, 0x51)
        time.sleep(0.05)
        # 2 Byte lesen ab Adresse 0x02
        range_val = i2c.read_word_data(0x70, 0x02)
        # Messwert Big Endian > MSB und LSB tauschen
        range_val = ((range_val << 8) & 0xFF00) + (range_val >> 8)
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
