#!/usr/bin/env python
# -*- coding: utf-8 -*-
## listener abonniert vom Simulink-Knoten adcconv_subpub das Topic
## /adc_val_mv umd published über das Topic /volt_thresh den Wert Null oder
## Eins je nachdem, ob der Spannungswert größer oder kleiner 1000 mV ist.
##
## S. Mack, 11.6.20

import rospy

from std_msgs.msg import Int16, Bool


def callback(data):
    global pub
    #rospy.loginfo('Listener: Empfangen = {}'.format(data.data))
    if data.data < 1000:
        pub.publish(False)
        print("Voltage < 1 V")
    else:
        pub.publish(True)
        print("Voltage > 1 V")

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('simulink_listener_talker', anonymous=False)
    rospy.Subscriber('adc_val_mv', Int16, callback)
    print('Listener started...')
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try: 
        pub = rospy.Publisher('volt_thresh', Bool, queue_size=10)
        print('Simulink node starting...')
        listener()
    except rospy.ROSInterruptException:
        print('ROSInterruptException')
        pass
    finally:
        print('Byebye...')

