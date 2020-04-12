#!/usr/bin/env python
# -*- coding: utf-8 -*-
## listener abonniert von SRF04_talker_int.py den gemessenen Abstandswert range_val
## als UInt16.
## Im Bereich 0 bis 100 cm wird die Servoposition proportional zum Abstandswert eingestellt.
## S. Mack, 9.12.19

import rospy
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO
from std_msgs.msg import UInt16

# Beide Motoren werden mit unterschiedlichen PWM angesteuert
pin_motL = "P9_14" # GPIO 50 PWMA linker Motor
pin_motR = "P9_16" # GPIO 51 PWMB rechter Motor
# Eingänge In2 und In2 invertiert zueinander (geben Drehrichtung vor)
pin_in1 = "P9_25" # GPIO 117
pin_in2 = "P9_27" # GPIO 115
# PWM Parameter
FREQ = 250000
DUTYC = 0.6


def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + 'Abstand = {}'.format(data.data))
    #rospy.loginfo('Listener: Empfangen = {}'.format(data.data))
    if data.data < 30:
        PWM.set_duty_cycle(pin_motL, DUTYC*100*0.8)
        PWM.set_duty_cycle(pin_motR, DUTYC*100*0.8)
    else:
        PWM.set_duty_cycle(pin_motL, DUTYC*100)
        PWM.set_duty_cycle(pin_motR, DUTYC*100)

def listener():
    # anonymous=False flag means that rospy will choose exaclty the
    # given name for the  'listener' node. If two nodes with the same
    # name are launched, the previous one is kicked off.
    rospy.init_node('motor_listener_int', anonymous=False)

    rospy.Subscriber('range_val', UInt16, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        PWM.start(pin_motL, 0, FREQ, 0)
        PWM.start(pin_motR, 0, FREQ, 0)
        GPIO.setup(pin_in1, GPIO.OUT)
        GPIO.setup(pin_in2, GPIO.OUT)
        print('PWM outpunt enabled...')
        GPIO.output(pin_in1, GPIO.HIGH)
        GPIO.output(pin_in2, GPIO.LOW)
        listener()
        print('')
    finally:
        PWM.set_duty_cycle(pin_motL, 0)
        PWM.set_duty_cycle(pin_motR, 0)
        PWM.cleanup()
        print('...Pin disabled.')
        print('Byebye...')

