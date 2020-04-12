#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Programm zum Testen H-Brücken-Treiber mit 2 DC Motoren.
# Pin P9_14 wird an den PWM-Eingang des Treibers.  
# In1 und In2 sind komplimentär und bestimmen Drehrichtung.
# Damit der Roboter hin und zurück fährt,  beide Motoren parallel
# an den drei Pins anschließen.
# 
# S. Mack, 11.4.20

import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO
import sys
import time

print("Python-Interpreter: {}\n".format(sys.version))

# Beide Motoren werden mit unterschiedlichen PWM angesteuert
pin_motL = "P9_14" # GPIO 50 PWMA linker Motor
pin_motR = "P9_16" # GPIO 51 PWMB rechter Motor
# Eingänge In2 und In2 invertiert zueinander (geben Drehrichtung vor)
pin_in1 = "P9_25" # GPIO 117
pin_in2 = "P9_27" # GPIO 115
# PWM Parameter
FREQ = 250000
DUTYC = 0.6

PWM.start(pin_motL, 0, FREQ, 0)
PWM.start(pin_motR, 0, FREQ, 0)
GPIO.setup(pin_in1, GPIO.OUT)
GPIO.setup(pin_in2, GPIO.OUT)

try:
    while True:
        print('Vorwärts immer ...',end=' ', flush=True)
        GPIO.output(pin_in1, GPIO.HIGH)
        GPIO.output(pin_in2, GPIO.LOW)
        PWM.set_duty_cycle(pin_motL, DUTYC*100)
        PWM.set_duty_cycle(pin_motR, DUTYC*100)
        time.sleep(2)
        PWM.set_duty_cycle(pin_motL, 0)
        PWM.set_duty_cycle(pin_motR, 0)
        time.sleep(0.5)
        print('rückwärts nimmer')
        GPIO.output(pin_in1, GPIO.LOW)
        GPIO.output(pin_in2, GPIO.HIGH)
        PWM.set_duty_cycle(pin_motL, DUTYC*100)
        PWM.set_duty_cycle(pin_motR, DUTYC*100)
        time.sleep(2)
        PWM.set_duty_cycle(pin_motL, 0)
        PWM.set_duty_cycle(pin_motR, 0)
        time.sleep(0.5)
#Falls Strg+c gedrueckt wird
except KeyboardInterrupt:
    PWM.cleanup()
    print("")
    print("Byebye...Honecker ;-)")
