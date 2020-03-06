#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Programm zum Testen eines Modellbauservos an Pin P9_14
# Die Randwerte der Dutycycles fuer die beiden Extrempositionen
# haben bei anderen Servos andere Werte.
# S. Mack, 5.3.20


import Adafruit_BBIO.PWM as PWM
import sys
import time

print("Python-Interpreter: {}\n".format(sys.version))

pin = "P9_14"

#Parameters for Micro Servo
FREQ = 50
MINDUTYC = 0.03
MAXDUTYC = 0.12
POL = 0

dutyC = MINDUTYC

PWM.start(pin, ((MINDUTYC+MAXDUTYC)/2*100), FREQ, 0)

try:
    while True:
        PWM.set_duty_cycle(pin, dutyC*100)
        time.sleep(1)
        dutyC = dutyC + 0.01
        if (dutyC > MAXDUTYC):
            dutyC = MINDUTYC
#Falls Strg+c gedrueckt wird
except KeyboardInterrupt:
    PWM.cleanup()
    print("")
    print("Byebye...")    

