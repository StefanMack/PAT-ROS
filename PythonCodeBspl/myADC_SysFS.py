#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Auslesen des AD-Wandlers
# S. Mack, 27.11.19

import time
import sys

print("Python-Interpreter: {}\n".format(sys.version))

adc0 = open("/sys/bus/iio/devices/iio:device0/in_voltage0_raw", "r")

try:
    while True:
        voltage = int(adc0.read())
        print("ADC-Wert Spannung Rohwert: {}".format(voltage))
        adc0.seek(0)
        time.sleep(1)
except KeyboardInterrupt:
    print("")
    print("Byebye...")
    adc0.close()
