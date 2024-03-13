#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Abstandsmessung mit Ultraschallsensor SRF02
# objektorientiert mit Klasse usSens
#
# S. Mack, 13.3.24

import smbus
import time
from usSens import UsSens

print('====================================')
print('SRF02 cm-Abstandssignal Miniprogramm')
print('====================================')

try:
    i2c = smbus.SMBus(2)
    print('i2c-bus opened...')
    my_us = UsSens(i2c, ic2_addr=0x70)
    while True:
        # Abstand in cm ausgeben und in mm umrechnen
        range = my_us.read_range()*10
        tof = my_us.read_us_tof()
        print("Range (mm): {}  ToF (µs): {}".format(range, tof))
        time.sleep(0.1)
except KeyboardInterrupt:
    print('')
    i2c.close()
    print('...i2c-bus closed.')
    print('Byebye...')
