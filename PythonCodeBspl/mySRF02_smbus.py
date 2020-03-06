#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Abstandsmessung mit Ultraschallsensor SRF02
# S. Mack, 25.11.19

import sys
import smbus
import time

print('Python-Interpreter: {}\n'.format(sys.version))
i2c = smbus.SMBus(2)
print('i2c-bus opened...')

print('====================================')
print('SRF02 cm-Abstandssignal Miniprogramm')
print('====================================')

try:
    while True:
        i2c.write_byte_data(0x70, 0, 0x51)
        time.sleep(0.2)
        print(i2c.read_word_data(0x70, 2) / 255)
        time.sleep(1)
except KeyboardInterrupt:
    print('')
    i2c.close()
    print('...i2c-bus closed.')
    print('Byebye...')

