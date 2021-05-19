#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Abstandsmessung mit Ultraschallsensor SRF02
# Grove Cape V2 mit 5 V VCC verwenden!
# Auf dem RasPi funktioniert i2c.write_byte_data() nicht zuverlässig.
# Hier zweimal je ein Byte lesen:
#        wert_a = i2c.read_byte_data(0x70, 0x02)
#        wert_b = i2c.read_byte_data(0x70, 0x03)
#        # Messwert Big Endian > MSB und LSB tauschen
#        wert = ((wert_a << 8) & 0xFF00) + (wert_b)
# S. Mack, 27.5.21

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
        # 0x51 Messwert in cm, 0x52 Messwert in µs
        i2c.write_byte_data(0x70, 0, 0x52)
        time.sleep(0.2)
        # 2 Byte lesen ab Adresse 0x02
        wert = i2c.read_word_data(0x70, 0x02)
        # Messwert Big Endian > MSB und LSB tauschen
        wert = ((wert << 8) & 0xFF00) + (wert >> 8)
        print(wert)
        time.sleep(1)
except KeyboardInterrupt:
    print('')
    i2c.close()
    print('...i2c-bus closed.')
    print('Byebye...')

