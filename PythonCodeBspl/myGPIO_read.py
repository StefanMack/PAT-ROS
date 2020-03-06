#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Pollen des GPIO P9_27 (GPIO 115) ueber das Sys FS
# ohne Pythonbibliothek dafuer.
# Abbruch mit strg + c
# S. Mack, 5.3.20

import time
import sys

print("Python-Interpreter: {}\n".format(sys.version))

setup = open('/sys/class/gpio/gpio115/direction', 'w')
setup.write('in')
setup.close()
time.sleep(1)
print('GPIO 115 als Eingang gesetzt')

gpio = open('/sys/class/gpio/gpio115/value', 'r')

try:
    while True:
        value=gpio.read().strip() # strip() entfernt Zeilenumbruch in value
        print('GPIO-Zustand: {}'.format(value))
        gpio.seek(0)
        time.sleep(1)
except KeyboardInterrupt:
    print("")
    print("Byebye...")
    gpio.close()
