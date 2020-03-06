#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Toggeln des Pins P9_25 (GPIO 117) mit bytweisen Schreiben.
# Anders als bei der Pythonfunktion "open(path, 'a')", bei der "a" für
# "append" steht, funktioniert bei os.open() der entsprechende Parameter
# "os.O_APPEND" nicht!
# Mit "os.O_RDWR" oder "os.O_WRONLY" funktioniert es aber.
# Abbruch mit strg + c
# S. Mack, 5.3.20

import time
import os
import sys

print("Python-Interpreter: {}\n".format(sys.version))

setup = os.open('/sys/class/gpio/gpio117/direction', os.O_WRONLY)
os.write(setup, b'out')
os.close(setup)
time.sleep(1)
print('GPIO 117 als Ausgang gesetzt')

gpio = os.open('/sys/class/gpio/gpio117/value', os.O_WRONLY)

try:
    while True:
        os.write(gpio, b'1')
        print('GPIO auf High gesetzt')
        time.sleep(1)
        os.write(gpio, b'0')
        print('GPIO auf Low gesetzt')
        time.sleep(1)
except KeyboardInterrupt:
    print("")
    print("Byebye...")
    os.close(gpio)
 
