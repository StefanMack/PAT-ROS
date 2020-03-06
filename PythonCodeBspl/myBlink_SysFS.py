# !/usr/bin/python3
# -*- coding: utf-8 -*-
# Toggeln des GPIO P9_25 (GPIO 117) ueber das Sys FS ohne Pythonbibliothek dafuer.
# Um ein analoges  Verhalten wie die Bash-Befehle " echo out > direction" zu errreichen
# müssen die jeweiligen (virtuellen) Dateien wie "direction" im Append-Modus 
# geöffnet werden. Daher der Flag "a" bei open(). "w" oder "r+" funktioniert aber auch.
# Abbruch mit strg + c
# S. Mack, 5.3.20

import time
import sys

print("Python-Interpreter: {}\n".format(sys.version))

setup = open('/sys/class/gpio/gpio117/direction', 'a')
setup.write('out')
setup.flush()
setup.close()
time.sleep(1)
print('GPIO 117 als Ausgang gesetzt')

gpio = open('/sys/class/gpio/gpio117/value', 'a')

try:
    while True:
        gpio.write('1')
        print('GPIO auf High gesetzt')
        gpio.flush()
        time.sleep(1)
        gpio.write('0')
        print('GPIO auf Low gesetzt')
        gpio.flush()
        time.sleep(1)
except KeyboardInterrupt:
    print("")
    print("Byebye...")
    gpio.close()
