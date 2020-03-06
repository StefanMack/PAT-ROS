# !/usr/bin/python3
# -*- coding: utf-8 -*-
# Toggeln des GPIO P9_25 (GPIO 117) ueber die Adafruit_BBIO-Bibliothek.

# S. Mack, 5.3.20

import Adafruit_BBIO.GPIO as GPIO
import time
import sys

print("Python-Interpreter: {}\n".format(sys.version))

pin = "P9_25"

GPIO.setup(pin, GPIO.OUT)
print("Pin enabled...")

try:
    while True:
        GPIO.output(pin, GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(pin, GPIO.LOW)
        time.sleep(0.5)
except KeyboardInterrupt:
    print("")
    GPIO.output(pin, GPIO.LOW)
    print("...Pin disabled.")
    print("Byebye...")
