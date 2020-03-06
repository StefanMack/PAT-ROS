#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Auslesen des AD-Wandlers
# !!!! ACHRTUNG !!!!
# UNBEDINGT "ANALOG INPUT" DES GROVE CAPES VERWENDEN 
# Die P9-ADC Pins der Buchsenleisten vertragen maximal 1,8 V
# S. Mack, 27.11.19

import sys
import Adafruit_BBIO.ADC as ADC
import time

print("Python-Interpreter: {}\n".format(sys.version))
ADC.setup()

try:
    while True:
        #read returns values 0-1.0
        valueNorm = ADC.read("P9_39")
        #read_raw returns non-normalized value in LSB
        valueRaw = ADC.read_raw("P9_39")
        time.sleep(1)
        print("Normierter Wert: {}".format(valueNorm))
        print("Rohwert: {}".format(valueRaw))
except KeyboardInterrupt:
    print("")
    print("Byebye...")
