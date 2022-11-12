#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Beispiel-Skript für den STM TOF-Sensor VL53L1X
# Grundgeruest stammt aus VL53L1X library for Arduino von Pololu
# github.com/pololu/vl53l1x-arduino fuer Arduino.
# Version mit ROI-Zentrum in Richtung lange Seite des Sensors mit nur 4 Winkeln scannen.
# Da der ROI minimal 4 Pixel breit sein muss, kann dieser um bis zu 13 (überlappenden)
# Positionen verschoben werden. Ohne Überlappung nur um 4 Position = 4 Scanwinkel.
# Vom Sensor wird nur die Reichweite abgefragt, damit höhere Messrate.
# Modul VL53L1XRegAddr.py mit den Registeradressen sowie Modul tofSens.py
# müssen im selben Verzeichnis sein.
# Objektorientierte Variante des Skripts
# S. Mack, 12.11.22


import smbus
import sys
from time import sleep
import VL53L1XRegAddr as REG
from tofSens import TofSens, countRateFixedToFloat, writeReg, writeReg16Bit, writeReg32Bit, readReg, readReg16Bit

DEBUG_PRINT = False



# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Variablen ------------------------------------------------------------------
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

calibrated = False;


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# main()----------------------------------------------------------------------
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

try:
    print("Python-Interpreter: {}\n".format(sys.version))
    i2c = smbus.SMBus(2) # I2C-Kommunikation aktivieren
    my_tof = TofSens(i2c, ic2_addr=0x29, timeout_ms=500)
    if not my_tof.initSensor(True): 
        print('Failed to detect and initialize sensor!')
    else:
        print('Sensor Initialisierung i.O.') 
    
    # The minimum timing budget is 20 ms for short distance mode and 33 ms for
    # medium and long distance modes. See the VL53L1X datasheet for more
    # information on range and timing limits.
    my_tof.setDistanceMode(0) # 0=short, 1=medium, 2=long
    my_tof.setMeasurementTimingBudget(20000) # µs nicht! ms eingeben!
    # Groesse des ROI festlegen
    my_tof.setROISize(4,4)
    print('Range (mm)')
    while(1):
        scan_data = []
        # 13 Scanwinkel 0...12 = range(13), 4 Scanwinkel 0,4,8,12 = range(0,13,4)
        for angle in range(0,13,4):
            roi_center = 8*angle + 151;
            my_tof.setROICenter(roi_center) # ROI setzen
            my_tof.sensorReadSingle(blocking=True) # Abstandswert auslesen (blocking und nur Reichweite)       
            scan_data.append('{:6.1f}'.format(my_tof.ranging_data['range_mm']))
        print(scan_data)
except KeyboardInterrupt:
    print(' ')
    i2c.close()
    print('...i2c-bus closed.')
    print('ByeBye')
