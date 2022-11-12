#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Beispiel-Skript für den STM TOF-Sensor VL53L1X
# Grundgeruest stammt aus VL53L1X library for Arduino von Pololu
# github.com/pololu/vl53l1x-arduino fuer Arduino.
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
    # Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
    # You can change these settings to adjust the performance of the sensor, but
    # the minimum timing budget is 20 ms for short distance mode and 33 ms for
    # medium and long distance modes. See the VL53L1X datasheet for more
    # information on range and timing limits.
    my_tof.setDistanceMode(1) # 0=short, 1=medium, 2=long
    my_tof.setMeasurementTimingBudget(50000) #µs
    # Start continuous readings at a rate of one measurement every 50 ms (the
    # inter-measurement period). This period should be at least as long as the
    # timing budget.
    my_tof.startContinuous(50)
    print('Range (mm)  Sigma (mm)  Range Status  Peak Sign Cnt  Ambient Cnt')
    while(1):
        # Abstandswert auslesen
        my_tof.sensorRead(True)
        print('r={:6.1f} mm  r_s={:5.1f} mm   r_st={:2.0f}   PSigCnt={:5.1f}   AmbCnt={:4.1f}'.\
        format(my_tof.ranging_data['range_mm'],my_tof.ranging_data['sigma_mm'],my_tof.ranging_data['range_status'],\
        my_tof.ranging_data['peak_signal_count_rate_MCPS'],my_tof.ranging_data['ambient_count_rate_MCPS']))        
        sleep(0.5)
except KeyboardInterrupt:
    print(' ')
    i2c.close() # I2C-Kommunikation beenden, damit keine Fehlermeldung bei erneuter Instanzierung
    print('...i2c-bus closed.')
    print('ByeBye')