#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Skript zum Aendern der I2C-Adresse am Ultraschallsensor SRF02.
# Achtung: Das LSB bei der I2C-Kommunikation gehoert nicht zur Adresse.
# Dies wird von der I2C-Bibliothek automatisch berueckschtigt, bei
# dem Schreibvorgang der neuen Adresse auf das Sensorregister jedoch
# nicht. Daher muss hier der Wert der neuen Adresse um ein Bit
# nach links geschoben (= mit 2 multipliziert)
# Kontrolle in der Linux Bash Shell mit 'i2cdetect -y -r 2'
# !!!! Unbedingt darauf achten, dass addr_neu gültige I2C-Adresse ist !!!
# Ansonsten keine Kommunikation mit Sensor mehr möglich > Sensor tot.
# S. Mack, 23.05.2020

import smbus

addr_alt = 0x72
addr_neu = 0x70

i2c = smbus.SMBus(2)
print('i2c-bus opened...')
print('Start Adressenaenderung...')

# i2c Kommunikation auf alter ! I2C-Adresse addr_alt auf Register 0x00
i2c.write_byte_data(addr_alt, 0x00, 0xA0)
i2c.write_byte_data(addr_alt, 0x00, 0xAA)
i2c.write_byte_data(addr_alt, 0x00, 0xA5)

# Der SRF02 erwartet die gesamten 8 Bit. Daher Adresse um 1 Bit verschoben.
# Z.B. Adresse 0x72 wird damit zu 0xE4
i2c.write_byte_data(addr_alt, 0x00, addr_neu << 1) 
# Nach diesem Befehl leuchtet die LED des SRF02 dauerhaft. Anschließend
# Versorgungsspannung aus- und einschalten
print('...Fertig')

# Vorher:
# beagle@beaglebone i2cdetect -y -r 2
#      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
# 00:          -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 50: -- -- -- -- UU UU UU UU -- -- -- -- -- -- -- -- 
# 60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 70: 70 -- -- -- -- -- -- --   

# Nachher:
# beagle@beaglebone i2cdetect -y -r 2 1                                                                                             
#      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
# 00:          -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 50: -- -- -- -- UU UU UU UU -- -- -- -- -- -- -- -- 
# 60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 70: -- -- 72 -- -- -- -- --
