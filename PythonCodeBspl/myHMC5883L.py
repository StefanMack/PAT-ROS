# -*- coding: utf-8 -*-

#Beispielskript zum Auslesen des AMR-Sensors GY-271 von Reichelt.
#Chipaufdruck L883 2126. Verhält sich wie original HMC5883L,
#welcher abgekündigt ist.
#!! Neuere Versionen des GY-271 beinhalten jedoch den QMC5883L,
#der auf dem HMC5883L basiert, pinkompatibel ist jedoch eine
#andere I²C-Adresse und viel weniger Funktionen hat.
#
#Direkte I2C-Kommunikation via Linux:
#i2cdetect -y -r 2
#i2cdump -y 2 0x1e
#i2cset -y 2 0x1e 0x01 0xe0
#
# S. Mack, 4.11.20


import sys
import smbus
import time

i2c = smbus.SMBus(2)
print('i2c-bus opened...')

addr = 0x1e

# 16 Bit UInt in 2er-Kompliment umwandeln
def twos_comp(val):
	if (val >= 0x0001<<15): val = -(0xffff - val)
	return val
try:
	while True:
		# Mode Reg: Contineous Measurement		
		i2c.write_byte_data(addr, 0x02, 0x00)
		time.sleep(0.2)
		# Conf Reg B: Gain
		i2c.write_byte_data(addr, 0x01, 0x20) # 0.92 mG/LSB
		#i2c.write_byte_data(addr, 0x01, 0xe0) # 4.35 mG/LSB
		time.sleep(0.2)
		# z-Value Magnetic Field Data		
		val_msb = i2c.read_byte_data(addr, 0x05)
		val_lsb = i2c.read_byte_data(addr, 0x06)
		val = val_msb << 8 | val_lsb
		#print("val: {}".format(val))
		val = twos_comp(val)
		print("lsb: {}".format(val_lsb))
		print("msb: {}".format(val_msb))
		print("val two-comp: {}".format(val))
		print()
		time.sleep(1)
except KeyboardInterrupt:
	print('')
	i2c.close()
	print('...i2c-bus closed.')
	print('Byebye...')

