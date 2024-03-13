# Klasse für den Ultraschallsensor SRF02
# 
# 13.3.24, S. Mack

from time import sleep

class UsSens:  
    def __init__(self, i2c_bus=None, ic2_addr=0x70):
        self.i2c_bus = i2c_bus # Nummer des I2C-Bus auf dem BeagleBone
        self.i2c_adress = ic2_addr # I2C-Adresse des Sensors

    def read_range(self): # Abstandsmesswert in cm
        self.i2c_bus.write_byte_data(self.i2c_adress, 0, 0x51)
        sleep(0.08) # 0.066 minimale Wartezeit
        # 2 Byte lesen ab Adresse 0x02
        data=self.i2c_bus.read_i2c_block_data(self.i2c_adress, 0x02, 2)
        # Messwert Big Endian > MSB und LSB tauschen
        val=(data[0]<<8) | data[1]
        return val
            
    def read_us_tof(self): # Echolaufzeit in µs
        self.i2c_bus.write_byte_data(self.i2c_adress, 0, 0x52)
        sleep(0.08) # 0.066 minimale Wartezeit
        # 2 Byte lesen ab Adresse 0x02
        data=self.i2c_bus.read_i2c_block_data(self.i2c_adress, 0x02, 2)
        # Messwert Big Endian > MSB und LSB tauschen
        val=(data[0]<<8) | data[1]
        return val
