# Programm zum Testen der Abstandssensorfunktion des Sensors ADPS9960
# Der Code stammt aus github.com/liske/python-apds9960 und basiert auf einer
# C++ Library von bitbucket.org/justin_woodman/apds-9960-raspberry-pi-library/src
# 4.11.2020 S. Mack

import smbus
from time import sleep
import sys

# APDS9960 i2c address
APDS9960_I2C_ADDR = 0x39

# APDS9960 device IDs
APDS9960_DEV_ID = [0xab, 0x9c, 0xa8]

# APDS9960 register addresses
APDS9960_REG_ENABLE = 0x80
APDS9960_REG_ATIME = 0x81
APDS9960_REG_WTIME = 0x83
APDS9960_REG_AILTL = 0x84
APDS9960_REG_AILTH = 0x85
APDS9960_REG_AIHTL = 0x86
APDS9960_REG_AIHTH = 0x87
APDS9960_REG_PILT = 0x89
APDS9960_REG_PIHT = 0x8b
APDS9960_REG_PERS = 0x8c
APDS9960_REG_CONFIG1 = 0x8d
APDS9960_REG_PPULSE = 0x8e
APDS9960_REG_CONTROL = 0x8f
APDS9960_REG_CONFIG2 = 0x90
APDS9960_REG_ID = 0x92
APDS9960_REG_STATUS = 0x93
APDS9960_REG_CDATAL = 0x94
APDS9960_REG_CDATAH = 0x95
APDS9960_REG_RDATAL = 0x96
APDS9960_REG_RDATAH = 0x97
APDS9960_REG_GDATAL = 0x98
APDS9960_REG_GDATAH = 0x99
APDS9960_REG_BDATAL = 0x9a
APDS9960_REG_BDATAH = 0x9b
APDS9960_REG_PDATA = 0x9c
APDS9960_REG_POFFSET_UR = 0x9d
APDS9960_REG_POFFSET_DL = 0x9e
APDS9960_REG_CONFIG3 = 0x9f

# APDS9960 bit fields
APDS9960_BIT_PON = 0b00000001
APDS9960_BIT_AEN = 0b00000010
APDS9960_BIT_PEN = 0b00000100
APDS9960_BIT_WEN = 0b00001000
APSD9960_BIT_AIEN =0b00010000
APDS9960_BIT_PIEN = 0b00100000
APDS9960_BIT_GEN = 0b01000000
APDS9960_BIT_GVALID = 0b00000001

# APDS9960 modes
APDS9960_MODE_POWER = 0
APDS9960_MODE_AMBIENT_LIGHT = 1
APDS9960_MODE_PROXIMITY = 2
APDS9960_MODE_WAIT = 3
APDS9960_MODE_AMBIENT_LIGHT_INT = 4
APDS9960_MODE_PROXIMITY_INT = 5
APDS9960_MODE_GESTURE = 6
APDS9960_MODE_ALL = 7

# LED Drive values
APDS9960_LED_DRIVE_100MA = 0
APDS9960_LED_DRIVE_50MA = 1
APDS9960_LED_DRIVE_25MA = 2
APDS9960_LED_DRIVE_12_5MA = 3

# Proximity Gain (PGAIN) values
APDS9960_PGAIN_1X = 0
APDS9960_PGAIN_2X = 1
APDS9960_PGAIN_4X = 2
APDS9960_PGAIN_8X = 3

# LED Boost values
APDS9960_LED_BOOST_100 = 0
APDS9960_LED_BOOST_150 = 1
APDS9960_LED_BOOST_200 = 2
APDS9960_LED_BOOST_300 = 3    

# Default values
APDS9960_DEFAULT_ATIME = 219                            # 103ms
APDS9960_DEFAULT_WTIME = 246                            # 27ms
APDS9960_DEFAULT_PROX_PPULSE = 0x87                     # 16us, 8 pulses
APDS9960_DEFAULT_POFFSET_UR = 0                         # 0 offset
APDS9960_DEFAULT_POFFSET_DL = 0                         # 0 offset
APDS9960_DEFAULT_CONFIG1 = 0x60                         # No 12x wait (WTIME) factor
APDS9960_DEFAULT_LDRIVE = APDS9960_LED_DRIVE_100MA
APDS9960_DEFAULT_PGAIN = APDS9960_PGAIN_4X
APDS9960_DEFAULT_AILT = 0xffff                          # Force interrupt for calibration
APDS9960_DEFAULT_AIHT = 0
APDS9960_DEFAULT_PERS = 0x11                            # 2 consecutive prox or ALS for int.
APDS9960_DEFAULT_CONFIG2 = 0x01                         # No saturation interrupts or LED boost  
APDS9960_DEFAULT_CONFIG3 = 0                            # Enable all photodiodes, no SAI


def apdsInit(i2c): #Sensorinitialisierung
    # check device id
    dev_id = read_byte_data(APDS9960_REG_ID)
    if not dev_id in APDS9960_DEV_ID:
        sys.exit('Device ID stimmt nicht -> Programmabbruch')
        
    # disable all features
    setMode(APDS9960_MODE_ALL, False)

    # set default values for ambient light and proximity registers
    write_byte_data(APDS9960_REG_ATIME, APDS9960_DEFAULT_ATIME)
    write_byte_data(APDS9960_REG_WTIME, APDS9960_DEFAULT_WTIME)
    write_byte_data(APDS9960_REG_PPULSE, APDS9960_DEFAULT_PROX_PPULSE)
    write_byte_data(APDS9960_REG_POFFSET_UR, APDS9960_DEFAULT_POFFSET_UR)
    write_byte_data(APDS9960_REG_POFFSET_DL, APDS9960_DEFAULT_POFFSET_DL)
    write_byte_data(APDS9960_REG_CONFIG1, APDS9960_DEFAULT_CONFIG1)
    setLEDDrive(APDS9960_DEFAULT_LDRIVE)
    
    write_byte_data(APDS9960_REG_PERS, APDS9960_DEFAULT_PERS)
    write_byte_data(APDS9960_REG_CONFIG2, APDS9960_DEFAULT_CONFIG2)
    write_byte_data(APDS9960_REG_CONFIG3, APDS9960_DEFAULT_CONFIG3)


def getMode():
    return read_byte_data(APDS9960_REG_ENABLE)
    
def setMode(mode, enable=True):
    # read ENABLE register
    reg_val = getMode()

    if mode < 0 or mode > APDS9960_MODE_ALL:
        sys.exit('Ausgewaehlter Modus unplausibel -> Programmabbruch')

    # change bit(s) in ENABLE register */
    if mode == APDS9960_MODE_ALL:
        if enable:
            reg_val = 0x7f # 0b01111111
        else:
            reg_val = 0x00
    else:
        if enable:
            reg_val |= (1 << mode); # An der mode-ten Stellewird eine 1 eingefuegt
        else:
            reg_val &= ~(1 << mode); # ~ = Komplementoperator (kippt jeweils die Bits)
            # An der mode-ten Stelle wird eine 0 eingefuegt
    # write value to ENABLE register
    write_byte_data(APDS9960_REG_ENABLE, reg_val)


# start the proximity sensor
def enableProximitySensor(interrupts=True):
    setProximityGain(APDS9960_DEFAULT_PGAIN)
    setLEDDrive(APDS9960_DEFAULT_LDRIVE)
    enablePower()
    setMode(APDS9960_MODE_PROXIMITY, True)

# turn the APDS-9960 on
def enablePower():
    setMode(APDS9960_MODE_POWER, True)


# *******************************************************************************
# Proximity sensor controls
# *******************************************************************************

# reads the proximity level as an 8-bit value
def readProximity():
    return read_byte_data(APDS9960_REG_PDATA)


# *******************************************************************************
# Getters and setters for register values
# *******************************************************************************

def setLEDDrive(drive):
    """Sets LED drive strength for proximity and ALS.
        Value    LED Current
          0        100 mA
          1         50 mA
          2         25 mA
          3         12.5 mA
        Args: drive (int): value for the LED drive strength
    """
    val = read_byte_data(APDS9960_REG_CONTROL)

    # set bits in register to given value
    drive &= 0b00000011
    drive = drive << 6
    val &= 0b00111111
    val |= drive

    write_byte_data(APDS9960_REG_CONTROL, val)

def setProximityGain(drive):
    """Sets receiver gain for proximity detection.
        Value    Gain
          0       1x
          1       2x
          2       4x
          3       8x
    """
    val = read_byte_data(APDS9960_REG_CONTROL)

    # set bits in register to given value
    drive &= 0b00000011
    drive = drive << 2
    val &= 0b11110011
    val |= drive

    write_byte_data(APDS9960_REG_CONTROL, val)


# *******************************************************************************
# Raw I2C Reads and Writes
# *******************************************************************************

def read_byte_data(cmd): # Ein Byte von Register cmd lesen
    #print("read_byte_data(Add,Val): {:x}, {:x}".format(cmd, i2c.read_byte_data(APDS9960_I2C_ADDR, cmd)))
    return i2c.read_byte_data(APDS9960_I2C_ADDR, cmd)

def write_byte_data(cmd, val): # Ein Byte val schreiben an Register cmd
    #print("write_byte_data(Add,Val): {:x}, {:x}".format(cmd, val))
    return i2c.write_byte_data(APDS9960_I2C_ADDR, cmd, val)
#def read_i2c_block_data(cmd, num): # Bytearray mit num Elementen von Register cmd lesen 
#    return i2c.read_i2c_block_data(APDS9960_I2C_ADDR, cmd, num)

# *******************************************************************************
# Hauptprogramm
# *******************************************************************************

print('Python-Interpreter: {}\n'.format(sys.version))
i2c = smbus.SMBus(2)
print('i2c-bus opened...')

apdsInit(i2c)

try:
    print("Python-Interpreter: {}\n".format(sys.version))
    print("APDS9960 Abstandssignal Miniprogramm")
    print("====================================")
    enableProximitySensor()
    while True:
        sleep(0.5)
        val = readProximity()
        print("proximity={}".format(val))
except KeyboardInterrupt:
    print(' ')
    i2c.close()
    print('...i2c-bus closed.')
    print('ByeBye')
