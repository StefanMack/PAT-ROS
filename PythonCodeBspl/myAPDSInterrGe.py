# Programm zum Testen der Gestensensorfunktion des Sensors ADPS9960
# **mit** Triggerung durch die Abstandssensorfunktion.
# Es wird nur dier Signalveraluf für die vier verschiedenen Photodioden ausgegeben.
# Gesten werden daraus nicht berechnet.
# Der Code stammt aus github.com/liske/python-apds9960 und basiert auf einer
# C++ Library von bitbucket.org/justin_woodman/apds-9960-raspberry-pi-library/src
# 4.11.2020 S. Mack

import smbus
from time import sleep

# APDS9960 i2c address
APDS9960_I2C_ADDR = 0x39

# APDS9960 device IDs
APDS9960_DEV_ID = [0xab, 0x9c, 0xa8]

# APDS9960 times
APDS9960_TIME_FIFO_PAUSE = 0.03

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
APDS9960_REG_GPENTH = 0xa0
APDS9960_REG_GEXTH = 0xa1
APDS9960_REG_GCONF1 = 0xa2
APDS9960_REG_GCONF2 = 0xa3
APDS9960_REG_GCONF3 = 0xaA
APDS9960_REG_GCONF4 = 0xaB
APDS9960_REG_GFLVL = 0xae
APDS9960_REG_GSTATUS = 0xaf
APDS9960_REG_GOFFSET_U = 0xa4
APDS9960_REG_GOFFSET_D = 0xa5
APDS9960_REG_GOFFSET_L = 0xa7
APDS9960_REG_GOFFSET_R = 0xa9
APDS9960_REG_GPULSE = 0xa6
APDS9960_REG_GFIFO_U = 0xfc
APDS9960_REG_GFIFO_D = 0xfd
APDS9960_REG_GFIFO_L = 0xfe
APDS9960_REG_GFIFO_R = 0xff


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

# Gesture Gain (GGAIN) values
APDS9960_GGAIN_1X = 0
APDS9960_GGAIN_2X = 1
APDS9960_GGAIN_4X = 2
APDS9960_GGAIN_8X = 3

# Gesture wait time values
APDS9960_GWTIME_0MS = 0
APDS9960_GWTIME_2_8MS = 1
APDS9960_GWTIME_5_6MS = 2
APDS9960_GWTIME_8_4MS = 3
APDS9960_GWTIME_14_0MS = 4
APDS9960_GWTIME_22_4MS = 5
APDS9960_GWTIME_30_8MS = 6
APDS9960_GWTIME_39_2MS = 7

# ALS Gain (AGAIN) values
APDS9960_AGAIN_1X = 0
APDS9960_AGAIN_4X = 1
APDS9960_AGAIN_16X = 2
APDS9960_AGAIN_64X = 3

# LED Boost values
APDS9960_LED_BOOST_100 = 0
APDS9960_LED_BOOST_150 = 1
APDS9960_LED_BOOST_200 = 2
APDS9960_LED_BOOST_300 = 3    

# Default values
APDS9960_DEFAULT_ATIME = 219                            # 103ms
APDS9960_DEFAULT_WTIME = 246                            # 27ms
APDS9960_DEFAULT_PROX_PPULSE = 0x87                     # 16us, 8 pulses
APDS9960_DEFAULT_GESTURE_PPULSE = 0x89                  # 16us, 10 pulses
APDS9960_DEFAULT_POFFSET_UR = 0                         # 0 offset
APDS9960_DEFAULT_POFFSET_DL = 0                         # 0 offset
APDS9960_DEFAULT_CONFIG1 = 0x60                         # No 12x wait (WTIME) factor
APDS9960_DEFAULT_LDRIVE = APDS9960_LED_DRIVE_100MA
APDS9960_DEFAULT_AGAIN = APDS9960_AGAIN_4X
APDS9960_DEFAULT_AILT = 0xffff                          # Force interrupt for calibration
APDS9960_DEFAULT_AIHT = 0
APDS9960_DEFAULT_PERS = 0x11                            # 2 consecutive prox or ALS for int.
APDS9960_DEFAULT_CONFIG2 = 0x01                         # No saturation interrupts or LED boost  
APDS9960_DEFAULT_CONFIG3 = 0                            # Enable all photodiodes, no SAI
APDS9960_DEFAULT_GPENTH = 40                           # Threshold for entering gesture mode
APDS9960_DEFAULT_GEXTH = 30                             # Threshold for exiting gesture mode   
APDS9960_DEFAULT_GCONF1 = 0x40  
APDS9960_DEFAULT_GGAIN = APDS9960_GGAIN_4X
APDS9960_DEFAULT_GLDRIVE = APDS9960_LED_DRIVE_100MA
APDS9960_DEFAULT_GWTIME = APDS9960_GWTIME_2_8MS
APDS9960_DEFAULT_GOFFSET = 0                            # No offset scaling for gesture mode
APDS9960_DEFAULT_GPULSE = 0xc9                          # 32us, 10 pulses
APDS9960_DEFAULT_GCONF3 = 0                             # All photodiodes active during gesture
APDS9960_DEFAULT_GIEN = 0                               # Disable gesture interrupts

# Variablen fuer Gestenerkennung
u_data = [0] * 32 # Sensorwert fuer u Photodiode
d_data = [0] * 32
l_data = [0] * 32
r_data = [0] * 32


def apdsInit(i2c): #Sensorinitialisierung
    #gesture_data_ = APDS9960.GestureData()
    
    # check device id
    dev_id = read_byte_data(APDS9960_REG_ID)
    if not dev_id in APDS9960_DEV_ID:
        sys.exit('Device ID stimmt nicht -> Programmabbruch')
       
    # disable all features
    setMode(APDS9960_MODE_ALL, False)

    # set default values 
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
    setGestureEnterThresh(APDS9960_DEFAULT_GPENTH)
    setGestureExitThresh(APDS9960_DEFAULT_GEXTH)
    write_byte_data(APDS9960_REG_GCONF1, APDS9960_DEFAULT_GCONF1)
    setGestureGain(APDS9960_DEFAULT_GGAIN)
    setGestureLEDDrive(APDS9960_DEFAULT_GLDRIVE)
    setGestureWaitTime(APDS9960_DEFAULT_GWTIME)
    write_byte_data(APDS9960_REG_GOFFSET_U, APDS9960_DEFAULT_GOFFSET)
    write_byte_data(APDS9960_REG_GOFFSET_D, APDS9960_DEFAULT_GOFFSET)
    write_byte_data(APDS9960_REG_GOFFSET_L, APDS9960_DEFAULT_GOFFSET)
    write_byte_data(APDS9960_REG_GOFFSET_R, APDS9960_DEFAULT_GOFFSET)
    write_byte_data(APDS9960_REG_GPULSE, APDS9960_DEFAULT_GPULSE)
    write_byte_data(APDS9960_REG_GCONF3, APDS9960_DEFAULT_GCONF3)
    setGestureIntEnable(APDS9960_DEFAULT_GIEN)


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

# Gestensensor starten
def enableGestureSensor():
    write_byte_data(APDS9960_REG_WTIME, 0xff)
    write_byte_data(APDS9960_REG_PPULSE, APDS9960_DEFAULT_GESTURE_PPULSE)
    setLEDBoost(APDS9960_LED_BOOST_300)
    setGestureIntEnable(True)
    setGestureMode(True)
    enablePower()
    setMode(APDS9960_MODE_WAIT, True)
    setMode(APDS9960_MODE_PROXIMITY, True)
    setMode(APDS9960_MODE_GESTURE, True) 

# turn the APDS-9960 on
def enablePower():
    setMode(APDS9960_MODE_POWER, True)

# *******************************************************************************
# Messwerte Photodioden up, down, left und right (Rohdaten Gestenerkennung)
# *******************************************************************************

# Nachschauen ob eine Gest erkannt wurde.
def gestureAvailable():
    val = read_byte_data(APDS9960_REG_GSTATUS)
    # shift and mask out GVALID bit
    val &= APDS9960_BIT_GVALID
    return (val == APDS9960_BIT_GVALID)
    
# Werte der vier Photodioden u, d, l und r auslesen fuer 4 aufeinanderfolgende Abtastungen.
# Damit wird in processGesture() per Sensorfusion die Geste ermittelt
def readGesture():    
    fifo_level = 0 # Anzahl Messwerte im FIFO-Speicher
    fifo_data = [] # Lesepuffer FIFO-Speicher

    # make sure that power and gesture is on and data is valid
    if not (getMode() & 0b01000001) or not gestureAvailable():
        return APDS9960_DIR_NONE

    # keep looping as long as gesture data is valid
    while(gestureAvailable()):
        fifo_level = read_byte_data(APDS9960_REG_GFLVL) # Anzahl bereitliegender Datensaetze
        # if there's stuff in the FIFO, read it into our data block
        if fifo_level > 0:
            fifo_data = [] # Lesepuffer loeschen
            for i in range(0, fifo_level):
                fifo_data += read_i2c_block_data(APDS9960_REG_GFIFO_U, 4)
            # if at least 1 set of data, sort the data into U/D/L/R
            if len(fifo_data) >= 4:
                print('len(fifo_data) {}'.format(len(fifo_data)))
                for i in range(0, len(fifo_data), 4):
                    #print('i / i/4', i, (i/4))
                    u_data[int(i/4)] = fifo_data[i + 0]
                    d_data[int(i/4)] = fifo_data[i + 1]
                    l_data[int(i/4)] = fifo_data[i + 2]
                    r_data[int(i/4)] = fifo_data[i + 3]
        # wait some time to collect next batch of FIFO data
        sleep(APDS9960_TIME_FIFO_PAUSE)
    # determine best guessed gesture and clean up
    sleep(APDS9960_TIME_FIFO_PAUSE)
    return fifo_level

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
        Args: drive (int): value for the LED drive strength"""
    val = read_byte_data(APDS9960_REG_CONTROL)
    # set bits in register to given value
    drive &= 0b00000011
    drive = drive << 6
    val &= 0b00111111
    val |= drive
    write_byte_data(APDS9960_REG_CONTROL, val)
    
def setLEDBoost(boost):
    """Sets the LED current boost value.
        Value    Gain
          0        100%
          1        150%
          2        200%
          3        300%
        Args: boost (int): value for the LED boost"""
    val = read_byte_data(APDS9960_REG_CONFIG2)
    # set bits in register to given value
    boost &= 0b00000011
    boost = boost << 4
    val &= 0b11001111
    val |= boost
    write_byte_data(APDS9960_REG_CONFIG2, val)

def setGestureLEDDrive(drive):
    """Sets LED drive strength for proximity and ALS.
        Value    LED Current
          0        100 mA
          1         50 mA
          2         25 mA
          3         12.5 mA
        Args: drive (int): value for the LED drive current"""
    val = read_byte_data(APDS9960_REG_GCONF2)
    # set bits in register to given value
    drive &= 0b00000011;
    drive = drive << 3;
    val &= 0b11100111;
    val |= drive;
    write_byte_data(APDS9960_REG_GCONF2, val)

def setGestureEnterThresh(threshold):
    """Sets the entry proximity threshold for gesture sensing. Args:
    threshold (int): threshold proximity value needed to start gesture mode"""
    write_byte_data(APDS9960_REG_GPENTH, threshold)
        
def setGestureExitThresh(threshold):
    """Sets the exit proximity threshold for gesture sensing. Args:
    threshold (int): threshold proximity value needed to end gesture mode"""
    write_byte_data(APDS9960_REG_GEXTH, threshold)
        
def setGestureGain(gain):
    """Sets the gain of the photodiode during gesture mode.
        Value    Gain
          0       1x
          1       2x
          2       4x
          3       8x
    Args: gain (int): the value for the photodiode gain"""
    val = read_byte_data(APDS9960_REG_GCONF2)
    # set bits in register to given value
    gain &= 0b00000011
    gain = gain << 5
    val &= 0b10011111
    val |= gain
    write_byte_data(APDS9960_REG_GCONF2, val)

def setGestureWaitTime(time):
    """Sets the time in low power mode between gesture detections.
        Value    Wait time
          0          0 ms
          1          2.8 ms
          2          5.6 ms
          3          8.4 ms
          4         14.0 ms
          5         22.4 ms
          6         30.8 ms
          7         39.2 ms
        Args: time (int): value for the wait time"""
    val = read_byte_data(APDS9960_REG_GCONF2)
    # set bits in register to given value
    time &= 0b00000111
    val &= 0b11111000
    val |= time
    write_byte_data(APDS9960_REG_GCONF2, val)

def setGestureIntEnable(enable):
    """Turns gesture-related interrupts on or off.
    Args:enable (bool): True to enable interrupts, False to turn them off"""
    val = read_byte_data(APDS9960_REG_GCONF4)
    # set bits in register to given value    
    val &= 0b11111101
    if enable:
        val |= 0b00000010
    write_byte_data(APDS9960_REG_GCONF4, val)

def setGestureMode(enable):
    """Turns gesture-related interrupts on or off.
    Args: enable (bool): True to enter gesture state machine, False to turn them off"""
    val = read_byte_data(APDS9960_REG_GCONF4)
    # set bits in register to given value
    val &= 0b11111110
    if enable:
        val |= 0b00000001
    write_byte_data(APDS9960_REG_GCONF4, val)  


# *******************************************************************************
# Raw I2C Reads and Writes
# *******************************************************************************

def read_byte_data(cmd): # Ein Byte von Register cmd lesen
    #print("read_byte_data(Add,Val): {:x}, {:x}".format(cmd, i2c.read_byte_data(APDS9960_I2C_ADDR, cmd)))
    return i2c.read_byte_data(APDS9960_I2C_ADDR, cmd)

def write_byte_data(cmd, val): # Ein Byte val schreiben an Register cmd
    #print("write_byte_data(Add,Val): {:x}, {:x}".format(cmd, val))
    return i2c.write_byte_data(APDS9960_I2C_ADDR, cmd, val)
    
def read_i2c_block_data(cmd, num): # Bytearray mit num Elementen von Register cmd lesen 
    return i2c.read_i2c_block_data(APDS9960_I2C_ADDR, cmd, num)    

# *******************************************************************************
# Hauptprogramm
# *******************************************************************************

i2c = smbus.SMBus(2)

apdsInit(i2c)

try:
    print("APDS9960 Gestensensor Miniprogramm mit Interrupt")
    print(">>>>>>>>Messung erst nach Objekterkennung<<<<<<<")
    print("================================================")
    enableGestureSensor()
    while True:
        sleep(0.5)
        if gestureAvailable():
            gestDataPoints = readGesture()
            print('Gesture Data Points:', gestDataPoints)
            print('Result u: ',u_data[0:gestDataPoints])
            print('Result d: ',d_data[0:gestDataPoints])
            print('Result l: ',l_data[0:gestDataPoints])
            print('Result r: ',r_data[0:gestDataPoints])
            print('---------------------------------------------------------------------------------')
except KeyboardInterrupt:
    print(' ')
    i2c.close()
    print('...i2c-bus closed.')
    print('ByeBye')
