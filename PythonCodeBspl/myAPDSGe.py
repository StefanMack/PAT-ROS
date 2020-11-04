# Programm zum Testen der Gestensensorfunktion des Sensors ADPS9960
# ohne Triggerung durch die Abstandssensorfunktion.
# Es wird nur dier Signalveraluf für die vier verschiedenen Photodioden ausgegeben.
# Gesten werden daraus nicht berechnet.
# Der Code stammt aus github.com/liske/python-apds9960 und basiert auf einer
# C++ Library von bitbucket.org/justin_woodman/apds-9960-raspberry-pi-library/src
# 4.11.2020 S. Mack

import smbus
from time import sleep
import sys

# APDS9960 I2C Adresse
APDS9960_I2C_ADDR = 0x39
# APDS9960 Device IDs
APDS9960_DEV_ID = [0xab, 0x9c, 0xa8]
# I2C Latenzzeit (damit kein Fehler bei Buszugriffen) 
I2C_PAUSE = 0.001
# APDS9960 Registeradressen
APDS9960_REG_ENABLE = 0x80
APDS9960_REG_CONFIG2 = 0x90
APDS9960_REG_ID = 0x92
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
# LED Boost values
APDS9960_LED_BOOST_100 = 0
APDS9960_LED_BOOST_150 = 1
APDS9960_LED_BOOST_200 = 2
APDS9960_LED_BOOST_300 = 3    
# Default values
APDS9960_DEFAULT_GESTURE_PPULSE = 0x89                  # 16us, 10 pulses
APDS9960_DEFAULT_CONFIG1 = 0x60                         # No 12x wait (WTIME) factor
APDS9960_DEFAULT_LDRIVE = APDS9960_LED_DRIVE_100MA
APDS9960_DEFAULT_CONFIG2 = 0x01                         # No saturation interrupts or LED boost  
APDS9960_DEFAULT_CONFIG3 = 0                            # Enable all photodiodes, no SAI
APDS9960_DEFAULT_GCONF1 = 0x40  
APDS9960_DEFAULT_GWTIME = APDS9960_GWTIME_2_8MS
APDS9960_DEFAULT_GPULSE = 0xc9                          # 32us, 10 pulses
APDS9960_DEFAULT_GCONF3 = 0                             # Alle vier Photodioden aktiv
APDS9960_DEFAULT_GIEN = 0                               # Gesture Engine ohne Interruptsteuerung

# Variablen fuer Gestenerkennung
u_data = [0] * 32 # Sensorwert fuer u Photodiode
d_data = [0] * 32
l_data = [0] * 32
r_data = [0] * 32

def apdsInit(i2c): #Sensorinitialisierung
    # check device id
    dev_id = read_byte_data(APDS9960_REG_ID)
    if not dev_id in APDS9960_DEV_ID:
        sys.exit('Device ID stimmt nicht -> Programmabbruch')
    # Alle Funktionen ausschalten
    setMode(APDS9960_MODE_ALL, False) # Alle Bits in 0x80 auf Null setzen (reset)
    # Fuer Gestensensor relevante Werte setzen 
    write_byte_data(APDS9960_REG_GCONF1, APDS9960_DEFAULT_GCONF1) # Diverse Gesture-Einstellungen
    setGestureGain(APDS9960_GGAIN_4X) # Photodiodengain 4x (max. 8x) betrifft GCONF2
    setGestureLEDDrive(APDS9960_LED_DRIVE_100MA) # max. LED-Strom betrifft GCONF2
    write_byte_data(APDS9960_REG_GCONF3, APDS9960_DEFAULT_GCONF3) # Gesture dimension: u/d und r/l alle aktiviert
    setLEDBoost(APDS9960_LED_BOOST_300) # LED-Strom zusaetzlich um 300 % erhoehen (max. Detektionsempfindlichkeit)
    write_byte_data(APDS9960_REG_GOFFSET_U, 10) # Sensorspezifisch einstellen, Gain beruecksichtigen !
    write_byte_data(APDS9960_REG_GOFFSET_D, 0)
    write_byte_data(APDS9960_REG_GOFFSET_L, 0)
    write_byte_data(APDS9960_REG_GOFFSET_R, 10)
    write_byte_data(APDS9960_REG_GPULSE, APDS9960_DEFAULT_GPULSE) # Anzahl Pulse Gestenmessung
    setGestureEnterThresh(0) # Vermutlich noetig, auch wenn kein Interrupt aktiv, sonst keine kontinuierliche Messung
    setGestureExitThresh(0) # Vermutlich noetig, auch wenn kein Interrupt aktiv, sonst keine kontinuierliche Messung
    setGestureMode(False) # Gesten-Interrupt ausschalten
    enablePower() # Schlafmoduls beenden
    setMode(APDS9960_MODE_GESTURE, True) # Gesture Enable Bit GEN setzen
    readGestureRaw() # Dummy Leseprozess

def getMode(): # Enable Register auslesen
    return read_byte_data(APDS9960_REG_ENABLE)
    
def setMode(mode, enable=True): # Enable Register einstellen
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
            reg_val |= (1 << mode); # An der mode-ten Stelle wird eine 1 eingefuegt
        else:
            reg_val &= ~(1 << mode); # ~ = Komplementoperator (kippt jeweils die Bits)
            # An der mode-ten Stelle wird eine 0 eingefuegt
    # write value to ENABLE register
    write_byte_data(APDS9960_REG_ENABLE, reg_val)

# Sleep Modus sensor beenden
def enablePower():
    setMode(APDS9960_MODE_POWER, True)

# *******************************************************************************
# Photodiodensignal up, down, left und right ohne internen Interrupt
# *******************************************************************************
def readGestureRaw():   
    #print 'readGestureRaw()'
    fifoData = [] # Lesepuffer FIFO-Speicher
    setGestureMode(True) # Gesture Enginge (d.h. Messreihe) starten mit Setzen GMODE Bit
    while int(read_byte_data(APDS9960_REG_GFLVL)) < 32: # Warten bis 32 Messwerte da
        #print 'ich warte ...'
        sleep(I2C_PAUSE) # Wartezeit Lesezugriff FIFO
    setGestureMode(False) # Gesture Enginge stoppen mit Loeschen GMODE Bit
    #print 'FIFO Level', int(read_byte_data(APDS9960_REG_GFLVL))
    #print 'FIFO Overflow', read_byte_data(APDS9960_REG_GSTATUS)
    for i in range(0, 32): # Daten auslesen und in die Listen fuer u, d, l, und r abspeichern
        fifoData = read_i2c_block_data(APDS9960_REG_GFIFO_U, 4) # Groessere Blocks funktionieren nicht
        u_data[i] = fifoData[0]
        d_data[i] = fifoData[1]
        l_data[i] = fifoData[2]
        r_data[i] = fifoData[3]
        sleep(I2C_PAUSE) # Wartezeit Lesezugriff FIFO
    #setGestureMode(False) # Gesture Enginge stoppen mit Loeschen GMODE Bit

# *******************************************************************************
# Getters and setters for register values
# *******************************************************************************
def setLEDBoost(boost): # Sendeleistung LED einstellen (Boost)
    val = read_byte_data(APDS9960_REG_CONFIG2)
    # set bits in register to given value
    boost &= 0b00000011
    boost = boost << 4
    val &= 0b11001111
    val |= boost
    write_byte_data(APDS9960_REG_CONFIG2, val)

def setGestureLEDDrive(drive): # Sendeleistung LED einstellen (Strom)
    val = read_byte_data(APDS9960_REG_GCONF2)
    # set bits in register to given value
    drive &= 0b00000011
    drive = drive << 3
    val &= 0b11100111
    val |= drive
    write_byte_data(APDS9960_REG_GCONF2, val)

def setGestureEnterThresh(threshold): # Proximity Schwelle einstellen ab der Gesture Enginge abtastet
    write_byte_data(APDS9960_REG_GPENTH, threshold)
        
def setGestureExitThresh(threshold): # Proximity Schwelle einstellen ab der Gesture Enginge Abtastung stoppt
    write_byte_data(APDS9960_REG_GEXTH, threshold)
        
def setGestureGain(gain): # Photodiodeverstaerkung im Gesture Mode
    val = read_byte_data(APDS9960_REG_GCONF2)
    gain &= 0b00000011
    gain = gain << 5
    val &= 0b10011111
    val |= gain
    write_byte_data(APDS9960_REG_GCONF2, val)

def setGestureMode(enable): # GMODE Bit ein- oder ausschalten zum Starten/Stoppen Gesture Engine
    val = read_byte_data(APDS9960_REG_GCONF4)
    # set bits in register to given value
    val &= 0b11111110
    if enable:
        val |= 0b00000001
    write_byte_data(APDS9960_REG_GCONF4, val)  

# *******************************************************************************
# I2C Buskommunikation
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
i=0
i2c = smbus.SMBus(2)
apdsInit(i2c) #Grundeinstellungen am Sensor vornehmen
try:
    print("Python-Interpreter: {}\n".format(sys.version))
    print("APDS9960 Gestenrohsignal ohne internen Interrupt Miniprogramm")
    print("=============================================================")
    while True:
        sleep(0.1)
        readGestureRaw()
        i+=1
        print(i)
        print('Result u: ',u_data)
        print('Result d: ',d_data)
        print('Result l: ',l_data)
        print('Result r: ',r_data)
        print('---------------------------------------------------------------------------------')
except KeyboardInterrupt:
    print(' ')
    i2c.close()
    print('...i2c-bus closed.')
    print('ByeBye')
