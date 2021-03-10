# -*- coding: utf-8 -*-
# Beispiel-Skript für den STM TOF-Sensor VL53L1X
# Grundgeruest stammt aus VL53L1X library for Arduino von Pololu
# github.com/pololu/vl53l1x-arduino fuer Arduino.
# Modul VL53L1XRegAddr.py mit den Registeradressen muss im selben Verzeichnis sein.
# S. Mack, 24.2.21

import smbus
import sys
from time import sleep
from VL53L1XRegAddr import *

DEBUG_PRINT = False

I2C_ADDRESS = 0x29 # Default I2C-Adresse des Sensors
I2C_BUS_NO = 0x01 # Default I2C-Bus am BeagleBone 
TIMEOUT_MS = 500 # Timeout in Millisekunden

# value used in measurement timing budget calculationsassumes PresetMode is LOWPOWER_AUTONOMOUS
# vhv = LOWPOWER_AUTO_VHV_LOOP_DURATION_US + LOWPOWERAUTO_VHV_LOOP_BOUND(tuning parm default) * 
# LOWPOWER_AUTO_VHV_LOOP_DURATION_US = 245 + 3 * 245 = 980
# TIMING_GUARD = LOWPOWER_AUTO_OVERHEAD_BEFORE_A_RANGING + LOWPOWER_AUTO_OVERHEAD_BETWEEN_A_B_RANGING+ vhv
# 1448 + 2100 + 980 = 4528
TIMING_GUARD = 4528;
# value in DSS_CONFIG__TARGET_TOTAL_RATE_MCPS register, used in DSS calculations
TARGET_RATE = 0x0A00;


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Variablen ------------------------------------------------------------------
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

calibrated = False;
i2c_handle = 0x00; 
saved_vhv_init = 0x00;
saved_vhv_timeout = 0x00;
fast_osc_frequency = 0x0000;
osc_calibrate_val = 0x0000;

distance_mode = ('Short', 'Medium', 'Long', 'Unknown')
distance_mode_select = 1 # Medium als Default

results = {'range_status':0,'stream_count':0,'dss_actual_effective_spads_sd0':0,\
'ambient_count_rate_mcps_sd0':0,'sigma_sd0':0,'final_crosstalk_corrected_range_mm_sd0':0,\
'peak_signal_count_rate_crosstalk_corrected_mcps_sd0':0}

ranging_data = {'range_mm':0,'sigma_mm':0,'range_status':255,\
'peak_signal_count_rate_MCPS':0.0,'ambient_count_rate_MCPS':0.0}


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Allgemeine Funktionen-------------------------------------------------------
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

# Convert count rate from fixed point 9.7 format to float
def countRateFixedToFloat(count_rate_fixed):
    if DEBUG_PRINT: print('countRateFixedToFloat()')
    return (float(count_rate_fixed) / (1 << 7))

# Ein Byte schreiben an 16 Bit Adresse
def writeReg(register_address, data):
    if DEBUG_PRINT: print('writeReg(): Adr: ' + str(register_address) +' Data: ' + str(data))
    global i2c
    a1 = (register_address >> 8) & 0xFF
    a0 = register_address & 0xFF
    i2c.write_i2c_block_data(I2C_ADDRESS, a1, [a0, (data & 0xFF)])

# Zwei Bytes (ein Wort) schreiben an 16 Bit Adresse
def writeReg16Bit(register_address, data):
    if DEBUG_PRINT: print('writeReg16Bit(): Adr: ' + str(register_address) +' Data: ' + str(data))
    global i2c
    a1 = (register_address >> 8) & 0xFF
    a0 = register_address & 0xFF
    d1 = (data >> 8) & 0xFF
    d0 = data & 0xFF
    i2c.write_i2c_block_data(I2C_ADDRESS, a1, [a0, d1, d0])

# Vier Bytes (zwei Worte) schreiben an 16 Bit Adresse
def writeReg32Bit(register_address, data):
    if DEBUG_PRINT: print('writeReg32Bit(): Adr: ' + str(register_address) +' Data: ' + str(data))
    global i2c
    a1 = (register_address >> 8) & 0xFF
    a0 = register_address & 0xFF
    d3 = (data >> 24) & 0xFF
    d2 = (data >> 16) & 0xFF
    d1 = (data >> 8) & 0xFF
    d0 = data & 0xFF
    i2c.write_i2c_block_data(I2C_ADDRESS, a1, [a0, d3, d2, d1, d0])

# Ein Byte lesen von einer 16 Bit Adresse
def readReg(register_address):
    global i2c
    a1 = (register_address >> 8) & 0xFF
    a0 = register_address & 0xFF
    i2c.write_i2c_block_data(I2C_ADDRESS, a1, [a0])
    data = i2c.read_byte(I2C_ADDRESS)
    if DEBUG_PRINT: print('readReg(): Adr: ' + str(register_address) +' Data: ' + str(data))
    return data

# Zwei Bytes lesen von einer 16 Bit Adresse
def readReg16bit(register_address):
    global i2c
    a1 = (register_address >> 8) & 0xFF
    a0 = register_address & 0xFF
    i2c.write_i2c_block_data(I2C_ADDRESS, a1, [a0])
    data0 = i2c.read_byte(I2C_ADDRESS)
    data1 = i2c.read_byte(I2C_ADDRESS)
    if DEBUG_PRINT: print('readReg16Bit(): Adr: ' + str(register_address) +' Data: ' + str(data0) + " " + str(data1) + ' ' + str((data0 << 8) | (data1 & 0xFF)))
    return (data0 << 8) | (data1 & 0xFF)


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Sensorspezifische Funktionen------------------------------------------------
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

# Set the measurement timing budget in microseconds, which is the time allowed for one measurement. A longer 
# timing budget allows for more accuratemeasurements, based on VL53L1_SetMeasurementTimingBudgetMicroSeconds()
def setMeasurementTimingBudget(budget_us):
    if DEBUG_PRINT: print('setMeasurementTimingBudget()')
    if (budget_us <= TIMING_GUARD): return False
    range_config_timeout_us = (budget_us - TIMING_GUARD)
    if (range_config_timeout_us > 1100000): return False # FDA_MAX_TIMING_BUDGET_US * 2
    range_config_timeout_us = int(range_config_timeout_us/2)
    macro_period_us = calcMacroPeriod(readReg(RANGE_CONFIG__VCSEL_PERIOD_A))
    phasecal_timeout_mclks = timeoutMicrosecondsToMclks(1000, macro_period_us)
    if (phasecal_timeout_mclks > 0xFF): phasecal_timeout_mclks = 0xFF
    writeReg(PHASECAL_CONFIG__TIMEOUT_MACROP, phasecal_timeout_mclks)
    writeReg16Bit(MM_CONFIG__TIMEOUT_MACROP_A, encodeTimeout(timeoutMicrosecondsToMclks(1, macro_period_us)))
    writeReg16Bit(RANGE_CONFIG__TIMEOUT_MACROP_A, encodeTimeout(timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));
    macro_period_us = calcMacroPeriod(readReg(RANGE_CONFIG__VCSEL_PERIOD_B))
    writeReg16Bit(MM_CONFIG__TIMEOUT_MACROP_B, encodeTimeout(timeoutMicrosecondsToMclks(1, macro_period_us)))
    writeReg16Bit(RANGE_CONFIG__TIMEOUT_MACROP_B, encodeTimeout(timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)))
    return True;

# Get the measurement timing budget in microseconds, based on VL53L1_SetMeasurementTimingBudgetMicroSeconds()
def getMeasurementTimingBudget():
    if DEBUG_PRINT: print('getMeasurementTimingBudget()')
    macro_period_us = calcMacroPeriod(readReg(RANGE_CONFIG__VCSEL_PERIOD_A))
    range_config_timeout_us = timeoutMclksToMicroseconds(decodeTimeout(readReg16bit(\
    RANGE_CONFIG__TIMEOUT_MACROP_A)),macro_period_us)
    return  2 * range_config_timeout_us + TIMING_GUARD

# set distance mode to Short, Medium, or Long
def setDistanceMode(distance_mode_select):
    if DEBUG_PRINT: print('setDistanceMode()')
    budget_us = getMeasurementTimingBudget()
    if (distance_mode[distance_mode_select] == 'Short'):
        writeReg(RANGE_CONFIG__VCSEL_PERIOD_A, 0x07)
        writeReg(RANGE_CONFIG__VCSEL_PERIOD_B, 0x05)
        writeReg(RANGE_CONFIG__VALID_PHASE_HIGH, 0x38)
        writeReg(SD_CONFIG__WOI_SD0, 0x07)
        writeReg(SD_CONFIG__WOI_SD1, 0x05)
        writeReg(SD_CONFIG__INITIAL_PHASE_SD0, 6) # tuning parm default
        writeReg(SD_CONFIG__INITIAL_PHASE_SD1, 6) # tuning parm default
    elif (distance_mode[distance_mode_select] == 'Medium'):
        writeReg(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0B)
        writeReg(RANGE_CONFIG__VCSEL_PERIOD_B, 0x09)
        writeReg(RANGE_CONFIG__VALID_PHASE_HIGH, 0x78)
        writeReg(SD_CONFIG__WOI_SD0, 0x0B)
        writeReg(SD_CONFIG__WOI_SD1, 0x09)
        writeReg(SD_CONFIG__INITIAL_PHASE_SD0, 10) # tuning parm default
        writeReg(SD_CONFIG__INITIAL_PHASE_SD1, 10) # tuning parm default
    elif (distance_mode[distance_mode_select] == 'Long'):
        writeReg(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F)
        writeReg(RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D)
        writeReg(RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8)
        writeReg(SD_CONFIG__WOI_SD0, 0x0F)
        writeReg(SD_CONFIG__WOI_SD1, 0x0D)
        writeReg(SD_CONFIG__INITIAL_PHASE_SD0, 14) # tuning parm default
        writeReg(SD_CONFIG__INITIAL_PHASE_SD1, 14) # tuning parm default
    else:
        return False
    setMeasurementTimingBudget(budget_us)
    return True

# Initialize sensor using settings taken mostly from VL53L1_DataInit() and
# VL53L1_StaticInit(). If io_2v8 (optional) is True or not given, the sensor is configured for 2V8 mode.
def initSensor(io_2v8 = True):
    if DEBUG_PRINT: print('initSensor()')
    timeout_count = 0
    global fast_osc_frequency
    global osc_calibrate_val
    if DEBUG_PRINT: print('Sensor ID: '+str(readReg16bit(IDENTIFICATION__MODEL_ID)))
    #if (readReg16bit(IDENTIFICATION__MODEL_ID) != 0xEACC): return False
    writeReg(SOFT_RESET, 0x00)
    sleep(0.0001)
    writeReg(SOFT_RESET, 0x01)
    while ((readReg(FIRMWARE__SYSTEM_STATUS) & 0x01) == 0):
        sleep(0.01)
        timeout_count += 1
        if (timeout_count > TIMEOUT_MS/10): # Fehlermeldung bei ueberschreitung Timeout
            print('Timeout ocurred!') 
            return False
    if (io_2v8):
        writeReg(PAD_I2C_HV__EXTSUP_CONFIG, readReg(PAD_I2C_HV__EXTSUP_CONFIG) | 0x01)
    fast_osc_frequency = readReg16bit(OSC_MEASURED__FAST_OSC__FREQUENCY)
    osc_calibrate_val = readReg16bit(RESULT__OSC_CALIBRATE_VAL)
    writeReg16Bit(DSS_CONFIG__TARGET_TOTAL_RATE_MCPS, TARGET_RATE) # should already be this value after reset
    writeReg(GPIO__TIO_HV_STATUS, 0x02)
    writeReg(SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS, 8) # tuning parm default
    writeReg(SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS, 16) # tuning parm default
    writeReg(ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM, 0x01)
    writeReg(ALGO__RANGE_IGNORE_VALID_HEIGHT_MM, 0xFF)
    writeReg(ALGO__RANGE_MIN_CLIP, 0) # tuning parm default
    writeReg(ALGO__CONSISTENCY_CHECK__TOLERANCE, 2) # tuning parm default
    writeReg16Bit(SYSTEM__THRESH_RATE_HIGH, 0x0000)
    writeReg16Bit(SYSTEM__THRESH_RATE_LOW, 0x0000)
    writeReg(DSS_CONFIG__APERTURE_ATTENUATION, 0x38)
    writeReg16Bit(RANGE_CONFIG__SIGMA_THRESH, 360) # tuning parm default
    writeReg16Bit(RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, 192) # tuning parm default
    writeReg(SYSTEM__GROUPED_PARAMETER_HOLD_0, 0x01)
    writeReg(SYSTEM__GROUPED_PARAMETER_HOLD_1, 0x01)
    writeReg(SD_CONFIG__QUANTIFIER, 2) # tuning parm default
    writeReg(SYSTEM__GROUPED_PARAMETER_HOLD, 0x00)
    writeReg(SYSTEM__SEED_CONFIG, 1) # tuning parm default
    writeReg(SYSTEM__SEQUENCE_CONFIG, 0x8B) # VHV, PHASECAL, DSS1, RANGE
    writeReg16Bit(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 200 << 8)
    writeReg(DSS_CONFIG__ROI_MODE_CONTROL, 2) # REQUESTED_EFFFECTIVE_SPADS
    setDistanceMode(1)
    setMeasurementTimingBudget(50000)
    writeReg16Bit(ALGO__PART_TO_PART_RANGE_OFFSET_MM,readReg16bit(MM_CONFIG__OUTER_OFFSET_MM) * 4)
    return True

# check if sensor has new reading available, assumes interrupt is active low (GPIO_HV_MUX__CTRL bit 4 is 1)
def dataReady():
    if DEBUG_PRINT: print('dataReady()')
    return (readReg(GPIO__TIO_HV_STATUS) & 0x01) == 0

# Start continuous ranging measurements, with the given inter-measurement
# period in milliseconds determining how often the sensor takes a measurement.
def startContinuous(period_ms):
    writeReg32Bit(SYSTEM__INTERMEASUREMENT_PERIOD, period_ms * osc_calibrate_val)
    writeReg(SYSTEM__INTERRUPT_CLEAR, 0x01) # sys_interrupt_clear_range
    writeReg(SYSTEM__MODE_START, 0x40) # mode_range__timed

# Stop continuous measurements, based on VL53L1_stop_range()
def stopContinuous():
    if DEBUG_PRINT: print('stopContinuous()')
    global calibrated
    writeReg(SYSTEM__MODE_START, 0x80) # mode_range__abort
    calibrated = False
    if (saved_vhv_init != 0):
        writeReg(VHV_CONFIG__INIT, saved_vhv_init)
    if (saved_vhv_timeout != 0):
        writeReg(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, saved_vhv_timeout)
    writeReg(PHASECAL_CONFIG__OVERRIDE, 0x00)

# Returns a range reading in millimeters when continuous mode is active
# (readRangeSingleMillimeters() also calls this function after starting a single-shot range measurement)
def sensorRead(blocking=True):
    if DEBUG_PRINT: print('sensorRead()')
    global ranging_data, calibrated
    timeout_count = 0
    if (blocking):
        while not dataReady():
            sleep(0.01)
            timeout_count += 1
            if (timeout_count > TIMEOUT_MS/10): # Fehlermeldung bei ueberschreitung Timeout
                print('Timeout ocurred!') 
                ranging_data['range_status'] = None
                ranging_data['range_mm'] = 0
                ranging_data['sigma_mm'] = 0
                ranging_data['peak_signal_count_rate_MCPS'] = 0
                ranging_data['ambient_count_rate_MCPS'] = 0
    readResults()
    if not calibrated:
        setupManualCalibration()
        calibrated = True
    updateDSS()
    getRangingData()
    writeReg(SYSTEM__INTERRUPT_CLEAR, 0x01) # sys_interrupt_clear_range

# "Setup ranges after the first one in low power auto mode by turning off  FW calibration steps and programming static values"
# based on VL53L1_low_power_auto_setup_manual_calibration()
def setupManualCalibration():
    if DEBUG_PRINT: print('setupManualCalibration()')
    saved_vhv_init = readReg(VHV_CONFIG__INIT)
    saved_vhv_timeout = readReg(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND)
    writeReg(VHV_CONFIG__INIT, saved_vhv_init & 0x7F)
    writeReg(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,
    (saved_vhv_timeout & 0x03) + (3 << 2)) # tuning parm default (LOWPOWERAUTO_VHV_LOOP_BOUND_DEFAULT)
    writeReg(PHASECAL_CONFIG__OVERRIDE, 0x01)
    writeReg(CAL_CONFIG__VCSEL_START, readReg(PHASECAL_RESULT__VCSEL_START))

# read measurement results into buffer
def readResults():
    if DEBUG_PRINT: print('readResults()')
    global results
    results['range_status'] = readReg(RESULT__RANGE_STATUS)
    results['stream_count'] = readReg(RESULT__STREAM_COUNT)
    results['dss_actual_effective_spads_sd0'] = readReg16bit(RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0)
    results['ambient_count_rate_mcps_sd0'] = readReg16bit(RESULT__AMBIENT_COUNT_RATE_MCPS_SD0)
    results['final_crosstalk_corrected_range_mm_sd0'] = readReg16bit(RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0)
    results['peak_signal_count_rate_crosstalk_corrected_mcps_sd0'] = readReg16bit(RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0)
    results['sigma_sd0'] = readReg16bit(RESULT__SIGMA_SD0)

# perform Dynamic SPAD Selection calculation/update  based on VL53L1_low_power_auto_update_DSS()
def updateDSS():
    if DEBUG_PRINT: print('updateDSS()')
    spadCount = results['dss_actual_effective_spads_sd0']
    if (spadCount != 0):
        totalRatePerSpad = results['peak_signal_count_rate_crosstalk_corrected_mcps_sd0'] + results['ambient_count_rate_mcps_sd0']
        if (totalRatePerSpad > 0xFFFF): totalRatePerSpad = 0xFFFF
        totalRatePerSpad = totalRatePerSpad/spadCount
        if (totalRatePerSpad != 0):
            requiredSpads = (TARGET_RATE * spadCount) / totalRatePerSpad
            if (requiredSpads > 0xFFFF): requiredSpads = 0xFFFF
            writeReg16Bit(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, requiredSpads)
            return
    writeReg16Bit(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 0x8000)

# get range, status, rates from results buffer based on VL53L1_GetRangingMeasurementData()
def getRangingData():
    if DEBUG_PRINT: print('getRangingData()')
    global ranging_data
    global results
    range_val = results['final_crosstalk_corrected_range_mm_sd0']
    ranging_data['range_mm'] = (range_val * 2011 + 0x0400) / 0x0800
    ranging_data['sigma_mm'] = results['sigma_sd0'] # ?lt. API-Doku hier Float>Int noetig
    ranging_data['range_status'] = results['range_status']
    ranging_data['peak_signal_count_rate_MCPS'] = countRateFixedToFloat\
    (results['peak_signal_count_rate_crosstalk_corrected_mcps_sd0'])
    ranging_data['ambient_count_rate_MCPS'] = countRateFixedToFloat(results['ambient_count_rate_mcps_sd0'])


# Decode sequence step timeout in MCLKs from register value based on VL53L1_decode_timeout()
def decodeTimeout(reg_val):
    if DEBUG_PRINT: print('decodeTimeout()')
    return ((reg_val & 0xFF) << (reg_val >> 8)) + 1

# Encode sequence step timeout register value from timeout in MCLKs based on VL53L1_encode_timeout()
def encodeTimeout(timeout_mclks):
    if DEBUG_PRINT: print('encodeTimeout()')
    ls_byte = 0x00
    ms_byte = 0x00
    if (timeout_mclks > 0):
        ls_byte = timeout_mclks - 1
        while ((ls_byte & 0xFFFFFF00) > 0):
            ls_byte >>= 1
            ms_byte += 1
        return (ms_byte << 8) | (ls_byte & 0xFF)
    else:  return 0x00

# Convert sequence step timeout from macro periods to microseconds with given
# macro period in microseconds (12.12 format) based on VL53L1_calc_timeout_us()
def timeoutMclksToMicroseconds(timeout_mclks, macro_period_us):
    if DEBUG_PRINT: print('timeoutMclksToMicroseconds()')
    return (timeout_mclks * macro_period_us + 0x0800) >> 12

# Convert sequence step timeout from microseconds to macro periods with given
# macro period in microseconds (12.12 format) based on VL53L1_calc_timeout_mclks()
def timeoutMicrosecondsToMclks(timeout_us, macro_period_us):
    if DEBUG_PRINT: print('timeoutMicrosecondsToMclks()')
    return int(((timeout_us << 12) + (macro_period_us >> 1)) / macro_period_us)


# Calculate macro period in microseconds (12.12 format) with given VCSEL period
# assumes fast_osc_frequency has been read and stored based on VL53L1_calc_macro_period_us()
def calcMacroPeriod(vcsel_period):
    if DEBUG_PRINT: print('calcMacroPeriod()')
    global fast_osc_frequency
    pll_period_us = (0x01 << 30) / fast_osc_frequency
    vcsel_period_pclks = (vcsel_period + 1) << 1
    macro_period_us = int(2304 * pll_period_us)
    macro_period_us >>= 6
    macro_period_us *= vcsel_period_pclks
    macro_period_us >>= 6
    return macro_period_us


# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# main()----------------------------------------------------------------------
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

i2c = smbus.SMBus(2)

try:
    print("Python-Interpreter: {}\n".format(sys.version))
    if not initSensor(True): 
        print('Failed to detect and initialize sensor!')
    else:
        print('Sensor Initialisierung i.O.')
    
    # Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
    # You can change these settings to adjust the performance of the sensor, but
    # the minimum timing budget is 20 ms for short distance mode and 33 ms for
    # medium and long distance modes. See the VL53L1X datasheet for more
    # information on range and timing limits.
    setDistanceMode(1) # 0=short, 1=medium, 2=long
    setMeasurementTimingBudget(50000) #µs
    # Start continuous readings at a rate of one measurement every 50 ms (the
    # inter-measurement period). This period should be at least as long as the
    # timing budget.
    startContinuous(50)
    print('Range (mm)  Sigma (mm)  Range Status  Peak Sign Cnt  Ambient Cnt')
    while(1):
        # Abstandswert auslesen
        sensorRead(True)
        print('r={:6.1f} mm  r_s={:5.1f} mm   r_st={:2.0f}   PSigCnt={:5.1f}   AmbCnt={:4.1f}'.\
        format(ranging_data['range_mm'],ranging_data['sigma_mm'],ranging_data['range_status'],ranging_data['peak_signal_count_rate_MCPS'],ranging_data['ambient_count_rate_MCPS']))        
        sleep(0.5)
except KeyboardInterrupt:
    print(' ')
    i2c.close()
    print('...i2c-bus closed.')
    print('ByeBye')
