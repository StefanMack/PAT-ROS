#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Klasse für den STM TOF-Sensor VL53L1X
# Grundgeruest stammt aus VL53L1X library for Arduino von Pololu
# github.com/pololu/vl53l1x-arduino fuer Arduino.
# 12.11.22, S. Mack

from time import sleep
import VL53L1XRegAddr as REG

DEBUG_PRINT = False

# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Allgemeine Funktionen-------------------------------------------------------
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

# Convert count rate from fixed point 9.7 format to float
def countRateFixedToFloat(count_rate_fixed):
    if DEBUG_PRINT: print('countRateFixedToFloat()')
    return (float(count_rate_fixed) / (1 << 7))

# Ein Byte schreiben an 16 Bit Adresse
def writeReg(i2c, i2c_adress, register_address, data):
    if DEBUG_PRINT: print('writeReg(): Adr: ' + str(register_address) +' Data: ' + str(data))
    a1 = (register_address >> 8) & 0xFF
    a0 = register_address & 0xFF
    i2c.write_i2c_block_data(i2c_adress, a1, [a0, (data & 0xFF)])

# Zwei Bytes (ein Wort) schreiben an 16 Bit Adresse
def writeReg16Bit(i2c, i2c_adress, register_address, data):
    if DEBUG_PRINT: print('writeReg16Bit(): Adr: ' + str(register_address) +' Data: ' + str(data))
    a1 = (register_address >> 8) & 0xFF
    a0 = register_address & 0xFF
    d1 = (data >> 8) & 0xFF
    d0 = data & 0xFF
    i2c.write_i2c_block_data(i2c_adress, a1, [a0, d1, d0])

# Vier Bytes (zwei Worte) schreiben an 16 Bit Adresse
def writeReg32Bit(i2c, i2c_adress, register_address, data):
    if DEBUG_PRINT: print('writeReg32Bit(): Adr: ' + str(register_address) +' Data: ' + str(data))
    a1 = (register_address >> 8) & 0xFF
    a0 = register_address & 0xFF
    d3 = (data >> 24) & 0xFF
    d2 = (data >> 16) & 0xFF
    d1 = (data >> 8) & 0xFF
    d0 = data & 0xFF
    i2c.write_i2c_block_data(i2c_adress, a1, [a0, d3, d2, d1, d0])

# Ein Byte lesen von einer 16 Bit Adresse
def readReg(i2c, i2c_adress, register_address):
    a1 = (register_address >> 8) & 0xFF
    a0 = register_address & 0xFF
    i2c.write_i2c_block_data(i2c_adress, a1, [a0])
    data = i2c.read_byte(i2c_adress)
    if DEBUG_PRINT: print('readReg(): Adr: ' + str(register_address) +' Data: ' + str(data))
    return data

# Zwei Bytes lesen von einer 16 Bit Adresse
def readReg16Bit(i2c, i2c_adress, register_address):
    a1 = (register_address >> 8) & 0xFF
    a0 = register_address & 0xFF
    i2c.write_i2c_block_data(i2c_adress, a1, [a0])
    data0 = i2c.read_byte(i2c_adress)
    data1 = i2c.read_byte(i2c_adress)
    if DEBUG_PRINT: print('readReg16Bit(): Adr: ' + str(register_address) +' Data: ' + str(data0) + " " + str(data1) + ' ' + str((data0 << 8) | (data1 & 0xFF)))
    return (data0 << 8) | (data1 & 0xFF)


class TofSens:  
    def __init__(self, i2c_bus=None, ic2_addr=0x29, timeout_ms=500):
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Attribute ------------------------------------------------------------------
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        self.calibrated = False; 
        self.saved_vhv_init = 0x00;
        self.saved_vhv_timeout = 0x00;
        self.fast_osc_frequency = 0x0000;
        self.osc_calibrate_val = 0x0000;
        self.distance_mode = ('Short', 'Medium', 'Long', 'Unknown')
        self.distance_mode_select = 1 # Medium als Default
        self.results = {'range_status':0,'stream_count':0,'dss_actual_effective_spads_sd0':0,\
        'ambient_count_rate_mcps_sd0':0,'sigma_sd0':0,'final_crosstalk_corrected_range_mm_sd0':0,\
        'peak_signal_count_rate_crosstalk_corrected_mcps_sd0':0}
        self.ranging_data = {'range_mm':0,'sigma_mm':0,'range_status':255,\
        'peak_signal_count_rate_MCPS':0.0,'ambient_count_rate_MCPS':0.0}
        self.i2c_bus = i2c_bus # Nummer des I2C-Bus auf dem BeagleBone
        self.i2c_adress = ic2_addr # I2C-Adresse des Sensors
        self.timeout_ms = timeout_ms # Timeout in Millisekunden
        # value used in measurement timing budget calculationsassumes PresetMode is LOWPOWER_AUTONOMOUS
        # vhv = LOWPOWER_AUTO_VHV_LOOP_DURATION_US + LOWPOWERAUTO_VHV_LOOP_BOUND(tuning parm default) * 
        # LOWPOWER_AUTO_VHV_LOOP_DURATION_US = 245 + 3 * 245 = 980
        # TIMING_GUARD = LOWPOWER_AUTO_OVERHEAD_BEFORE_A_RANGING + LOWPOWER_AUTO_OVERHEAD_BETWEEN_A_B_RANGING+ vhv
        # 1448 + 2100 + 980 = 4528
        self.timing_guard = 4528
        # value in DSS_CONFIG__TARGET_TOTAL_RATE_MCPS register, used in DSS calculations
        self.target_rate = 0x0A00


    # Set the measurement timing budget in microseconds, which is the time allowed for one measurement. A longer 
    # timing budget allows for more accuratemeasurements, based on VL53L1_SetMeasurementTimingBudgetMicroSeconds()
    def setMeasurementTimingBudget(self, budget_us):
        if DEBUG_PRINT: print('setMeasurementTimingBudget()')
        if (budget_us <= self.timing_guard): return False
        range_config_timeout_us = (budget_us - self.timing_guard)
        if (range_config_timeout_us > 1100000): return False # FDA_MAX_TIMING_BUDGET_US * 2
        range_config_timeout_us = int(range_config_timeout_us/2)
        macro_period_us = self.calcMacroPeriod(readReg(self.i2c_bus, self.i2c_adress,REG.RANGE_CONFIG__VCSEL_PERIOD_A))
        phasecal_timeout_mclks = self.timeoutMicrosecondsToMclks(1000, macro_period_us)
        if (phasecal_timeout_mclks > 0xFF): phasecal_timeout_mclks = 0xFF
        writeReg(self.i2c_bus, self.i2c_adress, REG.PHASECAL_CONFIG__TIMEOUT_MACROP, phasecal_timeout_mclks)
        writeReg16Bit(self.i2c_bus, self.i2c_adress, REG.MM_CONFIG__TIMEOUT_MACROP_A, self.encodeTimeout(self.timeoutMicrosecondsToMclks(1, macro_period_us)))
        writeReg16Bit(self.i2c_bus, self.i2c_adress, REG.RANGE_CONFIG__TIMEOUT_MACROP_A, self.encodeTimeout(self.timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));
        macro_period_us = self.calcMacroPeriod(readReg(self.i2c_bus, self.i2c_adress, REG.RANGE_CONFIG__VCSEL_PERIOD_B))
        writeReg16Bit(self.i2c_bus, self.i2c_adress, REG.MM_CONFIG__TIMEOUT_MACROP_B, self.encodeTimeout(self.timeoutMicrosecondsToMclks(1, macro_period_us)))
        writeReg16Bit(self.i2c_bus, self.i2c_adress, REG.RANGE_CONFIG__TIMEOUT_MACROP_B, self.encodeTimeout(self.timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)))
        return True;

    # Get the measurement timing budget in microseconds, based on VL53L1_SetMeasurementTimingBudgetMicroSeconds()
    def getMeasurementTimingBudget(self):
        if DEBUG_PRINT: print('getMeasurementTimingBudget()')
        macro_period_us = self.calcMacroPeriod(readReg(self.i2c_bus, self.i2c_adress, REG.RANGE_CONFIG__VCSEL_PERIOD_A))
        range_config_timeout_us = self.timeoutMclksToMicroseconds(self.decodeTimeout(readReg16Bit(self.i2c_bus, self.i2c_adress, \
        REG.RANGE_CONFIG__TIMEOUT_MACROP_A)),macro_period_us)
        return  2 * range_config_timeout_us + self.timing_guard
    
    # set distance mode to Short, Medium, or Long
    def setDistanceMode(self, distance_mode_select):
        if DEBUG_PRINT: print('setDistanceMode()')
        budget_us = self.getMeasurementTimingBudget()
        if (self.distance_mode[distance_mode_select] == 'Short'):
            writeReg(self.i2c_bus, self.i2c_adress, REG.RANGE_CONFIG__VCSEL_PERIOD_A, 0x07)
            writeReg(self.i2c_bus, self.i2c_adress, REG.RANGE_CONFIG__VCSEL_PERIOD_B, 0x05)
            writeReg(self.i2c_bus, self.i2c_adress, REG.RANGE_CONFIG__VALID_PHASE_HIGH, 0x38)
            writeReg(self.i2c_bus, self.i2c_adress, REG.SD_CONFIG__WOI_SD0, 0x07)
            writeReg(self.i2c_bus, self.i2c_adress, REG.SD_CONFIG__WOI_SD1, 0x05)
            writeReg(self.i2c_bus, self.i2c_adress, REG.SD_CONFIG__INITIAL_PHASE_SD0, 6) # tuning parm default
            writeReg(self.i2c_bus, self.i2c_adress, REG.SD_CONFIG__INITIAL_PHASE_SD1, 6) # tuning parm default
        elif (self.distance_mode[distance_mode_select] == 'Medium'):
            writeReg(self.i2c_bus, self.i2c_adress, REG.RANGE_CONFIG__VCSEL_PERIOD_A, 0x0B)
            writeReg(self.i2c_bus, self.i2c_adress, REG.RANGE_CONFIG__VCSEL_PERIOD_B, 0x09)
            writeReg(self.i2c_bus, self.i2c_adress, REG.RANGE_CONFIG__VALID_PHASE_HIGH, 0x78)
            writeReg(self.i2c_bus, self.i2c_adress, REG.SD_CONFIG__WOI_SD0, 0x0B)
            writeReg(self.i2c_bus, self.i2c_adress, REG.SD_CONFIG__WOI_SD1, 0x09)
            writeReg(self.i2c_bus, self.i2c_adress, REG.SD_CONFIG__INITIAL_PHASE_SD0, 10) # tuning parm default
            writeReg(self.i2c_bus, self.i2c_adress, REG.SD_CONFIG__INITIAL_PHASE_SD1, 10) # tuning parm default
        elif (self.distance_mode[distance_mode_select] == 'Long'):
            writeReg(self.i2c_bus, self.i2c_adress, REG.RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F)
            writeReg(self.i2c_bus, self.i2c_adress, REG.RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D)
            writeReg(self.i2c_bus, self.i2c_adress, REG.RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8)
            writeReg(self.i2c_bus, self.i2c_adress, REG.SD_CONFIG__WOI_SD0, 0x0F)
            writeReg(self.i2c_bus, self.i2c_adress, REG.SD_CONFIG__WOI_SD1, 0x0D)
            writeReg(self.i2c_bus, self.i2c_adress, REG.SD_CONFIG__INITIAL_PHASE_SD0, 14) # tuning parm default
            writeReg(self.i2c_bus, self.i2c_adress, REG.SD_CONFIG__INITIAL_PHASE_SD1, 14) # tuning parm default
        else:
            return False
        self.setMeasurementTimingBudget(budget_us)
        return True
    
    # Initialize sensor using settings taken mostly from VL53L1_DataInit() and
    # VL53L1_StaticInit(). If io_2v8 (optional) is True or not given, the sensor is configured for 2V8 mode.
    def initSensor(self, io_2v8 = True):
        if DEBUG_PRINT: print('initSensor()')
        timeout_count = 0
        if DEBUG_PRINT: print('Sensor ID: '+str(readReg16Bit(self.i2c_bus, self.i2c_adress, REG.IDENTIFICATION__MODEL_ID)))
        #if (readReg16Bit(self.i2c_bus, self.i2c_adress, IDENTIFICATION__MODEL_ID) != 0xEACC): return False
        writeReg(self.i2c_bus, self.i2c_adress, REG.SOFT_RESET, 0x00)
        sleep(0.0001)
        writeReg(self.i2c_bus, self.i2c_adress, REG.SOFT_RESET, 0x01)
        while ((readReg(self.i2c_bus, self.i2c_adress, REG.FIRMWARE__SYSTEM_STATUS) & 0x01) == 0):
            sleep(0.01)
            timeout_count += 1
            if (timeout_count > self.timeout_ms/10): # Fehlermeldung bei ueberschreitung Timeout
                print('Timeout ocurred!') 
                return False
        if (io_2v8):
            writeReg(self.i2c_bus, self.i2c_adress, REG.PAD_I2C_HV__EXTSUP_CONFIG, readReg(self.i2c_bus, self.i2c_adress, REG.PAD_I2C_HV__EXTSUP_CONFIG) | 0x01)
        self.fast_osc_frequency = readReg16Bit(self.i2c_bus, self.i2c_adress, REG.OSC_MEASURED__FAST_OSC__FREQUENCY)
        self.osc_calibrate_val = readReg16Bit(self.i2c_bus, self.i2c_adress, REG.RESULT__OSC_CALIBRATE_VAL)
        writeReg16Bit(self.i2c_bus, self.i2c_adress, REG.DSS_CONFIG__TARGET_TOTAL_RATE_MCPS, self.target_rate) # should already be this value after reset
        writeReg(self.i2c_bus, self.i2c_adress, REG.GPIO__TIO_HV_STATUS, 0x02)
        writeReg(self.i2c_bus, self.i2c_adress, REG.SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS, 8) # tuning parm default
        writeReg(self.i2c_bus, self.i2c_adress, REG.SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS, 16) # tuning parm default
        writeReg(self.i2c_bus, self.i2c_adress, REG.ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM, 0x01)
        writeReg(self.i2c_bus, self.i2c_adress, REG.ALGO__RANGE_IGNORE_VALID_HEIGHT_MM, 0xFF)
        writeReg(self.i2c_bus, self.i2c_adress, REG.ALGO__RANGE_MIN_CLIP, 0) # tuning parm default
        writeReg(self.i2c_bus, self.i2c_adress, REG.ALGO__CONSISTENCY_CHECK__TOLERANCE, 2) # tuning parm default
        writeReg16Bit(self.i2c_bus, self.i2c_adress, REG.SYSTEM__THRESH_RATE_HIGH, 0x0000)
        writeReg16Bit(self.i2c_bus, self.i2c_adress, REG.SYSTEM__THRESH_RATE_LOW, 0x0000)
        writeReg(self.i2c_bus, self.i2c_adress, REG.DSS_CONFIG__APERTURE_ATTENUATION, 0x38)
        writeReg16Bit(self.i2c_bus, self.i2c_adress, REG.RANGE_CONFIG__SIGMA_THRESH, 360) # tuning parm default
        writeReg16Bit(self.i2c_bus, self.i2c_adress, REG.RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, 192) # tuning parm default
        writeReg(self.i2c_bus, self.i2c_adress, REG.SYSTEM__GROUPED_PARAMETER_HOLD_0, 0x01)
        writeReg(self.i2c_bus, self.i2c_adress, REG.SYSTEM__GROUPED_PARAMETER_HOLD_1, 0x01)
        writeReg(self.i2c_bus, self.i2c_adress, REG.SD_CONFIG__QUANTIFIER, 2) # tuning parm default
        writeReg(self.i2c_bus, self.i2c_adress, REG.SYSTEM__GROUPED_PARAMETER_HOLD, 0x00)
        writeReg(self.i2c_bus, self.i2c_adress, REG.SYSTEM__SEED_CONFIG, 1) # tuning parm default
        writeReg(self.i2c_bus, self.i2c_adress, REG.SYSTEM__SEQUENCE_CONFIG, 0x8B) # VHV, PHASECAL, DSS1, RANGE
        writeReg16Bit(self.i2c_bus, self.i2c_adress, REG.DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 200 << 8)
        writeReg(self.i2c_bus, self.i2c_adress, REG.DSS_CONFIG__ROI_MODE_CONTROL, 2) # REQUESTED_EFFFECTIVE_SPADS
        self.setDistanceMode(1)
        self.setMeasurementTimingBudget(50000)
        writeReg16Bit(self.i2c_bus, self.i2c_adress, REG.ALGO__PART_TO_PART_RANGE_OFFSET_MM,readReg16Bit(self.i2c_bus, self.i2c_adress, REG.MM_CONFIG__OUTER_OFFSET_MM) * 4)
        return True
    
    # check if sensor has new reading available, assumes interrupt is active low (GPIO_HV_MUX__CTRL bit 4 is 1)
    def dataReady(self):
        if DEBUG_PRINT: print('dataReady()')
        return (readReg(self.i2c_bus, self.i2c_adress, REG.GPIO__TIO_HV_STATUS) & 0x01) == 0
    
    # Start continuous ranging measurements, with the given inter-measurement
    # period in milliseconds determining how often the sensor takes a measurement.
    def startContinuous(self, period_ms):
        writeReg32Bit(self.i2c_bus, self.i2c_adress, REG.SYSTEM__INTERMEASUREMENT_PERIOD, period_ms * self.osc_calibrate_val)
        writeReg(self.i2c_bus, self.i2c_adress, REG.SYSTEM__INTERRUPT_CLEAR, 0x01) # sys_interrupt_clear_range
        writeReg(self.i2c_bus, self.i2c_adress, REG.SYSTEM__MODE_START, 0x40) # mode_range__timed
    
    # Stop continuous measurements, based on VL53L1_stop_range()
    def stopContinuous(self):
        if DEBUG_PRINT: print('stopContinuous()')
        writeReg(self.i2c_bus, self.i2c_adress, REG.SYSTEM__MODE_START, 0x80) # mode_range__abort
        self.calibrated = False
        if (self.saved_vhv_init != 0):
            writeReg(self.i2c_bus, self.i2c_adress, REG.VHV_CONFIG__INIT, self.saved_vhv_init)
        if (self.saved_vhv_timeout != 0):
            writeReg(self.i2c_bus, self.i2c_adress, REG.VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, self.saved_vhv_timeout)
        writeReg(self.i2c_bus, self.i2c_adress, REG.PHASECAL_CONFIG__OVERRIDE, 0x00)
    
    # Returns a range reading in millimeters when continuous mode is active
    # (readRangeSingleMillimeters() also calls this function after starting a single-shot range measurement)
    def sensorRead(self, blocking=True):
        if DEBUG_PRINT: print('sensorRead()')
        timeout_count = 0
        if (blocking):
            while not self.dataReady():
                sleep(0.01)
                timeout_count += 1
                if (timeout_count > self.timeout_ms/10): # Fehlermeldung bei ueberschreitung Timeout
                    print('Timeout ocurred!') 
                    self.ranging_data['range_status'] = None
                    self.ranging_data['range_mm'] = 0
                    self.ranging_data['sigma_mm'] = 0
                    self.ranging_data['peak_signal_count_rate_MCPS'] = 0
                    self.ranging_data['ambient_count_rate_MCPS'] = 0
        self.readResults()
        if not self.calibrated:
            self.setupManualCalibration()
            self.calibrated = True
        self.updateDSS()
        self.getRangingData()
        writeReg(self.i2c_bus, self.i2c_adress, REG.SYSTEM__INTERRUPT_CLEAR, 0x01) # sys_interrupt_clear_range
            
    # Starts a single-shot range measurement. If blocking is true (the default),
    # this function waits for the measurement to finish and returns the reading.
    # Otherwise, it returns 0 immediately.
    def sensorReadSingle(self, blocking=True):
        if DEBUG_PRINT: print('readSingle()')
        writeReg(self.i2c_bus, self.i2c_adress, REG.SYSTEM__INTERRUPT_CLEAR, 0x01) # sys_interrupt_clear_range
        writeReg(self.i2c_bus, self.i2c_adress, REG.SYSTEM__MODE_START, 0x10); # mode_range__single_shot
        if (blocking):
            return self.sensorRead(True)
        else:
            return 0
    
    # "Setup ranges after the first one in low power auto mode by turning off  FW calibration steps and programming static values"
    # based on VL53L1_low_power_auto_setup_manual_calibration()
    def setupManualCalibration(self):
        if DEBUG_PRINT: print('setupManualCalibration()')
        saved_vhv_init = readReg(self.i2c_bus, self.i2c_adress, REG.VHV_CONFIG__INIT)
        saved_vhv_timeout = readReg(self.i2c_bus, self.i2c_adress, REG.VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND)
        writeReg(self.i2c_bus, self.i2c_adress, REG.VHV_CONFIG__INIT, saved_vhv_init & 0x7F)
        writeReg(self.i2c_bus, self.i2c_adress, REG.VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,
        (saved_vhv_timeout & 0x03) + (3 << 2)) # tuning parm default (LOWPOWERAUTO_VHV_LOOP_BOUND_DEFAULT)
        writeReg(self.i2c_bus, self.i2c_adress, REG.PHASECAL_CONFIG__OVERRIDE, 0x01)
        writeReg(self.i2c_bus, self.i2c_adress, REG.CAL_CONFIG__VCSEL_START, readReg(self.i2c_bus, self.i2c_adress, REG.PHASECAL_RESULT__VCSEL_START))
    
    # read measurement results into buffer
    def readResults(self):
        if DEBUG_PRINT: print('readResults()')
        self.results['range_status'] = readReg(self.i2c_bus, self.i2c_adress, REG.RESULT__RANGE_STATUS)
        self.results['stream_count'] = readReg(self.i2c_bus, self.i2c_adress, REG.RESULT__STREAM_COUNT)
        self.results['dss_actual_effective_spads_sd0'] = readReg16Bit(self.i2c_bus, self.i2c_adress, REG.RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0)
        self.results['ambient_count_rate_mcps_sd0'] = readReg16Bit(self.i2c_bus, self.i2c_adress, REG.RESULT__AMBIENT_COUNT_RATE_MCPS_SD0)
        self.results['final_crosstalk_corrected_range_mm_sd0'] = readReg16Bit(self.i2c_bus, self.i2c_adress, REG.RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0)
        self.results['peak_signal_count_rate_crosstalk_corrected_mcps_sd0'] = readReg16Bit(self.i2c_bus, self.i2c_adress, REG.RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0)
        self.results['sigma_sd0'] = readReg16Bit(self.i2c_bus, self.i2c_adress, REG.RESULT__SIGMA_SD0)
    
    # perform Dynamic SPAD Selection calculation/update  based on VL53L1_low_power_auto_update_DSS()
    def updateDSS(self):
        if DEBUG_PRINT: print('updateDSS()')
        spadCount = self.results['dss_actual_effective_spads_sd0']
        if (spadCount != 0):
            totalRatePerSpad = self.results['peak_signal_count_rate_crosstalk_corrected_mcps_sd0'] + self.results['ambient_count_rate_mcps_sd0']
            if (totalRatePerSpad > 0xFFFF): totalRatePerSpad = 0xFFFF
            totalRatePerSpad = totalRatePerSpad/spadCount
            if (totalRatePerSpad != 0):
                requiredSpads = (self.target_rate * spadCount) / totalRatePerSpad
                if (requiredSpads > 0xFFFF): requiredSpads = 0xFFFF
                writeReg16Bit(self.i2c_bus, self.i2c_adress, REG.DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, requiredSpads)
                return
        writeReg16Bit(self.i2c_bus, self.i2c_adress, REG.DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 0x8000)
    
    # get range, status, rates from results buffer based on VL53L1_GetRangingMeasurementData()
    def getRangingData(self):
        if DEBUG_PRINT: print('getRangingData()')
        self.range_val = self.results['final_crosstalk_corrected_range_mm_sd0']
        self.ranging_data['range_mm'] = (self.range_val * 2011 + 0x0400) / 0x0800
        self.ranging_data['sigma_mm'] = self.results['sigma_sd0'] # ?lt. API-Doku hier Float>Int noetig
        self.ranging_data['range_status'] = self.results['range_status']
        self.ranging_data['peak_signal_count_rate_MCPS'] = countRateFixedToFloat\
        (self.results['peak_signal_count_rate_crosstalk_corrected_mcps_sd0'])
        self.ranging_data['ambient_count_rate_MCPS'] = countRateFixedToFloat(self.results['ambient_count_rate_mcps_sd0'])
    
    
    # Decode sequence step timeout in MCLKs from register value based on VL53L1_decode_timeout()
    def decodeTimeout(self, reg_val):
        if DEBUG_PRINT: print('decodeTimeout()')
        return ((reg_val & 0xFF) << (reg_val >> 8)) + 1
    
    # Encode sequence step timeout register value from timeout in MCLKs based on VL53L1_encode_timeout()
    def encodeTimeout(self, timeout_mclks):
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
    def timeoutMclksToMicroseconds(self, timeout_mclks, macro_period_us):
        if DEBUG_PRINT: print('timeoutMclksToMicroseconds()')
        return (timeout_mclks * macro_period_us + 0x0800) >> 12
    
    # Convert sequence step timeout from microseconds to macro periods with given
    # macro period in microseconds (12.12 format) based on VL53L1_calc_timeout_mclks()
    def timeoutMicrosecondsToMclks(self, timeout_us, macro_period_us):
        if DEBUG_PRINT: print('timeoutMicrosecondsToMclks()')
        return int(((timeout_us << 12) + (macro_period_us >> 1)) / macro_period_us)
    
    
    # Calculate macro period in microseconds (12.12 format) with given VCSEL period
    # assumes fast_osc_frequency has been read and stored based on VL53L1_calc_macro_period_us()
    def calcMacroPeriod(self, vcsel_period):
        if DEBUG_PRINT: print('calcMacroPeriod()')
        global fast_osc_frequency
        pll_period_us = (0x01 << 30) / self.fast_osc_frequency
        vcsel_period_pclks = (vcsel_period + 1) << 1
        macro_period_us = int(2304 * pll_period_us)
        macro_period_us >>= 6
        macro_period_us *= vcsel_period_pclks
        macro_period_us >>= 6
        return macro_period_us

    # Set the width and height of the region of interest
    # based on VL53L1X_SetROI() from STSW-IMG009 Ultra Lite Driver
    # ST user manual UM2555 explains ROI selection in detail, so we recommend
    # reading that document carefully.
    def setROISize(self, width=16, height=16):
        if width > 16: width = 16
        if height > 16: height = 16
        
        # Force ROI to be centered if width or height > 10, matching what the ULD API
        # does. (This can probably be overridden by calling setROICenter afterwards.)
        if (width > 10 or height > 10):
            writeReg(self.i2c_bus, self.i2c_adress, REG.ROI_CONFIG__USER_ROI_CENTRE_SPAD, 199)
        writeReg(self.i2c_bus, self.i2c_adress, REG.ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE, ((height - 1) << 4) | (width - 1))
    
    
    # Set the center SPAD of the region of interest (ROI)
    # based on VL53L1X_SetROICenter() from STSW-IMG009 Ultra Lite Driver
    #
    # ST user manual UM2555 explains ROI selection in detail, so we recommend
    # reading that document carefully. Here is a table of SPAD locations from
    # UM2555 (199 is the default/center):
    #
    # 128,136,144,152,160,168,176,184,  192,200,208,216,224,232,240,248
    # 129,137,145,153,161,169,177,185,  193,201,209,217,225,233,241,249
    # 130,138,146,154,162,170,178,186,  194,202,210,218,226,234,242,250
    # 131,139,147,155,163,171,179,187,  195,203,211,219,227,235,243,251
    # 132,140,148,156,164,172,180,188,  196,204,212,220,228,236,244,252
    # 133,141,149,157,165,173,181,189,  197,205,213,221,229,237,245,253
    # 134,142,150,158,166,174,182,190,  198,206,214,222,230,238,246,254
    # 135,143,151,159,167,175,183,191,  199,207,215,223,231,239,247,255
    #
    # 127,119,111,103, 95, 87, 79, 71,   63, 55, 47, 39, 31, 23, 15,  7
    # 126,118,110,102, 94, 86, 78, 70,   62, 54, 46, 38, 30, 22, 14,  6
    # 125,117,109,101, 93, 85, 77, 69,   61, 53, 45, 37, 29, 21, 13,  5
    # 124,116,108,100, 92, 84, 76, 68,   60, 52, 44, 36, 28, 20, 12,  4
    # 123,115,107, 99, 91, 83, 75, 67,   59, 51, 43, 35, 27, 19, 11,  3
    # 122,114,106, 98, 90, 82, 74, 66,   58, 50, 42, 34, 26, 18, 10,  2
    # 121,113,105, 97, 89, 81, 73, 65,   57, 49, 41, 33, 25, 17,  9,  1
    # 120,112,104, 96, 88, 80, 72, 64,   56, 48, 40, 32, 24, 16,  8,  0 <- Pin 1
    #
    # This table is oriented as if looking into the front of the sensor (or top of
    # the chip). SPAD 0 is closest to pin 1 of the VL53L1X, which is the corner
    # closest to the VDD pin on the Pololu VL53L1X carrier board:
    #
    #   +--------------+
    #   |             O| GPIO1
    #   |              |
    #   |             O|
    #   | 128    248   |
    #   |+----------+ O|
    #   ||+--+  +--+|  |
    #   |||  |  |  || O|
    #   ||+--+  +--+|  |
    #   |+----------+ O|
    #   | 120      0   |
    #   |             O|
    #   |              |
    #   |             O| VDD
    #   +--------------+
    #
    # However, note that the lens inside the VL53L1X inverts the image it sees
    # (like the way a camera works). So for example, to shift the sensor's FOV to
    # sense objects toward the upper left, you should pick a center SPAD in the
    # lower right.
    
    def setROICenter(self, spadNumber = 199):
        writeReg(self.i2c_bus, self.i2c_adress, REG.ROI_CONFIG__USER_ROI_CENTRE_SPAD, spadNumber)
