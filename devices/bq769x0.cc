/* Battery management system based on bq769x0 for ARM mbed
 * Copyright (c) 2015-2018 Martin Jäger (www.libre.solar)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <math.h>     // log for thermistor calculation
#include "bq769x0.h"

#include <string.h>
#include "mcu/timer.h"
#include <util/delay.h>
#include <stdint.h>
#include <stdlib.h>
#include <limits.h>
#include <stdint.h>
#include <avr/pgmspace.h>
#include <stdint.h>

using stream::Flags::PGM;

namespace devices {

const char *byte2char(int x) {
    static char b[9];
    b[0] = '\0';
    int z;
    for (z = 128; z > 0; z >>= 1) {
        strcat(b, ((x & z) == z) ? "1" : "0");
    }
    return b;
}

uint8_t _crc8_ccitt_update (uint8_t inCrc, uint8_t inData) {
    uint8_t data = inCrc ^ inData;
    for (uint8_t i = 0; i < 8; i++) {
        if ((data & 0x80) != 0) {
            data <<= 1;
            data ^= 0x07;
        } else {
            data <<= 1;
        }
    }
    return data;
}

bq769x0::bq769x0(bq769_conf &_conf, bq769_data &_data, bq769_stats &_stats):
    cout(mcu::Usart::get()),
    conf(_conf),
    data(_data),
    stats(_stats),
    mChargingEnabled(false),
    mDischargingEnabled(false)
{
    chargingDisabled_ = 0;
    dischargingDisabled_ = 0;
}

void bq769x0::begin() {
    memset(&data, 0, sizeof(data));
    data.alertInterruptFlag_ = true;
    // test communication
    while (true) {
        // should be set to 0x19 according to datasheet
        writeRegister(CC_CFG, 0x19);
        if (readRegister(CC_CFG) == 0x19) break;
        cout << PGM << PSTR("bq769x0 CFG error!\r\n");
        _delay_ms(125);
    }
    // initial settings for bq769x0
    writeRegister(SYS_CTRL1, 0b00011000);  // switch external thermistor and ADC on
    writeRegister(SYS_CTRL2, 0b01000000);  // switch CC_EN on
    // reset balance registers just in case
    writeRegister(CELLBAL1, 0x0);
    writeRegister(CELLBAL2, 0x0);
    writeRegister(CELLBAL3, 0x0);
    // get ADC offset and gain
    stats.adcOffset_ = (int8_t)readRegister(ADCOFFSET);
    stats.adcGain_ = 365 + (
        ((readRegister(ADCGAIN1) & 0b00001100) << 1) |
        ((readRegister(ADCGAIN2) & 0b11100000) >> 5)
    ); // uV/LSB
}


//----------------------------------------------------------------------------
// Fast function to check whether BMS has an error
// (returns 0 if everything is OK)

uint8_t bq769x0::checkStatus() {
    if (data.alertInterruptFlag_ || errorStatus_.regByte) {
        regSYS_STAT_t sys_stat;
        sys_stat.regByte = readRegister(SYS_STAT);
        // first check, if only a new CC reading is available
        if (sys_stat.bits.CC_READY == 1) {
            if(conf.BQ_dbg) cout << PGM << PSTR("bq769x0: CC ready\r\n");
            updateCurrent();  // automatically clears CC ready flag
        }
        // Serious error occured
        if (sys_stat.regByte & STAT_FLAGS) {
            if (!errorStatus_.bits.DEVICE_XREADY && sys_stat.bits.DEVICE_XREADY) { // XR error
                mChargingEnabled = mDischargingEnabled = false;
                chargingDisabled_ |= (1 << ERROR_XREADY);
                dischargingDisabled_ |= (1 << ERROR_XREADY);
                stats.errorCounter_[ERROR_XREADY]++;
                stats.errorTimestamps_[ERROR_XREADY] = mcu::Timer::millis();
                if(conf.BQ_dbg) cout << PGM << PSTR("bq769x0 ERROR: XREADY\r\n");
            }
            if (!errorStatus_.bits.OVRD_ALERT && sys_stat.bits.OVRD_ALERT) { // Alert error
                mChargingEnabled = mDischargingEnabled = false;
                chargingDisabled_ |= (1 << ERROR_ALERT);
                dischargingDisabled_ |= (1 << ERROR_ALERT);
                stats.errorCounter_[ERROR_ALERT]++;
                stats.errorTimestamps_[ERROR_ALERT] = mcu::Timer::millis();
                if(conf.BQ_dbg) cout << PGM << PSTR("bq769x0 ERROR: ALERT\r\n");
            }
            if (sys_stat.bits.UV) { // UV error
                mDischargingEnabled = false;
                dischargingDisabled_ |= (1 << ERROR_UVP);
                stats.errorCounter_[ERROR_UVP]++;
                stats.errorTimestamps_[ERROR_UVP] = mcu::Timer::millis();
                if(conf.BQ_dbg) cout << PGM << PSTR("bq769x0 ERROR: UVP\r\n");
            }
            if (sys_stat.bits.OV) { // OV error
                mChargingEnabled = false;
                chargingDisabled_ |= (1 << ERROR_OVP);
                stats.errorCounter_[ERROR_OVP]++;
                stats.errorTimestamps_[ERROR_OVP] = mcu::Timer::millis();
                if(conf.BQ_dbg) cout << PGM << PSTR("bq769x0 ERROR: OVP\r\n");

            }
            if (sys_stat.bits.SCD) { // SCD
                mDischargingEnabled = false;
                dischargingDisabled_ |= (1 << ERROR_SCD);
                stats.errorCounter_[ERROR_SCD]++;
                stats.errorTimestamps_[ERROR_SCD] = mcu::Timer::millis();
                if(conf.BQ_dbg) cout << PGM << PSTR("bq769x0 ERROR: SCD\r\n");
            }
            if (sys_stat.bits.OCD) { // OCD
                mDischargingEnabled = false;
                dischargingDisabled_ |= (1 << ERROR_OCD);
                stats.errorCounter_[ERROR_OCD]++;
                stats.errorTimestamps_[ERROR_OCD] = mcu::Timer::millis();

                if(conf.BQ_dbg) cout << PGM << PSTR("bq769x0 ERROR: OCD\r\n");
            }
            errorStatus_.regByte = sys_stat.regByte;
        } else { errorStatus_.regByte = 0;
        }
    }
    return errorStatus_.regByte;
}

//----------------------------------------------------------------------------
// tries to clear errors which have been found by checkStatus()

void bq769x0::clearErrors() {
    
    if (errorStatus_.bits.DEVICE_XREADY) {
        // datasheet recommendation: try to clear after waiting a few seconds
        if((uint32_t)(mcu::Timer::millis() - stats.errorTimestamps_[ERROR_XREADY]) > 3UL * 1000UL) {
            if(conf.BQ_dbg) cout << PGM << PSTR("Attempting to clear XREADY error\r\n");
            writeRegister(SYS_STAT, STAT_DEVICE_XREADY);
            enableCharging(1 << ERROR_XREADY);
            enableDischarging(1 << ERROR_XREADY);
            errorStatus_.bits.DEVICE_XREADY = 0;
        }
    }
    
    if(errorStatus_.bits.OVRD_ALERT) {
        if(conf.BQ_dbg) cout << PGM << PSTR("Attempt clear ALERT err!\r\n");
        writeRegister(SYS_STAT, STAT_OVRD_ALERT);
        enableCharging(1 << ERROR_ALERT);
        enableDischarging(1 << ERROR_ALERT);
        errorStatus_.bits.OVRD_ALERT = 0;
    }
    
    if(errorStatus_.bits.UV) {
        if(stats.cellVoltages_[stats.idCellMinVoltage_] > conf.Cell_UVP_mV) {
            if(conf.BQ_dbg) cout << PGM << PSTR("Attempt clear under voltage err!\r\n");
            writeRegister(SYS_STAT, STAT_UV);
            enableDischarging(1 << ERROR_UVP);
            errorStatus_.bits.UV = 0;
        }
    }
    
    if(errorStatus_.bits.OV) {
        if(stats.cellVoltages_[stats.idCellMaxVoltage_] < conf.Cell_OVP_mV) {
            if(conf.BQ_dbg) cout << PGM << PSTR("Attempt clear over voltage err!\r\n");
            writeRegister(SYS_STAT, STAT_OV);
            enableCharging(1 << ERROR_OVP);
            errorStatus_.bits.OV = 0;
        }
    }

    if(errorStatus_.bits.SCD) {
        if((uint32_t)(mcu::Timer::millis() - stats.errorTimestamps_[ERROR_SCD]) > 10UL * 1000UL) {
            if(conf.BQ_dbg) cout << PGM << PSTR("Attempt clear short circuit err!\r\n");
            writeRegister(SYS_STAT, STAT_SCD);
            enableDischarging(1 << ERROR_SCD);
            errorStatus_.bits.SCD = 0;
        }
    }
    
    if(errorStatus_.bits.OCD) {
        if((uint32_t)(mcu::Timer::millis() - stats.errorTimestamps_[ERROR_OCD]) > 10UL * 1000UL) {
            if(conf.BQ_dbg) cout << PGM << PSTR("Attempt clear overcurrent charge err!\r\n");
            writeRegister(SYS_STAT, STAT_OCD);
            enableDischarging(1 << ERROR_OCD);
            errorStatus_.bits.OCD = 0;
        }
    }
}

//----------------------------------------------------------------------------
// should be called at least once every 250 ms to get correct coulomb counting

uint8_t bq769x0::update() {
    
    uint8_t ret = checkStatus(); // does updateCurrent()
    //updateCurrent(); // will only read new current value if alert was triggered
    updateVoltages();
    updateTemperatures();
    updateBalancingSwitches();
    if(ret) { clearErrors(); }
    checkUser();
    return ret;
}

//----------------------------------------------------------------------------
// puts BMS IC into SHIP mode (i.e. switched off)
void bq769x0::shutdown() {
    writeRegister(SYS_CTRL1, 0x0);
    writeRegister(SYS_CTRL1, 0x1);
    writeRegister(SYS_CTRL1, 0x2);
}

//----------------------------------------------------------------------------
bool bq769x0::enableCharging(uint16_t flag) {
    chargingDisabled_ &= ~flag;
    if(!mChargingEnabled && !chargingDisabled_) {
        int sys_ctrl2;
        sys_ctrl2 = readRegister(SYS_CTRL2);
        writeRegister(SYS_CTRL2, sys_ctrl2 | 0b00000001);  // switch CHG on
        mChargingEnabled = true;

        if(conf.BQ_dbg) cout << PGM << PSTR("Enabling CHG FET\r\n");
        return true;
    } else { return mChargingEnabled; }
}

//----------------------------------------------------------------------------
void bq769x0::disableCharging(uint16_t flag) {
    chargingDisabled_ |= flag;
    if(mChargingEnabled && chargingDisabled_) {
        int sys_ctrl2;
        sys_ctrl2 = readRegister(SYS_CTRL2);
        writeRegister(SYS_CTRL2, sys_ctrl2 & ~0b00000001);  // switch CHG off
        mChargingEnabled = false;
        if(conf.BQ_dbg) cout << PGM << PSTR("Disabling CHG FET\r\n");
    }
}

//----------------------------------------------------------------------------
bool bq769x0::enableDischarging(uint16_t flag) {
    dischargingDisabled_ &= ~flag;
    
    if(!mDischargingEnabled && !dischargingDisabled_) {
        int sys_ctrl2;
        sys_ctrl2 = readRegister(SYS_CTRL2);
        writeRegister(SYS_CTRL2, sys_ctrl2 | 0b00000010);  // switch DSG on
        mDischargingEnabled = true;
        if(conf.BQ_dbg) cout << PGM << PSTR("Enabling DISCHG FET\r\n");
        return true;
    } else { return mDischargingEnabled; }
}

//----------------------------------------------------------------------------
void bq769x0::disableDischarging(uint16_t flag) {
    dischargingDisabled_ |= flag;

    if(mDischargingEnabled && dischargingDisabled_) {
        int sys_ctrl2;
        sys_ctrl2 = readRegister(SYS_CTRL2);
        writeRegister(SYS_CTRL2, sys_ctrl2 & ~0b00000010);  // switch DSG off
        mDischargingEnabled = false;
        if(conf.BQ_dbg) cout << PGM << PSTR("Disabling DISCHG FET\r\n");
    }
}

//----------------------------------------------------------------------------
// sets balancing registers if balancing is allowed
// (sufficient idle time + voltage)

void bq769x0::updateBalancingSwitches(void) {
    int32_t idleSeconds = (mcu::Timer::millis() - stats.idleTimestamp_) / 1000;
    uint8_t numberOfSections = (MAX_NUMBER_OF_CELLS + 4) / 5;

    // check for millis() overflow
    if (idleSeconds < 0) {
        stats.idleTimestamp_ = 0;
        idleSeconds = mcu::Timer::millis() / 1000;
    }

    // check if balancing allowed
    if (conf.BalancingEnable && errorStatus_.regByte == 0 &&
        ((conf.BalancingInCharge && data.charging_ == 2) || idleSeconds >= conf.BalancingIdleTimeMin_s) &&
        stats.cellVoltages_[stats.idCellMaxVoltage_] > conf.BalancingCellMin_mV &&
        (stats.cellVoltages_[stats.idCellMaxVoltage_] - stats.cellVoltages_[stats.idCellMinVoltage_]) > conf.BalancingCellMaxDifference_mV)
    {
        //Serial.println("Balancing enabled!");
        data.balancingStatus_ = 0;  // current status will be set in following loop

        //regCELLBAL_t cellbal;
        uint16_t balancingFlags;
        uint16_t balancingFlagsTarget;

        for (uint8_t section = 0; section < numberOfSections; section++) {
            // find cells which should be balanced and sort them by voltage descending
            uint8_t cellList[5];
            uint8_t cellCounter = 0;
            for (uint8_t i = 0; i < 5; i++) {
                if (stats.cellVoltages_[section*5 + i] < 500) continue;

                if ((stats.cellVoltages_[section*5 + i] - stats.cellVoltages_[stats.idCellMinVoltage_]) > conf.BalancingCellMaxDifference_mV) {
                    int j = cellCounter;
                    while (j > 0 && stats.cellVoltages_[section*5 + cellList[j - 1]] < stats.cellVoltages_[section*5 + i]) {
                        cellList[j] = cellList[j - 1];
                        j--;
                    }
                    cellList[j] = i;
                    cellCounter++;
                }
            }

            balancingFlags = 0;
            for (uint8_t i = 0; i < cellCounter; i++) {
                // try to enable balancing of current cell
                balancingFlagsTarget = balancingFlags | (1 << cellList[i]);
                // check if attempting to balance adjacent cells
                bool adjacentCellCollision =
                    ((balancingFlagsTarget << 1) & balancingFlags) ||
                    ((balancingFlags << 1) & balancingFlagsTarget);
                if (adjacentCellCollision == false) {
                    balancingFlags = balancingFlagsTarget;
                }
            }

            if(conf.BQ_dbg) {
                cout << PGM << PSTR("Setting CELLBAL ") << uint8_t(section+1);
                cout << PGM << PSTR(" register to: ") << byte2char(balancingFlags) << EOL;
            }
            
            data.balancingStatus_ |= balancingFlags << section*5;

            // set balancing register for this section
            writeRegister(CELLBAL1+section, balancingFlags);
        } // section loop
    } else if (data.balancingStatus_ > 0) {
        // clear all CELLBAL registers
        for (uint8_t section = 0; section < numberOfSections; section++) {
            if(conf.BQ_dbg) { cout << PGM << PSTR("Clearing Register CELLBAL ") << uint8_t(section+1) << EOL; }
            writeRegister(CELLBAL1+section, 0x0);
        }
        data.balancingStatus_ = 0;
    }
}

//----------------------------------------------------------------------------
void bq769x0::setOCV(uint16_t voltageVsSOC[NUM_OCV_POINTS]) { OCV_ = voltageVsSOC; }

//----------------------------------------------------------------------------
float bq769x0::getSOC(void) { return (float) coulombCounter_ / conf.Batt_CapaNom_mAsec * 100.0; }

//----------------------------------------------------------------------------
// SOC calculation based on average cell open circuit voltage

void bq769x0::resetSOC(int percent) {
    if (percent <= 100 && percent >= 0) {
        coulombCounter_ = (int32_t)(conf.Batt_CapaNom_mAsec * percent) / 100L;
    } else {  // reset based on OCV
        if(conf.BQ_dbg) {
            cout << PGM << PSTR("NumCells: ") << getNumberOfConnectedCells() << 
            PGM << PSTR(", voltage: ") << data.batVoltage_ << 'V';
        }
        uint16_t voltage = data.batVoltage_ / getNumberOfConnectedCells();
        coulombCounter_ = 0;  // initialize with totally depleted battery (0% SOC)
        for (int i = 0; i < NUM_OCV_POINTS; i++) {
            if (OCV_[i] <= voltage) {
                if (i == 0) { coulombCounter_ = conf.Batt_CapaNom_mAsec;  // 100% full 
                } else {
                    // interpolate between OCV[i] and OCV[i-1]
                    coulombCounter_ = (double) conf.Batt_CapaNom_mAsec / (NUM_OCV_POINTS - 1.0) *
                    (NUM_OCV_POINTS - 1.0 - i + ((float)voltage - OCV_[i])/(OCV_[i-1] - OCV_[i]));
                }
                return;
            }
        }
    }
}

//----------------------------------------------------------------------------

int16_t bq769x0::getADCOffset() { return stats.adcOffset_; }
int16_t bq769x0::getADCCellOffset(uint8_t cell) { return stats.adcOffset_ + conf.adcCellsOffset_[cell]; }

uint32_t bq769x0::setShortCircuitProtection(uint32_t current_mA, uint16_t delay_us) {
    conf.Cell_SCD_mA = current_mA;
    conf.Cell_SCD_us = delay_us;
    regPROTECT1_t protect1;
    protect1.bits.RSNS = PROTECT1_RSNS;
    protect1.bits.SCD_THRESH = 0;
    uint8_t temp = (current_mA * conf.RS_uOhm) / 1000000UL;
    for (uint8_t i = sizeof(SCD_threshold_setting)-1; i > 0; i--) {
        if (temp >= SCD_threshold_setting[i]) {
            protect1.bits.SCD_THRESH = i;
            break;
        }
    }
    protect1.bits.SCD_DELAY = 0;
    for (uint8_t i = sizeof(SCD_delay_setting)-1; i > 0; i--) {
        if (delay_us >= SCD_delay_setting[i]) {
            protect1.bits.SCD_DELAY = i;
            break;
        }
    }
    writeRegister(PROTECT1, protect1.regByte);
    // returns the actual current threshold value
    return ((uint32_t)SCD_threshold_setting[protect1.bits.SCD_THRESH] * 1000000UL) / conf.RS_uOhm;
}

//----------------------------------------------------------------------------

uint32_t bq769x0::setOvercurrentChargeProtection(uint32_t current_mA, uint16_t delay_ms) {
    conf.Cell_OCD_mA = current_mA;
    conf.Cell_OCD_ms = delay_ms;
    return current_mA;
}

//----------------------------------------------------------------------------

uint32_t bq769x0::setOvercurrentDischargeProtection(uint32_t current_mA, uint16_t delay_ms) {
    conf.Cell_ODP_mA = current_mA;
    conf.Cell_ODP_ms = delay_ms;
    regPROTECT2_t protect2;
    protect2.bits.OCD_THRESH = 0;
    uint8_t temp = (current_mA * conf.RS_uOhm) / 1000000UL;
    for (uint8_t i = sizeof(OCD_threshold_setting)-1; i > 0; i--) {
        if (temp >= OCD_threshold_setting[i]) {
            protect2.bits.OCD_THRESH = i;
            break;
        }
    }
    protect2.bits.OCD_DELAY = 0;
    for (uint8_t i = sizeof(OCD_delay_setting)-1; i > 0; i--) {
        if (delay_ms >= OCD_delay_setting[i]) {
            protect2.bits.OCD_DELAY = i;
            break;
        }
    }
    writeRegister(PROTECT2, protect2.regByte);
    // returns the actual current threshold value
    return ((uint32_t)OCD_threshold_setting[protect2.bits.OCD_THRESH] * 1000000UL) / conf.RS_uOhm;
}


//----------------------------------------------------------------------------

uint16_t bq769x0::setCellUndervoltageProtection(uint16_t voltage_mV, uint16_t delay_s) {
    conf.Cell_UVP_mV = voltage_mV;
    conf.Cell_UVP_sec = delay_s;
    regPROTECT3_t protect3;
    protect3.regByte = readRegister(PROTECT3);
    uint16_t uv_trip = ((((voltage_mV - stats.adcOffset_) * 1000UL) / stats.adcGain_) >> 4) & 0x00FF;
    uv_trip += 1;   // always round up for lower cell voltage
    writeRegister(UV_TRIP, uv_trip);
    protect3.bits.UV_DELAY = 0;
    for (uint8_t i = sizeof(UV_delay_setting)-1; i > 0; i--) {
        if (delay_s >= UV_delay_setting[i]) {
            protect3.bits.UV_DELAY = i;
            break;
        }
    }
    writeRegister(PROTECT3, protect3.regByte);
    // returns the actual voltage threshold value
    return ((1UL << 12UL | uv_trip << 4) * stats.adcGain_) / 1000UL + stats.adcOffset_;
}

//----------------------------------------------------------------------------

uint16_t bq769x0::setCellOvervoltageProtection(uint16_t voltage_mV, uint16_t delay_s) {
    conf.Cell_OVP_mV = voltage_mV;
    conf.Cell_OVP_sec = delay_s;
    regPROTECT3_t protect3;
    protect3.regByte = readRegister(PROTECT3);
    uint16_t ov_trip = ((((voltage_mV - stats.adcOffset_) * 1000UL) / stats.adcGain_) >> 4) & 0x00FF;
    writeRegister(OV_TRIP, ov_trip);
    protect3.bits.OV_DELAY = 0;
    for (uint8_t i = sizeof(OV_delay_setting)-1; i > 0; i--) {
        if (delay_s >= OV_delay_setting[i]) {
            protect3.bits.OV_DELAY = i;
            break;
        }
    }
    writeRegister(PROTECT3, protect3.regByte);
    // returns the actual voltage threshold value
    return ((uint32_t)(1 << 13 | ov_trip << 4) * stats.adcGain_) / 1000UL + stats.adcOffset_;
}

//----------------------------------------------------------------------------
uint16_t bq769x0::getMaxCellVoltage() { return stats.cellVoltages_[stats.idCellMaxVoltage_]; }

//----------------------------------------------------------------------------
uint16_t bq769x0::getMinCellVoltage() { return stats.cellVoltages_[stats.idCellMinVoltage_]; }

//----------------------------------------------------------------------------
uint16_t bq769x0::getAvgCellVoltage() { return data.batVoltage_ / getNumberOfConnectedCells(); }

//----------------------------------------------------------------------------
uint16_t bq769x0::getCellVoltage(uint8_t idCell, bool raw) {
    uint8_t i = stats.cellIdMap_[idCell];
    if (raw) return data.cellVoltages_raw_[i];
    return stats.cellVoltages_[i];
}

//----------------------------------------------------------------------------
uint16_t bq769x0::getCellVoltage_(uint8_t i, bool raw) {
    if (raw) return data.cellVoltages_raw_[i];
    return stats.cellVoltages_[i];
}

//----------------------------------------------------------------------------
uint8_t bq769x0::getNumberOfCells(void) { return MAX_NUMBER_OF_CELLS; }

//----------------------------------------------------------------------------

uint8_t bq769x0::getNumberOfConnectedCells(void) { return data.connectedCells_; }

//----------------------------------------------------------------------------

float bq769x0::getTemperatureDegC(uint8_t channel) {
    if (channel <= 2) {
        return (float)stats.temperatures_[channel] / 10.0;
    } else { return -273.15; }  // Error: Return absolute minimum temperature
}

//----------------------------------------------------------------------------

float bq769x0::getTemperatureDegF(uint8_t channel) { return getTemperatureDegC(channel) * 1.8 + 32; }

//----------------------------------------------------------------------------

int16_t bq769x0::getLowestTemperature() {
    int16_t minTemp = INT16_MAX;
    for(uint8_t i = 0; i < MAX_NUMBER_OF_THERMISTORS; i++) {
        if(conf.RT_bits & (1 << i) && stats.temperatures_[i] < minTemp) minTemp = stats.temperatures_[i];
    }
    return minTemp;
}

int16_t bq769x0::getHighestTemperature() {
    int16_t maxTemp = INT16_MIN;
    for(uint8_t i = 0; i < MAX_NUMBER_OF_THERMISTORS; i++) {
        if(conf.RT_bits & (1 << i) && stats.temperatures_[i] > maxTemp) maxTemp = stats.temperatures_[i];
    }
    return maxTemp;
}

//----------------------------------------------------------------------------
int16_t updateTemperatures_calc(const uint16_t val, const uint16_t beta) {
    // calculate R_thermistor according to bq769x0 datasheet
    uint16_t vtsx = val * 0.382; // mV
    uint32_t rts = 10000.0 * vtsx / (3300.0 - vtsx); // Ohm
    // Temperature calculation using Beta equation
    // - According to bq769x0 datasheet, only 10k thermistors should be used
    // - 25°C reference temperature for Beta equation assumed
    float tmp = 1.0/(1.0/(273.15+25) + 1.0/beta * log(rts/10000.0)); // K
    return (tmp - 273.15) * 10.0;
}

void bq769x0::updateTemperatures() {
    stats.temperatures_[0] = updateTemperatures_calc(readDoubleRegister(TS1_HI_BYTE), conf.RT_Beta[0]);
#ifdef IC_BQ76930
    stats.temperatures_[1] = updateTemperatures_calc(readDoubleRegister(TS2_HI_BYTE), conf.RT_Beta[1]);
#endif
#ifdef IC_BQ76940
    stats.temperatures_[1] = updateTemperatures_calc(readDoubleRegister(TS2_HI_BYTE), conf.RT_Beta[1]);
    stats.temperatures_[2] = updateTemperatures_calc(readDoubleRegister(TS3_HI_BYTE), conf.RT_Beta[1]);
#endif
}


//----------------------------------------------------------------------------

void bq769x0::updateCurrent() {
    regSYS_STAT_t sys_stat;
    sys_stat.regByte = readRegister(SYS_STAT);
    // check if new current reading available
    if (sys_stat.bits.CC_READY == 1) {
        //Serial.println("reading CC register...");
        data.batCurrent_ = (int16_t)readDoubleRegister(CC_HI_BYTE);
        data.batCurrent_ = ((int32_t)data.batCurrent_ * 8440L) / (int32_t)conf.RS_uOhm;  // mA

        // is read every 250 ms
        coulombCounter_ += data.batCurrent_ / 4;

        if (coulombCounter_ > conf.Batt_CapaNom_mAsec) {
            coulombCounter_ = conf.Batt_CapaNom_mAsec;
        }
        if (coulombCounter_ < 0) {
            coulombCounter_ = 0;
        }

        if (data.batCurrent_ < 0) {
            coulombCounter2_ += -data.batCurrent_ / 4;
            if (coulombCounter2_ > conf.Batt_CapaNom_mAsec) {
                stats.batCycles_++;
                coulombCounter2_ = 0;
            }
        }

        if (data.batCurrent_ > (int32_t)conf.CurrentThresholdIdle_mA) {
            if (!data.charging_) {
                data.charging_ = 1;
                stats.chargeTimestamp_ = mcu::Timer::millis();
            }
            else if (data.charging_ == 1 && (uint32_t)(mcu::Timer::millis() - stats.chargeTimestamp_) > 60UL * 1000UL) {
                data.charging_ = 2;
                stats.chargedTimes_++;
            }
        }
        else if (data.charging_ != 2 || data.batCurrent_ < 10)
            data.charging_ = 0;

        // reset idleTimestamp
        if (abs(data.batCurrent_) > conf.CurrentThresholdIdle_mA) {
            if(data.batCurrent_ < 0 || !(conf.BalancingInCharge && data.charging_ == 2))
                stats.idleTimestamp_ = mcu::Timer::millis();
        }

        // no error occured which caused alert
        if (!(sys_stat.regByte & 0b00111111)) {
            data.alertInterruptFlag_ = false;
        }
        writeRegister(SYS_STAT, 0b10000000);  // Clear CC ready flag
    }
}

//----------------------------------------------------------------------------
// reads all cell voltages to array cellVoltages[NUM_CELLS] and updates batVoltage

void bq769x0::updateVoltages() {
    mcu::I2CMaster Wire;
    uint16_t adcVal = 0;
    uint8_t idCell = 0;
    // read cell voltages
    i2buf[0] = VC1_HI_BYTE;
    Wire.write(BQ769X0_I2C_ADDR, i2buf, 1);
    stats.idCellMaxVoltage_ = 0;
    stats.idCellMinVoltage_ = 0;
    for (int i = 0; i < MAX_NUMBER_OF_CELLS; i++) {
#ifdef BQ769X0_CRC_ENABLED
        Wire.read(BQ769X0_I2C_ADDR, i2buf, 4);
        uint8_t crc;
        uint8_t data_b = i2buf[0];
        adcVal = (data_b & 0b00111111) << 8;
        // CRC of first bytes includes slave address (including R/W bit) and data
        crc = _crc8_ccitt_update(0, (BQ769X0_I2C_ADDR << 1) | 1);
        crc = _crc8_ccitt_update(crc, data_b);
        if (crc != i2buf[1]) return; // don't save corrupted value
        data_b = i2buf[2];
        adcVal |= data_b;
        // CRC of subsequent bytes contain only data
        crc = _crc8_ccitt_update(0, data_b);
        if (crc != i2buf[3]) return; // don't save corrupted value
#else
        Wire.read(BQ769X0_I2C_ADDR, i2buf, 2);
        // reply
        adcVal = (i2buf[0] & 0b00111111) << 8 | i2buf[1];
#endif
        data.cellVoltages_raw_[i] = adcVal;
        stats.cellVoltages_[i] = ((uint32_t)adcVal * stats.adcGain_) / 1000 + getADCCellOffset(i);
        if (stats.cellVoltages_[i] < 500) { continue; }
        stats.cellIdMap_[idCell] = i;
        if (stats.cellVoltages_[i] > stats.cellVoltages_[stats.idCellMaxVoltage_]) { stats.idCellMaxVoltage_ = i; }
        if (stats.cellVoltages_[i] < stats.cellVoltages_[stats.idCellMinVoltage_]) { stats.idCellMinVoltage_ = i; }
        idCell++;
    }
    data.connectedCells_ = idCell;
    // read battery pack voltage
    data.batVoltage_raw_ = readDoubleRegister(BAT_HI_BYTE);
    data.batVoltage_ = ((uint32_t)4.0 * stats.adcGain_ * data.batVoltage_raw_) / 1000.0 + data.connectedCells_ * getADCOffset(); // TODO common offset!
    if(data.batVoltage_ >= data.connectedCells_ * conf.Cell_CapaFull_mV) {
        if(fullVoltageCount_ == 240) { // 60s * 4(250ms)
            resetSOC(100);
        }
        if(fullVoltageCount_ < 255)
            fullVoltageCount_++;
    } else {
        fullVoltageCount_ = 0;
    }
}

//----------------------------------------------------------------------------

void bq769x0::writeRegister(uint8_t address, uint8_t data) {
    mcu::I2CMaster Wire;
    i2buf[0] = address;
    i2buf[1] = data;
#ifdef BQ769X0_CRC_ENABLED
    // CRC is calculated over the slave address (including R/W bit), register address, and data.
    i2buf[2] = _crc8_ccitt_update(0, (BQ769X0_I2C_ADDR << 1) | 0);
    i2buf[2] = _crc8_ccitt_update(i2buf[2], address);
    i2buf[2] = _crc8_ccitt_update(i2buf[2], data);
    Wire.write(BQ769X0_I2C_ADDR, i2buf, 3);
#else
    Wire.write(BQ769X0_I2C_ADDR, i2buf, 2);
#endif
    }


//----------------------------------------------------------------------------

uint8_t bq769x0::readRegister(uint8_t address) {
    mcu::I2CMaster Wire;
    i2buf[0] = address;
    Wire.write(BQ769X0_I2C_ADDR, i2buf, 1);
    uint8_t data;
#ifdef BQ769X0_CRC_ENABLED
    uint8_t wantcrc;
    uint8_t gotcrc;
    do {
        Wire.read(BQ769X0_I2C_ADDR, i2buf, 2);
        data   = i2buf[0];
        gotcrc = i2buf[1];
        // CRC is calculated over the slave address (including R/W bit) and data.
        wantcrc = _crc8_ccitt_update(0, (BQ769X0_I2C_ADDR << 1) | 1);
        wantcrc = _crc8_ccitt_update(wantcrc, data);
    } while (gotcrc != wantcrc);
#else
    Wire.read(BQ769X0_I2C_ADDR, i2buf, 1);
    data = i2buf[0];
#endif
    return data;
}

//----------------------------------------------------------------------------

uint16_t bq769x0::readDoubleRegister(uint8_t address) {
    mcu::I2CMaster Wire;
    i2buf[0] = address;
    Wire.write(BQ769X0_I2C_ADDR, i2buf, 1);
    uint16_t result;
#ifdef BQ769X0_CRC_ENABLED
    while(true) {
        Wire.read(BQ769X0_I2C_ADDR, i2buf, 4);
        uint8_t crc;
        uint8_t data = i2buf[0];
        result = (uint16_t)data << 8;
        // CRC of first bytes includes slave address (including R/W bit) and data
        crc = _crc8_ccitt_update(0, (BQ769X0_I2C_ADDR << 1) | 1);
        crc = _crc8_ccitt_update(crc, data);
        if (crc != i2buf[1]) continue;
        data = i2buf[2];
        result |= data;
        // CRC of subsequent bytes contain only data
        crc = _crc8_ccitt_update(0, data);
        if (crc != i2buf[3]) continue;
        break;
    }
#else
    Wire.read(BQ769X0_I2C_ADDR, i2buf, 2);
    result = ((uint16_t)i2buf[0] << 8) | i2buf[1];
#endif
    return result;
}

//----------------------------------------------------------------------------
// Check custom error conditions like over/under temperature, over charge current
void bq769x0::checkUser() {
    // charge temperature limits
    if(getLowestTemperature() < conf.Cell_TempCharge_min || getHighestTemperature() > conf.Cell_TempCharge_max) {
        if(!(chargingDisabled_ & (1 << ERROR_USER_CHG_TEMP))) {
            disableCharging(1 << ERROR_USER_CHG_TEMP);
            stats.errorCounter_[ERROR_USER_CHG_TEMP]++;
            stats.errorTimestamps_[ERROR_USER_CHG_TEMP] = mcu::Timer::millis();
        }
    } else if(chargingDisabled_ & (1 << ERROR_USER_CHG_TEMP)) {
        enableCharging(1 << ERROR_USER_CHG_TEMP);
    }
    // discharge temperature limits
    if(getLowestTemperature() < conf.Cell_TempDischarge_min || getHighestTemperature() > conf.Cell_TempDischarge_max) {
        if(!(dischargingDisabled_ & (1 << ERROR_USER_DISCHG_TEMP))) {
            disableDischarging(1 << ERROR_USER_DISCHG_TEMP);
            stats.errorCounter_[ERROR_USER_DISCHG_TEMP]++;
            stats.errorTimestamps_[ERROR_USER_DISCHG_TEMP] = mcu::Timer::millis();
        }
    } else if(dischargingDisabled_ & (1 << ERROR_USER_DISCHG_TEMP)) {
        enableDischarging(1 << ERROR_USER_DISCHG_TEMP);
    }

    // charge current limit
    // charge current can also come through discharge FET that we can't turn off (regen on P-)
    // that's why this looks a bit funky
    if(data.batCurrent_ > conf.Cell_OCD_mA) {
        user_CHGOCD_ReleaseTimestamp_ = 0;
        if(mChargingEnabled && !(chargingDisabled_ & (1 << ERROR_USER_CHG_OCD))) {
            if(!user_CHGOCD_TriggerTimestamp_)
                user_CHGOCD_TriggerTimestamp_ = mcu::Timer::millis();
            if((mcu::Timer::millis() - user_CHGOCD_TriggerTimestamp_) > conf.Cell_OCD_ms || data.user_CHGOCD_ReleasedNow_) {
                disableCharging(1 << ERROR_USER_CHG_OCD);
                stats.errorCounter_[ERROR_USER_CHG_OCD]++;
                stats.errorTimestamps_[ERROR_USER_CHG_OCD] = mcu::Timer::millis();
            }
        }
    } else {
        user_CHGOCD_TriggerTimestamp_ = 0;
        data.user_CHGOCD_ReleasedNow_ = false;
        if(chargingDisabled_ & (1 << ERROR_USER_CHG_OCD)) {
            if(!user_CHGOCD_ReleaseTimestamp_)
                user_CHGOCD_ReleaseTimestamp_ = mcu::Timer::millis();
            if((uint32_t)(mcu::Timer::millis() - user_CHGOCD_ReleaseTimestamp_) > 10UL * 1000UL) {
                enableCharging(1 << ERROR_USER_CHG_OCD);
                user_CHGOCD_ReleaseTimestamp_ = 0;
                data.user_CHGOCD_ReleasedNow_ = true;
            }
        }
    }
}

void bq769x0::printRegisters() {
    cout 
        << PGM << PSTR("\r\n   ADCGAIN: ") << stats.adcGain_
        << PGM << PSTR("\r\n ADCOFFSET: ") << stats.adcOffset_
        << PGM << PSTR("\r\n   CHG DIS: ") << chargingDisabled_
        << PGM << PSTR("\r\nDISCHG DIS: ") << dischargingDisabled_ << EOL
        << PGM << PSTR("\r\n0x00  SYS_STAT: ") << byte2char(readRegister(SYS_STAT))
        << PGM << PSTR("\r\n0x01  CELLBAL1: ") << byte2char(readRegister(CELLBAL1))
        << PGM << PSTR("\r\n0x04 SYS_CTRL1: ") << byte2char(readRegister(SYS_CTRL1))
        << PGM << PSTR("\r\n0x05 SYS_CTRL2: ") << byte2char(readRegister(SYS_CTRL2))
        << PGM << PSTR("\r\n0x06  PROTECT1: ") << byte2char(readRegister(PROTECT1))
        << PGM << PSTR("\r\n0x07  PROTECT2: ") << byte2char(readRegister(PROTECT2))
        << PGM << PSTR("\r\n0x08  PROTECT3: ") << byte2char(readRegister(PROTECT3))
        << PGM << PSTR("\r\n0x09   OV_TRIP: ") << byte2char(readRegister(OV_TRIP))
        << PGM << PSTR("\r\n0x0A   UV_TRIP: ") << byte2char(readRegister(UV_TRIP))
        << PGM << PSTR("\r\n0x0B    CC_CFG: ") << byte2char(readRegister(CC_CFG))
        << PGM << PSTR("\r\n0x32  CC_HI_LO: ") << readDoubleRegister(CC_HI_BYTE)
        << PGM << PSTR("\r\n0x2A BAT_HI_LO: ") << readDoubleRegister(BAT_HI_BYTE)
        << EOL;
}

}
