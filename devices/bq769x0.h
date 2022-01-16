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

#pragma once

#include "bq769x0_registers.h"
#include "mcu/i2c_master.h"
#include "stream/uartstream.h"

//#define IC_BQ76920
//#define IC_BQ76930
#define IC_BQ76940

#ifdef IC_BQ76920
#define MAX_NUMBER_OF_CELLS         5
#define MAX_NUMBER_OF_THERMISTORS   1
#define BQ769X0_I2C_ADDR            0x08
#define BQ769X0_CRC_ENABLED
#define BQ769X0_THERMISTORS         0b001
#endif

#ifdef IC_BQ76930
#define MAX_NUMBER_OF_CELLS         10
#define MAX_NUMBER_OF_THERMISTORS   2
#define BQ769X0_I2C_ADDR            0x08
#define BQ769X0_CRC_ENABLED
#define BQ769X0_THERMISTORS         0b011
#endif

#ifdef IC_BQ76940
#define MAX_NUMBER_OF_CELLS         15
#define MAX_NUMBER_OF_THERMISTORS   3
#define BQ769X0_I2C_ADDR            0x08
#define BQ769X0_CRC_ENABLED
#define BQ769X0_THERMISTORS         0b111
#endif

#define NUM_OCV_POINTS 21

namespace devices {

const char *byte2char(int x);
uint8_t _crc8_ccitt_update (uint8_t inCrc, uint8_t inData); 

enum BQ769xERR {
    ERROR_XREADY = 0,
    ERROR_ALERT = 1,
    ERROR_UVP = 2,
    ERROR_OVP = 3,
    ERROR_SCD = 4,
    ERROR_OCD = 5,
    ERROR_USER_SWITCH = 6,
    ERROR_USER_DISCHG_TEMP = 7,
    ERROR_USER_CHG_TEMP = 8,
    ERROR_USER_CHG_OCD = 9,
    NUM_ERRORS
};

typedef struct __attribute__((packed)) {
    bool        BQ_dbg;                 // false
    bool        Allow_Charging;         // false
    bool        Allow_Discharging;      // false
    int32_t     Batt_CapaNom_mAsec;     // *3600 mAs, nominal capacity of battery pack, max. 580 Ah possible @ 3.7V
    uint16_t    Cell_CapaNom_mV;        // 3600 mV, nominal voltage of single cell in battery pack
    uint16_t    Cell_CapaFull_mV;       // 4200 mV, full voltage of single cell in battery pack
                                        // PROTECT1 Cell short circuit protection
    uint32_t    Cell_SCD_mA;            // Current limits (mA)
    uint16_t    Cell_SCD_us;            // Current limits us
                                        // PROTECT2 Cell overcurrent discharge protection
    uint32_t    Cell_ODP_mA;            // Current limits (mA)
    uint16_t    Cell_ODP_ms;            // Current limits Ms
                                        // checkUser Cell overcurrent charge protection
    uint32_t    Cell_OCD_mA;            // 5500 Current limits (mA)
    uint16_t    Cell_OCD_ms;            // 8
                                        // PROTECT3 Cell voltage protection limits
    uint16_t    Cell_OVP_mV;            // Cell voltage limits (4200 mV)
    uint16_t    Cell_OVP_sec;           // 2 s
    uint16_t    Cell_UVP_mV;            // Cell voltage limits (4200 mV)
    uint16_t    Cell_UVP_sec;           // 2 s
    int16_t     Cell_TempCharge_min;    // 000
    int16_t     Cell_TempCharge_max;    // 500 Temperature limits (C/10)
    int16_t     Cell_TempDischarge_min; // -200 Temperature limits (C/10)
    int16_t     Cell_TempDischarge_max; // 650 Temperature limits (C/10)
    uint8_t     RT_bits;                // 0-3
    uint32_t    RS_uOhm;                // 1000 1mOhm
    bool        BalancingInCharge;      // false
    bool        BalancingEnable;        // false
    uint16_t    BalancingCellMin_mV;    // Cell voltage (3600 mV)
    uint16_t    BalancingIdleTimeMin_s; // 1800
    uint32_t    CurrentThresholdIdle_mA;// 30 mA
    uint8_t     BalancingCellMaxDifference_mV;          // 20 mV
    int16_t     adcCellsOffset_[MAX_NUMBER_OF_CELLS];   // 0 mV
    uint16_t    RT_Beta[MAX_NUMBER_OF_THERMISTORS];     // 3435 typical value for Semitec 103AT-5 thermistor: 3435
    uint32_t    ts;
    uint8_t     crc8;
} bq769_conf;

typedef struct __attribute__((packed)) {
    bool        alertInterruptFlag_;        // true, indicates if a new current reading or an error is available from BMS IC
    bool        user_CHGOCD_ReleasedNow_;   // false
    uint8_t     charging_;
    uint8_t     connectedCells_;        // 0, actual number of cells connected
    uint32_t    batVoltage_;            // 0, mV
    uint16_t    batVoltage_raw_;        // 0, adc val
    int32_t     batCurrent_;            // 0, mA
    int16_t     batCurrent_raw_;        // adc val
    uint32_t    balancingStatus_;       // 0 holds on/off status of balancing switches
    uint16_t    cellVoltages_raw_[MAX_NUMBER_OF_CELLS];     //null, adc val
} bq769_data;

typedef struct __attribute__((packed)) {
    uint16_t    adcGain_;               // 0 uV/LSB
    int8_t      adcOffset_;             // 0 mV
    uint16_t    batCycles_;
    uint16_t    chargedTimes_;
    uint8_t     idCellMaxVoltage_;
    uint8_t     idCellMinVoltage_;
    uint32_t    idleTimestamp_;
    uint32_t    chargeTimestamp_;
    uint8_t     errorCounter_[NUM_ERRORS];
    uint8_t     cellIdMap_[MAX_NUMBER_OF_CELLS];            // null, logical cell id -> physical cell id
    uint16_t    cellVoltages_[MAX_NUMBER_OF_CELLS];         //null, mV
    int16_t     temperatures_[MAX_NUMBER_OF_THERMISTORS];   // null, C/10
    uint32_t    errorTimestamps_[NUM_ERRORS];               // null
    uint32_t    ts;
    uint8_t     crc8;
} bq769_stats;









class bq769x0 {
    stream::UartStream  cout;
    bq769_conf          &conf;
    bq769_data          &data;
    bq769_stats         &stats;
    uint8_t             i2buf[4];
public:
    bq769x0(bq769_conf &_conf, bq769_data &_data, bq769_stats &_stats);
    void begin();
    uint8_t checkStatus();  // returns 0 if everything is OK
    void checkUser();
    void clearErrors();
    uint8_t update(void);  // returns checkStatus retval
    void shutdown(void);
    // charging control
    bool enableCharging(uint16_t flag=(1 << ERROR_USER_SWITCH));
    void disableCharging(uint16_t flag=(1 << ERROR_USER_SWITCH));
    bool enableDischarging(uint16_t flag=(1 << ERROR_USER_SWITCH));
    void disableDischarging(uint16_t flag=(1 << ERROR_USER_SWITCH));
    void resetSOC(int percent = -1); // 0-100 %, -1 for automatic reset based on OCV
    void setOCV(uint16_t voltageVsSOC[NUM_OCV_POINTS]);
    int16_t getADCOffset();
    int16_t getADCCellOffset(uint8_t cell);
    uint8_t getNumberOfCells(void);
    uint8_t getNumberOfConnectedCells(void);
    uint32_t setShortCircuitProtection(uint32_t current_mA, uint16_t delay_us = 70);
    uint32_t setOvercurrentChargeProtection(uint32_t current_mA, uint16_t delay_ms);
    uint32_t setOvercurrentDischargeProtection(uint32_t current_mA, uint16_t delay_ms = 8);
    uint16_t setCellUndervoltageProtection(uint16_t voltage_mV, uint16_t delay_s = 1);
    uint16_t setCellOvervoltageProtection(uint16_t voltage_mV, uint16_t delay_s = 1);
    uint16_t getCellVoltage(uint8_t idCell, bool raw = false); // logical ids -> without gaps
    uint16_t getCellVoltage_(uint8_t i, bool raw = false); // physical ids -> possible gaps
    uint16_t getMinCellVoltage(void);
    uint16_t getMaxCellVoltage(void);
    uint16_t getAvgCellVoltage(void);
    float getTemperatureDegC(uint8_t channel = 1);
    float getTemperatureDegF(uint8_t channel = 1);
    int16_t getLowestTemperature(); // °C/10
    int16_t getHighestTemperature(); // °C/10
    float getSOC(void);
    void printRegisters(void);
private:
    uint16_t    chargingDisabled_;
    uint16_t    dischargingDisabled_;
    bool mChargingEnabled;
    bool mDischargingEnabled;
    uint16_t *OCV_; // Open Circuit Voltage of cell for SOC 100%, 95%, ..., 5%, 0%
    uint8_t fullVoltageCount_;
    uint32_t user_CHGOCD_TriggerTimestamp_;
    uint32_t user_CHGOCD_ReleaseTimestamp_;
    int32_t coulombCounter_; // mAs (= milli Coulombs) for current integration
    int32_t coulombCounter2_; // mAs (= milli Coulombs) for tracking battery cycles
    regSYS_STAT_t errorStatus_;
    // Methods    
    void updateVoltages(void);
    void updateCurrent(void);
    void updateTemperatures(void);
    void updateBalancingSwitches(void);
    uint8_t readRegister(uint8_t address);
    uint16_t readDoubleRegister(uint8_t address);
    void writeRegister(uint8_t address, uint8_t data);
};
    
}
