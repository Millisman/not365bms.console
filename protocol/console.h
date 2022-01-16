/* Shell console for battery management based on bq769x Ic
 * Copyright (c) 2022 Sergey Kostanoy (https://arduino.uno)
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

#include <stdint.h>
#include "utils/cpp.h"
#include "stream/uartstream.h"
#include <string.h>
#include <avr/eeprom.h>
#include "devices/bq769x0.h"
#include "mcu/timer.h"
#include <avr/pgmspace.h>
#include "mcu/pin.h"

#define CONS_BUFF   100
#define BackSpace   0x08
#define Delete      0x7F

namespace protocol {

enum PrintParam {
    Conf_Allow_Charging,
    Conf_Allow_Discharging,
    Conf_BQ_dbg,
    Conf_RT_bits,
    Conf_RS_uOhm,
    Conf_RT_Beta,
    Conf_Cell_CapaNom_mV,
    Conf_Cell_CapaFull_mV,
    Conf_Batt_CapaNom_mAsec,
    Conf_CurrentThresholdIdle_mA,
    Conf_Cell_TempCharge_min,
    Conf_Cell_TempCharge_max,
    Conf_Cell_TempDischarge_min,
    Conf_Cell_TempDischarge_max,
    Conf_BalancingInCharge,
    Conf_BalancingEnable,
    Conf_BalancingCellMin_mV,
    Conf_BalancingCellMaxDifference_mV,
    Conf_BalancingIdleTimeMin_s,
    Conf_Cell_OCD_mA,
    Conf_Cell_OCD_ms,
    Conf_Cell_SCD_mA,
    Conf_Cell_SCD_us,
    Conf_Cell_ODP_mA,
    Conf_Cell_ODP_ms,
    Conf_Cell_OVP_mV,
    Conf_Cell_OVP_sec,
    Conf_Cell_UVP_mV,
    Conf_Cell_UVP_sec,
    Conf_adcCellsOffset,
    Conf_ts,
    Conf_CRC8,
    FIRST = Conf_Allow_Charging,
    LAST = Conf_CRC8
};

class Console;
typedef void (Console::*SerialCommandHandler)();
struct SerialCommand { const char *command; SerialCommandHandler handler; };

class Console {
    mcu::Usart &ser;
    stream::UartStream cout;
    devices::bq769_conf  bq769x_conf;
    devices::bq769_data  bq769x_data;
    devices::bq769_stats bq769x_stats;
    devices::bq769x0     bq;
    bool debug_events;
    bool handle_result;
    uint8_t param_len;
    uint8_t handle_len;
    uint32_t m_lastUpdate;
    uint32_t m_oldMillis = 0;
    uint32_t m_millisOverflows;
    uint8_t len;
    uint16_t m_BatCycles_prev;
    uint16_t m_ChargedTimes_prev;
    uint8_t shd;
public:
    Console();
    bool update(mcu::Pin job, const bool force);
    void begin();
    bool Recv();
private:
    void debug_print();
    void conf_begin_protect();
    void conf_default();
    void conf_load();
    void conf_save();
    void stats_load();
    void stats_save();
    void print_all_stats();
    
    
    void print_conf(const PrintParam c);
    void print_all_conf();
    
    void command_restore();
    void command_save();
    void command_print();
    void command_bqregs();
    void command_wdreset();
    void command_bootloader();
    void command_freemem();
    void command_format_EEMEM();
    void command_help();
    void command_shutdown();
    
    void cmd_conf_print();
    void cmd_stats_print();
    void cmd_stats_save();
    void cmd_Allow_Charging();
    void cmd_Allow_Discharging();
    void cmd_BQ_dbg();
    void cmd_RT_bits();
    void cmd_RS_uOhm();
    void cmd_RT_Beta();
    void cmd_Cell_CapaNom_mV();
    void cmd_Cell_CapaFull_mV();
    void cmd_Batt_CapaNom_mAsec();
    void cmd_CurrentThresholdIdle_mA();
    void cmd_Cell_TempCharge_min();
    void cmd_Cell_TempCharge_max();
    void cmd_Cell_TempDischarge_min();
    void cmd_Cell_TempDischarge_max();
    void cmd_BalancingInCharge();
    void cmd_BalancingEnable();
    void cmd_BalancingCellMin_mV();
    void cmd_BalancingCellMaxDifference_mV();
    void cmd_BalancingIdleTimeMin_s();
    void cmd_Cell_OCD_mA();
    void cmd_Cell_OCD_ms();
    void cmd_Cell_SCD_mA();
    void cmd_Cell_SCD_us();
    void cmd_Cell_ODP_mA();  
    void cmd_Cell_ODP_ms();
    void cmd_Cell_OVP_mV();
    void cmd_Cell_OVP_sec();
    void cmd_Cell_UVP_mV();
    void cmd_Cell_UVP_sec();
    
    bool handleCommand(const char *buffer, const uint8_t len);
    void write_help(stream::OutputStream &out, const char *cmd, const char *help);
    char buffer[CONS_BUFF];
    void compare_cmd(const char *name_P, SerialCommandHandler handler);
    enum SerialState { CONSOLE_STARTUP, CONSOLE_ACCUMULATING, CONSOLE_COMMAND };
    SerialState state;
    const char *handle_buffer;
    const char *param;
};

uint8_t gencrc8(uint8_t *data, uint16_t len);

}
