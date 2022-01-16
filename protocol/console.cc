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

#include "console.h"
#include "console_strings.h"
#include <stdlib.h>
#include "mcu/watchdog.h"
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

using stream::Flags::PGM;

namespace {

extern "C" uint16_t get_free_mem() {
    extern int16_t __heap_start, *__brkval;
    int16_t v;
    int16_t Free__Ram = (int16_t) &v - (__brkval == 0 ? (int16_t) &__heap_start : (int16_t) __brkval);
    return (uint16_t)abs(Free__Ram);
}

}

namespace protocol {

Console::Console():
    ser(mcu::Usart::get()),
    cout(ser),
    bq(bq769x_conf, bq769x_data, bq769x_stats),
    handle_result(false),
    param_len(0),
    handle_len(0),
    m_lastUpdate(0),
    m_oldMillis(0),
    m_millisOverflows(0),
    len(0),
    state(CONSOLE_STARTUP)
{
    cout << PGM << STR_msg_coy << EOL;
    cout << PGM << STR_msg_warn << EOL;
    cout << PGM << STR_msg_ver << EOL;
    conf_load();
    stats_load();
    m_BatCycles_prev    = bq769x_stats.batCycles_;
    m_ChargedTimes_prev = bq769x_stats.chargedTimes_;
    shd = 255;
}

void Console::conf_begin_protect() {
    bq.setShortCircuitProtection(           bq769x_conf.Cell_SCD_mA, bq769x_conf.Cell_SCD_us);
    bq.setOvercurrentChargeProtection(      bq769x_conf.Cell_OCD_mA, bq769x_conf.Cell_OCD_ms);
    bq.setOvercurrentDischargeProtection(   bq769x_conf.Cell_ODP_mA, bq769x_conf.Cell_ODP_ms);
    bq.setCellUndervoltageProtection(       bq769x_conf.Cell_UVP_mV, bq769x_conf.Cell_UVP_sec);
    bq.setCellOvervoltageProtection(        bq769x_conf.Cell_OVP_mV, bq769x_conf.Cell_OVP_sec);
}

void Console::begin() {
    bq.begin();
    bq.update();
    bq.resetSOC(100);
    bq.enableCharging();
    conf_begin_protect();
    
    if (bq769x_conf.Allow_Discharging) {
        bq.enableDischarging();
    } else {
        bq.disableDischarging();
    }
    bq.printRegisters();
    debug_print();
    print_all_conf();
    print_all_stats();
}

void Console::conf_default() {
    bq769x_conf.BQ_dbg            = false;
    bq769x_conf.Allow_Charging    = true;
    bq769x_conf.Allow_Discharging = true;
    bq769x_conf.RT_bits    = BQ769X0_THERMISTORS;
    bq769x_conf.RS_uOhm    = 1000; // Shunt, 1mOhm
    bq769x_conf.RT_Beta[0] = 3435; // for Semitec 103AT-5 thermistor
#ifdef IC_BQ76930
    bq76940_conf.RT_Beta[1] = 3435;
#endif
#ifdef IC_BQ76940
    bq769x_conf.RT_Beta[1] = 3435;
    bq769x_conf.RT_Beta[2] = 3435;
#endif
    // Capacity calc
    bq769x_conf.Cell_CapaNom_mV         = 3600;     // mV, nominal voltage for single cell
    bq769x_conf.Cell_CapaFull_mV        = 4180;     // mV, full voltage for single cell
    bq769x_conf.Batt_CapaNom_mAsec      = 360000;   // mA*sec, nominal capacity of battery pack, max. 580 Ah possible @ 3.7V

    bq769x_conf.CurrentThresholdIdle_mA = 100;  // 30 Current (mA)

    // TODO Temperature for any sensors
    bq769x_conf.Cell_TempCharge_min     =    0; // Temperature limits (Cx10)
    bq769x_conf.Cell_TempCharge_max     =  500; // Temperature limits (Cx10)
    bq769x_conf.Cell_TempDischarge_min  = -200; // Temperature limits (Cx10)
    bq769x_conf.Cell_TempDischarge_max  =  650; // Temperature limits (Cx10)
    
    
    bq769x_conf.BalancingInCharge       = true; // false
    bq769x_conf.BalancingEnable         = true; // false
    bq769x_conf.BalancingCellMin_mV     = 3600; // Cell voltage (mV)
    bq769x_conf.BalancingCellMaxDifference_mV   = 10;   // 20 
    bq769x_conf.BalancingIdleTimeMin_s          = 1800;
   
    // checkUser Cell overcurrent charge protection
    bq769x_conf.Cell_OCD_mA    = 5500;
    bq769x_conf.Cell_OCD_ms    = 3000;
    
    // PROTECT1 Cell short circuit protection
    bq769x_conf.Cell_SCD_mA    = 80000;
    bq769x_conf.Cell_SCD_us    = 200;
    
    // PROTECT2 Cell overcurrent discharge protection
    bq769x_conf.Cell_ODP_mA    = 40000;
    bq769x_conf.Cell_ODP_ms    = 2000;

    // PROTECT3 Cell voltage protection limits
    bq769x_conf.Cell_OVP_mV    = 4200;
    bq769x_conf.Cell_OVP_sec   = 2;
    // min
    bq769x_conf.Cell_UVP_mV    = 2850;
    bq769x_conf.Cell_UVP_sec   = 2;

    memset(bq769x_conf.adcCellsOffset_, 0, sizeof(bq769x_conf.adcCellsOffset_));
    
}

void Console::cmd_conf_print() { print_all_conf(); }

void Console::cmd_stats_print() { print_all_stats(); }

void Console::cmd_stats_save() {
    stats_save();
    cout << PGM << PSTR("stats saved");
}

void Console::cmd_Allow_Charging() {
    if (param_len) {
        bq769x_conf.Allow_Charging = (bool)atoi(param);
        if (bq769x_conf.Allow_Charging) {
            cout << bq.enableCharging() << EOL;
        } else {
            bq.disableCharging();
        }
    }
    print_conf(PrintParam::Conf_Allow_Charging);
}

void Console::cmd_Allow_Discharging() {
    if (param_len) {
        bq769x_conf.Allow_Discharging = (bool)atoi(param);
        if (bq769x_conf.Allow_Discharging) {
            cout << bq.enableDischarging() << EOL;
        } else {
            bq.disableDischarging();
        }
    }
    print_conf(PrintParam::Conf_Allow_Discharging);
}

void Console::cmd_BQ_dbg() {
    if (param_len) {
        bq769x_conf.BQ_dbg = (bool)atoi(param);    
    }
    print_conf(PrintParam::Conf_BQ_dbg);    
}

void Console::cmd_RT_bits() {
    if (param_len) {
        if (param_len == 5) {
            uint8_t a,b,c;
            a = atoi(param);
            b = atoi(param+2);
            c = atoi(param+4);
            if (a) bq769x_conf.RT_bits |= (1 << 0); else bq769x_conf.RT_bits &= ~(1 << 0);
            if (b) bq769x_conf.RT_bits |= (1 << 1); else bq769x_conf.RT_bits &= ~(1 << 1);
            if (c) bq769x_conf.RT_bits |= (1 << 2); else bq769x_conf.RT_bits &= ~(1 << 2);
        } else write_help(cout, STR_cmd_RT_bits, STR_cmd_RT_bits_HELP);
    }
    print_conf(PrintParam::Conf_RT_bits);
}

void Console::cmd_RS_uOhm() {
    if (param_len) {
        uint32_t sr = 0;
        sr = atoi(param);
        if (sr) bq769x_conf.RS_uOhm = sr;
        else write_help(cout, STR_cmd_RS_uOhm, STR_cmd_RS_uOhm_HELP);
    }
    print_conf(PrintParam::Conf_RS_uOhm);
}

void Console::cmd_RT_Beta() {
    if (param_len) {
        if (param_len == 14) {
            bq769x_conf.RT_Beta[0] = atoi(param);
            bq769x_conf.RT_Beta[1] = atoi(param+5);
            bq769x_conf.RT_Beta[2] = atoi(param+10);
        } else write_help(cout, STR_cmd_RT_Beta, STR_cmd_RT_Beta_HELP);
    }
    print_conf(PrintParam::Conf_RT_Beta);
}

void Console::cmd_Cell_CapaNom_mV() {
    if (param_len) {
        if (param_len == 4) {
            bq769x_conf.Cell_CapaNom_mV = atoi(param);
        } else write_help(cout, STR_cmd_Cell_CapaNom_mV, STR_cmd_Cell_CapaNom_mV_HELP);
    }
    print_conf(PrintParam::Conf_Cell_CapaNom_mV);
}

void Console::cmd_Cell_CapaFull_mV() {
    if (param_len) {
        if (param_len == 4) {
            bq769x_conf.Cell_CapaFull_mV = atoi(param);
        } else write_help(cout, STR_cmd_Cell_CapaFull_mV, STR_cmd_Cell_CapaFull_mV_HELP);
    }
    print_conf(PrintParam::Conf_Cell_CapaFull_mV);
}

void Console::cmd_Batt_CapaNom_mAsec() {
    if (param_len) {
        int32_t t = atol(param);
        if (t) bq769x_conf.Batt_CapaNom_mAsec = t * 60 * 60;
        else write_help(cout, STR_cmd_Batt_CapaNom_mAsec, STR_cmd_Batt_CapaNom_mAsec_HELP);
    }
    print_conf(PrintParam::Conf_Batt_CapaNom_mAsec);
}

void Console::cmd_CurrentThresholdIdle_mA() {
    if (param_len) {
        uint32_t t = atoi(param);
        if (t > 0) bq769x_conf.CurrentThresholdIdle_mA = t;
        else write_help(cout, STR_cmd_CurrentThresholdIdle_mA, STR_cmd_CurrentThresholdIdle_mA_HELP);
    }
    print_conf(PrintParam::Conf_CurrentThresholdIdle_mA);
}

void Console::cmd_Cell_TempCharge_min() {
    if (param_len) {
        int16_t t = atoi(param);
        if (t < bq769x_conf.Cell_TempCharge_max) bq769x_conf.Cell_TempCharge_min = t;
        else write_help(cout, STR_cmd_Cell_TempCharge_min, STR_cmd_Cell_TempCharge_min_HELP);
    }
    print_conf(PrintParam::Conf_Cell_TempCharge_min);
}

void Console::cmd_Cell_TempCharge_max() {
    if (param_len) {
        int16_t t = atoi(param);
        if (t > bq769x_conf.Cell_TempCharge_min) bq769x_conf.Cell_TempCharge_max = t;
        else write_help(cout, STR_cmd_Cell_TempCharge_max, STR_cmd_Cell_TempCharge_max_HELP);
    }
    print_conf(PrintParam::Conf_Cell_TempCharge_max);
}

void Console::cmd_Cell_TempDischarge_min() {
    if (param_len) {
        int16_t t = atoi(param);
        if (t < bq769x_conf.Cell_TempDischarge_max) bq769x_conf.Cell_TempDischarge_min = t;
        else write_help(cout, STR_cmd_Cell_TempDischarge_min, STR_cmd_Cell_TempDischarge_min_HELP);
    }
    print_conf(PrintParam::Conf_Cell_TempDischarge_min);
}

void Console::cmd_Cell_TempDischarge_max() {
    if (param_len) {
        int16_t t = atoi(param);
        if (t > bq769x_conf.Cell_TempDischarge_min) bq769x_conf.Cell_TempDischarge_max = t;
        else write_help(cout, STR_cmd_Cell_TempDischarge_max, STR_cmd_Cell_TempDischarge_max_HELP);
    }
    print_conf(PrintParam::Conf_Cell_TempDischarge_max);
}

void Console::cmd_BalancingInCharge() {
    if (param_len) {
        bq769x_conf.BalancingInCharge = (bool)atoi(param);
    }
    print_conf(PrintParam::Conf_BalancingInCharge);
}

void Console::cmd_BalancingEnable() {
    if (param_len) {
        bq769x_conf.BalancingEnable = (bool)atoi(param);
    }
    print_conf(PrintParam::Conf_BalancingEnable);
}

void Console::cmd_BalancingCellMin_mV() {
    if (param_len) {
        int32_t t = atoi(param);
        if (t > 0) bq769x_conf.BalancingCellMin_mV = t;
        else write_help(cout, STR_cmd_BalancingCellMin_mV, STR_cmd_BalancingCellMin_mV_HELP);
    }
    print_conf(PrintParam::Conf_BalancingCellMin_mV);
}

void Console::cmd_BalancingCellMaxDifference_mV() {
    if (param_len) {
        uint8_t t = atoi(param);
        if (t > 0) bq769x_conf.BalancingCellMaxDifference_mV = t;
        else write_help(cout, STR_cmd_BalancingCellMaxDifference_mV, STR_cmd_BalancingCellMaxDifference_mV_HELP);
    }
    print_conf(PrintParam::Conf_BalancingCellMaxDifference_mV);
}

void Console::cmd_BalancingIdleTimeMin_s() {
    if (param_len) {
        uint16_t t = atoi(param);
        if (t > 0) bq769x_conf.BalancingIdleTimeMin_s = t;
        else write_help(cout, STR_cmd_BalancingIdleTimeMin_s, STR_cmd_BalancingIdleTimeMin_s_HELP);
    }
    print_conf(PrintParam::Conf_BalancingIdleTimeMin_s);
}

void Console::cmd_Cell_OCD_mA() {
    if (param_len) {
        int32_t t = atoi(param);
        if (t > 0) {
            bq769x_conf.Cell_OCD_mA = t;
            cout << bq.setOvercurrentChargeProtection(bq769x_conf.Cell_OCD_mA, bq769x_conf.Cell_OCD_ms) << EOL;
        }
        else write_help(cout, STR_cmd_Cell_OCD_mA, STR_cmd_Cell_OCD_mA_HELP);
    }
    print_conf(PrintParam::Conf_Cell_OCD_mA);
}

void Console::cmd_Cell_OCD_ms() {
    if (param_len) {
        uint16_t t = atoi(param);
        if (t > 0) {
            bq769x_conf.Cell_OCD_ms = t;
            cout << bq.setOvercurrentChargeProtection(bq769x_conf.Cell_OCD_mA, bq769x_conf.Cell_OCD_ms) << EOL;
        }
        else write_help(cout, STR_cmd_Cell_OCD_ms, STR_cmd_Cell_OCD_ms_HELP);
    }
    print_conf(PrintParam::Conf_Cell_OCD_ms);
}

void Console::cmd_Cell_SCD_mA() {
    if (param_len) {
        uint32_t t = atoi(param);
        if (t > 0) {
            bq769x_conf.Cell_SCD_mA = t;
            cout << bq.setShortCircuitProtection(bq769x_conf.Cell_SCD_mA, bq769x_conf.Cell_SCD_us) << EOL;
        }
        else write_help(cout, STR_cmd_Cell_SCD_mA, STR_cmd_Cell_SCD_mA_HELP);
    }
    print_conf(PrintParam::Conf_Cell_SCD_mA);
}

void Console::cmd_Cell_SCD_us() {
    if (param_len) {
        uint16_t t = atoi(param);
        if (t > 0) {
            bq769x_conf.Cell_SCD_us = t;
            cout << bq.setShortCircuitProtection(bq769x_conf.Cell_SCD_mA, bq769x_conf.Cell_SCD_us) << EOL;
        }
        else write_help(cout, STR_cmd_Cell_SCD_us, STR_cmd_Cell_SCD_us_HELP);
    }
    print_conf(PrintParam::Conf_Cell_SCD_us);
}

void Console::cmd_Cell_ODP_mA() {
    if (param_len) {
        uint32_t t = atoi(param);
        if (t > 0) {
            bq769x_conf.Cell_ODP_mA = t;
            cout << bq.setOvercurrentDischargeProtection(bq769x_conf.Cell_ODP_mA, bq769x_conf.Cell_ODP_ms) << EOL;
        }
        else write_help(cout, STR_cmd_Cell_ODP_mA, STR_cmd_Cell_ODP_mA_HELP);
    }
    print_conf(PrintParam::Conf_Cell_ODP_mA);
}

void Console::cmd_Cell_ODP_ms() {
    if (param_len) {
        uint16_t t = atoi(param);
        if (t > 0) {
            bq769x_conf.Cell_ODP_ms = t;
            cout << bq.setOvercurrentDischargeProtection(bq769x_conf.Cell_ODP_mA, bq769x_conf.Cell_ODP_ms) << EOL;
        }
        else write_help(cout, STR_cmd_Cell_ODP_ms, STR_cmd_Cell_ODP_ms_HELP);
    }
    print_conf(PrintParam::Conf_Cell_ODP_ms);
}

void Console::cmd_Cell_OVP_mV() {
    if (param_len) {
        uint16_t t = atoi(param);
        if (t > bq769x_conf.Cell_UVP_mV) {
            bq769x_conf.Cell_OVP_mV = t;
            cout << bq.setCellOvervoltageProtection(bq769x_conf.Cell_OVP_mV, bq769x_conf.Cell_OVP_sec) << EOL;
        }
        else write_help(cout, STR_cmd_Cell_OVP_mV, STR_cmd_Cell_OVP_mV_HELP);
    }
    print_conf(PrintParam::Conf_Cell_OVP_mV);
}

void Console::cmd_Cell_OVP_sec() {
    if (param_len) {
        bq769x_conf.Cell_OVP_sec = atoi(param);
        cout << bq.setCellOvervoltageProtection(bq769x_conf.Cell_OVP_mV, bq769x_conf.Cell_OVP_sec) << EOL;
    }
    print_conf(PrintParam::Conf_Cell_OVP_sec);
    
}

void Console::cmd_Cell_UVP_mV() {
    if (param_len) {
        uint16_t t = atoi(param);
        if (t < bq769x_conf.Cell_OVP_mV) {
            bq769x_conf.Cell_UVP_mV = t;
            cout << bq.setCellUndervoltageProtection(bq769x_conf.Cell_UVP_mV, bq769x_conf.Cell_UVP_sec) << EOL;
        }
        else write_help(cout, STR_cmd_Cell_UVP_mV, STR_cmd_Cell_UVP_mV_HELP);
    }
    print_conf(PrintParam::Conf_Cell_UVP_mV);
}

void Console::cmd_Cell_UVP_sec() {
    if (param_len) {
        bq769x_conf.Cell_UVP_sec = atoi(param);
        cout << bq.setCellUndervoltageProtection(bq769x_conf.Cell_UVP_mV, bq769x_conf.Cell_UVP_sec) << EOL;
    }
    print_conf(PrintParam::Conf_Cell_UVP_sec);
}

void Console::print_conf(const PrintParam c) {
    switch (c) {
        case Conf_Allow_Charging:
            cout << PGM << STR_cmd_Allow_Charging << '=' <<
            bq769x_conf.Allow_Charging;
            cout << PGM << STR_cmd_Allow_Charging_HELP;
            break;
            break;
        case Conf_Allow_Discharging:
            cout << PGM << STR_cmd_Allow_Discharging << '=' <<
            bq769x_conf.Allow_Discharging;
            cout << PGM << STR_cmd_Allow_Discharging_HELP;
            break;
            break;
        case Conf_BQ_dbg:
            cout << PGM << STR_cmd_BQ_dbg << '=' <<
            bq769x_conf.BQ_dbg;
            cout << PGM << STR_cmd_BQ_dbg_HELP;
            break;
        case Conf_RT_bits:
            cout << PGM << STR_cmd_RT_bits << '=';
            if (bq769x_conf.RT_bits & (1 << 0)) cout << '1'; else cout << '0'; cout << ' ';
            if (bq769x_conf.RT_bits & (1 << 1)) cout << '1'; else cout << '0'; cout << ' ';
            if (bq769x_conf.RT_bits & (1 << 2)) cout << '1'; else cout << '0';
            cout << PGM << STR_cmd_RT_bits_HELP;
            break;
            
        case Conf_RS_uOhm:
            cout << PGM << STR_cmd_RS_uOhm << '=' <<
            bq769x_conf.RS_uOhm;
            cout << PGM << STR_cmd_RS_uOhm_HELP;
            break;
            
        case Conf_RT_Beta:
            cout << PGM << STR_cmd_RT_Beta << '=' <<
            bq769x_conf.RT_Beta[0] << ' ' <<
            bq769x_conf.RT_Beta[1] << ' ' <<
            bq769x_conf.RT_Beta[2];
            cout << PGM << STR_cmd_RT_Beta_HELP;
            break;
            
        case Conf_Cell_CapaNom_mV:
            cout << PGM << STR_cmd_Cell_CapaNom_mV << '=' <<
            bq769x_conf.Cell_CapaNom_mV;
            cout << PGM << STR_cmd_Cell_CapaNom_mV_HELP;
            break;
            
        case Conf_Cell_CapaFull_mV:
            cout << PGM << STR_cmd_Cell_CapaFull_mV << '=' <<
            bq769x_conf.Cell_CapaFull_mV;
            cout << PGM << STR_cmd_Cell_CapaFull_mV_HELP;
            break;
            
        case Conf_Batt_CapaNom_mAsec:
            cout << PGM << STR_cmd_Batt_CapaNom_mAsec << '=' <<
            bq769x_conf.Batt_CapaNom_mAsec/ 60 * 60;
            cout << PGM << STR_cmd_Batt_CapaNom_mAsec_HELP;
            break;
            
        case Conf_CurrentThresholdIdle_mA:
            cout << PGM << STR_cmd_CurrentThresholdIdle_mA << '=' <<
            bq769x_conf.CurrentThresholdIdle_mA;
            cout << PGM << STR_cmd_CurrentThresholdIdle_mA_HELP;
            break;
            
        case Conf_Cell_TempCharge_min:
            cout << PGM << STR_cmd_Cell_TempCharge_min << '=' <<
            bq769x_conf.Cell_TempCharge_min;
            cout << PGM << STR_cmd_Cell_TempCharge_min_HELP;
            break;
            
        case Conf_Cell_TempCharge_max:
            cout << PGM << STR_cmd_Cell_TempCharge_max << '=' <<
            bq769x_conf.Cell_TempCharge_max;
            cout << PGM << STR_cmd_Cell_TempCharge_max_HELP;
            break;
            
        case Conf_Cell_TempDischarge_min:
            cout << PGM << STR_cmd_Cell_TempDischarge_min << '=' <<
            bq769x_conf.Cell_TempDischarge_min;
            cout << PGM << STR_cmd_Cell_TempDischarge_min_HELP;
            break;
            
        case Conf_Cell_TempDischarge_max:
            cout << PGM << STR_cmd_Cell_TempDischarge_max << '=' <<
            bq769x_conf.Cell_TempDischarge_max;
            cout << PGM << STR_cmd_Cell_TempDischarge_max_HELP;
            break;
            
        case Conf_BalancingInCharge:
            cout << PGM << STR_cmd_BalancingInCharge << '=' <<
            bq769x_conf.BalancingInCharge;
            cout << PGM << STR_cmd_BalancingInCharge_HELP;
            break;
            
        case Conf_BalancingEnable:
            cout << PGM << STR_cmd_BalancingEnable << '=' <<
            bq769x_conf.BalancingEnable;
            cout << PGM << STR_cmd_BalancingEnable_HELP;
            break;
            
        case Conf_BalancingCellMin_mV:
            cout << PGM << STR_cmd_BalancingCellMin_mV << '=' <<
            bq769x_conf.BalancingCellMin_mV;
            cout << PGM << STR_cmd_BalancingCellMin_mV_HELP;
            break;
            
        case Conf_BalancingCellMaxDifference_mV:
            cout << PGM << STR_cmd_BalancingCellMaxDifference_mV << '=' <<
            bq769x_conf.BalancingCellMaxDifference_mV;
            cout << PGM << STR_cmd_BalancingCellMaxDifference_mV_HELP;
            break;
            
        case Conf_BalancingIdleTimeMin_s:
            cout << PGM << STR_cmd_BalancingIdleTimeMin_s << '=' <<
            bq769x_conf.BalancingIdleTimeMin_s;
            cout << PGM << STR_cmd_BalancingIdleTimeMin_s_HELP;
            break;
            
        case Conf_Cell_OCD_mA:
            cout << PGM << STR_cmd_Cell_OCD_mA << '=' <<
            bq769x_conf.Cell_OCD_mA;
            cout << PGM << STR_cmd_Cell_OCD_mA_HELP;
            break;
            
        case Conf_Cell_OCD_ms:
            cout << PGM << STR_cmd_Cell_OCD_ms << '=' <<
            bq769x_conf.Cell_OCD_ms;
            cout << PGM << STR_cmd_Cell_OCD_ms_HELP;
            break;
            
        case Conf_Cell_SCD_mA:
            cout << PGM << STR_cmd_Cell_SCD_mA << '=' <<
            bq769x_conf.Cell_SCD_mA;
            cout << PGM << STR_cmd_Cell_SCD_mA_HELP;
            break;
            
        case Conf_Cell_SCD_us:
            cout << PGM << STR_cmd_Cell_SCD_us << '=' <<
            bq769x_conf.Cell_SCD_us;
            cout << PGM << STR_cmd_Cell_SCD_us_HELP;
            break;
            
        case Conf_Cell_ODP_mA:
            cout << PGM << STR_cmd_Cell_ODP_mA << '=' <<
            bq769x_conf.Cell_ODP_mA;
            cout << PGM << STR_cmd_Cell_ODP_mA_HELP;
            break;
            
        case Conf_Cell_ODP_ms:
            cout << PGM << STR_cmd_Cell_ODP_ms << '=' <<
            bq769x_conf.Cell_ODP_ms;
            cout << PGM << STR_cmd_Cell_ODP_ms_HELP;
            break;
            
        case Conf_Cell_OVP_mV:
            cout << PGM << STR_cmd_Cell_OVP_mV << '=' <<
            bq769x_conf.Cell_OVP_mV;
            cout << PGM << STR_cmd_Cell_OVP_mV_HELP;
            break;
            
        case Conf_Cell_OVP_sec:
            cout << PGM << STR_cmd_Cell_OVP_sec << '=' <<
            bq769x_conf.Cell_OVP_sec;
            cout << PGM << STR_cmd_Cell_OVP_sec_HELP;
            break;
            
        case Conf_Cell_UVP_mV:
            cout << PGM << STR_cmd_Cell_UVP_mV << '=' <<
            bq769x_conf.Cell_UVP_mV;
            cout << PGM << STR_cmd_Cell_UVP_mV_HELP;
            break;
            
        case Conf_Cell_UVP_sec:
            cout << PGM << STR_cmd_Cell_UVP_sec << '=' <<
            bq769x_conf.Cell_UVP_sec;
            cout << PGM << STR_cmd_Cell_UVP_sec_HELP;
            break;
            
        case Conf_adcCellsOffset:
            cout << "TODO";
            break;
            
        case Conf_ts:
            cout << "TS: " << bq769x_conf.ts;
            break;
            
        case Conf_CRC8:
            cout << "CRC8: " << bq769x_conf.crc8;
            break;
        default: {
            cout << '?';
        }
    }
}

void Console::print_all_conf() {
    for (uint8_t i = FIRST ; i <= LAST; i++) {
        print_conf((PrintParam)i);
        cout << EOL;
    }
}
char const STR_TS[] PROGMEM = " timestamp = ";

void Console::print_all_stats() {
    cout
        << PGM << PSTR("ADC Gain=") << bq769x_stats.adcGain_
        << PGM << PSTR(" Offset=")  << bq769x_stats.adcOffset_
        << PGM << PSTR("\r\nBAT Cycles=") << bq769x_stats.batCycles_
        << PGM << PSTR(" Charged times=")  << bq769x_stats.chargedTimes_
        << PGM << PSTR("\r\nLook Cell mVmin=") << bq769x_stats.idCellMinVoltage_
        << PGM << PSTR(" mVmax=")  << bq769x_stats.idCellMaxVoltage_
        << PGM << PSTR("\r\nTimestamp idle=") << bq769x_stats.idleTimestamp_
        << PGM << PSTR(" charge=")  << bq769x_stats.chargeTimestamp_
        << PGM << PSTR(" saved in EEPROM=")  << bq769x_stats.ts;
        
    cout
        << PGM << PSTR("\r\nErrors counter:")
        << PGM << PSTR("\r\nXREADY = ") << bq769x_stats.errorCounter_[devices::ERROR_XREADY] << PGM << STR_TS << bq769x_stats.errorTimestamps_[devices::ERROR_XREADY]
        << PGM << PSTR("\r\n ALERT = ") << bq769x_stats.errorCounter_[devices::ERROR_ALERT] << PGM << STR_TS << bq769x_stats.errorTimestamps_[devices::ERROR_ALERT]
        << PGM << PSTR("\r\n   UVP = ") << bq769x_stats.errorCounter_[devices::ERROR_UVP] << PGM << STR_TS << bq769x_stats.errorTimestamps_[devices::ERROR_UVP]
        << PGM << PSTR("\r\n   OVP = ") << bq769x_stats.errorCounter_[devices::ERROR_OVP] << PGM << STR_TS << bq769x_stats.errorTimestamps_[devices::ERROR_OVP]
        << PGM << PSTR("\r\n   SCD = ") << bq769x_stats.errorCounter_[devices::ERROR_SCD] << PGM << STR_TS << bq769x_stats.errorTimestamps_[devices::ERROR_SCD]
        << PGM << PSTR("\r\n   OCD = ") << bq769x_stats.errorCounter_[devices::ERROR_OCD] << PGM << STR_TS << bq769x_stats.errorTimestamps_[devices::ERROR_OCD]
        << PGM << PSTR("\r\n     USR_SWITCH = ") << bq769x_stats.errorCounter_[devices::ERROR_USER_SWITCH] << PGM << STR_TS << bq769x_stats.errorTimestamps_[devices::ERROR_USER_SWITCH]
        << PGM << PSTR("\r\nUSR_DISCHG_TEMP = ") << bq769x_stats.errorCounter_[devices::ERROR_USER_DISCHG_TEMP] << PGM << STR_TS << bq769x_stats.errorTimestamps_[devices::ERROR_USER_DISCHG_TEMP]
        << PGM << PSTR("\r\n   USR_CHG_TEMP = ") << bq769x_stats.errorCounter_[devices::ERROR_USER_CHG_TEMP] << PGM << STR_TS << bq769x_stats.errorTimestamps_[devices::ERROR_USER_CHG_TEMP]
        << PGM << PSTR("\r\n    USR_CHG_OCD = ") << bq769x_stats.errorCounter_[devices::ERROR_USER_CHG_OCD] << PGM << STR_TS << bq769x_stats.errorTimestamps_[devices::ERROR_USER_CHG_OCD];

    cout << PGM << PSTR("\r\nCell ID Map:\r\n");
    for (uint8_t i = 0; i < MAX_NUMBER_OF_CELLS; i++) {
        cout << i << " = " << bq769x_stats.cellIdMap_[i] << '\t';
        if ((i+1) % 3 == 0) cout << EOL;
    }
    cout << PGM << PSTR("Cell Voltages:\r\n");
    for (uint8_t i = 0; i < MAX_NUMBER_OF_CELLS; i++) {
        cout << i << " = " << bq769x_stats.cellVoltages_[i] << " mV\t";
        if ((i+1) % 3 == 0) cout << EOL;
    }
    cout << PGM << PSTR("Temperatures x10C: ");
    for (uint8_t i = 0; i < MAX_NUMBER_OF_THERMISTORS; i++) {
        cout << i << " = " << bq769x_stats.temperatures_[i] << ' ';
    }
}


devices::bq769_stats EEMEM In_EEPROM_stats;

void Console::stats_load() {
    cout << PGM << PSTR("Stats load ");
    eeprom_read_block(&bq769x_stats, &In_EEPROM_stats, sizeof(bq769x_stats));
    if (bq769x_stats.crc8 != gencrc8((uint8_t*)&bq769x_stats, sizeof(bq769x_stats)-1)) {
        cout << PGM << PSTR("bad crc, restore zero");
        memset(&bq769x_stats, 0, sizeof(bq769x_stats));
        stats_save();
    } else cout << "OK";
    cout << EOL;
}

void Console::stats_save() {
    bq769x_stats.ts = mcu::Timer::millis();
    bq769x_stats.crc8 = gencrc8((uint8_t*)&bq769x_stats, sizeof(bq769x_stats)-1);
    eeprom_write_block(&bq769x_stats, &In_EEPROM_stats, sizeof(bq769x_stats));
}

devices::bq769_conf  EEMEM In_EEPROM_conf;

void Console::conf_load() {
    cout << PGM << PSTR("Conf load ");
    eeprom_read_block(&bq769x_conf, &In_EEPROM_conf, sizeof(bq769x_conf));
    if (bq769x_conf.crc8 != gencrc8((uint8_t*)&bq769x_conf, sizeof(bq769x_conf)-1)) {
        conf_default();
        cout << PGM << PSTR("bad crc, restore defs");
        conf_save();
    } else cout << "OK";
    cout << EOL;
}


void Console::conf_save() {
    bq769x_conf.ts = mcu::Timer::millis();
    bq769x_conf.crc8 = gencrc8((uint8_t*)&bq769x_conf, sizeof(bq769x_conf)-1);
    eeprom_write_block(&bq769x_conf, &In_EEPROM_conf, sizeof(bq769x_conf));
}
    
void Console::write_help(stream::OutputStream &out, const char *cmd, const char *help) {
    out << ' ' << PGM << cmd;
    uint8_t len = strlen_P(cmd);
    while (len++ < 24) out << ' ';
    out << PGM << help << EOL;
}

bool Console::update(mcu::Pin job, const bool force) {
    bool result = force;
    bq769x_data.alertInterruptFlag_ = force;
    uint32_t now = mcu::Timer::millis();
    if(now - m_lastUpdate >= 250) { // 250
//     if(force || (uint32_t)(now - m_lastUpdate) >= 250) { // 250
        result = false;
        job = 1;
        uint8_t error = bq.update(); // should be called at least every 250 ms
        m_lastUpdate = now;
        if(error & STAT_OV)  { cout << PGM << PSTR("Overvoltage!\r\n"); }
        if(error & STAT_UV)  {
            cout << PGM << PSTR("Undervoltage!\r\n");
            shd--;
            if (shd == 0) command_shutdown();
        }
        if(error & STAT_SCD) { cout << PGM << PSTR("Short Circuit Protection!\r\n"); }
        if(error & STAT_OCD) { cout << PGM << PSTR("Overcurrent Charge Protection!\r\n"); }
        if (bq769x_stats.batCycles_ != m_BatCycles_prev) {
            m_BatCycles_prev    = bq769x_stats.batCycles_;
            stats_save();
        }
        if (bq769x_stats.chargedTimes_ != m_ChargedTimes_prev) {
            m_ChargedTimes_prev = bq769x_stats.chargedTimes_;
            stats_save();
        }
        uint16_t bigDelta = bq.getMaxCellVoltage() - bq.getMinCellVoltage();
        if(bigDelta > 100) cout << PGM << PSTR("Difference too big!\r\n");
        if(m_oldMillis > now)
            m_millisOverflows++;
        m_oldMillis = now;
        job = 0;
    }
    return result;
}


void Console::command_restore() { conf_default(); conf_begin_protect(); }
void Console::command_save()    { conf_save(); }
void Console::command_print()   { debug_print(); }
void Console::command_bqregs()  { bq.printRegisters(); }
void Console::command_wdreset()  {
    stats_save();
    mcu::Watchdog::forceRestart(); //for (;;) { (void)0; }
}
void Console::command_freemem() { cout << PGM << PSTR(" Free RAM:") << get_free_mem() << EOL; }

void Console::command_shutdown() {
    stats_save();
    cout << PGM << STR_CMD_SHUTDOWN_HLP;
    bq.shutdown();
}

void Console::command_help() {
    cout << PGM << PSTR("Available commands:\r\n") << EOL;
    write_help(cout, STR_cmd_conf_print,    STR_cmd_conf_print_HELP);
    write_help(cout, STR_cmd_stats_print,   STR_cmd_stats_print_HELP);
    write_help(cout, STR_cmd_stats_save,    STR_cmd_stats_save_HELP);
    write_help(cout, STR_CMD_RESTORE,       STR_CMD_RESTORE_HLP);
    write_help(cout, STR_CMD_SAVE,          STR_CMD_SAVE_HLP);
    write_help(cout, STR_CMD_BQREGS,        STR_CMD_BQREGS_HLP);
    write_help(cout, STR_CMD_PRINT,         STR_CMD_PRINT_HLP);
    write_help(cout, STR_CMD_WDRESET,       STR_CMD_WDRESET_HLP);
    write_help(cout, STR_CMD_BOOTLOADER,    STR_CMD_BOOTLOADER_HLP);
    write_help(cout, STR_CMD_FREEMEM,       STR_CMD_FREEMEM_HLP);
    write_help(cout, STR_CMD_EPFORMAT,      STR_CMD_EPFORMAT_HLP);
    write_help(cout, STR_CMD_HELP,          STR_CMD_HELP_HLP);
    write_help(cout, STR_CMD_SHUTDOWN,      STR_CMD_SHUTDOWN_HLP);
    write_help(cout, STR_cmd_Allow_Charging,    STR_cmd_Allow_Charging_HELP);
    write_help(cout, STR_cmd_Allow_Discharging, STR_cmd_Allow_Discharging_HELP);
    write_help(cout, STR_cmd_BQ_dbg,            STR_cmd_BQ_dbg_HELP); 
    write_help(cout, STR_cmd_RT_bits,           STR_cmd_RT_bits_HELP); 
    write_help(cout, STR_cmd_RS_uOhm,           STR_cmd_RS_uOhm_HELP); 
    write_help(cout, STR_cmd_RT_Beta,           STR_cmd_RT_Beta_HELP); 
    write_help(cout, STR_cmd_Cell_CapaNom_mV,   STR_cmd_Cell_CapaNom_mV_HELP); 
    write_help(cout, STR_cmd_Cell_CapaFull_mV,          STR_cmd_Cell_CapaFull_mV_HELP); 
    write_help(cout, STR_cmd_Batt_CapaNom_mAsec,        STR_cmd_Batt_CapaNom_mAsec_HELP); 
    write_help(cout, STR_cmd_CurrentThresholdIdle_mA,   STR_cmd_CurrentThresholdIdle_mA_HELP); 
    write_help(cout, STR_cmd_Cell_TempCharge_min,       STR_cmd_Cell_TempCharge_min_HELP); 
    write_help(cout, STR_cmd_Cell_TempCharge_max,       STR_cmd_Cell_TempCharge_max_HELP); 
    write_help(cout, STR_cmd_Cell_TempDischarge_min,    STR_cmd_Cell_TempDischarge_min_HELP); 
    write_help(cout, STR_cmd_Cell_TempDischarge_max,    STR_cmd_Cell_TempDischarge_max_HELP); 
    write_help(cout, STR_cmd_BalancingInCharge,         STR_cmd_BalancingInCharge_HELP); 
    write_help(cout, STR_cmd_BalancingEnable,           STR_cmd_BalancingEnable_HELP); 
    write_help(cout, STR_cmd_BalancingCellMin_mV,       STR_cmd_BalancingCellMin_mV_HELP); 
    write_help(cout, STR_cmd_BalancingCellMaxDifference_mV,  STR_cmd_BalancingCellMaxDifference_mV_HELP); 
    write_help(cout, STR_cmd_BalancingIdleTimeMin_s,  STR_cmd_BalancingIdleTimeMin_s_HELP); 
    write_help(cout, STR_cmd_Cell_OCD_mA,   STR_cmd_Cell_OCD_mA_HELP); 
    write_help(cout, STR_cmd_Cell_OCD_ms,   STR_cmd_Cell_OCD_ms_HELP); 
    write_help(cout, STR_cmd_Cell_SCD_mA,   STR_cmd_Cell_SCD_mA_HELP); 
    write_help(cout, STR_cmd_Cell_SCD_us,   STR_cmd_Cell_SCD_us_HELP); 
    write_help(cout, STR_cmd_Cell_ODP_mA,   STR_cmd_Cell_ODP_mA_HELP); 
    write_help(cout, STR_cmd_Cell_ODP_ms,   STR_cmd_Cell_ODP_ms_HELP); 
    write_help(cout, STR_cmd_Cell_OVP_mV,   STR_cmd_Cell_OVP_mV_HELP); 
    write_help(cout, STR_cmd_Cell_OVP_sec,  STR_cmd_Cell_OVP_sec_HELP); 
    write_help(cout, STR_cmd_Cell_UVP_mV,   STR_cmd_Cell_UVP_mV_HELP); 
    write_help(cout, STR_cmd_Cell_UVP_sec,  STR_cmd_Cell_UVP_sec_HELP);    
    cout << EOL;
}


void Console::compare_cmd(const char *name_P, SerialCommandHandler handler) {
    const uint8_t cmd_len = strlen_P(name_P);
    if (strncmp_P(handle_buffer, name_P, cmd_len) == 0) {
        if ((handle_len > cmd_len) && (handle_buffer[cmd_len] == ' ')) {
            param = handle_buffer + cmd_len + 1;
            param_len = handle_len - cmd_len - 1;
            (this->*handler)(); // cmd with arg
        } else {
            param = nullptr;
            param_len = 0;
            (this->*handler)(); // single cmd
        }
        handle_result = true;
    }
}

bool Console::handleCommand(const char *buffer, const uint8_t len) {
    if (buffer[0] == 0) return false;
    handle_result = false;
    handle_buffer = buffer;
    handle_len = len;
    compare_cmd(STR_cmd_conf_print,             &Console::cmd_conf_print);
    compare_cmd(STR_cmd_stats_print,            &Console::cmd_stats_print);
    compare_cmd(STR_cmd_stats_save,             &Console::cmd_stats_save);
    compare_cmd(STR_CMD_RESTORE,                &Console::command_restore);
    compare_cmd(STR_CMD_SAVE,                   &Console::command_save);
    compare_cmd(STR_CMD_BQREGS,                 &Console::command_bqregs);
    compare_cmd(STR_CMD_PRINT,                  &Console::command_print);
    compare_cmd(STR_CMD_WDRESET,                &Console::command_wdreset);
    compare_cmd(STR_CMD_BOOTLOADER,             &Console::command_bootloader);
    compare_cmd(STR_CMD_FREEMEM,                &Console::command_freemem);
    compare_cmd(STR_CMD_EPFORMAT,               &Console::command_format_EEMEM);
    compare_cmd(STR_CMD_HELP,                   &Console::command_help);
    compare_cmd(STR_CMD_SHUTDOWN,               &Console::command_shutdown);
    compare_cmd(STR_cmd_Allow_Charging,         &Console::cmd_Allow_Charging);
    compare_cmd(STR_cmd_Allow_Discharging,      &Console::cmd_Allow_Discharging);
    compare_cmd(STR_cmd_BQ_dbg,                 &Console::cmd_BQ_dbg);
    compare_cmd(STR_cmd_RT_bits,                &Console::cmd_RT_bits);
    compare_cmd(STR_cmd_RS_uOhm,                &Console::cmd_RS_uOhm);
    compare_cmd(STR_cmd_RT_Beta,                &Console::cmd_RT_Beta);
    compare_cmd(STR_cmd_Cell_CapaNom_mV,        &Console::cmd_Cell_CapaNom_mV);
    compare_cmd(STR_cmd_Cell_CapaFull_mV,       &Console::cmd_Cell_CapaFull_mV);
    compare_cmd(STR_cmd_Batt_CapaNom_mAsec,     &Console::cmd_Batt_CapaNom_mAsec);
    compare_cmd(STR_cmd_CurrentThresholdIdle_mA,&Console::cmd_CurrentThresholdIdle_mA);
    compare_cmd(STR_cmd_Cell_TempCharge_min,    &Console::cmd_Cell_TempCharge_min);
    compare_cmd(STR_cmd_Cell_TempCharge_max,    &Console::cmd_Cell_TempCharge_max);
    compare_cmd(STR_cmd_Cell_TempDischarge_min, &Console::cmd_Cell_TempDischarge_min);
    compare_cmd(STR_cmd_Cell_TempDischarge_max, &Console::cmd_Cell_TempDischarge_max);
    compare_cmd(STR_cmd_BalancingInCharge,      &Console::cmd_BalancingInCharge);
    compare_cmd(STR_cmd_BalancingEnable,        &Console::cmd_BalancingEnable);
    compare_cmd(STR_cmd_BalancingCellMin_mV,    &Console::cmd_BalancingCellMin_mV);
    compare_cmd(STR_cmd_BalancingCellMaxDifference_mV, &Console::cmd_BalancingCellMaxDifference_mV);
    compare_cmd(STR_cmd_BalancingIdleTimeMin_s, &Console::cmd_BalancingIdleTimeMin_s);
    compare_cmd(STR_cmd_Cell_OCD_mA,            &Console::cmd_Cell_OCD_mA);
    compare_cmd(STR_cmd_Cell_OCD_ms,            &Console::cmd_Cell_OCD_ms);
    compare_cmd(STR_cmd_Cell_SCD_mA,            &Console::cmd_Cell_SCD_mA);
    compare_cmd(STR_cmd_Cell_SCD_us,            &Console::cmd_Cell_SCD_us);
    compare_cmd(STR_cmd_Cell_ODP_mA,            &Console::cmd_Cell_ODP_mA);  
    compare_cmd(STR_cmd_Cell_ODP_ms,            &Console::cmd_Cell_ODP_ms);
    compare_cmd(STR_cmd_Cell_OVP_mV,            &Console::cmd_Cell_OVP_mV);
    compare_cmd(STR_cmd_Cell_OVP_sec,           &Console::cmd_Cell_OVP_sec);
    compare_cmd(STR_cmd_Cell_UVP_mV,            &Console::cmd_Cell_UVP_mV);
    compare_cmd(STR_cmd_Cell_UVP_sec,           &Console::cmd_Cell_UVP_sec);

    if (!handle_result) { cout << PGM << PSTR("Unknown command. Try 'help'") << EOL; }
    return handle_result;
}

bool Console::Recv() {
    bool result = false;
    char ch;
    if (state == CONSOLE_STARTUP) {
        state = CONSOLE_ACCUMULATING;
    } else if (state == CONSOLE_ACCUMULATING) {
        while (ser.avail()) {
            result = true;
            ch = ser.read();
                ser.write(ch);
                if (ch == CR) ser.write(LF);
            if (ch == BackSpace || ch == Delete) {
                if (len) buffer[--len] = 0;
            } else {
                if (ch != LF) buffer[len++] = ch;
            }
            if (len == CONS_BUFF || ch == CR) {
                buffer[--len] = 0;
                state = CONSOLE_COMMAND;
                break;
            }
        }
    } else if (state == CONSOLE_COMMAND) {
        handleCommand(buffer, len);
        cout << "\r\nBMS>";
        len = 0;
        state = CONSOLE_ACCUMULATING;
    }
    return result;
}


void Console::debug_print() {
    uint32_t uptime = m_millisOverflows * (0xffffffffLL / 1000UL);
    uptime += mcu::Timer::millis() / 1000;

    cout
        << PGM << PSTR("BMS uptime: ") << uptime
        << PGM << PSTR(" BAT Temp: ") // TODO macro for x20 x30 Ic
        << bq.getTemperatureDegC(0) << ' '
        << bq.getTemperatureDegC(1) << ' '
        << bq.getTemperatureDegC(2)
        << EOL;

    cout 
        << PGM << PSTR("BAT Voltage: ")     << bq769x_data.batVoltage_
        << PGM << PSTR(" mV (")             << bq769x_data.batVoltage_raw_
        << PGM << PSTR(" raw), current: ")  << bq769x_data.batCurrent_
        << PGM << PSTR(" mA (")             << bq769x_data.batCurrent_raw_
        << PGM << PSTR(" raw)\r\n")
        << PGM << PSTR("SOC: ") << bq.getSOC()
        << PGM << PSTR(" Balancing status: ") << bq769x_data.balancingStatus_
        << PGM << PSTR("\r\nCell voltages:\r\n");
    
    for(uint8_t x = 0; x < MAX_NUMBER_OF_CELLS; x++) {
        uint8_t y = bq769x_stats.cellIdMap_[x];
        cout << bq769x_stats.cellVoltages_[y] << PGM << PSTR(" mV (") << bq769x_data.cellVoltages_raw_[y] << " raw)\t";
        if ((x+1) % 3 == 0) cout << EOL;
    }
    
    cout
        << PGM << PSTR("\r\nCell mV: Min: ")       << bq.getMinCellVoltage()
        << PGM << PSTR(" | Avg: ")                 << bq.getAvgCellVoltage()
        << PGM << PSTR(" | Max: ")                 << bq.getMaxCellVoltage()
        << PGM << PSTR(" | Delta: ")               << bq.getMaxCellVoltage() - bq.getMinCellVoltage()
        << PGM << PSTR("\r\nXREADY errors: ")      << bq769x_stats.errorCounter_[devices::BQ769xERR::ERROR_XREADY]
        << PGM << PSTR("\r\n ALERT errors: ")      << bq769x_stats.errorCounter_[devices::BQ769xERR::ERROR_ALERT]
        << PGM << PSTR("\r\n   UVP errors: ")      << bq769x_stats.errorCounter_[devices::BQ769xERR::ERROR_UVP]
        << PGM << PSTR("\r\n   OVP errors: ")      << bq769x_stats.errorCounter_[devices::BQ769xERR::ERROR_OVP]
        << PGM << PSTR("\r\n   SCD errors: ")      << bq769x_stats.errorCounter_[devices::BQ769xERR::ERROR_SCD]
        << PGM << PSTR("\r\n   OCD errors: ")      << bq769x_stats.errorCounter_[devices::BQ769xERR::ERROR_OCD]
        << PGM << PSTR("\r\nDISCHG TEMP errors: ") << bq769x_stats.errorCounter_[devices::BQ769xERR::ERROR_USER_DISCHG_TEMP]
        << PGM << PSTR("\r\n   CHG TEMP errors: ") << bq769x_stats.errorCounter_[devices::BQ769xERR::ERROR_USER_CHG_TEMP]
        << PGM << PSTR("\r\n    CHG OCD errors: ") << bq769x_stats.errorCounter_[devices::BQ769xERR::ERROR_USER_CHG_OCD]
        << EOL;
}

typedef void (*do_reboot_t)(void);
const do_reboot_t do_reboot = (do_reboot_t)((FLASHEND - 511) >> 1); // optiboot size

void Console::command_bootloader() {
    mcu::Watchdog::disable();
    cli();
    TCCR0A = 0;
    TCCR1A = 0;
    TCCR2A = 0;
    MCUSR = 0;
    do_reboot();
}

uint8_t gencrc8(uint8_t *data, uint16_t len) {
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        crc = devices::_crc8_ccitt_update(crc, data[i]);
    }
    return crc;
}

void Console::command_format_EEMEM() {
    for (int i = 0 ; i < E2END + 1 ; i++) {
        eeprom_write_byte((uint8_t*)i, 0xff);
        ser.write('.');
        mcu::Watchdog::reset();
    }
    cout << EOL;
}

}
