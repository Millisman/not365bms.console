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

#include "console_strings.h"
#include <avr/pgmspace.h>

namespace protocol {
char const STR_msg_coy[]                    PROGMEM = "not365 console app, Copyright (c) 2022 Sergey Kostanoy, https://arduino.uno";
char const STR_msg_warn[]                   PROGMEM = "WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND!";
char const STR_msg_ver[]                    PROGMEM = "Version 0.1 Alpha. USING IT IS YOUR RISK!";

char const STR_cmd_conf_print[]             PROGMEM = "confprint";
char const STR_cmd_conf_print_HELP[]        PROGMEM = " print all conf";
char const STR_cmd_stats_print[]            PROGMEM = "statsprint";
char const STR_cmd_stats_print_HELP[]       PROGMEM = " save statistics";
char const STR_cmd_stats_save[]             PROGMEM = "statssave";
char const STR_cmd_stats_save_HELP[]        PROGMEM = " print statistics";
char const STR_cmd_Allow_Charging[]         PROGMEM = "charging";
char const STR_cmd_Allow_Charging_HELP[]    PROGMEM = " on (1) or off (0) allow charging";
char const STR_cmd_Allow_Discharging[]      PROGMEM = "discharging";
char const STR_cmd_Allow_Discharging_HELP[] PROGMEM = " on (1) or off (0) allow discharging";
char const STR_cmd_BQ_dbg[]         PROGMEM = "bqdbg";
char const STR_cmd_BQ_dbg_HELP[]    PROGMEM = " on (1) or off (0) debug events on BQ769x0";
char const STR_cmd_RT_bits[]        PROGMEM = "thermistors";
char const STR_cmd_RT_bits_HELP[]   PROGMEM = " <1> <1> <1> - enable 3 of 3";
char const STR_cmd_RS_uOhm[]        PROGMEM = "shuntresistor";
char const STR_cmd_RS_uOhm_HELP[]   PROGMEM = " (1000) = 1mOhm";
char const STR_cmd_RT_Beta[]        PROGMEM = "thermistorbeta";
char const STR_cmd_RT_Beta_HELP[]   PROGMEM = " (3435) (3435) (3435) / Semitec 103AT-5";
char const STR_cmd_Cell_CapaNom_mV[]            PROGMEM = "cellnominalmv";
char const STR_cmd_Cell_CapaNom_mV_HELP[]       PROGMEM = " mV (3600)";
char const STR_cmd_Cell_CapaFull_mV[]           PROGMEM = "cellfullmv";
char const STR_cmd_Cell_CapaFull_mV_HELP[]      PROGMEM = " mV (4200)";
char const STR_cmd_Batt_CapaNom_mAsec[]         PROGMEM = "nominalcapacity";
char const STR_cmd_Batt_CapaNom_mAsec_HELP[]    PROGMEM = " ma*h, capacity of battery pack, max. 580 Ah";
char const STR_cmd_CurrentThresholdIdle_mA[]        PROGMEM = "idlecurrentth";
char const STR_cmd_CurrentThresholdIdle_mA_HELP[]   PROGMEM = " mA, for marking 'IDLE', 30-500";
char const STR_cmd_Cell_TempCharge_min[]        PROGMEM = "celltempchargemin";
char const STR_cmd_Cell_TempCharge_min_HELP[]   PROGMEM = " (0), x10 multipled, less celltempchargemax";
char const STR_cmd_Cell_TempCharge_max[]        PROGMEM = "celltempchargemax";
char const STR_cmd_Cell_TempCharge_max_HELP[]   PROGMEM = " (0), x10 multipled, above celltempchargemin";
char const STR_cmd_Cell_TempDischarge_min[]     PROGMEM = "celltempdischargemin";
char const STR_cmd_Cell_TempDischarge_min_HELP[]PROGMEM = " (0), x10 multipled, less celltempchargemax";
char const STR_cmd_Cell_TempDischarge_max[]     PROGMEM = "celltempdischargemax";
char const STR_cmd_Cell_TempDischarge_max_HELP[]PROGMEM = " (0), x10 multipled, above celltempchargemin";
char const STR_cmd_BalancingInCharge[]          PROGMEM = "balancecharging";
char const STR_cmd_BalancingInCharge_HELP[]     PROGMEM = " on (1) or off (0) on charging";
char const STR_cmd_BalancingEnable[]            PROGMEM = "autobalancing";
char const STR_cmd_BalancingEnable_HELP[]       PROGMEM = " on (1) or off (0)";
char const STR_cmd_BalancingCellMin_mV[]        PROGMEM = "balancingminmv";
char const STR_cmd_BalancingCellMin_mV_HELP[]   PROGMEM = " mV, min for balancing (3600)";
char const STR_cmd_BalancingCellMaxDifference_mV[]       PROGMEM = "balancingmaxdiff";
char const STR_cmd_BalancingCellMaxDifference_mV_HELP[]  PROGMEM = " mV, max for balancing (10)";
char const STR_cmd_BalancingIdleTimeMin_s[]       PROGMEM = "balancingidletime";
char const STR_cmd_BalancingIdleTimeMin_s_HELP[]  PROGMEM = " sec, min value (1800)";
char const STR_cmd_Cell_OCD_mA[]        PROGMEM = "maxchargecurrent";
char const STR_cmd_Cell_OCD_mA_HELP[]   PROGMEM = " mA, max charge (5000)";
char const STR_cmd_Cell_OCD_ms[]        PROGMEM = "maxchargecurrentdelay";
char const STR_cmd_Cell_OCD_ms_HELP[]   PROGMEM = " ms, overcurrent protect delay (3000)";
char const STR_cmd_Cell_SCD_mA[]        PROGMEM = "shortcircuitma";
char const STR_cmd_Cell_SCD_mA_HELP[]   PROGMEM = " mA, trigger current (80000)";
char const STR_cmd_Cell_SCD_us[]        PROGMEM = "shortcircuitus";
char const STR_cmd_Cell_SCD_us_HELP[]   PROGMEM = " us, trigger window (200)";
char const STR_cmd_Cell_ODP_mA[]        PROGMEM = "dischargema";
char const STR_cmd_Cell_ODP_mA_HELP[]   PROGMEM = " mA, max discharge (40000)";
char const STR_cmd_Cell_ODP_ms[]        PROGMEM = "dischargems";
char const STR_cmd_Cell_ODP_ms_HELP[]   PROGMEM = " ms, trigger window (2000)";
char const STR_cmd_Cell_OVP_mV[]        PROGMEM = "overvoltagemv";
char const STR_cmd_Cell_OVP_mV_HELP[]   PROGMEM = " mV, limit for cell (4200)";
char const STR_cmd_Cell_OVP_sec[]       PROGMEM = "overvoltagesec";
char const STR_cmd_Cell_OVP_sec_HELP[]  PROGMEM = " sec, trigger window (2)";
char const STR_cmd_Cell_UVP_mV[]        PROGMEM = "undervoltagemv";
char const STR_cmd_Cell_UVP_mV_HELP[]   PROGMEM = " mV, limit for cell (2850)";
char const STR_cmd_Cell_UVP_sec[]       PROGMEM = "undervoltagesec";
char const STR_cmd_Cell_UVP_sec_HELP[]  PROGMEM = " sec, trigger window (2)";
char const STR_CMD_RESTORE[]        PROGMEM = "restore";
char const STR_CMD_RESTORE_HLP[]    PROGMEM = " load saved conf from EEPROM";
char const STR_CMD_SAVE[]           PROGMEM = "save";
char const STR_CMD_SAVE_HLP[]       PROGMEM = " current conf to EEPROM";
char const STR_CMD_PRINT[]          PROGMEM = "print";
char const STR_CMD_PRINT_HLP[]      PROGMEM = " print status";
char const STR_CMD_WDRESET[]        PROGMEM = "reset";
char const STR_CMD_WDRESET_HLP[]    PROGMEM = " reset with watchdog";
char const STR_CMD_BOOTLOADER[]     PROGMEM = "bootloader";
char const STR_CMD_BOOTLOADER_HLP[] PROGMEM = " jump to bootloader";
char const STR_CMD_FREEMEM[]        PROGMEM = "mem";
char const STR_CMD_FREEMEM_HLP[]    PROGMEM = " show free memory";
char const STR_CMD_EPFORMAT[]       PROGMEM = "format";
char const STR_CMD_EPFORMAT_HLP[]   PROGMEM = " EEPROM (forced load defs in next boot)";
char const STR_CMD_HELP[]           PROGMEM = "help";
char const STR_CMD_HELP_HLP[]       PROGMEM = " this 'help'";
char const STR_CMD_BQREGS[]         PROGMEM = "bqregs";
char const STR_CMD_BQREGS_HLP[]     PROGMEM = " print regs in BQ769x0";
char const STR_CMD_SHUTDOWN[]       PROGMEM = "shutdown";
char const STR_CMD_SHUTDOWN_HLP[]   PROGMEM = " bye...bye...";

}
