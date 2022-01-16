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

namespace protocol {
extern char const STR_msg_coy[];
extern char const STR_msg_warn[];
extern char const STR_msg_ver[];

extern char const STR_cmd_conf_print[];
extern char const STR_cmd_conf_print_HELP[];
extern char const STR_cmd_stats_print[];
extern char const STR_cmd_stats_print_HELP[];
extern char const STR_cmd_stats_save[];
extern char const STR_cmd_stats_save_HELP[];
extern char const STR_cmd_Allow_Charging[];
extern char const STR_cmd_Allow_Charging_HELP[];
extern char const STR_cmd_Allow_Discharging[];
extern char const STR_cmd_Allow_Discharging_HELP[];
extern char const STR_cmd_BQ_dbg[];
extern char const STR_cmd_BQ_dbg_HELP[];
extern char const STR_cmd_RT_bits[];
extern char const STR_cmd_RT_bits_HELP[];
extern char const STR_cmd_RS_uOhm[];
extern char const STR_cmd_RS_uOhm_HELP[];
extern char const STR_cmd_RT_Beta[];
extern char const STR_cmd_RT_Beta_HELP[];
extern char const STR_cmd_Cell_CapaNom_mV[];
extern char const STR_cmd_Cell_CapaNom_mV_HELP[];
extern char const STR_cmd_Cell_CapaFull_mV[];
extern char const STR_cmd_Cell_CapaFull_mV_HELP[];
extern char const STR_cmd_Batt_CapaNom_mAsec[];
extern char const STR_cmd_Batt_CapaNom_mAsec_HELP[];
extern char const STR_cmd_CurrentThresholdIdle_mA[];
extern char const STR_cmd_CurrentThresholdIdle_mA_HELP[];
extern char const STR_cmd_Cell_TempCharge_min[];
extern char const STR_cmd_Cell_TempCharge_min_HELP[];
extern char const STR_cmd_Cell_TempCharge_max[];
extern char const STR_cmd_Cell_TempCharge_max_HELP[];
extern char const STR_cmd_Cell_TempDischarge_min[];
extern char const STR_cmd_Cell_TempDischarge_min_HELP[];
extern char const STR_cmd_Cell_TempDischarge_max[];
extern char const STR_cmd_Cell_TempDischarge_max_HELP[];
extern char const STR_cmd_BalancingInCharge[];
extern char const STR_cmd_BalancingInCharge_HELP[];
extern char const STR_cmd_BalancingEnable[];
extern char const STR_cmd_BalancingEnable_HELP[];
extern char const STR_cmd_BalancingCellMin_mV[];
extern char const STR_cmd_BalancingCellMin_mV_HELP[];
extern char const STR_cmd_BalancingCellMaxDifference_mV[];
extern char const STR_cmd_BalancingCellMaxDifference_mV_HELP[];
extern char const STR_cmd_BalancingIdleTimeMin_s[];
extern char const STR_cmd_BalancingIdleTimeMin_s_HELP[];
extern char const STR_cmd_Cell_OCD_mA[];
extern char const STR_cmd_Cell_OCD_mA_HELP[];
extern char const STR_cmd_Cell_OCD_ms[];
extern char const STR_cmd_Cell_OCD_ms_HELP[];
extern char const STR_cmd_Cell_SCD_mA[];
extern char const STR_cmd_Cell_SCD_mA_HELP[];
extern char const STR_cmd_Cell_SCD_us[];
extern char const STR_cmd_Cell_SCD_us_HELP[];
extern char const STR_cmd_Cell_ODP_mA[];
extern char const STR_cmd_Cell_ODP_mA_HELP[];
extern char const STR_cmd_Cell_ODP_ms[];
extern char const STR_cmd_Cell_ODP_ms_HELP[];
extern char const STR_cmd_Cell_OVP_mV[];
extern char const STR_cmd_Cell_OVP_mV_HELP[];
extern char const STR_cmd_Cell_OVP_sec[];
extern char const STR_cmd_Cell_OVP_sec_HELP[];
extern char const STR_cmd_Cell_UVP_mV[];
extern char const STR_cmd_Cell_UVP_mV_HELP[];
extern char const STR_cmd_Cell_UVP_sec[];
extern char const STR_cmd_Cell_UVP_sec_HELP[];
extern char const STR_CMD_RESTORE[];
extern char const STR_CMD_RESTORE_HLP[];
extern char const STR_CMD_SAVE[];
extern char const STR_CMD_SAVE_HLP[];
extern char const STR_CMD_PRINT[];
extern char const STR_CMD_PRINT_HLP[];
extern char const STR_CMD_WDRESET[];
extern char const STR_CMD_WDRESET_HLP[];
extern char const STR_CMD_BOOTLOADER[];
extern char const STR_CMD_BOOTLOADER_HLP[];
extern char const STR_CMD_FREEMEM[];
extern char const STR_CMD_FREEMEM_HLP[];
extern char const STR_CMD_EPFORMAT[];
extern char const STR_CMD_EPFORMAT_HLP[];
extern char const STR_CMD_HELP[];
extern char const STR_CMD_HELP_HLP[];
extern char const STR_CMD_BQREGS[];
extern char const STR_CMD_BQREGS_HLP[];
extern char const STR_CMD_SHUTDOWN[];
extern char const STR_CMD_SHUTDOWN_HLP[];
}
