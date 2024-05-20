#pragma once

#include <stdint.h>

#include <string>

#include "state_machine.hpp"

struct ControlConfig
{
    /* data */
    uint32_t control_cycle;
    uint8_t dummy_data;
    ControlConfig() {
        dummy_data = 0;
        control_cycle = 100;  // ms
    }
};

struct ControlStatus
{
    /* data */
    StateMachine state_machine;
    uint8_t control_status;
    uint32_t ave_calc_time_for_control_task;
    uint32_t max_calc_time_for_control_task;
    uint32_t control_cycle_cnt;
    bool emergency_stop_switch_for_control_task;
    uint8_t error_cnt;
    uint8_t warning_cnt;

    uint8_t dummy_data;

    // Control
    bool is_serv_on;

    // Data Status
    DataStatus config_data_status;
    DataStatus status_data_status;

    ControlStatus() {
        dummy_data = 0;
        control_status = 0;
        ave_calc_time_for_control_task = 0;  // ms
        max_calc_time_for_control_task = 0;  // ms
        control_cycle_cnt = 0;
        emergency_stop_switch_for_control_task = false;
        error_cnt = 0;
        warning_cnt = 0;
        is_serv_on = false;
        config_data_status = DataStatus::DATA_NOT_READY;
        status_data_status = DataStatus::DATA_NOT_READY;
    }
};

struct ControlSysCmd
{
    uint8_t dummy_control_status_data;
    uint8_t dummy_control_config_data;

    ControlSysCmd() {
        dummy_control_status_data = 0;
        dummy_control_config_data = 0;
    }
};

struct ControlCmd
{
};