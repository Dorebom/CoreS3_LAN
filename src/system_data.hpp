#pragma once

#include <string>

#include "state_machine.hpp"

struct SystemConfig
{
    /* data */
    uint32_t heart_beat_interval;
    bool is_heart_beat;
    bool is_enable_loop_process;
    SystemConfig() {
        heart_beat_interval = 1000;  // ms
        is_heart_beat = false;
        is_enable_loop_process = false;
    }
};

struct SystemStatus
{
    /* data */
    StateMachine state_machine;
    uint8_t system_status;
    uint32_t ave_calc_time_for_main_task;
    uint32_t max_calc_time_for_main_task;
    uint32_t heart_beat_cnt;
    bool emergency_stop_switch_for_control_task;

    // Control Task Data Status
    uint8_t control_config_age;
    uint8_t control_status_age;
    SystemStatus() {
        system_status = 0;
        ave_calc_time_for_main_task = 0;  // ms
        max_calc_time_for_main_task = 0;  // ms
        heart_beat_cnt = 0;
        emergency_stop_switch_for_control_task = false;
        control_config_age = 0;
        control_status_age = 0;
    }
};
