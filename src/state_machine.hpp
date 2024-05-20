#pragma once

#include <stdint.h>

#include <string>

enum class StateMachine : uint8_t
{
    STATE_INIT = 0,
    STATE_READY,
    STATE_STABLE,
    STATE_EM_STOP
};

enum class DataStatus : uint8_t
{
    DATA_NOT_READY = 0,
    DATA_READY = 1
};

enum class ControlTaskCmd : uint8_t
{
    SERV_ON = 10,   // enable motor
    SERV_OFF = 11,  // stop motor

    GET_MOTOR_ID = 12,
    CHANGE_CAN_ID = 13,
    SET_MECHANICAL_POSITION_TO_ZERO = 14,
    DUMP_CYBERGEAR_PARAMS = 15,

    SET_POS_MODE = 20,
    SET_VEL_MODE = 21,
    SET_TRQ_MODE = 22,
    READ_CTRL_MODE = 23,

    SET_POS_KP = 30,
    SET_POS_REF = 31,

    UPDATE_DUMMY_CONFIG_DATA = 101,
    UPDATE_DUMMY_STATUS_DATA = 102,
};