#pragma once

#include <string>

struct PhySystemPacket
{
    /* data */
    uint32_t cmd_code;
    uint64_t time_stamp;
    uint16_t data_size;
    uint8_t data[400];

    PhySystemPacket() {
        cmd_code = 0;
        time_stamp = 0;
        data_size = 0;
    }
};

enum class phy_cmd : uint32_t
{
    CMD_COMM_START = 1,
    CMD_COMM_STOP = 2,
    CMD_COMM_CHANGE = 3,
    CMD_READ_SYS_CONFIG = 10,
    CMD_EM_SYS_STOP = 100,
    CMD_EM_CTRL_STOP = 101
};

struct comm_data
{
    bool is_heart_beat;
    uint8_t heart_beat_interval;
};