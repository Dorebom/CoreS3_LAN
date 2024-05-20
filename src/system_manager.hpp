#pragma once

#include <EthernetUdp.h>
#include <M5CoreS3.h>
#include <M5GFX.h>
#include <M5Module_LAN.h>
#include <SPI.h>

#include "circular_buffer.hpp"
#include "control_data.hpp"
#include "phy_system_driver.hpp"
#include "state_machine.hpp"
#include "system_data.hpp"

struct SystemStatus_
{
    /* data */

    uint32_t heart_beat_interval;
    bool is_heart_beat;
    uint32_t heart_beat_cnt;
    uint32_t ave_calc_time_for_main_task;
    uint32_t max_calc_time_for_main_task;

    SystemStatus_() {
        heart_beat_interval = 1000;  // ms
        heart_beat_cnt = 0;
        is_heart_beat = false;
        ave_calc_time_for_main_task = 0;  // ms
        max_calc_time_for_main_task = 0;  // ms
    }
};

struct ClipBoard_RecvCmd
{
    // CMD_COMM_START, CMD_COMM_STOP
    bool is_heart_beat;
    uint32_t heart_beat_interval;
};

class SystemManager {
private:
    /* data */
    M5Canvas* canvas;
    M5Module_LAN* LAN;
    IPAddress local_ip;
    uint32_t udp_packet_size;
    EthernetUDP* Udp;
    uint32_t recv_port;
    IPAddress destination_ip;
    uint32_t send_port;

    /*
     * Status and Config
     */
    SystemStatus system_status;
    SystemConfig system_config;

    CircularBuffer<u_int8_t, 10> recv_cmd_buffer;
    CircularBuffer<PhySystemPacket, 10> send_packet_buffer;
    ClipBoard_RecvCmd recv_cmd_clipboard;

    bool is_init_canvas = false;
    bool is_init_lan = false;

    void heart_beat();
    bool display_heart_beat = false;

public:
    SystemManager(/* args */) {
        // System
        system_status.state_machine = StateMachine::STATE_INIT;
    }
    ~SystemManager() {
    }
    /* External Data !!!Read Only!!!*/
    // Control System
    ControlConfig control_config_;
    ControlStatus control_status_;

    // void set_lan(M5Module_LAN* LAN_);
    // void set_udp(EthernetUDP* Udp_, uint32_t recv_port, uint32_t send_port,
    //              IPAddress destination_ip_);
    void set_canvas(M5Canvas* canvas_);
    void set_lan_info(IPAddress local_ip_, IPAddress destination_ip_,
                      uint32_t recv_port, uint32_t send_port);
    void set_calc_time_for_main_task(uint32_t ave_calc_time,
                                     uint32_t max_calc_time);
    // System
    StateMachine get_state_machine();
    void management_process(uint8_t cmd_);
    void set_state_machine(StateMachine state_machine) {
        system_status.state_machine = state_machine;
    }
    // EMS
    void set_emergency_stop_for_control_task(bool em_stop);

    // Communication for conrtol tasks
    void add_control_config_age();
    void add_control_status_age();
    void reset_control_config_age();
    void reset_control_status_age();

    // UDP
    uint32_t get_udp_packet_size();
    bool get_udp_heart_beat_status();
    void set_udp_packet(uint8_t* packetBuffer);
    void get_udp_heart_beat_packet(uint8_t* packetBuffer);
    bool get_send_packet(PhySystemPacket& packetBuffer);
    uint8_t get_recv_cmd();
    // Display
    void updateDisplay();
};