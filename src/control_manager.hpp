#pragma once

#include <map>
#include <string>

#include "circular_buffer.hpp"
#include "control_data.hpp"
#include "driver/twai.h"
#include "xiaomi_cybergear_driver.hpp"

class ControlManager {
private:
    /*
     * Status and Config
     */
    ControlConfig control_config_;
    ControlStatus control_status_;

    CircularBuffer<u_int8_t, 10> recv_sys_cmd_buffer;
    ControlSysCmd sys_cmd_;
    ControlCmd control_cmd_;

    // CAN and motor driver(private)
    twai_message_t receive_can_packet;
    twai_message_t send_can_packet;
    CircularBuffer<twai_message_t, 10> receive_can_packet_buffer;
    CircularBuffer<twai_message_t, 10> send_can_packet_buffer;
    uint8_t rx_can_cnt;
    void update_cybergear_status(twai_message_t& rx_msg);
    XiaomiCyberGearDriver cybergear_driver;
    std::map<uint8_t, XiaomiCyberGearStatus> motor_status;
    void init_motor_status() {
        motor_status.clear();
        motor_status[0x10] = XiaomiCyberGearStatus();
        motor_status[0x10].can_id = 0x10;
    }
    void get_motor_id(uint8_t can_id);
    void enable_motor(uint8_t can_id);
    void stop_motor(uint8_t can_id);
    void change_can_id(uint8_t can_id, uint8_t new_can_id);
    void set_mechanical_position_to_zero(uint8_t can_id);
    void set_pos_mode(uint8_t can_id);
    void set_pos_kp(uint8_t can_id, float pos_kp);
    void set_pos_ref(uint8_t can_id, float pos_ref);
    void read_ctrl_mode(uint8_t can_id);
    void dump_cybergear_params(uint8_t can_id);
    // ------------------------------------------------
public:
    ControlManager(/* args */) {
        control_status_.state_machine = StateMachine::STATE_INIT;
    }
    ~ControlManager() {
    }

    void init();
    void sys_cmd_process();
    void control_process();

    void set_emergency_stop(bool em_stop);
    void set_state_machine(StateMachine state_machine);
    StateMachine get_state_machine() {
        return control_status_.state_machine;
    }
    // ------------------------------------------------

    /* CAN and motor driver(public)
     * 51: init_twai
     * 52: send_can_packet_task
     * 53: recv_can_packet_task
     * 54: set_receive_packet
     * 55: set_send_packet
     */
    void init_twai(uint8_t tx_num, uint8_t rx_num);
    void send_can_packet_task(const twai_message_t& packet);
    void recv_can_packet_task(twai_message_t& packet);
    void set_receive_packet(const twai_message_t& packet);
    void set_send_packet(const twai_message_t& packet);
    void init_motor_driver() {
        cybergear_driver.set_master_can_id(0x00);
        init_motor_status();
    }

    /* Config and status functions
     * 91: get_config_data      // Get config data
     * 92: get_status_data      // Get status data
     */
    bool get_config_data(ControlConfig& config);  // 91
    bool get_status_data(ControlStatus& status);  // 92
    // ------------------------------------------------

    /* Control system cmd
     * 事前にsys_cmd_にコマンドをセットしておく
     * その後、set_sys_cmd()を呼び出すことで、コマンドを送信する
     * 101: set_sys_cmd
     * 102-199: 予約
     */
    void set_sys_cmd(uint8_t cmd_) {  // 101
        recv_sys_cmd_buffer.push(cmd_);
    }
    void set_dummy_config_data(uint8_t cmd_data) {
        sys_cmd_.dummy_control_config_data = cmd_data;
    }
    void set_dummy_status_data(uint8_t cmd_data) {
        sys_cmd_.dummy_control_status_data = cmd_data;
    }
    // ------------------------------------------------

    /* Control cmd
     *
     */
};