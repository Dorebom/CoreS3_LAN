#include "control_manager.hpp"

void ControlManager::set_state_machine(StateMachine state_machine) {
    // Set Emergency Stop
    if ((control_status_.emergency_stop_switch_for_control_task == true) or
        (state_machine == StateMachine::STATE_EM_STOP)) {
        control_status_.state_machine = StateMachine::STATE_EM_STOP;
        // ここで、サーボオフ命令を1回だけ送る or control_processで1回だけ送る
    } else {
        // Set Stable
        if ((state_machine == StateMachine::STATE_STABLE) and
            (control_status_.error_cnt == 0) and
            (control_status_.state_machine == StateMachine::STATE_READY)) {
            control_status_.state_machine = StateMachine::STATE_STABLE;
        }
        // Set Ready
        else if ((state_machine == StateMachine::STATE_READY) and
                 (control_status_.error_cnt == 0)) {
            control_status_.state_machine = StateMachine::STATE_READY;
        }
    }
}

void ControlManager::sys_cmd_process() {
    static uint8_t act_cmd = 0;
    if (recv_sys_cmd_buffer.pop(act_cmd)) {
        switch ((ControlTaskCmd)act_cmd) {
            case ControlTaskCmd::UPDATE_DUMMY_CONFIG_DATA:
                control_config_.dummy_data = sys_cmd_.dummy_control_config_data;
                break;
            case ControlTaskCmd::UPDATE_DUMMY_STATUS_DATA:
                control_status_.dummy_data = sys_cmd_.dummy_control_status_data;
                break;
            case ControlTaskCmd::SERV_ON:
                if (control_status_.state_machine !=
                    StateMachine::STATE_EM_STOP)
                    control_status_.is_serv_on = true;
                break;
            case ControlTaskCmd::SERV_OFF:
                // 現在値を目標値にする or 停止状態になってることを確認する
                control_status_.is_serv_on = false;
                break;
            default:
                break;
        }
    }
}

void ControlManager::control_process() {
}

void ControlManager::init() {
    /* Init Config */
    control_config_.dummy_data = 0;

    control_status_.config_data_status = DataStatus::DATA_READY;
    // ------------------------------------------------

    /* Init Status */
    control_status_.dummy_data = 0;
    // Servo control
    control_status_.is_serv_on = false;
    // Control Task System
    control_status_.warning_cnt = 0;
    control_status_.error_cnt = 0;
    control_status_.ave_calc_time_for_control_task = 0;
    control_status_.max_calc_time_for_control_task = 0;

    control_status_.status_data_status = DataStatus::DATA_READY;
    // ------------------------------------------------

    init_motor_driver();

    // Init Done.
    control_status_.state_machine = StateMachine::STATE_READY;
}

void ControlManager::set_emergency_stop(bool em_stop) {
    control_status_.emergency_stop_switch_for_control_task = em_stop;
    if (em_stop == true) {
        control_status_.state_machine = StateMachine::STATE_EM_STOP;
    }
}

bool ControlManager::get_config_data(ControlConfig& config) {
    // コンフィグデータを更新していなくても、データが取得できるようにする
    // だから、is_config_data_ready()でReadyかどうかを事前に確認してから取得する
    if (control_status_.config_data_status == DataStatus::DATA_NOT_READY) {
        return false;
    }
    config = control_config_;
    return true;
}
bool ControlManager::get_status_data(ControlStatus& status) {
    if (control_status_.status_data_status == DataStatus::DATA_NOT_READY) {
        return false;
    }
    status = control_status_;
    return true;
}

/*
 * CAN関連
 */
void ControlManager::init_twai(uint8_t tx_num, uint8_t rx_num) {
    gpio_num_t TX_GPIO_NUM = gpio_num_t(tx_num);
    gpio_num_t RX_GPIO_NUM = gpio_num_t(rx_num);

    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
}

void ControlManager::send_can_packet_task(const twai_message_t& packet) {
    twai_transmit(&packet, portMAX_DELAY);
    set_send_packet(packet);
}
void ControlManager::recv_can_packet_task(twai_message_t& packet) {
    twai_receive(&packet, portMAX_DELAY);
    update_cybergear_status(packet);
    rx_can_cnt++;
    set_receive_packet(packet);
}
void ControlManager::set_receive_packet(const twai_message_t& packet) {
    receive_can_packet = packet;
    receive_can_packet_buffer.push(packet);
}
void ControlManager::set_send_packet(const twai_message_t& packet) {
    send_can_packet = packet;
    send_can_packet_buffer.push(packet);
}
// ------------------------------------------------

/*
 * CyberGear Motor Driver
 */
void ControlManager::update_cybergear_status(twai_message_t& rx_msg) {
    uint8_t can_id;
    uint64_t serial_id;
    if (cybergear_driver.check_motor_id(rx_msg, can_id)) {
        // set_motor_id
        USBSerial.printf("Motor ID: %02X\r\n", can_id);
        cybergear_driver.update_motor_status(rx_msg, motor_status[can_id]);
    } else if (cybergear_driver.check_serial_response(rx_msg, can_id,
                                                      serial_id)) {
        // set_serial_id
        // not implemented yet
        motor_status[can_id].serial_id = serial_id;
    }
}

void ControlManager::get_motor_id(uint8_t can_id) {
    twai_message_t msg;
    cybergear_driver.get_motor_id(can_id, msg);
    send_can_packet_task(msg);
}

void ControlManager::enable_motor(uint8_t can_id) {
    twai_message_t msg;
    cybergear_driver.enable_motor(can_id, msg);
    send_can_packet_task(msg);
}
void ControlManager::stop_motor(uint8_t can_id) {
    twai_message_t msg;
    cybergear_driver.stop_motor(can_id, msg);
    send_can_packet_task(msg);
}
void ControlManager::change_can_id(uint8_t can_id, uint8_t new_can_id) {
    twai_message_t msg;
    cybergear_driver.change_can_id(can_id, new_can_id, msg);
    send_can_packet_task(msg);
}
void ControlManager::set_mechanical_position_to_zero(uint8_t can_id) {
    twai_message_t msg;
    cybergear_driver.set_mechanical_position_to_zero(can_id, msg);
    send_can_packet_task(msg);
}

void ControlManager::set_pos_mode(uint8_t can_id) {
    twai_message_t msg;
    cybergear_driver.set_position_mode(can_id, msg);
    send_can_packet_task(msg);
}

void ControlManager::set_pos_kp(uint8_t can_id, float pos_kp) {
    twai_message_t msg;
    cybergear_driver.set_pos_kp(can_id, pos_kp, msg);
    send_can_packet_task(msg);
}

void ControlManager::set_pos_ref(uint8_t can_id, float pos_ref) {
    twai_message_t msg;
    cybergear_driver.set_position_ref(can_id, pos_ref, msg);
    send_can_packet_task(msg);
}

void ControlManager::read_ctrl_mode(uint8_t can_id) {
    twai_message_t msg;
    cybergear_driver.get_ctrl_mode(can_id, msg);
    send_can_packet_task(msg);
}

void ControlManager::dump_cybergear_params(uint8_t can_id) {
    int sleep_time = 5;
    twai_message_t msg;
    // ctrl mode
    cybergear_driver.get_ctrl_mode(can_id, msg);
    send_can_packet_task(msg);
    delay(sleep_time);
    // pos ref
    cybergear_driver.get_pos_ref(can_id, msg);
    send_can_packet_task(msg);
    delay(sleep_time);
    // vel ref
    cybergear_driver.get_vel_ref(can_id, msg);
    send_can_packet_task(msg);
    delay(sleep_time);
    // iq ref
    cybergear_driver.get_iq_ref(can_id, msg);
    send_can_packet_task(msg);
    delay(sleep_time);
    // limit velocity
    cybergear_driver.get_limit_velocity(can_id, msg);
    send_can_packet_task(msg);
    delay(sleep_time);
    // limit torque
    cybergear_driver.get_limit_torque(can_id, msg);
    send_can_packet_task(msg);
    delay(sleep_time);
    // limit current
    cybergear_driver.get_limit_current(can_id, msg);
    send_can_packet_task(msg);
    delay(sleep_time);
    // pos kp
    cybergear_driver.get_pos_kp(can_id, msg);
    send_can_packet_task(msg);
    delay(sleep_time);
    // vel kp
    cybergear_driver.get_vel_kp(can_id, msg);
    send_can_packet_task(msg);
    delay(sleep_time);
    // vel ki
    cybergear_driver.get_vel_ki(can_id, msg);
    send_can_packet_task(msg);
    delay(sleep_time);
    // cur kp
    cybergear_driver.get_cur_kp(can_id, msg);
    send_can_packet_task(msg);
    delay(sleep_time);
    // cur ki
    cybergear_driver.get_cur_ki(can_id, msg);
    send_can_packet_task(msg);
    delay(sleep_time);
    // Cur flt
    cybergear_driver.get_cur_filter_gain(can_id, msg);
    send_can_packet_task(msg);
    delay(sleep_time);
    // mech pos
    cybergear_driver.get_mech_position(can_id, msg);
    send_can_packet_task(msg);
    delay(sleep_time);
    // mech vel
    cybergear_driver.get_mech_velocity(can_id, msg);
    send_can_packet_task(msg);
    delay(sleep_time);
    // rotation
    cybergear_driver.get_rotation(can_id, msg);
    send_can_packet_task(msg);
    delay(sleep_time);
    // vbus
    cybergear_driver.get_vbus(can_id, msg);
    send_can_packet_task(msg);
    delay(sleep_time);
    // iqf
    cybergear_driver.get_iqf(can_id, msg);
    send_can_packet_task(msg);
    delay(sleep_time);
}