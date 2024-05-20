#include "system_manager.hpp"

void SystemManager::heart_beat() {
    if (system_config.is_heart_beat) {
        system_status.heart_beat_cnt++;
        if (system_status.heart_beat_cnt >= system_config.heart_beat_interval) {
            system_status.heart_beat_cnt = 0;
            PhySystemPacket packet;
            packet.cmd_code = (uint32_t)phy_cmd::CMD_COMM_START;
            comm_data data_;
            data_.is_heart_beat = true;
            data_.heart_beat_interval = system_config.heart_beat_interval;
            memcpy(packet.data, &data_, sizeof(comm_data));
            Udp->beginPacket(destination_ip, send_port);
            Udp->write((uint8_t*)&packet, udp_packet_size);
            Udp->endPacket();
        }
    }
}

/*
void SystemManager::set_lan(M5Module_LAN* LAN_) {
    LAN = LAN_;
    local_ip = LAN->localIP();
    udp_packet_size = sizeof(phy_system_packet);
}

void SystemManager::set_udp(EthernetUDP* Udp_, uint32_t recv_port_,
                            uint32_t send_port_, IPAddress destination_ip_) {
    Udp = Udp_;
    recv_port = recv_port_;
    send_port = send_port_;
    destination_ip = destination_ip_;
    Udp->begin(recv_port);
    Udp->setTimeout(10);  // ms
}
*/

void SystemManager::set_canvas(M5Canvas* canvas_) {
    canvas = canvas_;
    is_init_canvas = true;
}

void SystemManager::set_lan_info(IPAddress local_ip_, IPAddress destination_ip_,
                                 uint32_t recv_port_, uint32_t send_port_) {
    local_ip = local_ip_;
    destination_ip = destination_ip_;
    recv_port = recv_port_;
    send_port = send_port_;
    is_init_lan = true;
}

void SystemManager::set_calc_time_for_main_task(uint32_t ave_calc_time,
                                                uint32_t max_calc_time) {
    system_status.ave_calc_time_for_main_task = ave_calc_time;
    system_status.max_calc_time_for_main_task = max_calc_time;
}

StateMachine SystemManager::get_state_machine() {
    return system_status.state_machine;
}

void SystemManager::management_process(uint8_t cmd_) {
    switch (cmd_) {
        case (uint8_t)phy_cmd::CMD_COMM_START: {
            system_config.is_heart_beat = recv_cmd_clipboard.is_heart_beat;
            system_config.heart_beat_interval =
                recv_cmd_clipboard.heart_beat_interval;
        } break;
        case (uint8_t)phy_cmd::CMD_COMM_STOP: {
            system_config.is_heart_beat = false;
            system_config.heart_beat_interval = 0;
        } break;
        case (uint8_t)phy_cmd::CMD_EM_SYS_STOP: {
            system_status.state_machine = StateMachine::STATE_EM_STOP;
        } break;
        case (uint8_t)phy_cmd::CMD_EM_CTRL_STOP: {
            system_status.state_machine = StateMachine::STATE_READY;
        } break;
        default:
            break;
    }
}

void SystemManager::set_emergency_stop_for_control_task(bool em_stop) {
    system_status.emergency_stop_switch_for_control_task = em_stop;
}

// Communication for conrtol tasks
void SystemManager::add_control_config_age() {
    if (system_status.control_config_age < 255)
        system_status.control_config_age++;
}
void SystemManager::add_control_status_age() {
    if (system_status.control_status_age < 255)
        system_status.control_status_age++;
}
void SystemManager::reset_control_config_age() {
    system_status.control_config_age = 0;
}
void SystemManager::reset_control_status_age() {
    system_status.control_status_age = 0;
}

uint32_t SystemManager::get_udp_packet_size() {
    return sizeof(PhySystemPacket);
}

bool SystemManager::get_udp_heart_beat_status() {
    return system_config.is_heart_beat;
}

void SystemManager::set_udp_packet(uint8_t* packetBuffer) {
    PhySystemPacket* packet = (PhySystemPacket*)packetBuffer;

    // Write clip board
    switch ((phy_cmd)packet->cmd_code) {
        case phy_cmd::CMD_COMM_START: {
            comm_data* data_ = (comm_data*)packet->data;
            system_config.is_heart_beat = data_->is_heart_beat;
            system_config.heart_beat_interval = data_->heart_beat_interval;
        } break;
        case phy_cmd::CMD_COMM_STOP: {
            comm_data* data_ = (comm_data*)packet->data;
            system_config.is_heart_beat = data_->is_heart_beat;
            system_config.heart_beat_interval = data_->heart_beat_interval;
        } break;
        case phy_cmd::CMD_COMM_CHANGE: {
            // Do something
        } break;
        case phy_cmd::CMD_READ_SYS_CONFIG: {
            PhySystemPacket packet_;
            packet_.cmd_code = (uint32_t)phy_cmd::CMD_READ_SYS_CONFIG;
            packet_.time_stamp = millis();
            SystemConfig* data_ = (SystemConfig*)packet_.data;
            data_->is_heart_beat = system_config.is_heart_beat;
            data_->heart_beat_interval = system_config.heart_beat_interval;
            send_packet_buffer.push(packet_);
        } break;
        case phy_cmd::CMD_EM_SYS_STOP: {
            // Do something
        } break;
        case phy_cmd::CMD_EM_CTRL_STOP: {
            // Do something
        } break;
        default: {
            // Do something
        } break;
    }

    recv_cmd_buffer.push(packet->cmd_code);
}

void SystemManager::get_udp_heart_beat_packet(uint8_t* packetBuffer) {
    PhySystemPacket* packet = (PhySystemPacket*)packetBuffer;
    packet->cmd_code = (uint32_t)phy_cmd::CMD_COMM_START;
    packet->time_stamp = millis();
}

bool SystemManager::get_send_packet(PhySystemPacket& packetBuffer) {
    if (send_packet_buffer.size() > 0) {
        send_packet_buffer.pop(packetBuffer);
        return true;
    }
    return false;
}

uint8_t SystemManager::get_recv_cmd() {
    uint8_t cmd_ = 0;
    recv_cmd_buffer.pop(cmd_);
    return cmd_;
}

void SystemManager::updateDisplay() {
    if (!is_init_canvas || !is_init_lan) {
        return;
    }

    canvas->clear();
    // canvas->setColorDepth(1);  // mono color
    // canvas->createSprite(CoreS3.Display.width(), CoreS3.Display.height());
    // canvas->setPaletteColor(1, GREEN);
    // canvas->setTextScroll(false);

    canvas->setCursor(0, 0);

    canvas->startWrite(true);
    if (display_heart_beat) {
        canvas->printf("System Manager ^-^-^- \r\n");
        display_heart_beat = false;
    } else {
        canvas->printf("System Manager ------ \r\n");
        display_heart_beat = true;
    }

    if (system_config.is_heart_beat) {
        canvas->printf("UDP Heart Beat: ON\r\n");
    } else {
        canvas->printf("UDP Heart Beat: OFF\r\n");
    }
    canvas->printf("MyIP:%s_%d\r\n", local_ip.toString().c_str(), recv_port);
    canvas->printf("D_IP:%s_%d\r\n", destination_ip.toString().c_str(),
                   send_port);
    canvas->printf("CalcTime:%d us, %d us\r\n",
                   system_status.ave_calc_time_for_main_task,
                   system_status.max_calc_time_for_main_task);
    if (system_status.emergency_stop_switch_for_control_task) {
        canvas->printf("EMS:ON\r\n");
    } else {
        canvas->printf("EMS:OFF\r\n");
    }
    canvas->pushSprite(0, 0);
    canvas->endWrite();
}
