
/**
 * @file WebServer.ino
 * @author S.Azarashi (dorebom.b@gmail.com)
 * @brief CyberGearの制御システムのメインプログラム
 * @version 0.1
 * @date 2023-10-26
 *
 * @details
 * このプログラムは、CyberGearの制御システムのメインプログラムです。
 * このプログラムは、M5Stack
 * CoreS3を使用して、CyberGearの制御システムを実装します。
 * このプログラムは、以下の機能を提供します。
 * 1. M5Stack CoreS3の入力信号を取得し、緊急停止ステータスを更新します。
 * 2. Control TaskのConfigとStatusデータを取得します。
 * 3. UDPパケットから外部コマンドを取得します。
 * 4. ディスプレイを更新します。
 * 5. 管理プロセスを実行します。
 * 6. ループプロセスを実行します。
 * 7. UDPパケットを送信します。
 * 8. UDPハートビートパケットを送信します。
 * 9. システムの計算時間を測定します。
 * 10. システムの状態を更新します。
 * 11. システムの状態を表示します。
 * 12. システムの状態を管理します。
 * 13. システムの状態を制御します。
 * 14. システムの状態を初期化します。
 * 15. システムの緊急停止ステータスを設定します。
 * 16. システムのConfigデータの準備状態を確認します。
 * 17. システムのStatusデータの準備状態を確認します。
 * 18. システムのConfigデータを取得します。
 * 19. システムのStatusデータを取得します。
 *
 * @Hardwares: M5Core/Core2/CoreS3 + LAN Module 13.2
 * @Platform Version: Arduino M5Stack Board Manager v2.0.7
 * @Dependent Library:
 * M5_Ethernet: https://github.com/m5stack/M5-Ethernet
 * M5Unified: https://github.com/m5stack/M5Unified
 * M5GFX: https://github.com/m5stack/M5GFX
 */

#include <EthernetUdp.h>
#include <M5GFX.h>
#include <M5Module_LAN.h>
#include <M5Unified.h>
#include <SPI.h>

#include <chrono>

#include "control_manager.hpp"
#include "system_manager.hpp"

M5Canvas canvas(&CoreS3.Display);
M5Module_LAN LAN;
EthernetUDP Udp;

#define UDP_PORT_RECV 50001
#define UDP_PORT_SEND 50002

// M5Stack PIN
#define EMS_INPUT   8
#define TX_TWAI_NUM 17  // CAN TX pin
#define RX_TWAI_NUM 18  // CAN RX pin

#define MAIN_TASK_PRIO   10
#define UDP_RX_TASK_PRIO 8
#define CTRL_TASK_PRIO   9

#define THEME_COLOR 0x0760

// Caluculation time Data
uint32_t max_calc_time_for_main_task = 0;
uint32_t ave_calc_time_for_main_task = 0;

// UDP Data
uint8_t cs_pin;
uint8_t rst_pin;
uint8_t int_pin;

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x89};
IPAddress ip(192, 168, 8, 217);

IPAddress destination_ip(192, 168, 8, 116);
// END UDP Data

SystemManager sys_manager;
ControlManager ctrl_manager;

/* UDP Receive Task
 * 1. Receive UDP packet
 * 2. Parse UDP packet and set the packet data into the system manager
 */
static void udp_receive_task(void *arg) {
    int packet_size = 0;
    uint32_t packet_buffer_size = sys_manager.get_udp_packet_size();
    uint8_t packetBuffer[packet_buffer_size];
    while (1) {
        /* code */
        // system_manager.receive_udp_packet_task();

        packet_size = Udp.parsePacket();

        if (packet_size) {
            // read the packet into packetBufffer
            Udp.read(packetBuffer, packet_size);

            sys_manager.set_udp_packet(packetBuffer);
            // sys_manager.recv_packet(packetBuffer, packet_size);
            // sys_manager.recv_packet();
        }

        // sys_manager.recv_packet();
        // sys_manager.updateDisplay();

        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(NULL);
}

/* Main Task
 * 1. GET INPUT SIGNALS, Config Data, Status Data, and Command Data
 * ___1.1. Get M5Stack CoreS3 input signal
 * ___1.2. Get Control Task Config and Status Data
 * ___1.3. Get External Commands from UDP packet
 * 4. Update display
 * 5. Management Process
 * 6. Loop Process
 * ___6.1. Send UDP packet
 * ___6.2. Send UDP heart beat packet
 * 7. Mesure calcuration time
 */
static void main_task(void *arg) {
    // Initialize the task
    uint32_t time_interval = 1;                    // ms
    uint32_t display_time_interval = 250;          // ms
    uint32_t udp_heart_beat_time_interval = 1000;  // ms
    uint32_t reset_time_interval = 10000;          // ms

    uint8_t ems_input = 0;
    uint32_t time_cnt = 0;

    uint32_t heart_beat_interval =
        (uint32_t)((float)display_time_interval / (float)time_interval);  // ms
    if (heart_beat_interval == 0) {
        heart_beat_interval = 1;
    }

    uint32_t udp_heart_beat_interval =
        (uint32_t)((float)udp_heart_beat_time_interval /
                   (float)time_interval);  // ms

    uint32_t reset_time_cnt =
        (uint32_t)((float)reset_time_interval / (float)time_interval);  // ms

    std::chrono::system_clock::time_point start, end;  // 型は auto で可
    start = std::chrono::system_clock::now();          // 計測開始時間
    // 処理
    end = std::chrono::system_clock::now();  // 計測終了時間
    double elapsed =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start)
            .count();  // 処理に要した時間をミリ秒に変換

    PhySystemPacket *packet_;
    uint8_t packet_buffer[sys_manager.get_udp_packet_size()];
    packet_ = (PhySystemPacket *)packet_buffer;

    uint8_t recv_cmd = 0;

    // Main loop
    while (1) {
        // Mesure calcuration time
        start = std::chrono::system_clock::now();

        /* GET INPUT SIGNALS, Config Data, Status Data, and Command Data
         * 1. Get M5Stack CoreS3 input signal
         * 2. Get Control Task Config and Status Data
         * 3. Get External Commands from UDP packet
         */

        // 1. Get M5Stack CoreS3 input signal
        // Get EMS input signal and update emergency stop status
        // 説明：M5Stack CoreS3のGPIO信号を取得し、緊急停止ステータスを更新する
        // 要改善ポイント：-
        CoreS3.update();  // Update CoreS3
        ems_input = digitalRead(EMS_INPUT);
        if (ems_input == 0) {
            sys_manager.set_emergency_stop_for_control_task(false);
            ctrl_manager.set_emergency_stop(false);
        } else {
            sys_manager.set_emergency_stop_for_control_task(true);
            ctrl_manager.set_emergency_stop(true);
        }
        // 2. Get Control Task Config and Status Data
        // 説明：Control TaskのConfigとStatusデータを取得する
        //       データが取得可能か確認し、取得可能な場合はデータを取得する
        // 要改善ポイント：更新されてない場合はデータを取得しないようにする
        if (ctrl_manager.get_config_data(sys_manager.control_config_)) {
            sys_manager.reset_control_config_age();
        } else {
            sys_manager.add_control_config_age();
        }
        if (ctrl_manager.get_status_data(sys_manager.control_status_)) {
            sys_manager.reset_control_status_age();
        } else {
            sys_manager.add_control_status_age();
        }
        // 3. Get External Commands from UDP packet
        // 説明：UDPパケットから外部コマンドを取得する
        // 要改善ポイント：-
        // Get command from UDP packet
        recv_cmd = sys_manager.get_recv_cmd();

        /* Update display
         *
         */
        if (time_cnt % heart_beat_interval == 0) {
            sys_manager.updateDisplay();
            time_cnt = 0;
        }

        // Management Process
        sys_manager.management_process(recv_cmd);

        // Loop Process
        if (sys_manager.get_state_machine() != StateMachine::STATE_EM_STOP) {
        }

        // END Loop Process

        // OLD ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓

        if (sys_manager.get_send_packet(*packet_)) {
            Udp.beginPacket(destination_ip, UDP_PORT_SEND);
            Udp.write(packet_buffer, sys_manager.get_udp_packet_size());
            Udp.endPacket();
        } else {
            if (sys_manager.get_udp_heart_beat_status() &&
                time_cnt % udp_heart_beat_interval == 0) {
                uint8_t packetBuffer[sys_manager.get_udp_packet_size()];

                sys_manager.get_udp_heart_beat_packet(packetBuffer);

                Udp.beginPacket(destination_ip, UDP_PORT_SEND);
                Udp.write(packetBuffer, sys_manager.get_udp_packet_size());
                Udp.endPacket();
            }
        }
        // Send UDP heart beat packet
        // test
        int i = 0;
        for (i = 0; i < 100; i++) {
            i++;
        }

        // END OLD ↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑

        end = std::chrono::system_clock::now();
        elapsed =
            std::chrono::duration_cast<std::chrono::microseconds>(end - start)
                .count();
        if (elapsed > max_calc_time_for_main_task) {
            max_calc_time_for_main_task = (uint32_t)elapsed;
        }
        ave_calc_time_for_main_task =
            ave_calc_time_for_main_task * 0.9 + (uint32_t)elapsed * 0.1;
        sys_manager.set_calc_time_for_main_task(ave_calc_time_for_main_task,
                                                max_calc_time_for_main_task);

        time_cnt++;
        if (time_cnt >= reset_time_cnt) {
            time_cnt = 0;
            max_calc_time_for_main_task = (uint32_t)elapsed;
        }
        vTaskDelay(pdMS_TO_TICKS(time_interval));
    }
    vTaskDelete(NULL);
}

/* Control Task
 * 1. Get CAN input signal
 * 2. System Loop Process
 * 3. Control Loop Process
 * ___3.1. Send CAN output signal
 */
static void ctrl_task(void *arg) {
    uint32_t time_interval = 10;  // ms

    ctrl_manager.init();

    while (1) {
        // GET INPUT SIGNALS, Config Data, Status Data, and Command Data
        // MainTaskからのsys_cmdデータ取得は、set_sys_cmd()を使用してすでに実施済み
        // MAinTaskからのctrl_cmdデータ取得は行わない。補間などの処理をsys_cmdで実施して内部処理したほうがいいため。
        // 1. Get CAN input signal

        // System Loop Process
        // ここで、control_processで毎周期実行する命令を作成する
        ctrl_manager.sys_cmd_process();
        // ------------------------------

        // Control Loop Process
        ctrl_manager.control_process();

        // 3.1. Send CAN output signal
        // ここで、CAN送信処理を実施する
        if (ctrl_manager.get_state_machine() != StateMachine::STATE_EM_STOP) {
            // CAN送信処理
            // 1. get CAN output signal
            // 2. send CAN output signal
        } else {
            // 緊急停止時の処理
            // 1. 緊急停止時のCAN出力信号を設定する
            // Position モードに変更
            // 現在値を目標値にする
            // 2. 緊急停止時のCAN出力信号を送信する
        }
        // ------------------------------

        vTaskDelay(pdMS_TO_TICKS(time_interval));
    }
    vTaskDelete(NULL);
}

void setup() {
    auto cfg = M5.config();
    CoreS3.begin(cfg);
    /*
     * Initialize
     * 1. IO
     * 2. Serial
     * 3. Canvas
     * 4. LAN
     * 5. CAN
     */

    // 1. Initialize the IO
    pinMode(EMS_INPUT, INPUT_PULLUP);

    // 2. Initialize the serial port
    // USBSerial.begin(115200);

    // 3. Initialize the canvas
    // canvas.setPsram(true);
    canvas.setPsram(false);
    canvas.setColorDepth(1);  // mono color
    canvas.createSprite((int32_t)CoreS3.Display.width(),
                        (int32_t)CoreS3.Display.height());
    canvas.setPaletteColor(1, GREEN);
    canvas.setTextScroll(false);
    canvas.setTextSize(2);
    // canvas.setTextColor(TFT_WHITE, TFT_BLACK);
    sys_manager.set_canvas(&canvas);
    // <--END Initialize the canvas

    // 4. Initialize the LAN module
    m5::board_t board = M5.getBoard();
    switch (board) {
        case m5::board_t::board_M5Stack: {
            cs_pin = 5;
            rst_pin = 0;
            int_pin = 35;
        } break;
        case m5::board_t::board_M5StackCore2: {
            cs_pin = 33;
            rst_pin = 0;
            int_pin = 35;
        } break;
        case m5::board_t::board_M5StackCoreS3: {
            cs_pin = 1;
            rst_pin = 0;
            int_pin = 10;
        } break;
    }
    SPI.begin(SCK, MISO, MOSI, -1);
    LAN.setResetPin(rst_pin);
    LAN.reset();
    LAN.init(cs_pin);
    LAN.begin(mac);
    while (LAN.hardwareStatus() == EthernetNoHardware) {
        Serial.println(
            "Ethernet shield was not found.  Sorry, can't run without "
            "hardware. :(");
        delay(1000);
    }
    while (LAN.linkStatus() == LinkOFF) {
        Serial.println("Ethernet cable is not connected.");
        delay(1000);
    }
    sys_manager.set_lan_info(LAN.localIP(), destination_ip, UDP_PORT_RECV,
                             UDP_PORT_SEND);
    // sys_manager.set_lan(&LAN);
    // sys_manager.set_udp(&Udp, UDP_PORT_RECV, UDP_PORT_SEND, destination_ip);

    // start UDP
    // Udp.setTimeout(10);  // ms
    Udp.begin(UDP_PORT_RECV);
    // <--END Initialize the LAN module

    // 5. Initialize the CAN
    ctrl_manager.init_twai(TX_TWAI_NUM, RX_TWAI_NUM);
    ctrl_manager.init_motor_driver();
    // <--END Initialize the CAN

    // Make thread for receiving UDP packet
    xTaskCreatePinnedToCore(main_task, "main_task", 2048, NULL, MAIN_TASK_PRIO,
                            NULL, 0);
    xTaskCreatePinnedToCore(ctrl_task, "control_task", 2048, NULL,
                            CTRL_TASK_PRIO, NULL, 0);
    xTaskCreatePinnedToCore(udp_receive_task, "udp_receive_task", 2048, NULL,
                            UDP_RX_TASK_PRIO, NULL, 0);  // tskNO_AFFINITY
}

void loop() {
}
