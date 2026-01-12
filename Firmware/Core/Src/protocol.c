#include <protocol.h>
#include "main.h"
#include "SX1278.h"
#include <DS18B20_Sensors.h>
#include <string.h>

extern SX1278_t SX1278;
volatile uint8_t Node_assigned_slot = 0;
volatile uint8_t is_joined = 0;
volatile uint8_t assigned_slot_time = 0;
volatile uint32_t target_action_time = 0;
volatile NodeState_t node_state = STATE_IDLE;

// 1. Gửi yêu cầu gia nhập mạng (14 bytes)
void Send_Join_Request(void) {
    JoinReq_Packet pkt;
    pkt.header = 0xAA;
    pkt.type = 0x02; // PKT_JOIN_REQ

    // Sao chép toàn bộ 12 byte UUID của chip STM32
    memcpy(pkt.device_uuid, (uint8_t*)0x1FFFF7E8, 12);

    SX1278_transmit(&SX1278, (uint8_t*)&pkt, sizeof(pkt), 1000);

    // Gateway gửi ACK dài 16 bytes, nên ta đợi nhận 16 bytes
    SX1278_receive(&SX1278, 16, 1000);
//    printf(">>> Đang gửi Join Request (96-bit UUID)...\r\n");
}

// 2. Gửi dữ liệu cảm biến (12 bytes)
void Send_Sensor_Data(uint8_t id, float t_e, float h_e, float t_s, float h_s) {
    Data_Packet pkt;
    pkt.header = 0xAA;
    pkt.type = 0x04; // PKT_DATA
    pkt.node_id = id;

    // Chuyển đổi sang số nguyên x100 để truyền nhị phân
    pkt.temp_env = (int16_t)(t_e * 100);
    pkt.hum_env = (uint16_t)(h_e * 100);
    pkt.temp_soil = (int16_t)(t_s * 100);
    pkt.hum_soil = (uint16_t)(h_s * 100);

    SX1278_transmit(&SX1278, (uint8_t*)&pkt, sizeof(pkt), 1000);

    // Quay lại chế độ nhận để đợi gói SYNC chu kỳ sau
    SX1278_receive(&SX1278, 16, 1000);
}

// 3. Xử lý gói đồng bộ thời gian (SYNC)
void handle_sync_packet(uint8_t *buffer) {
    if (buffer[0] == 0xAA && buffer[1] == 0x01) {
        uint32_t cycle_start_time = HAL_GetTick(); // Mốc 0ms của chu kỳ

        // Luôn bắt đầu đo DS18B20 ngay khi có SYNC để tránh Delay
        DS18B20_StartMeasure();

        if (is_joined == 0) {
            node_state = STATE_WAIT_TO_JOIN;
            // Gateway mở cửa Join từ giây thứ 5 đến 30
            target_action_time = cycle_start_time + 6000 + (HAL_GetTick() % 15000);
            printf(">>> Nhận SYNC: Sẽ gửi Join Request sau vài giây...\r\n");
        } else {
            node_state = STATE_WAIT_MY_SLOT;
            // Tính toán mốc gửi dựa trên số giây Gateway đã cấp
            // Ví dụ: assigned_slot_time = 30 -> gửi sau 30.5 giây (bù 0.5s sai số)
            target_action_time = cycle_start_time + (assigned_slot_time * 1000) + 500;
            printf(">>> Nhận SYNC: Đã đặt lịch gửi DATA tại giây thứ %d\r\n", assigned_slot_time);
        }
    }
}

// 4. Xử lý gói xác nhận gia nhập (JOIN_ACK)
void handle_join_ack(uint8_t *buffer) {
    JoinAck_Packet *pkt = (JoinAck_Packet *)buffer;
    uint8_t my_uuid[12];
    memcpy(my_uuid, (uint8_t*)0x1FFFF7E8, 12);

    // So sánh chuẩn xác 12 byte UUID nhận được với UUID của chip
    if (pkt->header == 0xAA && pkt->type == 0x03) {
        if (memcmp(pkt->device_uuid, my_uuid, 12) == 0) {
            Node_assigned_slot = pkt->assigned_id; // ID 1 hoặc 2
            assigned_slot_time = pkt->slot_time;   // Mốc giây (30 hoặc 40)
            is_joined = 1;
            node_state = STATE_JOINED;

            printf(">>> JOIN THÀNH CÔNG! ID: %d | Slot gửi: %d giây\r\n",
                    Node_assigned_slot, assigned_slot_time);
        } else {
            printf(">>> Cảnh báo: Nhận gói ACK nhưng UUID không khớp.\r\n");
        }
    }
}
