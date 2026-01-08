/*
 * protocol.c
 *
 *  Created on: Jan 8, 2026
 *      Author: Asus ROG
 */
#include <protocol.h>
#include "main.h"

extern uint8_t Node_assigned_slot = 0;
extern uint8_t is_joined = 0;
extern NodeState_t node_state;

//node xin id vào mạng
void Send_Join_Request(void) {
    JoinReq_Packet pkt;
    pkt.header = 0xAA;
    pkt.type = PKT_JOIN_REQ;

    // Lấy 4 byte cuối của Unique ID của STM32 (mỗi chip có 1 mã riêng)
    pkt.device_uuid = *(uint32_t*)(0x1FFFF7E8);

    // Gửi qua LoRa hoặc UART
    // LoRa_Send((uint8_t*)&pkt, sizeof(pkt));
}

// node gửi data thu thập
void Send_Sensor_Data(uint8_t id, float t_e, float h_e, float t_s, float h_s, float bat) {
    Data_Packet pkt;
    pkt.header = 0xAA;
    pkt.type = PKT_DATA;
    pkt.node_id = id;

    // Chuyển đổi dữ liệu sang số nguyên để truyền binary
    pkt.temp_env = (int16_t)(t_e * 100);
    pkt.hum_env = (uint16_t)(h_e * 100);
    pkt.temp_soil = (int16_t)(t_s * 100);
    pkt.hum_soil = (uint16_t)(h_s * 100);
    //pkt.battery = (uint16_t)(bat * 100);

    // Gửi gói tin 13 bytes
    // LoRa_Send((uint8_t*)&pkt, sizeof(pkt));
}

void handle_sync_packet(uint8_t *buffer) {
    Sync_Packet *pkt = (Sync_Packet *)buffer;

    if (pkt->header == 0xAA && pkt->type == 0x01) {
        uint16_t current_cycle = pkt->cycle_id;

        if (is_joined == 0) {
            // Trạng thái chưa Join: Chuẩn bị gửi JOIN_REQ
            // Nên delay ngẫu nhiên 1-5 giây để tránh nghẽn mạch
            uint32_t random_delay = (HAL_GetTick() % 5000);
            node_state = STATE_WAIT_TO_JOIN;
            set_timer_callback(random_delay, Send_Join_Request);
        } else {
            // Đã Join: Tính toán thời điểm gửi DATA dựa trên Slot
            // Ví dụ: Mỗi slot cách nhau 5 giây
            uint32_t wait_time = Node_assigned_slot * 5000;
            node_state = STATE_WAIT_MY_SLOT;
            //Khúc này nên delay để
            //set_timer_callback(wait_time, read_sensors_and_send_data);
        }
        printf("Nhan SYNC - Chu ky: %d\n", current_cycle);
    }
}

void handle_join_ack(uint8_t *buffer) {
    JoinAck_Packet *pkt = (JoinAck_Packet *)buffer;

    // Lấy UUID của chip STM32 này
    uint32_t my_uuid = *(uint32_t*)(0x1FFFF7E8);

    if (pkt->header == 0xAA && pkt->type == 0x03) {
        if (pkt->device_uuid == my_uuid) {
            // Chúc mừng! Node đã được Gateway chấp nhận
            Node_assigned_slot = pkt->assigned_slot;
            is_joined = 1;
            node_state = STATE_JOINED;

            // Có thể lưu my_assigned_slot vào Flash ở đây
            printf("Join thanh cong! Slot duoc cap: %d\n", Node_assigned_slot);
        }
    }
}
