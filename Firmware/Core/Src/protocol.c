/*
 * protocol.c
 *
 *  Created on: Jan 8, 2026
 *      Author: Asus ROG
 */
#include <protocol.h>
#include "main.h"
#include "SX1278.h" // Thêm header LoR

 extern SX1278_t SX1278; // Lấy đối tượng đã khai báo từ main.c
 uint8_t Node_assigned_slot = 0;
 uint8_t is_joined = 0;
 NodeState_t node_state =  STATE_IDLE;
 uint32_t target_action_time = 0; // Định nghĩa biến thực tế ở đây

//node xin id vào mạng
void Send_Join_Request(void) {
    JoinReq_Packet pkt;
    pkt.header = 0xAA;
    pkt.type = PKT_JOIN_REQ;
    pkt.device_uuid = *(uint32_t*)(0x1FFFF7E8);

    // Bạn phải thực sự gửi nó đi qua LoRa ở đây
    extern SX1278_t SX1278;
    SX1278_transmit(&SX1278, (uint8_t*)&pkt, sizeof(pkt), 1000);

    SX1278_receive(&SX1278, 15, 1000);
}

// node gửi data thu thập
void Send_Sensor_Data(uint8_t id, float t_e, float h_e, float t_s, float h_s) {
    Data_Packet pkt;
    pkt.header = 0xAA;
    pkt.type = PKT_DATA;
    pkt.node_id = id;

    // Chuyển đổi dữ liệu sang số nguyên (x100) để truyền nhị phân
    pkt.temp_env = (int16_t)(t_e * 100);
    pkt.hum_env = (uint16_t)(h_e * 100);
    pkt.temp_soil = (int16_t)(t_s * 100);
    pkt.hum_soil = (uint16_t)(h_s * 100);

    // Gửi gói tin qua LoRa (timeout 1000ms)
    SX1278_transmit(&SX1278, (uint8_t*)&pkt, sizeof(pkt), 1000);
}

//void handle_sync_packet(uint8_t *buffer) {
//    Sync_Packet *pkt = (Sync_Packet *)buffer;
//
//    if (pkt->header == 0xAA && pkt->type == 0x01) {
//        uint16_t current_cycle = pkt->cycle_id;
//
//        if (is_joined == 0) {
//            // Trạng thái chưa Join: Chuẩn bị gửi JOIN_REQ
//            // Nên delay ngẫu nhiên 1-5 giây để tránh nghẽn mạch
//            uint32_t random_delay = (HAL_GetTick() % 5000);
//            node_state = STATE_WAIT_TO_JOIN;
//            set_timer_callback(random_delay, Send_Join_Request);
//        } else {
//            // Đã Join: Tính toán thời điểm gửi DATA dựa trên Slot
//            // Ví dụ: Mỗi slot cách nhau 5 giây
//            uint32_t wait_time = Node_assigned_slot * 5000;
//            node_state = STATE_WAIT_MY_SLOT;
//            //Khúc này nên delay để
//            //set_timer_callback(wait_time, read_sensors_and_send_data);
//        }
//        printf("Nhan SYNC - Chu ky: %d\n", current_cycle);
//    }
//}
void handle_sync_packet(uint8_t *buffer) {
    Sync_Packet *pkt = (Sync_Packet *)buffer;
    if (pkt->header == 0xAA && pkt->type == PKT_SYNC) {
        // Mỗi khi nhận SYNC, reset mốc thời gian của chu kỳ mới
        uint32_t cycle_start_time = HAL_GetTick();

        if (is_joined == 0) {
            // Nếu chưa có ID: Đợi ngẫu nhiên 1-5s rồi gửi JOIN_REQ
            node_state = STATE_WAIT_TO_JOIN;
            target_action_time = cycle_start_time + (HAL_GetTick() % 5000) + 1000;
        } else {
            // Nếu đã có ID: Tính toán thời điểm đến Slot của mình
            // Ví dụ: Slot 1 gửi ở giây thứ 30, Slot 2 giây thứ 40...
            node_state = STATE_WAIT_MY_SLOT;
            target_action_time = cycle_start_time + 30000 + (Node_assigned_slot * 10000);
        }
        DS18B20_StartMeasure();
        printf("DS18B20 starts converting...\r\n");
    }
}
void handle_join_ack(uint8_t *buffer) {
    JoinAck_Packet *pkt = (JoinAck_Packet *)buffer;

    // Lấy UUID của chip STM32 này
    uint32_t my_uuid = *(uint32_t*)(0x1FFFF7E8);
    printf("Nhan duoc goi loai: %d, UUID trong goi: %X, UUID cua toi: %X\n", pkt->type, pkt->device_uuid, my_uuid);
    if (pkt->header == 0xAA && pkt->type == 0x03) {
    	printf("Nhan ACK! UUID trong goi: %X | UUID cua toi: %X\r\n", pkt->device_uuid, my_uuid);
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
