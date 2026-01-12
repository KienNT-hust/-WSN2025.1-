/*
 * protocol.h
 *
 *  Created on: Jan 8, 2026
 *      Author: Asus ROG
 */

#ifndef INC_PROTOCOL_H_
#define INC_PROTOCOL_H_

#include <stdint.h>
#include <stdio.h>
// Định nghĩa các loại gói tin
#define PKT_SYNC     0x01
#define PKT_JOIN_REQ  0x02
#define PKT_JOIN_ACK  0x03
#define PKT_DATA     0x04

typedef enum {
    STATE_IDLE,             // Trạng thái nghỉ chờ SYNC
    STATE_WAIT_TO_JOIN,     // Đang chờ hết thời gian ngẫu nhiên để gửi Join
    STATE_WAIT_MY_SLOT,      // Đã Join, đang chờ đến lượt (slot) của mình để gửi Data
    STATE_SENDING_DATA,      // Đang trong quá trình gửi dữ liệu
	STATE_JOINED, 			// joind mạng
    STATE_SLEEP             // Đi ngủ tiết kiệm pin
} NodeState_t;


extern volatile uint8_t Node_assigned_slot;
extern volatile uint8_t is_joined;
extern volatile NodeState_t node_state;
extern volatile uint32_t target_action_time; // Biến mốc thời gian hành động




// 1. Gói SYNC (4 Bytes)
typedef struct __attribute__((packed)) {
    uint8_t header;    // 0xAA byte đầu tiên ký hiệu của  1 gói tin / start of frame
    uint8_t type;      // 0x01
    uint16_t cycle_id; // ID chu kỳ
} Sync_Packet;

// 2. Gói JOIN_REQ (14 Bytes)
//typedef struct __attribute__((packed)) {
//    uint8_t header;      // 0xAA
//    uint8_t type;        // 0x02
//    uint32_t device_uuid0; // 4 bytes cuối của UID STM32
//    uint32_t device_uuid1;
//    uint32_t device_uuid2;
//} JoinReq_Packet;

typedef struct __attribute__((packed)) {
    uint8_t header;
    uint8_t type;
    uint8_t device_uuid[12]; // Chuyển từ uint32_t sang mảng 12 byte
} JoinReq_Packet;

// 3. Gói JOIN_ACK (7 Bytes)
typedef struct __attribute__((packed)) {
    uint8_t header;
    uint8_t type;
    uint8_t device_uuid[12];
    uint8_t assigned_id;
    uint8_t slot_time;
} JoinAck_Packet;

// 4. Gói DATA (11 Bytes)
typedef struct __attribute__((packed)) {
    uint8_t header;
    uint8_t type;
    uint8_t node_id;
    int16_t temp_env;
    uint16_t hum_env;
    int16_t temp_soil;
    uint16_t hum_soil;
//    uint8_t pin_level; // Thêm mức pin theo code Gateway mới
} Data_Packet;

//gói 1
void Send_Join_Request(void);
//gói 2
void Send_Sensor_Data(uint8_t id, float t_e, float h_e, float t_s, float h_s);
//gói 3
void handle_sync_packet(uint8_t *buffer);
//gói 4
void handle_join_ack(uint8_t *buffer);
#endif /* INC_PROTOCOL_H_ */
