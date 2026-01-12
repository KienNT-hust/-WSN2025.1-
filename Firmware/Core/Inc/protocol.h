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


extern uint8_t Node_assigned_slot;
extern uint8_t is_joined;
extern NodeState_t node_state;
extern uint32_t target_action_time; // Biến mốc thời gian hành động




// 1. Gói SYNC (4 Bytes)
typedef struct __attribute__((packed)) {
    uint8_t header;    // 0xAA byte đầu tiên ký hiệu của  1 gói tin / start of frame
    uint8_t type;      // 0x01
    uint16_t cycle_id; // ID chu kỳ
} Sync_Packet;

// 2. Gói JOIN_REQ (6 Bytes)
typedef struct __attribute__((packed)) {
    uint8_t header;      // 0xAA
    uint8_t type;        // 0x02
    uint32_t device_uuid; // 4 bytes cuối của UID STM32
} JoinReq_Packet;

// 3. Gói JOIN_ACK (7 Bytes)
typedef struct __attribute__((packed)) {
    uint8_t header;       // 0xAA
    uint8_t type;         // 0x03
    uint32_t device_uuid;
    uint8_t assigned_slot;
} JoinAck_Packet;

// 4. Gói DATA (13 Bytes)
typedef struct __attribute__((packed)) {
    uint8_t header;    // 0xAA
    uint8_t type;      // 0x04
    uint8_t node_id;
    int16_t temp_env;  // x100 bỏ qua phần thập phân vd 25.26 -> 2526
    uint16_t hum_env;  // x100
    int16_t temp_soil; // x100
    uint16_t hum_soil; // x100
    //uint16_t bat;
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
