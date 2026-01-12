/*
 * DS18B20_Sensors.h
 * Created on: Jan 10, 2026
 * Author: Kien Nguyen
 */

#ifndef INC_DS18B20_SENSORS_H_
#define INC_DS18B20_SENSORS_H_

#include "main.h"
#include "delay_util.h"
// --- Định nghĩa mã lệnh DS18B20 ---
#define DS18B20_CMD_CONVERTTEMP       0x44
#define DS18B20_CMD_RSCRATCHPAD       0xBE
#define DS18B20_CMD_SKIPROM           0xCC

// --- Cấu hình chân kết nối (Kiên có thể sửa ở đây) ---
#define DS18B20_PORT  GPIOB
#define DS18B20_PIN   GPIO_PIN_10

// --- Khai báo các hàm ---

uint8_t DS18B20_Init(void);   // Kiểm tra sự hiện diện của cảm biến
float DS18B20_ReadTemp(void); // Đọc nhiệt độ (trả về số thực)
void DS18B20_StartMeasure(void);
float DS18B20_GetTempResult(void);
#endif /* INC_DS18B20_SENSORS_H_ */
