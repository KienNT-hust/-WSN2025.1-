/*
 * DHT11_Sensors.c
 *
 *  Created on: Dec 04, 2025
 *      Author: KienNT
 */
#include "main.h"
#include "DHT11_Sensors.h"

#include <stdio.h>
#include <string.h>

static char msg_send[64];

#define DWT_DELAY_NEWBIE 0

void DHT11_Init_Timer(void) {
  // Mở khóa biến DWT
  if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  }

  // Reset bộ đếm
  DWT->CYCCNT = 0;

  // Bật bộ đếm
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t us) {
  uint32_t startTick = DWT->CYCCNT;
  // SystemCoreClock là biến có sẵn trong system_stm32f1xx.c (thường là 72000000)
  uint32_t delayTicks = us * (SystemCoreClock / 1000000);

  while (DWT->CYCCNT - startTick < delayTicks);
}
// --- HÀM RIÊNG TƯ (Helper Functions) ---
// Hàm đổi chân thành Output
static void Set_Pin_Output(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Đẩy kéo
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

// Hàm đổi chân thành Input
static void Set_Pin_Input(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP; // Kéo lên để không bị nhiễu
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

// Hàm delay us (copy từ phần 1 vào đây, nhớ include DWT header nếu cần)
extern void delay_us(uint32_t us); // Giả sử đã khai báo ở main hoặc 1 file common

// --- HÀM CHÍNH ---

DHT11_Data_t DHT11_Read(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    DHT11_Data_t result = {0, 0};
    uint8_t data[5] = {0};

    // --- GỬI START ---
    Set_Pin_Output(GPIOx, GPIO_Pin);
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, 0);
    HAL_Delay(20); // Tăng lên 20ms cho chắc (datasheet yêu cầu tối thiểu 18ms)
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, 1);
    delay_us(30);
    Set_Pin_Input(GPIOx, GPIO_Pin);

    // --- CHECK PHẢN HỒI (QUAN TRỌNG) ---
    // Kiểm tra xem cảm biến có kéo chân xuống thấp không?
    if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == 1) // Nếu vẫn cao -> Lỗi!
    {
        // Gán giá trị đặc biệt để biết lỗi: -1
        result.Humidity = -1.0;
        result.Temperature = -1.0;
        return result;
    }

    // Nếu xuống thấp rồi, chờ nó lên cao lại (80us thấp -> 80us cao)
    delay_us(80);
    if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == 0) // Nếu vẫn thấp -> Lỗi!
    {
        result.Humidity = -2.0; // Lỗi kẹt ở mức thấp
        return result;
    }

    // Chờ nốt đoạn mức cao còn lại để bắt đầu vào data
    delay_us(80);

    // --- ĐỌC DATA (Như cũ) ---
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 8; j++) {
            // Chờ bit bắt đầu (mức thấp 50us)
            while (!(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)));

            delay_us(40); // Đợi để kiểm tra mức cao

            if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)) {
                data[i] |= (1 << (7 - j));
                // Chờ cho hết bit 1 (xuống thấp lại)
                while (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin));
            }
        }
    }

    // --- TÍNH TOÁN ---
    if (data[4] == (data[0] + data[1] + data[2] + data[3])) {
        result.Humidity = (float)data[0] + (float)data[1] / 10.0;
        result.Temperature = (float)data[2] + (float)data[3] / 10.0;
    }
    // Nếu checksum sai, giữ nguyên 0.0 để biết là có đọc nhưng sai data

    return result;
}

//void send_DHT11_Data(UART_HandleTypeDef *huart, DHT11_Data_t value_send){
//	sprintf(msg_send, "Temp: %.1f C - Hum: %.1f %%\r\n", value_send.Temperature, value_send.Humidity);
//	HAL_UART_Transmit(huart, (uint8_t*)msg_send, strlen(msg_send), 100);
//}

void send_DHT11_Data(UART_HandleTypeDef *huart, DHT11_Data_t value_send){
    // 1. Khai báo bộ đệm ngay trong hàm (để tránh lỗi thiếu biến)
    char msg_send[64];

    // 2. Tách phần nguyên và phần thập phân
    // Ví dụ: 25.4 -> nguyen=25, thap_phan=4
    int temp_int = (int)value_send.Temperature;
    int temp_dec = (int)((value_send.Temperature - temp_int) * 10);

    int hum_int = (int)value_send.Humidity;
    int hum_dec = (int)((value_send.Humidity - hum_int) * 10);

    // 3. Dùng %d để in (Code này chạy trên mọi dòng chip)
    sprintf(msg_send, "Temp: %d.%d C - Hum: %d.%d %%\r\n", temp_int, temp_dec, hum_int, hum_dec);

    // 4. Gửi đi
    HAL_UART_Transmit(huart, (uint8_t*)msg_send, strlen(msg_send), 100);
}
