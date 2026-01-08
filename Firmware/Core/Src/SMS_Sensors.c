/*
 * SMS_Sensors.c
 *
 *  Created on: Dec 01, 2025
 *      Author: KienNT
 */

#include "SMS_Sensors.h"
#include <stdio.h>
#include <string.h>

static char msg[64];


SMS_Data_t read_SMS_V1(ADC_HandleTypeDef *hadc){
	HAL_ADC_Start(hadc);
	SMS_Data_t sms_data;
	if (HAL_ADC_PollForConversion(hadc, 100) == HAL_OK)
		{
			sms_data.adc_raw = HAL_ADC_GetValue(hadc);
		}
	HAL_ADC_Stop(hadc);

	uint16_t dry_val = 4000;
	uint16_t wet_val = 1500;

	if(sms_data.adc_raw > dry_val) sms_data.adc_raw = dry_val;
	if(sms_data.adc_raw < wet_val) sms_data.adc_raw = wet_val;

	sms_data.moist_pct = 100 - ((sms_data.adc_raw - wet_val) * 100 / (dry_val - wet_val));

	return sms_data;
}

void send_SMS_V1(UART_HandleTypeDef *huart, SMS_Data_t value_send){
	sprintf(msg, "ADC: %d - Do am: %d %%\r\n", value_send.adc_raw, value_send.moist_pct);
	HAL_UART_Transmit(huart, (uint8_t*)msg, strlen(msg), 100);
}
