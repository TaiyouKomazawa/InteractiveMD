/*
 * CurrentSensor.cpp
 *
 *  Created on: Jan 4, 2022
 *      Author: Taiyou Komazawa
 */

#include <CurrentSensor.hpp>

#include <string.h>

CurrentSensor::CurrentSensor(ADC_HandleTypeDef *hadcx, uint8_t num_of_unit, float gain, uint16_t offset_count)
:	hadcx_(hadcx),
	num_of_unit_(num_of_unit),
	gain_(gain)
{
	adc_data_ = new adc_data_t[num_of_unit_];
	adc_data_raw_ = new uint16_t[num_of_unit_];
	memset(adc_data_raw_, 0, num_of_unit_);

	if(HAL_ADCEx_Calibration_Start(hadcx_, ADC_SINGLE_ENDED)!= HAL_OK)
		Error_Handler();
	hadcx_->DMA_Handle->Instance->CCR &= ~(DMA_IT_TC | DMA_IT_HT);
	if(HAL_ADC_Start_DMA(hadcx_, (uint32_t *)adc_data_raw_, num_of_unit_) != HAL_OK)
		Error_Handler();

	HAL_Delay(1000);
	adc_get_offset(offset_count);
}

CurrentSensor::~CurrentSensor()
{
	if(HAL_ADC_Stop_DMA(hadcx_) != HAL_OK)
			Error_Handler();
	delete[] adc_data_;
	delete[] adc_data_raw_;
}

float CurrentSensor::get_current(int unit)
{
	adc_data_[unit].data[0] = adc_data_[unit].data[1];
	adc_data_[unit].data[1] = adc_data_[unit].data[2];
	adc_data_[unit].data[2] = 3.3 * (get_raw_(unit) - adc_data_[unit].offset) * gain_;

	return (adc_data_[unit].data[2]+adc_data_[unit].data[1]+adc_data_[unit].data[0])/3.0;
}

void CurrentSensor::adc_get_offset(uint16_t offset_count)
{
	double *sum = new double[num_of_unit_];

	for(int i = 0; i < num_of_unit_; i++)
		sum[i] = 0;

	for(int i = 0; i < offset_count; i++){
		for(int j = 0; j < num_of_unit_; j++)
			sum[j] += get_raw_(j);
		HAL_Delay(10);
	}
	for(int i = 0; i < num_of_unit_; i++){
		adc_data_[i].offset = sum[i] / offset_count;
		for(int j = 0; j < 3; j++)
			adc_data_[i].data[j] = 0;
	}

	delete[] sum;
}

float CurrentSensor::get_raw_(uint8_t unit)
{
	return (float)adc_data_raw_[unit] / ADC_RESOLUTION;
}
