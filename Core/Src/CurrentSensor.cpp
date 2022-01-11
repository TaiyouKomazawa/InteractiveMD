/*
 * CurrentSensor.cpp
 *
 *  Created on: Jan 4, 2022
 *      Author: Taiyou Komazawa
 */

#include <CurrentSensor.hpp>

#include <string.h>

/**
 * @brief Construct a new Current Sensor:: Current Sensor object
 * 
 * @param[in] hadcx 		ADC_HandleTypeDef構造体ポインタ変数
 * @param[in] num_of_unit	同ADCモジュール内で使用する電流センサの数
 * @param[in] gain 			電圧->電流の変換値(電流センサ感度)[A/V]
 * @param[in] offset_count	オフセット値の積算回数
 */
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
	/* 処理が重くなるのでDMAの割り込みを無効 */
	hadcx_->DMA_Handle->Instance->CCR &= ~(DMA_IT_TC | DMA_IT_HT);
	if(HAL_ADC_Start_DMA(hadcx_, (uint32_t *)adc_data_raw_, num_of_unit_) != HAL_OK)
		Error_Handler();

	HAL_Delay(1000);
	adc_get_offset(offset_count);
}

/**
 * @brief Destroy the Current Sensor:: Current Sensor object
 */
CurrentSensor::~CurrentSensor()
{
	if(HAL_ADC_Stop_DMA(hadcx_) != HAL_OK)
			Error_Handler();
	delete[] adc_data_;
	delete[] adc_data_raw_;
}

/**
 * @brief 現在の電流値を取得
 * 
 * @param[in] unit どの電流センサの値を読むかを指定(Rank順に0から指定)
 * @return float 現在の電流値[A]
 */
float CurrentSensor::get_current(int unit)
{
	adc_data_[unit].data[0] = adc_data_[unit].data[1];
	adc_data_[unit].data[1] = adc_data_[unit].data[2];
	adc_data_[unit].data[2] = 3.3 * (get_raw_(unit) - adc_data_[unit].offset) * gain_;

	return (adc_data_[unit].data[2]+adc_data_[unit].data[1]+adc_data_[unit].data[0])/3.0;
}

/**
 * @brief 現在のセンサー電圧値を測定しオフセット値とする関数
 * 
 * @param[in] offset_count オフセット値の積算回数
 * @retval None
 */
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

/**
 * @brief 現在の生の電圧値を取得
 * 
 * @param[in] unit どの電流センサの値を読むかを指定(Rank順に0から指定)
 * @return float 現在の電圧値(0.0~1.0)
 */
float CurrentSensor::get_raw_(uint8_t unit)
{
	return (float)adc_data_raw_[unit] / ADC_RESOLUTION;
}
