/*
 * CurrentSensor.hpp
 *
 *  Created on: Jan 4, 2022
 *      Author: Taiyou Komazawa
 */

#ifndef INC_CURRENTSENSOR_HPP_
#define INC_CURRENTSENSOR_HPP_

#include "main.h"

class CurrentSensor
{
public:
	enum
	{
		ADC_RESOLUTION = 4096,
	};

	typedef struct ADCDataType
	{
		float data[3];
		float offset;
	}adc_data_t;

	CurrentSensor(ADC_HandleTypeDef *hadcx, uint8_t num_of_unit, float gain, uint16_t offset_count=100);

	~CurrentSensor();

	float get_current(int unit);

	void adc_get_offset(uint16_t offset_count);

private:

	float get_raw_(uint8_t unit);

	ADC_HandleTypeDef *hadcx_;
	uint8_t num_of_unit_;
	float gain_;

	adc_data_t *adc_data_;
	uint16_t *adc_data_raw_;
};



#endif /* INC_CURRENTSENSOR_HPP_ */
