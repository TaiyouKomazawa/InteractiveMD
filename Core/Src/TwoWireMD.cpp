/*
 * TwoWireMD.cpp
 *
 *  Created on: Nov 13, 2021
 *      Author: Taiyou Komazawa
 */

#include "TwoWireMD.hpp"

TwoWireMD::TwoWireMD(TIM_HandleTypeDef *htim_pwm, uint16_t tim_pwm_ch,
		GPIO_TypeDef *dir_port, uint16_t dir_pin,
		bool inverse_dir)
: MotorDriver(),
  htim_pwm_(htim_pwm),
  tim_pwm_ch_(tim_pwm_ch),
  dir_port_(dir_port),
  dir_pin_(dir_pin),
  inverse_dir_(inverse_dir),
  limit_(htim_pwm->Init.Period)
{
	HAL_TIM_PWM_Start(htim_pwm_, tim_pwm_ch_);
	__HAL_TIM_SET_COMPARE(htim_pwm_, tim_pwm_ch_, 0);
	TwoWireMD::set(0);
}

TwoWireMD::~TwoWireMD()
{
	TwoWireMD::set(0);
}

void TwoWireMD::set(float power)
{
	bool dir = power > 0;
	TwoWireMD::set(abs((int)(power*limit_)), dir);
}

void TwoWireMD::set(unsigned int pwm, bool dir)
{
	HAL_GPIO_WritePin(dir_port_, dir_pin_, (GPIO_PinState)(dir^inverse_dir_));
	if(limit_ < pwm)
		pwm = limit_;
	__HAL_TIM_SET_COMPARE(htim_pwm_, tim_pwm_ch_, pwm);
}
