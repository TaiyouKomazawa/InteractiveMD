/*
 * QEI.cpp
 *
 *  Created on: Jan 2, 2022
 *      Author: Taiyou Komazawa
 */

#include <QEI.hpp>

QEI::QEI(TIM_HandleTypeDef *htim_enc, double scale)
:	htim_enc_(htim_enc),
	scale_(scale)
{
	if(HAL_TIM_Encoder_Start(htim_enc_, TIM_CHANNEL_ALL) != HAL_OK)
		Error_Handler();
	reset_count();
	last_angle_ = get_angle();
}

QEI::~QEI()
{
	if(HAL_TIM_Encoder_Stop(htim_enc_, TIM_CHANNEL_ALL) != HAL_OK)
		Error_Handler();
	reset_count();
}

void QEI::reset_count()
{
	count_ = 0;
	__HAL_TIM_GET_COUNTER(htim_enc_) = 0;
}

double QEI::get_angle()
{
	int16_t enc_buff = (int16_t)__HAL_TIM_GET_COUNTER(htim_enc_);
	__HAL_TIM_GET_COUNTER(htim_enc_) = 0;
	count_ += enc_buff;
	return count_ * scale_;
}

double QEI::get_velocity(double dt)
{
	if(dt){
		double vel = (get_angle() - last_angle_) / dt;
		last_angle_ = get_angle();
		return vel;
	}
	return 0;
}
