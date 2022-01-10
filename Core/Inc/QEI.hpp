/*
 * QEI.hpp
 *
 *  Created on: Jan 2, 2022
 *      Author: Taiyou Komazawa
 */

#ifndef INC_QEI_HPP_
#define INC_QEI_HPP_

#include "main.h"

class QEI
{
public:
	QEI(TIM_HandleTypeDef *htim_enc, double scale=1.0);

	~QEI();

	void reset_count();

	double get_angle();

	double get_velocity(double dt);

private:
	TIM_HandleTypeDef *htim_enc_;
	volatile long count_;
	double scale_;
	double last_angle_;
};

#endif /* INC_QEI_HPP_ */
