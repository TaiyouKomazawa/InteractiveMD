/*
 * MotorDriver.hpp
 *
 *  Created on: Nov 13, 2021
 *      Author: Taiyou Komazawa
 */

#ifndef INC_MOTORDRIVER_HPP_
#define INC_MOTORDRIVER_HPP_

class MotorDriver {
public:
	MotorDriver();
	virtual ~MotorDriver();

	virtual void set(float) = 0;
};

#endif /* INC_MOTORDRIVER_HPP_ */
