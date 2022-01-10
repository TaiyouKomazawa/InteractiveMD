/*
 * PID.hpp
 *
 *  Created on: Nov 8, 2021
 *      Author: Taiyou Komazawa
 */

#ifndef INC_PID_HPP_
#define INC_PID_HPP_

#include <stdint.h>

class PID
{
public:
	typedef struct ControllerVariableType
	{
		double target;
		double feedback;
		double output;
	}ctrl_variable_t;
	typedef struct ControllerParameterType
	{
		float 	kp = 0.1;
		float 	ki = 0;
		float 	kd = 0;
		float 	forward_gain = 0;
		bool 	dpi_mode = true;
	}ctrl_param_t;

	PID(ctrl_variable_t *cv, ctrl_param_t *cp);
	~PID();

	void reset();

	bool step(double dt);

private:
	ctrl_variable_t *cv_;
	ctrl_param_t *cp_;

	double last_diff_, integral_;
	double last_feedback_;
};


#endif /* INC_PID_H_ */
