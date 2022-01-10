/*
 * ParamMsgs.hpp
 *
 *  Created on: 2022/01/06
 *      Author: Taiyou Komazawa
 */

#ifndef INC_PARAMMSGS_HPP_
#define INC_PARAMMSGS_HPP_

#include <stdint.h>
#include <Message.hpp>

typedef struct CtrlParamsMsgType
{
	bool is_received;
	float kp;
	float ki;
	float kd;
}ctrl_param_msg_t;

typedef sb::DuplexMessage<ctrl_param_msg_t> CtrlParamMsg;

typedef struct CtrlInitMsgType
{
	bool is_received;
	uint16_t encoder_cpr;
	float gear_ratio;
	float max_rps;
}ctrl_init_msg_t;

typedef sb::DuplexMessage<ctrl_init_msg_t> CtrlInitMsg;

#endif /* INC_PARAMMSGS_HPP_ */
