/*
 * CtrlrMsgs.hpp
 *
 *  Created on: Jan 4, 2022
 *      Author: Taiyou Komazawa
 */

#ifndef INC_CTRLMSGS_HPP_
#define INC_CTRLMSGS_HPP_

#include <stdint.h>
#include <Message.hpp>

typedef struct ControlCommandMsgType
{
	uint32_t frame_id;
	float command[2];
}ctrl_cmd_msg_t;

typedef sb::Message<ctrl_cmd_msg_t> CtrlCmdMsg;

typedef struct ControlFeedbackMsgType
{
	uint32_t frame_id;
	float angle[2];
	float velocity[2];
	int16_t current[2];
}ctrl_feed_msg_t;

typedef sb::Message<ctrl_feed_msg_t> CtrlFeedMsg;

#endif /* INC_CTRLMSGS_HPP_ */
