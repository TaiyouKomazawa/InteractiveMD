/*
 * IMDConfigMsgs.hpp
 *
 *  Created on: Jan 7, 2022
 *      Author: Taiyou komazawa
 */

#ifndef INC_IMDCONFIGMSGS_HPP_
#define INC_IMDCONFIGMSGS_HPP_

#include <stdint.h>
#include <Message.hpp>

namespace IMD
{

enum{
	STATE_NOEEROR		= 0,
	STATE_TIMEOUT		= 1,
};

enum{
	CMD_NONE			= 0,
	CMD_GET_STATE		= 1,
	CMD_RESET 			= 255,
};

typedef struct SysCmdMsgType{
	bool is_received;
	uint8_t command;
}sys_cmd_msg_t;

typedef struct SysStateMsgType{
	bool is_received;
	uint8_t state;
}sys_state_msg_t;

typedef sb::DuplexMessage<sys_cmd_msg_t, sys_state_msg_t> DevMsg;
typedef sb::DuplexMessage<sys_state_msg_t, sys_cmd_msg_t> CtrlrMsg;

};
#endif /* INC_IMDCONFIGMSGS_HPP_ */
