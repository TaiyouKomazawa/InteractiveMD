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

/**
 * @brief 回転指令値を扱うパケット構造体型
 */
typedef struct ControlCommandMsgType
{
	/* フレーム数(データの前後を示す変数) */
	uint32_t frame_id;
	/* 回転速度指令値[rps](0:モータ1, 1:モータ2) */
	float command[2];
}ctrl_cmd_msg_t;

/**
 * @typedef 回転指令を扱うメッセージ型
 */
typedef sb::Message<ctrl_cmd_msg_t> CtrlCmdMsg;

/**
 * @brief モータの状態を扱うパケット構造体型
 */
typedef struct ControlFeedbackMsgType
{
	/* フレーム数(データの前後を示す変数) */
	uint32_t frame_id;
	/* 現在の回転角度(回転数)[rev](0:モータ1, 1:モータ2) */
	float angle[2];
	/* 現在の回転速度[rps](0:モータ1, 1:モータ2) */
	float velocity[2];
	/* 現在の電流値[mA](0:モータ1, 1:モータ2) */
	int16_t current[2];
}ctrl_feed_msg_t;

/**
 * @typedef モータの状態を扱うメッセージ型
 */
typedef sb::Message<ctrl_feed_msg_t> CtrlFeedMsg;

#endif /* INC_CTRLMSGS_HPP_ */
