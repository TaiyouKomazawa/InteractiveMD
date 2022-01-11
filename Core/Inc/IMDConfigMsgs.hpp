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

/**
 * @brief IMDのシステムコマンド用の名前空間
 */
namespace IMD
{
	enum{
		/* ステータス : エラーなし */
		STATE_NOEEROR		= 0,
		/* ステータス : タイムアウト */
		STATE_TIMEOUT		= 1,
	};

	enum{
		/* コマンド : なし */
		CMD_NONE			= 0,
		/* コマンド : 状態取得要求 */
		CMD_GET_STATE		= 1,
		/* コマンド : 制御器(再)起動 */
		CMD_RESET 			= 255,
	};

	/**
	 * @brief システムコマンドを扱うパケット構造体型
	 */
	typedef struct SysCmdMsgType{
		/* データが受信されたときの応答を確認する変数(true:受信した,false:受信できてないor送信データ) */
		bool is_received;
		/* 指定するコマンド */
		uint8_t command;
	}sys_cmd_msg_t;

	/**
 	 * @brief システムの状態を扱うパケット構造体型
	 */
	typedef struct SysStateMsgType{
		/* データが受信されたときの応答を確認する変数(true:受信した,false:受信できてないor送信データ) */
		bool is_received;
		/* 現在の状態 */
		uint8_t state;
	}sys_state_msg_t;

	/**
	 * @typedef デバイス(IMD)側のメッセージ型
	 */
	typedef sb::DuplexMessage<sys_cmd_msg_t, sys_state_msg_t> DevMsg;
	/**
	 * @typedef コントローラ(ホスト)側のメッセージ型
	 */
	typedef sb::DuplexMessage<sys_state_msg_t, sys_cmd_msg_t> CtrlrMsg;

};
#endif /* INC_IMDCONFIGMSGS_HPP_ */
