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

/**
 * @brief 制御パラメータを扱うパッケージ構造体型
 */
typedef struct CtrlParamsMsgType
{
	/* データが受信されたときの応答を確認する変数(true:受信した,false:受信できてないor送信データ) */
	bool is_received;
	/* 比例制御ゲイン */
	float kp;
	/* 積分制御ゲイン */
	float ki;
	/* 微分制御ゲイン */
	float kd;
}ctrl_param_msg_t;

/**
 * @typedef 制御パラメータを扱うメッセージ型
 */
typedef sb::DuplexMessage<ctrl_param_msg_t> CtrlParamMsg;

/**
 * @brief 制御初期パラメータを扱うパッケージ構造体型
 */
typedef struct CtrlInitMsgType
{
	/* データが受信されたときの応答を確認する変数(true:受信した,false:受信できてないor送信データ) */
	bool is_received;
	/* エンコーダのCPR(count par revolution)単相パルス数(PPR)の４倍 */
	uint16_t encoder_cpr;
	/* ギア比(分子は1で固定 例 50:1->gear_ratio=50)(エンコーダがモータ出力軸側にある場合gear_ratio=1) */
	float gear_ratio;
	/* モータの(出力軸の)定格回転速度[rps] */
	float max_rps;
}ctrl_init_msg_t;

/**
 * @brief 制御初期パラメータを扱うメッセージ型
 */
typedef sb::DuplexMessage<ctrl_init_msg_t> CtrlInitMsg;

#endif /* INC_PARAMMSGS_HPP_ */
