/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.cpp
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* 通信用ライブラリをインクルード */
#include <SerialBridge.hpp>
#include <STM32SPISerial.hpp>
/* 通信で使用する定義済みメッセージをインクルード */
#include <IMDConfigMsgs.hpp>
#include <CtrlMsgs.hpp>
#include <ParamMsgs.hpp>
/* モータドライバクラスをインクルード */
#include <MotorDriver.hpp>
#include <TwoWireMD.hpp>
/* 直交エンコーダクラスをインクルード */
#include <QEI.hpp>
/* 電流センサクラスをインクルード */
#include <CurrentSensor.hpp>
/* PID制御クラスをインクルード */
#include <PID.hpp>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define M1		0U  /* モータ1の要素 */
#define M2		1U  /* モータ2の要素 */

/** @brief モータ制御器の更新タイミング */
#define CTRL_INTERVAL 	(1.0 / 200)   //[sec]
/** @brief 電流センサ(ACS712ELCTR-05B)の感度 */
#define ACS712_V_2_I		(1 / 0.185)   //[A/V]
/** @brief 暴走検知用カウンタの許容値(この回数を超えると非常停止する) */
#define CTRL_FAULT_COUNT_LIMIT 16

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/** 
 * @brief tim_countから実時間[マイクロ秒]を取得する
 */
#define TIM_COUNT_2_US (tim_count * 8UL) //[us]
/** 
 * @brief tim_countに値xを入れる
 * @param[in] x 入力値
 */
#define TIM_COUNT_SET(x) (tim_count = x)
/**
 * @brief htim(ウォッチドックタイマ)をリセットする
 * @param[in] htim TIM_HandleTypeDefインスタンス
 */
#define TIM_WD_RESET(htim) (__HAL_TIM_SET_COUNTER(&(htim), 0))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
STM32SPISerial *dev;
SerialBridge *serial;

/** @brief システムコマンドを扱うメッセージ型 */
IMD::DevMsg dev_msg;
/** @brief モータ指示値を受け付けるメッセージ型 */
CtrlCmdMsg cmd;
/** @brief 制御フィードバック値を送信するメッセージ型 */
CtrlFeedMsg feed;
/** @brief 速度制御の制御パラメータを受け付けるメッセージ型 */
CtrlParamMsg vel_param[2];
/** @brief 電流制御の制御パラメータを受け付けるメッセージ型 */
CtrlParamMsg cur_param[2];
/** @brief 制御起動時に必要なパラメータを受け付けるメッセージ型 */
CtrlInitMsg init[2];

MotorDriver *md[2];
QEI *enc[2];
CurrentSensor *am;

PID *vel_ctrl[2];
PID::ctrl_variable_t v_vel[2];
PID::ctrl_param_t p_vel[2];

PID *cur_ctrl[2];
PID::ctrl_variable_t v_cur[2];
PID::ctrl_param_t p_cur[2];

/** @brief 制御器の起動状態を示す変数 true:動作中/false:停止中 */
volatile bool ctrlr_active = false;
/**
 * @brief 時間計測用カウンタ(TIM15)の割り込み数を数える変数
 *  1カウントつき8マイクロ秒が経過する
 */
volatile long tim_count = 0; //tim_count [8 * us]
/**
 * @brief 暴走検知用のカウンタ変数
 *  指示値とフィードバックの符号の不一致回数によりカウントアップする
 */
uint8_t ctrl_fault_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
static void MotorControl_WaitInitParams(int id, SerialBridge *serial,
		CtrlInitMsg &init);
static bool Check_ControlParameters(int id, SerialBridge *serial,
		PID::ctrl_param_t &cparam, CtrlParamMsg &param);
static void Controller_Init(void);
static void Controller_Deinit(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	MX_DMA_Init();
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_SPI1_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_USART2_UART_Init();
	MX_DMA_Init();
	MX_TIM15_Init();
	MX_TIM17_Init();
	/* USER CODE BEGIN 2 */
	/* MD enable ピンをLOW(モータへの出力を無効)にする */
	HAL_GPIO_WritePin(MD_EN_GPIO_Port, MD_EN_Pin, GPIO_PIN_RESET);

	/* STM32 SPIを有効化 受送信バッファ数:28bytes */
	dev = new STM32SPISerial(&hspi1, 28);
	serial = new SerialBridge(dev);

	serial->add_frame(0, &dev_msg);
	serial->add_frame(1, &cmd);
	serial->add_frame(2, &feed);
	serial->add_frame(3, &vel_param[M1]);
	serial->add_frame(4, &vel_param[M2]);
	serial->add_frame(5, &cur_param[M1]);
	serial->add_frame(6, &cur_param[M2]);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* データの更新の有無を出力するための変数 */
		static bool link_led_state = false;

		serial->update(); /* データの更新 */

		/**
		 *  システムコマンドが更新されたときの処理
		 */
		if (dev_msg.was_updated()) {
			uint8_t cmd = dev_msg.rx.command;
			dev_msg.tx.is_received = true; /* 受領確認をtrueにする */
			switch (cmd) {
			case IMD::CMD_RESET: /*** リセット指令を受け取った場合 ***/
				serial->write(0); /* 応答を送信 */
				serial->update();
				feed.data.frame_id = 0; /* フレーム数をリセット */
				Controller_Init(); /* 制御器起動処理を呼び出す */
				break;
			case IMD::CMD_GET_STATE: /*** 状態要求指令を受け取った場合 ***/
			default:
				serial->write(0); /* 応答を送信 */
				serial->update();
				break;
			};
			dev_msg.rx.command = IMD::CMD_NONE; /* コマンドをクリア */
		}
		/* モータ1の速度制御パラメータの更新をチェックする */
		if (Check_ControlParameters(3, serial, p_vel[M1], vel_param[M1])) {
			vel_ctrl[M1]->reset();
			cur_ctrl[M1]->reset();
		}
		/* モータ2の速度制御パラメータの更新をチェックする */
		if (Check_ControlParameters(4, serial, p_vel[M2], vel_param[M2])) {
			vel_ctrl[M2]->reset();
			cur_ctrl[M2]->reset();
		}
		/* モータ1の電流制御パラメータの更新をチェックする */
		if (Check_ControlParameters(5, serial, p_cur[M1], cur_param[M1]))
			cur_ctrl[M1]->reset();
		/* モータ2の電流制御パラメータの更新をチェックする */
		if (Check_ControlParameters(6, serial, p_cur[M2], cur_param[M2]))
			cur_ctrl[M2]->reset();

		/**
		 *  モータ指示値の更新があったときの処理
		 */
		if (cmd.was_updated() && ctrlr_active) { /* 指示値の更新があるかつ制御器がアクティブであるとき */
			if (feed.data.frame_id != cmd.data.frame_id) { /* 新規(フレーム数が違う)データであったとき */
				TIM_WD_RESET(htim17); /* 通信タイムアウト監視用カウンタ(TIM17)をリセット */
				feed.data.frame_id = cmd.data.frame_id; /* フレーム数を返信データにセット */
				/* モータ1の限界速度範囲内の速度指示値を制御器に渡す */
				if (cmd.data.command[M1] * cmd.data.command[M1]
						<= init[M1].rx.max_rps * init[M1].rx.max_rps)
					v_vel[M1].target = (double) cmd.data.command[M1];
				/* モータ2の限界速度範囲内の速度指示値を制御器に渡す */
				if (cmd.data.command[M2] * cmd.data.command[M2]
						<= init[M2].rx.max_rps * init[M2].rx.max_rps)
					v_vel[M2].target = (double) cmd.data.command[M2];
				feed.data.angle[M1] = (float) enc[M1]->get_angle(); /* モータ1の現在の回転角度[rev]を返信データに渡す */
				feed.data.angle[M2] = (float) enc[M2]->get_angle(); /* モータ2の現在の回転角度[rev]を返信データに渡す */
				feed.data.velocity[M1] = (float) enc[M1]->get_velocity(); /* モータ1の現在の回転速度[rps]を返信データに渡す */
				feed.data.velocity[M2] = (float) enc[M2]->get_velocity(); /* モータ2の現在の回転速度[rps]を返信データに渡す */
				feed.data.current[M1] = (int16_t) (v_cur[M1].feedback * 1E3);/* モータ1の現在の電流値[mA]を返信データに渡す */
				feed.data.current[M2] = (int16_t) (v_cur[M2].feedback * 1E3);/* モータ2の現在の電流値[mA]を返信データに渡す */
				serial->write(2); /* 返信データ(feed)を送信 */
				link_led_state = true; /* データの更新があったので点灯 */
			} else
				/* 新規データではなかったとき */
				link_led_state = false; /* データの更新がなかったので消灯 */
		}
		/**
		 * モータ制御処理
		 * - vel: モータの速度を制御する
		 * - cur: モータに流れる電流を制御する(トルク制御)
		 * *制御器の配置図*
		 *                V.output -> C.target
		 * V.target  +-----------+   +-----------+ C.output
		 *     ----->| Velocity  +-->| Current   +---> PWM
		 *           | Controller|   | Controller|
		 *           +---------+-+   +---------+-+
		 *             ^       |       ^       |
		 *             +-------+       +-------+
		 *              V.feedback      C.feedback
		 */
		if (ctrlr_active) { /* 制御器がアクティブであるとき */
			double tim = TIM_COUNT_2_US / 1E6; /* 時間計測用タイマ(TIM15)より現在のインターバル時間[秒]を取得*/
			if (tim >= CTRL_INTERVAL) { /* 現在のインターバルが制御周期以上のとき制御を実行する*/
				v_vel[M1].feedback = enc[M1]->get_velocity(tim); /* モータ1の速度制御器に現在の回転速度[rps]を渡す */
				vel_ctrl[M1]->step(tim); /* モータ1の速度制御器を更新 */
				v_vel[M2].feedback = enc[M2]->get_velocity(tim); /* モータ2の速度制御器に現在の回転速度[rps]を渡す */
				vel_ctrl[M2]->step(tim); /* モータ2の速度制御器を更新 */

				v_cur[M1].target = v_vel[M1].output; /* モータ1の速度制御出力を電流制御器の目標値に渡す */
				v_cur[M2].target = v_vel[M2].output; /* モータ2の速度制御出力を電流制御器の目標値に渡す */

				v_cur[M1].feedback = am->get_current(M1); /* モータ1の電流制御器に現在の電流値[A]を渡す */
				cur_ctrl[M1]->step(tim); /* モータ1の電流制御器を更新 */
				v_cur[M2].feedback = am->get_current(M2); /* モータ2の電流制御器に現在の電流値[A]を渡す */
				cur_ctrl[M2]->step(tim); /* モータ2の電流制御器を更新 */

				TIM_COUNT_SET(0); /* 時間計測用タイマ(TIM15)のカウンタをリセット */

				md[M1]->set(v_cur[M1].output); /* モータ1のモータドライバに出力を渡す */
				md[M2]->set(v_cur[M2].output); /* モータ2のモータドライバに出力を渡す */

				/* 符号が不一致かどうかをチェックする */
				if (v_vel[M1].target * v_vel[M1].feedback < 0
						|| v_vel[M2].target * v_vel[M2].feedback < 0)
					ctrl_fault_count++;
				else
					/* 問題なく収束するならばカウントリセット */
					ctrl_fault_count = 0;
				/* 許容値を超えて符号が一致しない場合暴走したと判断する */
				if (ctrl_fault_count > CTRL_FAULT_COUNT_LIMIT)
					Error_Handler();

				HAL_GPIO_WritePin(LINK_LED_GPIO_Port, LINK_LED_Pin,
						(GPIO_PinState) link_led_state);
			}
		} else
			TIM_COUNT_SET(0); /* 時間計測用タイマ(TIM15)のカウンタをリセット */

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1;
	PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_MultiModeTypeDef multimode = { 0 };
	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 2;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.SamplingTime = ADC_SAMPLETIME_181CYCLES_5;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_11;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_SLAVE;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1799;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief TIM15 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM15_Init(void) {

	/* USER CODE BEGIN TIM15_Init 0 */

	/* USER CODE END TIM15_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM15_Init 1 */

	/* USER CODE END TIM15_Init 1 */
	htim15.Instance = TIM15;
	htim15.Init.Prescaler = 71;
	htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim15.Init.Period = 7;
	htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim15.Init.RepetitionCounter = 0;
	htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_OC_Init(&htim15) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_OC_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM15_Init 2 */
	HAL_TIM_Base_Start_IT(&htim15); /* 割り込み処理を開始 */
	/* USER CODE END TIM15_Init 2 */

}

/**
 * @brief TIM17 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM17_Init(void) {

	/* USER CODE BEGIN TIM17_Init 0 */

	/* USER CODE END TIM17_Init 0 */

	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM17_Init 1 */

	/* USER CODE END TIM17_Init 1 */
	htim17.Instance = TIM17;
	htim17.Init.Prescaler = 7199;
	htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim17.Init.Period = 4999;
	htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim17.Init.RepetitionCounter = 0;
	htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim17) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_OC_Init(&htim17) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_OC_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM17_Init 2 */
	HAL_TIM_Base_Start_IT(&htim17); /* 割り込み処理を開始 */
	/* USER CODE END TIM17_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	/* DMA1_Channel2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	/* DMA1_Channel3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LINK_LED_GPIO_Port, LINK_LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, MD_EN_Pin | MD1_DIR_Pin | MD2_DIR_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : LINK_LED_Pin */
	GPIO_InitStruct.Pin = LINK_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LINK_LED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : MD_EN_Pin */
	GPIO_InitStruct.Pin = MD_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(MD_EN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : MD1_DIR_Pin MD2_DIR_Pin */
	GPIO_InitStruct.Pin = MD1_DIR_Pin | MD2_DIR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : MD_STATE_Pin */
	GPIO_InitStruct.Pin = MD_STATE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(MD_STATE_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
 * @brief Timer Period Elapsed Callback Function  
 *  時間経過後に呼ばれる割り込み関数(予約済み関数)
 * @param[in] htim TIM_HandleTypeDef構造体ポインタ
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim15) /* 時間計測の割り込み処理 */
		tim_count++;
	if (htim == &htim17) { /* 通信タイムアウト時の処理 */
		if (ctrlr_active) { /* 制御がアクティブである場合、制御器をリセット */
			v_vel[M1].target = 0;
			v_vel[M2].target = 0;
			vel_ctrl[M1]->reset();
			vel_ctrl[M2]->reset();
			v_cur[M1].target = 0;
			v_cur[M2].target = 0;
			cur_ctrl[M1]->reset();
			cur_ctrl[M2]->reset();
			md[M1]->set(0);
			md[M2]->set(0);
			/* 現在の状態をタイムアウトに指定 */
			dev_msg.tx.state = IMD::STATE_TIMEOUT;
		}
	}
}

/**
 * @brief 制御に必要な初期パラメータの受信を待つ関数
 * 
 * @param[in] id     メッセージの登録先
 * @param[in] serial 使用するSerialBridgeクラスポインタ
 * @param[out] init  参照するCtrlInitMsg構造体
 * @retval None
 */
static void MotorControl_WaitInitParams(int id, SerialBridge *serial,
		CtrlInitMsg &init) {
	init.rx = ctrl_init_msg_t();
	init.tx.is_received = false;
	const float min_limit = 0.000001;

	serial->add_frame(id, &init); /* initメッセージをidとして登録する */

	while (1) {
		bool rc = init.rx.is_received; /* コントローラ側からの応答結果を格納 */
		if (rc){
			if((init.rx.encoder_cpr != 0) && (init.rx.gear_ratio >= min_limit) && (init.rx.max_rps >= min_limit))
				break; /* データに問題がなければループを抜ける */
			else
				init.rx.is_received = false;
		}
		serial->write(id);
		serial->update();
	}
	serial->rm_frame(id); /* メッセージの登録を解除 */
}

/**
 * @brief 制御器の各制御パラメータの更新を確認する関数
 * 
 * @param[in] id      メッセージの登録先
 * @param[in] serial  使用するSerialBridgeクラスポインタ
 * @param[out] cparam 参照するPID::ctrl_param_t構造体
 * @param[out] param  参照するCtrlParamMsg構造体
 * @retval true データの更新あり
 * @retval false  データの更新なし
 */
static bool Check_ControlParameters(int id, SerialBridge *serial,
		PID::ctrl_param_t &cparam, CtrlParamMsg &param) {
	if (param.was_updated()) { /* データの更新があるとき */
		param.tx.is_received = true;

		/* 受信データを制御器/返信データに渡す */
		param.tx.kp = cparam.kp = param.rx.kp;
		param.tx.ki = cparam.ki = param.rx.ki;
		param.tx.kd = cparam.kd = param.rx.kd;

		serial->write(id);

		return true;
	} else
		/* データの更新がないとき */
		return false;
}

/**
 * @brief 制御器の起動処理
 *   初期パラメータの待機/制御用のオブジェクトを展開
 * @retval None
 */
static void Controller_Init(void) {
	/* 起動処理の開始がわかりやすいようにLINK_LEDを点灯する */
	HAL_GPIO_WritePin(LINK_LED_GPIO_Port, LINK_LED_Pin, GPIO_PIN_SET);
	/* 制御がアクティブであるとき、停止処理を行う */
	if (ctrlr_active) {
		Controller_Deinit();
	}
	/* コントローラからの初期パラメータを待機 */
	MotorControl_WaitInitParams(7, serial, init[M1]);
	MotorControl_WaitInitParams(8, serial, init[M2]);

	/* モータドライバを初期化 */
	md[M1] = new TwoWireMD(&htim3, TIM_CHANNEL_1, MD1_DIR_GPIO_Port,
	MD1_DIR_Pin, init[M1].rx.dir_reverse);
	md[M2] = new TwoWireMD(&htim3, TIM_CHANNEL_2, MD2_DIR_GPIO_Port,
	MD2_DIR_Pin, init[M2].rx.dir_reverse);
	/* エンコーダを初期化 */
	enc[M1] = new QEI(&htim1,
			1.0 / init[M1].rx.encoder_cpr / init[M1].rx.gear_ratio);
	enc[M2] = new QEI(&htim2,
			1.0 / init[M2].rx.encoder_cpr / init[M2].rx.gear_ratio);
	/* 速度制御用構造体を初期化 */
	v_vel[M1] = PID::ctrl_variable_t { 0, 0, 0 };
	v_vel[M2] = PID::ctrl_variable_t { 0, 0, 0 };
	p_vel[M1] =
			PID::ctrl_param_t { 0.6, 0.3, 0, 1 / init[M1].rx.max_rps, true };
	p_vel[M2] =
			PID::ctrl_param_t { 0.6, 0.3, 0, 1 / init[M2].rx.max_rps, true };
	/* 電流制御用構造体を初期化 */
	v_cur[M1] = PID::ctrl_variable_t { 0, 0, 0 };
	v_cur[M2] = PID::ctrl_variable_t { 0, 0, 0 };
	p_cur[M1] = PID::ctrl_param_t { 0.1, 0, 0, 1, true };
	p_cur[M2] = PID::ctrl_param_t { 0.1, 0, 0, 1, true };
	/* 速度制御器を初期化 */
	vel_ctrl[M1] = new PID(&v_vel[M1], &p_vel[M1]);
	vel_ctrl[M2] = new PID(&v_vel[M2], &p_vel[M2]);
	/* 速度制御器を初期化 */
	cur_ctrl[M1] = new PID(&v_cur[M1], &p_cur[M1]);
	cur_ctrl[M2] = new PID(&v_cur[M2], &p_cur[M2]);
	/* MD enable ピンをHIGH(モータへの出力を有効)にする */
	HAL_GPIO_WritePin(MD_EN_GPIO_Port, MD_EN_Pin, GPIO_PIN_SET);
	/* 電流センサを初期化 */
	am = new CurrentSensor(&hadc1, 2, ACS712_V_2_I, 100);
	/* 制御をアクティブに設定 */
	ctrlr_active = true;
	/* 起動処理の終了がわかりやすいようにLINK_LEDを消灯する */
	HAL_GPIO_WritePin(LINK_LED_GPIO_Port, LINK_LED_Pin, GPIO_PIN_RESET);
}
/**
 * @brief 制御器の停止処理
 *   制御用オブジェクトの解体
 * @retval None
 */
static void Controller_Deinit(void) {
	/* MD enable ピンをLOW(モータへの出力を無効)にする */
	HAL_GPIO_WritePin(MD_EN_GPIO_Port, MD_EN_Pin, GPIO_PIN_RESET);
	/* 各オブジェクト要素を解体 */
	delete md[M1];
	delete md[M2];

	delete enc[M1];
	delete enc[M2];

	delete vel_ctrl[M1];
	delete vel_ctrl[M2];

	delete cur_ctrl[M1];
	delete cur_ctrl[M2];

	delete am;
	/* 制御を非アクティブに設定 */
	ctrlr_active = false;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	//__disable_irq();
	/* 制御がアクティブであるとき、停止処理を行う */
	if (ctrlr_active)
		Controller_Deinit();

	char error_msg[] = { "IMD EMERGENCY STOP! control failed :(\r\n"
			"To restart, press the reset button\r\n\r\n" };
	while (1) {
		/* エラー表示(UART) */
		HAL_UART_Transmit(&huart2, (uint8_t*) error_msg, strlen(error_msg),
				1000);
		/* エラー表示(LED) 4回高速点滅, 2回点滅 */
		HAL_GPIO_WritePin(LINK_LED_GPIO_Port, LINK_LED_Pin, GPIO_PIN_RESET);
		uint8_t i = 8;
		while (i--) {
			HAL_Delay(100);
			HAL_GPIO_TogglePin(LINK_LED_GPIO_Port, LINK_LED_Pin);
		}
		i = 4;
		while (i--) {
			HAL_Delay(450);
			HAL_GPIO_TogglePin(LINK_LED_GPIO_Port, LINK_LED_Pin);
		}

		HAL_Delay(1000);
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
