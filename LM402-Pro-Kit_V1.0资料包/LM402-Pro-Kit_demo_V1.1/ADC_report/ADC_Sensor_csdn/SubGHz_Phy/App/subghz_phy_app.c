/*!
 * \file      subghz_phy_app.c
 *
 * \brief     Ping-Pong implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
/**
  ******************************************************************************
  *
  *          Portions COPYRIGHT 2020 STMicroelectronics
  *
  * @file    subghz_phy_app.c
  * @author  MCD Application Team
  * @brief   Application of the SubGHz_Phy Middleware
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include "sys_app.h"
#include "subghz_phy_app.h"
#include "radio.h"
#include "app_version.h"

/* USER CODE BEGIN Includes */
#include "stm32_timer.h"
#include "stm32_seq.h"
#include <stdint.h>
#include "utilities_def.h"
#include "adc_if.h"
/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */
typedef enum
{
  S_IDLE,
  S_RX,
  S_RX_TIMEOUT,
  S_RX_ERROR,
  S_TX,
  S_TXing,
  S_TX_TIMEOUT,
  S_SEND_DATA,
} States_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Configurations */
/*Timeout*/
#define RX_TIMEOUT_VALUE              5000
#define TX_TIMEOUT_VALUE              3000
//上报周期
#define SEND_PERIOD_MS              	10000
//本端ID
#define LOCAL_ID						0x01
//远端ID
#define REMOTE_ID						0x10
//数据类型
#define DATA_TYPE						0x01
/*Size of the payload to be sent*/
/* Size must be greater of equal the PING and PONG*/
#define MAX_APP_BUFFER_SIZE          255
#if (PAYLOAD_LEN > MAX_APP_BUFFER_SIZE)
#error PAYLOAD_LEN must be less or equal than MAX_APP_BUFFER_SIZE
#endif /* (PAYLOAD_LEN > MAX_APP_BUFFER_SIZE) */


static RadioEvents_t RadioEvents;
/* USER CODE BEGIN PV */

/*Ping Pong FSM states */
static States_t State = S_IDLE;
/* App Rx Buffer*/
static uint8_t BufferRx[MAX_APP_BUFFER_SIZE];
/* App Tx Buffer*/
static uint8_t BufferTx[MAX_APP_BUFFER_SIZE];
/* Last  Received Buffer Size*/
uint16_t RxBufferSize = 0;
/* Last  Received packer Rssi*/
int8_t RssiValue = 0;
/* Last  Received packer SNR (in Lora modulation)*/
int8_t SnrValue = 0;
/* Led Timers objects*/
//上报定时器
static UTIL_TIMER_Object_t timerSendData;
/* random delay to make sure 2 devices will sync*/
/* the closest the random delays are, the longer it will
   take for the devices to sync when started simultaneously*/
static int32_t random_delay;
//采集数据
static int16_t temp_v=0;
static uint16_t bat_v=0;
static uint16_t pa11_v=0;


/* USER CODE END PV */



/* Private function prototypes -----------------------------------------------*/
/*!
 * @brief Function to be executed on Radio Tx Done event
 */
static void OnTxDone(void);

/**
  * @brief Function to be executed on Radio Rx Done event
  * @param  payload ptr of buffer received
  * @param  size buffer size
  * @param  rssi
  * @param  LoraSnr_FskCfo
  */
static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo);

/**
  * @brief Function executed on Radio Tx Timeout event
  */
static void OnTxTimeout(void);

/**
  * @brief Function executed on Radio Rx Timeout event
  */
static void OnRxTimeout(void);

/**
  * @brief Function executed on Radio Rx Error event
  */
static void OnRxError(void);

/* USER CODE BEGIN PFP */
/**
  * @brief  Function executed on when led timer elapses
  * @param  context ptr of LED context
  */
static void OnSendDataEvent(void *context);

/**
  * @brief PingPong state machine implementation
  */
static void SendData_Process(void);
static uint16_t PackData(uint16_t r_id);
static uint8_t SumCheck(uint8_t *buf,uint8_t len);
/* USER CODE END PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
    case  BUTTON_SW1_PIN:
        BSP_LED_Toggle(LED_BLUE) ;
        HAL_Delay(20);
        BSP_LED_Toggle(LED_BLUE) ;
        APP_PRINTF("BUTTON SW1\r\n");
         
      break;
    default:
        APP_PRINTF("Unkonw Button\r\n");
      break;
  }
}
/* Exported functions ---------------------------------------------------------*/
void SubghzApp_Init(void)
{
  /* USER CODE BEGIN SubghzApp_Init_1 */
  APP_LOG(TS_OFF, VLEVEL_M, "\n\rPING PONG\n\r");
  /* Print APP version*/
  APP_LOG(TS_OFF, VLEVEL_M, "APP_VERSION= V%X.%X.%X\r\n",
          (uint8_t)(__APP_VERSION >> __APP_VERSION_MAIN_SHIFT),
          (uint8_t)(__APP_VERSION >> __APP_VERSION_SUB1_SHIFT),
          (uint8_t)(__APP_VERSION >> __APP_VERSION_SUB2_SHIFT));

  /*创建定时器对象*/
  UTIL_TIMER_Create(&timerSendData, 0xFFFFFFFFU, UTIL_TIMER_ONESHOT, OnSendDataEvent, NULL);
/*设置定时周期*/
  UTIL_TIMER_SetPeriod(&timerSendData, SEND_PERIOD_MS);
/*启动定时*/	
  UTIL_TIMER_Start(&timerSendData);
  /* USER CODE END SubghzApp_Init_1 */

  /* Radio initialization */
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;
/*初始化射频*/
  Radio.Init(&RadioEvents);

  /*设置射频频率*/
  Radio.SetChannel(RF_FREQUENCY);

  /* Radio configuration */
  APP_LOG(TS_OFF, VLEVEL_M, "---------------\n\r");
  APP_LOG(TS_OFF, VLEVEL_M, "LORA_MODULATION\n\r");
  APP_LOG(TS_OFF, VLEVEL_M, "LORA_BW=%d kHz\n\r", (1 << LORA_BANDWIDTH) * 125);
  APP_LOG(TS_OFF, VLEVEL_M, "LORA_SF=%d\n\r", LORA_SPREADING_FACTOR);
  /*设置发送参数*/
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);
/*设置接收参数*/
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
/*设置最大负载长度*/
  Radio.SetMaxPayloadLength(MODEM_LORA, MAX_APP_BUFFER_SIZE);

  /* LED initialization*/
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_RED);
  BSP_LED_Init(LED_BLUE);
  /* 按键 initialization*/
  BSP_PB_Init(BUTTON_SW1, BUTTON_MODE_EXTI);

  /*calculate random delay for synchronization*/
  random_delay = (Radio.Random()) >> 22; /*10bits random e.g. from 0 to 1023 ms*/
  /*fills tx buffer*/
  memset(BufferTx, 0x0, MAX_APP_BUFFER_SIZE);

  APP_LOG(TS_ON, VLEVEL_L, "rand=%d\n\r", random_delay);
  /*starts reception*/
  //Radio.Rx(RX_TIMEOUT_VALUE + random_delay);
  /*register task to to be run in while(1) after Radio IT*/
  /* 注册任务*/
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), UTIL_SEQ_RFU, SendData_Process);
  /* USER CODE END SubghzApp_Init_2 */
}

/* USER CODE BEGIN EF */

/* USER CODE END EF */

/* Private functions ---------------------------------------------------------*/
/* 发送完成中断*/
static void OnTxDone(void)
{
  /* USER CODE BEGIN OnTxDone */
  APP_LOG(TS_ON, VLEVEL_L, "OnTxDone\n\r");
  /* Update the State of the FSM*/
  State = S_TX;
  /* Run PingPong process in background*/
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
  /* USER CODE END OnTxDone */
}
/* 接收完成中断*/
static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo)
{
  /* USER CODE BEGIN OnRxDone */
  APP_LOG(TS_ON, VLEVEL_L, "OnRxDone\n\r");
  APP_LOG(TS_ON, VLEVEL_L, "RssiValue=%d dBm, SnrValue=%ddB\n\r", rssi, LoraSnr_FskCfo);
	
  /* Update the State of the FSM*/
  State = S_RX;
  /* Clear BufferRx*/
  memset(BufferRx, 0, MAX_APP_BUFFER_SIZE);
  /* Record payload size*/
  RxBufferSize = size;
  if (RxBufferSize <= MAX_APP_BUFFER_SIZE)
  {
    memcpy(BufferRx, payload, RxBufferSize);
  }
  /* Record Received Signal Strength*/
  RssiValue = rssi;
  /* Record payload content*/
  APP_LOG(TS_ON, VLEVEL_H, "payload. size=%d \n\r", size);
  for (int i = 0; i < PAYLOAD_LEN; i++)
  {
    APP_LOG(TS_OFF, VLEVEL_H, "%02X", BufferRx[i]);
    if (i % 16 == 15)
    {
      APP_LOG(TS_OFF, VLEVEL_H, "\n\r");
    }
  }
  APP_LOG(TS_OFF, VLEVEL_H, "\n\r");
  /* Run PingPong process in background*/
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
  /* USER CODE END OnRxDone */
}
/* 发送超时中断*/
static void OnTxTimeout(void)
{
  /* USER CODE BEGIN OnTxTimeout */
  APP_LOG(TS_ON, VLEVEL_L, "OnTxTimeout\n\r");
  /* Update the State of the FSM*/
  State = S_TX_TIMEOUT;
  /* Run PingPong process in background*/
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
  /* USER CODE END OnTxTimeout */
}
/* 接收超时中断*/
static void OnRxTimeout(void)
{
  /* USER CODE BEGIN OnRxTimeout */
  APP_LOG(TS_ON, VLEVEL_L, "OnRxTimeout\n\r");
  /* Update the State of the FSM*/
  State = S_RX_TIMEOUT;
  /* Run PingPong process in background*/
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
  /* USER CODE END OnRxTimeout */
}
/* 接收错误中断*/
static void OnRxError(void)
{
  /* USER CODE BEGIN OnRxError */
  APP_LOG(TS_ON, VLEVEL_L, "OnRxError\n\r");
  /* Update the State of the FSM*/
  State = S_RX_ERROR;
  /* Run PingPong process in background*/
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
  /* USER CODE END OnRxError */
}

/* USER CODE BEGIN PrFD */
/* 采集发送数据的主函数*/
static void SendData_Process(void)
{

	uint16_t temp = 0;
	uint16_t sendlen = 0;
//射频休眠	
  switch (State)
  {
	 case S_IDLE:
	 case S_TXing:	 
		break;  
    case S_RX:
		Radio.Sleep();
		State = S_IDLE;
      break;
    case S_TX:
		BSP_LED_Off(LED_GREEN);
		BSP_LED_Off(LED_RED);
		 Radio.Sleep();
		State = S_IDLE;
      break;
    case S_RX_TIMEOUT:
    case S_RX_ERROR:
    case S_TX_TIMEOUT:
		 Radio.Sleep();
		State = S_IDLE;
      break;
	case S_SEND_DATA:
		
		temp_v = SYS_GetTemperatureLevel();
		bat_v = SYS_GetBatteryLevel();
		temp = GetADC_PA11();
		//将PA11的ADC转换成电压，单位mV
		pa11_v =__LL_ADC_CALC_DATA_TO_VOLTAGE(bat_v, temp,ADC_RESOLUTION_12B);
		APP_LOG(TS_ON, VLEVEL_L, "TemperatureLevel = %d.%d  BatteryLevel = %d PA11Level = %d\n\r",temp_v>>8,(uint8_t)temp_v,bat_v,pa11_v);
		//组包
		sendlen = PackData(REMOTE_ID);
		if(sendlen < MAX_APP_BUFFER_SIZE)
			Radio.Send(BufferTx,sendlen);
		else
			APP_PRINTF("The length of the sent data is greater than MAX_APP_BUFFER_SIZE(%d)\n\r",MAX_APP_BUFFER_SIZE);
		State = S_TXing;
      break;
    default:
      break;
  }
}

static void OnSendDataEvent(void *context)
{
  BSP_LED_On(LED_GREEN);
  BSP_LED_On(LED_RED);
  UTIL_TIMER_Start(&timerSendData);
	//空闲状态时进行数据发送
  if(State == S_IDLE)
  {
	  State = S_SEND_DATA;
	  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
  }	  
  else
  {
	  APP_PRINTF("OnSendDataEvent Error State(%d)\n\r",State);
  }
}

static uint16_t PackData(uint16_t r_id)
{
	uint8_t len = 0;
	uint8_t i = 0;
	uint16_t id = LOCAL_ID;
	BufferTx[len++]= 0xA5;
	BufferTx[len++]= 0x7E;
	len++;
	memcpy(&BufferTx[len],&r_id,2);
	len+=2;
	memcpy(&BufferTx[len],&id,2);
	len+=2;
	BufferTx[len++]= DATA_TYPE;
	memcpy(&BufferTx[len],&temp_v,2);
	len+=2;
	memcpy(&BufferTx[len],&bat_v,2);
	len+=2;
	memcpy(&BufferTx[len],&pa11_v,2);
	len+=2;
	//长度
	BufferTx[2]= len+1;
	BufferTx[len]= SumCheck(BufferTx,len);
	len++;
	
	APP_LOG(TS_ON, VLEVEL_L, "%s: payload. size=%d \n\r", __func__,len);
	  for (int i = 0; i < len; i++)
	  {
		APP_LOG(TS_OFF, VLEVEL_L, "%02X", BufferTx[i]);
		if (i % 16 == 15)
		{
		  APP_LOG(TS_OFF, VLEVEL_L, "\n\r");
		}
	  }
	APP_LOG(TS_OFF, VLEVEL_L, "\n\r");
	return len;
}

static uint8_t SumCheck(uint8_t *buf,uint8_t len)
{
	uint8_t i = 0;
	uint8_t sum = 0;
	for(i = 0;i < len;i++)
	{
		sum += buf[i];
	}
	return sum;
}
/* USER CODE END PrFD */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
