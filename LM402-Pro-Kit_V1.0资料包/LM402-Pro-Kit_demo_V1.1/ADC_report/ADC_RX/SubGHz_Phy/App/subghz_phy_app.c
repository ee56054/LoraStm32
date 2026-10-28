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
#include "utilities_def.h"
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
  S_TX_TIMEOUT,
} States_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Configurations */
/*Timeout*/
#define RX_TIMEOUT_VALUE              5000
#define TX_TIMEOUT_VALUE              3000
#define SEND_PERIOD_MS              	10000
#define LOCAL_ID						0x01
#define Concentrator_ID						0x10
#define DATA_TYPE01						0x01

/*Size of the payload to be sent*/
/* Size must be greater of equal the PING and PONG*/
#define MAX_APP_BUFFER_SIZE          255
#if (PAYLOAD_LEN > MAX_APP_BUFFER_SIZE)
#error PAYLOAD_LEN must be less or equal than MAX_APP_BUFFER_SIZE
#endif /* (PAYLOAD_LEN > MAX_APP_BUFFER_SIZE) */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Radio events function pointer */
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
/* device state. Master: true, Slave: false*/
bool isMaster = true;
/* random delay to make sure 2 devices will sync*/
/* the closest the random delays are, the longer it will
   take for the devices to sync when started simultaneously*/
static int32_t random_delay;
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

/**
  * @brief PingPong state machine implementation
  */
static void Concentrator_Process(void);
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
    case  BUTTON_SW2_PIN:
        BSP_LED_Toggle(LED_GREEN) ;
        HAL_Delay(20);
        BSP_LED_Toggle(LED_GREEN) ;
        APP_PRINTF("BUTTON SW2\r\n");
      break;
    case  BUTTON_SW3_PIN:
        BSP_LED_Toggle(LED_RED) ;
        HAL_Delay(20);
        BSP_LED_Toggle(LED_RED) ;
        APP_PRINTF("BUTTON SW3\r\n");
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


  /* USER CODE END SubghzApp_Init_1 */

  /* Radio initialization */
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;

  Radio.Init(&RadioEvents);

  /* USER CODE BEGIN SubghzApp_Init_2 */
  /* Radio Set frequency */
  Radio.SetChannel(RF_FREQUENCY);

  /* Radio configuration */
  APP_LOG(TS_OFF, VLEVEL_M, "---------------\n\r");
  APP_LOG(TS_OFF, VLEVEL_M, "LORA_MODULATION\n\r");
  APP_LOG(TS_OFF, VLEVEL_M, "LORA_BW=%d kHz\n\r", (1 << LORA_BANDWIDTH) * 125);
  APP_LOG(TS_OFF, VLEVEL_M, "LORA_SF=%d\n\r", LORA_SPREADING_FACTOR);

  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);

  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

  Radio.SetMaxPayloadLength(MODEM_LORA, MAX_APP_BUFFER_SIZE);

  /* LED initialization*/
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_RED);
  BSP_LED_Init(LED_BLUE);
  BSP_PB_Init(BUTTON_SW1, BUTTON_MODE_EXTI);
  BSP_PB_Init(BUTTON_SW2, BUTTON_MODE_EXTI);
  BSP_PB_Init(BUTTON_SW3, BUTTON_MODE_EXTI);

  /*calculate random delay for synchronization*/
  random_delay = (Radio.Random()) >> 22; /*10bits random e.g. from 0 to 1023 ms*/
  /*fills tx buffer*/
  memset(BufferTx, 0x0, MAX_APP_BUFFER_SIZE);

  APP_LOG(TS_ON, VLEVEL_L, "rand=%d\n\r", random_delay);
  /*starts reception*/
  //持续接收
  Radio.Rx(0);
  /*register task to to be run in while(1) after Radio IT*/
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), UTIL_SEQ_RFU, Concentrator_Process);
  /* USER CODE END SubghzApp_Init_2 */
}

/* USER CODE BEGIN EF */

/* USER CODE END EF */

/* Private functions ---------------------------------------------------------*/

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

static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo)
{
  /* USER CODE BEGIN OnRxDone */
  APP_LOG(TS_ON, VLEVEL_L, "OnRxDone\n\r");
  APP_LOG(TS_ON, VLEVEL_L, "RssiValue=%d dBm, SnrValue=%ddB\n\r", rssi, LoraSnr_FskCfo);
	BSP_LED_On(LED_GREEN);
  BSP_LED_On(LED_RED);
  /* Record payload Signal to noise ratio in Lora*/
  SnrValue = LoraSnr_FskCfo;
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
  for (int i = 0; i < RxBufferSize; i++)
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
static void Concentrator_Process(void)
{
	uint16_t SenID = 0;
	uint16_t ConID = 0;
	int16_t temp_v = 0;
	uint16_t bat_v = 0;
	uint16_t pa11_v = 0;
	Radio.Rx(0);
  switch (State)
  {
	  //接收处理
    case S_RX:
		 APP_LOG(TS_ON, VLEVEL_L, "payload. size=%d \n\r", RxBufferSize);
	
	for (int i = 0; i < RxBufferSize; i++)
		{
			APP_LOG(TS_OFF, VLEVEL_L, "%02X", BufferRx[i]);
			if (i % 16 == 15)
			{
			APP_LOG(TS_OFF, VLEVEL_L, "\n\r");
			}
		}
		APP_LOG(TS_OFF, VLEVEL_L, "\n\r");
	
	//判断长度
	 if(BufferRx[2] == RxBufferSize)
	 {
		//判断帧头
		if((BufferRx[0] == 0xA5) && (BufferRx[1] == 0x7E))	
		{
			//判断SUM
			if(BufferRx[RxBufferSize - 1] == SumCheck(BufferRx,RxBufferSize - 1))
			{
				//判断地址是否正确
				memcpy(&ConID,&BufferRx[3],2);
				if((ConID == Concentrator_ID) || (ConID == 0xFFFF))
				{
					//传感器类型
					if(BufferRx[7] == DATA_TYPE01)
					{
						//获取传感器ID
						memcpy(&SenID,&BufferRx[5],2);
						//获取温度
						memcpy(&temp_v,&BufferRx[8],2);
						//获取BAT电压
						memcpy(&bat_v,&BufferRx[10],2);
						//获取PA11的ADC
						memcpy(&pa11_v,&BufferRx[12],2);
						APP_LOG(TS_ON, VLEVEL_L, "Recv:ID=0x%04x Temperature = %d.%d  Battery = %d PA11Value = %d\n\r",SenID,temp_v>>8,(uint8_t)temp_v,bat_v,pa11_v);
					}
					/*
					//其他类型的解析
					else if
					{
					}
					*/
					else
					{
						APP_LOG(TS_OFF, VLEVEL_M, "Unknown type\n\r");
					}
					
				}
				else
				{
					APP_LOG(TS_OFF, VLEVEL_M, "The desired ID =%d ,ID = %d error\n\r",Concentrator_ID,ConID);
				}
			}
			else
			{
				APP_LOG(TS_OFF, VLEVEL_M, "Checksum error\n\r");
			}
		  
			
		}
		else
		{
			APP_LOG(TS_OFF, VLEVEL_M, "The data header is incorrect\n\r");
		}
	 }
	 else
	 {
		 APP_LOG(TS_OFF, VLEVEL_M, "Packet length error\n\r");
	 }
	 BSP_LED_Off(LED_GREEN);
	BSP_LED_Off(LED_RED);
     State = S_IDLE;
      break;
	case S_IDLE: 
    case S_TX:
    case S_RX_TIMEOUT:
    case S_RX_ERROR:
    case S_TX_TIMEOUT:
		State = S_IDLE;
      break;
    default:
      break;
  }
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
