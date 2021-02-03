/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum _eENGINE_PROCESS {
	EGNP_NONE,
	EGNP_ACC,
	EGNP_WAIT1S,
	EGNP_AFTER_ON,
	EGNP_ON,
	EGNP_START,
	EGNP_AFTER_START,
	EGNP_MAX
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CANDRV_RX_BUF_CNT					20
#define DIO1_ON 				HAL_GPIO_WritePin(DOUT1_GPIO_Port,DOUT1_Pin,GPIO_PIN_SET);
#define DIO1_OFF 			HAL_GPIO_WritePin(DOUT1_GPIO_Port,DOUT1_Pin,GPIO_PIN_RESET);
#define DIO2_ON 				HAL_GPIO_WritePin(DOUT2_GPIO_Port,DOUT2_Pin,GPIO_PIN_SET);
#define DIO2_OFF 			HAL_GPIO_WritePin(DOUT2_GPIO_Port,DOUT2_Pin,GPIO_PIN_RESET);
#define DIO3_ON 				HAL_GPIO_WritePin(DOUT3_GPIO_Port,DOUT3_Pin,GPIO_PIN_SET);
#define DIO3_OFF 			HAL_GPIO_WritePin(DOUT3_GPIO_Port,DOUT3_Pin,GPIO_PIN_RESET);
#define DIO4_ON 				HAL_GPIO_WritePin(DOUT4_GPIO_Port,DOUT4_Pin,GPIO_PIN_SET);
#define DIO4_OFF 			HAL_GPIO_WritePin(DOUT4_GPIO_Port,DOUT4_Pin,GPIO_PIN_RESET);
#define DIO5_ON 				HAL_GPIO_WritePin(DOUT5_GPIO_Port,DOUT5_Pin,GPIO_PIN_SET);
#define DIO5_OFF 			HAL_GPIO_WritePin(DOUT5_GPIO_Port,DOUT5_Pin,GPIO_PIN_RESET);
#define DIO6_ON 				HAL_GPIO_WritePin(DOUT6_GPIO_Port,DOUT6_Pin,GPIO_PIN_SET);
#define DIO6_OFF 			HAL_GPIO_WritePin(DOUT6_GPIO_Port,DOUT6_Pin,GPIO_PIN_RESET);
#define DIO7_ON 				HAL_GPIO_WritePin(DOUT7_GPIO_Port,DOUT7_Pin,GPIO_PIN_SET);
#define DIO7_OFF 			HAL_GPIO_WritePin(DOUT7_GPIO_Port,DOUT7_Pin,GPIO_PIN_RESET);
#define DIO8_ON 				HAL_GPIO_WritePin(DOUT8_GPIO_Port,DOUT8_Pin,GPIO_PIN_SET);
#define DIO8_OFF 			HAL_GPIO_WritePin(DOUT8_GPIO_Port,DOUT8_Pin,GPIO_PIN_RESET);
//
#define LED_RED_OFF			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
#define LED_RED_ON			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
#define LED_RED_TOG			HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
#define LED_GREEN_OFF		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
#define LED_GREEN_ON		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
#define LED_GREEN_TOG		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
#define LED_ORANGE_OFF		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
#define LED_ORANGE_ON		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
#define LED_ORANGE_TOG		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

//

//#define _3STEP_TOGGLE_UP_SW_MASK			0x10
#define _3STEP_TOGGLE_UP_SW_MASK        0x08    // shjang
#define SAFETY_BAR_ON_MASK      0x08

#define KEY_IN_MASK     0x10    //shajng, for A
#define KEY_ACC_MASK    0x20 // shjang, for B
#define KEY_ON_MASK  0x40    // shjang, for C
#define KEY_IGNITION_MASK       0x80    // shjang, for D


//#define _2STEP_TOGGLE_UP_SW_MASK			0x40
//#define RIGHT_SIDE_PUSH_FR_SW_MASK		0x80


//#define RIGHT_SIDE_PUSH_RR_SW_MASK		0x04

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void MCP41HV51_Control(uint8_t CH, uint8_t Value);
void MCP41HV51_DIO_Initialization(void);
void SPI1_CS1_Enable(void);
void SPI1_CS1_Disable(void);
void SPI1_CS2_Enable(void);
void SPI1_CS2_Disable(void);
void CAN1Tx(long cid, unsigned int txbyte, uint8_t * cdata);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//CAN_RxHeaderTypeDef	can1RxHeader;
//CAN_TxHeaderTypeDef	can1TxHeader;
//uint8_t	can1Rx0Data[8];
//uint32_t	can1TxMailBox;
//uint8_t	can1Tx0Data[8];
//uint8_t	Can1TxMsgBuff[8] = NULL;
volatile uint8_t	can1_rx0_flag = 0;
uint8_t spi_address;
struct _tagCAN_INFO {
	struct {
		CAN_RxHeaderTypeDef	header;
		uint8_t	data[8];
	}rx[CANDRV_RX_BUF_CNT];
	uint8_t rxHead;
	uint8_t rxTail;
	struct {
		CAN_TxHeaderTypeDef	header;
		uint8_t	data[8];
		uint8_t	msgBuff[8];
	}tx;
	uint32_t	TxMailBox;
}CANInfo;

struct _tagENGINE_PROCESS_INFO {
	uint8_t step;
	uint32_t onCnt;
	uint32_t time;
	uint8_t status;
}EgnInfo, AttInfo;
struct _tagPOTENTIONMETER_INFO {
	uint8_t step;
	uint32_t delayCnt;
}PtrMtrInfo;
uint32_t LED_RunCnt;

void CANDrv_Init(void)
{
	CAN_FilterTypeDef	canFilter1;
	
	// CAN1 Filter 설정
	canFilter1.FilterMaskIdHigh = 0x0000;
	canFilter1.FilterIdHigh = 0x00;
	canFilter1.FilterMaskIdLow = 0x0000;
	canFilter1.FilterIdLow = 0x00;
	canFilter1.FilterMode = CAN_FILTERMODE_IDMASK;
	canFilter1.FilterScale = CAN_FILTERSCALE_16BIT;
	canFilter1.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	canFilter1.FilterBank = 0;
	canFilter1.FilterActivation = ENABLE;

	// CAN1 초기화
	HAL_CAN_ConfigFilter(&hcan1, &canFilter1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(&hcan1);
 }

void Engine_Control(uint8_t data)
{
	if ( (data & 0xF0) > 0)  // status B, C, D
        {
          if (data&KEY_IN_MASK) {
                  DIO1_OFF; //key acc 
                  DIO2_OFF; //key on 
                  DIO3_OFF; //key ignition 
                  EgnInfo.step = EGNP_NONE;
                  EgnInfo.status = EGNP_NONE;
                  AttInfo.status = EGNP_NONE;
          }
            if (data&KEY_ACC_MASK) {
                  DIO1_ON; //key acc 
                  DIO2_OFF; //key on 
                  DIO3_OFF; //key ignition 
                   EgnInfo.status = EGNP_ACC;
            }
            if (data&KEY_ON_MASK) {
                  DIO1_ON; //key acc 
                  DIO2_ON; //key on 
                  DIO3_OFF; //key ignition 
                  EgnInfo.status = EGNP_ON;
            }
            if (data&KEY_IGNITION_MASK && !(data&SAFETY_BAR_ON_MASK)) {
                  DIO1_ON; //key acc 
                  DIO2_ON; //key on 
                  DIO3_ON; //key ignition 
                  EgnInfo.status = EGNP_START;
                  EgnInfo.step = EGNP_START;
             }
        }
        
	//side bar
	if (data&SAFETY_BAR_ON_MASK) {
		DIO4_ON;
	}
	else {
		DIO4_OFF;
	}
        
        //3way Toggle check 
        if (data&0x02) // UP
          if (EgnInfo.status == EGNP_START)
            AttInfo.status = EGNP_ON;
}

void DIO_Control(void)
{
}

void PtnMtr_Control(uint8_t data)
{
	/* if (data&RIGHT_SIDE_PUSH_RR_SW_MASK) {
		if (PtrMtrInfo.step < 0xcc) {
			PtrMtrInfo.step += 10;
			if (PtrMtrInfo.step > 0xcc) PtrMtrInfo.step = 0xcc;
			MCP41HV51_Control(1, PtrMtrInfo.step);                 // 0x00 ~ 0xff값을 입력
		}
		//MCP41HV51_Control(2,can1Rx0Data[1]);
	}
	else {
		if (PtrMtrInfo.step > 0x7f) {
			PtrMtrInfo.step -= 10;
			if (PtrMtrInfo.step < 0x7f) PtrMtrInfo.step = 0x7f;
			MCP41HV51_Control(1, PtrMtrInfo.step);                // 0x00 ~ 0xff값을 입력
		}
	}*/
        
  // Data in : 0x0~0x50
  //if (AttInfo.step == EGNP_ON) {
    if (data > 0x2A && data < 0x3A)
      PtrMtrInfo.step =  0x80;
    else
      PtrMtrInfo.step =  (uint8_t)((float)data * 255. / 0x50);  // scale to 0~0xFF
    MCP41HV51_Control(1, PtrMtrInfo.step);                // 0x00 ~ 0xff값을 입력
  //}
    
}

void System_Init(void)
{
	int i;
	char *p;

	p = (char *)&PtrMtrInfo;
	for (i = 0; i < sizeof(struct _tagPOTENTIONMETER_INFO); i++) p[i] = 0;
	p = (char *)&EgnInfo;
	for (i = 0; i < sizeof(struct _tagENGINE_PROCESS_INFO); i++) p[i] = 0;
	EgnInfo.step = EGNP_NONE;
	CANInfo.rxHead = CANInfo.rxTail = 0;
	PtrMtrInfo.step = 0x7f;
	LED_RunCnt = 0;
	LED_RED_OFF;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  CANDrv_Init();
  MCP41HV51_DIO_Initialization();
  System_Init();
  
  
  EgnInfo.step = EGNP_NONE;
  EgnInfo.status = EGNP_NONE;
  AttInfo.status = EGNP_NONE;
  AttInfo.step = EGNP_NONE;
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if 0  
	if(can1_rx0_flag == 1){
		can1_rx0_flag = 0;
		if(can1RxHeader.StdId == 0x18A){		                        //0x18A                           
			MCP41HV51_Control(1,can1Rx0Data[0]);                                // 0x00 ~ 0xff값을 입력
			MCP41HV51_Control(2,can1Rx0Data[1]);

			if(can1Rx0Data[2] == 1){
				HAL_GPIO_WritePin(DOUT1_GPIO_Port,DOUT1_Pin,GPIO_PIN_RESET);
			}
			else{
				HAL_GPIO_WritePin(DOUT1_GPIO_Port,DOUT1_Pin,GPIO_PIN_SET);
			}
		}
		HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
		HAL_Delay(100);
	}
#else
	//if(CANInfo.rxHead != CANInfo.rxTail) {
	if (can1_rx0_flag == 1) {
		can1_rx0_flag = 0;
		if(CANInfo.rx[CANInfo.rxHead].header.StdId == 0x18A){                           
			Engine_Control(CANInfo.rx[CANInfo.rxHead].data[2]);
			//DIO_Control();
			//PtnMtr_Control(CANInfo.rx[CANInfo.rxHead].data[2]);
                        PtnMtr_Control(CANInfo.rx[CANInfo.rxHead].data[4]); // shjang
			LED_ORANGE_TOG;
		}
		//CANInfo.rxTail++;
		//if (CANInfo.rxTail >= CANDRV_RX_BUF_CNT) CANInfo.rxTail = 0;
	}
	if (LED_RunCnt++ > 2000000) {
		LED_GREEN_TOG;
		LED_RunCnt = 0;
	}
#endif
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 24;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;//4TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;//1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;//2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED1_Pin|LED2_Pin|LED3_Pin|SHDN_Pin 
                          |WLAT_Pin|DOUT3_Pin|DOUT4_Pin|DOUT5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS2_Pin|DOUT7_Pin|DOUT8_Pin|DOUT1_Pin 
                          |DOUT2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DOUT6_GPIO_Port, DOUT6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin SHDN_Pin 
                           WLAT_Pin DOUT3_Pin DOUT4_Pin DOUT5_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|SHDN_Pin 
                          |WLAT_Pin|DOUT3_Pin|DOUT4_Pin|DOUT5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : CS1_Pin */
  GPIO_InitStruct.Pin = CS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS2_Pin DOUT7_Pin DOUT8_Pin DOUT1_Pin 
                           DOUT2_Pin */
  GPIO_InitStruct.Pin = CS2_Pin|DOUT7_Pin|DOUT8_Pin|DOUT1_Pin 
                          |DOUT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DOUT6_Pin */
  GPIO_InitStruct.Pin = DOUT6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DOUT6_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//------------------------------------------------------------------------------
void MCP41HV51_Control(uint8_t CH, uint8_t Value)
{
  if(CH == 1){
      SPI1_CS1_Enable();
      HAL_GPIO_WritePin(WLAT_GPIO_Port, WLAT_Pin, GPIO_PIN_RESET);
      spi_address = 0x00;
      HAL_SPI_Transmit(&hspi1, &spi_address, sizeof(spi_address), 0x1000);
      spi_address = Value;
      HAL_SPI_Transmit(&hspi1, &spi_address, sizeof(spi_address), 0x1000);
      HAL_GPIO_WritePin(WLAT_GPIO_Port, WLAT_Pin, GPIO_PIN_SET);
      SPI1_CS1_Disable(); 
  }
  else if(CH == 2){
      SPI1_CS2_Enable();
      HAL_GPIO_WritePin(WLAT_GPIO_Port, WLAT_Pin, GPIO_PIN_RESET);
      spi_address = 0x00;
      HAL_SPI_Transmit(&hspi1, &spi_address, sizeof(spi_address), 0x1000);
      spi_address = Value;
      HAL_SPI_Transmit(&hspi1, &spi_address, sizeof(spi_address), 0x1000);
      HAL_GPIO_WritePin(WLAT_GPIO_Port, WLAT_Pin, GPIO_PIN_SET);
      SPI1_CS2_Disable(); 

  }
}
//------------------------------------------------------------------------------
void MCP41HV51_DIO_Initialization(void)
{
  HAL_GPIO_WritePin(DOUT1_GPIO_Port,DOUT1_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DOUT2_GPIO_Port,DOUT2_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DOUT3_GPIO_Port,DOUT3_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DOUT4_GPIO_Port,DOUT4_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DOUT5_GPIO_Port,DOUT5_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DOUT6_GPIO_Port,DOUT6_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DOUT7_GPIO_Port,DOUT7_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DOUT8_GPIO_Port,DOUT8_Pin,GPIO_PIN_RESET);
  
  HAL_GPIO_WritePin(CS1_GPIO_Port,CS1_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(CS2_GPIO_Port,CS2_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(SHDN_GPIO_Port,SHDN_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(WLAT_GPIO_Port, WLAT_Pin, GPIO_PIN_SET);  
  
  SPI1_CS1_Enable();
  HAL_GPIO_WritePin(WLAT_GPIO_Port, WLAT_Pin, GPIO_PIN_RESET);
  spi_address = 0x00;
  HAL_SPI_Transmit(&hspi1, &spi_address, sizeof(spi_address), 0x1000);
  spi_address = 0x7F;
  HAL_SPI_Transmit(&hspi1, &spi_address, sizeof(spi_address), 0x1000);
  HAL_GPIO_WritePin(WLAT_GPIO_Port, WLAT_Pin, GPIO_PIN_SET);
  SPI1_CS1_Disable(); 
  
  SPI1_CS2_Enable();
  HAL_GPIO_WritePin(WLAT_GPIO_Port, WLAT_Pin, GPIO_PIN_RESET);
  spi_address = 0x00;
  HAL_SPI_Transmit(&hspi1, &spi_address, sizeof(spi_address), 0x1000);
  spi_address = 0x7F;
  HAL_SPI_Transmit(&hspi1, &spi_address, sizeof(spi_address), 0x1000);
  HAL_GPIO_WritePin(WLAT_GPIO_Port, WLAT_Pin, GPIO_PIN_SET);
  SPI1_CS2_Disable();   

}
//------------------------------------------------------------------------------
void SPI1_CS1_Enable(void)
{
      HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_RESET);
}
//------------------------------------------------------------------------------
void SPI1_CS1_Disable(void)
{
      HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_SET);
}
//------------------------------------------------------------------------------
void SPI1_CS2_Enable(void)
{
      HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, GPIO_PIN_RESET);
}
//------------------------------------------------------------------------------
void SPI1_CS2_Disable(void)
{
      HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, GPIO_PIN_SET);
}
//------------------------------------------------------------------------------
// CAN RCV Interrupt Callback Function
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
 
    if(hcan->Instance == CAN1){
        /* Release RX FIFO 0 */
        //HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can1RxHeader, &can1Rx0Data[0]);
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, 
        		&CANInfo.rx[CANInfo.rxHead].header, &CANInfo.rx[CANInfo.rxHead].data[0]);
        can1_rx0_flag = 1;
        //CANInfo.rxHead++;
	 //if (CANInfo.rxHead >= CANDRV_RX_BUF_CNT) CANInfo.rxHead = 0;
    }
}
//------------------------------------------------------------------------------
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
