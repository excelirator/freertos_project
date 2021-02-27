/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pca9698.h"
#include "jhd12864e.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

typedef struct {
	uint16_t pinsRequired;
	uint16_t itersRequired;
} configPacket_t;

typedef enum {
	up_button = 0x00,
	down_button = 0x01,
	select_button = 0x02,
	usb_mount = 0x03,
	usb_unmount = 0x04,
	no_button = 0xFF
} hmi_state;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

RNG_HandleTypeDef hrng;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for hmi */
osThreadId_t hmiHandle;
uint32_t glcdDisplayBuffer[ 128 ];
osStaticThreadDef_t glcdDisplayControlBlock;
const osThreadAttr_t hmi_attributes = {
  .name = "hmi",
  .stack_mem = &glcdDisplayBuffer[0],
  .stack_size = sizeof(glcdDisplayBuffer),
  .cb_mem = &glcdDisplayControlBlock,
  .cb_size = sizeof(glcdDisplayControlBlock),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for I2CManager */
osThreadId_t I2CManagerHandle;
uint32_t I2CManagerBuffer[ 256 ];
osStaticThreadDef_t I2CManagerControlBlock;
const osThreadAttr_t I2CManager_attributes = {
  .name = "I2CManager",
  .stack_mem = &I2CManagerBuffer[0],
  .stack_size = sizeof(I2CManagerBuffer),
  .cb_mem = &I2CManagerControlBlock,
  .cb_size = sizeof(I2CManagerControlBlock),
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for feeder */
osThreadId_t feederHandle;
const osThreadAttr_t feeder_attributes = {
  .name = "feeder",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for i2cDataQ */
osMessageQueueId_t i2cDataQHandle;
const osMessageQueueAttr_t i2cDataQ_attributes = {
  .name = "i2cDataQ"
};
/* USER CODE BEGIN PV */
extern PCA9698_t *active_line;	//this handle is updated by I2CMasterTxCmpltCallback
								//so that the the i2cTxCpltDeferred task can work
extern GLCD_t glcd_handle;
SemaphoreHandle_t weftSemphr;
EventGroupHandle_t i2cTxEvent;
QueueHandle_t xMailbox;

hmi_state select_flag=no_button;
int filecount;
uint16_t slave_Adresses_line1[2] = { 0x40, 0x42 };
uint16_t slave_Adresses_line2[2] = { 0x40, 0x42 };
uint16_t slave_Adresses_line3[2] = { 0x40, 0x42 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_RNG_Init(void);
static void MX_I2C2_Init(void);
void StartDefaultTask(void *argument);
void HMITask(void *argument);
void i2cTask(void *argument);
void feederTask(void *argument);

/* USER CODE BEGIN PFP */
//void feederTask(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_RNG_Init();
  MX_I2C2_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  weftSemphr = xSemaphoreCreateBinary();
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of i2cDataQ */
  i2cDataQHandle = osMessageQueueNew (100, sizeof(PCA9698_data_t), &i2cDataQ_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  xMailbox = xQueueCreate(1, sizeof(configPacket_t));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of hmi */
  hmiHandle = osThreadNew(HMITask, NULL, &hmi_attributes);

  /* creation of I2CManager */
  I2CManagerHandle = osThreadNew(i2cTask, NULL, &I2CManager_attributes);

  /* creation of feeder */
  feederHandle = osThreadNew(feederTask, NULL, &feeder_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  i2cTxEvent = xEventGroupCreate();
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Vbus_EN_Pin|GLCD_RS_Pin|GLCD_RW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |GLCD_DB0_Pin|GLCD_DB1_Pin|GLCD_DB2_Pin|GLCD_DB3_Pin
                          |GLCD_DB4_Pin|GLCD_DB5_Pin|GLCD_DB6_Pin|GLCD_DB7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GLCD_CS1_Pin|GLCD_CS2_Pin|GLCD_EN_Pin|GLCD_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Vbus_EN_Pin GLCD_RS_Pin GLCD_RW_Pin */
  GPIO_InitStruct.Pin = Vbus_EN_Pin|GLCD_RS_Pin|GLCD_RW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           GLCD_DB0_Pin GLCD_DB1_Pin GLCD_DB2_Pin GLCD_DB3_Pin
                           GLCD_DB4_Pin GLCD_DB5_Pin GLCD_DB6_Pin GLCD_DB7_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |GLCD_DB0_Pin|GLCD_DB1_Pin|GLCD_DB2_Pin|GLCD_DB3_Pin
                          |GLCD_DB4_Pin|GLCD_DB5_Pin|GLCD_DB6_Pin|GLCD_DB7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : GLCD_CS1_Pin GLCD_CS2_Pin GLCD_EN_Pin GLCD_RST_Pin */
  GPIO_InitStruct.Pin = GLCD_CS1_Pin|GLCD_CS2_Pin|GLCD_EN_Pin|GLCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_HOST */
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  /* send data packets to the i2cQ continuously */

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_HMITask */
/**
* @brief Function implementing the hmi thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HMITask */
void HMITask(void *argument)
{
  /* USER CODE BEGIN HMITask */
  /* Infinite loop */
  for(;;)
  {
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	switch(select_flag)
	{
		case down_button:
			if(glcd_handle.cursor_pos<=glcd_handle.cursor_max)
				GLCD_Cursor_update(incr);
			else//update new content
				;//refresh the content of the display filenames by reading from usb;
			select_flag=no_button;
			break;

		case up_button:
			if(glcd_handle.cursor_pos>=glcd_handle.cursor_min)
				GLCD_Cursor_update(decr);
			else//update new content
				;//refresh the content of the display filenames by reading from usb;
			select_flag=no_button;
			break;

		case select_button:
			GLCD_Clear_All();
			GLCD_printLine(1, glcd_handle.text_buf);
			GLCD_printLine(2, "Selected");
			select_flag=no_button;
			break;

		default:
			break;
	}
  }
  /* USER CODE END HMITask */
}

/* USER CODE BEGIN Header_i2cTask */
/**
* @brief Function implementing the pca9698_line1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_i2cTask */
void i2cTask(void *argument)
{
  /* USER CODE BEGIN i2cTask */
	I2C_HandleTypeDef *hi2cx[] = {&hi2c1, &hi2c2, &hi2c3};
	uint16_t *slave_adresses[] = {slave_Adresses_line1, slave_Adresses_line2, slave_Adresses_line3};
	const uint16_t maxPins = MAX_SLAVE_COUNT * PINS_PER_SLAVE;	//max no. of pins in 1 line
	uint16_t pinCount = 0, iter = 0;	//updated by mailbox parameters
	uint8_t activeSlaveCnt[3];	// the number of slaves active in the line (<=MAX_SLAVE_COUNT)
	uint8_t lineIndex = 0, linesActivated = 0;
	PCA9698_MultiRegBank *dataBasePointer[3];	//stores the i2c buffer addresses of different i2c lines
	for(;lineIndex<3;lineIndex++)
		dataBasePointer[lineIndex] = (PCA9698_MultiRegBank*) pvPortMalloc(sizeof(PCA9698_MultiRegBank)*MAX_SLAVE_COUNT);
	lineIndex = 0;

  /* Infinite loop */
  for(;;)
  {
	  if(iter==0)	//signifies weaving job complete
	  {
		  // 1. readMailbox --> wait for new weaving job parameters
		  configPacket_t params;
		  xQueueReceive(xMailbox, &params, portMAX_DELAY);
		  pinCount = params.pinsRequired;
		  iter = params.itersRequired;

		  // 2. reconfigure i2c line parameters
		  while(pinCount > 0)
		  {
			  if(pinCount >= maxPins)	//if-else block calculates the number of slaves required per line
			  {
				  activeSlaveCnt[lineIndex] = MAX_SLAVE_COUNT;
				  pinCount -= maxPins;
			  }
			  else
			  {
				  activeSlaveCnt[lineIndex] = ((pinCount % maxPins)/PINS_PER_SLAVE) + 1;
				  pinCount = 0;
			  }
			  PCA9698_Adv_Init(hi2cx[lineIndex], slave_adresses[lineIndex],
					  activeSlaveCnt[lineIndex], (uint8_t*)dataBasePointer[lineIndex]);
			  lineIndex++;
		  }
		  linesActivated = lineIndex; lineIndex = 0;
	  }

	  /* Fill/Re-fill the parallel i2c Buffers */
	  if(xEventGroupWaitBits(i2cTxEvent, 0x07, 0x07, pdTRUE, pdMS_TO_TICKS(2000)))
	  {
		  while(lineIndex <= linesActivated) //this loop makes sure data is read serially by different i2c buffers
		  {
			  PCA9698_MultiRegBank *dataBuffer = dataBasePointer[lineIndex];
			  for(uint8_t index = 0; index<activeSlaveCnt[lineIndex]; index++)
			  {
				  //write the command byte to the slave packet
				  dataBuffer->cmd = PCA9698_CMD_OP | PCA9698_CMD_AI_MSK;

				  //get the data from the dataQ
				  while(osMessageQueueGet(i2cDataQHandle, &(dataBuffer->fullData), NULL, 0) != osOK)
					  osDelay(1);	// block for a small time to allow feeder task to feed the packets to the Q
			  }
			  lineIndex++;
		  }
		  lineIndex = 0; //reset index
	  }
	  else
	  {
		  //previous i2c transmission faced some error?
	  }

	  xSemaphoreTake(weftSemphr, portMAX_DELAY); //--> wait for external button interrupt trigger
	  //task notification possible?
	  while(lineIndex <= linesActivated)
	  {
		  PCA9698_LineX_Write_Start(lineIndex);
		  lineIndex++;
	  }
	  lineIndex=0;
	  iter--;
  }
  /* USER CODE END i2cTask */
}

/* USER CODE BEGIN Header_feederTask */
/**
* @brief Function implementing the feeder thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_feederTask */
void feederTask(void *argument)
{
  /* USER CODE BEGIN feederTask */
	PCA9698_data_t dataBuffer;
	uint16_t iter = 0, config_done = 0, pkt_cnt = 0;
	UINT br = 0;
	configPacket_t params;
  /* Infinite loop */
  for(;;)
  {
	  	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);	//wait for task notification from HMI task
		f_read(&USBHFile, &(params), sizeof(params), &br);	//read 4 by: 2by for l & b each
		iter = params.itersRequired;
		pkt_cnt = (params.pinsRequired/PINS_PER_SLAVE);
		if(params.pinsRequired % PINS_PER_SLAVE)
			pkt_cnt++;

		while(iter != 0)
		{
			for(int i=0; i<pkt_cnt; i++)
			{
				if(config_done == 0)
				{
					config_done = 1;
					while(xQueueSendToBack(xMailbox, &params, 100) != pdPASS);
				}
				memset(&dataBuffer, 0, sizeof(dataBuffer));
				f_read(&USBHFile, &dataBuffer, sizeof(dataBuffer), &br); /* -- try to read 5 bytes at a time until EOF -- */
				while((osMessageQueuePut(i2cDataQHandle, &dataBuffer, 1, 10)) != osOK);
			}

			iter--;
		}
		config_done = 0;
  }
  /* USER CODE END feederTask */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
