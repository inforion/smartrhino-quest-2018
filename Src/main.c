/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"
#include "touchsensing.h"

/* USER CODE BEGIN Includes */
#include "stm32f0xx_hal_flash.h"
#include <string.h>
#include <stdlib.h>
#include "ws2812b.h"
#include "tinyprintf.h"

typedef struct {
	uint8_t* buffer;
	int length;
} message_t;

typedef struct {
	int idx;
	uint16_t hue;
	uint8_t sat;
	uint8_t val;
} led_control_t;

typedef struct {
	char* buf;
	int pos;
	int remain;
	int size;
} stream_t;

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim3_ch1_trig;

TSC_HandleTypeDef htsc;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

osThreadId ledsTaskHandle;
osThreadId bluetoothTaskHandle;
osThreadId sensorTaskHandle;
osThreadId watchdogTaskHandle;
osThreadId uartRxTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
osMailQDef(uartRxQueueHandle, 8, message_t);
osMailQId  uartRxQueueHandle;
osMailQDef(ledControlQueueHandle, 2, led_control_t);
osMailQId  ledControlQueueHandle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TSC_Init(void);
static void MX_IWDG_Init(void);
void leds_task_entry(void const * argument);
void bluetooth_task_entry(void const * argument);
void sensor_task_entry(void const * argument);
void watchdog_task_entry(void const * argument);
void uart_rx_task_entry(void const * argument);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
int is_auth = 0;
const uint32_t secret = 0x46f70674;
int auth_start = 0;

ws2812b_t ws2812b;

#define TOP_LEFT 0
#define TOP_RIGHT 1
#define BOTTOM_LEFT 2
#define BOTTOM_RIGHT 3

static void uart_putf(void *unused, char c) {
	HAL_StatusTypeDef status;
	do {
		status = HAL_UART_Transmit(&huart1, (uint8_t*)&c, 1, 1000);
	} while (status == HAL_BUSY);
}

static void uart_printf_enter(void) {
	osThreadSuspendAll();
}

static void uart_printf_exit(void) {
	osThreadResumeAll();
}

void unsafe_blinking_error(void) {
	osThreadSuspendAll();
	while(1) {
		int idx = rand() % 4;
		int hue = (rand() % 360) ;
		int val = (rand() % 255) ;

		HAL_IWDG_Refresh(&hiwdg);

		ws2812b_set_hsv(&ws2812b, idx, hue, 255, val);
		while (ws2812b_send(&ws2812b) == -1)
		HAL_IWDG_Refresh(&hiwdg);

		volatile int k = 0;
		for (k = 0; k < 10; k++) {
			HAL_IWDG_Refresh(&hiwdg);
			HAL_Delay(1);
		}

		ws2812b_set_hsv(&ws2812b, idx, hue, 255, 0);
		while (ws2812b_send(&ws2812b) == -1)
		HAL_IWDG_Refresh(&hiwdg);

		for (k = 0; k < 10; k++) {
			HAL_IWDG_Refresh(&hiwdg);
			HAL_Delay(1);
		}
	}
}

void safe_blinking_error(void) {
	taskDISABLE_INTERRUPTS();
	volatile int k = 0;
	while (1) {
		HAL_GPIO_TogglePin(LED_BTN_GPIO_Port, LED_BTN_Pin);
		for (k = 0; k < 1024 * 1024; k++)
			HAL_IWDG_Refresh(&hiwdg);
		HAL_GPIO_TogglePin(LED_BTN_GPIO_Port, LED_BTN_Pin);
		for (k = 0; k < 1024 * 1024; k++)
			HAL_IWDG_Refresh(&hiwdg);
	}
}

#define EEPROM_START_ADDRESS 0x08007C00

int flash_erase() {
	uint32_t PageError = 0;
	FLASH_EraseInitTypeDef desc;
	desc.PageAddress = EEPROM_START_ADDRESS;
	desc.NbPages = 1;
	desc.TypeErase = FLASH_TYPEERASE_PAGES;
	HAL_FLASH_Unlock();
	HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&desc, &PageError);
	HAL_FLASH_Lock();

	if (status != HAL_OK)
		return -1;

	return 0;
}

int flash_write(uint32_t address, const uint8_t* data, int size) {
	int k = 0;

    address = address + EEPROM_START_ADDRESS;
    HAL_FLASH_Unlock();

    for (k = 0; k < size; k += 2) {
    	uint16_t tmp = (data[k + 1] << 8) | data[k];
    	HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address + k, tmp);
//    	printf("address = %08lX stat = %d\r\n", address + k, status);
    	if (status != HAL_OK) {
    		return -1;
    	}
    }
    HAL_FLASH_Lock();

    return 0;
}

int flash_read(uint32_t address, uint8_t* data, int size) {
	memcpy(data, (void*) (address + EEPROM_START_ADDRESS), size);
	return 0;
}

int flash_compare(uint32_t address, uint8_t* data, int size) {
	return memcmp(data, (void*) (address + EEPROM_START_ADDRESS), size);
}

#define PASSWORD_CONST_LENGTH 8

int write_password(char* data) {
	if (strlen(data) != PASSWORD_CONST_LENGTH)
		return -1;
	int status = flash_erase();
	if (status < 0)
		return status;
	status = flash_write(0, (uint8_t*) data, PASSWORD_CONST_LENGTH);
	if (status < 0)
		return status;
	char end[2] = { 0 };
	status = flash_write(PASSWORD_CONST_LENGTH, (uint8_t*) end, 2);
	return status;
}

int compare_password(char* password) {
	if (strlen(password) != PASSWORD_CONST_LENGTH)
		return -1;

	return flash_compare(0, (uint8_t*) password, strlen(password));
}

int read_password(char* data) {
	memset(data, 0, PASSWORD_CONST_LENGTH);
	return flash_read(0, (uint8_t*) data, PASSWORD_CONST_LENGTH);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  init_printf(NULL, uart_putf, uart_printf_enter, uart_printf_exit);
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TSC_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  printf("\r\nHardware init done... Starting FreeRTOS\r\n");
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of ledsTask */
  osThreadDef(ledsTask, leds_task_entry, osPriorityNormal, 0, 96);
  ledsTaskHandle = osThreadCreate(osThread(ledsTask), NULL);

  /* definition and creation of bluetoothTask */
  osThreadDef(bluetoothTask, bluetooth_task_entry, osPriorityNormal, 0, 96);
  bluetoothTaskHandle = osThreadCreate(osThread(bluetoothTask), NULL);

  /* definition and creation of sensorTask */
  osThreadDef(sensorTask, sensor_task_entry, osPriorityNormal, 0, 96);
  sensorTaskHandle = osThreadCreate(osThread(sensorTask), NULL);

  /* definition and creation of watchdogTask */
  osThreadDef(watchdogTask, watchdog_task_entry, osPriorityNormal, 0, 64);
  watchdogTaskHandle = osThreadCreate(osThread(watchdogTask), NULL);

  /* definition and creation of uartRxTask */
  osThreadDef(uartRxTask, uart_rx_task_entry, osPriorityHigh, 0, 96);
  uartRxTaskHandle = osThreadCreate(osThread(uartRxTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
//  uartTxQueueHandle = osMailCreate(osMailQ(uartTxQueueHandle), NULL);
  uartRxQueueHandle = osMailCreate(osMailQ(uartRxQueueHandle), NULL);
  ledControlQueueHandle = osMailCreate(osMailQ(ledControlQueueHandle), NULL);
  /* USER CODE END RTOS_QUEUES */
 

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 3, 0);
}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Window = WDG_COUNTER;
  hiwdg.Init.Reload = WDG_COUNTER;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* TSC init function */
static void MX_TSC_Init(void)
{

    /**Configure the TSC peripheral 
    */
  htsc.Instance = TSC;
  htsc.Init.CTPulseHighLength = TSC_CTPH_2CYCLES;
  htsc.Init.CTPulseLowLength = TSC_CTPL_2CYCLES;
  htsc.Init.SpreadSpectrum = DISABLE;
  htsc.Init.SpreadSpectrumDeviation = 1;
  htsc.Init.SpreadSpectrumPrescaler = TSC_SS_PRESC_DIV1;
  htsc.Init.PulseGeneratorPrescaler = TSC_PG_PRESC_DIV4;
  htsc.Init.MaxCountValue = TSC_MCV_16383;
  htsc.Init.IODefaultMode = TSC_IODEF_IN_FLOAT;
  htsc.Init.SynchroPinPolarity = TSC_SYNC_POLARITY_FALLING;
  htsc.Init.AcquisitionMode = TSC_ACQ_MODE_NORMAL;
  htsc.Init.MaxCountInterrupt = DISABLE;
  htsc.Init.ChannelIOs = TSC_GROUP2_IO1;
  htsc.Init.ShieldIOs = 0;
  htsc.Init.SamplingIOs = TSC_GROUP2_IO2;
  if (HAL_TSC_Init(&htsc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_BTN_GPIO_Port, LED_BTN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_BTN_Pin */
  GPIO_InitStruct.Pin = LED_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_BTN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

message_t* createMessage(osMailQId queue, const char* data, int length) {
	message_t* message = osMailAlloc(queue, osWaitForever);
	message->length = length;
	message->buffer = (uint8_t*) calloc(length + 1, sizeof(char));
	memcpy(message->buffer, data, length);
	return message;
}

void sendMessage(osMailQId queue, const char* data, int length) {
	message_t* message = createMessage(queue, data, length);

	if (osOK != osMailPut(queue, message)) {
		printf("sendMsg error %s\r\n", message->buffer);
		Error_Handler();
	}
}

message_t* recvMessage(osMailQId queue, uint32_t milisec) {
	osEvent evt = osMailGet(queue, milisec);

	if (evt.status == osEventTimeout) {
		return NULL;
	}

	if (evt.status != osEventMail) {
		printf("recvMsg error\r\n");
		Error_Handler();
	}

	return (message_t*) evt.value.p;
}

void freeMessage(osMailQId queue, message_t* message) {
//	printf("freeMessage: message = %s\r\n", message->buffer);
	free(message->buffer);
	if (osOK != osMailFree(queue, message)) {
		printf("freeMsg error\r\n");
		Error_Handler();
	}
}

int tsc_acquisition_value(TSC_HandleTypeDef* htsc, int group_idx, int count) {
	int k = 0;
	int uhTSCAcquisitionValue = 0;

	for (k = 0; k < count; k++) {
		osThreadSuspendAll();
		HAL_TSC_IODischarge(htsc, ENABLE);
		HAL_TSC_Start(htsc);
		HAL_TSC_PollForAcquisition(htsc);
		if (HAL_TSC_GroupGetStatus(htsc, group_idx) == TSC_GROUP_COMPLETED) {
			uhTSCAcquisitionValue += HAL_TSC_GroupGetValue(htsc, group_idx);
		}
		HAL_TSC_Stop(htsc);
		HAL_TSC_IODischarge(htsc, DISABLE);
		osThreadResumeAll();
	}

	return uhTSCAcquisitionValue / k;
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	ws2812b_interrupt(&ws2812b);
}

int bt_recv(char* buffer, int size, uint32_t timeout) {
	message_t* message = recvMessage(uartRxQueueHandle, timeout);
	if (message == NULL)
		return -1;
	if (message->length > size) {
//		printf("bt_recv: Message too large buf_size = %d size = %d message = %sr\n",
//				size, message->length, message->buffer);
		Error_Handler();
	}
	memset(buffer, 0, size);
	memcpy(buffer, message->buffer, message->length);
	freeMessage(uartRxQueueHandle, message);
	return 0;
}

void bt_send(const char* buffer) {
	HAL_UART_Transmit(&huart2, (uint8_t*) buffer, strlen(buffer), 1000);
//	sendMessage(uartTxQueueHandle, buffer, strlen(buffer));
}

int bt_command(const char* command, char* response, int size, uint32_t timeout) {
	static int no = 0;
	printf("cmd[%d] %s\r\n", no, command);
	bt_send(command);
	int status = bt_recv(response, size, timeout);
	printf("rsp[%d] %s\r\n", no, response);
	no++;
	return status;
}

stream_t* stream_init(stream_t* stream, char* buf, int size) {
	stream->buf = buf;
	stream->size = size;
	stream->pos = 0;
	stream->remain = 0;
	return stream;
}

void stream_increase_remain(stream_t* stream, int size) {
	stream->remain += size;
}

stream_t* stream_create_from_buf(char* buf, int size) {
	stream_t* stream = (stream_t*) calloc(1, sizeof(stream));
	stream_init(stream, buf, size);
	stream_increase_remain(stream, size);
	return stream;
}

void stream_destroy(stream_t* stream) {
	free(stream);
}

void stream_destroy_with_buf(stream_t* stream) {
	free(stream->buf);
	stream_destroy(stream);
}

int stream_read_string(stream_t* stream, char* output, int size) {
	if (size > stream->remain)
		return -1;
	memcpy(output, stream->buf + stream->pos, size);
	stream->pos += size;
	stream->remain -= size;
	return 0;
}

char* stream_read_astring(stream_t* stream, int size) {
	char* buf = (char*) calloc(size + 1, sizeof(char));
	if (stream_read_string(stream, buf, size) < 0) {
		free(buf);
		return NULL;
	}
	return buf;
}

int stream_read_num(stream_t* stream, int size) {
	char* buf = stream_read_astring(stream, size);
	if (buf == NULL)
		return 0;
	int result = strtol(buf, NULL, 16);
	free(buf);
	return result;
}

uint8_t stream_read_byte(stream_t* stream) {
	return stream_read_num(stream, 2);
}

uint16_t stream_read_half(stream_t* stream) {
	return stream_read_num(stream, 4);
}

uint32_t stream_read_word(stream_t* stream) {
	return stream_read_num(stream, 8);
}

int circular_stream_lookup_char(stream_t* stream, char ch) {
	int k = 0;
	for (k = 0; k < stream->remain; k++) {
		int offset = stream->pos + k;
		uint16_t pos = offset % stream->size;
		char sch = stream->buf[pos];
		if (sch == ch)
			return offset;
	}
	return -1;
}

int circular_stream_skip(stream_t* stream, int count) {
	if (stream->remain < count)
		return -1;
	stream->pos += count;
	stream->remain -= count;
	return count;
}

char circular_stream_peek_char(stream_t* stream) {
	if (stream->remain == 0)
		return 0;
	uint16_t pos = stream->pos % stream->size;
	return stream->buf[pos];
}

char circular_stream_read_char(stream_t* stream) {
	char ch = circular_stream_peek_char(stream);
	if (ch != 0) {
		stream->pos += 1;
		stream->remain -= 1;
	}
	return ch;
}

int circular_stream_read_string(stream_t* stream, char* buf, int size) {
	int k = 0;
	for (k = 0; k < size; k++) {
		buf[k] = circular_stream_read_char(stream);
		if (buf[k] == 0)
			return -1;
	}
	return 0;
}

char* circular_stream_read_astring(stream_t* stream, int size) {
	char* buf = (char*) calloc(size + 1, sizeof(char));
	if (circular_stream_read_string(stream, buf, size) < 0) {
		free(buf);
		return NULL;
	}
	return buf;
}

char* circular_stream_read_astring_until(stream_t* stream, char ch) {
	int offset = circular_stream_lookup_char(stream, ch);
	if (offset == -1)
		return NULL;
	int size = offset - stream->pos;
	return circular_stream_read_astring(stream, size);
}

uint32_t adler32(const uint8_t *buf, size_t buflength) {
     uint32_t s1 = 1;
     uint32_t s2 = 0;

     while( buflength-- ) {
         s1 = ( s1 + *( buf++ ) ) % 65521;
         s2 = ( s2 + s1 ) % 65521;
     }
     return (s2 << 16) | s1;
}

uint32_t magic_salt() {
	return 0xFEE1DEAD;
}

void passgen(char* buf, int size) {
	const int start = 'A';
	const int end = 'Z';
	const int total = end + 1 - start;

	memset(buf, 0, size);

	/* Intializes random number generator */
	int seed = tsc_acquisition_value(&htsc, TSC_GROUP2_IDX, 32);
	srand(seed);

	for (int i = 0; i < size; i++) {
		buf[i] = (char) ((rand() % total) + start);
	}
	buf[size] = 0;
}

/* USER CODE END 4 */

/* leds_task_entry function */
void leds_task_entry(void const * argument)
{
  /* init code for TOUCHSENSING */
  MX_TOUCHSENSING_Init();

  /* USER CODE BEGIN 5 */

  printf("LED task\r\n");

//  flash_erase();

  	char pass[PASSWORD_CONST_LENGTH + 1] = { 0 };
  	read_password(pass);

  	if (pass[0] < 'A' || pass[0] > 'Z') {
  		osThreadSuspendAll();

  		int erase_status = flash_erase();
  		printf("erase=%d\r\n", erase_status);
  		passgen(pass, PASSWORD_CONST_LENGTH);
  		printf("gen=%s\r\n", pass);
  		int password_status = write_password(pass);
  		printf("flash=%d\r\n", password_status);

  		printf("reset\r\n");
  		while (1);
  	}

  	printf("[%s]\r\n", (char*) EEPROM_START_ADDRESS);

	if (ws2812b_init(&ws2812b, 4, &htim3, TIM_CHANNEL_1) < 0) {
//		printf("ws2812b driver init failure...\r\n");
		Error_Handler();
	}

	ws2812b_send_all_rgb_wait(&ws2812b, 20, 0, 0);
	osDelay(300);

	ws2812b_send_all_rgb_wait(&ws2812b, 0, 20, 0);
	osDelay(300);

	ws2812b_send_all_rgb_wait(&ws2812b, 0, 0, 20);
	osDelay(300);

	ws2812b_send_hsv_wait(&ws2812b, 0,   0, 255, 10);
	ws2812b_send_hsv_wait(&ws2812b, 1,  90, 255, 10);
	ws2812b_send_hsv_wait(&ws2812b, 2, 180, 255, 10);
	ws2812b_send_hsv_wait(&ws2812b, 3, 270, 255, 10);

	int16_t shift = 90;
	int16_t step = -10;
	uint8_t locked[4] = { 0 };
	uint8_t shine = 1;
	uint16_t delay = 15;

	for (;;) {
		osEvent evt = osMailGet(ledControlQueueHandle, 1000);
		if (evt.status == osEventMail) {
			led_control_t* led = (led_control_t*) evt.value.p;
			if (led->idx == 0xB816D8D9) {
				printf("I've got a super power and now I'm seeing invisible tactical combatant nano-ants everywhere\r\n");
				printf("I'm crazy rhino: www.youtube.com/watch?v=R6Rf5tyc4rA\r\n");
				printf("Thou should see them too I'm enable this super power for you\r\n");
				printf("!!!MaD bLiNkInG should begin\r\n");
				unsafe_blinking_error();
			}
			// unlock if hue is invalid
			if (led->hue < 360) {
				locked[led->idx] = 1;
				ws2812b_send_hsv_wait(&ws2812b, led->idx, led->hue, led->sat, led->val);
			} else {
				printf("hue > max: change shine\r\n");
				if (led->hue == 520) {
					shine = 0;
				} else if (led->hue == 720) {
					shine = 1;
				} else if (led->hue == 920) {
					locked[led->idx] = 0;
				} else if (led->hue > 1000) {
					delay = led->hue / 100;
				}
			}
			osMailFree(ledControlQueueHandle, led);
		} else if (evt.status == osEventTimeout) {
			if (shine == 1) {
				int hue = 0;
				for (hue = 0; hue < 360; hue += 5) {
					if (!locked[0]) ws2812b_set_hsv(&ws2812b, 0, hue + shift * 1, 255, 20);
					if (!locked[1]) ws2812b_set_hsv(&ws2812b, 1, hue + shift * 2, 255, 20);
					if (!locked[2]) ws2812b_set_hsv(&ws2812b, 2, hue + shift * 3, 255, 20);
					if (!locked[3]) ws2812b_set_hsv(&ws2812b, 3, hue + shift * 4, 255, 20);
					ws2812b_send_wait(&ws2812b);
					osDelay(delay);
				}
				if (!locked[0]) ws2812b_set_hsv(&ws2812b, 0,   0, 255, 10);
				if (!locked[1]) ws2812b_set_hsv(&ws2812b, 1,  90, 255, 10);
				if (!locked[2]) ws2812b_set_hsv(&ws2812b, 2, 180, 255, 10);
				if (!locked[3]) ws2812b_set_hsv(&ws2812b, 3, 270, 255, 10);
				ws2812b_send_wait(&ws2812b);
				shift += step;
				if (shift == 0)
					step *= -1;
			}
		} else {
			Error_Handler();
		}
	}
  /* USER CODE END 5 */ 
}

/* bluetooth_task_entry function */
void bluetooth_task_entry(void const * argument)
{
  /* USER CODE BEGIN bluetooth_task_entry */
#define MAX_MESSAGE_SIZE 64
#define STATE_DISCONNECTED 0
#define STATE_CONNECTED 2
#define STATE_BYPASS 3

	printf("Bluetooth task\r\n");

	char* response = (char*) calloc(MAX_MESSAGE_SIZE + 1, sizeof(char));
	int state = STATE_DISCONNECTED;
	int echo_enabled = 0;

	bt_command("^#^$^%", response, MAX_MESSAGE_SIZE, osWaitForever);

	while (bt_recv(response, MAX_MESSAGE_SIZE, 1000) != -1) {
		printf("leftovers %s\r\n", response);
	}

	bt_command("AT+AB ShowConnection\r", response, MAX_MESSAGE_SIZE, osWaitForever);
	if (strstr(response, "No device connected") == NULL) {
//		printf("bt disconnect peers\r\n");
		bt_command("AT+AB SPPDisconnect\r", response, MAX_MESSAGE_SIZE, osWaitForever);
	}

//	bt_command("AT+AB ReadClock\r", response, MAX_MESSAGE_SIZE, osWaitForever);
	bt_command("AT+AB DefaultLocalName RHINOCEROS-X\r", response, MAX_MESSAGE_SIZE, osWaitForever);
	bt_command("AT+AB Config COD = 000000\r", response, MAX_MESSAGE_SIZE, osWaitForever);
	bt_command("AT+AB EnableBond\r", response, MAX_MESSAGE_SIZE, osWaitForever);
//	bt_command("AT+AB ShowDev\r", response, BUFFER_MESSAGE_SIZE, osWaitForever);

  /* Infinite loop */
	printf("infinite loop\r\n");
  for(;;)
  {
//	  bt_command("AT+AB ReadClock\r", response, MAX_MESSAGE_SIZE, osWaitForever);
	  bt_recv(response, MAX_MESSAGE_SIZE, osWaitForever);
	  printf("recv %s state %d\r\n", response, state);
	  if (state == STATE_DISCONNECTED) {
		  if (strstr(response, "AT-AB ConnectionUp") == response) {
			  printf("state connected\r\n");
			  state = STATE_CONNECTED;
		  }
	  } else if (state == STATE_CONNECTED) {
		  if (strstr(response, "AT-AB ConnectionDown") == response) {
			  printf("state disconnected\r\n");
			  state = STATE_DISCONNECTED;
		  } else if (strstr(response, "AT-AB -BypassMode-") == response) {
			  printf("state bypass\r\n");
			  state = STATE_BYPASS;
		  } else {
			  // ignored
		  }
	  } else if (state == STATE_BYPASS) {
		  if (strstr(response, "AT-AB ConnectionDown") == response) {
			  printf("state disconnected\r\n");
			  state = STATE_DISCONNECTED;
			  continue;
		  }

		  int len = strlen(response);
		  if (echo_enabled) {
			  response[len+0] = '\r';
			  response[len+1] = '\n';
			  bt_send(response);
		  }

		  if (len < 4) {
			  bt_send("ERROR: Wrong header length\r\n");
			  continue;
		  }

//		  printf("resp: %s\r\n", response);

		  stream_t* stream = stream_create_from_buf(response, len);
		  char* cmd = stream_read_astring(stream, 4);

		  printf("cmd: %s\r\n", cmd);

		  if (strcmp(cmd, "ECH1") == 0) {
//		      printf("enable echo mode\r\n");
			  echo_enabled = 1;
			  bt_send("OK\r\n");
		  } else if (strcmp(cmd, "ECH0") == 0) {
//		      printf("disable echo mode\r\n");
			  echo_enabled = 0;
			  bt_send("OK\r\n");
		  } else if (strcmp(cmd, "MEOW") == 0) {
			  printf("mow?\r\n");
			  bt_send("mur-mur (>._.<)\r\n");
		  } else if (strcmp(cmd, "LED ") == 0) {
			  led_control_t* led = osMailAlloc(ledControlQueueHandle, osWaitForever);
			  led->idx = stream_read_byte(stream) % 4;
			  led->hue = stream_read_half(stream);
			  led->sat = stream_read_byte(stream);
			  led->val = stream_read_byte(stream);
			  printf("led idx %d hue %d sat %d val %d\r\n", led->idx, led->hue, led->sat, led->val);
			  osMailPut(ledControlQueueHandle, led);
			  bt_send("OK\r\n");
		  } else if (strcmp(cmd, "UART") == 0) {
			  int size = stream_read_byte(stream);
			  char* msg = stream_read_astring(stream, size);
			  printf("msg %s\r\n", msg);
			  free(msg);
			  bt_send("OK\r\n");
		  } else if (strcmp(cmd, "BLE ") == 0) {
			  HAL_GPIO_WritePin(LED_BTN_GPIO_Port, LED_BTN_Pin, GPIO_PIN_SET);
			  bt_send("OK\r\n");
		  } else if (strcmp(cmd, "READ") == 0) {
			  if (is_auth == 1) {
				  int address = stream_read_word(stream);
				  int size = stream_read_half(stream);
				  printf("addr=%x, size=%x\r\n", address, size);
				  if (size > 58) {
					  bt_send("ERROR: Too much!\r\n");
				  } else {
					  strcpy(response, "OK: ");
					  memcpy(response, (void*) address, size);
					  strcpy(response + size, "\r\n");
					  bt_send(response);
				  }
			  } else {
				  bt_send("ERROR: Not auth!\r\n");
			  }
		  } else if (strcmp(cmd, "WRIT") == 0) {
			  if (is_auth == 1) {
				  int address = stream_read_word(stream);
				  int size = stream_read_half(stream);
				  char* data = stream_read_astring(stream, size);
				  memcpy((void*) address, data, size);
				  free(data);
				  bt_send("OK\r\n");
			  } else {
				  bt_send("ERROR: Not auth!\r\n");
			  }
		  } else if (strcmp(cmd, "AUTP") == 0) {
			  if (is_auth == 0) {
				  char* bt_pass = stream_read_astring(stream, PASSWORD_CONST_LENGTH);
				  if (compare_password(bt_pass) == 0) {
					  printf("User auth pass %s\r\n", bt_pass);
					  is_auth = 1;
    				  auth_start = HAL_GetTick();
					  bt_send("OK\r\n");
				  } else {
					  bt_send("ERROR: auth error\r\n");
				  }
				  free(bt_pass);
			  } else {
				  bt_send("ERROR: Already auth!\r\n");
			  }
		  } else if (strcmp(cmd, "SETP") == 0) {
			  if (is_auth == 1) {
				  char* password = stream_read_astring(stream, PASSWORD_CONST_LENGTH);
				  write_password(password);
				  printf("pass set: %s\r\n", password);
				  free(password);
				  bt_send("OK\r\n");
			  } else {
				  bt_send("ERROR: Not auth!\r\n");
			  }
		  } else if (strcmp(cmd, "VIP ") == 0) {
			  int sz = stream->remain;
			  char* password = stream_read_astring(stream, sz);
			  uint32_t hash = adler32((uint8_t*) password, sz);
			  if (hash == secret) {
				  bt_send("Congrats amigo!\r\n");
				  led_control_t* led = osMailAlloc(ledControlQueueHandle, osWaitForever);
				  led->idx = hash ^ magic_salt();
				  led->hue = 0x00E1;
				  led->sat = 0xBA;
				  led->val = 0xAD;
				  osMailPut(ledControlQueueHandle, led);
			  } else {
				  bt_send("Wrong won't give up!\r\n");
			  }
		  } else {
			  bt_send("ERROR: Unk cmd\r\n");
		  }

		  free(cmd);
		  free(stream);
	  }
  }
  /* USER CODE END bluetooth_task_entry */
}

/* sensor_task_entry function */
void sensor_task_entry(void const * argument)
{
  /* USER CODE BEGIN sensor_task_entry */
	printf("Sensor task\r\n");

  HAL_TSC_Init(&htsc);

  // don't set large count (> 48) watchdog may not be in time
  int threshold = tsc_acquisition_value(&htsc, TSC_GROUP2_IDX, 128) / 2;
  printf("TSC %d\r\n", threshold);

  int prv_auth = is_auth;
  int press_start = 0;
  int press_time = 0;

  for(;;)
  {
	    int uhTSCAcquisitionValue = tsc_acquisition_value(&htsc, TSC_GROUP2_IDX, 32);
//		printf("uhTSCAcquisitionValue = %d\r\n", uhTSCAcquisitionValue);
		if (uhTSCAcquisitionValue < threshold) {
			HAL_GPIO_WritePin(LED_BTN_GPIO_Port, LED_BTN_Pin, GPIO_PIN_SET);
			press_time = HAL_GetTick() - press_start;
			if (press_time > 1000) {
				if (is_auth == 0) {
					printf("AUTH BTN\r\n");
					osThreadSuspendAll();
					auth_start = HAL_GetTick();
					prv_auth = is_auth;
					is_auth = 1;
					osThreadResumeAll();
				}
			} else {

			}
		} else {
			GPIO_PinState state = (is_auth == 0) ? GPIO_PIN_RESET : GPIO_PIN_SET;
			HAL_GPIO_WritePin(LED_BTN_GPIO_Port, LED_BTN_Pin, state);
			press_start = HAL_GetTick();
		}

		if (is_auth && HAL_GetTick() - auth_start > 60000) {
			printf("SET AUTH %d\r\n", prv_auth);
			is_auth = prv_auth;
		}
//		osDelay(10);
  }
  /* USER CODE END sensor_task_entry */
}

/* watchdog_task_entry function */
void watchdog_task_entry(void const * argument)
{
  /* USER CODE BEGIN watchdog_task_entry */
  /* Infinite loop */
  for(;;)
  {
    HAL_IWDG_Refresh(&hiwdg);
  }
  /* USER CODE END watchdog_task_entry */
}

/* uart_rx_task_entry function */
void uart_rx_task_entry(void const * argument)
{
  /* USER CODE BEGIN uart_rx_task_entry */
	printf("UART task\r\n");

	stream_t local_stream;
	const int UART_RX_BUFFER_SIZE = 128;
	char* dma_buf = calloc(UART_RX_BUFFER_SIZE + 1, sizeof(char));
    stream_t* stream = stream_init(&local_stream, dma_buf, UART_RX_BUFFER_SIZE);

    HAL_UART_Receive_DMA(&huart2, (uint8_t*) stream->buf, stream->size);

    volatile int cur_remain = 0;
    volatile int prv_remain = stream->size;

    for(;;)
    {
    	osDelay(WDG_PERIOD_MS / 4);

    	// get current remain from DMA buffer
    	cur_remain = huart2.hdmarx->Instance->CNDTR;

    	// check if something incomming
    	int addend = 0;
		if (cur_remain > prv_remain) {
			int size1 = prv_remain;
			int size2 = stream->size - cur_remain;
			addend = size1 + size2;
		} else if (cur_remain < prv_remain) {
			addend = prv_remain - cur_remain;
		} else {
			continue;
		}
		stream_increase_remain(stream, addend);

		int start_tick = HAL_GetTick();

		// processing messages
		while (1) {
			// filter out all bad characters
			while (circular_stream_peek_char(stream) == '\0') {
				if (circular_stream_skip(stream, 1) == -1) {
					break;
				}
			}

			char* msg = circular_stream_read_astring_until(stream, '\r');
//			printf("uart_rx: msg=%s\r\n", msg);
			if (msg == NULL)
				break;

//			printf("bt: %s\r\n", msg);
			sendMessage(uartRxQueueHandle, msg, strlen(msg));
			free(msg);

			// get rid of '\r' char
			circular_stream_read_char(stream);
			// drop next char if it is '\n'
			if ('\n' == circular_stream_peek_char(stream)) {
				circular_stream_read_char(stream);
			}

			// give some time to others (watchdog reset)
			if (HAL_GetTick() - start_tick > WDG_PERIOD_MS / 4) {
				osDelay(WDG_PERIOD_MS / 4);
				start_tick = HAL_GetTick();
			}
		}

		prv_remain = cur_remain;
    }
  /* USER CODE END uart_rx_task_entry */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	safe_blinking_error();
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//	printf("assertion failed: file %s on line %d\r\n", file, line);
	safe_blinking_error();
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
