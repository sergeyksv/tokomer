/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <stdbool.h> 

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_DMA_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 1000000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
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
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 719;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DISPDC_Pin|DISPRES_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RANGE3_GPIO_Port, RANGE3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RANGE2_Pin|ONEKLOAD_Pin|RANGE1_Pin|EN_VOUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DISPDC_Pin DISPRES_Pin */
  GPIO_InitStruct.Pin = DISPDC_Pin|DISPRES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : INA_ALERT_Pin */
  GPIO_InitStruct.Pin = INA_ALERT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INA_ALERT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RANGE3_Pin RANGE2_Pin ONEKLOAD_Pin RANGE1_Pin */
  GPIO_InitStruct.Pin = RANGE3_Pin|RANGE2_Pin|ONEKLOAD_Pin|RANGE1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : EN_VOUT_Pin */
  GPIO_InitStruct.Pin = EN_VOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EN_VOUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY1_Pin KEY2_Pin KEY3_Pin */
  GPIO_InitStruct.Pin = KEY1_Pin|KEY2_Pin|KEY3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}

/* USER CODE BEGIN 4 */
osThreadId osMainThreadId;
osThreadId osUpdateScreenThreadId;
osThreadId osflushUsbBufferThreadId;

uint8_t buf[3];
int16_t inaRes;
void USR_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  inaRes = buf[1]|buf[0]<<8;
  osSignalSet (osMainThreadId, 0x2);  
}

void USR_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
  osSignalSet (osMainThreadId, 0x2);  
}

void USR_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  osSignalSet (osMainThreadId, 0x1);
}

int32_t  miliVolts =0;
int32_t  microAmps =0;
uint64_t sumBusMillVolts =         0;
uint64_t sumBusMillVoltsOrig =     0;
int32_t  maxBusMillVolts =         0;
int32_t  minBusMillVolts =         0;
int64_t  sumBusMicroAmps =         0;
int32_t  minBusMicroAmps =         0;
int32_t  maxBusMicroAmps =         0;
int64_t  sumBusMicroAmpsOrig  =    0;
int64_t  totalBusMicroAmps    =    0;
uint32_t readings             =    0;
int16_t  zero             = 11;
uint8_t  skip             = 0;
uint8_t  power            = 0;
uint8_t  overload         = 0;
uint8_t  power_state      = 0; 
uint8_t  ina226           = 0;
uint8_t  ina226_state     = 0;
uint8_t  range            = 3;
uint8_t  range_last       = 3;
uint8_t  minRange         = 0;
bool serialEnable = false;
uint16_t ranges[4]={0,128,1222,10683};
uint8_t rangeScale=0;
uint32_t rangeScales[4][4]={{11000,12000,110000,120000},
                           {5500,6000,55000,60000},
                           {2750,3000,27500,30000},
                           {1100,1200,11000,12000},
                          };
uint16_t voltageK = 17000;
uint16_t refreshT = 500;
uint8_t  rangeRiseT = 3; // with 10k/4.7k rise time is ~200us, at that time measurment can'be valid
                         // our cycle now is eaxactly 200us (rangeRiseT=1)
                         // because change can happen unaligned to mesurment cycle we need to wait 2T
                         // but for safety probably better 3T

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_2)
  {
    // shunt overvoltage, don't think, try to set range 3 right away
    HAL_GPIO_WritePin(RANGE3_GPIO_Port, RANGE3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RANGE2_GPIO_Port, RANGE2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RANGE1_GPIO_Port, RANGE1_Pin, GPIO_PIN_RESET);
    range = 3;  
    skip=rangeRiseT;  
  } 
}

uint64_t lsumBusMillVolts;
uint64_t lsumBusMillVoltsOrig;
int32_t  lmaxBusMillVolts;
int32_t  lminBusMillVolts;
int64_t  lsumBusMicroAmps;
int32_t  lmaxBusMicroAmps;
int32_t  lminBusMicroAmps;
int64_t  lsumBusMicroAmpsOrig;
int64_t  ltotalBusMicroAmps;
uint32_t lreadings=1;
int32_t  lnow;

#include "display_thread.h"
osThreadDef (updateScreen, updateScreenX, osPriorityIdle, 1, 200); 

// using 2 buffers to pack data to best suite for 64 byte CDC blocks
#define USB_SERIAL_TX_BUF_SIZE 64
char	bufUsb[2][USB_SERIAL_TX_BUF_SIZE];
uint16_t usbLen=0;
uint8_t usbPage=0;
uint8_t* usbToSendBuf=NULL;
uint16_t usbToSendSize=0;

// flush buffer thread, send chunks upon fullfilment
void flushUsbBuffer(void const *arg) {
  osflushUsbBufferThreadId = osThreadGetId();

  for(;;)
  {
    osSignalWait(0x1,1000);    
    CDC_Transmit_FS(usbToSendBuf,usbToSendSize);    
  }
}
osThreadDef (flushUsbThread, flushUsbBuffer, osPriorityBelowNormal, 1, 200); 

#include <math.h>
#include "eeprom.h"

uint16_t VirtAddVarTab[NumbOfVar] = {0x1700, 0x1701, 0x1702, 0x1703};

// mode 1 - calibration, measure shunt and voltage with 1ms conversion
// mode 0 - measuring current every 200ms 5khz
void configureINA226Mode(uint8_t mode) {
  buf[0]=0; buf[1]=0x40; buf[2]=0;
  // configure ina
  // mode: 0 - off, 1 - shunt, 2 - bus, 3 - both, 4 - shutdown, 5 - cont.shunt, 6 - cont.bus, 7 - cont.both
  buf[2] = mode==1?7:5; // in mode 1 we measure bith 
  // shunt conversion time:  0 - 140, 1 - 204, 2 - 322, 3 - 588, 4 - 1100, 5 - 2116, 6 - 4156, 7 - 8244  
  buf[2] |= (mode==1?4:1)<<3;  
  // bus conversion time:  0 - 140, 1 - 204, 2 - 332, 3 - 588, 4 - 1100, 5 - 2116, 6 - 4156, 7 - 8244  
  buf[2] |= (mode==1?0:1)<<6;   
  buf[1] |= (mode==1?1:0)>>2;   
  // averaging: 0 - 1, 1 - 4, 2 - 16, 3 - 64, 4 - 128, 5 -256, 6 - 512, 7 - 1024
  // buf[1] = 0x1<<1;
  HAL_I2C_Master_Transmit_IT(&hi2c1,0x80,buf,3);
  osSignalWait(0x2,10);

  // preselect current shunt register after configuration
  buf[0]=1;
  HAL_I2C_Master_Transmit_IT(&hi2c1,0x80,buf,1);
  osSignalWait(0x2,10); 
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  osMainThreadId = osThreadGetId ();
  osThreadCreate(osThread (updateScreen), NULL);  
  osThreadCreate(osThread (flushUsbThread), NULL);    
  HAL_TIM_Base_Start_IT(&htim3);

  HAL_FLASH_Unlock();

  EE_Init();

  //uint16_t ee_val;
  //EE_ReadVariable(0x1700,&ee_val); zero = ee_val;
  //EE_ReadVariable(0x1701,&ranges[2]);
  //EE_ReadVariable(0x1702,&ranges[3]);  
  //EE_ReadVariable(0x1703,&voltageK);  

  /* Infinite loop */
  char*  pageBuf=bufUsb[usbPage];

  range = 3;
  HAL_GPIO_WritePin(RANGE1_GPIO_Port, RANGE1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RANGE2_GPIO_Port, RANGE2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RANGE3_GPIO_Port, RANGE3_Pin, GPIO_PIN_SET);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  // send INA226 reset
  buf[0]=0; buf[1]=0x80; buf[2]=0x00;
  HAL_I2C_Master_Transmit(&hi2c1,0x80,buf,3,100);
  osDelay(1);

  // configure ina
  configureINA226Mode(0);
  osDelay(1);
  // set SOL flag (shut overvoltage)
  buf[0]=6; buf[1] = 0x80; buf[2]=0;
  HAL_I2C_Master_Transmit(&hi2c1,0x80,buf,3,100);
  // set Alert limit to maxium positive value -1
  buf[0]=7; buf[1] = 0x7F; buf[2]=0xFE;
  HAL_I2C_Master_Transmit(&hi2c1,0x80,buf,3,100);

  // select shunt raw register
  buf[0]=1;
  HAL_I2C_Master_Transmit(&hi2c1,0x80,buf,1,100);

  HAL_I2C_RegisterCallback(&hi2c1,HAL_I2C_MASTER_RX_COMPLETE_CB_ID,&USR_I2C_MasterRxCpltCallback);
  HAL_I2C_RegisterCallback(&hi2c1,HAL_I2C_MASTER_TX_COMPLETE_CB_ID,&USR_I2C_MasterTxCpltCallback);
  HAL_TIM_RegisterCallback(&htim3,HAL_TIM_PERIOD_ELAPSED_CB_ID,&USR_TIM_PeriodElapsedCallback);

  int cicleCounter =0;
  int32_t  absMicroAmps =0;

  for(;;)
  {
    osSignalWait(0x1,10);
    HAL_ADC_Start(&hadc1);

    if (ina226!=ina226_state) {
      ina226_state = ina226;
      configureINA226Mode(ina226_state);
    }
    if (ina226_state==1) {
      buf[0]=2;
      HAL_I2C_Master_Transmit_IT(&hi2c1,0x80,buf,1);    
      osSignalWait(0x2,10);
      HAL_I2C_Master_Receive_IT(&hi2c1,0x80,buf,2);
      osSignalWait(0x2,10);
      sumBusMillVoltsOrig += (uint16_t)inaRes+(((uint16_t)inaRes)>>2);
      buf[0]=1;
      HAL_I2C_Master_Transmit_IT(&hi2c1,0x80,buf,1);        
      osSignalWait(0x2,10);
    }

	  HAL_I2C_Master_Receive_IT(&hi2c1,0x80,buf,2);
    osSignalWait(0x2,10);
    if (skip==0) {
      // virtual current breaker, on max range max values, STOP!
      if (range==3 && (inaRes>=0x7fff || inaRes<=-0x7fff)) {
        power = 0;
        overload = true;
      }
      // we might need to skeep measurement which happened when we switch ranges
      microAmps = inaRes;
      microAmps = (microAmps*ranges[range])>>8;
//      microAmps = round((float)microAmps/100);
    } 
    else 
       skip--;

    // adjust range
    if (power!=power_state) {
      HAL_GPIO_WritePin(EN_VOUT_GPIO_Port, EN_VOUT_Pin, power?GPIO_PIN_SET:GPIO_PIN_RESET); 
      power_state=power;
    }   
    absMicroAmps = abs(microAmps);

    // to put into right range when forced fake range control current
    if (minRange!=0) {
      if (minRange==2)
        absMicroAmps = 13000;
      else
        absMicroAmps = 130000;
    }
    
    // range 1 locks if current < 11 ma
    if (range!=1 && absMicroAmps < rangeScales[rangeScale][0]) {
          HAL_GPIO_WritePin(RANGE1_GPIO_Port, RANGE1_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(RANGE2_GPIO_Port, RANGE2_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(RANGE3_GPIO_Port, RANGE3_Pin, GPIO_PIN_RESET);
          range = 1;
          skip=rangeRiseT;
    }
    
    // range 2 lock if current grows above 12 ma or reduces less than 110    
    if (range!=2 && absMicroAmps > rangeScales[rangeScale][1] && absMicroAmps < rangeScales[rangeScale][2]) {
          HAL_GPIO_WritePin(RANGE2_GPIO_Port, RANGE2_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(RANGE1_GPIO_Port, RANGE1_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(RANGE3_GPIO_Port, RANGE3_Pin, GPIO_PIN_RESET);
          range = 2;
          skip=rangeRiseT;
    }
    
    // range 3 locks for current bigger than 120 ma
    if (range!=3 && absMicroAmps > rangeScales[rangeScale][3]) {
      HAL_GPIO_WritePin(RANGE3_GPIO_Port, RANGE3_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(RANGE2_GPIO_Port, RANGE2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(RANGE1_GPIO_Port, RANGE1_Pin, GPIO_PIN_RESET);
      range = 3;  
      skip=rangeRiseT;      
    }

    sumBusMicroAmpsOrig += microAmps; // microAmps;
    microAmps-=zero;
    // update current accumulators
    if (readings==0) {
      minBusMicroAmps = maxBusMicroAmps = microAmps;
    } else {
      if (microAmps<minBusMicroAmps) 
        minBusMicroAmps = microAmps;
      if (microAmps>maxBusMicroAmps)
        maxBusMicroAmps = microAmps;
    }
    sumBusMicroAmps += microAmps;     // Add current value to sum
    totalBusMicroAmps += (microAmps * 0x333)>>12; // 0.2;
    miliVolts = (HAL_ADC_GetValue(&hadc1)*voltageK)>>12; // 18000/4096;
    sumBusMillVolts += miliVolts; // 14970 = 6 * 3000 (3.0000 v)    

    // update voltage accumulators
    if (readings==0) {
      minBusMillVolts = maxBusMillVolts = miliVolts;
    } else {
      if (miliVolts<minBusMillVolts) 
        minBusMillVolts = miliVolts;
      if (miliVolts>maxBusMillVolts)
        maxBusMillVolts = miliVolts;
    }
    readings++;    

    if (serialEnable) {
      itoa(microAmps,pageBuf,10);
      uint8_t len=strlen(pageBuf);
      pageBuf += len;
      *(pageBuf++)=',';
      usbLen+=len+1;
      itoa(miliVolts/10,pageBuf,10);
      len=strlen(pageBuf);
      pageBuf += len;
      *(pageBuf++)='\n';    
      usbLen+=len+1;

      // 14 is maxium line size that we can send -1800000,1200n
      // so as soon as have less than that we need to send them out
      if (usbLen>USB_SERIAL_TX_BUF_SIZE-14) {
        usbToSendBuf = (uint8_t *)bufUsb[usbPage];
        usbToSendSize = usbLen;
        // signalize that we have some data to send
        osSignalSet (osflushUsbBufferThreadId, 0x1);        
        usbPage = (usbPage+1)%2; 
        usbLen = 0;
        pageBuf = bufUsb[usbPage];
      }
    }

    if ((cicleCounter++)>=refreshT) {
      // make data snapshot
      lsumBusMillVoltsOrig=sumBusMillVoltsOrig;
      lsumBusMillVolts=sumBusMillVolts;
      lminBusMillVolts=minBusMillVolts;
      lmaxBusMillVolts=maxBusMillVolts;            
      lsumBusMicroAmps=sumBusMicroAmps;
      lminBusMicroAmps=minBusMicroAmps;
      lmaxBusMicroAmps=maxBusMicroAmps;
      lsumBusMicroAmpsOrig=sumBusMicroAmpsOrig;
      ltotalBusMicroAmps=totalBusMicroAmps;
      lreadings=readings;
      if (overload==0)
        lnow+=100;

      // reset readings
      readings = 0;
      sumBusMillVolts = 0;
      sumBusMicroAmps = 0;
      sumBusMicroAmpsOrig = 0;      
      sumBusMillVoltsOrig = 0;

      // signaling udpate screen thread
      osSignalSet (osUpdateScreenThreadId, 0x1);
      cicleCounter=0;

      // blink with led if range was changed
      if (range!=range_last) { 
        range_last=range;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
      } else {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
      }
      
    }
  }
  /* USER CODE END 5 */ 
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
