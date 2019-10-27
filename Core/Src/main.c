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
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
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
  htim3.Init.Period = 14;
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
  HAL_GPIO_WritePin(GPIOB, RANGE2_Pin|RANGE1_Pin, GPIO_PIN_RESET);

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
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : INA_ALERT_Pin */
  GPIO_InitStruct.Pin = INA_ALERT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INA_ALERT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RANGE3_Pin RANGE2_Pin RANGE1_Pin */
  GPIO_InitStruct.Pin = RANGE3_Pin|RANGE2_Pin|RANGE1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN_LEFT_Pin BTN_SEL_Pin BTN_UP_Pin BTN_RIGHT_Pin 
                           BTN_DOWN_Pin */
  GPIO_InitStruct.Pin = BTN_LEFT_Pin|BTN_SEL_Pin|BTN_UP_Pin|BTN_RIGHT_Pin 
                          |BTN_DOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
osThreadId osMainThreadId;
osThreadId osUpdateScreenThreadId;
uint8_t buf[3];
int16_t inaRes;
void USR_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  inaRes = buf[1]|buf[0]<<8;
  osSignalSet (osMainThreadId, 0x2);  
}

void USR_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  osSignalSet (osMainThreadId, 0x1);
}

int32_t  miliVolts =0;
int32_t  microAmps =0;
uint64_t sumBusMillVolts =         0;
int64_t  sumBusMicroAmps =         0;
int32_t  minBusMicroAmps =         0;
int32_t  maxBusMicroAmps =         0;
int64_t  sumBusMicroAmpsOrig  =    0;
int64_t  totalBusMicroAmps    =    0;
uint32_t readings             =    0;
int16_t  zero             = 11;
uint8_t  skip             = 0;
uint8_t  power            = 1;
uint8_t  range            = 3;
uint8_t  minRange         = 0;
bool serialEnable = false;
uint16_t ranges[4]={0,50,500,5000};

uint64_t lsumBusMillVolts;
int64_t  lsumBusMicroAmps;
int32_t  lmaxBusMicroAmps;
int32_t  lminBusMicroAmps;
int64_t  lsumBusMicroAmpsOrig;
int64_t  ltotalBusMicroAmps;
uint32_t lreadings=1;
int32_t  lnow;

#include "display_thread.h"
osThreadDef (updateScreen, updateScreenX, osPriorityIdle, 1, 200); 

char	bufUsb[2][1000];
uint16_t usbLen=0;
uint8_t usbPage=0;

#include <math.h>
#include "eeprom.h"

uint16_t VirtAddVarTab[NumbOfVar] = {0x1700, 0x1701, 0x1702};

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
  HAL_TIM_Base_Start_IT(&htim3);

  HAL_FLASH_Unlock();

  EE_Init();

  EE_ReadVariable(0x1700,(uint16_t)(&zero));
  EE_ReadVariable(0x1701,&ranges[2]);
  EE_ReadVariable(0x1702,&ranges[3]);  

  /* Infinite loop */
  char*  pageBuf=bufUsb[usbPage];

  HAL_GPIO_WritePin(RANGE1_GPIO_Port, RANGE1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RANGE2_GPIO_Port, RANGE2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RANGE3_GPIO_Port, RANGE3_Pin, GPIO_PIN_RESET);

  // send INA226 reset
  buf[0]=0; buf[1]=0x80; buf[2]=0x00;
  HAL_I2C_Master_Transmit(&hi2c1,0x80,buf,3,100);
  osDelay(1);

  // configure ina
  HAL_I2C_Master_Transmit(&hi2c1,0x80,buf,1,100);  
  HAL_I2C_Master_Receive(&hi2c1,0x80,buf+1,2,100);
  // mode: 0 - off, 1 - shunt, 2 - bus, 3 - both, 4 - shutdown, 5 - cont.shunt, 6 - cont.bus, 7 - cont.both
  buf[2] = (buf[2]&0xf8)|0x5; 
  // shunt conversion time:  0 - 140, 1 - 204, 2 - 322, 3 - 588, 4 - 1100, 5 - 2116, 6 - 4156, 7 - 8244  
  buf[2] = (buf[2]&0xc7)|(0x0<<3);   
  // bus conversion time:  0 - 140, 1 - 204, 2 - 332, 3 - 588, 4 - 1100, 5 - 2116, 6 - 4156, 7 - 8244  
  buf[2] = (buf[2]&0x3f)|(0x0<<6);   
  buf[1] = (buf[1]&0xFE)|(0x0>>2);   
  // averaging: 0 - 1, 1 - 4, 2 - 16, 3 - 64, 4 - 128, 5 -256, 6 - 512, 7 - 1024
  buf[1] = (buf[1]&0xF1)|0x0<<1;
  HAL_I2C_Master_Transmit(&hi2c1,0x80,buf,3,100);
  osDelay(1);

  // select shunt raw register
  buf[0]=1;
  HAL_I2C_Master_Transmit(&hi2c1,0x80,buf,1,100);

  HAL_I2C_RegisterCallback(&hi2c1,HAL_I2C_MASTER_RX_COMPLETE_CB_ID,&USR_I2C_MasterRxCpltCallback);
  HAL_TIM_RegisterCallback(&htim3,HAL_TIM_PERIOD_ELAPSED_CB_ID,&USR_TIM_PeriodElapsedCallback);

  int cicleCounter =0;

  for(;;)
  {
    osSignalWait(0x1,10);
//	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_ADC_Start(&hadc1);
	  HAL_I2C_Master_Receive_IT(&hi2c1,0x80,buf,2);
    osSignalWait(0x2,10);
    if (skip==0) {
      // we might need to skeep measurement which happened when we switch ranges
      microAmps = inaRes;
      microAmps *= ranges[range];
      microAmps = round((float)microAmps/100);
      } 
      else 
          skip--;

      // adjust range
      if (minRange<2 && microAmps < 11000) {
          if (range!=1) {
            HAL_GPIO_WritePin(RANGE1_GPIO_Port, RANGE1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(RANGE2_GPIO_Port, RANGE2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(RANGE3_GPIO_Port, RANGE3_Pin, GPIO_PIN_RESET);
            range = 1;
            skip=1;
          }   
      } else if (minRange<3 && microAmps < 110000) {
          if (range!=2) {
            HAL_GPIO_WritePin(RANGE2_GPIO_Port, RANGE2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(RANGE1_GPIO_Port, RANGE1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(RANGE3_GPIO_Port, RANGE3_Pin, GPIO_PIN_RESET);
            range = 2;
            skip=1;
          }        
      } else if (range!=3) {
        HAL_GPIO_WritePin(RANGE2_GPIO_Port, RANGE3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(RANGE2_GPIO_Port, RANGE2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(RANGE1_GPIO_Port, RANGE1_Pin, GPIO_PIN_RESET);
        range = 3;  
        skip=1;      
      }

      sumBusMicroAmpsOrig += microAmps; // microAmps;
      microAmps-=zero;
      // update accumulators
      if (readings==0) {
        minBusMicroAmps = maxBusMicroAmps = microAmps;
      } else {
        if (microAmps<minBusMicroAmps) 
          minBusMicroAmps = microAmps;
        if (microAmps>maxBusMicroAmps)
          maxBusMicroAmps = microAmps;
      }
      sumBusMicroAmps += microAmps;     // Add current value to sum
      totalBusMicroAmps += microAmps * 140 / 1000;
      readings++;    
      miliVolts = HAL_ADC_GetValue(&hadc1)*14970/4096;
      sumBusMillVolts += miliVolts; // 14970 = 6 * 24950    

    if (true || serialEnable) {
      itoa(microAmps,pageBuf,10);
      uint8_t len=strlen(pageBuf);
      pageBuf += len;
//        *(scbuf++)=',';
//        slen+=len+1;
//        itoa(miliVolts,scbuf,10);
//        len=strlen(scbuf);
//        scbuf += len;
      *(pageBuf++)='\r';
      *(pageBuf++)='\n';    
      usbLen+=len+2;
      if (usbLen>900) {
        CDC_Transmit_FS((uint8_t *)bufUsb[usbPage],usbLen);
        usbPage = (usbPage+1)%2; 
        usbLen = 0;
        pageBuf = bufUsb[usbPage];
      }
    }

//	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

    if (cicleCounter++>=714) {
      // make data snapshot
      lsumBusMillVolts=sumBusMillVolts;
      lsumBusMicroAmps=sumBusMicroAmps;
      lminBusMicroAmps=minBusMicroAmps;
      lmaxBusMicroAmps=maxBusMicroAmps;
      lsumBusMicroAmpsOrig=sumBusMicroAmpsOrig;
      ltotalBusMicroAmps=totalBusMicroAmps;
      lreadings=readings;
      lnow+=100;

      // reset readings
      readings = 0;
      sumBusMillVolts = 0;
      sumBusMicroAmps = 0;
      sumBusMicroAmpsOrig = 0;      

      // signaling udpate screen thread
      osSignalSet (osUpdateScreenThreadId, 0x1);
      cicleCounter=0;
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
