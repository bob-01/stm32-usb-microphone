/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define FRAME_SIZE 160 //*0.125ìñ = 20ìñ (ñýìïëèðîâàíèå 8êÃö)
#define ENCODED_FRAME_SIZE 20 //óæèìàåò â 8 ðàç
#define MAX_REC_FRAMES 90 //ìàêñèìàëüíîå ÷èñëî çàïèñûâàåìûõ ôðåéìîâ, Âðåìÿ = MAX_REC_FRAMES*0,02ñåê

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

__IO uint16_t IN_Buffer[2][FRAME_SIZE];
__IO uint8_t Start_Encoding;

uint8_t Index_Encoding = 0;

uint32_t count = 0;

uint16_t sine_table[36] = {
      512, 601, 687, 768, 841, 904, 955, 993, 1016, 1024,
      1016, 993, 956, 905, 842, 769, 688, 602, 513, 424,
      338, 257, 184, 120, 69, 31, 8, 0, 8, 30,
      68, 119, 182, 255, 335, 422,
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

void init_PWM();

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  if(HAL_TIM_Base_Start_IT(&htim3) != HAL_OK) Error_Handler();
  if(HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&IN_Buffer[0],FRAME_SIZE*2) != HAL_OK) Error_Handler();

  init_PWM();
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    //GPIOB->CRH &=  ~ ( 0b1111 << 16 ); // сбрасываем биты 16-19
    //GPIOB->CRH |= 0b1000 << 16;  // устанавливаем биты

    if(Start_Encoding > 0)
    { 
      Index_Encoding = Start_Encoding - 1;
      for (uint8_t i = 0; i < FRAME_SIZE; i++) {         
          if(IN_Buffer[Index_Encoding][i] > 2000) {
              if(GPIOC->IDR & GPIO_IDR_IDR13) GPIOC->ODR &= ~GPIO_ODR_ODR13;
              else GPIOC->ODR |= GPIO_ODR_ODR13;
          }

    	  //IN_Buffer[Index_Encoding][i]^=0x8000;
      }

      //IN_Buffer[Index_Encoding][i]^=0x8000

      /* Flush all the bits in the struct so we can encode a new frame */
      //speex_bits_reset(&bits);
      /* Encode the frame */
      //speex_encode_int(enc_state, (spx_int16_t*)IN_Buffer[Index_Encoding], &bits);
      /* Copy the bits to an array of char that can be decoded */
      //speex_bits_write(&bits, (char *)Rec_Data_ptr, ENCODED_FRAME_SIZE);
        
      //Rec_Data_ptr += ENCODED_FRAME_SIZE;
      //Encoded_Frames += 1;
      
      Start_Encoding = 0;	
    }

    if(count > 5 * 1e5) {
      if(GPIOC->IDR & GPIO_IDR_IDR13) GPIOC->ODR &= ~GPIO_ODR_ODR13;
      else GPIOC->ODR |= GPIO_ODR_ODR13;
      count = 0;
    }
    
    count++;

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 8999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Led_Pin */
  GPIO_InitStruct.Pin = Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Led_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void init_PWM()
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

  GPIOB->CRL &= ~GPIO_CRL_CNF6;
  GPIOB->CRL |= GPIO_CRL_CNF6_1 | GPIO_CRL_MODE6_0;      // PB6: AF PP

  /*---- Настройка генератора Ш�?М (TIM4)---------------*/
  /* несущая частота Ш�?М = SystemCoreClock / 1024 */
  TIM4->PSC = 0;
  TIM4->ARR = 1024 - 1;                         // 10 бит
  TIM4->DIER |= TIM_DIER_UDE;   
  TIM4->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 |   // PWM mode 2
                TIM_CCMR1_OC1PE;                  // Preload CCR Enable
  TIM4->CCER = TIM_CCER_CC1E;                     // Out Enable
  TIM4->CCR1 = 512;                              // Центральное значение (1/2)

  TIM4->DCR = 0x0D | 0;               //доступ к регистру CCR1 для изменения Ш�?М
  /*----------------------------------------------------*/

  /*---- Настройка генерации синусоиды (TIM4, DMA)------*/
  /* например, синусоида 1 кГц, размер табл.синусов 36 знач.*/

  RCC->AHBENR |= RCC_AHBENR_DMA1EN;
  /* DMA для автоматического изменения скважности */

  DMA1_Channel7->CCR |= 
                  DMA_CCR_DIR  |               // Mem->Periph
                  DMA_CCR_MINC |               // Mem Incr
                  DMA_CCR_CIRC |               // Circular mode
                  DMA_CCR_PSIZE_0 |            // 16 bit
                  DMA_CCR_MSIZE_0;             // 16 bit

  DMA1_Channel7->CNDTR = 36;                     // размер таблицы синуса
  DMA1_Channel7->CMAR = (uint32_t)sine_table;         // адрес таблицы
  DMA1_Channel7->CPAR = (uint32_t)&TIM4->DMAR;        // арес периф.регистра
  //DMA1_Channel7->CCR |= DMA_CCR_EN;               // канал DMA включен

  /*----------------------------------------------------*/

  /*---- Запуск ----------------------------------------*/
  TIM4->CR1 |= TIM_CR1_CEN;
  /*----------------------------------------------------*/

}

/* USER CODE END 4 */

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
