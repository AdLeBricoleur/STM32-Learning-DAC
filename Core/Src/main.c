/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac_ch1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
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
	// My sinus lookup table
	/*uint32_t sinus_lut[]={
		2047,2147,2248,2347,2446,2545,2641,2737,2831,2922,3012,3100,3185,3267,3346,3422,3495,3564,3630,3692,3749,3803,3853,3898,3939,3975,4006,4033,4055,4072,4085,4092,4095,4092,4085,4072,4055,4033,4006,3975,3939,3898,3853,3803,3749,3692,3630,3564,3495,3422,3346,3267,3185,3100,3012,2922,2831,2737,2641,2545,2446,2347,2248,2147,2047,1947,1846,1747,1648,1549,1453,1357,1263,1172,1082,994,909,827,748,672,599,530,464,402,345,291,241,196,155,119,88,61,39,22,9,2,0,2,9,22,39,61,88,119,155,196,241,291,345,402,464,530,599,672,748,827,909,994,1082,1172,1263,1357,1453,1549,1648,1747,1846,1947,2047
	};*/
	uint32_t sinus_lut[]={
		2047,2057,2067,2077,2087,2097,2107,2117,2127,2137,2148,2158,2168,2178,2188,2198,2208,2218,2228,2238,2248,2258,2268,2278,2288,2298,2308,2318,2328,2338,2348,2358,2368,2377,2387,2397,2407,2417,2427,2437,2447,2457,2466,2476,2486,2496,2506,2516,2525,2535,2545,2555,2564,2574,2584,2594,2603,2613,2623,2632,2642,2651,2661,2671,2680,2690,2699,2709,2718,2728,2737,2747,2756,2766,2775,2784,2794,2803,2813,2822,2831,2840,2850,2859,2868,2877,2887,2896,2905,2914,2923,2932,2941,2950,2959,2968,2977,2986,2995,3004,3013,3022,3031,3039,3048,3057,3066,3074,3083,3092,3100,3109,3118,3126,3135,3143,3152,3160,3169,3177,3185,3194,3202,3210,3219,3227,3235,3243,3251,3259,3268,3276,3284,3292,3300,3308,3315,3323,3331,3339,3347,3355,3362,3370,3378,3385,3393,3400,3408,3415,3423,3430,3438,3445,3452,3460,3467,3474,3481,3489,3496,3503,3510,3517,3524,3531,3538,3545,3551,3558,3565,3572,3578,3585,3592,3598,3605,3611,3618,3624,3631,3637,3643,3650,3656,3662,3668,3674,3680,3686,3692,3698,3704,3710,3716,3722,3728,3733,3739,3745,3750,3756,3761,3767,3772,3778,3783,3788,3794,3799,3804,3809,3814,3819,3824,3829,3834,3839,3844,3849,3854,3858,3863,3868,3872,3877,3881,3886,3890,3894,3899,3903,3907,3911,3916,3920,3924,3928,3932,3936,3939,3943,3947,3951,3954,3958,3962,3965,3969,3972,3975,3979,3982,3985,3989,3992,3995,3998,4001,4004,4007,4010,4013,4015,4018,4021,4024,4026,4029,4031,4034,4036,4038,4041,4043,4045,4047,4050,4052,4054,4056,4058,4059,4061,4063,4065,4066,4068,4070,4071,4073,4074,4076,4077,4078,4079,4081,4082,4083,4084,4085,4086,4087,4088,4088,4089,4090,4090,4091,4092,4092,4093,4093,4093,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4094,4093,4093,4092,4092,4091,4091,4090,4089,4089,4088,4087,4086,4085,4084,4083,4082,4081,4080,4079,4077,4076,4075,4073,4072,4070,4069,4067,4066,4064,4062,4060,4058,4057,4055,4053,4051,4048,4046,4044,4042,4040,4037,4035,4032,4030,4027,4025,4022,4020,4017,4014,4011,4008,4005,4003,4000,3996,3993,3990,3987,3984,3980,3977,3974,3970,3967,3963,3960,3956,3953,3949,3945,3941,3937,3934,3930,3926,3922,3918,3913,3909,3905,3901,3897,3892,3888,3883,3879,3874,3870,3865,3861,3856,3851,3846,3842,3837,3832,3827,3822,3817,3812,3807,3801,3796,3791,3786,3780,3775,3770,3764,3759,3753,3748,3742,3736,3731,3725,3719,3713,3707,3701,3695,3689,3683,3677,3671,3665,3659,3653,3646,3640,3634,3627,3621,3615,3608,3602,3595,3588,3582,3575,3568,3562,3555,3548,3541,3534,3527,3520,3513,3506,3499,3492,3485,3478,3471,3463,3456,3449,3441,3434,3427,3419,3412,3404,3397,3389,3381,3374,3366,3358,3351,3343,3335,3327,3319,3311,3304,3296,3288,3280,3272,3263,3255,3247,3239,3231,3223,3214,3206,3198,3189,3181,3173,3164,3156,3147,3139,3130,3122,3113,3105,3096,3087,3079,3070,3061,3053,3044,3035,3026,3017,3008,3000,2991,2982,2973,2964,2955,2946,2937,2928,2919,2909,2900,2891,2882,2873,2864,2854,2845,2836,2826,2817,2808,2798,2789,2780,2770,2761,2751,2742,2733,2723,2714,2704,2695,2685,2675,2666,2656,2647,2637,2627,2618,2608,2598,2589,2579,2569,2560,2550,2540,2530,2520,2511,2501,2491,2481,2471,2462,2452,2442,2432,2422,2412,2402,2392,2382,2373,2363,2353,2343,2333,2323,2313,2303,2293,2283,2273,2263,2253,2243,2233,2223,2213,2203,2193,2183,2173,2163,2153,2143,2132,2122,2112,2102,2092,2082,2072,2062,2052,2042,2032,2022,2012,2002,1992,1982,1972,1962,1951,1941,1931,1921,1911,1901,1891,1881,1871,1861,1851,1841,1831,1821,1811,1801,1791,1781,1771,1761,1751,1741,1731,1721,1712,1702,1692,1682,1672,1662,1652,1642,1632,1623,1613,1603,1593,1583,1574,1564,1554,1544,1534,1525,1515,1505,1496,1486,1476,1467,1457,1447,1438,1428,1419,1409,1399,1390,1380,1371,1361,1352,1343,1333,1324,1314,1305,1296,1286,1277,1268,1258,1249,1240,1230,1221,1212,1203,1194,1185,1175,1166,1157,1148,1139,1130,1121,1112,1103,1094,1086,1077,1068,1059,1050,1041,1033,1024,1015,1007,998,989,981,972,964,955,947,938,930,921,913,905,896,888,880,871,863,855,847,839,831,822,814,806,798,790,783,775,767,759,751,743,736,728,720,713,705,697,690,682,675,667,660,653,645,638,631,623,616,609,602,595,588,581,574,567,560,553,546,539,532,526,519,512,506,499,492,486,479,473,467,460,454,448,441,435,429,423,417,411,405,399,393,387,381,375,369,363,358,352,346,341,335,330,324,319,314,308,303,298,293,287,282,277,272,267,262,257,252,248,243,238,233,229,224,220,215,211,206,202,197,193,189,185,181,176,172,168,164,160,157,153,149,145,141,138,134,131,127,124,120,117,114,110,107,104,101,98,94,91,89,86,83,80,77,74,72,69,67,64,62,59,57,54,52,50,48,46,43,41,39,37,36,34,32,30,28,27,25,24,22,21,19,18,17,15,14,13,12,11,10,9,8,7,6,5,5,4,3,3,2,2,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,2,2,3,4,4,5,6,6,7,8,9,10,11,12,13,15,16,17,18,20,21,23,24,26,28,29,31,33,35,36,38,40,42,44,47,49,51,53,56,58,60,63,65,68,70,73,76,79,81,84,87,90,93,96,99,102,105,109,112,115,119,122,125,129,132,136,140,143,147,151,155,158,162,166,170,174,178,183,187,191,195,200,204,208,213,217,222,226,231,236,240,245,250,255,260,265,270,275,280,285,290,295,300,306,311,316,322,327,333,338,344,349,355,361,366,372,378,384,390,396,402,408,414,420,426,432,438,444,451,457,463,470,476,483,489,496,502,509,516,522,529,536,543,549,556,563,570,577,584,591,598,605,613,620,627,634,642,649,656,664,671,679,686,694,701,709,716,724,732,739,747,755,763,771,779,786,794,802,810,818,826,835,843,851,859,867,875,884,892,900,909,917,925,934,942,951,959,968,976,985,994,1002,1011,1020,1028,1037,1046,1055,1063,1072,1081,1090,1099,1108,1117,1126,1135,1144,1153,1162,1171,1180,1189,1198,1207,1217,1226,1235,1244,1254,1263,1272,1281,1291,1300,1310,1319,1328,1338,1347,1357,1366,1376,1385,1395,1404,1414,1423,1433,1443,1452,1462,1471,1481,1491,1500,1510,1520,1530,1539,1549,1559,1569,1578,1588,1598,1608,1618,1628,1637,1647,1657,1667,1677,1687,1697,1707,1717,1726,1736,1746,1756,1766,1776,1786,1796,1806,1816,1826,1836,1846,1856,1866,1876,1886,1896,1906,1916,1926,1936,1946,1957,1967,1977,1987,1997,2007,2017,2027,2037,2047
	};
	// Calculate the number of elements in my sinus lookup table
	int n = 0;
	n = sizeof(sinus_lut)/sizeof(uint32_t);
	// I don't take the last element because it is equal to the first
	n -= 1;
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
  MX_USART2_UART_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  // Start DAC with DMA
  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)sinus_lut, (uint32_t)n, DAC_ALIGN_12B_R);
  HAL_TIM_Base_Start(&htim2);
  /* USER CODE END 2 */

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 624;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
