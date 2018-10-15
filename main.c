/*
  http://qrz.by/forum/viewtopic.php?t=2133
  EU3EU :
  Ну а с AD8302 - просто оцифровываем с помощью АЦП Uref, Uab и Upha и используем прямо в "условных единицах" АЦП, не переводя в Вольты.

  mid = Uref/2 соответствует Uab 0dB (и фазе 90градусов).
  Находим "цену" 1 дБ в у.е.: onedb = mid/30 (AD8302 выдаёт 0 при -30dB, и Uref при +30dB)
  UabdB = (Uab - mid) / onedb <-- ещё нужно добавить минус, если A и B поменять местами в схеме.
  отсюда отношение (Ua/Ub) = 10^(UabdB/20)

  С фазой проще:
  phaseRadians = PI - PI * (Upha/ Uref)
 ----------------------------------------

  RL=-20log(Rho)
  Rho=10^(Rl/-20)
  Z=(ZL-ZO)/(ZL+Z0) avec Z0=50ohms
  Z=a+jb avec 
  a=Rho*cos(phi)
  b=Rho*sin(phi)
  ZL=(1+Z)/(1-Z)*Z0
  ZL=RS+jXS avec
  RS=abs(1-a²-b²)/((1-a)²+b²)
  XS=abs(2b/((1-a)²-b²))
  |Z|=sqrt(RS²+XS²)
  SWR=(1+Rho)/(1-Rho)

  3.3v - 4096

  1.8v = 0 degrees
  0v   = 180 degrees

  0v = -30dB
  1.8v= 30dB

  VMAG[V ] = 0.03[V /dB] × Gain[dB] + 0.9[V ]
  vmag = α1 · log(|VA|/|VB|) + 900 mV
  α1 = 600 mV/decade (or 30 mV dB−1)
  
  VP HS[V ] = 0.01[V /°] × (±Phase[°] + 180°)
  vp = 0.01 (phase + 180) 
  vp/0.01 = phase + 180

  vphs = α2 · (|θA − θB| − 90◦) + 900 mV
  α2 = −10 mV/degree

*/

#include "main.h"
#include <math.h>
#include "s6d1121.h"
#include "si5351.h"

#define ADC18 ((4096/3.3)*1.8)  // count for 1.8v 2234,1818181818181818181818181818
#define offsetDb (-30.0)
#define offsetPhi (180.0)
#define ADC2dB (60.0/ADC18)       // 0,02685546875
#define ADC2Angle (180.0/ADC18)   // 0,08056640625

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);

extern unsigned char SmallFont[];
uint8_t position_x = 0;

float RL;
float Phi;
float Rho;
float Rs;
float Xs;
float Swr;
float Z;

float Z0 = 50.0;
float calPhs = 0.0;           // Calibration Phase
float calMag = 0.0;

  int main(void)
  {
    // Инициализация переферии
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_TIM2_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    // Конец Инициализации

    disp_x_size = 239;
    disp_y_size = 319;

	  InitLCD(1);
	  setFont(SmallFont);

    clrScr();

    while (1) {

      uint8_t  error = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(96) , 10, 100);

      if(error == HAL_OK) {
        setColor(255, 0, 0);
        print("SI5351 is not ready!!!", 1, position_x, 0);
      }

      setColor(0, 255, 0);

      si5351_init(SI5351_CRYSTAL_LOAD_0PF, 0, 0);
      si5351_drive_strength(SI5351_CLK0, SI5351_DRIVE_2MA);

      for (uint8_t i = 20; i < 40; i++) {

        si5351_set_freq(i*10000000UL, SI5351_CLK0);
        HAL_Delay(100);  

        Phi = 0;
        RL = 0;

          for (uint8_t n = 0; n < 255; n++){
            HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1,100);
            Phi += (uint32_t) HAL_ADC_GetValue(&hadc1);        
            HAL_ADC_Stop(&hadc1);

            HAL_ADC_Start(&hadc2);
            HAL_ADC_PollForConversion(&hadc2,100);
            RL += (uint32_t) HAL_ADC_GetValue(&hadc2);
            HAL_ADC_Stop(&hadc2);
          }

        Phi = Phi/255;
        RL = RL/255;

        print("RL", 1, position_x +=cfont.y_size, 0);      
        printNumI(RL, cfont.x_size*3, position_x, 1,' ');
        print("Phi", cfont.x_size*8, position_x, 0);      
        printNumI(Phi, cfont.x_size*12, position_x, 1,' ');

        RL = ADC2dB*RL + offsetDb + calMag;
        
        Phi = ADC2Angle*Phi;
        if (Phi > 180) Phi = 180;

        Phi = offsetPhi - Phi + calPhs;

        print("db", cfont.x_size*17, position_x, 0);      
        printNumI(RL, cfont.x_size*20, position_x, 1,' ');

        Rho = powf(10.0, RL/-20.0);

/*        
        float re = (float)(Rho*cosf(Phi));
        float im = (float)(Rho*sinf(Phi));
        float denominator = ((1.0-re)*(1.0-re)+(im*im));
        Rs = fabs((1.0-(re*re)-(im*im))/denominator) * Z0;
        Xs = fabs(2.0*im)/denominator * Z0;
        Z = sqrtf(Rs*Rs+Xs*Xs);
        Swr = fabs((1.0+Rho)/(1.0-Rho));
*/
        Rho = sqrtf(powf(Rho,2)+1-2*Rho*cosf(Phi));
        Swr = fabs((1.0+Rho)/(1.0-Rho));

        print("T", cfont.x_size*24, position_x, 0);
        printNumI(Swr , cfont.x_size*26, position_x, 1,' ');

        if ((position_x + cfont.y_size) > disp_x_size) {
          position_x = 0;
          clrScr();
        }

      }
/*
      Ko = sqrt( (pow(RL, 2) + 4 - (4*RL*cos(Phi))) );
      B = asin( ((RL * (sin(Phi)))/Ko) );
      R = ((2*cos(B) - Ko)/Ko)*75;
      X = ((2*sin(B))/Ko)*75;
      Z = sqrt((pow(R,2) + pow(X,2)));

      setColor(255, 10, 10);

      print("R:", 1, position_x +=cfont.y_size, 0);      
      printNumI(R , cfont.x_size*3, position_x, 1,' ');
      print("X:", cfont.x_size*12, position_x, 0);      
      printNumI(X , cfont.x_size*15, position_x, 1,' ');
      print("Z:", cfont.x_size*28, position_x, 0);      
      printNumI(Z , cfont.x_size*31, position_x, 1,' ');
*/
      HAL_Delay(1000);

      position_x = 0;
      clrScr();

    }

    return 0;

  }

void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* ADC2 init function */
static void MX_ADC2_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{
  __HAL_RCC_I2C1_CLK_ENABLE();
  HAL_Delay(100);
  __HAL_RCC_I2C1_FORCE_RESET();
  HAL_Delay(100);
  __HAL_RCC_I2C1_RELEASE_RESET();
  HAL_Delay(100);

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 35999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 800;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_CS_Pin|LCD_RST_Pin|LCD_RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_WR_GPIO_Port, LCD_WR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : AD8302_A_Blue_Pin */
  GPIO_InitStruct.Pin = AD8302_A_Blue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(AD8302_A_Blue_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_CS_Pin LCD_RST_Pin LCD_RS_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin|LCD_RST_Pin|LCD_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ENC_2_Pin ENC_1_Pin ENC_BUTTON_Pin */
  GPIO_InitStruct.Pin = ENC_2_Pin|ENC_1_Pin|ENC_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_WR_Pin */
  GPIO_InitStruct.Pin = LCD_WR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LCD_WR_GPIO_Port, &GPIO_InitStruct);

}

void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
