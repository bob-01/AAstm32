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
*/

#include "main.h"
#include <math.h>
#include "s6d1121.h"
#include "si5351.h"
#include "ad8302.h"

#define ADC2VOLT (3.3/4096.0)
#define offsetdB (30.0)
#define offsetDegrees (90.0)

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
I2C_HandleTypeDef hi2c1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);

extern unsigned char SmallFont[];
uint8_t position_x = 0;

float RL;
float Phi;
float Pho;

  int main(void) {
    // Инициализация переферии
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    while(HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK);  
    while(HAL_ADCEx_Calibration_Start(&hadc2) != HAL_OK);  
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

        AD8302_RX t = AD8302_MeasureRX();

        print("R", 1, position_x, 0);      
        printNumI(t.R, cfont.x_size*2, position_x, 1,' '); // FIXME:
        print("X", cfont.x_size*6, position_x, 0);      
        printNumI(t.X , cfont.x_size*8, position_x, 1,' ');
        

/*
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

        Phi = Phi/255*ADC2VOLT;
        if (Phi > 1.80) Phi = 1.80;
        RL = RL/255*ADC2VOLT;
        if (RL > 1.80) RL = 1.80;

        RL = RL/0.03 - offsetdB;
        Phi = Phi/0.01 - offsetDegrees;

        if (RL > 30.0) RL = 30.0;
        if (RL < -30.0) RL = -30.0;
        if (Phi > 180.0) Phi = 180.0;
        if (Phi < 0.0) Phi = 0.0;

        print("RL", 1, position_x, 0);      
        printNumI(RL, cfont.x_size*3, position_x, 1,' '); // FIXME:
        print("Phi", cfont.x_size*6, position_x, 0);      
        printNumI(Phi , cfont.x_size*10, position_x, 1,' ');

        float Rho = powf(10.0, RL * 0.05);
        print("Pho", cfont.x_size*13, position_x, 0);      
        printNumI(Pho , cfont.x_size*17, position_x, 1,' ');


        Phi = 3.1416 - Phi * 3.1416;
        if (Phi > 3.1416 / 2) Phi = 3.1416 / 2;

        float R = (cosf(Phi) * Rtotal * Rho) - (Rmeas + RmeasAdd);
        if(R < 0.0) R = 0.0;

        print("R", cfont.x_size*21, position_x, 0);      
        printNumI(R, cfont.x_size*23, position_x, 1,' ');

/*


/*        
        float re = (float)(Rho*cosf(Phi));
        float im = (float)(Rho*sinf(Phi));
        float denominator = ((1.0-re)*(1.0-re)+(im*im));
        Rs = fabs((1.0-(re*re)-(im*im))/denominator) * Z0;
        Xs = fabs(2.0*im)/denominator * Z0;
        Z = sqrtf(Rs*Rs+Xs*Xs);
        Swr = fabs((1.0+Rho)/(1.0-Rho));
*/
/*
        Rho = sqrtf(powf(Rho,2)+1-2*Rho*cosf(Phi));
        Swr = fabs((1.0+Rho)/(1.0-Rho));

        print("T", cfont.x_size*24, position_x, 0);
        printNumI(Swr , cfont.x_size*26, position_x, 1,' ');

        if ((position_x + cfont.y_size) > disp_x_size) {
          position_x = 0;
          clrScr();
        }
*/
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
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
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
  HAL_GPIO_WritePin(GPIOB, LCD_CS_Pin_Pin|LCD_RST_Pin_Pin|LCD_RS_Pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_WR_Pin_GPIO_Port, LCD_WR_Pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : AD8302_a_Pin AD8302_a_b_Pin */
  GPIO_InitStruct.Pin = AD8302_a_Pin|AD8302_a_b_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_CS_Pin_Pin LCD_RST_Pin_Pin LCD_RS_Pin_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin_Pin|LCD_RST_Pin_Pin|LCD_RS_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : enc_A_Pin enc_B_Pin enc_Button_Pin */
  GPIO_InitStruct.Pin = enc_A_Pin|enc_B_Pin|enc_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_WR_Pin_Pin */
  GPIO_InitStruct.Pin = LCD_WR_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_WR_Pin_GPIO_Port, &GPIO_InitStruct);

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
