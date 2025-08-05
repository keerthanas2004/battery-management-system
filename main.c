#include "main.h"
#include <stdio.h>
#include "lcd16x2.h"
#include <math.h>

ADC_HandleTypeDef hadc1;

#define SERIES_RESISTOR 9400.0   // 10k pull-up resistor
#define NOMINAL_RESISTANCE 10000.0 // Thermistor resistance at 25°C
#define NOMINAL_TEMPERATURE 25.0  // In °C
#define B_COEFFICIENT 3380.0      // Thermistor Beta value
#define ADC_MAX 4095.0
#define VCC 3.3f                  // Supply voltage for voltage divider


char buffer[16];

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);

void DWT_Delay_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void DWT_Delay_us(volatile uint32_t microseconds)
{
    uint32_t clk_cycle_start = DWT->CYCCNT;
    microseconds *= (HAL_RCC_GetHCLKFreq() / 1000000);
    while ((DWT->CYCCNT - clk_cycle_start) < microseconds);
}

float read_voltage(uint16_t adcVal) {
	float vout = ((float)adcVal * 3.3f) / 4095.0f;
	float vin = vout * (5.0f);  // Adjust based on resistor divider ratio

	return vin;
}

float read_thermistor_temp(uint16_t adcVal) {
    float voltage = ((float)adcVal / 4095.0) * 3.3f;
    float resistance = SERIES_RESISTOR * (5.0f / voltage - 1.0);

    float steinhart = resistance / NOMINAL_RESISTANCE;
    steinhart = log(steinhart);
    steinhart /= B_COEFFICIENT;
    steinhart += 1.0 / (NOMINAL_TEMPERATURE + 273.15);
    steinhart = 1.0 / steinhart;
    steinhart -= 273.15;

    return steinhart;
}


int main(void)
{
    HAL_Init();
    SystemClock_Config();
    DWT_Delay_Init();
    MX_GPIO_Init();
    MX_ADC1_Init();
    LCD_Init();
    LCD_Clear();
    LCD_SetCursor(0, 0);
    LCD_SendString("Initializing...");
    HAL_Delay(1000);

    ADC_ChannelConfTypeDef sConfig = {0};
    GPIO_PinState prev1 = GPIO_PIN_RESET;
    GPIO_PinState prev2 = GPIO_PIN_RESET;
    float d_init = 200.0f;
    while (1)
    {
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
    	GPIO_PinState state1 = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1);
    	GPIO_PinState state2 = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2);

    	if(state1 != prev1){
    		HAL_Delay(50);
    		state1 = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1);
    		prev1 = state1;
    	}
    	if(state2 != prev2){
    	    		HAL_Delay(50);
    	    		state2 = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2);
    	    		prev2 = state2;
    	}

    	while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_SET && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_SET){
    		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
    	}

        if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_SET){
        	LCD_Clear();
        	LCD_SetCursor(0, 0);
        	LCD_SendString("Discharging...");
        	HAL_Delay(1000);
        }
        if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_SET){
			LCD_Clear();
			LCD_SetCursor(0, 0);
			LCD_SendString("Charging...");
			HAL_Delay(1000);
		}

        /** --------- Temperature Sensor (ADC1_IN2) --------- */
        sConfig.Channel = ADC_CHANNEL_2;
        sConfig.Rank = ADC_REGULAR_RANK_1;
        sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
        HAL_ADC_ConfigChannel(&hadc1, &sConfig);

        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        uint16_t adc_temp = HAL_ADC_GetValue(&hadc1);
        float temperature = read_thermistor_temp(adc_temp);

        snprintf(buffer, sizeof(buffer), "Temp: %.2f C", temperature);
        LCD_SetCursor(0, 0);
        LCD_SendString(buffer);
    	HAL_Delay(500);
        LCD_Clear();
        if(temperature > 32){
			 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
		}
		else{
			 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
		}

        sConfig.Channel = ADC_CHANNEL_1;
        HAL_ADC_ConfigChannel(&hadc1, &sConfig);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		uint32_t adc_val_voltage = HAL_ADC_GetValue(&hadc1);
		float vin = read_voltage(adc_val_voltage);
		float soc = (vin / 7.4f) * 100.0f;

		snprintf(buffer, sizeof(buffer), "SOC: %.2f%%", soc);
		LCD_SetCursor(0, 0);
		LCD_SendString(buffer);

		/** --------- Current Sensor (ADC1_IN12) --------- */
		sConfig.Channel = ADC_CHANNEL_12;
		HAL_ADC_ConfigChannel(&hadc1, &sConfig);
		uint32_t sum = 0;
		int samples = 200;
		for(int i = 0; i < samples; i++) {
		        HAL_ADC_Start(&hadc1);
		        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		        sum += HAL_ADC_GetValue(&hadc1);
		    }
		sum = sum/samples;
		float voltage_c = ((float)sum * 3.3f) / 4095.0f;
		float current = (voltage_c - 2.48f ) / 0.083f + 1.3f; // 0.083mV/A sensitivity

		snprintf(buffer, sizeof(buffer), "Current: %.2fA",current);
		LCD_SetCursor(1, 0);
		LCD_SendString(buffer);

		HAL_Delay(1000);
		LCD_Clear();
        while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_SET && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_RESET){

        	/** --------- Voltage Sensor (ADC1_IN1) --------- */
			sConfig.Channel = ADC_CHANNEL_1;
			HAL_ADC_ConfigChannel(&hadc1, &sConfig);

			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
			uint32_t adc_val_voltage = HAL_ADC_GetValue(&hadc1);
			float vin = read_voltage(adc_val_voltage);
			float soc = (vin / 7.4f) * 100.0f;
			float distance = d_init * (soc/100.0f);

			snprintf(buffer, sizeof(buffer), "SOC: %.2f%%", soc);
			LCD_SetCursor(0, 0);
			LCD_SendString(buffer);

			HAL_Delay(1000);
			if(vin > 2){
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);

			}
			else{
				LCD_Clear();
				LCD_SetCursor(0, 0);
			    LCD_SendString("LOW BATTERY!");
			    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
			}

			HAL_Delay(100);

	/** --------- Current Sensor (ADC1_IN12) --------- */
			sConfig.Channel = ADC_CHANNEL_12;
			HAL_ADC_ConfigChannel(&hadc1, &sConfig);
			uint32_t sum = 0;
			int samples = 200;
			for(int i = 0; i < samples; i++) {
					HAL_ADC_Start(&hadc1);
					HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
					sum += HAL_ADC_GetValue(&hadc1);
				}
			sum = sum/samples;
			float voltage_c = ((float)sum * 3.3f) / 4095.0f;
			float current = (voltage_c - 2.48f ) / 0.083f + 1.3f; // 0.083mV/A sensitivity
			snprintf(buffer, sizeof(buffer), "Distance: %.2f%%", distance);
			LCD_SetCursor(0, 0);
			LCD_SendString(buffer);
			snprintf(buffer, sizeof(buffer), "Current: %.2fA",current);
			LCD_SetCursor(1, 0);
			LCD_SendString(buffer);

			HAL_Delay(500);
			LCD_Clear();

			sConfig.Channel = ADC_CHANNEL_2;
			sConfig.Rank = ADC_REGULAR_RANK_1;
			sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
			HAL_ADC_ConfigChannel(&hadc1, &sConfig);

			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
			uint16_t adc_temp = HAL_ADC_GetValue(&hadc1);

			float temperature = read_thermistor_temp(adc_temp);

			LCD_Clear();
			if(temperature > 32){
				 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
			}
			else{
				 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
			}
        }

        while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_SET){
        	sConfig.Channel = ADC_CHANNEL_1;
			HAL_ADC_ConfigChannel(&hadc1, &sConfig);

			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
			uint32_t adc_val_voltage = HAL_ADC_GetValue(&hadc1);
			float vin = read_voltage(adc_val_voltage);
			float soc = (vin / 7.4f) * 100.0f;

			snprintf(buffer, sizeof(buffer), "SOC: %.2f%%", soc);
			LCD_SetCursor(0, 0);
			LCD_SendString(buffer);

			if(vin <= 7.4){
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);

			}
			else{
				LCD_Clear();
				LCD_SetCursor(0, 0);
				LCD_SendString("FULL BATTERY!");
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
			}

			HAL_Delay(100);

			/** --------- Current Sensor (ADC1_IN12) --------- */
			sConfig.Channel = ADC_CHANNEL_12;
			HAL_ADC_ConfigChannel(&hadc1, &sConfig);
			uint32_t sum = 0;
			int samples = 200;
			for(int i = 0; i < samples; i++) {
					HAL_ADC_Start(&hadc1);
					HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
					sum += HAL_ADC_GetValue(&hadc1);
				}
			sum = sum/samples;
			float voltage_c = ((float)sum * 3.3f) / 4095.0f;
			float current = (voltage_c - 2.48f ) / 0.083f + 1.3f; // 0.083mV/A sensitivity

			snprintf(buffer, sizeof(buffer), "Current: %.2fA",current);
			LCD_SetCursor(1, 0);
			LCD_SendString(buffer);
			HAL_Delay(1000);

			sConfig.Channel = ADC_CHANNEL_2;
			sConfig.Rank = ADC_REGULAR_RANK_1;
			sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
			HAL_ADC_ConfigChannel(&hadc1, &sConfig);

			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
			uint16_t adc_temp = HAL_ADC_GetValue(&hadc1);

			float temperature = read_thermistor_temp(adc_temp);

			LCD_Clear();
			if(temperature > 32){
				 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
			}
			else{
				 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
			}

        }

    }
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
  RCC_OscInitStruct.HSICalibrationValue = 64;
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
