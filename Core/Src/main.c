/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
//#include "ssd1306_tests.h"
#include "stdio.h"
#include "math.h"
#define PI 3.1415926
#define ADC_MAX_NUM 2*10 // dma num
#define HORIZONTAL_BLOK_NUM 3 // max horizontal bolock num on display
#define VERTICAL_BLOK_NUM 5 // max horizontal bolock num on display
#define MAX_SAMPLE_NUM 256 //max sample num
#define INFO_PIXEL_BOTTOM 12 //12px space used to show information on bottom of the display
#define INFO_PIXEL_UP 12 //12px space used to show information on up of the display

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*----------------------------*/

int channel_num = 2;

// FALGS
// use the flags to switch running modes

// 1 for wait; 0 for stop.
int wait = 1;

// 0 for enable both CHA and CHB; 1 for enable CHA only; 2 for enable CHB only; 3 for FFT.
int chan = 3;

// the number of trigger voltage with unit in millivolt. only CHA can be used to trigger.
float tri_voltage = 0;

int time_scale = 10; // ms/div
int amp_scale = 1; // V/div

// 0 for normal; 1 for auto.
int mode = 0;

// 0 for channel, 1 for trigger, 2 for mode.
int menu_switch = 0;

int state = 0;

// Variables needed for running.

float adc_factor_1 = (1*3.3/4095); //CHA
float adc_factor_2 = (1*3.3/4095); //CHB

volatile uint16_t adc_values[ADC_MAX_NUM]; //in dma
volatile float adc_voltages_1[MAX_SAMPLE_NUM] = {0}; //change adc_values to voltages
int adc_voltages_index_1 = 0; //pointer in adc_voltages, increase to MAX_SAMPLE_NUM and back to 0
volatile float adc_voltages_2[MAX_SAMPLE_NUM] = {0};
int adc_voltages_index_2 = 0;
int plot_values_1[128]; // CHA plot
int plot_values_2[128]; // CHB plot

// in interrupt block
////Calculating sampling frequency
//int freq_samp = 128/(time_scale*HORIZONTAL_BLOK_NUM);

/*---------------------------*/

void init() {
	HAL_TIM_Base_Init(&htim2);
	HAL_TIM_Base_Start_IT(&htim2);
//	HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_3); //for debug output compare on PA8

	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_values, ADC_MAX_NUM);

	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	ssd1306_Init();
}

//adc convert
float adc_convert(float adc_value, float factor) {
	return adc_value*factor;
}

//plot convert
//voltage -> (127-bottom, 0+up)
void plot_convert(float amp_low, float amp_high) {
	for (int i=0; i<128; i++){
		plot_values_1[i] = (float)(127-INFO_PIXEL_BOTTOM - INFO_PIXEL_UP)/(amp_low-amp_high)*adc_voltages_1[i]
			+ 127-INFO_PIXEL_BOTTOM - (127-INFO_PIXEL_BOTTOM - INFO_PIXEL_UP)/(amp_low-amp_high)*amp_low;
		plot_values_2[i] = (float)(127-INFO_PIXEL_BOTTOM - INFO_PIXEL_UP)/(amp_low-amp_high)*adc_voltages_2[i]
			+ 127-INFO_PIXEL_BOTTOM - (127-INFO_PIXEL_BOTTOM - INFO_PIXEL_UP)/(amp_low-amp_high)*amp_low;
	}
}

//help function
void float2str(char* str,float value)
{
	int Head = (int)value;
	int Point = (int)((value - Head)*10.0);
	sprintf(str, "%d.%d", Head, Point);

}

//update plot
void update_plot() {
	// update wave
	ssd1306_Fill(White);
	for( int x = 0; x < 128; x++ ) {
		ssd1306_DrawPixel(x, plot_values_1[x], Black);
		ssd1306_DrawPixel(x, plot_values_2[x], Black);
	}
	ssd1306_Line(0, INFO_PIXEL_UP, 127, INFO_PIXEL_UP, Black);

	// update upon info area
	// wait
	ssd1306_SetCursor(2, 1);
	switch (wait) {
		case 1:
			ssd1306_WriteString("W", Font_7x10, Black);
			break;
		case 0:
			ssd1306_WriteString("S", Font_7x10, Black);
			break;
		default:
			break;
	}

	// CH
	ssd1306_SetCursor(15, 1);
	switch (chan) {
		case 0:
			ssd1306_WriteString("A/B", Font_7x10, Black);
			break;
		case 1:
			ssd1306_WriteString("CHA", Font_7x10, Black);
			break;
		case 2:
			ssd1306_WriteString("CHB", Font_7x10, Black);
			break;
		case 3:
			ssd1306_WriteString("FFT", Font_7x10, Black);
			break;
		default:
			break;
	}

	//time scale
	ssd1306_SetCursor(40, 1);
	char str_time_scale[10] = {0};
	sprintf(str_time_scale,"%d",time_scale);
	ssd1306_WriteString(str_time_scale, Font_7x10, Black);
	ssd1306_WriteString("ms", Font_7x10, Black);

	//amp scale
	ssd1306_SetCursor(75, 1);
	char str_amp_scale[10] = {0};
	sprintf(str_amp_scale,"%d",amp_scale);
	ssd1306_WriteString(str_amp_scale, Font_7x10, Black);
	ssd1306_WriteString("V", Font_7x10, Black);


	//trigger
	ssd1306_SetCursor(100, 1);
	char str_tri_voltage[10];
	char* p = str_tri_voltage;
	float2str(p, tri_voltage);
	ssd1306_WriteString(str_tri_voltage, Font_7x10, Black);
	ssd1306_WriteString("V", Font_7x10, Black);

	//mode
	ssd1306_SetCursor(85, 118);
	switch (mode) {
		case 0:
			ssd1306_WriteString("Normal", Font_7x10, Black);
			break;
		case 1:
			ssd1306_WriteString("Auto", Font_7x10, Black);
			break;
		default:
			break;
	}


	ssd1306_Line(0, 127 - INFO_PIXEL_BOTTOM, 127, 127 - INFO_PIXEL_BOTTOM, Black);
	ssd1306_UpdateScreen();
}

void loop() {
	plot_convert(0, 3.3);
	update_plot();
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
  MX_SPI2_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  loop();

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 15;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	// 转换完成后，将两个�?�道的�?�分别�?�入adc_voltages_1和adc_voltages_2，并闪烁指示�????????
	float voltage1 = 0;
	float voltage2 = 0;
	for (int i=0; i<ADC_MAX_NUM; i++) {
		if (i%2==0){
			voltage1 += adc_convert(adc_values[i], adc_factor_1);
		}
		if (i%2==1){
			voltage2 += adc_convert(adc_values[i], adc_factor_2);
		}
	}
	voltage1 = voltage1/(ADC_MAX_NUM/2); //(ADC_MAX_NUM)�????????定要打括�????????
	voltage2 = voltage2/(ADC_MAX_NUM/2);
	adc_voltages_1[adc_voltages_index_1] = voltage1;
	adc_voltages_2[adc_voltages_index_2] = voltage2;
	if (adc_voltages_index_1<MAX_SAMPLE_NUM){
		adc_voltages_index_1++;
	} else adc_voltages_index_1 = 0;
	if (adc_voltages_index_2<MAX_SAMPLE_NUM){
			adc_voltages_index_2++;
		} else adc_voltages_index_2 = 0;
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

	// �????????始下�????????次转�????????
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_values, ADC_MAX_NUM);
}


uint32_t previousMillis = 0;
int num =0;
int flag = 0;  //标志�????????
int CW_1 = 0;
int CW_2 = 0;
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == MENU_Pin) {
		if (menu_switch<2&&menu_switch>=0) {
			menu_switch++;
		}else {
			menu_switch = 0;
		}
	}
	if (GPIO_Pin == ENCODER_Z_Pin) {
		if (wait==0) {
			wait = 1;
		} else {
			wait = 0;
		}
//		__HAL_TIM_DISABLE(&htim2);
//		 htim2.Instance->CNT=0;
//		 htim2.Instance->ARR=5000-1;
//		__HAL_TIM_ENABLE(&htim2);
	}

	if (GPIO_Pin == ENCODER_A_Pin) {
		uint32_t currentMillis = HAL_GetTick();
		if (currentMillis - previousMillis < 50) {
			return;
		}
		int alv = HAL_GPIO_ReadPin(ENCODER_A_GPIO_Port, ENCODER_A_Pin);
		int blv = HAL_GPIO_ReadPin(ENCODER_B_GPIO_Port, ENCODER_B_Pin);
		if (flag == 0 && alv == GPIO_PIN_RESET) {
			CW_1 = blv;
			flag = 1;
		}
		if (flag && alv) {
			CW_2 = !blv;  //取反是因�???????? alv,blv必然异步，一高一�????????
			if (CW_1 && CW_2) {
				num++;
				previousMillis = HAL_GetTick();
			}
			if (CW_1 == 0 && CW_2 == 0) {
				num--;
				previousMillis = HAL_GetTick();
			}
			flag = 0;
		}
	}
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == ENCODER_A_Pin) {
		uint32_t currentMillis = HAL_GetTick();
		if (currentMillis - previousMillis < 50) {
			return;
		}
		int alv = HAL_GPIO_ReadPin(ENCODER_A_GPIO_Port, ENCODER_A_Pin);
		int blv = HAL_GPIO_ReadPin(ENCODER_B_GPIO_Port, ENCODER_B_Pin);
		if (flag == 0 && alv == GPIO_PIN_RESET) {
			CW_1 = blv;
			flag = 1;
		}
		if (flag && alv) {
			CW_2 = !blv;  //取反是因�???????? alv,blv必然异步，一高一�????????
			if (CW_1 && CW_2) {
				num++;
				previousMillis = HAL_GetTick();
			}
			if (CW_1 == 0 && CW_2 == 0) {
				num--;
				previousMillis = HAL_GetTick();
			}
			flag = 0;
		}
	}
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

