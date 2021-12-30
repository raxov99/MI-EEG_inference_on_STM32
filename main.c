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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM4
#include "arm_math.h"
#include "ai_platform.h"
#include "ai_datatypes_defines.h"
#include "a0.h"
#include "a1.h"
#include "b.h"
#include "c.h"
#include "a0_data.h"
#include "a1_data.h"
#include "b_data.h"
#include "c_data.h"
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
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static ai_handle a0 = AI_HANDLE_NULL;
static ai_network_report a0_info;
static ai_handle a1 = AI_HANDLE_NULL;
static ai_network_report a1_info;
static ai_handle b = AI_HANDLE_NULL;
static ai_network_report b_info;
static ai_handle c = AI_HANDLE_NULL;
static ai_network_report c_info;
AI_ALIGNED(4)
static ai_u8 activations_a0[AI_A0_DATA_ACTIVATIONS_SIZE];
AI_ALIGNED(4)
static ai_u8 activations_a1[AI_A1_DATA_ACTIVATIONS_SIZE];
AI_ALIGNED(4)
static ai_u8 activations_b[AI_B_DATA_ACTIVATIONS_SIZE];
AI_ALIGNED(4)
static ai_u8 activations_c[AI_C_DATA_ACTIVATIONS_SIZE];
AI_ALIGNED(4)
static ai_u8 in_data_a0[AI_A0_IN_1_SIZE_BYTES];
AI_ALIGNED(4)
static ai_u8 in_data_a1[AI_A1_IN_1_SIZE_BYTES];
AI_ALIGNED(4)
static ai_u8 in_data_b[AI_B_IN_1_SIZE_BYTES];
AI_ALIGNED(4)
static ai_u8 in_data_c[AI_C_IN_1_SIZE_BYTES];
AI_ALIGNED(4)
static ai_u8 out_data_c[AI_C_OUT_1_SIZE_BYTES];


#define N (64)
#define C (64)
#define B  (4)
#define n (17)

static float i_data[B * C];

static arm_rfft_fast_instance_f32 rfft_fast_i;

static uint8_t links[n * n] = { 0,  0,  0,  0,  0,  0,  0, 29,  0, 38,  0,  0,  0,  0,  0,  0,  0,
								0,  0, 23, 24, 24, 24, 28, 29, 37, 37, 38, 40, 40, 40, 44, 44,  0,
								0, 23, 23, 23, 24, 28, 28, 36, 37, 37,  7,  7, 40, 42, 42, 44,  0,
								0, 22, 23, 23, 27, 28, 35, 36, 36,  6,  7,  7, 14, 42, 42, 46,  0,
								0, 22, 22, 27, 27, 34, 35, 35,  6,  6,  6, 13, 14, 14, 46, 46,  0,
								0, 22, 26, 26, 34, 34, 34,  5,  5,  5, 13, 13, 13, 21, 21, 46,  0,
								0, 26, 26, 33, 33, 34,  4,  4,  5, 12, 12, 13, 20, 21, 21, 55,  0,
							   25, 25, 32, 32, 33,  3,  4,  4, 11, 12, 12, 19, 20, 20, 54, 54, 55,
							    0, 31, 31, 32,  2,  3,  3, 11, 11, 11, 19, 19, 20, 53, 54, 54,  0,
							   30, 31, 31,  2,  2,  3, 10, 10, 11, 18, 18, 19, 52, 53, 53, 60, 60,
							    0, 30,  1,  1,  2,  9, 10, 10, 17, 18, 18, 51, 52, 52, 59, 59,  0,
								0, 39,  1,  1,  9,  9,  9, 17, 17, 17, 51, 51, 51, 59, 59, 63,  0,
								0, 39, 39,  8,  8,  9, 16, 16, 16, 50, 50, 51, 58, 58, 63, 63,  0,
								0, 39, 41, 41,  8, 15, 15, 16, 49, 49, 50, 57, 58, 62, 62, 63,  0,
								0, 43, 41, 41, 45, 15, 15, 48, 48, 49, 57, 57, 61, 62, 62, 64,  0,
								0, 43, 43, 45, 45, 45, 47, 48, 48, 56, 57, 61, 61, 61, 64, 64,  0,
								0,  0,  0,  0,  0,  0,  0, 47,  0, 56,  0,  0,  0,  0,  0,  0,  0 };

static float w[N] = {-1.3877788e-17,  8.9841132e-04,  3.6318530e-03,  8.3126994e-03,
					  1.5120840e-02,  2.4292914e-02,  3.6107894e-02,  5.0869633e-02,
					  6.8887137e-02,  9.0453424e-02,  1.1582390e-01,  1.4519517e-01,
					  1.7868534e-01,  2.1631649e-01,  2.5800049e-01,  3.0352849e-01,
					  3.5256478e-01,  4.0464568e-01,  4.5918295e-01,  5.1547271e-01,
					  5.7270867e-01,  6.3000000e-01,  6.8639290e-01,  7.4089545e-01,
					  7.9250437e-01,  8.4023350e-01,  8.8314229e-01,  9.2036361e-01,
					  9.5112985e-01,  9.7479635e-01,  9.9086124e-01,  9.9898094e-01,
					  9.9898094e-01,  9.9086124e-01,  9.7479635e-01,  9.5112985e-01,
					  9.2036361e-01,  8.8314229e-01,  8.4023350e-01,  7.9250437e-01,
					  7.4089545e-01,  6.8639290e-01,  6.3000000e-01,  5.7270867e-01,
					  5.1547271e-01,  4.5918295e-01,  4.0464568e-01,  3.5256478e-01,
					  3.0352849e-01,  2.5800049e-01,  2.1631649e-01,  1.7868534e-01,
					  1.4519517e-01,  1.1582390e-01,  9.0453424e-02,  6.8887137e-02,
					  5.0869633e-02,  3.6107894e-02,  2.4292914e-02,  1.5120840e-02,
					  8.3126994e-03,  3.6318530e-03,  8.9841132e-04, -1.3877788e-17 };

static float t_std = 72.335754, img_std[B] = { 4392.27370734, 1092.39808504,  627.72994599,  345.57622865 };

int acquire_and_process_data_a(void *data0, void *data1)
{
  float t_data[N], f_data[N], temp[N], temp_0[N], temp_1[N*4];
  for (uint8_t i = 0; i < C; i++) {  //for each channel
	HAL_UART_Receive(&huart1, (uint8_t *)temp, N * 4, 0xFFFF);  //get t_data[:][channel]

	arm_scale_f32(temp, 1/t_std, temp_0, N);  //normalize t_data[:][channel]
	for (int j = 0; j < N; j++) //for each sample
	  *((float *)data0 + j*C + i) = temp_0[j];  //load t_data[sample][channel]

    arm_mult_f32(temp, w, t_data, N);  //Blackman windowing
    arm_rfft_fast_f32(&rfft_fast_i, t_data, temp, 0);  //fft
    arm_scale_f32(temp, (float)2/N, f_data, N);  //get f_data[channel]
    arm_cmplx_mag_f32(f_data, temp, N / 2);  //get magnitudes
    arm_power_f32(temp + 1, 1, temp_1 + C*0 + i);  //delta power spectrum of channel
    arm_power_f32(temp + 2, 1, temp_1 + C*1 + i);  //theta power spectrum of channel
    arm_power_f32(temp + 3, 3, temp_1 + C*2 + i);  //mu power spectrum of channel
    arm_power_f32(temp + 6, 6, temp_1 + C*3 + i);  //beta power spectrum of channel
	HAL_UART_Transmit(&huart1, (uint8_t *)&i, 1, 0xFFFF);  //send channel_ready
  }
  for (int i = 0; i < B; i++)  //for each band
	arm_scale_f32(temp_1 + i*C, 1/img_std[i], i_data + i*C, C);  //normalize image[band]
  for (int i = 0; i < n*n; i++)  //for each pixel
    for (int j = 0; j < B; j++)  //for each band
	  *((float *)data1 + i*B + j) = links[i] ? i_data[j*C + (links[i] - 1)] : 0;  //load image[pixel][band]

  return 0;
}

static int ai_run_a0(void *data_in, void *data_out)
{
  ai_buffer *ai_input = a0_info.inputs;
  ai_buffer *ai_output = a0_info.outputs;

  ai_input[0].data = AI_HANDLE_PTR(data_in);
  ai_output[0].data = AI_HANDLE_PTR(data_out);

  ai_a0_run(a0, ai_input, ai_output);

  return 0;
}

static int ai_run_a1(void *data_in, void *data_out)
{
  ai_buffer *ai_input = a1_info.inputs;
  ai_buffer *ai_output = a1_info.outputs;

  ai_input[0].data = AI_HANDLE_PTR(data_in);
  ai_output[0].data = AI_HANDLE_PTR(data_out);

  ai_a1_run(a1, ai_input, ai_output);

  return 0;
}

static int ai_run_b(void *data_in, void *data_out)
{
  ai_buffer *ai_input = b_info.inputs;
  ai_buffer *ai_output = b_info.outputs;

  ai_input[0].data = AI_HANDLE_PTR(data_in);
  ai_output[0].data = AI_HANDLE_PTR(data_out);

  ai_b_run(b, ai_input, ai_output);

  return 0;
}

static int ai_run_c(void *data_in, void *data_out)
{
  ai_buffer *ai_input = c_info.inputs;
  ai_buffer *ai_output = c_info.outputs;

  ai_input[0].data = AI_HANDLE_PTR(data_in);
  ai_output[0].data = AI_HANDLE_PTR(data_out);

  ai_c_run(c, ai_input, ai_output);

  return 0;
}

int post_process_c(void *data)
{
  int8_t *out = (int8_t *)data, idx = 0, p = 0;
  if  (out[0] > 0)
	for (int i = 1; i<=4; i++)
	  if (out[i] > p) {
		p = out[i];
		idx = i;
	  }
  HAL_UART_Transmit(&huart1, (uint8_t *)&idx, 1, 0xFFFF);
  return 0;
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
  MX_CRC_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  ai_a0_create(&a0, AI_A0_DATA_CONFIG);
  ai_a1_create(&a1, AI_A1_DATA_CONFIG);
  ai_b_create(&b, AI_B_DATA_CONFIG);
  ai_c_create(&c, AI_C_DATA_CONFIG);

  const ai_network_params params_a0 = AI_NETWORK_PARAMS_INIT(
		AI_A0_DATA_WEIGHTS(ai_a0_data_weights_get()),
	  	AI_A0_DATA_ACTIVATIONS(activations_a0) );
  const ai_network_params params_a1 = AI_NETWORK_PARAMS_INIT(
  		AI_A1_DATA_WEIGHTS(ai_a1_data_weights_get()),
  	  	AI_A1_DATA_ACTIVATIONS(activations_a1) );
  const ai_network_params params_b = AI_NETWORK_PARAMS_INIT(
		AI_B_DATA_WEIGHTS(ai_b_data_weights_get()),
		AI_B_DATA_ACTIVATIONS(activations_b) );
  const ai_network_params params_c = AI_NETWORK_PARAMS_INIT(
		AI_C_DATA_WEIGHTS(ai_c_data_weights_get()),
		AI_C_DATA_ACTIVATIONS(activations_c) );

  ai_a0_init(a0, &params_a0);
  ai_a1_init(a1, &params_a1);
  ai_b_init(b, &params_b);
  ai_c_init(c, &params_c);
  ai_a0_get_info(a0, &a0_info);
  ai_a1_get_info(a1, &a1_info);
  ai_b_get_info(b, &b_info);
  ai_c_get_info(c, &c_info);

  arm_rfft_fast_init_f32(&rfft_fast_i, 64);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int8_t ready = 1;
  while (1) {
    for (int i = 0; i < 10; i++) {
	  acquire_and_process_data_a(in_data_a0, in_data_a1);
	  ai_run_a0(in_data_a0, in_data_b + i*AI_A0_OUT_1_SIZE_BYTES + i*AI_A1_OUT_1_SIZE_BYTES);
	  ai_run_a1(in_data_a1, in_data_b + (i+1)*AI_A0_OUT_1_SIZE_BYTES + i*AI_A1_OUT_1_SIZE_BYTES);
	  HAL_UART_Transmit(&huart1, (uint8_t *)&ready, 1, 0xFFFF);
    }
    ai_run_b(in_data_b, in_data_c);
    ai_run_c(in_data_c, out_data_c);
    post_process_c(out_data_c);
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
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  huart1.Init.BaudRate = 115200*16;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_8;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

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

