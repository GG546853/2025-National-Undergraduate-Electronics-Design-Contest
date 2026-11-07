/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "can.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "uart_me.h"
#include "bsp_can.h"
#include "dm_motor_ctrl.h"
#include "dm_motor_drv.h"
#include "key.h"
#include "OLED.h"
#include <string.h>
#include <stdio.h>
#include "wit_c_sdk.h"
#include "hal_hard_i2c.h"
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
void WitDataUpdate(uint32_t uiReg, uint32_t uiRegNum);
void sdk_delay_ms(uint16_t nms);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define FIND_ANGLE     20

uint16_t g_Tim3_Num = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
			if (HAL_GetTick() - vision_data.last_update_tick > 200) {
        vision_data.is_tracking = 0;
			}

			if(gimbal_ctrl.mode == 1)
			{
				if(vision_data.is_tracking == 1)
				{
					WitReadReg(Roll, 3);
					float current_gyro_yaw = sReg[Yaw] / 32768.0f * 180.0f;
					
					gimbal_ctrl.yaw_target_angle = current_gyro_yaw * (PI / 180.0f);
					gimbal_ctrl.pitch_target_angle = motor[Motor2].para.pos;
					
					gimbal_ctrl.last_yaw_target_angle = gimbal_ctrl.yaw_target_angle;
					gimbal_ctrl.last_pitch_target_angle = gimbal_ctrl.pitch_target_angle;
					
					gimbal_ctrl.pid_yaw.i_err = 0;
					gimbal_ctrl.pid_yaw.last_err = 0;
					gimbal_ctrl.pid_pitch.i_err = 0;
					gimbal_ctrl.pid_pitch.last_err = 0;
					
					gimbal_ctrl.mode = 2;
				}
			}
			else if(gimbal_ctrl.mode == 2 || gimbal_ctrl.mode == 3)
			Gimbal_Control_Loop();
    }
		
		if (htim->Instance == TIM4)
    {
			KeyScan(&Key1);
			KeyScan(&Key2);
			KeyScan(&Key3);
    }
		
		if (htim->Instance == TIM5)    //  0.5s
    {
			
    }
}


volatile uint8_t System_Busy = 0;


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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART6_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
	WitInit(WIT_PROTOCOL_I2C, 0x50);
	WitI2cFuncRegister(HAL_Hardware_I2C_Write, HAL_Hardware_I2C_Read);
	WitDelayMsRegister(sdk_delay_ms);
  WitRegisterCallBack(WitDataUpdate);
	
	
	Gimbal_Init();
	OLED_Init();
	HAL_TIM_Base_Start_IT(&htim4); 
	
	//HAL_TIM_Base_Start_IT(&htim3);
	
	char c_Oled[20];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		Process_Vision_Packet();
		/*Gimbal Enable and Disable*/
			if(Key3_Nub == 1 && System_Busy == 0)
		{
			System_Busy = 1;
			vision_data.is_tracking = 0; 
			vision_data.last_update_tick = HAL_GetTick(); 
			Motor_Enable();
			read_motor_ctrl_fbdata(&hcan1, motor[Motor1].id);
			read_motor_ctrl_fbdata(&hcan2, motor[Motor2].id);
			HAL_Delay(50); 
			
			WitReadReg(Roll, 3);
      fGyro_ang[2] = sReg[Yaw] / 32768.0f * 180.0f;
			gimbal_ctrl.yaw_target_angle = fGyro_ang[2] * PI / 180.0f;
			gimbal_ctrl.pitch_target_angle = motor[Motor2].para.pos;	
			gimbal_ctrl.last_yaw_target_angle = gimbal_ctrl.yaw_target_angle;
      gimbal_ctrl.last_pitch_target_angle = gimbal_ctrl.pitch_target_angle;
			
//			gimbal_ctrl.yaw_target_angle = motor[Motor1].para.pos;
//			gimbal_ctrl.pitch_target_angle = motor[Motor2].para.pos;
			gimbal_ctrl.pid_yaw.i_err = 0;
			gimbal_ctrl.pid_yaw.out = 0;
			gimbal_ctrl.pid_pitch.i_err = 0;
			gimbal_ctrl.pid_pitch.out = 0;

			HAL_TIM_Base_Start_IT(&htim3);
			Key3_Nub = -1;
			System_Busy = 0;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
		}
		else if(Key3_Nub == 2 && System_Busy == 0)
		{
			System_Busy = 1;
			HAL_TIM_Base_Stop_IT(&htim3);
			Motor_Disable();
			Key3_Nub = -2;
			System_Busy = 0;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
			is_target_locked = 0;
		}
		/*Gimbal Zero Position Setting*/
		if(Key2_Nub == 1 && System_Busy == 0)
		{
			Key2_Nub = 0;
			//Gimbal_Set_Zero();
			Sensor_ResetZ_Hardware();
			System_Busy = 0;
		}

		/***OLED***/
		if(Key1_Nub)
		{
			sprintf(c_Oled, "Mode:%d", gimbal_ctrl.mode);
			OLED_ShowString(1, 1, c_Oled, OLED_8X16);
			OLED_Update();
			if(gimbal_ctrl.mode == 2) {
				PID_Init(&gimbal_ctrl.pid_yaw, 0.015f, 0.0002f, 0.00f, 10.0f, 1.0f);  //0.015 0.0002 0.00
				PID_Init(&gimbal_ctrl.pid_pitch, 0.0075f, 0.0f, 0.000f, 5.0f, 2.0f); //0.0075 0.0 0.0
			}
			if(gimbal_ctrl.mode == 3) {
				PID_Init(&gimbal_ctrl.pid_yaw, 0.014f, 0.000f, 0.00f, 10.0f, 1.0f);  //0.015 0.0002 0.00
				PID_Init(&gimbal_ctrl.pid_pitch, 0.0075f, 0.0f, 0.000f, 5.0f, 2.0f); //0.0075 0.0 0.0
			}

			if(gimbal_ctrl.mode == 1) PID_Init(&gimbal_ctrl.pid_yaw, 0.00f, 0.0f, 0.0f, 10.0f, 2.0f);  //8.0
		}

		/*gimbal_ctrl.mode1*/
		if(gimbal_ctrl.mode == 1)
		{
			Motor_Two_Ctrl(0, 0.0f, 4.0f, 0.03f, 0.0f);
			Motor_One_Spd_Ctrl(2.5);
			HAL_Delay(1);
		}
		
		
//		printf("%d\n",(int)fGyro_ang[2]);
//		HAL_Delay(1);
		//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
		
//		WitReadReg(Roll, 3);
//    fGyro_ang[0] = sReg[Roll] / 32768.0f * 180.0f;
//    fGyro_ang[1] = sReg[Pitch] / 32768.0f * 180.0f;
//		fGyro_ang[2] = sReg[Yaw] / 32768.0f * 180.0f;
//		HAL_Delay(1);
		
		//Motor_One_Spd_Ctrl(-1 * fGyro[2]/180.0f * 3.1416f);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

int fputc(int ch, FILE *f) {
    HAL_UART_Transmit(&huart6, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
  }
}

void WitDataUpdate(uint32_t uiReg, uint32_t uiRegNum)
{
	
}
void sdk_delay_ms(uint16_t nms)
{
    HAL_Delay(nms);
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
#ifdef USE_FULL_ASSERT
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
