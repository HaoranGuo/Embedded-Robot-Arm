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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void user_Function(uint8_t* string){
    uint8_t* pt = string + 1;
    uint8_t num = 0;
    while(*string != '+'){
        string++;
        pt = string + 1;
        num++;
    }
    if ((*pt == 'L' || *pt == 'l') &&
        (*(pt+1) == 'E' || *(pt+1) == 'e') &&
        (*(pt+2) == 'D' || *(pt+2) == 'd') &&
        (*(pt+3) == ',')) {
        pt = pt + 4;
        uint8_t idx;
        uint8_t led_num = 0;
        for (idx = 0; *(pt + idx) != ','; idx++) {
            led_num *= 10;
            led_num += *(pt + idx) - '0';
        }
        pt = pt + idx + 1;
        uint8_t led_sta = 0;
        for (idx = 0; *(pt + idx) != ','; idx++) {
            led_sta *= 10;
            led_sta += *(pt + idx) - '0';
        }
        if (led_num == 0) {
            if (led_sta == 0) {
                LED0_OFF();
                uint8_t TX_Str[] = "\r\nLED0: Close\r\n";
                HAL_UART_Transmit(&huart1, TX_Str, sizeof(TX_Str), 1000);
            } else if (led_sta == 1) {
                LED0_ON();
                uint8_t TX_Str[] = "\r\nLED0: Open\r\n";
                HAL_UART_Transmit(&huart1, TX_Str, sizeof(TX_Str), 1000);
            } else {
                uint8_t TX_Str[] = "\r\nLED0: Status Error\r\n";
                HAL_UART_Transmit(&huart1, TX_Str, sizeof(TX_Str), 1000);
            }
        } else if (led_num == 1) {
            if (led_sta == 0) {
                LED1_OFF();
                uint8_t TX_Str[] = "\r\nLED1: Close\r\n";
                HAL_UART_Transmit(&huart1, TX_Str, sizeof(TX_Str), 1000);
            } else if (led_sta == 1) {
                LED1_ON();
                uint8_t TX_Str[] = "\r\nLED1: Open\r\n";
                HAL_UART_Transmit(&huart1, TX_Str, sizeof(TX_Str), 1000);
            } else {
                uint8_t TX_Str[] = "\r\nLED1: Status Error\r\n";
                HAL_UART_Transmit(&huart1, TX_Str, sizeof(TX_Str), 1000);
            }
        } else {
            uint8_t TX_Str[] = "\r\nLED: Index Error\r\n";
            HAL_UART_Transmit(&huart1, TX_Str, sizeof(TX_Str), 1000);
        }
    }
    else if ((*pt == 'S' || *pt == 's') &&
             (*(pt+1) == 'E' || *(pt+1) == 'e') &&
             (*(pt+2) == 'R' || *(pt+2) == 'r') &&
             (*(pt+3) == ',')) {
        pt = pt + 4;
        uint8_t idx;
        uint8_t servo_num = 0;
        for (idx = 0; *(pt+idx)!= ','; idx++){
            servo_num *= 10;
            servo_num += *(pt+idx) - '0';
        }
        pt = pt + idx + 1;
        if (servo_num < 0 || servo_num > 6){
            uint8_t TX_Str[] = "\r\nSERVO: Index Error\r\n";
            HAL_UART_Transmit(&huart1, TX_Str, sizeof(TX_Str), 1000);
        }
        else{
            double servo_angle = 0;
            double servo_angle_tmp = 0;
            uint8_t flag = 0;
            for (idx = 0; *(pt + idx) != ','; idx++) {
                if (*(pt + idx) == '.') {
                    flag = 1;
                    continue;
                }

                if (flag == 0) {
                    servo_angle *= 10;
                    servo_angle = servo_angle + (double)(*(pt + idx) - '0');
                }
                else {
                    uint8_t tmp;
                    for (tmp = 0; tmp < flag; tmp++) {
                        servo_angle_tmp = (double)(*(pt + idx) - '0') / 10;
                    }
                    servo_angle += servo_angle_tmp;
                    flag++;
                }
            }

            if (servo_angle < 0 || servo_angle > 360){
                uint8_t TX_Str[] = "\r\nSER: Angle Setting Error\r\n";
                HAL_UART_Transmit(&huart1, TX_Str, sizeof(TX_Str), 1000);
            }
            else{
                if (servo_num == 0){
                    servo_pulse_last[0] = servo_pulse[0];
                    servo_pulse[0] = (uint16_t)(servo_angle / SERVO0_MAX_ANGLE * 2000 + 500);
                }
                else if (servo_num == 1){
                    servo_pulse_last[1] = servo_pulse[1];
                    servo_pulse[1] = (uint16_t)(servo_angle / SERVO1_MAX_ANGLE * 2000 + 500);
                }
                else if (servo_num == 2){
                    servo_pulse_last[2] = servo_pulse[2];
                    servo_pulse[2] = (uint16_t)(servo_angle / SERVO2_MAX_ANGLE * 2000 + 500);
                }
                else if (servo_num == 3){
                    servo_pulse_last[3] = servo_pulse[3];
                    servo_pulse[3] = (uint16_t)(servo_angle / SERVO3_MAX_ANGLE * 2000 + 500);
                }
                else if (servo_num == 4){
                    servo_pulse_last[4] = servo_pulse[4];
                    servo_pulse[4] = (uint16_t)(servo_angle / SERVO4_MAX_ANGLE * 2000 + 500);
                }
                else if (servo_num == 5){
                    servo_pulse_last[5] = servo_pulse[5];
                    servo_pulse[5] = (uint16_t)(servo_angle / SERVO5_MAX_ANGLE * 2000 + 500);
                }

                uint8_t TX_Str[] = "\r\nSER: Move Successfully\r\n";
                HAL_UART_Transmit(&huart1, TX_Str, sizeof(TX_Str), 1000);

                servo_Move2Goal();
            }
        }
    }
    else if ((*pt == 'S' || *pt == 's') &&
             (*(pt+1) == 'E' || *(pt+1) == 'e') &&
             (*(pt+2) == 'R' || *(pt+2) == 'r') &&
             (*(pt+3) == 'V' || *(pt+3) == 'v') &&
             (*(pt+4) == 'O' || *(pt+4) == 'o') &&
             (*(pt+5) == 'S' || *(pt+5) == 's') &&
             (*(pt+6) == ',')) {
        pt = pt + 7;
        uint8_t idx;
        uint8_t valid = 0;
        double angle[USED_SERVO_NUM] = {0};
        double angle_tmp[USED_SERVO_NUM] = {0};
        for(uint8_t i = 0; i < USED_SERVO_NUM; ++i){
            uint8_t flag = 0;
            for (idx = 0; *(pt + idx) != ','; idx++) {
                if (*(pt + idx) == '.') {
                    flag = 1;
                    continue;
                }

                if (flag == 0) {
                    angle[i] *= 10;
                    angle[i] = angle[i] + (double)(*(pt + idx) - '0');
                }
                else {
                    uint8_t tmp;
                    for (tmp = 0; tmp < flag; tmp++) {
                        angle_tmp[i] = (double)(*(pt + idx) - '0') / 10;
                    }
                    angle[i] += angle_tmp[i];
                    flag++;
                }
            }
            if (angle[i] < 0 || angle[i] > 360){
                valid = 1;
                break;
            }
            pt = pt + idx + 1;
            servo_pulse_last[i] = servo_pulse[i];
            servo_pulse[i] = (uint16_t)(angle[i] / SERVO0_MAX_ANGLE * 2000 + 500);
        }
        if (valid == 0){
            uint8_t TX_Str[] = "\r\nSERVOS: Move Successfully\r\n";
            HAL_UART_Transmit(&huart1, TX_Str, sizeof(TX_Str), 1000);

            servo_Move2Goal();
        }
        else{
            uint8_t TX_Str[] = "\r\nSERVOS: Angle Setting Error\r\n";
            HAL_UART_Transmit(&huart1, TX_Str, sizeof(TX_Str), 1000);
        }
    }
    else{
        uint8_t TX_Str[] = "\r\nUnknown Command\r\n";
        HAL_UART_Transmit(&huart1, TX_Str, sizeof(TX_Str), 1000);
    }
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

    BEEP_ON();
    servo_Init();
    BEEP_OFF();

    uint8_t TX_Str1[] = "\r\nInit Successfully\r\n";
    uint8_t TX_Str2[] = "\r\nType: \"+FUNCTION,PARAMETER1,PARAMETER2,...\" to command the board. E.g.\"+LED,1,1,\" can open the LED1.\r\n";
    HAL_UART_Transmit(&huart1, TX_Str1, sizeof(TX_Str1), 1000);
    HAL_UART_Transmit(&huart1, TX_Str2, sizeof(TX_Str2), 1000);

    HAL_UART_Receive_IT(&huart1, (uint8_t *)rx_buffer, RXBUFFERSIZE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

      if (usart_rx_sta & 0x8000){
          user_Function(usart_rx_buf);
          usart_rx_sta = 0;
      }
      else {
          HAL_Delay(10);
      }

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
