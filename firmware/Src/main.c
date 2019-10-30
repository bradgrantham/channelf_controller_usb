/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void delay_ms(int ms)
{
    HAL_Delay(ms);
}

void delay_100ms(unsigned char count)
{
    HAL_Delay(count * 100);
}

#define INFO_LED_PIN_MASK LED_RED_Pin
#define INFO_LED_PORT LED_RED_GPIO_Port

#define HEARTBEAT_LED_PIN_MASK LED_YELLOW_Pin
#define HEARTBEAT_LED_PORT LED_YELLOW_GPIO_Port

void LED_set_panic(int on)
{
    HAL_GPIO_WritePin(INFO_LED_PORT, INFO_LED_PIN_MASK, on);
}

void LED_beat_heart()
{
    static int heartbeat_level = 1;
    static unsigned int previous_heartbeat_tick = 0;

    unsigned int now = HAL_GetTick();
    if(heartbeat_level == 1) {
        if(now - previous_heartbeat_tick > 350) {
            heartbeat_level = 0;
            HAL_GPIO_WritePin(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN_MASK, heartbeat_level);
            previous_heartbeat_tick = now;
        }
    } else {
        if(now - previous_heartbeat_tick > 650) {
            heartbeat_level = 1;
            HAL_GPIO_WritePin(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN_MASK, heartbeat_level);
            previous_heartbeat_tick = now;
        }
    }
}

void LED_set_info(int on)
{
    HAL_GPIO_WritePin(INFO_LED_PORT, INFO_LED_PIN_MASK, on);
}

void panic_worse()
{
    HAL_GPIO_WritePin(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN_MASK, 0);
    LED_set_panic(1);
    for(;;);
}

// void serial_flush();

void panic(void)
{
    static int entered = 0;

    LED_set_panic(1);

    HAL_GPIO_WritePin(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN_MASK, 1);

    int pin = 0;
    for(;;) {
        if(!entered) {
            // serial_flush() can itself panic(), so stop reentry here
            entered = 1;
            // serial_flush();
            entered = 0;
        }

        LED_set_panic(pin);
        pin = pin ? 0 : 1;
        delay_ms(100);
    }
}

#define TRANSMIT_BUFFER_SIZE 512
volatile unsigned char gTransmitBuffers[2][TRANSMIT_BUFFER_SIZE];
volatile int gNextTransmitBuffer = 0;
volatile int gTransmitBufferLengths[2] = {0, 0};
volatile int gUARTTransmitBusy = 0;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    gUARTTransmitBusy = 0;
}

void serial_try_to_transmit_buffers()
{
    if(!gUARTTransmitBusy && gTransmitBufferLengths[gNextTransmitBuffer] > 0) {
        gUARTTransmitBusy = 1;

        if(HAL_UART_Transmit_IT(&huart1, (uint8_t *)&gTransmitBuffers[gNextTransmitBuffer][0], gTransmitBufferLengths[gNextTransmitBuffer]) != HAL_OK) {
            panic();
        }

        gNextTransmitBuffer ^= 1;
        gTransmitBufferLengths[gNextTransmitBuffer] = 0;
    }
}

void serial_flush()
{
    while(gUARTTransmitBusy || gTransmitBufferLengths[gNextTransmitBuffer] > 0)
        serial_try_to_transmit_buffers();
}

void serial_enqueue_one_char(char c)
{
    do {
        // Transmit the current buffer if there is one and serial
        // port is not busy
        serial_try_to_transmit_buffers();

        // While there's no room in the current buffer, repeat until buffer becomes available
    } while(gTransmitBufferLengths[gNextTransmitBuffer] >= TRANSMIT_BUFFER_SIZE);

    int length = gTransmitBufferLengths[gNextTransmitBuffer];
    gTransmitBuffers[gNextTransmitBuffer][length] = c;
    gTransmitBufferLengths[gNextTransmitBuffer]++;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    panic();
}

void __io_putchar( char c )
{
    serial_enqueue_one_char(c);
}

int _write(int file, char *ptr, int len)
{
        int DataIdx;

                for (DataIdx = 0; DataIdx < len; DataIdx++)
                {
                   __io_putchar( *ptr++ );
                }
        return len;
}

const int C1 = 0;
const int C2 = 1;
const int NORTH = 0;
const int SOUTH = 1;
const int WEST = 2;
const int EAST = 3;
const int UP = 4;
const int DOWN = 5;
const int CW = 6;
const int CCW = 7;

uint8_t button_state[2][8];
uint64_t button_last[2][8];
uint64_t debounce_millis = 30;

typedef struct button_info
{
    GPIO_TypeDef* port;
    uint16_t pin;
} button_info;

button_info button_sources[2][8] =
{
    {
        { C1NORTH_GPIO_Port, C1NORTH_Pin },
        { C1SOUTH_GPIO_Port, C1SOUTH_Pin },
        { C1WEST_GPIO_Port, C1WEST_Pin },
        { C1EAST_GPIO_Port, C1EAST_Pin },
        { C1UP_GPIO_Port, C1UP_Pin },
        { C1DOWN_GPIO_Port, C1DOWN_Pin },
        { C1CW_GPIO_Port, C1CW_Pin },
        { C1CCW_GPIO_Port, C1CCW_Pin },
    },
    {
        { C2NORTH_GPIO_Port, C2NORTH_Pin },
        { C2SOUTH_GPIO_Port, C2SOUTH_Pin },
        { C2WEST_GPIO_Port, C2WEST_Pin },
        { C2EAST_GPIO_Port, C2EAST_Pin },
        { C2UP_GPIO_Port, C2UP_Pin },
        { C2DOWN_GPIO_Port, C2DOWN_Pin },
        { C2CW_GPIO_Port, C2CW_Pin },
        { C2CCW_GPIO_Port, C2CCW_Pin },
    },
};

void check_buttons()
{
    uint64_t now = HAL_GetTick();

    for(int controller = 0; controller < 2; controller++) {
        for(int button = 0; button < 8; button++) {
            int state = !HAL_GPIO_ReadPin(button_sources[controller][button].port, button_sources[controller][button].pin);

            if((state != button_state[controller][button]) && (now > button_last[controller][button] + debounce_millis)) {
                if(state) {
                    // printf("%d, %d pressed\n", controller, button);
                } else {
                    // printf("%d, %d released\n", controller, button);
                }
                serial_flush();
                button_state[controller][button] = state;
                button_last[controller][button] = now;
            }
        }
    }
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */

    printf("hello!\n");
    serial_flush();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

     check_buttons();
     LED_beat_heart();

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_RED_Pin|LED_YELLOW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_RED_Pin LED_YELLOW_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin|LED_YELLOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : C1EAST_Pin C1WEST_Pin C1SOUTH_Pin C1NORTH_Pin 
                           C2CCW_Pin C2CW_Pin C2DOWN_Pin C2UP_Pin 
                           C1CW_Pin */
  GPIO_InitStruct.Pin = C1EAST_Pin|C1WEST_Pin|C1SOUTH_Pin|C1NORTH_Pin 
                          |C2CCW_Pin|C2CW_Pin|C2DOWN_Pin|C2UP_Pin 
                          |C1CW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : C2NORTH_Pin C2SOUTH_Pin C1CCW_Pin C1DOWN_Pin 
                           C2EAST_Pin C2WEST_Pin C1UP_Pin */
  GPIO_InitStruct.Pin = C2NORTH_Pin|C2SOUTH_Pin|C1CCW_Pin|C1DOWN_Pin 
                          |C2EAST_Pin|C2WEST_Pin|C1UP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
