/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "FreeRTOSConfig.h"

#include "FreeRTOS.h"

#include "portmacro.h"
#include "projdefs.h"

// used components from freertos
#include "queue.h"
#include "task.h"

#include <cstdint>
#include <cstring>
#include <stdio.h>

#include <functional>
#include <string>

// application includes
#include "menu.hpp"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
template <bool USE_ISR = false>
class IQueueOutBuff : public OutBuffer<IQueueOutBuff<USE_ISR>> {
public:
  IQueueOutBuff() = default;
  IQueueOutBuff(QueueHandle_t q) : pQueueHandle(q) { pMessage = &message; }
  void do_write(const std::string &text) {
    message = text;
    pMessage = &message;
    if constexpr (USE_ISR) {
      xQueueSendFromISR(pQueueHandle, &pMessage, NULL);
    } else {
      xQueueSend(pQueueHandle, &pMessage, pdMS_TO_TICKS(1000));
    }
  }

private:
  QueueHandle_t pQueueHandle = NULL;
  std::string message;
  std::string *pMessage;
};
class QueueOutBuffISR : public IQueueOutBuff<true> {};

class QueueOutBuff : public IQueueOutBuff<false> {};

class UartOutBuff : public OutBuffer<UartOutBuff> {
public:
  UartOutBuff(UART_HandleTypeDef &uart_handler) : mUart_handler(uart_handler) {}
  void do_write(const std::string &text) {
    auto message = reinterpret_cast<const uint8_t *>(text.c_str());
    HAL_UART_Transmit(&mUart_handler, message, text.size(), 1000);
  }

private:
  UART_HandleTypeDef &mUart_handler;
};

class MenuState {
public:
  using FnHandler = std::function<MenuState *(void)>;

  MenuState *execute(std::string_view command) {
    if (mStateMap.count(command)) {
      return mStateMap[command]();
    }
    return mInvalidCommandHandler();
  }

  void register_command(std::string_view command, FnHandler handler) {
    mStateMap[command] = handler;
  }

  MenuState() = default;
  MenuState(FnHandler invalid_command_handler)
      : mInvalidCommandHandler(invalid_command_handler) {}

private:
  std::unordered_map<std::string_view, FnHandler> mStateMap;
  FnHandler mInvalidCommandHandler = []() { return this; };
};

using QMenu = Menu<QueueOutBuff>;
using QMenuISR = Menu<QueueOutBuffISR>;

class CliApp {
public:
  void try_command(std::string_view command) {
    current_menu = current_menu->execute(command);
  }

  CliApp() {}

private:
  QMenu *current_menu = nullptr;
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
static void menu_task(void *args);
static void state_machine_task(void *args);
static void command_processing_task(void *args);

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
QueueHandle_t uart_out_queue;
QueueHandle_t uart_in_queue;
xTaskHandle cmd_task_handler;
uint8_t recv_data;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code
   ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */
  BaseType_t status;
  xTaskHandle menu_task_handler;
  xTaskHandle state_manchine_task_handler;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART6_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  // LEDsMenuUI menuUI{};
  //

  /* QUEUES */

  uart_out_queue = xQueueCreate(1024, sizeof(std::string *));
  uart_in_queue = xQueueCreate(1024, sizeof(char));
  status =
      xTaskCreate(menu_task, "menu_task", 200, NULL, 2, &menu_task_handler);
  configASSERT(status == pdPASS);

  status = xTaskCreate(state_machine_task, "state_machine_task", 200, NULL, 3,
                       &state_manchine_task_handler);
  configASSERT(status == pdPASS);

  status = xTaskCreate(command_processing_task, "state_machine_task", 200, NULL,
                       3, &cmd_task_handler);
  configASSERT(status == pdPASS);

  HAL_UART_Receive_IT(&huart6, &recv_data, 1);

  vTaskStartScheduler();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType =
      RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void) {

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
   */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_12;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */
}

/**
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_UART_Init(void) {

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED2_Pin | LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN1_Pin */
  GPIO_InitStruct.Pin = BTN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED2_Pin | LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void menu_task(void *args) {
  std::string *pMessage;
  UartOutBuff uart6Buff{huart6};
  while (1) {
    if (xQueueReceive(uart_out_queue, &pMessage, pdMS_TO_TICKS(1000)) ==
        pdPASS) {
      uart6Buff.write(*pMessage);
    }
  }
}

void state_machine_task(void *args) {
  // auto menu_1 = reinterpret_cast<QMenu *>(args);
  while (1) {
    // menu_1->show();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void command_processing_task(void *args) {
  char buff[32];
  while (1) {
    memset(buff, 0, sizeof(buff));
    xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);

    uint8_t data;
    uint8_t *pBuff = buff;
    while (xQueueReceive(uart_in_queue, &data, portMAX_DELAY) !=
           errQUEUE_EMPTY) {
      *pBuff = data;
      pBuff++;
    }

    std::string_view command{buff};
    cliApp->try_command(command);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  // QMenuISR menu_1{QueueOutBuffISR{uart_out_queue}};
  // menu_1.show();
  QueueOutBuffISR buffer{uart_in_queue};
  buffer.write("Hello from write function\r\n");
  if ('\n' == recv_data) {
    xTaskNotifyFromISR(cmd_task_handler, 0, eNoAction, NULL);
  } else {
    if (!xQueueIsQueueFullFromISR(uart_in_queue)) {
      xQueueSendFromISR(uart_in_queue, &recv_data, NULL);
    }
  }

  HAL_UART_Receive_IT(&huart6, &recv_data, 1);
}

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM3 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state
   */
  __disable_irq();
  while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
     file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
