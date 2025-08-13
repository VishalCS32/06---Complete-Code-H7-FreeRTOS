/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : freertos.c
  * @brief          : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "../ICM42688P/icm42688p.h"
#include "semphr.h"
#include "../LED/MAIN_BOARD_RGB/ws2812.h"
#include "../LED/AIRCRAFTLIGHTS/AircraftLights.h"
#include "../CMD/cmd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
SemaphoreHandle_t spiDmaSem;
SemaphoreHandle_t ws2812_dma_semaphore;
SemaphoreHandle_t aircraftlights_dma_semaphore;

/* The semaphore given by the EXTI ISR when the ICM42688P DRDY line pulses.
   Defined in icm42688p.c as: SemaphoreHandle_t icmIntSem = NULL;
   We declare it extern here so we can create it in MX_FREERTOS_Init(). */

void run_imu(void);
extern void run_imu(void);
extern void run_mag(void);
extern void sensor_init(void);
extern void eeprom_startup(void);
void InitTask(void *argument);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* --- Task3 (IMU Read Task) --- */
/* Increased stack size for safety (printf/HAL calls) */
osThreadId_t Task_1khzHandle;
const osThreadAttr_t Task_1khz_attributes = {
  .name = "Task_1khz",
  .stack_size = 512 * 4,                /* increased from 256*4 */
  .priority = (osPriority_t) osPriorityAboveNormal,
};

osThreadId_t Task_100HzHandle;
const osThreadAttr_t Task_100Hz_attributes = {
  .name = "Task_100Hz",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal5,
};

osThreadId_t Task_50HzHandle;
const osThreadAttr_t Task_50Hz_attributes = {
  .name = "Task_50Hz",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal4,
};

/* --- InitTask (Initialization) --- */
osThreadId_t InitTaskHandle;
const osThreadAttr_t InitTask_attributes = {
  .name = "InitTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* --- LedTask (WS2812 LED Task) --- */
osThreadId_t LedTaskHandle;
const osThreadAttr_t LedTask_attributes = {
  .name = "LedTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* --- AircraftLights Task --- */
osThreadId_t AircraftLightsTaskHandle;
const osThreadAttr_t AircraftLightsTask_attributes = {
  .name = "AircraftLightsTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task2 */
osThreadId_t Task2Handle;
const osThreadAttr_t Task2_attributes = {
  .name = "Task2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void Task_1khz_init(void *argument);
void Task_100Hz_init(void *argument);
void Task_50Hz_init(void *argument);
void LedTask(void *argument);
void AircraftLightsTask(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Task2_init(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* Create InitTask first (it's fine to create before semaphores as we create semaphores
     below before the scheduler starts). */
  InitTaskHandle = osThreadNew(InitTask, NULL, &InitTask_attributes);
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* Add mutexes if required */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */

  ws2812_dma_semaphore = xSemaphoreCreateBinary();
  if (ws2812_dma_semaphore == NULL) {
      Error_Handler();
  }

  aircraftlights_dma_semaphore = xSemaphoreCreateBinary();
  if (aircraftlights_dma_semaphore == NULL) {
      Error_Handler();
  }

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* Start timers, add new ones if required */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* Add queues if required */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Task2 */
  Task2Handle = osThreadNew(Task2_init, NULL, &Task2_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  Task_1khzHandle = osThreadNew(Task_1khz_init, NULL, &Task_1khz_attributes);

  Task_100HzHandle = osThreadNew(Task_100Hz_init, NULL, &Task_100Hz_attributes);

  Task_50HzHandle = osThreadNew(Task_50Hz_init, NULL, &Task_50Hz_attributes);

  LedTaskHandle = osThreadNew(LedTask, NULL, &LedTask_attributes);
  osThreadSuspend(LedTaskHandle);

  AircraftLightsTaskHandle = osThreadNew(AircraftLightsTask, NULL, &AircraftLightsTask_attributes);
  osThreadSuspend(AircraftLightsTaskHandle);  // Start suspended, resume after init

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* Add event groups if required */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  for(;;) {
//    HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3);
    osDelay(800);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Task2_init */
/**
  * @brief  Function implementing the Task2 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Task2_init */
void Task2_init(void *argument)
{
  /* USER CODE BEGIN Task2_init */
  for(;;) {
    HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_5);
    osDelay(500);
  }
  /* USER CODE END Task2_init */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
  * @brief Task3: IMU data acquisition (interrupt-driven)
  *
  * This task blocks on icmIntSem which is given from the EXTI ISR when the
  * ICM42688P's DRDY/INT pin pulses. After waking it calls fusion() which
  * performs the read/process (unchanged behavior).
  */
void Task_1khz_init(void *argument)
{
	for(;;) {

		run_imu();
		osDelay(1);

    }
}

void Task_100Hz_init(void *argument)
{
	for(;;) {

//		run_mag();
//		FSiA6B_Print();
		osDelay(100);

	}
}

void Task_50Hz_init(void *argument)
{
	for(;;) {
//		printf("Hello, World\n");

		osDelay(20);
	}
}

void Task_1000Hz_init(void *argument)
{
	for(;;) {
		FailSafe_1000Hz();
		osDelay(1000);
	}
}
/**
  * @brief InitTask: Initializes sensors & WS2812
  */
void InitTask(void *argument)
{
    printf("Starting sensor initialization...\r\n");

    /* Suspend IMU loop while we init hardware */
    osThreadSuspend(Task_1khzHandle);
    osDelay(200);

    /* Initialize sensors (this will call ICM42688P_Init, which enables DRDY on the IMU) */
    sensor_init();
    printf("Sensor initialization complete.\r\n");

    /* Initialize WS2812 and AircraftLights using same timer (htim3) */
    extern TIM_HandleTypeDef htim3;
    WS2812_Init(&htim3);
    printf("WS2812 LED Driver Initialized.\r\n");

    AIRCRAFTLIGHTS_Init(&htim3);
    printf("AircraftLights Driver Initialized.\r\n");

    cmd_mode_check();

    /* Resume tasks that were suspended during init */
    osThreadResume(Task_1khzHandle);
    osThreadResume(LedTaskHandle);
    osThreadResume(AircraftLightsTaskHandle);

    printf("InitTask complete. Deleting InitTask...\r\n");
    osThreadTerminate(InitTaskHandle);
}

/**
  * @brief LedTask: Controls WS2812 LED
  */
void LedTask(void *argument)
{
    for(;;) {

    	main_led(0, 0, 0, 0, 1.0);
    	main_led_update();

    	vTaskDelay(pdMS_TO_TICKS(500));

    	main_led(0, 255, 0, 0, 1.0);
    	main_led_update();

    	vTaskDelay(pdMS_TO_TICKS(30));

    }
}

/**
  * @brief AircraftLightsTask: Controls Aircraft Lights
  */
void AircraftLightsTask(void *argument)
{
    for(;;) {

    	aircraftlights(0, 255, 0, 0, 1.0);  // Red
    	aircraftlights(1, 0, 255, 0, 1.0);  // Green
    	aircraftlights(2, 0, 255, 0, 1.0);  // Green
    	aircraftlights(3, 255, 0, 0, 1.0);  // Red
    	aircraftlights_update();

    	vTaskDelay(pdMS_TO_TICKS(2000));

    	aircraftlights(0, 255, 255, 255, 1.0);
    	aircraftlights(3, 255, 255, 255, 1.0);
    	aircraftlights_update();

    	vTaskDelay(pdMS_TO_TICKS(35));

    	aircraftlights(0, 255, 0, 0, 1.0);
    	aircraftlights(3, 255, 0, 0, 1.0);
    	aircraftlights_update();

    	vTaskDelay(pdMS_TO_TICKS(35));

    	aircraftlights(0, 255, 255, 255, 1.0);
    	aircraftlights(3, 255, 255, 255, 1.0);
    	aircraftlights_update();

    	vTaskDelay(pdMS_TO_TICKS(35));

    	aircraftlights(0, 255, 0, 0, 1.0);
    	aircraftlights(3, 255, 0, 0, 1.0);
    	aircraftlights_update();

    	vTaskDelay(pdMS_TO_TICKS(35));

    }
}


/* USER CODE END Application */

