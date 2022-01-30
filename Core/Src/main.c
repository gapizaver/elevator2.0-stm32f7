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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f769i_discovery.h"
#include "stm32f769i_discovery_lcd.h"
#include "stm32f769i_discovery_ts.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define NUM_BUTTONS 7
#define BUTTON_TEXT_MAX_LENGTH 30
#define BACKGROUND_COLOR (uint32_t) 0xFF050A30

  // struktura gumb
typedef struct {
	uint16_t x;
	uint16_t y;
	uint16_t w;
	uint16_t h;
	uint8_t text[BUTTON_TEXT_MAX_LENGTH];
	int text_x_diff;			// razlika začetka teksta in centra gumba
} Button;

// array gumbov
Button buttons[NUM_BUTTONS] = {
	// prva vrsta
	{20, 43, 175, 175, "P", -5},
	{215, 43, 175, 175, "1", -5},
	{410, 43, 175, 175, "2", -5},
	{605, 43, 175, 175, "3", -5},
	// druga vrsta
	{20, 262, 175, 175, "Odpri", -45},
	{215, 262, 175, 175, "Zapri", -40},
	{410, 262, 175, 175, "Alarm", -40}
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* Definitions for tasks */
osThreadId_t LCDInputTaskHandle;
const osThreadAttr_t LCDInputTask_attributes = {
  .name = "LCDInputTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* USER CODE BEGIN PV */
osThreadId_t LCDDrawTaskHandle;
const osThreadAttr_t LCDDrawTask_attributes = {
  .name = "LCDDrawTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

uint8_t  lcd_status = LCD_OK;
uint32_t ts_status = TS_OK;
TS_StateTypeDef  TS_State = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
void LCDInputTask(void *argument);		// task za detekcijo pritiska na zaslon
void LCDDrawTask(void *argument);		// task za izris elementov na zaslon
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  /* Initialize LEDs */
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED2);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  // inicializacija LCD zaslona:
  BSP_LCD_Init();

  BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);

  ts_status = BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
  while(ts_status != TS_OK);

  ts_status = BSP_TS_ITConfig();
  while(ts_status != TS_OK);

  /*
  uint8_t strptr[] = "Vesel bozic in srecno 2022!";
  BSP_LCD_SetFont(&Font24);
  BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
  BSP_LCD_DisplayStringAtLine(7, strptr);

  BSP_LCD_SetTextColor(LCD_COLOR_DARKRED);
  */
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  LCDInputTaskHandle = osThreadNew(LCDInputTask, NULL, &LCDInputTask_attributes);
  LCDDrawTaskHandle = osThreadNew(LCDDrawTask, NULL, &LCDDrawTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef  ret = HAL_OK;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
	 clocked below the maximum system frequency, to update the voltage scaling value
	 regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 7;

  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
	while(1) { ; }
  }

  /* Activate the OverDrive to reach the 216 MHz Frequency */
  ret = HAL_PWREx_EnableOverDrive();
  if(ret != HAL_OK)
  {
	while(1) { ; }
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);
  if(ret != HAL_OK)
  {
	while(1) { ; }
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
  /* USER CODE BEGIN 5 */
  /* TASKS */
void LCDInputTask(void *argument) {
  for(;;)
  {
	BSP_TS_GetState(&TS_State);
	/*
	if(TS_State.touchDetected > 0) {
		if (TS_State.touchY[0] > 30)
		BSP_LCD_DrawCircle(TS_State.touchX[0], TS_State.touchY[0], 30);
	}
	*/
	osDelay(20);
  }
}

void LCDDrawTask(void *argument) {
	for (;;) {
		// pobris ekrana
		//BSP_LCD_Clear(BACKGROUND_COLOR);

		// izris gumbov
		BSP_LCD_SetFont(&Font24);
		BSP_LCD_SetBackColor(LCD_COLOR_DARKBLUE);

		for (size_t i = 0; i < NUM_BUTTONS; i++) {
			// izris kvadrata
			BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);			// barva kvadrata
			BSP_LCD_FillRect(buttons[i].x, buttons[i].y, buttons[i].w, buttons[i].h);

			// prikaži tekst gumba v sredini gumba
			BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);		// text inside button color
			BSP_LCD_DisplayStringAt(
				  buttons[i].x + buttons[i].w/2 + buttons[i].text_x_diff,
				  buttons[i].y + buttons[i].h/2 - 7,
				  buttons[i].text,
				  LEFT_MODE
			);
		}

		BSP_LED_Toggle(LED1);

		osDelay(1000);
	}
}
  /* USER CODE END 5 */

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

