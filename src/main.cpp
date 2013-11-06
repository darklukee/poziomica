/**
 ******************************************************************************
 * @file    STM32F4-Discovery FreeRTOS demo\main.c
 * @author  T.O.M.A.S. Team
 * @version V1.1.0
 * @date    14-October-2011
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "aTask.hpp"
/** @addtogroup STM32F4-Discovery_Demo
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
//#define DELAY 250     /* msec */
//#define queueSIZE	6 //defined in main.h

/* Private macro -------------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Task functions declarations */
static void vLEDTask(void *pvParameters);
static void vSWITCHTask(void *pvParameters);
static void vMEMSTask(void *pvParameters);
//static void vBALANCETask(void *pvParameters);
//static void vSPEEDTask(void *pvParameters);

/* handlers to tasks to better control them */
xTaskHandle xLED_Tasks[4];
xTaskHandle xMEMS_Task, xBALANCE_Task, xSPEED_Task;

/* semaphores, queues declarations */
xSemaphoreHandle xSemaphoreSW = NULL;
xQueueHandle xQueue;

/* variables used by tasks */


/* initial arguments for vLEDTask task (which LED and what is the delay) */
static const int LEDS[4][2] =
{
		{ LED3, DELAY * 2 }, //orange
		{ LED4, DELAY * 4 }, //green
		{ LED5, DELAY * 1 }, //red
		{ LED6, DELAY * 8 } }; //blue


/**
 * @brief  Main program.
 * @param  None
 * @retval None
 */
int main(void)
{
	/* create a pipe for MEMS->TIM4 data exchange */
	xQueue = xQueueCreate(1, queueSIZE * sizeof(uint8_t));

	/* create semaphores... */
	vSemaphoreCreateBinary( xSemaphoreSW );

	/* ...and clean them up */
	if (xSemaphoreTake(xSemaphoreSW, ( portTickType ) 0) == pdTRUE)

	/* initialize hardware... */
	prvSetupHardware();

	/* Start the tasks defined within this file/specific to this demo. */
	xTaskCreate( vLEDTask, ( signed portCHAR * ) "LED3", configMINIMAL_STACK_SIZE, (void *)LEDS[0],tskIDLE_PRIORITY, &xLED_Tasks[0] );
	xTaskCreate( vLEDTask, ( signed portCHAR * ) "LED4", configMINIMAL_STACK_SIZE, (void *)LEDS[1],tskIDLE_PRIORITY, &xLED_Tasks[1] );
	xTaskCreate( vLEDTask, ( signed portCHAR * ) "LED5", configMINIMAL_STACK_SIZE, (void *)LEDS[2],tskIDLE_PRIORITY, &xLED_Tasks[2] );
	xTaskCreate( vLEDTask, ( signed portCHAR * ) "LED6", configMINIMAL_STACK_SIZE, (void *)LEDS[3],tskIDLE_PRIORITY, &xLED_Tasks[3] );
	xTaskCreate( vSWITCHTask, ( signed portCHAR * ) "SWITCH", configMINIMAL_STACK_SIZE, NULL,tskIDLE_PRIORITY, NULL );
	xTaskCreate( vMEMSTask, ( signed portCHAR * ) "MEMS", configMINIMAL_STACK_SIZE, NULL,tskIDLE_PRIORITY, &xMEMS_Task );
	xTaskCreate( vBALANCETask, ( signed portCHAR * ) "BALANCE", 4*configMINIMAL_STACK_SIZE, NULL,tskIDLE_PRIORITY, &xBALANCE_Task );
//	xTaskCreate( vSPEEDTask, ( signed portCHAR * ) "SPEED", 2*configMINIMAL_STACK_SIZE, NULL,tskIDLE_PRIORITY, &xSPEED_Task );

	vTaskSuspend(xLED_Tasks[0]);
	vTaskSuspend(xLED_Tasks[1]);
	vTaskSuspend(xLED_Tasks[2]);
	vTaskSuspend(xLED_Tasks[3]);
	prvLED_Config(TIMER);
	TIM_Cmd(TIM4, ENABLE);
	vTaskResume(xBALANCE_Task);
	//vTaskSuspend(xBALANCE_Task);
	//vTaskResume(xSPEED_Task);
	vTaskResume(xMEMS_Task);

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was not enough heap space to create the idle task. */
	return 0;
}

/*-----------------------------------------------------------*/

void vMEMSTask(void *pvParameters)
{
	/* queue for MEMS data length */
	uint8_t xBuffer_send[queueSIZE];
	uint8_t statReg;
	uint16_t flag; //data overrun count, approx 1/50kSampli
	portTickType xLastWakeTime;

	xLastWakeTime = xTaskGetTickCount();
	for (;;)
	{
		vTaskDelayUntil(&xLastWakeTime, 1 * prvTICK_MS); //co 1ms
		LIS302DL_Read(&statReg, LIS302DL_STATUS_REG_ADDR, 1);

		if ((0x08 & statReg) != 0)
		{
			LIS302DL_Read(xBuffer_send, LIS302DL_OUT_X_ADDR, queueSIZE);
			xQueueSendToBack(xQueue,xBuffer_send ,0);
		}
		if ((0x80 & statReg) != 0)
		{
			flag++;
		}

		//vTaskDelay((DELAY/portTICK_RATE_MS)/10);

		//vTaskDelay(1.6 * prvTICK_MS);
	}
}

/*-----------------------------------------------------------*/

//void vSPEEDTask(void *pvParameters)
//{
//	const uint16_t N = 256;
//	const float a = 0.7295 * TIM_ARR / 199, b = 0.02818 / (TIM_ARR / 199), c =
//			-0.7295 * TIM_ARR / 199, d = -0.02818 / (TIM_ARR / 199);
//	float accelXFloat, accelYFloat;
//	int8_t meanX[N], meanY[N];
//	int16_t meanXSigma = 0, meanYSigma = 0;
//	int16_t accelXCal = 0, accelYCal = 0;
//	const uint8_t maxValue = 51; //maksymalna wartosc wskazania przy wychyleniu 90st
//	const uint8_t cmpValue = 1; //wartosc do porownywania przyspieszenia
//	uint16_t k, j;
//	int8_t xBuffer_receive[queueSIZE];
//
//	float speedX =0, speedY=0;
//	float tempSpeedX=0, tempSpeedY=0;
//	int32_t maxSpeed=0;
//
//	//kalibracja
//	for (j = 0; j < N;)
//	{
//		if (xQueueReceive(xQueue,xBuffer_receive,0) == pdPASS)
//		{
//			meanX[j] = xBuffer_receive[0];
//			meanY[j] = xBuffer_receive[2];
//			j++;
//		}
//	}
//
//	meanXSigma = 0;
//	meanYSigma = 0;
//	for (k = 0; k < N; k++)
//	{
//		meanXSigma += meanX[k];
//		meanYSigma += meanY[k];
//	}
//
//	//dopasowanie do sterowania PWM
//	//accelXCal = (int16_t) ((TIM_ARR * meanXSigma) / (N * maxValue));
//	//accelYCal = (int16_t) ((TIM_ARR * meanYSigma) / (N * maxValue));
//
//	//ze skalowaniem nieliniowosci diody: f(x) = a*expf(b*x) + c*expf(d*x)
//	accelXFloat = (float) ((TIM_ARR * meanXSigma) / (N * maxValue)); //wartosc srednia z N probek
//	accelXFloat = a * expf(b * accelXFloat) + c * expf(d * accelXFloat);
//	if (accelXFloat > 0)
//	{
//		accelXCal = (int16_t) ceilf(accelXFloat);
//	}
//	else if (accelXFloat < 0)
//	{
//		accelXCal = (int16_t) floorf(accelXFloat);
//	}
//	else
//	{
//		accelXCal = 0;
//	}
//
//	accelYFloat = (float) ((TIM_ARR * meanYSigma) / (N * maxValue));
//	accelYFloat = a * expf(b * accelYFloat) + c * expf(d * accelYFloat);
//	if (accelYFloat > 0)
//	{
//		accelYCal = (int16_t) ceilf(accelYFloat);
//	}
//	else if (accelYFloat < 0)
//	{
//		accelYCal = (int16_t) floorf(accelYFloat);
//	}
//	else
//	{
//		accelYCal = 0;
//	}
//
//	//petla glowna
//
//	for (;;)
//	{
//		if (xQueueReceive(xQueue,xBuffer_receive,0) == pdPASS)
//		{
//			tempSpeedX = (xBuffer_receive[0]-accelXCal)/10;
//			tempSpeedY = (xBuffer_receive[2]-accelYCal)/10;
//
//			speedX += (fabs(tempSpeedX)>0)?(tempSpeedX):0;
//			speedY += (fabs(tempSpeedY)>0)?(tempSpeedY):0;
//
//			maxSpeed = (fabs(maxSpeed)>=(fabs(speedX)<fabs(speedY))?fabs(speedY):fabs(speedX)?maxSpeed:((fabs(speedX)<fabs(speedY))?speedY:speedX));
//
//			//os X
//						if (speedX < -cmpValue)
//						{
//							/* Enable TIM4 Capture Compare Channel 4 */
//							TIM_CCxCmd(TIM4, TIM_Channel_2, DISABLE);
//							TIM_CCxCmd(TIM4, TIM_Channel_4, ENABLE);
//							/* Sets the TIM4 Capture Compare4 Register value */
//							TIM_SetCompare4(TIM4, (uint32_t)  (TIM_CCR + (speedX * (-1))));
//						}
//						else if (speedX > cmpValue)
//						{
//							/* Enable TIM4 Capture Compare Channel 2 */
//							TIM_CCxCmd(TIM4, TIM_Channel_4, DISABLE);
//							TIM_CCxCmd(TIM4, TIM_Channel_2, ENABLE);
//							/* Sets the TIM4 Capture Compare2 Register value */
//							TIM_SetCompare2(TIM4, (uint32_t) (TIM_CCR + (speedX)));
//						}
//						else
//						{
//							TIM_CCxCmd(TIM4, TIM_Channel_4, DISABLE);
//							TIM_CCxCmd(TIM4, TIM_Channel_2, DISABLE);
//						}
//
//						//os Y
//						if (speedY > cmpValue)
//						{
//							/* Enable TIM4 Capture Compare Channel 1 */
//							TIM_CCxCmd(TIM4, TIM_Channel_3, DISABLE);
//							TIM_CCxCmd(TIM4, TIM_Channel_1, ENABLE);
//							/* Sets the TIM4 Capture Compare1 Register value */
//							TIM_SetCompare1(TIM4 , (uint32_t) (TIM_CCR + (speedY)));
//						}
//						else if (speedY < -cmpValue)
//						{
//							/* Enable TIM4 Capture Compare Channel 3 */
//							TIM_CCxCmd(TIM4, TIM_Channel_1, DISABLE);
//							TIM_CCxCmd(TIM4, TIM_Channel_3, ENABLE);
//							/* Sets the TIM4 Capture Compare3 Register value */
//							TIM_SetCompare3(TIM4,(uint32_t)   (TIM_CCR + (speedY * (-1))));
//						}
//						else
//						{
//							TIM_CCxCmd(TIM4, TIM_Channel_1, DISABLE);
//							TIM_CCxCmd(TIM4, TIM_Channel_3, DISABLE);
//						}
//		}
//		taskYIELD();
//	}
//}

//void vBALANCETask(void *pvParameters)
//{
//	const uint16_t N = 32;
//	const float a = 0.7295 * TIM_ARR / 199, b = 0.02818 / (TIM_ARR / 199), c =
//			-0.7295 * TIM_ARR / 199, d = -0.02818 / (TIM_ARR / 199);
//	float accelXFloat, accelYFloat;
//	int8_t meanX[N], meanY[N];
//	int16_t meanXSigma = 0, meanYSigma = 0, accelX, accelY;
//	int16_t accelXCal = 0, accelYCal = 0;
//	const uint8_t maxValue = 51; //maksymalna wartosc wskazania przy wychyleniu 90st
//	const uint8_t cmpValue = 1; //wartosc do porownywania przyspieszenia
//	uint16_t k, j;
//	int8_t xBuffer_receive[queueSIZE];
//
//	//kalibracja, przydaloby sie zrobic w osobnej funkcji :D
//	for (j = 0; j < N;)
//	{
//		if (xQueueReceive(xQueue,xBuffer_receive,0) == pdPASS)
//		{
//			meanX[j] = xBuffer_receive[0];
//			meanY[j] = xBuffer_receive[2];
//			j++;
//		}
//	}
//
//	meanXSigma = 0;
//	meanYSigma = 0;
//	for (k = 0; k < N; k++)
//	{
//		meanXSigma += meanX[k];
//		meanYSigma += meanY[k];
//	}
//
//	//dopasowanie do sterowania PWM
//	//accelXCal = (int16_t) ((TIM_ARR * meanXSigma) / (N * maxValue));
//	//accelYCal = (int16_t) ((TIM_ARR * meanYSigma) / (N * maxValue));
//
//	//ze skalowaniem nieliniowosci diody: f(x) = a*expf(b*x) + c*expf(d*x)
//	accelXFloat = (float) ((TIM_ARR * meanXSigma) / (N * maxValue)); //wartosc srednia z N probek
//	accelXFloat = a * expf(b * accelXFloat) + c * expf(d * accelXFloat);
//	if (accelXFloat > 0)
//	{
//		accelXCal = (int16_t) ceilf(accelXFloat);
//	}
//	else if (accelXFloat < 0)
//	{
//		accelXCal = (int16_t) floorf(accelXFloat);
//	}
//	else
//	{
//		accelXCal = 0;
//	}
//
//	accelYFloat = (float) ((TIM_ARR * meanYSigma) / (N * maxValue));
//	accelYFloat = a * expf(b * accelYFloat) + c * expf(d * accelYFloat);
//	if (accelYFloat > 0)
//	{
//		accelYCal = (int16_t) ceilf(accelYFloat);
//	}
//	else if (accelYFloat < 0)
//	{
//		accelYCal = (int16_t) floorf(accelYFloat);
//	}
//	else
//	{
//		accelYCal = 0;
//	}
//
//	for (;;)
//	{
//		if (xQueueReceive(xQueue,xBuffer_receive,0) == pdPASS)
//		{
//			//bufor cykliczny
//			j %= N;
//			meanX[j] = xBuffer_receive[0];
//			meanY[j] = xBuffer_receive[2];
//			j++;
//			//obliczanie wartosci sreniej
//			meanXSigma = 0;
//			meanYSigma = 0;
//			for (k = 0; k < N; k++)
//			{
//				meanXSigma += meanX[k];
//				meanYSigma += meanY[k];
//			}
//
//			//dopasowanie do sterowania PWM
//			//accelX = (int16_t) ((TIM_ARR * meanXSigma) / (N * maxValue));
//			//accelY = (int16_t) ((TIM_ARR * meanYSigma) / (N * maxValue));
//
//			//ze skalowaniem nieliniowosci diody: f(x) = a*expf(b*x) + c*expf(d*x)
//			accelXFloat = (float) ((TIM_ARR * meanXSigma) / (N * maxValue)); //wartosc srednia z N probek
//			accelXFloat = a * expf(b * accelXFloat) + c * expf(d * accelXFloat);
//			if (accelXFloat > 0)
//			{
//				accelX = (int16_t) ceilf(accelXFloat);
//			}
//			else if (accelXFloat < 0)
//			{
//				accelX = (int16_t) floorf(accelXFloat);
//			}
//			else
//			{
//				accelX = 0;
//			}
//
//			accelYFloat = (float) ((TIM_ARR * meanYSigma) / (N * maxValue));
//			accelYFloat = a * expf(b * accelYFloat) + c * expf(d * accelYFloat);
//			if (accelYFloat > 0)
//			{
//				accelY = (int16_t) ceilf(accelYFloat);
//			}
//			else if (accelYFloat < 0)
//			{
//				accelY = (int16_t) floorf(accelYFloat);
//			}
//			else
//			{
//				accelY = 0;
//			}
//
//			//uwzgledniajac kalbiracje
//			accelX -= accelXCal;
//			accelY -= accelYCal;
//
//			//os X
//			if (accelX < -cmpValue)
//			{
//				/* Enable TIM4 Capture Compare Channel 4 */
//				TIM_CCxCmd(TIM4, TIM_Channel_2, DISABLE);
//				TIM_CCxCmd(TIM4, TIM_Channel_4, ENABLE);
//				/* Sets the TIM4 Capture Compare4 Register value */
//				TIM_SetCompare4(TIM4, TIM_CCR + (accelX * (-1)));
//
//			}
//			else if (accelX > cmpValue)
//			{
//				/* Enable TIM4 Capture Compare Channel 2 */
//				TIM_CCxCmd(TIM4, TIM_Channel_4, DISABLE);
//				TIM_CCxCmd(TIM4, TIM_Channel_2, ENABLE);
//				/* Sets the TIM4 Capture Compare2 Register value */
//				TIM_SetCompare2(TIM4, TIM_CCR + (accelX));
//			}
//			else
//			{
//				TIM_CCxCmd(TIM4, TIM_Channel_4, DISABLE);
//				TIM_CCxCmd(TIM4, TIM_Channel_2, DISABLE);
//			}
//
//			//os Y
//			if (accelY > cmpValue)
//			{
//				/* Enable TIM4 Capture Compare Channel 1 */
//				TIM_CCxCmd(TIM4, TIM_Channel_3, DISABLE);
//				TIM_CCxCmd(TIM4, TIM_Channel_1, ENABLE);
//				/* Sets the TIM4 Capture Compare1 Register value */
//				TIM_SetCompare1(TIM4, TIM_CCR + (accelY));
//			}
//			else if (accelY < -cmpValue)
//			{
//				/* Enable TIM4 Capture Compare Channel 3 */
//				TIM_CCxCmd(TIM4, TIM_Channel_1, DISABLE);
//				TIM_CCxCmd(TIM4, TIM_Channel_3, ENABLE);
//				/* Sets the TIM4 Capture Compare3 Register value */
//				TIM_SetCompare3(TIM4, TIM_CCR + (accelY * (-1)));
//			}
//			else
//			{
//				TIM_CCxCmd(TIM4, TIM_Channel_1, DISABLE);
//				TIM_CCxCmd(TIM4, TIM_Channel_3, DISABLE);
//			}
//			//TIM_GenerateEvent(TIM4, TIM_EventSource_Update);
//			/* Time base configuration */
//			//TIM_SetAutoreload(TIM4,  TIM_ARR/TempAcceleration);
//
//		}
//		taskYIELD(); //task is going to ready state to allow next one to run
//	}
//}

/*-----------------------------------------------------------*/

void vLEDTask(void *pvParameters)
{
	volatile int *LED;
	LED = (int *) pvParameters;

	for (;;)
	{
		STM_EVAL_LEDToggle((Led_TypeDef) LED[0]);
		vTaskDelay(LED[1] * prvTICK_MS);
	}
}

/*-----------------------------------------------------------*/

void vSWITCHTask(void *pvParameters)
{
	static int i = 1;
	for (;;)
	{
		if (xSemaphoreTake(xSemaphoreSW,( portTickType ) 0) == pdTRUE)
		{
			i ^= 1; //just switch the state if semaphore was given

			if (i == 0) //LED3..LD6 tasks ready, BALANCE, MEMS suspended
			{
				vTaskSuspend(xBALANCE_Task);
				TIM_Cmd(TIM4, DISABLE);
				vTaskSuspend(xMEMS_Task);
				prvLED_Config(GPIO);
				vTaskResume(xLED_Tasks[0]);
				vTaskResume(xLED_Tasks[1]);
				vTaskResume(xLED_Tasks[2]);
				vTaskResume(xLED_Tasks[3]);
			}
			else //MEMS and BALANCE ready, LED tasks suspended
			{
				vTaskSuspend(xLED_Tasks[0]);
				vTaskSuspend(xLED_Tasks[1]);
				vTaskSuspend(xLED_Tasks[2]);
				vTaskSuspend(xLED_Tasks[3]);
				prvLED_Config(TIMER);
				TIM_Cmd(TIM4, ENABLE);
				vTaskResume(xBALANCE_Task);
				vTaskResume(xMEMS_Task);
			}
		}
		taskYIELD(); //task is going to ready state to allow next one to run
	}
}

/*-----------------------------------------------------------*/



#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *   where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}
#endif

/**
 * @}
 */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
