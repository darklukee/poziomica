/*
 * myTask.cpp
 *
 *  Created on: 10-11-2012
 *      Author: lukee
 */

/* Includes ------------------------------------------------------------------*/
#include "aTask.hpp"

const float CBalance::a = 0.7295 * TIM_ARR / 199;
const float CBalance::b = 0.02818 / (TIM_ARR / 199);
const float CBalance::c = -0.7295 * TIM_ARR / 199;
const float CBalance::d = -0.02818 / (TIM_ARR / 199);
const uint8_t CBalance::maxValue = 51; //maksymalna wartosc wskazania przy wychyleniu 90st
const uint8_t CBalance::cmpValue = 1; //wartosc do porownywania przyspieszenia

int CBalance::temp = 55;

/* semaphores, queues declarations */
extern xQueueHandle xQueue;

void vBALANCETask(void *pvParameters)
{
	CBalance Balance;
	Balance.Calibrate();
	while (1)
	{
		if (xQueueReceive(xQueue,Balance.xBuffer_receive,0) == pdPASS)
		{
			Balance.Calculate();
			Balance.SetData();
		}
		taskYIELD(); //task is going to ready state to allow next one to run
	}

}

CBalance::CBalance()
{
	//constants
	meanXSigma = 0;
	meanYSigma = 0;
	accelXCal = 0;
	accelYCal = 0;
}
//----------------------------------------------------------------------------

void CBalance::Calibrate(void)
{
	temp++;
	for (uint16_t i = 0; i < N;)
	{
		if (xQueueReceive(xQueue,xBuffer_receive,0) == pdPASS)
		{
			meanX[i] = xBuffer_receive[0];
			meanY[i] = xBuffer_receive[2];
			i++;
		}
	}

	meanXSigma = 0;
	meanYSigma = 0;
	for (uint16_t i = 0; i < N; i++)
	{
		meanXSigma += meanX[i];
		meanYSigma += meanY[i];
	}

	//dopasowanie do sterowania PWM
	//accelXCal = (int16_t) ((TIM_ARR * meanXSigma) / (N * maxValue));
	//accelYCal = (int16_t) ((TIM_ARR * meanYSigma) / (N * maxValue));

	//ze skalowaniem nieliniowosci diody: f(x) = a*expf(b*x) + c*expf(d*x)
	accelXFloat = (float) ((TIM_ARR * meanXSigma) / (N * maxValue)); //wartosc srednia z N probek
	accelXFloat = a * expf(b * accelXFloat) + c * expf(d * accelXFloat);
	if (accelXFloat > 0)
	{
		accelXCal = (int16_t) ceilf(accelXFloat);
	}
	else if (accelXFloat < 0)
	{
		accelXCal = (int16_t) floorf(accelXFloat);
	}
	else
	{
		accelXCal = 0;
	}

	accelYFloat = (float) ((TIM_ARR * meanYSigma) / (N * maxValue));
	accelYFloat = a * expf(b * accelYFloat) + c * expf(d * accelYFloat);
	if (accelYFloat > 0)
	{
		accelYCal = (int16_t) ceilf(accelYFloat);
	}
	else if (accelYFloat < 0)
	{
		accelYCal = (int16_t) floorf(accelYFloat);
	}
	else
	{
		accelYCal = 0;
	}
}
//----------------------------------------------------------------------------

void CBalance::Calculate(void)
{
	//bufor cykliczny
	j %= N;
	meanX[j] = xBuffer_receive[0];
	meanY[j] = xBuffer_receive[2];
	j++;
	//obliczanie wartosci sreniej
	meanXSigma = 0;
	meanYSigma = 0;
	for (k = 0; k < N; k++)
	{
		meanXSigma += meanX[k];
		meanYSigma += meanY[k];
	}

	//dopasowanie do sterowania PWM
	//accelX = (int16_t) ((TIM_ARR * meanXSigma) / (N * maxValue));
	//accelY = (int16_t) ((TIM_ARR * meanYSigma) / (N * maxValue));

	//ze skalowaniem nieliniowosci diody: f(x) = a*expf(b*x) + c*expf(d*x)
	accelXFloat = (float) ((TIM_ARR * meanXSigma) / (N * maxValue)); //wartosc srednia z N probek
	accelXFloat = a * expf(b * accelXFloat) + c * expf(d * accelXFloat);
	if (accelXFloat > 0)
	{
		accelX = (int16_t) ceilf(accelXFloat);
	}
	else if (accelXFloat < 0)
	{
		accelX = (int16_t) floorf(accelXFloat);
	}
	else
	{
		accelX = 0;
	}

	accelYFloat = (float) ((TIM_ARR * meanYSigma) / (N * maxValue));
	accelYFloat = a * expf(b * accelYFloat) + c * expf(d * accelYFloat);
	if (accelYFloat > 0)
	{
		accelY = (int16_t) ceilf(accelYFloat);
	}
	else if (accelYFloat < 0)
	{
		accelY = (int16_t) floorf(accelYFloat);
	}
	else
	{
		accelY = 0;
	}

	//Uwzgl�dniaj�c kalibracje
	accelX -= accelXCal;
	accelY -= accelYCal;
}
//----------------------------------------------------------------------------

void CBalance::SetData(void)
{
	//os X
	if (accelX < -cmpValue)
	{
		/* Enable TIM4 Capture Compare Channel 4 */
		TIM_CCxCmd(TIM4, TIM_Channel_2, DISABLE);
		TIM_CCxCmd(TIM4, TIM_Channel_4, ENABLE);
		/* Sets the TIM4 Capture Compare4 Register value */
		TIM_SetCompare4(TIM4, TIM_CCR + (accelX * (-1)));

	}
	else if (accelX > cmpValue)
	{
		/* Enable TIM4 Capture Compare Channel 2 */
		TIM_CCxCmd(TIM4, TIM_Channel_4, DISABLE);
		TIM_CCxCmd(TIM4, TIM_Channel_2, ENABLE);
		/* Sets the TIM4 Capture Compare2 Register value */
		TIM_SetCompare2(TIM4, TIM_CCR + (accelX));
	}
	else
	{
		TIM_CCxCmd(TIM4, TIM_Channel_4, DISABLE);
		TIM_CCxCmd(TIM4, TIM_Channel_2, DISABLE);
	}

	//os Y
	if (accelY > cmpValue)
	{
		/* Enable TIM4 Capture Compare Channel 1 */
		TIM_CCxCmd(TIM4, TIM_Channel_3, DISABLE);
		TIM_CCxCmd(TIM4, TIM_Channel_1, ENABLE);
		/* Sets the TIM4 Capture Compare1 Register value */
		TIM_SetCompare1(TIM4, TIM_CCR + (accelY));
	}
	else if (accelY < -cmpValue)
	{
		/* Enable TIM4 Capture Compare Channel 3 */
		TIM_CCxCmd(TIM4, TIM_Channel_1, DISABLE);
		TIM_CCxCmd(TIM4, TIM_Channel_3, ENABLE);
		/* Sets the TIM4 Capture Compare3 Register value */
		TIM_SetCompare3(TIM4, TIM_CCR + (accelY * (-1)));
	}
	else
	{
		TIM_CCxCmd(TIM4, TIM_Channel_1, DISABLE);
		TIM_CCxCmd(TIM4, TIM_Channel_3, DISABLE);
	}
	//TIM_GenerateEvent(TIM4, TIM_EventSource_Update);
	/* Time base configuration */
	//TIM_SetAutoreload(TIM4,  TIM_ARR/TempAcceleration);
}
//----------------------------------------------------------------------------
