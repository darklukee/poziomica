/*
 * myTask.hpp
 *
 *  Created on: 10-11-2012
 *      Author: lukee
 */

#ifndef ATASK_HPP_
#define ATASK_HPP_

#include "stm32f4xx.h"
/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
//my include
#include "hw_config.h"
#include "math.h"

static const uint8_t queueSIZE = 6;

extern "C" void vBALANCETask(void *pvParameters);

extern "C++"
{

class CBalance
{
private:
	static const uint16_t N = 32; //table size
	static const float a, b, c, d; //exponential coefficients
	float accelXFloat, accelYFloat;
	int8_t meanX[N], meanY[N];
	int16_t meanXSigma, meanYSigma, accelX, accelY;
	int16_t accelXCal, accelYCal;
	static const uint8_t maxValue; //maksymalna wartosc wskazania przy wychyleniu 90st
	static const uint8_t cmpValue; //wartosc do porownywania przyspieszenia
	uint16_t k, j;
	//void GetData(void);

	static int temp;

public:
	int8_t xBuffer_receive[queueSIZE]; //bufor wejsciowy

	CBalance(void);
	//~CBalance(void);
	void Calibrate(void);
	void Calculate(void);
	void SetData(void);
};
}
#endif /* ATASK_HPP_ */
