/*
 * TempPIDControl.c
 *
 * Created: 8/20/2019 2:55:34 PM
 *  Author: adam
 */ 

#include "avr//io.h"
#include "TempPIDControl.h"

//Returns temp set point adjustment in milli°C
int32_t compute_temp_PID(PIDStruct *curPID, int32_t error, uint32_t state)
{
	int32_t pTerm = 0;
	int32_t iTerm = 0;
	int32_t dTerm = 0;
	
	pTerm = curPID->pGain * error;
	
	curPID->iState += error;
	//if (curPID->iState > curPID->iMax)
	//{
		//curPID->iState = curPID->iMax;
	//}
	//else if (curPID->iState < curPID->iMin)
	//{
		//curPID->iState = curPID->iMin;
	//}
	
	iTerm = curPID->iGain * curPID->iState;
	dTerm = curPID->dGain * (curPID->dState - state);
	curPID->dState = state;
	curPID->pState = pTerm; //This is only used to expose it to pc cmd
	
	return (pTerm + iTerm + dTerm);
}

//Returns heater PWM duty output from 0-100%
void compute_temp_PID_duty(PIDStruct *curPID, float tempDif, float elapsedTime)
{
	//Gain terms can be set from 0 - 999.99	
	//Calculate p-term
	curPID->pState = tempDif;
	
	//Calculate i-term within ranges of iMin/iMax
	//if ((tempDif > curPID->minDif) && (tempDif < curPID->maxDif))
	//{
		//curPID->iState = curPID->iState + (curPID->iGain * tempDif);
	//}
	
	curPID->iState += tempDif;
	if (curPID->iState > curPID->iMax)
	{
		curPID->iState = curPID->iMax;
	}
	if (curPID->iState < curPID->iMin)
	{
		curPID->iState = curPID->iMin;
	}
	
	//Calculate d-term
	curPID->dState = ((tempDif - curPID->prevError) / elapsedTime);
	
	//Calculate final PID val
	curPID->PIDVal = (curPID->pGain * curPID->pState) + (curPID->iGain * curPID->iState) - (curPID->dGain * curPID->dState);
	curPID->PIDVal = curPID->PIDVal / OUTPUT_SCALE;
	
	
	//Limit PID value to pwm duty range (0 - 100)
	if (curPID->PIDVal < 0)
	{
		curPID->PIDVal = 0;
	}
	if (curPID->PIDVal > 100)
	{
		curPID->PIDVal = 100;
	}
	
	curPID->prevError = tempDif;
}