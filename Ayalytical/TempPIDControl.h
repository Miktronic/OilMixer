/*
 * TempPIDControl.h
 *
 * Created: 8/20/2019 2:55:47 PM
 *  Author: adam
 */ 


#ifndef TEMPPIDCONTROL_H_
#define TEMPPIDCONTROL_H_

#define OUTPUT_SCALE 10

typedef struct
{
	float pState;
	float iState;
	float dState;
	int32_t maxDif;
	int32_t minDif;
	int32_t iMax;
	int32_t iMin;
	float pGain;
	float iGain;
	float dGain;
	float prevError;
	float PIDVal;
}PIDStruct;

//////////////////////////////////////////////////////////////////////////
// Function prototypes
//////////////////////////////////////////////////////////////////////////
int32_t compute_temp_PID(PIDStruct *curPID, int32_t error, uint32_t state);
void compute_temp_PID_duty(PIDStruct *curPID, float tempDif, float elapsedTime);

#endif /* TEMPPIDCONTROL_H_ */