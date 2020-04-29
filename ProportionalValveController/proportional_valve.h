/* MIT License
% 
% Copyright (c) 2020 Jacek Garbulinski
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.
*/


#ifndef pvalve_controller_h
#define pvalve_controller_h
#include "Arduino.h"
#define VARIANT_MCK      84000000
#define PWM_INTERFACE    PWM
#define PWM_INTERFACE_ID  ID_PWM
// PWM_FREQUENCY < MCK(84MHz) / PWM_MAX_DUTY_CYCLE
#define PWM_FREQUENCY   5000
//#define PWM_MAX_DUTY_CYCLE  4096*2*2*2*2-1
#define PWM_MAX_DUTY_CYCLE  8191.0
#define PWM_MIN_DUTY_CYCLE  0
#define PRESSURES_SIZE  4
#define FAIL_SAFE_THRESHOLD  5

enum PValveStates {
  PBUILD, //
  PEXHAUST, //
  PKEEP,
  PCONTROL,
  PCONTROLRAMP
};

typedef enum PValveStates PValveState;

struct pvalve {
   
   PValveState vlvState;
   
   //+- bound when error is assumed to be zero
   float tolerance;
   
   //HARDWARE CONTROL
   uint32_t pwmPin;
   int32_t pwmDC;
   
   //PID
   float goalPressure;
   float rampPressure;
   float rampIncrement;
   float samplingFrequency;
   float samplingPeriod;
   float errors[PRESSURES_SIZE];
   float errorSum;
   float controlSignal;
   
   //PID constants
   float offsetGain;
   float windup;
   float pid_gain;
   float pid_derivative;
   float pid_integration;
   
   float intpart;
   float derpart;
   float normCtrlSig;
   uint8_t failSafeCounter;
   
};

typedef struct pvalve PValve ;



// x      --> 	y
//PWM MCU -->   Arduino PIN#
//PWM PINS [PWM 7-->6,6-->7,5-->8,4-->9] [DIGITAL 2-->43,38 3-->40 1-->36 0-->34]
uint8_t pvalve__chan(uint8_t pin) {
  //Serial.println( g_APinDescription[pin].ulPWMChannel);
  //Serial.println( pin);
  //return g_APinDescription[pin].ulPWMChannel;
  switch (pin) {
    case 43:
      return 2;
      break;
    case 40:
      return 3;
      break;
    case 38:
      return 2;
      break;
    case 36:
      return 1;
      break;
    case 34:
      return 0;
      break;
    case 9:
      return 4;
      break;
    case 8:
      return 5;
      break;
    case 7:
      return 6;
      break;
    case 6:
      return 7;
      break;
    default:
      // statements
      return 0;
      break;
  }
}

void pvalve__initPWM(uint32_t ulPin) {
  Serial.println("PIN:::");
  Serial.println(ulPin);
  Serial.println(g_APinDescription[ulPin].ulPin);
  Serial.println(g_APinDescription[ulPin].ulPinType);
  Serial.println(g_APinDescription[ulPin].ulPinConfiguration);
  Serial.println(":::PIN");
  PIO_Configure(g_APinDescription[ulPin].pPort,
                g_APinDescription[6].ulPinType,
                g_APinDescription[ulPin].ulPin,
                g_APinDescription[ulPin].ulPinConfiguration);
  //PIO_Configure( Pio* pPio, const EPioType dwType, const uint32_t dwMask, const uint32_t dwAttribute )
  //ulValue = 0;
  pmc_enable_periph_clk(PWM_INTERFACE_ID);
  //extern void PWMC_ConfigureClocks(uint32_t clka, uint32_t clkb, uint32_t mck ) ;
  PWMC_ConfigureClocks(PWM_FREQUENCY * PWM_MAX_DUTY_CYCLE, 0, VARIANT_MCK);
  //PWMEnabled = 1;
  PWMC_ConfigureChannel(PWM_INTERFACE, pvalve__chan(ulPin), PWM_CMR_CPRE_CLKA, 0, 0);
  //extern void PWMC_SetPeriod( Pwm* pPwm, uint32_t ul_channel, uint16_t period ) ;
  //extern void PWMC_SetDutyCycle( Pwm* pPwm, uint32_t ul_channel, uint16_t duty ) ;
  PWMC_SetPeriod(PWM_INTERFACE, pvalve__chan(ulPin), PWM_MAX_DUTY_CYCLE);
  PWMC_SetDutyCycle(PWM_INTERFACE, pvalve__chan(ulPin), 0);
  PWMC_EnableChannel(PWM_INTERFACE, pvalve__chan(ulPin));
  g_pinStatus[ulPin] = (g_pinStatus[ulPin] & 0xF0) | PIN_STATUS_PWM;
}


//from -1 to 1
void pvalve__setControlSignal(PValve* vlv, float value)
{
	vlv->controlSignal = value;
	if(value>1){
		vlv->controlSignal = 1;
		//Serial.println("control signal saturated");
	}
	else if(value<-1){
		vlv->controlSignal = -1;
		//Serial.println("control signal saturated");
	}
	if(vlv->controlSignal>0){
		vlv->pwmDC = (int32_t)(PWM_MAX_DUTY_CYCLE/2.0 +
		PWM_MAX_DUTY_CYCLE/2.0*vlv->controlSignal*vlv->offsetGain);
	}
	else{
		vlv->pwmDC = (int32_t)(PWM_MAX_DUTY_CYCLE/2.0 + PWM_MAX_DUTY_CYCLE/2.0*vlv->controlSignal);
	}
	if(vlv->pwmDC>PWM_MAX_DUTY_CYCLE) vlv->pwmDC = PWM_MAX_DUTY_CYCLE;
	if(vlv->pwmDC<0) vlv->pwmDC = 0;
	
	PWMC_SetDutyCycle(PWM_INTERFACE, pvalve__chan(vlv->pwmPin), (uint32_t)vlv->pwmDC);
	
}

PValve* pvalve__initialize(	uint32_t pwmPin, float tolerance, float samplingFrequency, 
							float pid_gain, float pid_derivative, float pid_integration, float windup, float offsetGain, float rampIncrement){
  PValve* vlv = (PValve*)malloc(sizeof(PValve));
  vlv->pwmPin = pwmPin;
  vlv->tolerance = tolerance;
  vlv->goalPressure = 0;
  vlv->rampPressure = 0;
  vlv->windup = windup;
  vlv->vlvState = PKEEP;
  vlv->controlSignal = 0;
  vlv->samplingFrequency = samplingFrequency;
  vlv->samplingPeriod = 1/samplingFrequency;
  vlv->rampIncrement = rampIncrement;
  for(uint8_t i=0;i<PRESSURES_SIZE;i++){
		vlv->errors[i] = 0;
	}
  
  vlv->errorSum = 0;
  vlv->pid_gain = pid_gain;
  vlv->pid_derivative = pid_derivative;
  vlv->pid_integration = pid_integration;
  vlv->offsetGain = offsetGain;
  
  vlv->intpart = 0;
  vlv->derpart = 0;
  vlv->normCtrlSig = 0;
  
  vlv->failSafeCounter = 0;
  
  if ( 	pwmPin == 6||pwmPin == 7||pwmPin == 8
		||pwmPin == 9||pwmPin == 34||pwmPin == 36
		||pwmPin == 38||pwmPin == 40){
  pvalve__initPWM(vlv->pwmPin);
  }
  else{
  Serial.println("PIN:::not a pwm");
  Serial.println(pwmPin);
  }
  pvalve__setControlSignal(vlv, -1);
  
  return vlv;
}


void pvalve__setGoalPressure(PValve* vlv, float goalPressure)
{
  if(goalPressure<0){ goalPressure = 0;}
  vlv->goalPressure = goalPressure;
  
  /*if( vlv->vlvState == PCONTROL ){ Serial.println("CONTROL ON");}
  else{ Serial.println("CONTROL OFF");};*/
  Serial.println(vlv->goalPressure);
}

void pvalve__startControl(PValve* vlv, float goalPressure)
{
  Serial.print("StartCTRL - STEP");
  pvalve__setGoalPressure(vlv, goalPressure);
  vlv->errorSum = 0;
  vlv->vlvState = PCONTROL;
}

void pvalve__startControlRamp(PValve* vlv, float goalPressure, float actualPressure)
{
  Serial.print("StartCTRL - RAMP");
  vlv->rampPressure = actualPressure;
  pvalve__setGoalPressure(vlv, goalPressure);
  vlv->errorSum = 0;
  vlv->vlvState = PCONTROLRAMP;
}

void pvalve__update(PValve* vlv, float measurement){
  //Serial.print('.');
  if (vlv->vlvState!=PCONTROL && vlv->vlvState!=PCONTROLRAMP) return;
    //error signal update
	
	if (vlv->errors[0] > -vlv->tolerance 
	  && vlv->errors[0] < vlv->tolerance){
	  vlv->errors[0] = 0;}
	
	for(uint8_t i=PRESSURES_SIZE-1;i>0;i--){
		vlv->errors[i] = vlv->errors[i-1];
	}
	
	if(vlv->vlvState==PCONTROL){
		vlv->errors[0] = vlv->goalPressure - measurement;
	}
	
	
	if(vlv->vlvState==PCONTROLRAMP){
		if(vlv->rampIncrement == 0){ vlv->rampPressure = vlv->goalPressure;}
		else{
		if(vlv->rampPressure<vlv->goalPressure) 
			vlv->rampPressure = vlv->rampPressure+vlv->rampIncrement;
		if(vlv->rampPressure>vlv->goalPressure)
			vlv->rampPressure = vlv->rampPressure-vlv->rampIncrement;
		if( (vlv->goalPressure-vlv->rampPressure)*(vlv->goalPressure-vlv->rampPressure) < vlv->rampIncrement*vlv->rampIncrement )
			vlv->rampPressure = vlv->goalPressure;
		}
		vlv->errors[0] = vlv->rampPressure - measurement;

		// UNSTABLE PROTECTION
		/*
		if( vlv->errors[0]*vlv->errors[0]>FAIL_SAFE_THRESHOLD*FAIL_SAFE_THRESHOLD ){
			vlv->failSafeCounter++;
			if( vlv->failSafeCounter >= 5 ){
			pvalve__startControl(vlv,2.0);
			vlv->failSafeCounter = 0;
			}
		}else{
			vlv->failSafeCounter = 0;
		}*/
	}
	
	  
	vlv->errorSum += vlv->errors[0];
	
	//anti wind up
	
	if(vlv->errorSum>vlv->windup){
	vlv->errorSum=vlv->windup;}
	if( vlv->errorSum<-vlv->windup){
	vlv->errorSum=-vlv->windup;}
	
	//
	vlv->intpart = vlv->pid_integration * vlv->errorSum;
	if (vlv->pid_integration == 0) vlv->intpart = 0;
	vlv->derpart = vlv->pid_derivative *
	( vlv->errors[0]+3*vlv->errors[1]-3*vlv->errors[2]-vlv->errors[3] )*0.1667*vlv->samplingFrequency;
	vlv->normCtrlSig = (vlv->pid_gain*vlv->errors[0]+ vlv->intpart+ vlv->derpart);
	pvalve__setControlSignal(vlv, vlv->normCtrlSig);
  
}

void pvalve__keep(PValve* vlv) {
	pvalve__setControlSignal(vlv, 0);
	vlv->vlvState = PKEEP;
}

void pvalve__stopControl(PValve* vlv)
{
  pvalve__keep(vlv);
}


void pvalve__exhaust(PValve* vlv)
{
  pvalve__setControlSignal(vlv, -1);
  vlv->vlvState = PEXHAUST;
}

void pvalve__buildup(PValve* vlv) {
  pvalve__setControlSignal(vlv, 1);
  vlv->vlvState = PBUILD;
}

void pvalve__setD(PValve* vlv, float derivative) {
  vlv->pid_derivative = derivative;
  Serial.print("PID _ Constant D: ");
  Serial.println(vlv->pid_derivative*1000);
}

void pvalve__setI(PValve* vlv, float integration) {
  vlv->pid_integration = integration;
  Serial.print("PID _ Constant I: ");
  Serial.println(vlv->pid_integration*1000);
}

void pvalve__setP(PValve* vlv, float gain) {
  vlv->pid_gain = gain;
  Serial.print("PID _ Gain P: ");
  Serial.println(vlv->pid_gain*1000);
}

void pvalve__setWindup(PValve* vlv, float windup) {
  vlv->windup = windup;
  Serial.print("PID _ Windup : ");
  Serial.println(vlv->windup*1000);
}

void pvalve__setOffsetGain(PValve* vlv, float offsetGain) {
  vlv->offsetGain = offsetGain;
  Serial.print("PID _ offsetGain : ");
  Serial.println(vlv->offsetGain);
}

void pvalve__sendUSBData(PValve* vlv){
  SerialUSB.write( (uint8_t*) &(vlv->goalPressure), 4);
  SerialUSB.write( (uint8_t*) &(vlv->rampPressure), 4);
  //SerialUSB.write( (uint8_t*) &(vlv->normCtrlSig), 4);
  //SerialUSB.write( (uint8_t*) &(vlv->errors[0]), 4);
  
}

void pvalve__deinitialize(PValve* vlv){
  free(vlv);
  return;
}

#endif