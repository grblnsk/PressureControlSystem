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

#ifndef pressure_transducer_h
#define pressure_transducer_h

#include "Arduino.h"

struct pressure_transducer {
   uint8_t channel;
   float psi2adc_a;
   float psi2adc_b;
   float adc2psi_a;
   float adc2psi_b;
   float pressureVal;
   uint32_t lastAdcVal;
   
};

typedef struct pressure_transducer PTrans ;

uint16_t ptrans__conv_psi_to_adc(PTrans* ptrans, float psiValue){
  return (ptrans->psi2adc_a) * psiValue + ptrans->psi2adc_b;
}

float ptrans__conv_adc_to_psi(PTrans* ptrans,uint32_t adcValue){
  return (ptrans->adc2psi_a) * adcValue + ptrans->adc2psi_b;
}


PTrans* ptrans__initialize(uint8_t pin,float a2p_a,float a2p_b){
  PTrans* ptrans = (PTrans*)malloc(sizeof(PTrans));
  if(pin<8){
		ptrans->channel = 7-pin;
	}else{
		ptrans->channel = pin;
	}
  ptrans->psi2adc_a = 1/a2p_a;
  ptrans->psi2adc_b = a2p_b/a2p_a;
  ptrans->adc2psi_a = a2p_a;
  ptrans->adc2psi_b = a2p_b;
  ptrans->lastAdcVal = 0;
  ptrans->pressureVal = 0;
  return ptrans;
}


void ptrans__update(PTrans* ptrans, uint32_t value){
	ptrans->lastAdcVal = value;
}

float ptrans__getPressure(PTrans* ptrans){
	ptrans->pressureVal = ptrans__conv_adc_to_psi(ptrans, ptrans->lastAdcVal);
	return ptrans->pressureVal;
}

uint32_t ptrans__getADCValue(PTrans* ptrans){
	return ptrans->lastAdcVal;
}

void ptrans__deinitialize(PTrans* ptrans){
  free(ptrans);
  return;
}





#endif
