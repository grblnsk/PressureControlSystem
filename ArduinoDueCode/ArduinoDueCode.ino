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

#include <pressure_transducer.h>
#include <proportional_valve.h>
////////////////
#define ADC_FREQ_10_5_MHZ 10500000
//ADC
#define ADCCH0 ADC_CHANNEL_0
#define ADCCH1 ADC_CHANNEL_1
#define ADCCH2 ADC_CHANNEL_2
#define ADCCH3 ADC_CHANNEL_3
#define ADCCH4 ADC_CHANNEL_4
#define ADCCH5 ADC_CHANNEL_5
#define ADCCH6 ADC_CHANNEL_6
#define ADCCH7 ADC_CHANNEL_7


const uint16_t NO_CHANNELS = 12; //MAX 12
const float FREQUENCY = 20000;
const uint16_t FRAMES_IN_BUFFER = 40;
const float MULTIPLIER = 2625000;

const uint16_t USB_SEND_DIVIDER = 10;

const uint32_t TCCLKS = TC_CMR_TCCLKS_TIMER_CLOCK3;
const uint16_t FREQUENCY_TIMER_UNITS = round( MULTIPLIER / FREQUENCY ); //6.89;//

volatile uint32_t *const cvValue2_ = &(TC2->TC_CHANNEL[1].TC_CV);
uint8_t *cvValue2__ = (uint8_t*)cvValue2_;
volatile uint32_t *const cvValue1_ = &(TC1->TC_CHANNEL[1].TC_CV);
uint8_t *cvValue1__ = (uint8_t*)cvValue1_;
volatile uint32_t *const cvValue3_ = &(TC2->TC_CHANNEL[0].TC_CV);
uint8_t *cvValue3__ = (uint8_t*)cvValue3_;

const uint16_t SLASH = 0xAAAA;
uint8_t *SLASH_ = (uint8_t*)(&SLASH);

const uint16_t SLASH2 = 0xDDDD;
uint8_t *SLASH2_ = (uint8_t*)(&SLASH2);

volatile uint32_t packetCounter = 1;
volatile uint16_t frameCounter = 0;

const uint16_t PACKET_COUNT_BUFFER_SIZE_32 = FRAMES_IN_BUFFER;//24
const uint16_t FRAME_COUNT_BUFFER_SIZE_16 = FRAMES_IN_BUFFER;//24
const uint16_t TIMER_BUFFER_SIZE_32 = FRAMES_IN_BUFFER;//24
const uint16_t MEAS_BUFFER_SIZE_16 = FRAMES_IN_BUFFER * NO_CHANNELS ;//96

const uint16_t PACKET_COUNT_BUFFER_SIZE_8 = PACKET_COUNT_BUFFER_SIZE_32 * 4; //48
const uint16_t FRAME_COUNT_BUFFER_SIZE_8 = FRAME_COUNT_BUFFER_SIZE_16 * 2; //24
const uint16_t TIMER_BUFFER_SIZE_8 = TIMER_BUFFER_SIZE_32 * 4; //48
const uint16_t MEAS_BUFFER_SIZE_8 = MEAS_BUFFER_SIZE_16 * 2 ;//192

volatile uint32_t packetBuffer[PACKET_COUNT_BUFFER_SIZE_32];//16bit->48
volatile uint16_t frameBuffer[FRAME_COUNT_BUFFER_SIZE_16];//16bit->24
volatile uint32_t timerBuffer[TIMER_BUFFER_SIZE_32];//16bit->48

volatile uint16_t buffer1[MEAS_BUFFER_SIZE_16];//16bit*4->96
volatile uint16_t buffer2[MEAS_BUFFER_SIZE_16];//16bit*4->96


const uint16_t bytesPerPacket =
  2 + PACKET_COUNT_BUFFER_SIZE_8 +
  FRAME_COUNT_BUFFER_SIZE_8 + TIMER_BUFFER_SIZE_8
  + MEAS_BUFFER_SIZE_8 + 2;

uint64_t bufferAverage = 0;

// PRESSURE TRANSDUCER
PTrans* ptSel;
PTrans* pt1;
PTrans* pt2;
PTrans* pt3;

// VALVE CONTROLLER
PValve* vlvSel;
PValve* vlv1;
PValve* vlv2;
PValve* vlv3;
// SERIAL

char serialBuffer[32];
int8_t serialCx;
char commandsUSB[] = {'S', 'X', 'N'};
char commandsRS[] = {'B', 'K', 'E', 'H', 'S', ']', '[', '}', '{',
                     'b', 'v', 'n', 'm', '.', ',' , 'c', 'x', 's', 'a', 'N'
                    };
char incomingByteUSB = '0';
char incomingByteRS = '0';
float commPressure = 0;

enum CommStates {
  ON_DEMAND,
  CONTINUOUS
};
typedef enum CommStates CommState;
CommState commState = ON_DEMAND;

enum NumInputs {
  NumInput_OFF,
  NumInput_DECIMAL,
  NumInput_INTEGER
};
typedef enum NumInputs NumInput;
NumInput numInput = NumInput_OFF;

enum USBSends {
  USBSend_ON,
  USBSend_OFF,
  USBSend_BATCH1,
  USBSend_BATCH2,
  USBSend_BATCH3,
  USBSend_BATCH4,
  USBSend_BATCH5,
  USBSend_BATCH6
};
typedef enum USBSends USBSend;
USBSend usbSend = USBSend_OFF;

void setup()
{
  for (uint32_t i = 0; i < MEAS_BUFFER_SIZE_16; i++) {
    buffer1[i] = 0;
    buffer2[i] = 0;
  }
  //COMM
  while (!Serial);
  SerialUSB.begin(2000000);
  Serial.begin(115200);
  delay(50);
  Serial.println("Start Setup.");
  ////POWER
  /* turn on the timer clock in the power management controller */
  pmc_set_writeprotect(false);
  /* 4 is the timer number * timer channels (3) + the channel number (=(1*3)+1) for timer1 channel1 */
  pmc_enable_periph_clk(ID_TC6);
  pmc_enable_periph_clk(ID_TC8);
  pmc_enable_periph_clk(ID_TC7);
  pmc_enable_periph_clk(ID_TC4);
  pmc_enable_periph_clk(ID_ADC);

  ////TIMER CONFIGURATION
  //TIMER 10.5MHz
  // TC2 clock: 42 MHz;Period: 23.81ns;
  /* we want wavesel 00 without RC */

  //MEASUREMENT TIME TEST TIMER
  TC_Configure(TC2,/* channel */1, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK1);
  TC_SetRC(TC2, 1, 42000000);


  //ONE SECOND TIMER
  TC_Configure(TC2,/* channel */2, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK1);
  TC_SetRC(TC2, 2, 42000000 / (FREQUENCY / FRAMES_IN_BUFFER) /*42000000 //->1 sek*/);

  //COMMUNICATION TIMER
  TC_Configure(TC2,/* channel */0, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK1);
  TC_SetRC(TC2, 0, 420000);//->100Hz

  //MEASUREMENT TIMER
  //TIMER 10.5 MHz
  // TC1 clock: 10.5 MHz;TCCLK3=>84/32; 2625 sets <> 1000 Hz,10ms, interrupt frequency rate
  TC_Configure(TC1,/* channel */1, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TCCLKS);
  TC_SetRC(TC1, 1, FREQUENCY_TIMER_UNITS);

  ////ADC
  //all

  adc_init(ADC, SystemCoreClock, ADC_FREQ_10_5_MHZ, ADC_STARTUP_FAST);
  adc_set_resolution(ADC, ADC_12_BITS);
  adc_configure_power_save(ADC, 0, 0); // Disable sleep
  adc_set_bias_current(ADC, 1);
  adc_configure_trigger(ADC, ADC_TRIG_SW , ADC_MR_FREERUN_OFF);
  ADC->ADC_MR |= ADC_MR_FREERUN_OFF;
  adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1); // Set timings - standard values
  adc_disable_anch( ADC );
  adc_set_writeprotect(ADC, 0x41444301);
  adc_stop_sequencer(ADC); // not using it,
  adc_disable_tag(ADC); // it has to do with sequencer, not using it
  adc_disable_ts(ADC); // deisable temperature sensor
  adc_disable_all_channel(ADC);

  for (uint8_t i = 0; i < NO_CHANNELS && i < 12; i++) {

    //ifs because of arduino pinout
    if (i == 8) {
      adc_disable_channel_differential_input(ADC, static_cast<adc_channel_num_t>(12));
      adc_enable_channel(ADC, static_cast<adc_channel_num_t>(12)); // just one channel enabled
      continue;
    }
    if (i == 9) {
      adc_disable_channel_differential_input(ADC, static_cast<adc_channel_num_t>(13));
      adc_enable_channel(ADC, static_cast<adc_channel_num_t>(13)); // just one channel enabled
      continue;
    }
    adc_disable_channel_differential_input(ADC, static_cast<adc_channel_num_t>(i));
    adc_enable_channel(ADC, static_cast<adc_channel_num_t>(i)); // just one channel enabled

  }

  ////DMA

  PDC_ADC->PERIPH_RPR = (uint32_t) buffer1; // address of buffer
  PDC_ADC->PERIPH_RNPR = (uint32_t) buffer2;

  PDC_ADC->PERIPH_RCR = MEAS_BUFFER_SIZE_16;
  PDC_ADC->PERIPH_RNCR = MEAS_BUFFER_SIZE_16;

  PDC_ADC->PERIPH_PTCR = PERIPH_PTCR_RXTEN; // enable receive

  ////INTERRUPTS
  //TIMER
  // enable timer interrupts on the timer
  TC2->TC_CHANNEL[1].TC_IER = TC_IER_CPCS;
  TC2->TC_CHANNEL[1].TC_IDR = ~TC_IER_CPCS;

  TC2->TC_CHANNEL[2].TC_IER = TC_IER_CPCS;
  TC2->TC_CHANNEL[2].TC_IDR = ~TC_IER_CPCS;

  TC2->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
  TC2->TC_CHANNEL[0].TC_IDR = ~TC_IER_CPCS;

  // enable timer interrupts on the timer
  TC1->TC_CHANNEL[1].TC_IER = TC_IER_CPCS;
  TC1->TC_CHANNEL[1].TC_IDR = ~TC_IDR_CPCS;
  //ADC
  // enable timer interrupts on the ADC
  //ADC->ADC_ISR = ADC_IER_ENDRX
  ADC->ADC_IDR = ~(1 << 27);
  ADC->ADC_IER = 1 << 27;

  /* Enable the interrupt in the nested vector interrupt controller */
  /* TC4_IRQn where 4 is the timer number * timer channels (3) + the channel number (=(1*3)+1) for timer1 channel1 */
  //PRIORITY
  NVIC_EnableIRQ(TC6_IRQn);//timer 2, channel 0
  NVIC_EnableIRQ(TC7_IRQn);//timer 2, channel 1
  NVIC_EnableIRQ(TC8_IRQn);//timer 2, channel 2
  NVIC_EnableIRQ(TC4_IRQn);//timer 1, channel 1
  NVIC_EnableIRQ(ADC_IRQn);

  //TRANSDUCER & VALVE INITIALIZE
  pt1 = ptrans__initialize(10, 0.031372, -0.21524);
  ptSel = pt1;
  pt2 = ptrans__initialize(8, 0.031484, -0.19403);
  pt3 = ptrans__initialize(9, 0.031597,  -0.49121);


  float temp_sampfreq = float( FREQUENCY ) / float(FRAMES_IN_BUFFER);
  const float RAMP_INCREMENT = 0.05;//typically 0.01
  vlv1 = pvalve__initialize(9,   0, temp_sampfreq,
                            0.15, 0.000001,  0.0005,
                            1000, 1, RAMP_INCREMENT);
  vlvSel = vlv1;
  vlv2 = pvalve__initialize(8,   0, temp_sampfreq,
                            0.15, 0.000001, 0.0005,
                            1000, 1, RAMP_INCREMENT);
  vlv3 = pvalve__initialize(7,   0, temp_sampfreq,
                            0.15, 0.000001,  0.0005,
                            1000, 1, RAMP_INCREMENT);



  //STARTUP INFO
  Serial.println("Finish Setup. Number of bytes per packet:");
  Serial.println(bytesPerPacket);
  Serial.println("Measurement Freq.:");
  Serial.println(FREQUENCY);
  Serial.println("Frames in a packet:");
  Serial.println(FRAMES_IN_BUFFER);
  Serial.println("No channels:");
  Serial.println(NO_CHANNELS);
  Serial.println("Bytes Per Second:");
  Serial.println(bytesPerPacket * FREQUENCY / FRAMES_IN_BUFFER);
  //adc_check(ADC, 84000000);

  delay(100);
  //START TIMERS
  TC_Start(TC2, 1);
  delay(33);
  TC_Start(TC2, 2);
  delay(33);
  TC_Start(TC2, 0);
  delay(33);
  /////////////STARTS MEASUREMENT
  TC_Start(TC1, 1);
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on
  TC_GetStatus(TC1, 1);

  pvalve__keep(vlv1);
  pvalve__keep(vlv2);
  pvalve__keep(vlv3);
  Serial.println("End Setup");
  delay(100);
  //////////////////////

}

uint32_t barCounter = 0;
void loop() {
  if (packetCounter > barCounter && commState == CONTINUOUS) {
    barCounter = packetCounter + USB_SEND_DIVIDER;
    printData();
  }
  commUSB();
  commRS232();
  Serial.flush();
  //while (1) {}
}

void onBufferFullRoutine(volatile uint16_t buff[]) {
  ////
  /////UPDATE PRESSURE TRANSDUCERS
  for (int i = 0; i < FRAMES_IN_BUFFER; i++) {
    bufferAverage += *(buff + pt1->channel + i * NO_CHANNELS);
  }
  bufferAverage = bufferAverage / FRAMES_IN_BUFFER;
  ptrans__update(pt1, bufferAverage);
  bufferAverage = 0;
  //

  for (int i = 0; i < FRAMES_IN_BUFFER; i++) {
    bufferAverage += *(buff + pt2->channel + i * NO_CHANNELS);
  }
  bufferAverage = bufferAverage / FRAMES_IN_BUFFER;
  ptrans__update(pt2, bufferAverage);
  bufferAverage = 0;
  /////

  /////
  for (int i = 0; i < FRAMES_IN_BUFFER; i++) {
    bufferAverage += *(buff + pt3->channel + i * NO_CHANNELS);
  }
  bufferAverage = bufferAverage / FRAMES_IN_BUFFER;
  ptrans__update(pt3, bufferAverage);
  bufferAverage = 0;
  ////////////////////////////////
  /////

}

void printData() {
  serialCx = snprintf ( serialBuffer, 32, "%3.2f,%3.2f,%3.2f,%3.2f\r\n",
                        vlvSel->goalPressure , vlvSel->rampPressure,
                        ptrans__getPressure(ptSel), vlvSel->errors[0] + vlvSel->goalPressure);

  if (serialCx >= 0 && serialCx < 32) { // check returned value
    SerialUSB.print(serialBuffer);
  }
}

void ADC_Handler()
{
  //Serial.println("1:" + String(ADC->ADC_ISR));
  //ENDRX
  PDC_ADC->PERIPH_RNCR = MEAS_BUFFER_SIZE_16;
  //jezeli byl buff 1
  packetCounter++;
  frameCounter = 0;
  if ( (packetCounter + 1) & 0x00000001) {
    PDC_ADC->PERIPH_RNPR = (uint32_t) buffer1;
    onBufferFullRoutine(buffer1);
  } else {
    PDC_ADC->PERIPH_RNPR = (uint32_t) buffer2;
    onBufferFullRoutine(buffer2);
  }
}

//tc1 ch1
void TC4_Handler()
{
  TC_GetStatus(TC1, 1);
  frameCounter++;
  packetBuffer[frameCounter - 1] = packetCounter;
  frameBuffer[frameCounter - 1] = frameCounter;
  timerBuffer[frameCounter - 1] = *(cvValue2_);
  adc_start(ADC);
}



//tc2 ch1
void TC7_Handler()
{
  TC_GetStatus(TC2, 1);
  Serial.println('.');
}

//tc2 ch2
void TC8_Handler()
{
  TC_GetStatus(TC2, 2);
  pvalve__update(vlv1, ptrans__getPressure(pt1));
  pvalve__update(vlv2, ptrans__getPressure(pt2));
  pvalve__update(vlv3, ptrans__getPressure(pt3));
}
//tc2 ch0
void TC6_Handler()
{
  TC_GetStatus(TC2, 0);

}

void commUSB() {
  if (SerialUSB.available() > 0) {
    // read the incoming byte:
    incomingByteUSB = SerialUSB.read();
    if (numInput == NumInput_OFF) {
      switch (incomingByteUSB) {
        case 'I':
          SerialUSB.write('I');
          numInput = NumInput_INTEGER;
          break;
        case 'S':
          //start S
          SerialUSB.write('S');
          //delay(100);
          TC_Start(TC1, 1);
          TC_Start(TC2, 2);
          digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on
          TC_GetStatus(TC1, 1);
          TC_GetStatus(TC2, 2);
          break;
        case 'X':
          //stop X
          SerialUSB.write('X');
          TC_Stop(TC1, 1);
          TC_Stop(TC2, 2);
          digitalWrite(LED_BUILTIN, LOW);
          break;
        case 'R':
          //stop X
          SerialUSB.write('R');
          if (usbSend == USBSend_OFF) {
            usbSend = USBSend_ON;
          }
          /*SerialUSB.write( (uint8_t*) &packetCounter, 4);     //u32
            SerialUSB.write( (uint8_t*) & (pt1->lastAdcVal), 4); //u32
            SerialUSB.write( (uint8_t*) & (pt2->lastAdcVal), 4); //u32
            SerialUSB.write( (uint8_t*) & (pt3->lastAdcVal), 4); //u32
            pvalve__sendUSBData(vlv1);                        //2xf32
            pvalve__sendUSBData(vlv2);                        //2xf32
            pvalve__sendUSBData(vlv3);                        //2xf32*/
          break;
        case 'C':
          //stop X
          SerialUSB.write('C');
          commState = CONTINUOUS;
          break;
        case 'D':
          //stop X
          SerialUSB.write('D');
          commState = ON_DEMAND;
          break;
        case '1':
          SerialUSB.write('1');
          vlvSel = vlv1;
          ptSel = pt1;
          //Serial.println("vlv1_sel");
          break;
        case '2':
          SerialUSB.write('2');
          vlvSel = vlv2;
          ptSel = pt2;
          //Serial.println("vlv2_sel");
          break;
        case '3':
          SerialUSB.write('3');
          vlvSel = vlv3;
          ptSel = pt3;
          //Serial.println("vlv3_sel");
          break;
        case 'J':
          SerialUSB.write('J');
          pvalve__startControlRamp(vlvSel, vlvSel->goalPressure, ptrans__getPressure(ptSel));
          break;
        case 'K':
          SerialUSB.write('K');
          pvalve__keep(vlvSel);
          break;
        default:
          //error N
          digitalWrite(LED_BUILTIN, LOW);
          SerialUSB.write('N');
          break;
      }
    }
    else if (numInput == NumInput_INTEGER) {
      SerialUSB.write(incomingByteUSB);
      commPressure = incomingByteUSB * 1.0;
      numInput = NumInput_DECIMAL;
    }
    else if (numInput == NumInput_DECIMAL) {
      SerialUSB.write(incomingByteUSB);
      commPressure += incomingByteUSB * 0.01;
      vlvSel->goalPressure = commPressure;
      numInput = NumInput_OFF;
    }

  }
switch (usbSend) {
  case USBSend_ON:
    SerialUSB.write( (uint8_t*) &packetCounter, 4);     //u32
    usbSend = USBSend_BATCH1;
    break;
  case USBSend_BATCH1:
    SerialUSB.write( (uint8_t*) & (pt1->pressureVal), 4);
    usbSend = USBSend_BATCH2;
    break;
  case USBSend_BATCH2:
    SerialUSB.write( (uint8_t*) & (pt2->pressureVal), 4);
    usbSend = USBSend_BATCH3;
    break;
  case USBSend_BATCH3:
    SerialUSB.write( (uint8_t*) & (pt3->pressureVal), 4);
    usbSend = USBSend_BATCH4;
    break;
  case USBSend_BATCH4:
    pvalve__sendUSBData(vlv1);
    usbSend = USBSend_BATCH5;
    break;
  case USBSend_BATCH5:
    pvalve__sendUSBData(vlv2);
    usbSend = USBSend_BATCH6;
    break;
  case USBSend_BATCH6:
    pvalve__sendUSBData(vlv3);
    usbSend = USBSend_OFF;
    break;
  default:
    usbSend = USBSend_OFF;
    break;
  }

}

void commRS232() {
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByteRS = Serial.read();
    switch (incomingByteRS) {
      case 'C':
        //stop X
        commState = CONTINUOUS;
        break;
      case 'D':
        //stop X
        commState = ON_DEMAND;
        break;
      case 'B':
        pvalve__buildup(vlvSel);
        Serial.write(commandsRS, 1);
        Serial.write(1 + 48);
        break;
      case 'K':
        pvalve__keep(vlvSel);
        Serial.write(commandsRS + 1, 1);
        Serial.write(1 + 48);
        break;
      case 'E':
        pvalve__exhaust(vlvSel);
        Serial.write(commandsRS + 2, 1);
        Serial.write(1 + 48);
        break;
      case 'H':
        pvalve__startControl(vlvSel, (vlvSel->goalPressure) + 0.0);
        Serial.write(commandsRS + 3, 1);
        break;
      case 'S':
        //start S
        //delay(100);
        TC_Start(TC1, 1);
        digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on
        TC_GetStatus(TC1, 1);
        Serial.write(commandsRS + 4, 1);
        break;
      case ']':
        pvalve__setGoalPressure(vlvSel, (vlvSel->goalPressure) + 1);
        Serial.write(commandsRS + 5, 1);
        break;
      case '[':
        pvalve__setGoalPressure(vlvSel, (vlvSel->goalPressure) - 1);
        Serial.write(commandsRS + 6, 1);
        break;
      case '}':
        pvalve__setGoalPressure(vlvSel, (vlvSel->goalPressure) + 5);
        Serial.write(commandsRS + 7, 1);
        break;
      case '{':
        pvalve__setGoalPressure(vlvSel, (vlvSel->goalPressure) - 5);
        Serial.write(commandsRS + 8, 1);
        break;
      case 'b':
        pvalve__setP(vlvSel, (vlvSel->pid_gain) + 1 / 1000.0);
        Serial.write(commandsRS + 9, 1);
        break;
      case 'v':
        pvalve__setP(vlvSel, (vlvSel->pid_gain) - 1 / 1000.0);
        Serial.write(commandsRS + 10, 1);
        break;
      case 'm':
        pvalve__setI(vlvSel, (vlvSel->pid_integration) + 0.01);
        Serial.write(commandsRS + 11, 1);
        break;
      case 'n':
        pvalve__setI(vlvSel, (vlvSel->pid_integration) - 0.01);
        Serial.write(commandsRS + 12, 1);
        break;
      case '.':
        pvalve__setD(vlvSel, (vlvSel->pid_derivative) + 0.00001);
        Serial.write(commandsRS + 13, 1);
        break;
      case ',':
        pvalve__setD(vlvSel, (vlvSel->pid_derivative) - 0.00001);
        Serial.write(commandsRS + 14, 1);
        break;
      case 'c':
        pvalve__setWindup(vlvSel, (vlvSel->windup) + 1);
        Serial.write(commandsRS + 13, 1);
        break;
      case 'x':
        pvalve__setWindup(vlvSel, (vlvSel->windup) - 1);
        Serial.write(commandsRS + 14, 1);
        break;
      case 's':
        pvalve__setOffsetGain(vlvSel, (vlvSel->offsetGain) + 0.01);
        Serial.write(commandsRS + 15, 1);
        break;
      case 'a':
        pvalve__setOffsetGain(vlvSel, (vlvSel->offsetGain) - 0.01);
        Serial.write(commandsRS + 16, 1);
        break;
      case 'J':
        pvalve__startControlRamp(vlvSel, (vlvSel->goalPressure) + 0.0, ptrans__getPressure(ptSel));
        Serial.write(commandsRS + 17, 1);
        break;
      case '1':
        vlvSel = vlv1;
        ptSel = pt1;
        Serial.println("Valve/Press-1 SELECTED");
        break;
      case '2':
        vlvSel = vlv2;
        ptSel = pt2;
        Serial.println("Valve/Press-2 SELECTED");
        break;
      case '3':
        vlvSel = vlv3;
        ptSel = pt3;
        Serial.println("Valve/Press-3 SELECTED");
        break;
      default:
        Serial.write(commandsRS + 18, 1);
        break;
    }
  }
}
