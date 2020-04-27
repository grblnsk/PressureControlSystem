# Pressure Control System with Arduino Duo

### 20kHz Pressure Measurement and PI Control of Pneumatic Artificial Muscles with Arduino Duo

##### User Code - collectData.m 
collectData.m is a user MATLAB interface to communicate with the Arduino Board. Its GUI shows figures with recent pressure levels values in the pneumatic artificial muscles, and allows to set new pressure values for each muscle.
##### Hardware Code - ArduinoDueCode.ino
ArduinoDueCode.ino contains a hybrid of standard C++ for Arduino code and embedded C code for ARM Cortex-M3 processors to greatly improve measurement speed. It uses pressure_transducer.h and proportional_valve.h which need to be added as Arduino libraries.

##### Overview of Hardware Setup
A custom pressure regulation system was developed to minimize the errors that were observed when using off-the-shelf pressure regulators. The system was composed of:
* an Adruino Due board running a PI controller
* 12V and 24V power supply
* 3-channel an active op-amp low pass filter (3.3V PWM to 5V Analog conversion) (based on LM358N op-amps)
* 3-channel tunable op-amp signal splitter (based on LM358N op-amps)
* 3 x Omega PC209-200G5V pressure transducer
* 3 x high-speed proportional pneumatic valve (Enfield M2d)
* pressure regulator (Exceleon R73G)
* air compressor (DeWalt D55146)
* a laptop computer to remotely supervise the pressure readings and change the desired pressure value

##### Signal Flow:
* The laptop sends desired pressure values to the control board and periodically acquires pressure data. 
* The control board reads the voltage from pressure transducers. Clocked by Arduino board's timer interrupts, the signal is sampled at the frequency of 20kHz with 12-bit resolution. 
* A proportional-integral (PI) feedback controller with an anti-windup scheme runs its loop at 50Hz for which an average pressure value is calculated based on the last 400 samples.
* The Arduino board translates a floating point control signal into a 3.3V pulse width modulation (PWM) signal with 5kHz frequency and 0-100% duty cycle of 13-bit resolution. 
* Then, the 3.3V PWM signal goes through the active low-pass filter that outputs 0-5V analog signal to control valve's inner driver.

