/*
    PCA9624.h - Library for NXP PCA9624 LED driver. 
    By: Antonio Tejada
    Modified: 2022-06-26

    Based on: 
        PCA9634.h - Library for NXP PCA9634 chip.
        Created by Nick van Tholen, June 2, 2020.
        Released for Emit IT.
        Last modified, 23 June, 2020.
*/
#ifndef pca9624
#define pca9624

#define LED_OFF_ALL 0x00
#define LED_ON_ALL 0x55
#define LED_PWM_ALL 0xAA
#define LEDOUT0 0x0C
#define LEDOUT1 0x0D
#define GRPPWM 0x0A

#include <Arduino.h>
#include <Wire.h>

class PCA9624
{
    public:
        PCA9624(uint8_t address);
        void softReset();
        void begin();
        uint8_t writeReg(uint8_t reg, uint8_t val);
        uint8_t readReg(uint8_t reg);
        void on(uint8_t pin);
        void off(uint8_t pin);
        void allOn();
        void allOff();
        void pwm(uint8_t pin, uint8_t value);
        void groupPwm(uint8_t value);
        uint8_t ledStatus(uint8_t pin);
        uint8_t pwmStatus(uint8_t pin);
    private:
        void chanPwm(uint8_t channel, uint8_t value);
        void grpPwm(uint8_t value);
        void chanMode(uint8_t type, uint8_t pin, bool all=false);
        uint8_t _addr;

};

#endif
