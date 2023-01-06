/*
    PCA9624.cpp - Library for NXP PCA9624 LED driver. 
    By: Antonio Tejada
    Modified: 2022-06-26

    Based on:
        PCA9634.cpp - Library for NXP PCA9634 chip.
        Created by Nick van Tholen, June 2, 2020.
        Released for Emit IT.
        Last modified, 23 June, 2020.
*/
#include "Arduino.h"
#include "PCA9624.h"

// Constructor //

// Initialize the Library
PCA9624::PCA9624(uint8_t address){
    _addr = address;
}

// Public //

// Uses a I2C adress reset
void PCA9624::softReset()
{
    Wire.beginTransmission(0x03);
    Wire.write(0xA5);
    Wire.write(0x5A);
    Wire.endTransmission();
}

// Sets up the whole chip and I2C connection
void PCA9624::begin()
{
    Wire.begin();
    delay(10);
    writeReg(0x00, 0x01);  //sets MODE1 to not use auto-increment, sleep off, no sub-addresses
    delayMicroseconds(500);
    writeReg(0x01, 0x05); //sets MODE2 to default values for PCA9624
    delay(10);
groupPwm(255);
    delay(10);
}

// Writes to a register
uint8_t PCA9624::writeReg(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission();
}

// Reads a register
uint8_t PCA9624::readReg(uint8_t reg)
{
    Wire.beginTransmission(_addr);
    Wire.write(reg & 0x1F);
    Wire.endTransmission();
    Wire.requestFrom(_addr, (uint8_t)1);
    return Wire.read();
}



// Turn a channel on
//AT: removed chanPwm calls, brightness should be separate from on/off state
void PCA9624::on(uint8_t pin){
    chanMode(3, pin);
}

// Turn a channel off
void PCA9624::off(uint8_t pin){
    chanMode(0, pin);
}

// Turn all channels on
void PCA9624::allOn(){
    chanMode(3, 0, true);

}

// Turn all channels off
void PCA9624::allOff(){
    chanMode(0, 0, true);
}


// Individual PWM control
void PCA9624::pwm(uint8_t pin, uint8_t value){
    chanPwm(pin, value);
}

// Group PWM control
void PCA9624::groupPwm(uint8_t value){
     grpPwm(value);
}

// Check the output state of an LED
uint8_t PCA9624::ledStatus(uint8_t pin){
    uint8_t regValue;
    bool first, second;
    if (pin < 4){
        regValue = readReg(LEDOUT0);
        first = bitRead(regValue, (pin*2));
        second = bitRead(regValue, (pin*2+1));
    }
    else {
        pin -= 4;
        regValue = readReg(LEDOUT1);
        first = bitRead(regValue, (pin*2));
        second = bitRead(regValue, (pin*2+1));
    }
    if (!first && !second){
        return 0;
    }
    else if(first && !second){
        return 1;
    }
    else if(!first && second){
        return 2;
    }
    else if(first && second){
        return 3;
    }
}

// Check the PWM value of a channel
uint8_t PCA9624::pwmStatus(uint8_t pin){
    return readReg(pin + 2);
}

// Private functions //

// Sets a channel or all channels to different output state
void PCA9624::chanMode(uint8_t type, uint8_t pin, bool all)
{
    uint8_t dataType, regValue;
    if(type > 3){
        type = 0;
    }
    switch (type)
    {
    case 0: //LDRx = 00 == LED driver x is off
        if(all){
            writeReg(LEDOUT0, LED_OFF_ALL);
            writeReg(LEDOUT1, LED_OFF_ALL);
            break;
        }
        else if(pin<4){
            regValue = readReg(LEDOUT0);
            bitClear(regValue, (pin*2));
            bitClear(regValue, (pin*2+1));
            writeReg(LEDOUT0, regValue);
            break;
        }
        else if(pin>=4){
            pin -= 4;
            regValue = readReg(LEDOUT1);
            bitClear(regValue, (pin*2));
            bitClear(regValue, (pin*2+1));
            writeReg(LEDOUT1, regValue);
            break;
        }
        break;
    case 1: //LDRx = 01 == LED driver x is fully on (no group or individual dimming)
        if(all){
            writeReg(LEDOUT0, LED_ON_ALL);
            writeReg(LEDOUT1, LED_ON_ALL);
            break;
        }
        else if(pin<4){
            regValue = readReg(LEDOUT0);
            bitSet(regValue, (pin*2));
            bitClear(regValue, (pin*2+1));
            writeReg(LEDOUT0, regValue);
            break;
        }
        else if(pin>=4){
            pin -= 4;
            regValue = readReg(LEDOUT1);
            bitSet(regValue, (pin*2));
            bitClear(regValue, (pin*2+1));
            writeReg(LEDOUT1, regValue);
            break;
        }
        break;
    case 2: //LDRx = 10 == LED driver x uses individual dimming, but no group dimming
        if(all){
            writeReg(LEDOUT0, LED_PWM_ALL);
            writeReg(LEDOUT1, LED_PWM_ALL);
            break;
        }
        else if(pin<4){
            regValue = readReg(LEDOUT0);
            bitClear(regValue, (pin*2));
            bitSet(regValue, (pin*2+1));
            writeReg(LEDOUT0, regValue);
            break;
        }
        else if(pin>=4){
            pin -= 4;
            regValue = readReg(LEDOUT1);
            bitClear(regValue, (pin*2));
            bitSet(regValue, (pin*2+1));
            writeReg(LEDOUT1, regValue);
            break;
        }
        break;
    case 3: //LDRx = 10 == LED driver x uses individual dimming and group dimming
        if(all){
            writeReg(LEDOUT0, 0xFF);
            writeReg(LEDOUT1, 0xFF);
            break;
        }
        else if(pin<4){
            regValue = readReg(LEDOUT0);
            bitSet(regValue, (pin*2));
            bitSet(regValue, (pin*2+1));
            writeReg(LEDOUT0, regValue);
            break;
        }
        else if(pin>=4){
            pin -= 4;
            regValue = readReg(LEDOUT1);
            bitSet(regValue, (pin*2));
            bitSet(regValue, (pin*2+1));
            writeReg(LEDOUT1, regValue);
            break;
        }
        break;
    }
}

// Writes a pwm value to a channel
void PCA9624::chanPwm(uint8_t channel, uint8_t value)
{
    writeReg((channel+2), value);
}


// Writes a pwm value to a channel
void PCA9624::grpPwm(uint8_t value)
{
    writeReg(GRPPWM, value);
}


