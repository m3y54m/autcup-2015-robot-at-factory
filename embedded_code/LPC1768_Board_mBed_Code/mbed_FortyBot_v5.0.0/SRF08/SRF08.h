/*
Copyright (c) 2012 Brent Dekker

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#ifndef SRF08_H
#define SRF08_H

#include "mbed.h"

/**
 * The SRF08 is an ultrasonic range finder with an I2C interface that allows 
 * the measurement to be read directly in centimetres. More information can be
 * found on this website: http://www.robot-electronics.co.uk/htm/srf08tech.shtml
 */
class SRF08 {

public:

    /**
     * Create a SRF08 object connected to the specified I2C pins and address
     *
     * @param SDA I2C SDA pin to connect to
     * @param SCL I2C SCL pin to connect to
     * @param i2cAddress Address of WSRF08 on I2C bus
     */
    SRF08(PinName SDA, PinName SCL, int i2cAddress);
    
    /**
     * Send the "Start ranging in cm" command via I2C
     */
    void startRanging();
    
    /**
     * Checks if the module has finished ranging
     *
     * @param returns Boolean stating module is finished or not
     */
    bool rangingFinished();
    
    /**
     * Gets the measured range from the module
     *
     * @param returns Integer range in centimetre
     */
    int getRange();
    
    /**
     * Gets the measured light intensity from the module
     *
     * @param returns A normalised number 0-255 representing dark to light
     */
     int getLightIntensity();
     
    /**
     * Sets the range register of the SRF08 for faster ranging.
     * 
     * The max range is ((rangeVal x 43mm) + 43mm). The sensors maximum range
     *  is about six metres
     *
     * @param rangeVal The value written to the range register of the SRF08
     */
    void setRangeRegister(unsigned char rangeVal);
     
    /**
     * Sets the max gain register of the SRF08.
     * 
     * @param gainVal The value written to the max gain register of the SRF08
     */
    void setMaxGainRegister(unsigned char gainVal);
    
    /**
     * Changes the I2C address of the SRF08.
     *
     * The factory default address is 0x0E (224)
     *  The address can have the following values:
     *   E0 | E2 | E4 | E6 ... FC | FE
     * 
     * @param i2cAddress The new I2C address for the SRF08. 
     */
    void setAddress(int i2cAddress);
     
protected:

    I2C i2cMod;
    unsigned char i2cAddress;
    Timeout rangeTimeout;
    bool rangingBusy;
    
    void setRangingFinished();
};

#endif