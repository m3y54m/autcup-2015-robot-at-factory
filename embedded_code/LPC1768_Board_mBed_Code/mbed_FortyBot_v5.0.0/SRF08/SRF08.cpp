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

#include "SRF08.h"

/*
 * Constructor: SRF08
 * Args:
 * Returns:     void
 * Description: Creates an instance of the SRF08 to communicate with a sRF08 module
 */
SRF08::SRF08(PinName SDA, PinName SCL, int i2cAddress) :
        i2cMod(SDA, SCL), 
        i2cAddress(i2cAddress), 
        rangeTimeout(), 
        rangingBusy(false) {
			
		i2cMod.frequency(400000); // fast i2c: 400 kHz
}

/*
 * Function:    startRanging
 * Args:        void
 * Returns:     void
 * Description: Sends command to module to start ranging.
 */
void SRF08::startRanging() {
    //Create a two byte command. The first first byte is the register address
    // on the SRF08 to write to. The second byte is the command which is written
    // to that address ("Start ranging in cm" in this case).
    const char command[] = {0x00, 0x51};
    i2cMod.write(i2cAddress, command, 2);
    this->rangingBusy = true;
    rangeTimeout.attach(this, &SRF08::setRangingFinished, 0.07);
}

/*
 * Function:    rangingFinished
 * Args:        void
 * Returns:     Bool: whether ranging is finished
 * Description: Checks if the ranging process on the module is finished
 */
bool SRF08::rangingFinished() {
    if(this->rangingBusy) return false;
    return true;
}

/*
 * Function:    getRange
 * Args:        void
 * Returns:     int range
 * Description: Range in cm. This function should only be called when ranging is finished, otherwise previous value is returned
 */
int SRF08::getRange() {
    //while (!rangingFinished() ) wait(0.01);   //Wait until ranging is finished
    const char command[]  = {0x02};           //Address of range register
    char response[] = {0x00, 0x00};
    i2cMod.write(i2cAddress, command, 1, 1);  //Send command
    i2cMod.read(i2cAddress, response, 2);     //Read 16bits result
    int range = (response[0]<<8)+response[1]; //Shift two bytes into int
    return range;                             //Return int range
}

/*
 * Function:    readLightIntensity
 * Args:        void
 * Returns:     int lightIntensity
 * Description: Reads the lightIntensity from the module
 *              The light intensity is updated if a range command is sent, so don't use
 *              this function only
 */
int SRF08::getLightIntensity() {
    const char command[] = {0x01};           //Light intensity register
    char response[] = {0x00};
    i2cMod.write(i2cAddress, command, 1, 1); //Send command
    i2cMod.read(i2cAddress, response, 1);    //Read response
    int lightIntensity = response[0];
    return lightIntensity;
}

/*
 * Function:    setRangeRegister
 * Args:        rangeVal
 * Returns:     void
 * Description: Sets the maximum range for which the module waits for an echo
 *              The range is ((rangeVal x 43mm) + 43mm)
 *              The max range is about six meters
 */
void SRF08::setRangeRegister(unsigned char rangeVal) {
    while (!rangingFinished() ) wait(0.01);  //Wait until ranging is finished
    char command[] = {0x02, rangeVal};       //Range register
    i2cMod.write(i2cAddress, command, 2);    //Send command
}


//Function setMaxGainRegister
void SRF08::setMaxGainRegister(unsigned char gainVal) {
    while (!rangingFinished() ) wait(0.01);  //Wait until ranging is finished
    char command[] = {0x01, gainVal};        //Max gain register
    i2cMod.write(i2cAddress, command, 2);    //Send command
}

/*
 * Function:    setAddress
 * Args:        address
 * Returns:     void
 * Description: Sets the address of the module on the I2C bus. The factory default address is 0x0E (224)
 *                  The address can have the following values:
 *                      E0 | E2 | E4 | E6 ... FC | FE
 */
void SRF08::setAddress(int address) {
    //Send address change sequence
    char command[] = {0x00, 0xA0};
    i2cMod.write(i2cAddress, command, 2);
    command[1] = 0xAA;
    i2cMod.write(i2cAddress, command, 2);
    command[1] = 0xA5;
    i2cMod.write(i2cAddress, command, 2);
    command[1] = address;
    i2cMod.write(i2cAddress, command, 2);
    //Save the updated address
    i2cAddress = address;
}

//Small helper function for Timeout object
void SRF08::setRangingFinished() {
    this->rangingBusy = false;
}