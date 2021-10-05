/*
 * FortyBot 4WD Mecanum Robot Controller
 * Developed By:          Meysam Parvizi
 * Date:                  1394/11/20
 * Version:               3.0.3
 * Description: Mecanum Drive Test
 * 				Removed HMC5883L Magnetometer from Calculations Beacause of Huge Error
 *				Added Gysoscope Wz
 */

#include "mbed.h"
#include "HMC5883L.h"
#include "MPU6050.h"
#include "SRF08.h"
#include "PID.h"

Serial serial(p13, p14);  // tx, rx

// Read Encoder Data
I2C i2c(p28, p27);

#define length 44.5F // Robot Length 
#define width 30.5F  // Robot Width

/*  SRF08 sensors position 
 *  
 *   w1 -S1--S2- w2
 *   |		A    |
 *  S8	 +x |    S3
 *   |		|    |
 *   | <-------- |
 *  S7  +y  |    S4
 *   |		     |
 *   w3 -S6--S5- w4
 *	    
 */ 
//SRF08: Define SDA, SCL pin and I2C address
SRF08 distance1(p28, p27, 0xE0);
SRF08 distance2(p28, p27, 0xE2);
SRF08 distance3(p28, p27, 0xE4);
SRF08 distance4(p28, p27, 0xE6);
SRF08 distance5(p28, p27, 0xEA); 
SRF08 distance6(p28, p27, 0xEC); 
SRF08 distance7(p28, p27, 0xEE); 
SRF08 distance8(p28, p27, 0xF0);

#define srfBias -3

#define FORWARD 0
#define BACKWARD 1
#define BRAKE_VCC 2
#define BRAKE_GND 3
//#define CS_THRESHOLD 0.5f

/*  VNH2SP30 pin definitions */
DigitalOut in1A(p11, 0);  	// INA: Clockwise input
DigitalOut in2A(p17, 0); 	// INA: Clockwise input
DigitalOut in1B(p12, 0); 	// INB: Counter-clockwise input
DigitalOut in2B(p18, 0); 	// INB: Counter-clockwise input
PwmOut pwm1(p21); 			// PWM input
PwmOut pwm2(p22); 			// PWM input

/*  VNH2SP30 pin definitions */
DigitalOut in3A(p25, 0);  	// INA: Clockwise input
DigitalOut in4A(p29, 0);  	// INA: Clockwise input
DigitalOut in3B(p26, 0); 	// INB: Counter-clockwise input
DigitalOut in4B(p30, 0); 	// INB: Counter-clockwise input
PwmOut pwm3(p23); 			// PWM input
PwmOut pwm4(p24); 			// PWM input

MPU6050 mpu6050;           
int16_t gyroOutput[3];
extern float gRes;

Ticker encoders;
Ticker gyro;
Ticker srf08;
Ticker pid;

int goalSpeed[4] = {0, 0, 0, 0};

int actualDistance[8];
int actualSpeed[4];

float desiredVx;
float desiredVy;
float desiredWz;

float actualVx;
float actualVy;

float odometryWz;
float gyroWz;
float actualWz;

float actualX;
float actualY;

float gyroAngle;
float odometryAngle;
float actualAngle;

#define RATE  0.04
#define Kc	0.035
#define Ti	0.1
#define Td	0.0

PID controlWheel1(Kc, Ti, Td, RATE);
PID controlWheel2(Kc, Ti, Td, RATE);
PID controlWheel3(Kc, Ti, Td, RATE);
PID controlWheel4(Kc, Ti, Td, RATE);

PID controlVx(Kc, Ti, Td, RATE);
PID controlVy(Kc, Ti, Td, RATE);
PID controlWz(Kc, Ti, Td, RATE);

void motorDrive(char motor, float pwm);
void pidInit(void);
void navigateRobot(void);
void omniMove(float vX, float vY, float wZ);
signed char getWheelSpeed(char wheelID);
void setWheelSpeed(char wheelID, signed char speed);
void srf08Init(void);
void getEncoders(void);
void getGyroYaw(void);
void getDistances(void);
void pidFourWheels(void);
float pidComputeVx(void);
float pidComputeVy(void);
float pidComputeWz(void);
void complementaryFilter(float a);
unsigned char rxByte = 0;
void serialRxInterrupt(void);
bool collision(int i, int distance);

float pitchAngle = 0;
float rollAngle = 0;
float yawAngle = 0;

int main() 
{
	serial.baud(115200);                           // baud rate: 115200
	serial.attach(&serialRxInterrupt, Serial::RxIrq);
	
	i2c.frequency(400000);						   // fast i2c: 400 kHz
	
	//mpu6050.whoAmI();                              // Communication test: WHO_AM_I register reading 
	mpu6050.calibrate(accelBias,gyroBias);         // Calibrate MPU6050 and load biases into bias registers
	
	mpu6050.init();                                // Initialize the sensors
	srf08Init();
	pidInit();
	
	gyro.attach(&getGyroYaw,0.005);			// Call the getPosition func. every 5 ms (200 Hz processing period)
	srf08.attach(&getDistances,0.07);				// Call the getEncoders func. every 70 ms (it works as delay for sensors data to get ready). 70 ms is the minimum delay
	pid.attach(&navigateRobot, 0.04);				// Call the pidWheels func. every 40 ms (25 Hz processing period)
	
	omniMove(0, 0, 30);
	
	while(1) 
	{
//	    serial.printf("%.1f\r\n",actualAngle);
//		serial.printf("%.1f, %.1f, %.1f\r\n",actualAngle, actualX ,actualY);
		serial.printf("%.1f, %.1f :: %.1f\r\n",gyroWz, odometryWz, actualAngle);		
//		serial.printf("             %d,%d,%d,%d\r\n",actualSpeed[0],actualSpeed[1],actualSpeed[2],actualSpeed[3]);
//		serial.printf("%d,%d,%d,%d,%d,%d,%d,%d\r\n",actualDistance[0],actualDistance[1],actualDistance[2],actualDistance[3],actualDistance[4],actualDistance[5],actualDistance[6],actualDistance[7]);
		wait_ms(40);
	}
}

bool collision(int i, int distance)
{
	return (actualDistance[i] < distance);
}

void serialRxInterrupt(void)
{
	LPC_UART1->IER = 0;        // Temporary Disable RX Interrupt (to avoid hangig up)
    rxByte = LPC_UART1->RBR;   // Read Receiver Buffer Register
	
	/********** processing the received byte **********/
	
	
	/**************************************************/
		
	LPC_UART1->IER = 1;			// Re-enable RX Interrupt
}

void motorDrive(char motor, float pwm)
{
	switch(motor)
	{
		case 1:
			
			if (pwm > 0)        // Motor 1 Forward
			{
				in1A = 1;	    // INA1 = 1
				in1B = 0;	    // INB1 = 0
				pwm1.write(pwm);
			}
			else if (pwm < 0)   // Motor 1 Backward
			{
				in1A = 0;	    // INA1 = 0
				in1B = 1;	    // INB1 = 1
				pwm1.write(-pwm);
			}
			else				// Motor 1 Brake to VCC 
			{
				in1A = 1;	    // INA1 = 1
				in1B = 1;	    // INB1 = 1
				pwm1.write(0);
			}
			
			break;
			
		case 2:
			
			if (pwm > 0)        // Motor 2 Forward
			{
				in2A = 1;	    // INA2 = 1
				in2B = 0;	    // INB2 = 0
				pwm2.write(pwm);
			}
			else if (pwm < 0)   // Motor 2 Backward
			{
				in2A = 0;	    // INA2 = 0
				in2B = 1;	    // INB2 = 1
				pwm2.write(-pwm);
			}
			else				// Motor 2 Brake to VCC 
			{
				in2A = 1;	    // INA2 = 1
				in2B = 1;	    // INB2 = 1
				pwm2.write(0);
			}
			
			break;
			
		case 3:
		
			if (pwm > 0)        // Motor 3 Forward
			{
				in3A = 1;	    // INA3 = 1
				in3B = 0;	    // INB3 = 0
				pwm3.write(pwm);
			}
			else if (pwm < 0)   // Motor 3 Backward
			{
				in3A = 0;	    // INA3 = 0
				in3B = 1;	    // INB3 = 1
				pwm3.write(-pwm);
			}
			else				// Motor 3 Brake to VCC 
			{
				in3A = 1;	    // INA3 = 1
				in3B = 1;	    // INB3 = 1
				pwm3.write(0);
			}
			
			break;
			
		case 4:
		
			if (pwm > 0)        // Motor 4 Forward
			{
				in4A = 1;	    // INA4 = 1
				in4B = 0;	    // INB4 = 0
				pwm4.write(pwm);
			}
			else if (pwm < 0)   // Motor 4 Backward
			{
				in4A = 0;	    // INA4 = 0
				in4B = 1;	    // INB4 = 1
				pwm4.write(-pwm);
			}
			else				// Motor 4 Brake to VCC 
			{
				in4A = 1;	    // INA4 = 1
				in4B = 1;	    // INB4 = 1
				pwm4.write(0);
			}

			break;	  
	}
}

void pidInit(void)
{
	//Encoder input from -127 to 127
	controlWheel1.setInputLimits(-127, 127);
	controlWheel2.setInputLimits(-127, 127);
	controlWheel3.setInputLimits(-127, 127);
	controlWheel4.setInputLimits(-127, 127);
	
	//Pwm output from -1.0 to 1.0
	controlWheel1.setOutputLimits(-1.0, 1.0);
	controlWheel2.setOutputLimits(-1.0, 1.0);
	controlWheel3.setOutputLimits(-1.0, 1.0);
	controlWheel4.setOutputLimits(-1.0, 1.0);
	
	//If there's a bias.
	controlWheel1.setBias(0);
	controlWheel2.setBias(0);
	controlWheel3.setBias(0);
	controlWheel4.setBias(0);
	
	controlWheel1.setMode(AUTO_MODE);
	controlWheel2.setMode(AUTO_MODE);
	controlWheel3.setMode(AUTO_MODE);
	controlWheel4.setMode(AUTO_MODE);
	
	// Mecanum wheels parameters
	controlVx.setInputLimits(-100, 100);
	controlVy.setInputLimits(-100, 100);
	controlWz.setInputLimits(-100, 100);
	
	controlVx.setOutputLimits(-100, 100);
	controlVy.setOutputLimits(-100, 100);
	controlWz.setOutputLimits(-100, 100);
	
	//If there's a bias.
	controlVx.setBias(0);
	controlVy.setBias(0);
	controlWz.setBias(0);
	
	controlVx.setMode(AUTO_MODE);
	controlVy.setMode(AUTO_MODE);
	controlWz.setMode(AUTO_MODE);
}

void navigateRobot(void)
{
	getEncoders();
	
	pidFourWheels();
	
//	omniMove(pidComputeVx(), pidComputeVy(), pidComputeWz());
}

void pidFourWheels(void)
{
	controlWheel1.setSetPoint(goalSpeed[0]);
	//Update the process variable.
	controlWheel1.setProcessValue(actualSpeed[0]);
	//Set the new output.
	motorDrive(1, controlWheel1.compute());
	
	controlWheel2.setSetPoint(goalSpeed[1]);
	//Update the process variable.
	controlWheel2.setProcessValue(actualSpeed[1]);
	//Set the new output.
	motorDrive(2, controlWheel2.compute());
	
	controlWheel3.setSetPoint(goalSpeed[2]);
	//Update the process variable.
	controlWheel3.setProcessValue(actualSpeed[2]);
	//Set the new output.
	motorDrive(3, controlWheel3.compute());
	
	controlWheel4.setSetPoint(goalSpeed[3]);
	//Update the process variable.
	controlWheel4.setProcessValue(actualSpeed[3]);
	//Set the new output.
	motorDrive(4, controlWheel4.compute());
}

float pidComputeVx(void)
{
	controlVx.setSetPoint(desiredVx);
	//Update the process variable.
	controlVx.setProcessValue(actualVx);
	//Set the new output.
	return controlVx.compute();
}

float pidComputeVy(void)
{
	controlVy.setSetPoint(desiredVy);
	//Update the process variable.
	controlVy.setProcessValue(actualVy);
	//Set the new output.
	return controlVy.compute();
}

float pidComputeWz(void)
{
	controlWz.setSetPoint(desiredWz);
	//Update the process variable.
	controlWz.setProcessValue(actualWz);
	//Set the new output.
	return controlWz.compute();
}

/* Get all quadrature encoders data */
void getEncoders(void)
{
	actualSpeed[0] = getWheelSpeed(1);
	actualSpeed[1] = -getWheelSpeed(2);
	actualSpeed[2] = getWheelSpeed(3);
	actualSpeed[3] = -getWheelSpeed(4);
	
	int w1 = actualSpeed[0];
	int	w2 = actualSpeed[1];
	int	w3 = actualSpeed[2];
	int	w4 = actualSpeed[3];
	
	odometryWz = (float)( -w1 + w2 - w3 + w4 ) / ( 2 * ( length + width ) );
	actualVx = (float)(  w1 + w2 + w3 + w4 ) / 4;
	actualVy = (float)( -w1 + w2 + w3 - w4 ) / 4;
	
	odometryWz = odometryWz * 18.86298121f;
	actualVx = actualVx * 0.3333333f;
	actualVy = actualVy * 0.3333333f;	
	
	// Integrate the odometry data over time to get X, Y and Angle
	// dt = 0.04 <=> 25 Hz sampling period	
	odometryAngle = odometryAngle + odometryWz  * 0.04;  													
	actualX += ( actualVx * cos(odometryAngle * (PI / 180)) + actualVy * sin(odometryAngle * (PI / 180)) ) * 0.04;
	actualY += ( actualVy * cos(odometryAngle * (PI / 180)) + actualVx * sin(odometryAngle * (PI / 180)) ) * 0.04;

	complementaryFilter(0.8);
}

void getGyroYaw(void)
{
	mpu6050.readGyroData(gyroOutput);
	mpu6050.getGres();
	gyroWz = gyroOutput[2] * gRes;
	
	// Integrate the gyro data(deg/s) over time to get angle
	gyroAngle = gyroAngle + gyroWz * dt;        // dt = 0.005 <=> 200 Hz sampling period	
}

void complementaryFilter(float a)
{
	if (a > 1.0f) a = 1.0f;
	if (a < 0) a = 0;
	
	actualAngle = a * gyroAngle + (1 - a) * odometryAngle;
	actualWz = a * gyroWz + (1 - a) * odometryWz;
}

/*  Omnidirectional Mecanum 4WD Robot Navigation 
 *  
 *  w1 -------- w2
 *  |		A   |
 *  |	 +x |   |
 *  |		|   | Length
 *  | <-------- |
 *  | +y    |   |
 *  |		    |
 *  w3 -------- w4
 *	    Width
 */ 
void omniMove(float vX, float vY, float wZ)  // vX: cm/s , vY: cm/s , wZ: deg/s
{
	float vx;
	float vy;
	float omega;
	
	vx = vX * 3.014893f;    	// optimized for 20 cm/s
	vy = vY * 3.052253f;    	// optimized for 20 cm/s
	omega = wZ * 0.05236875f;	// optimized for 30 deg/s
	
	int	w1 = 0;
	int	w2 = 0;
	int	w3 = 0;
	int	w4 = 0;
	
	w1 = (int)( vx - vy - omega * ((length + width) / 2));
	w2 = (int)( vx + vy + omega * ((length + width) / 2));
	w3 = (int)( vx + vy - omega * ((length + width) / 2));
	w4 = (int)( vx - vy + omega * ((length + width) / 2)); 
	
	if (w1 > 127) w1 = 127;
	if (w2 > 127) w2 = 127;
	if (w3 > 127) w3 = 127;
	if (w4 > 127) w4 = 127;
	
	if (w1 < -127) w1 = -127;
	if (w2 < -127) w2 = -127;
	if (w3 < -127) w3 = -127;
	if (w4 < -127) w4 = -127;

	setWheelSpeed(1, w1);
	setWheelSpeed(2, w2);
	setWheelSpeed(3, w3);
	setWheelSpeed(4, w4);
}

/* Get actual wheel speed */
signed char getWheelSpeed(char wheelID)
{
	const char command[] = { 0x01 };
	char response[] = { 0x00 };
	static int last_speed[4];
	signed char speed;
		
	char addr = wheelID << 1;
		
	i2c.write(addr, command, 1, 1); 	//Send command & repeated start
	i2c.read(addr, response, 1);		//Read response
	
	if ( (signed char)response[0] == -128)
		speed = last_speed[wheelID-1];
	else {
		speed = (signed char)response[0];
		last_speed[wheelID-1] = speed;
	}
		
	//max speed is about 125
	return speed;
}

/* Set desired wheel speed */
void setWheelSpeed(char wheelID, signed char speed)
{
	if ((wheelID == 1) || (wheelID == 2) || (wheelID == 3) || (wheelID == 4))
	{
		goalSpeed[wheelID-1] = speed;
	}
}

/* SRF08 ultrasonic sensors initialization */
void srf08Init()
{
	// The range is ((Range Register x 43mm) + 43mm) so setting the Range Register to 0 (0x00) gives
	//	a maximum range of 43mm. Setting the Range Register to 1 (0x01) gives a maximum range of 86mm.
	//	More usefully, 24 (0x18) gives a range of 1 metre and 140 (0x8C) is 6 metres. Setting 255 (0xFF)
	//	gives the original 11 metres (255 x 43 + 43 is 11008mm).
	
	const unsigned char srf08Range = 0x18; // 1 meters
	const unsigned char srf08Gain = 0x02;  // Set Maximum Analogue Gain to 100
	
	distance1.setRangeRegister(srf08Range);	
	distance1.setMaxGainRegister(srf08Gain);
	
	distance2.setRangeRegister(srf08Range);	
	distance2.setMaxGainRegister(srf08Gain);
	
	distance3.setRangeRegister(srf08Range);	
	distance4.setMaxGainRegister(srf08Gain);
	
	distance4.setRangeRegister(srf08Range);	
	distance4.setMaxGainRegister(srf08Gain);
	
	distance5.setRangeRegister(srf08Range);
	distance5.setMaxGainRegister(srf08Gain);
	
	distance6.setRangeRegister(srf08Range);	
	distance6.setMaxGainRegister(srf08Gain);
	
	distance7.setRangeRegister(srf08Range);	
	distance7.setMaxGainRegister(srf08Gain);
	
	distance8.setRangeRegister(srf08Range);	
	distance8.setMaxGainRegister(srf08Gain);
}

/* Get all SRF08 ultrasonic sensors data */
void getDistances()
{
	static char step = 0;
	
	switch (step)
	{
		case 0:
			distance1.startRanging();
			distance3.startRanging();
			distance5.startRanging();
			distance7.startRanging();
			break;
		
		case 1:
			actualDistance[0] = distance1.getRange() + srfBias;
			actualDistance[2] = distance3.getRange() + srfBias;
			actualDistance[4] = distance5.getRange() + srfBias;
			actualDistance[6] = distance7.getRange() + srfBias;	
		
			distance2.startRanging();
			distance4.startRanging();
			distance6.startRanging();
			distance8.startRanging();
			break;
		
		case 2:
			actualDistance[1] = distance2.getRange() + srfBias;
			actualDistance[3] = distance4.getRange() + srfBias;
			actualDistance[5] = distance6.getRange() + srfBias;	
			actualDistance[7] = distance8.getRange() + srfBias;
		
			distance1.startRanging();
			distance3.startRanging();
			distance5.startRanging();
			distance7.startRanging();
			break;		
	}
	
	step++;
	if (step == 3) step = 1;		
}