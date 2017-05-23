/*
 * speedcalc.c
 *
 *  Created on: 7 Nov 2016
 *      Author: rik
 */

#include "stm32f3xx_hal.h"
#include "speedcalc.h"
#include "commsfpga.h"
#include <math.h>
#include <myNRF24.h>
#define _POLE_PAIRS 8						// Amount of EL rotations for 1 mechanical rotation
#define _HALL_C_PER_ELROT 6			// Amount of changes in hall effect sensors for 1 EL rotation
#define _MECH_REDUCTION 3 			// Amount of mechanical gear reduction 1:M
#define _FPGA_CLOCK_F 1000000 	// Clock frequency of the FPGA used to count the time for a hall effect change
//#define _a1  -1.0471975512f
//#define _a3  1.0471975512f
//#define _a4  2.09439510239f
//#define _a2  -2.09439510239f
const float _a0 = 60 * 3.1415/180.0; //240
const float _a1 = 120 * 3.1415/180.0; //300
const float _a2 = 240  * 3.1415/180.0; //60
const float _a3 = 300 * 3.1415/180.0; //120
#define _R   0.09f
#define _r   0.0275f
#define PI 3.1415f

void splitVector(float magnitude, float direction, float* xComponent, float* yComponent){
	magnitude = magnitude / 1000; //from mm/s to m/s;

	direction = direction * (2*M_PI/512);;
	float cosDir = cos(direction);
	float sinDir = sin(direction);

	//sprintf(smallStrBuffer, "mag: %f;   cosDir: %f;   sinDir: %f;\n", magnitude, cosDir, sinDir);
	//TextOut(smallStrBuffer);

	*xComponent = cosDir*magnitude;
	*yComponent = sinDir*magnitude;
}

void calcMotorRaw(wheelVelocityPacket* calcPacket, float* prevWheelCommand, float vx, float vy, uint16_t w, uint8_t rotDir){
	float wRadPerSec = (w/180.0)*PI;

	float accStep = 1.0;
	float accSlowlyUntil = 15.0;

	float wheelScalar = 1/_r;
	float angularComponent;
	float speedmotor[4];
	int rotSign;

	if(rotDir != 0){
		rotSign = -1;
	}
	else{
		rotSign = 1;
	}

	angularComponent = rotSign*_R*wRadPerSec;

	speedmotor[0] = (-cos(_a0)*vy*1.4 + sin(_a0)*vx + angularComponent)*wheelScalar;
	if ((fabs(speedmotor[0]) - fabs(prevWheelCommand[0])) >= accStep && fabs(prevWheelCommand[0]) <= accSlowlyUntil) {
		int signSpeed = speedmotor[0] / fabs(speedmotor[0]);
		speedmotor[0] = prevWheelCommand[0] + accStep * signSpeed;
	}
	prevWheelCommand[0] = speedmotor[0];
	calcPacket->velocityWheel1=calcFPGAFromRPS(speedmotor[0] / (2*PI));

	speedmotor[1] = (-cos(_a1)*vy *1.4+ sin(_a1)*vx + angularComponent)*wheelScalar;
	if ((fabs(speedmotor[1]) - fabs(prevWheelCommand[1])) >= accStep && fabs(prevWheelCommand[1]) <= accSlowlyUntil) {
		int signSpeed = speedmotor[1] / fabs(speedmotor[1]);
		speedmotor[1] = prevWheelCommand[1] + accStep * signSpeed;
	}
	prevWheelCommand[1] = speedmotor[1];
	calcPacket->velocityWheel2=calcFPGAFromRPS(speedmotor[1] / (2*PI));

	speedmotor[2] = (-cos(_a2)*vy*1.4 + sin(_a2)*vx + angularComponent)*wheelScalar;
	if ((fabs(speedmotor[2]) - fabs(prevWheelCommand[2])) >= accStep && fabs(prevWheelCommand[2]) <= accSlowlyUntil) {
		int signSpeed = speedmotor[2] / fabs(speedmotor[2]);
		speedmotor[2] = prevWheelCommand[2] + accStep * signSpeed;
	}
	prevWheelCommand[2] = speedmotor[2];
	calcPacket->velocityWheel3=calcFPGAFromRPS(speedmotor[2] / (2*PI));

	speedmotor[3] = (-cos(_a3)*vy*1.4 + sin(_a3)*vx + angularComponent)*wheelScalar;
	if ((fabs(speedmotor[3]) - fabs(prevWheelCommand[3])) >= accStep && fabs(prevWheelCommand[3]) <= accSlowlyUntil) {
		int signSpeed = speedmotor[3] / fabs(speedmotor[3]);
		speedmotor[3] = prevWheelCommand[3] + accStep * signSpeed;
	}
    prevWheelCommand[3] = speedmotor[3];
    calcPacket->velocityWheel4=calcFPGAFromRPS(speedmotor[3] / (2*PI));

//    sprintf(smallStrBuffer, "wheelcommands: %f %f %f %f\n", speedmotor[0], speedmotor[1], speedmotor[2], speedmotor[3]);
//    TextOut(smallStrBuffer);

}
void calcMotorStefan(dataPacket *dataStruct, wheelVelocityPacket *PacketSpeed){
//	float xSpeed=0;
//	float ySpeed=0;
//	splitVector((float)dataStruct->robotVelocity, (float)dataStruct->movingDirection, &xSpeed, &ySpeed);
//	float wheelScalar = 1/_r;
//	int angularComponent = _R*dataStruct->movingDirection;
//	float speedmotor[4];
//	speedmotor[0] = (-1/sin(_a1)*xSpeed + -1/cos(_a1)*ySpeed + angularComponent)*wheelScalar;
//	PacketSpeed->velocityWheel1=calcFPGAFromRPS(speedmotor[0]);
//    speedmotor[1] = -1*((-1/sin(_a2)*xSpeed + -1/cos(_a2)*ySpeed + angularComponent)*wheelScalar);
//	PacketSpeed->velocityWheel2=calcFPGAFromRPS(speedmotor[1]);
//	speedmotor[2] = (-1/sin(_a3)*xSpeed + -1/cos(_a3)*ySpeed + angularComponent)*wheelScalar;
//	PacketSpeed->velocityWheel3=calcFPGAFromRPS(speedmotor[2]);
//	speedmotor[3] = -1*((-1/sin(_a4)*xSpeed + -1/cos(_a4)*ySpeed + angularComponent)*wheelScalar);
//	PacketSpeed->velocityWheel4=calcFPGAFromRPS(speedmotor[3]);
}

void calcMotorSpeed (dataPacket *dataStruct, wheelVelocityPacket *packetSpeed, float *prevWheelCommand){
	//see the paint file for clarification
	float xSpeed=0;
	float ySpeed=0;



	splitVector((float)dataStruct->robotVelocity, (float)dataStruct->movingDirection, &xSpeed, &ySpeed);
	calcMotorRaw(packetSpeed, prevWheelCommand, xSpeed, ySpeed,  dataStruct->angularVelocity, dataStruct->rotationDirection);
}

float calcRPSFromFGPA(int32_t iFPGASpeed){
	return ((_FPGA_CLOCK_F )/(_POLE_PAIRS * _HALL_C_PER_ELROT * iFPGASpeed * _MECH_REDUCTION));
};

float calcRPMFromFGPA(int32_t iFPGASpeed){
	return ((60 * _FPGA_CLOCK_F )/(_POLE_PAIRS * _HALL_C_PER_ELROT * iFPGASpeed * _MECH_REDUCTION));
};

int32_t calcFPGAFromRPS(float RPS){
    return (abs((_FPGA_CLOCK_F)/(_POLE_PAIRS * _HALL_C_PER_ELROT * _MECH_REDUCTION * RPS))>1000000 ? 0 : (_FPGA_CLOCK_F)/(_POLE_PAIRS * _HALL_C_PER_ELROT * _MECH_REDUCTION * RPS));
  };

int32_t calcFPGAFromRPM(float RPM){
    return (abs((_FPGA_CLOCK_F)/(_POLE_PAIRS * _HALL_C_PER_ELROT * _MECH_REDUCTION * RPM/60))>1000000 ? 0 : (_FPGA_CLOCK_F)/(_POLE_PAIRS * _HALL_C_PER_ELROT * _MECH_REDUCTION * RPM/60));
};

