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

void calcMotorRaw(wheelVelocityPacket* calcPacket, float vx, float vy, uint16_t w, uint8_t rotDir){
	float wRadPerSec = (w/180.0)*PI;

	float vxMPerSec = vx;
	float vyMPerSec = vy;

	sprintf(smallStrBuffer, "vxMPerSec:  %f,   vyMPerSec:  %f\n", vxMPerSec, vyMPerSec);
	TextOut(smallStrBuffer);

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

	//x positive = forward
	//y positive = left


	angularComponent = rotSign*_R*wRadPerSec;

	speedmotor[0] = (-cos(_a0)*vy + sin(_a0)*vx + angularComponent)*wheelScalar;
	calcPacket->velocityWheel1=calcFPGAFromRPS(speedmotor[0] / (2*PI));
	speedmotor[1] = (-cos(_a1)*vy + sin(_a1)*vx + angularComponent)*wheelScalar;
	calcPacket->velocityWheel2=calcFPGAFromRPS(speedmotor[1] / (2*PI));
	speedmotor[2] = (-cos(_a2)*vy + sin(_a2)*vx + angularComponent)*wheelScalar;
	calcPacket->velocityWheel3=calcFPGAFromRPS(speedmotor[2] / (2*PI));
    speedmotor[3] = (-cos(_a3)*vy + sin(_a3)*vx + angularComponent)*wheelScalar;
    calcPacket->velocityWheel4=calcFPGAFromRPS(speedmotor[3] / (2*PI));


	/*for(int i = 0; i < 4; i++){
		sprintf(smallStrBuffer, "speed %i: %f;\n", i, speedmotor[i] *_r);
		TextOut(smallStrBuffer);
	}*/

	/*
	speedmotor[0] = (-1/sin(_a1)*vx + -1/cos(_a1)*vy + angularComponent)*wheelScalar;
	calcPacket->velocityWheel1=calcFPGAFromRPS(speedmotor[0]);
    speedmotor[1] = -1*((-1/sin(_a2)*vx + -1/cos(_a2)*vy + angularComponent)*wheelScalar);
	calcPacket->velocityWheel2=calcFPGAFromRPS(speedmotor[1]);
	speedmotor[2] = (-1/sin(_a3)*vx + -1/cos(_a3)*vy + angularComponent)*wheelScalar;
	calcPacket->velocityWheel3=calcFPGAFromRPS(speedmotor[2]);
	speedmotor[3] = -1*((-1/sin(_a4)*vx + -1/cos(_a4)*vy + angularComponent)*wheelScalar);
	calcPacket->velocityWheel4=calcFPGAFromRPS(speedmotor[3]);*/

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

void calcMotorSpeed (dataPacket *dataStruct, wheelVelocityPacket *packetSpeed){
	//see the paint file for clarification
	float xSpeed=0;
	float ySpeed=0;



	splitVector((float)dataStruct->robotVelocity, (float)dataStruct->movingDirection, &xSpeed, &ySpeed);
	calcMotorRaw(packetSpeed, xSpeed, ySpeed,  dataStruct->angularVelocity, dataStruct->rotationDirection);
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

