/*
 * TextOut.c
 *
 *  Created on: 14 sep. 2016
 *      Author: Hans-van-der-Heide
 */

#include "stm32f3xx_hal.h"
#include <string.h>
#include <stdio.h>
#include "TextOut.h"


char smallStrBuffer[1024];

void backslashNfixer(char* str){
	/*int length = strlen(str);
	char* backslashNpointer = strchr(str, '\n');
	int backslashNposition = (int)(backslashNpointer - str);
	if(backslashNpointer != NULL){


		for(int i = length; i > backslashNposition; i--){
			smallStrBuffer[i+1] = str[i];
		}
		smallStrBuffer[backslashNposition] = '\r';
		backslashNfixer(smallStrBuffer);
		smallStrBuffer[backslashNposition + 1] = '\n';
		for(int i = 0; i < backslashNposition; i++){
			smallStrBuffer[i] = str[i];
		}
	}*/
	int length = strlen(str);
	char* backslashNpointer = strchr(str, '\n');
	memcpy(smallStrBuffer, str, length);
	if(backslashNpointer != NULL){
		smallStrBuffer[length] = '\r';
		smallStrBuffer[length+1] = '\0';
	}



}

void TextOut(char *str){

	backslashNfixer(str);
	//memcpy(smallStrBuffer, str, strlen(str));
	int length = strlen(smallStrBuffer);
	CDC_Transmit_FS(smallStrBuffer, length);
	HAL_Delay(1);
}

void HexOut(uint8_t data[], uint8_t length){

	CDC_Transmit_FS(data, length);
}

/*
void TextOut(char *str)
{
	unsigned char sendChar;
	int i = 0;
	while(str[i] != '&'){

		if(str[i] == '\n'){
			sendChar = '\r';
			sendOneChar(&sendChar);
		}
	    sendChar = str[i];
	    sendOneChar(&sendChar);
	    i++;
    }
}

void TextOutInt(char *str, int interger)
{
	unsigned char sendChar;
	int i = 0;
	while(str[i] != '&'){

		if(str[i]=='\n'){
			sendChar = '\r';
			sendOneChar(&sendChar);
		}
		if(str[i]=='%'){
			//determine the number of digits
			if(interger == 0){
				sendChar = '0';
				sendOneChar(&sendChar);
			}
			else{
				int length = 0;
				double intCopy = interger;
				while(intCopy >= 1){
					intCopy = intCopy / 10;
					length++;
				}

				while (length > 0){
					int devider = 1;
					int digit;
					for(int i = 0; i < (length - 1); i++){
						devider = devider*10;
					}
					digit = interger / devider;
					sendChar = digit + '0';
					sendOneChar(&sendChar);
					interger -= (digit*devider);
					length--;
				}
			}

		}
		else {
			sendChar = str[i];
			sendOneChar(&sendChar);
		}
		i++;
	}
}

void TextOutHex(char *str, int interger)
{
	unsigned char sendChar;
	int i = 0;
	while(str[i] != '&'){
		char currentChar = str[i];
		if(str[i]=='\n'){
			sendChar = '\r';
			sendOneChar(&sendChar);
		}
		if(str[i] == '%'){
			//determine the number of digits
			if(interger == 0){
				sendChar = '0';
				sendOneChar(&sendChar);
			}
			else{
				int length = 0;
				double intCopy = interger;
				while(intCopy >= 1){
					intCopy = intCopy / 16;
					length++;
				}

				while (length > 0){
					int devider = 1;
					int digit;
					for(int i = 0; i < (length - 1); i++){
						devider = devider*16;
					}
					digit = interger / devider;
					if(digit < 10){
						sendChar = digit + '0';
						sendOneChar(&sendChar);
					}
					else{
						ITM_SendChar(digit + '0' + 7);
						sendChar = digit + '0' + 7;
						sendOneChar(&sendChar);
					}
					interger -= (digit*devider);
					length--;
				}
			}
		}
		else {
			sendChar = str[i];
			sendOneChar(&sendChar);
		}
		i++;
	}
}*/
