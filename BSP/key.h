#ifndef __KEY_H
#define __KEY_H

#include "main.h"

typedef struct{
	GPIO_TypeDef *GPIOx;
	uint16_t GPIO_Pin;
	GPIO_PinState LastState;
	GPIO_PinState NowState;
	void (*hander)();
}typedefKey;


extern typedefKey Key1;
extern typedefKey Key2;
extern typedefKey Key3;
extern typedefKey Key4;

extern int8_t Key3_Nub;
extern int8_t Key2_Nub;
extern int8_t Key1_Nub;


void Key1Hander();
void Key2Hander();
void Key3Hander();
void KeyScan(typedefKey* key);

#endif