#include "main.h"
#include "tim.h"
#include "key.h"
#include "motor.h"

int8_t Key2_Nub = 0;
int8_t Key3_Nub = -2;
int8_t Key1_Nub = 0;

typedefKey Key1={GPIOC, GPIO_PIN_13, 1, 1, Key1Hander};
typedefKey Key2={GPIOA, GPIO_PIN_6, 1, 1, Key2Hander};
typedefKey Key3={GPIOB, GPIO_PIN_1, 1, 1, Key3Hander};


void Key1Hander()
{
	Key1_Nub ++;
	gimbal_ctrl.mode = (gimbal_ctrl.mode + 1) % 4;
}
void Key2Hander()
{
	if(System_Busy == 1) return;
	Key2_Nub = 1;
}

void Key3Hander()    //Gimbal Enable
{
	if(System_Busy == 1) return; 
	if(Key3_Nub == -1) Key3_Nub = 2;
	if(Key3_Nub == -2) Key3_Nub = 1;
}

void KeyScan(typedefKey* key)
{
	key->LastState=key->NowState;
	key->NowState=HAL_GPIO_ReadPin(key->GPIOx,key->GPIO_Pin);
	if(key->LastState==0&&key->NowState==1)
	{
		key->hander();
	}
}


