#ifndef _TASK_INIT_H_
#define _TASK_INIT_H_

#include "RMLibHead.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "queue.h"
#include "CANDrive.h"

#include "usart.h"
#include "bsp_dwt.h"
#include "Chassis.h"
#include "math.h"

#include "PID_old.h"
#include "math.h"


void Wheel_Task(void *pvParameters);

typedef struct{
	uint8_t Left_Key_Up;         
	uint8_t Left_Key_Down;       
	uint8_t Left_Key_Left;       
	uint8_t Left_Key_Right;       
	uint8_t Left_Switch_Up;       
	uint8_t Left_Switch_Down;
	uint8_t Left_Broadside_Key;

	uint8_t Right_Key_Up;        
	uint8_t Right_Key_Down;      
	uint8_t Right_Key_Left;      
	uint8_t Right_Key_Right;     
	uint8_t Right_Switch_Up;      
	uint8_t Right_Switch_Down;      
	uint8_t Right_Broadside_Key;
} hw_key_t;

typedef struct {
    float Ex;
    float Ey;
    float Eomega;
    hw_key_t First,Second;
} Remote_Handle_t;

extern Remote_Handle_t Remote_Control; //取出遥控器数据

#define KEY_RISING_EDGE(cur, last, field)  ((cur.field == 1) && (last.field == 0))

void Task_Init(void); 
float Gyro_CalcCorrection(float gyro_z);//打滑使用

#endif
