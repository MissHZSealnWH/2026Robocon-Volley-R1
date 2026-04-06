#ifndef _REMOTE_H_
#define _REMOTE_H_

#include "Task_Init.h"
#include <stdbool.h>
#include "PID_old.h"
#include "VESC.h"

#define PI 3.14159265359f
#define MAX_ROBOT_VEL 4.0f	  // 底盘最大速度
#define MAX_ROBOT_OMEGA PI	 	 //最大角速度
#define R 0.457f	 	//整车半径
#define WHEEL_RADIUS 0.075f  //轮的半径
#define Deadzone_Z 0.157f

//电机参数
typedef struct{
	PID2 PID;
	VESC_t steering;

}Motor_param;

typedef enum {
     STP,//自动模式下的急停
     STOP,//遥控模式下的急停
     REMOTE,
	   CHOOSE,
}Positon_label;

//任务
extern TaskHandle_t Remote_JY61_Handle;
extern TaskHandle_t Remote_Handle;
extern TaskHandle_t Remote_Go_Handle;
//模式
extern Positon_label MODE;

//任务函数
void Remote(void *pvParameters);
void Remote_JY61(void *pvParameters);
void Remote_Go(void *pvParameters);

void Remote_Analysis();

#endif
