#include "PID_old.h"
#include "hitball.h"
#include "Task_Init.h"
#include "RobStride.h"
#include "step.h"

//用于气缸电磁阀的GPIO:
GPIO_PinState GPIOA8_State;
GPIO_PinState GPIOC9_State;

//备用
//GPIO_PinState GPIOC3_State = GPIO_PIN_SET;


//作为光电门捕获的GPIO: 
static GPIO_PinState GPIOB10_State;
static GPIO_PinState GPIOB11_State;
static GPIO_PinState GPIOB12_State;
static GPIO_PinState GPIOB13_State;

static GPIO_PinState key1, key2, key3, key4;

uint8_t hit_ball_trigger = 0;
uint8_t flag_one = 0;
uint8_t flag_two = 0;

TaskHandle_t Volleyball_Serve_Handle; 
void Volleyball_Serve(void *pvParameters)
{
	TickType_t last_wake_time = xTaskGetTickCount();	  
  for(;;)
	  {		
		//读取电平 
		key1 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);
		key2 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
		key3 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
		key4 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);
		if(key1 == GPIO_PIN_SET || key2 == GPIO_PIN_SET || key3 == GPIO_PIN_SET || key4 == GPIO_PIN_SET)
			{
			flag_one = 1;	//这里做自动发球，手动发球方式暂存（chassis.c）
	    } 
		if(flag_one == 1)
		{
		hit_ball_trigger = 1;
		}
		if(hit_ball_trigger == 1)
			{
				
				//启动电磁阀门进行击球				
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
				vTaskDelay(200);
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
				vTaskDelay(200);
				hit_ball_trigger = 0;
				flag_one = 2;
				flag_two = 2;
			}
			vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(2));
		 }
}
