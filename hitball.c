#include "PID_old.h"
#include "hitball.h"
#include "Task_Init.h"
#include "RobStride.h"
#include "step.h"

//GPIO_PinState GPIOA8_State = GPIO_PIN_SET;
//GPIO_PinState GPIOC9_State = GPIO_PIN_SET;
//GPIO_PinState GPIOC2_State = GPIO_PIN_SET;
//GPIO_PinState GPIOC3_State = GPIO_PIN_SET;
//GPIO_PinState GPIOB10_State = GPIO_PIN_SET;
//GPIO_PinState GPIOB11_State = GPIO_PIN_SET;
//GPIO_PinState GPIOB12_State = GPIO_PIN_SET;
//GPIO_PinState GPIOB13_State = GPIO_PIN_SET;

GPIO_PinState key1, key2, key3;

uint8_t carry = 0;
uint8_t flag = 0;

TaskHandle_t Volleyball_Serve_Handle; 
void Volleyball_Serve(void *pvParameters)
{
	TickType_t last_wake_time = xTaskGetTickCount();
  for(;;)
	  {		
			if(flag == 1)
			  {
				carry = 1;
	       }
				if(carry == 1)
				{
			    //启动电磁阀门进行击球
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);
					vTaskDelay(1500);
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);
					carry = 0;
					flag = 2;
				}
			vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(2));
		 }
}

