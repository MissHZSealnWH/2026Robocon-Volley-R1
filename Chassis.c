#include "Chassis.h"
#include "VESC.h"
#include "PID_old.h"
#include "Task_Init.h"
#include "semphr.h"

//电机驱动
Motor_param motor1 = {
.PID = {
	.Kp = 0.0f,
	.Ki = 0.0f,
	.Kd = 0.0f,
	.limit = 10000.0f,
	.output_limit = 60.0f,
},
.steering={
	.motor_id=0x01,
	.hcan = &hcan2,
}
};
Motor_param motor2 = {
.PID = {
	.Kp = 0.0f,
	.Ki = 0.0f,
	.Kd = 0.0f,
	.limit = 10000.0f,
	.output_limit = 60.0f,
},
.steering={
	.motor_id=0x02,
	.hcan = &hcan2,
}
};
Motor_param motor3 = {
.PID = {
	.Kp = 0.0f,
	.Ki = 0.0f,
	.Kd = 0.0f,
	.limit = 10000.0f,
	.output_limit = 60.0f,
},
.steering={
	.motor_id=0x04,
	.hcan = &hcan2,
}
};
//extern GPIO_PinState GPIOA8_State;
//extern GPIO_PinState GPIOC9_State;
//extern GPIO_PinState GPIOC2_State;
//extern GPIO_PinState GPIOC3_State;
//extern GPIO_PinState GPIOB10_State;
//extern GPIO_PinState GPIOB11_State;
//extern GPIO_PinState GPIOB12_State;
//extern GPIO_PinState GPIOB13_State;
extern uint8_t flag;

//遥控模式
Positon_label MODE = REMOTE;

float Vx =0;   //前后移动
float Vy =0;   //左右移动
float Wz =0;   //顺逆自转

//该变量的值可能会被程序外的因素（如硬件、其他线程）修改
volatile float v1 = 0.0f;
volatile float v2 = 0.0f;
volatile float v3 = 0.0f;

volatile float wheel_one = 0.0f;  //前左
volatile float wheel_two = 0.0f;  //前右
volatile float wheel_three=0.0f;  //后右

TaskHandle_t Remote_Handle;
void Remote(void *pvParameters)
{
	TickType_t last_wake_time = xTaskGetTickCount();
	
	for(;;)
	{
		if(MODE == REMOTE)
		{			
			v1 = Vx -LENGTH * Wz;
			v2 = Vx*0.5f+ Vy*(sqrt(3.0f)/2.0) + LENGTH * Wz;
			v3 = Vx*0.5f-Vy*(sqrt(3.0f)/2.0) + LENGTH * Wz;
			
			wheel_one=  (int16_t)((v1 / (2.0f * PI * WHEEL_RADIUS)) * 60.0f);
			wheel_two=  (int16_t)((v2 / (2.0f * PI * WHEEL_RADIUS)) * 60.0f);
			wheel_three=(int16_t)((v3 / (2.0f * PI * WHEEL_RADIUS)) * 60.0f);
			
			PID_Control2((float)(motor1.steering.epm / 7.0f/(3.4f)), wheel_one, &motor1.PID);
			PID_Control2((float)(motor2.steering.epm / 7.0f/(3.4f)), wheel_two, &motor2.PID);
			PID_Control2((float)(motor3.steering.epm / 7.0f/(3.4f)), wheel_three,&motor3.PID);

      VESC_SetCurrent(&motor1.steering, motor1.PID.pid_out);
      VESC_SetCurrent(&motor2.steering, motor2.PID.pid_out);
	    VESC_SetCurrent(&motor3.steering, motor3.PID.pid_out);  
 
		}
		if(MODE == STP || MODE == STOP)
		{
			wheel_one = 0;
			wheel_two = 0;
			wheel_three = 0;
			
			VESC_SetCurrent(&motor1.steering, 0);
			VESC_SetCurrent(&motor2.steering, 0);
			VESC_SetCurrent(&motor3.steering, 0);
		}
			vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(2));
	}
}
extern SemaphoreHandle_t remote_semaphore;

TaskHandle_t Move_Remote_Handle;
void Move_Remote(void *pvParameters){
	
	TickType_t last_wake_time = xTaskGetTickCount();
    for(;;)
    {
        if(xSemaphoreTake(remote_semaphore, pdMS_TO_TICKS(200)) == pdTRUE)
        {
            memcpy(&RemoteData, usart4_dma_buff, sizeof(RemoteData));
            Updatekey(&Remote_Control);
            Remote_Control.Ex =-RemoteData.rocker[1];
            Remote_Control.Ey = RemoteData.rocker[0];
            Remote_Control.Eomega = RemoteData.rocker[2];
            Remote_Control.mode = RemoteData.rocker[3];
            Remote_Control.Key_Control = &RemoteData.Key;
        }else{
            Remote_Control.Ex = 0;
            Remote_Control.Ey = 0;
            Remote_Control.Eomega = 0;
            Remote_Control.mode = 0;
            //按键状态清零
            memset(&RemoteData.Key, 0, sizeof(hw_key_t));
            Remote_Control.Key_Control = &RemoteData.Key;
        }

     if(MODE == REMOTE)
      {
					//遥控映射
            Vx = -(Remote_Control.Ex / 2047.0f) * MAX_VELOCITY;
            Vy = -(Remote_Control.Ey / 2047.0f) * MAX_VELOCITY;
            Wz = (Remote_Control.Eomega / 2047.0f) * MAX_OMEGA;
			//进行击球动作（flag转到hitball.c）
				if(Remote_Control.First.Right_Key_Up== 1 && Remote_Control.Second.Right_Key_Up == 0)
		  	{
		  		flag = 1;
			  }
      }
				vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1));
    }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t Recv[8] = {0};
	uint32_t ID = CAN_Receive_DataFrame(hcan, Recv);
	VESC_ReceiveHandler(&motor1.steering, &hcan2, ID,Recv);
	VESC_ReceiveHandler(&motor2.steering, &hcan2, ID,Recv);
	VESC_ReceiveHandler(&motor3.steering, &hcan2, ID,Recv);
}
