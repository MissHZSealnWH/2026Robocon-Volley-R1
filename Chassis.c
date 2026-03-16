#include "Chassis.h"
#include "VESC.h"
#include "PID_old.h"
#include "Task_Init.h"
#include "semphr.h"
#include "dataFrame.h"
#include "comm.h"
#include "comm_stm32_hal_middle.h"
#include "data_poll.h"
#include "My_list.h"

//遥控器
PackControl_t recv_pack;
uint8_t recv_buff[20] = {0};
float rocker_filter[4] = {0};
uint8_t usart5_buff[30];
//电机驱动
Motor_param motor1 = {
.PID = {
	.Kp = 0.0f,
	.Ki = 0.0f,
	.Kd = 0.0f,
	.limit = 10000.0f,
	.output_limit = 40.0f,
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
	.output_limit = 40.0f,
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
	.output_limit = 40.0f,
},
.steering={
	.motor_id=0x03,
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

volatile float wheel_one = 0.0f;
volatile float wheel_two = 0.0f;
volatile float wheel_three=0.0f;

TaskHandle_t Remote_Handle;
void Remote(void *pvParameters)
{
	TickType_t last_wake_time = xTaskGetTickCount();
	
	for(;;)
	{
		if(MODE == REMOTE)
		{			
//			v1 = -Vx*0.5f+Vy*(sqrt(3.0f)/2.0) + R * Wz;
//			v2 = -Vx*0.5f-Vy*(sqrt(3.0f)/2.0) + R * Wz;
//			v3 = Vx + R * Wz;
			
			v1 = -Vy*0.5f+Vx*(sqrt(3.0f)/2.0) + R * Wz;
			v2 = -Vy*0.5f-Vx*(sqrt(3.0f)/2.0) + R * Wz;
			v3 = Vy + R * Wz;			
			
			wheel_one=  -((v1 / (2.0f * PI * WHEEL_RADIUS)) * 60.0f);
			wheel_two = ((v2 / (2.0f * PI * WHEEL_RADIUS)) * 60.0f);
			wheel_three=-((v3 / (2.0f * PI * WHEEL_RADIUS)) * 60.0f);
			
			PID_Control2((float)(motor1.steering.epm / 7.0f/(3.4f)), wheel_one, &motor1.PID);
			PID_Control2((float)(motor2.steering.epm / 7.0f/(3.4f)), wheel_two, &motor2.PID);
			PID_Control2((float)(motor3.steering.epm / 7.0f/(3.4f)), wheel_three,&motor3.PID);

			vTaskDelay(1);
      VESC_SetCurrent(&motor1.steering, motor1.PID.pid_out);
      VESC_SetCurrent(&motor2.steering, motor2.PID.pid_out);
	    VESC_SetCurrent(&motor3.steering, motor3.PID.pid_out);  
 
		}
		if(MODE == STP || MODE == STOP)
		{
			wheel_one = 0;
			wheel_two = 0;
			wheel_three=0;
			
			VESC_SetCurrent(&motor1.steering, 0);
			VESC_SetCurrent(&motor2.steering, 0);
			VESC_SetCurrent(&motor3.steering, 0);
		}
		vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(2));
	}
}

void Rocker_Filter(PackControl_t *data)
{
    float alpha = 0.6f;

    for(int i = 0; i < 4; i++)
    {
        rocker_filter[i] = alpha * data->rocker[i] +
                          (1.0f - alpha) * rocker_filter[i];

        data->rocker[i] = rocker_filter[i];
    }
}

void MyRecvCallback(uint8_t *src, uint16_t size, void *user_data)
{
    memcpy(&recv_buff, src, size);
    memcpy(&recv_pack, recv_buff, sizeof(recv_pack));
    Rocker_Filter(&recv_pack);
}

extern SemaphoreHandle_t remote_semaphore;
CommPackRecv_Cb  recv_cb = MyRecvCallback;

//遥控任务
TaskHandle_t Move_Remote_Handle;
void Move_Remote(void *pvParameters){
	
	TickType_t last_wake_time = xTaskGetTickCount();
    g_comm_handle = Comm_Init(&huart5);
    RemoteCommInit(NULL);
    register_comm_recv_cb(recv_cb, 0x01, &recv_pack);
    for(;;)
    {
      if(xSemaphoreTake(remote_semaphore, pdMS_TO_TICKS(200)) == pdTRUE)
        {
            memcpy(&recv_pack, usart5_buff, sizeof(PackControl_t));
            Updatekey(&Remote_Control);
            Remote_Control.Ex =-recv_pack.rocker[1];
            Remote_Control.Ey = recv_pack.rocker[0];
            Remote_Control.Eomega = recv_pack.rocker[2];
            Remote_Control.mode = recv_pack.rocker[3];
            Remote_Control.Key_Control = (hw_key_t*)&recv_pack.Key;
        }
				else
				{
            Remote_Control.Ex = 0;
            Remote_Control.Ey = 0;
            Remote_Control.Eomega = 0;
            Remote_Control.mode = 0;
            //按键状态清零
            memset(&recv_pack.Key, 0, sizeof(hw_key_t));
            Remote_Control.Key_Control = (hw_key_t*)&recv_pack.Key;
        }

     if(MODE == REMOTE)
      {
			  //遥控映射
				Vx = -(Remote_Control.Ex / 2047.0f) * MAX_VELOCITY;
				Vy = -(Remote_Control.Ey / 2047.0f) * MAX_VELOCITY;
				Wz = (Remote_Control.Eomega / 2047.0f) * MAX_OMEGA;
//			//进行击球动作（flag转到hitball.c）
//			if(Remote_Control.First.Right_Key_Up== 1 && Remote_Control.Second.Right_Key_Up == 0)
//		  	{
//		  		flag = 1;
//			  }
      }
		vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(2));
    }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t Recv[8] = {0};
	uint32_t ID = CAN_Receive_DataFrame(&hcan2, Recv);
	VESC_ReceiveHandler(&motor1.steering, &hcan2, ID,Recv);
	VESC_ReceiveHandler(&motor2.steering, &hcan2, ID,Recv);
	VESC_ReceiveHandler(&motor3.steering, &hcan2, ID,Recv);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
	if (huart->Instance == UART5)
	{
		HAL_UART_DMAStop(&huart5);
		Comm_UART_IRQ_Handle(g_comm_handle, &huart5, usart5_buff,size);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_buff,sizeof(usart5_buff));
   		__HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
    {
        HAL_UART_DMAStop(huart);
        // 重置HAL状态
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        huart->RxState = HAL_UART_STATE_READY;
        huart->gState = HAL_UART_STATE_READY;
        
        // 然后清除错误标志 - 按照STM32F4参考手册要求的顺序
        uint32_t isrflags = READ_REG(huart->Instance->SR);
        
        // 按顺序处理各种错误标志，必须先读SR再读DR来清除错误
        if (isrflags & (USART_SR_ORE | USART_SR_NE | USART_SR_FE)) 
        {
            // 对于ORE、NE、FE错误，需要先读SR再读DR
            volatile uint32_t temp_sr = READ_REG(huart->Instance->SR);
            volatile uint32_t temp_dr = READ_REG(huart->Instance->DR); // 这个读取会清除ORE、NE、FE        

        if (isrflags & USART_SR_PE)
        {
            volatile uint32_t temp_sr = READ_REG(huart->Instance->SR);
        }
        
    }
      Comm_UART_IRQ_Handle(g_comm_handle, &huart5, usart5_buff, 0);
      HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_buff,sizeof(usart5_buff));
      __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
    }
}
