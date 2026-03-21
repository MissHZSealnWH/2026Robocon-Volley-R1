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
#include "math.h"

//遥控器
PackControl_t recv_pack;
uint8_t recv_buff[20] = {0};
float rocker_filter[4] = {0};
uint8_t usart5_buff[30];


//电机驱动
Motor_param motor1 = {
.PID = {
	.Kp = 5.0f,
	.Ki = 0.0f,
	.Kd = 30.0f,
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
	.Kp = 5.0f,
	.Ki = 0.0f,
	.Kd = 30.0f,
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
	.Kp = 5.0f,
	.Ki = 0.0f,
	.Kd = 30.0f,
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

volatile float Vx =0;   //前后移动
volatile float Vy =0;   //左右移动
volatile float Wz =0;   //顺逆自转

volatile float v1 = 0.0f;
volatile float v2 = 0.0f;
volatile float v3 = 0.0f;

volatile float wheel_one = 0.0f;
volatile float wheel_two = 0.0f;
volatile float wheel_three=0.0f;


static void Key_Parse(uint32_t key, hw_key_t *out)
{
  	out->Right_Switch_Up     = (key & KEY_Right_Switch_Up)     ? 1 : 0;
    out->Right_Switch_Down   = (key & KEY_Right_Switch_Down)   ? 1 : 0;

    out->Right_Key_Up        = (key & KEY_Right_Key_Up)        ? 1 : 0;
    out->Right_Key_Down      = (key & KEY_Right_Key_Down)      ? 1 : 0;
    out->Right_Key_Left      = (key & KEY_Right_Key_Left)      ? 1 : 0;
    out->Right_Key_Right     = (key & KEY_Right_Key_Right)     ? 1 : 0;

    out->Right_Broadside_Key = (key & KEY_Right_Broadside_Key) ? 1 : 0;

    out->Left_Switch_Up      = (key & KEY_Left_Switch_Up)      ? 1 : 0;
    out->Left_Switch_Down    = (key & KEY_Left_Switch_Down)    ? 1 : 0;

    out->Left_Key_Up         = (key & KEY_Left_Key_Up)         ? 1 : 0;
    out->Left_Key_Down       = (key & KEY_Left_Key_Down)       ? 1 : 0;
    out->Left_Key_Left       = (key & KEY_Left_Key_Left)       ? 1 : 0;
    out->Left_Key_Right      = (key & KEY_Left_Key_Right)      ? 1 : 0;

    out->Left_Broadside_Key  = (key & KEY_Left_Broadside_Key)  ? 1 : 0;
}

void Remote_Analysis()
{
	/* 1. 保存上一帧 */
	Remote_Control.Second = Remote_Control.First;
	/* 2. 解析当前按键 */
	Key_Parse(recv_pack.Key, &Remote_Control.First);
	
	Remote_Control.Ex = recv_pack.rocker[1] / 1847.0f *MAX_ROBOT_VEL;
	Remote_Control.Ey = recv_pack.rocker[0] / 1798.0f *MAX_ROBOT_VEL;
	Remote_Control.Eomega = recv_pack.rocker[2] / 1847.0f * MAX_ROBOT_OMEGA;
}


//遥控器滤波降噪 
void Rocker_Filter(PackControl_t *data)
{
    float alpha = 0.6f;

    for(int i = 0; i < 4; i++)
    {
        rocker_filter[i] = alpha * data->rocker[i] + (1.0f - alpha) * rocker_filter[i];

        data->rocker[i] = rocker_filter[i];
    }
}

void MyRecvCallback(uint8_t *src, uint16_t size, void *user_data)
{
    memcpy(&recv_buff, src, size);
    memcpy(&recv_pack, recv_buff, sizeof(recv_pack));
    Rocker_Filter(&recv_pack);
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


CommPackRecv_Cb recv_cb = MyRecvCallback;

TaskHandle_t Remote_Handle;
void Remote(void *pvParameters)
{
	TickType_t last_wake_time = xTaskGetTickCount();
	
    g_comm_handle = Comm_Init(&huart5);
    RemoteCommInit(NULL);
    register_comm_recv_cb(recv_cb, 0x01, &recv_pack);
	for(;;)
	{
		if(MODE == REMOTE)
		{			
	  		Remote_Analysis();
				Vx = Remote_Control.Ex;
				Vy = -Remote_Control.Ey;
		    Wz = Remote_Control.Eomega;
			
			v1 = -Vy*0.5f+Vx*(sqrt(3.0f)/2.0) + R * Wz;
			v2 = -Vy*0.5f-Vx*(sqrt(3.0f)/2.0) + R * Wz;
			v3 = Vy + R * Wz;			
			
			wheel_one=  -((v1 / (2.0f * PI * WHEEL_RADIUS)) * 60.0f);
			wheel_two = (( v2 / (2.0f * PI * WHEEL_RADIUS)) * 60.0f);
			wheel_three=-((v3 / (2.0f * PI * WHEEL_RADIUS)) * 60.0f);
			
			PID_Control2((float)(motor1.steering.epm / 7.0f/(3.4f)), wheel_one, &motor1.PID);
			PID_Control2((float)(motor2.steering.epm / 7.0f/(3.4f)), wheel_two, &motor2.PID);
			PID_Control2((float)(motor3.steering.epm / 7.0f/(3.4f)), wheel_three,&motor3.PID);

			vTaskDelay(1);
      VESC_SetCurrent(&motor1.steering, motor1.PID.pid_out);
      VESC_SetCurrent(&motor2.steering, motor2.PID.pid_out);
	    VESC_SetCurrent(&motor3.steering, motor3.PID.pid_out);  
			
			if(recv_pack.rocker[0] == 0 || recv_pack.rocker[1] == 0 ||recv_pack.rocker[2] == 0 )
			{
        Remote_Control.Ex = 0;
				Remote_Control.Ey = 0;
				Remote_Control.Eomega = 0;

				//按键状态清零
				memset(&recv_pack.Key, 0, sizeof(uint32_t));
		 	}
  
		}
		if(MODE == STP || MODE == STOP )
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


////遥控任务
//TaskHandle_t Move_Remote_Handle;
//void Move_Remote(void *pvParameters){
//	
//	TickType_t last_wake_time = xTaskGetTickCount();
//	

//	
//    for(;;)
//    {

//				
//				if(MODE == REMOTE)
//      {


//      }
//			

//		vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(2));
//    }
//}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t Recv[8] = {0};
	uint32_t ID = CAN_Receive_DataFrame(&hcan2, Recv);
	VESC_ReceiveHandler(&motor1.steering, &hcan2, ID, Recv);
	VESC_ReceiveHandler(&motor2.steering, &hcan2, ID, Recv);
	VESC_ReceiveHandler(&motor3.steering, &hcan2, ID, Recv);
}

