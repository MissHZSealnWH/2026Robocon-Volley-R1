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
#include "JY61.h"

extern SemaphoreHandle_t Jy61_semaphore;
extern SemaphoreHandle_t remote_semaphore;


//陀螺仪姿态矫正
PID2 JY61_adjust = {
	.Kp = 0.8f,
	.Ki = 0.0f,
	.Kd = 0.1f,
	.limit = 10000.0f,
	.output_limit = 50.0f,
};

//遥控器
PackControl_t recv_pack;
uint8_t recv_buff[20] = {0};
float rocker_filter[4] = {0};
uint8_t usart5_buff[30];
uint8_t uart4_buff[30];


//电机驱动
Motor_param motor1 = {
.PID = {
	.Kp = 0.3f,
	.Ki = 0.0005f,
	.Kd = 1.5f,
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
	.Kp = 0.3f,
	.Ki = 0.0005f,
	.Kd = 1.5f,
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
	.Kp = 0.3f,
	.Ki = 0.0005f,
	.Kd = 1.5f,
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
extern uint8_t flag_two;

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

static float lock_Yaw = 0.0f;
//void Remote_Analysis()
//{
//    if(xSemaphoreTake(remote_semaphore, pdMS_TO_TICKS(200)) == pdTRUE)
//    {
//      /* 1. 保存上一帧 */
//      Remote_Control.Second = Remote_Control.First;
//      /* 2. 解析当前按键 */
//      Key_Parse(recv_pack.Key, &Remote_Control.First);
//			Remote_Control.Ex = recv_pack.rocker[1] / 1977.0f *MAX_ROBOT_VEL;
//			Remote_Control.Ey = recv_pack.rocker[0] / 1798.0f *MAX_ROBOT_VEL;
//			Remote_Control.Eomega = recv_pack.rocker[2] / 1847.0f * MAX_ROBOT_OMEGA;
//    }else {
//	    Remote_Control.Ex = 0;
//      Remote_Control.Ey = 0;
//      Remote_Control.Eomega = 0;
//			
//      memset(&Remote_Control.First, 0, sizeof(Remote_Control.First));
//    }
//}
void Remote_Analysis()
{
	/* 1. 保存上一帧 */
	Remote_Control.Second = Remote_Control.First;
	/* 2. 解析当前按键 */
	Key_Parse(recv_pack.Key, &Remote_Control.First);
	
//	Remote_Control.Ex = recv_pack.rocker[1] / 1977.0f *MAX_ROBOT_VEL;
//	Remote_Control.Ey = recv_pack.rocker[0] / 1798.0f *MAX_ROBOT_VEL;
//	Remote_Control.Eomega = recv_pack.rocker[2] / 1847.0f * MAX_ROBOT_OMEGA;
	Remote_Control.Ex = recv_pack.rocker[1] / 1647.0f *MAX_ROBOT_VEL;
	Remote_Control.Ey = recv_pack.rocker[0] / 1647.0f *MAX_ROBOT_VEL;
	Remote_Control.Eomega = recv_pack.rocker[2] / 1647.0f * MAX_ROBOT_OMEGA;
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
		//遥控器
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(remote_semaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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

volatile float Wz_correction;//反馈值
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
//			Wz = Remote_Control.Eomega;
float Wz_cmd = Remote_Control.Eomega;
			//前馈
			float Wz_ff = 0;
			
			static float Vy_last = 0;
			static float dVy_f = 0;
			float dt = 0.002f;

			float dVy = (Vy - Vy_last) / dt;
			Vy_last = Vy;

			//Vy限幅（根据最大速度4/dt）
			if (dVy > 2000.0f)  {dVy = 2000.0f;}
			if (dVy < -2000.0f) {dVy = -2000.0f;}

			//最终滤波
			dVy_f = 0.7f * dVy_f + 0.3f * dVy;

			if (fabs(dVy_f) > 10.0f)
			{
					Wz_ff = -0.02f * dVy_f;
			}
			//前馈Wz限幅
			if (Wz_ff > 0.314f)  {Wz_ff = 0.314f;}
			if (Wz_ff < -0.314f) {Wz_ff = -0.314f;}
			//矫正判断
			if (fabs(Wz_cmd) >= Deadzone_Z)
			{
				Wz = Wz_cmd;
			}
			else
			{
				float k = fabs(Vy) / (fabs(Vx) + fabs(Vy) + 0.001f);
				Wz = Wz_cmd + k * Wz_correction;
			}
			
			v1 = -Vy*0.5f+Vx*(sqrtf(3.0f)/2.0f) + R * Wz;
			v2 = -Vy*0.5f-Vx*(sqrtf(3.0f)/2.0f) + R * Wz;
			v3 = Vy + R * Wz;			
			
			wheel_one=  -((v1 / (2.0f * PI * WHEEL_RADIUS)) * 60.0f);
			wheel_two = (( v2 / (2.0f * PI * WHEEL_RADIUS)) * 60.0f);
			wheel_three=-((v3 / (2.0f * PI * WHEEL_RADIUS)) * 60.0f);
			
			PID_Control2((float)(((float)motor1.steering.epm / 7.0f/(3.4f))), wheel_one, &motor1.PID);
      PID_Control2((float)(((float)motor2.steering.epm / 7.0f/(3.4f))), wheel_two, &motor2.PID);
			PID_Control2((float)(((float)motor3.steering.epm / 7.0f/(3.4f))), wheel_three, &motor3.PID);
			
      VESC_SetCurrent(&motor1.steering, motor1.PID.pid_out);
      VESC_SetCurrent(&motor2.steering, motor2.PID.pid_out);
	    VESC_SetCurrent(&motor3.steering, motor3.PID.pid_out);  
			
			if(KEY_RISING_EDGE(Remote_Control.Second, Remote_Control.First, Right_Key_Down))
			{
				flag_two = 1;
			}
		if(recv_pack.rocker[0] == 0 && recv_pack.rocker[1] == 0 && recv_pack.rocker[2] == 0 )
//				if(abs(recv_pack.rocker[3]>1500))第二判断法
			{
	    Remote_Control.Ex = 0;
      Remote_Control.Ey = 0;
      Remote_Control.Eomega = 0;
			
      memset(&Remote_Control.First, 0, sizeof(Remote_Control.First));
			
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

TaskHandle_t Remote_JY61_Handle;
void Remote_JY61(void *pvParameters){
	
  TickType_t last_wake_time = xTaskGetTickCount();
	
   static float gyro_z_filter = 0;
	
   for(;;)
	{
	 float gyro_z = JY61.AngularVelocity.Z;
	
   gyro_z_filter = 0.7f * gyro_z_filter + 0.3f * gyro_z;
   gyro_z = gyro_z_filter;
		
		if (fabs(gyro_z) < 0.3f)
		{
				gyro_z = 0;
		}
  
		PID_Control2(-gyro_z, 0.0f, &JY61_adjust);

		float out = JY61_adjust.pid_out;

		if (out > 1.5f)  {out = 1.5f;}
		if (out < -1.5f) {out = -1.5f;}
		
		if (fabs(out) < 0.05f)//防抖
		{
				out = 0;
		}
		//滤波
		static float wz_f = 0;
		wz_f = 0.6f * wz_f + 0.4f * out;

		Wz_correction = wz_f;//转至Remote

	 vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(2));
	 }
}

TaskHandle_t Remote_Go_Handle;
void Remote_Go(void *pvParameters){
	  TickType_t last_wake_time = xTaskGetTickCount();
	
   for(;;)
	{
		Remote_Analysis();

	 vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(2));
	 }
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t Recv[8] = {0};
	uint32_t ID = CAN_Receive_DataFrame(&hcan2, Recv);
	VESC_ReceiveHandler(&motor1.steering, &hcan2, ID, Recv);
	VESC_ReceiveHandler(&motor2.steering, &hcan2, ID, Recv);
	VESC_ReceiveHandler(&motor3.steering, &hcan2, ID, Recv);
}

