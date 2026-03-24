#include "RMLibHead.h"
#include "Task_Init.h"
#include "can.h"
#include "Chassis.h"
#include "CANDrive.h"
#include "hitball.h"

extern uint8_t usart5_buff[30];

void Task_Init(){
	
	 //Ò£¿ØÆ÷
   __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
   HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_buff, sizeof(usart5_buff));
   __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
	
	vPortEnterCritical();
	
	xTaskCreate(Remote,
         "Remote",
          400,
          NULL,
          3,
          &Remote_Handle); 
	
	xTaskCreate(Remote_JY61,
         "Remote_JY61",
          256,
          NULL,
          3,
          &Remote_JY61_Handle);
					
//	xTaskCreate(Volleyball_Serve,
//         "hit_ball",
//          256,
//          NULL,
//          3,
//          &Volleyball_Serve_Handle);
					
	vPortExitCritical();
}
