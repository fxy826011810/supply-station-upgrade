#ifndef __CAN_H
#define __CAN_H
#include "stm32f10x.h"

#define RATE_BUF_SIZE 6
typedef enum
{
	CAN_RM6025_12v_ID				=0x200,
	CAN_RM6025_24v_ID				=0x1FF,
	CAN_RM6025_12v_REC_ID		=0x201,
	CAN_RM6025_24v_REC_ID		=0x206,
}CAN_ID;


typedef struct{
	int32_t raw_value;   									//编码器不经处理的原始值
	int32_t last_raw_value;								//上一次的编码器原始值
	int32_t ecd_value;                       //经过处理后连续的编码器值
	int32_t diff;													//两次编码器之间的差值
	int32_t temp_count;                   //计数用
	uint8_t buf_count;								//滤波更新buf用
	int32_t ecd_bias;											//初始编码器值	
	int32_t ecd_raw_rate;									//通过编码器计算得到的速度原始值
	int32_t rate_buf[RATE_BUF_SIZE];	//buf，for filter
	int32_t round_cnt;										//圈数
	float filter_rate;											//速度
	float ecd_angle;											//角度
}Encoder;


extern Encoder _12V_RM6025Encoder;
extern Encoder _24V_RM6025Encoder;



void Bsp_Can_Init(void);
void Can_RecviveData(CanRxMsg * rec);//返回值解算

void cm_senddata(CAN_TypeDef* CANx, int16_t id, int16_t num);
void ec60_senddata(CAN_TypeDef* CANx, int16_t id, int16_t num1, int16_t num2, int16_t num3, int16_t num4);
#endif

