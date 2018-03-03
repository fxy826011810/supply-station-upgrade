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
	int32_t raw_value;   									//���������������ԭʼֵ
	int32_t last_raw_value;								//��һ�εı�����ԭʼֵ
	int32_t ecd_value;                       //��������������ı�����ֵ
	int32_t diff;													//���α�����֮��Ĳ�ֵ
	int32_t temp_count;                   //������
	uint8_t buf_count;								//�˲�����buf��
	int32_t ecd_bias;											//��ʼ������ֵ	
	int32_t ecd_raw_rate;									//ͨ������������õ����ٶ�ԭʼֵ
	int32_t rate_buf[RATE_BUF_SIZE];	//buf��for filter
	int32_t round_cnt;										//Ȧ��
	float filter_rate;											//�ٶ�
	float ecd_angle;											//�Ƕ�
}Encoder;


extern Encoder _12V_RM6025Encoder;
extern Encoder _24V_RM6025Encoder;



void Bsp_Can_Init(void);
void Can_RecviveData(CanRxMsg * rec);//����ֵ����

void cm_senddata(CAN_TypeDef* CANx, int16_t id, int16_t num);
void ec60_senddata(CAN_TypeDef* CANx, int16_t id, int16_t num1, int16_t num2, int16_t num3, int16_t num4);
#endif

