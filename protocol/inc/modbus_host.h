#ifndef __MODBUS_HOST_H
#define __MODBUS_HOST_H
#include "stm32f10x.h"

#define RSP_OK				0		/* �ɹ� */
#define RSP_ERR_CMD			0x01	/* ��֧�ֵĹ����� */
#define RSP_ERR_REG_ADDR	0x02	/* �Ĵ�����ַ���� */
#define RSP_ERR_VALUE		0x03	/* ����ֵ����� */
#define RSP_ERR_WRITE		0x04	/* д��ʧ�� */

#define H_RX_BUF_SIZE		255
#define H_TX_BUF_SIZE      	128
typedef struct
{
	uint8_t RxBuf[H_RX_BUF_SIZE];
	uint8_t RxCount;
	uint8_t RxStatus;
	uint8_t RxNewFlag;

	uint8_t RspCode;

	uint8_t TxBuf[H_TX_BUF_SIZE];
	uint8_t TxCount;
	
	uint16_t Reg01H;		/* �����������͵ļĴ����׵�ַ */
	uint16_t Reg02H;
	uint16_t Reg03H;		
	uint16_t Reg04H;

	uint8_t RegNum;			/* �Ĵ������� */

	uint8_t fAck01H;		/* Ӧ�������־ 0 ��ʾִ��ʧ�� 1��ʾִ�гɹ� */
	uint8_t fAck02H;
	uint8_t fAck03H;
	uint8_t fAck04H;
	uint8_t fAck05H;		
	uint8_t fAck06H;		
	uint8_t fAck10H;
	
}MODH_T;

typedef struct
{
	/* 03H 06H ��д���ּĴ��� */
	uint16_t P01;
	uint16_t P02;
	
	/* 02H ��д��ɢ����Ĵ��� */
	uint16_t T01;
	uint16_t T02;
	uint16_t T03;
	
	
	/* 04H ��ȡģ�����Ĵ��� */
	uint16_t A01;
	int16_t DR[100];
    
	/* 01H 05H ��д����ǿ����Ȧ */
	uint16_t D01;
	uint16_t D02;
	uint16_t D03;
	uint16_t D04;
	
}VAR_HOST;
void MODH_Send01H(uint8_t _addr, uint16_t _reg, uint16_t _num);
void MODH_Send04H(uint8_t _addr, uint16_t _reg, uint16_t _num);
uint8_t MODH_ReadParam_01H(uint8_t addr,uint16_t _reg, uint16_t _num);
uint8_t MODH_ReadParam_04H(uint8_t _addr,uint16_t _reg, uint16_t _num);
#endif
