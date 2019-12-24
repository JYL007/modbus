#include "modbus_host.h"
#include "bsp_modbus_crc16.h"
#include "bsp_rs485.h"
#include "bsp_timer.h"
#include "bsp_dataconvert.h"

#include<stdio.h>


#define TIMEOUT		500		/* �������ʱʱ��, ��λms */
#define NUM			1		/* ѭ�����ʹ��� */
void bsp_Idle(void);
void MODH_AnalyzeApp(void);
MODH_T g_tModH;	//modbus host protocol data buff
VAR_HOST g_tvar_host;
uint8_t g_modh_timeout = 0; //timeout flag

void MODH_RxTimeOut(void)
{
    g_modh_timeout = 1;
}

/*
*********************************************************************************************************
*	�� �� ��: MODH_SendPacket
*	����˵��: �������ݰ� COM1��
*	��    ��: _buf : ���ݻ�����
*			  _len : ���ݳ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MODH_SendPacket(uint8_t *_buf, uint16_t _len)
{
    RS485_Host_SendBuf(_buf, _len);
}
static void MODH_SendAckWithCRC(void)
{
	uint16_t crc,i;
    crc = CRC16_Modbus(g_tModH.TxBuf, g_tModH.TxCount);
    g_tModH.TxBuf[g_tModH.TxCount++] = crc >> 8;
    g_tModH.TxBuf[g_tModH.TxCount++] = crc;
	for(i=0;i<g_tModH.TxCount;i++)
	{
		printf("%02X ",g_tModH.TxBuf[i]);
	}
	MODH_SendPacket(g_tModH.TxBuf, g_tModH.TxCount);
}
/*
*********************************************************************************************************
*	�� �� ��: MODH_Send01H
*	����˵��: ����01Hָ���ѯ1���������ּĴ���
*	��    ��: _addr : ��վ��ַ
*			  _reg : �Ĵ������
*			  _num : �Ĵ�������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MODH_Send01H(uint8_t _addr, uint16_t _reg, uint16_t _num)
{
    g_tModH.TxCount = 0;
    g_tModH.TxBuf[g_tModH.TxCount++] = _addr;		/* ��վ��ַ */
    g_tModH.TxBuf[g_tModH.TxCount++] = 0x01;		/* ������ */
    g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;	/* �Ĵ������ ���ֽ� */
    g_tModH.TxBuf[g_tModH.TxCount++] = _reg;		/* �Ĵ������ ���ֽ� */
    g_tModH.TxBuf[g_tModH.TxCount++] = _num >> 8;	/* �Ĵ������� ���ֽ� */
    g_tModH.TxBuf[g_tModH.TxCount++] = _num;		/* �Ĵ������� ���ֽ� */

    MODH_SendAckWithCRC();		/* �������ݣ��Զ���CRC */
    g_tModH.fAck01H = 0;		/* ����ձ�־ */
    g_tModH.RegNum = _num;		/* �Ĵ������� */
    g_tModH.Reg01H = _reg;		/* ����03Hָ���еļĴ�����ַ�������Ӧ�����ݽ��з��� */
}

/*
*********************************************************************************************************
*	�� �� ��: MODH_Send02H
*	����˵��: ����02Hָ�����ɢ����Ĵ���
*	��    ��: _addr : ��վ��ַ
*			  _reg : �Ĵ������
*			  _num : �Ĵ�������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MODH_Send02H(uint8_t _addr, uint16_t _reg, uint16_t _num)
{
    g_tModH.TxCount = 0;
    g_tModH.TxBuf[g_tModH.TxCount++] = _addr;		/* ��վ��ַ */
    g_tModH.TxBuf[g_tModH.TxCount++] = 0x02;		/* ������ */
    g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;	/* �Ĵ������ ���ֽ� */
    g_tModH.TxBuf[g_tModH.TxCount++] = _reg;		/* �Ĵ������ ���ֽ� */
    g_tModH.TxBuf[g_tModH.TxCount++] = _num >> 8;	/* �Ĵ������� ���ֽ� */
    g_tModH.TxBuf[g_tModH.TxCount++] = _num;		/* �Ĵ������� ���ֽ� */

    MODH_SendAckWithCRC();		/* �������ݣ��Զ���CRC */
    g_tModH.fAck02H = 0;		/* ����ձ�־ */
    g_tModH.RegNum = _num;		/* �Ĵ������� */
    g_tModH.Reg02H = _reg;		/* ����03Hָ���еļĴ�����ַ�������Ӧ�����ݽ��з��� */
}

/*
*********************************************************************************************************
*	�� �� ��: MODH_Send03H
*	����˵��: ����03Hָ���ѯ1���������ּĴ���
*	��    ��: _addr : ��վ��ַ
*			  _reg : �Ĵ������
*			  _num : �Ĵ�������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MODH_Send03H(uint8_t _addr, uint16_t _reg, uint16_t _num)
{
    g_tModH.TxCount = 0;
    g_tModH.TxBuf[g_tModH.TxCount++] = _addr;		/* ��վ��ַ */
    g_tModH.TxBuf[g_tModH.TxCount++] = 0x03;		/* ������ */
    g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;	/* �Ĵ������ ���ֽ� */
    g_tModH.TxBuf[g_tModH.TxCount++] = _reg;		/* �Ĵ������ ���ֽ� */
    g_tModH.TxBuf[g_tModH.TxCount++] = _num >> 8;	/* �Ĵ������� ���ֽ� */
    g_tModH.TxBuf[g_tModH.TxCount++] = _num;		/* �Ĵ������� ���ֽ� */

    MODH_SendAckWithCRC();		/* �������ݣ��Զ���CRC */
    g_tModH.fAck03H = 0;		/* ����ձ�־ */
    g_tModH.RegNum = _num;		/* �Ĵ������� */
    g_tModH.Reg03H = _reg;		/* ����03Hָ���еļĴ�����ַ�������Ӧ�����ݽ��з��� */
}

/*
*********************************************************************************************************
*	�� �� ��: MODH_Send04H
*	����˵��: ����04Hָ�������Ĵ���
*	��    ��: _addr : ��վ��ַ
*			  _reg : �Ĵ������
*			  _num : �Ĵ�������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MODH_Send04H(uint8_t _addr, uint16_t _reg, uint16_t _num)
{
    g_tModH.TxCount = 0;
    g_tModH.TxBuf[g_tModH.TxCount++] = _addr;		/* ��վ��ַ */
    g_tModH.TxBuf[g_tModH.TxCount++] = 0x04;		/* ������ */
    g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;	/* �Ĵ������ ���ֽ� */
    g_tModH.TxBuf[g_tModH.TxCount++] = _reg;		/* �Ĵ������ ���ֽ� */
    g_tModH.TxBuf[g_tModH.TxCount++] = _num >> 8;	/* �Ĵ������� ���ֽ� */
    g_tModH.TxBuf[g_tModH.TxCount++] = _num;		/* �Ĵ������� ���ֽ� */

    MODH_SendAckWithCRC();		/* �������ݣ��Զ���CRC */
    g_tModH.fAck04H = 0;		/* ����ձ�־ */
    g_tModH.RegNum = _num;		/* �Ĵ������� */
    g_tModH.Reg04H = _reg;		/* ����03Hָ���еļĴ�����ַ�������Ӧ�����ݽ��з��� */
}

/*
*********************************************************************************************************
*	�� �� ��: MODH_Send05H
*	����˵��: ����05Hָ�дǿ�õ���Ȧ
*	��    ��: _addr : ��վ��ַ
*			  _reg : �Ĵ������
*			  _value : �Ĵ���ֵ,2�ֽ�
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MODH_Send05H(uint8_t _addr, uint16_t _reg, uint16_t _value)
{
    g_tModH.TxCount = 0;
    g_tModH.TxBuf[g_tModH.TxCount++] = _addr;			/* ��վ��ַ */
    g_tModH.TxBuf[g_tModH.TxCount++] = 0x05;			/* ������ */
    g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;		/* �Ĵ������ ���ֽ� */
    g_tModH.TxBuf[g_tModH.TxCount++] = _reg;			/* �Ĵ������ ���ֽ� */
    g_tModH.TxBuf[g_tModH.TxCount++] = _value >> 8;		/* �Ĵ���ֵ ���ֽ� */
    g_tModH.TxBuf[g_tModH.TxCount++] = _value;			/* �Ĵ���ֵ ���ֽ� */

    MODH_SendAckWithCRC();		/* �������ݣ��Զ���CRC */

    g_tModH.fAck05H = 0;		/* ����յ��ӻ���Ӧ���������־����Ϊ1 */
}

/*
*********************************************************************************************************
*	�� �� ��: MODH_Send06H
*	����˵��: ����06Hָ�д1�����ּĴ���
*	��    ��: _addr : ��վ��ַ
*			  _reg : �Ĵ������
*			  _value : �Ĵ���ֵ,2�ֽ�
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MODH_Send06H(uint8_t _addr, uint16_t _reg, uint16_t _value)
{
    g_tModH.TxCount = 0;
    g_tModH.TxBuf[g_tModH.TxCount++] = _addr;			/* ��վ��ַ */
    g_tModH.TxBuf[g_tModH.TxCount++] = 0x06;			/* ������ */
    g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;		/* �Ĵ������ ���ֽ� */
    g_tModH.TxBuf[g_tModH.TxCount++] = _reg;			/* �Ĵ������ ���ֽ� */
    g_tModH.TxBuf[g_tModH.TxCount++] = _value >> 8;		/* �Ĵ���ֵ ���ֽ� */
    g_tModH.TxBuf[g_tModH.TxCount++] = _value;			/* �Ĵ���ֵ ���ֽ� */

    MODH_SendAckWithCRC();		/* �������ݣ��Զ���CRC */

    g_tModH.fAck06H = 0;		/* ����յ��ӻ���Ӧ���������־����Ϊ1 */
}

/*
*********************************************************************************************************
*	�� �� ��: MODH_Send10H
*	����˵��: ����10Hָ�����д������ּĴ���. ���һ��֧��23���Ĵ�����
*	��    ��: _addr : ��վ��ַ
*			  _reg : �Ĵ������
*			  _num : �Ĵ�������n (ÿ���Ĵ���2���ֽ�) ֵ��
*			  _buf : n���Ĵ��������ݡ����� = 2 * n
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MODH_Send10H(uint8_t _addr, uint16_t _reg, uint8_t _num, uint8_t *_buf)
{
    uint16_t i;

    g_tModH.TxCount = 0;
    g_tModH.TxBuf[g_tModH.TxCount++] = _addr;		/* ��վ��ַ */
    g_tModH.TxBuf[g_tModH.TxCount++] = 0x10;		/* ��վ��ַ */
    g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;	/* �Ĵ������ ���ֽ� */
    g_tModH.TxBuf[g_tModH.TxCount++] = _reg;		/* �Ĵ������ ���ֽ� */
    g_tModH.TxBuf[g_tModH.TxCount++] = _num >> 8;	/* �Ĵ������� ���ֽ� */
    g_tModH.TxBuf[g_tModH.TxCount++] = _num;		/* �Ĵ������� ���ֽ� */
    g_tModH.TxBuf[g_tModH.TxCount++] = 2 * _num;	/* �����ֽ��� */

    for (i = 0; i < 2 * _num; i++)
    {
        if (g_tModH.TxCount > H_RX_BUF_SIZE - 3)
        {
            return;		/* ���ݳ������������ȣ�ֱ�Ӷ��������� */
        }
        g_tModH.TxBuf[g_tModH.TxCount++] = _buf[i];		/* ��������ݳ��� */
    }

    MODH_SendAckWithCRC();	/* �������ݣ��Զ���CRC */
}
static void MODH_Read_01H(void)
{
	uint8_t bytes,i;
    uint8_t *p;
	if(g_tModH.RxCount>=7){
		
		bytes = g_tModH.RxBuf[2];	/* ���ݳ��� �ֽ��� */
		for(i=0;i<bytes/2;i++)
		{
			
		}
		g_tModH.fAck01H = 1;
	}
}
static void MODH_Read_02H(void)
{
	
}
static void MODH_Read_03H(void)
{
	uint8_t bytes,i;
    uint8_t *p;
	if(g_tModH.RxCount>=7){
		
		bytes = g_tModH.RxBuf[2];	/* ���ݳ��� �ֽ��� */
		for(i=0;i<bytes/2;i++)
		{
			g_tvar_host.DR[i]=BEBufToUint16(p);	//modbus Э����ô��ģʽ��������
			p+=2;
		}
		g_tModH.fAck03H = 1;
	}
}
/*
*********************************************************************************************************
*	�� �� ��: MODH_Read_04H
*	����˵��: ����04Hָ���Ӧ������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void MODH_Read_04H(void)
{
    uint8_t bytes,i;
    uint8_t *p;
	printf("\nMODH_Read_04H");
    if (g_tModH.RxCount > 0)
    {
        bytes = g_tModH.RxBuf[2];	/* ���ݳ��� �ֽ��� */

        if(bytes<=100)
        {
            p = &g_tModH.RxBuf[3];
            g_tModH.fAck04H = 1;
        }


    }
}
/*
*********************************************************************************************************
*	�� �� ��: MODH_Poll
*	����˵��: ���տ�����ָ��. 1ms ��Ӧʱ�䡣
*	��    ��: ��
*	�� �� ֵ: 0 ��ʾ������ 1��ʾ�յ���ȷ����
*********************************************************************************************************
*/
void MODH_Poll(void)
{
    uint16_t crc1;

    if (g_modh_timeout == 0)	/* ����3.5���ַ�ʱ���ִ��MODH_RxTimeOut()������ȫ�ֱ��� g_rtu_timeout = 1 */
    {
        /* û�г�ʱ���������ա���Ҫ���� g_tModH.RxCount */
        return ;
    }

    /* �յ�����
    	05 06 00 88 04 57 3B70 (8 �ֽ�)
    		05    :  ��������ĺ�վ��
    		06    :  ָ��
    		00 88 :  �����������ʾ�Ĵ���
    		04 57 :  ����,,,ת���� 10 ������ 1111.��λ��ǰ,
    		3B70  :  �����ֽ� CRC ��	��05�� 57��У��
    */
    g_modh_timeout = 0;

    if (g_tModH.RxCount < 4){
        goto err_ret;
    }

    /* ����CRCУ��� */
    crc1 = CRC16_Modbus(g_tModH.RxBuf, g_tModH.RxCount);
    if (crc1 != 0){
        goto err_ret;
    }
	printf("crc ok");
    /* ����Ӧ�ò�Э�� */
    MODH_AnalyzeApp();

err_ret:
    g_tModH.RxCount = 0;	/* ��������������������´�֡ͬ�� */
}
void MODH_AnalyzeApp(void)
{
	switch (g_tModH.RxBuf[1])			/* ��2���ֽ� ������ */
	{
		case 0x01:
			MODH_Read_01H();
			break;
		case 0x02:
			break;
		case 0x03:
			break;
		case 0x04:
			MODH_Read_04H();
			break;
		case 0x05:
			break;
		case 0x06:
			break;
		case 0x10:
			break;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: MODH_ReadParam_01H
*	����˵��: ��������. ͨ������01Hָ��ʵ�֣�����֮�󣬵ȴ��ӻ�Ӧ��
*	��    ��: ��
*	�� �� ֵ: 1 ��ʾ�ɹ���0 ��ʾʧ�ܣ�ͨ�ų�ʱ�򱻾ܾ���
*********************************************************************************************************
*/
uint8_t MODH_ReadParam_01H(uint8_t addr,uint16_t _reg, uint16_t _num)
{
	int32_t time1;
    uint8_t i;
	
	for (i = 0; i < NUM; i++)
	{
		MODH_Send01H (addr, _reg, _num);		  /* �������� */
		time1 = bsp_GetRunTime();	/* ��¼����͵�ʱ�� */
		while(1)
		{
//			bsp_Idle();
			MODH_AnalyzeApp();
			if (bsp_CheckRunTime(time1) > TIMEOUT){
                break;		/* ͨ�ų�ʱ�� */
            }
			if (g_tModH.fAck01H > 0){
                break;		/* ���յ�Ӧ�� */
            }
			
		}
		if (g_tModH.fAck01H > 0){
            break;			/* ѭ��NUM�Σ�������յ�������breakѭ�� */
        }	
	}
	if (g_tModH.fAck01H == 0){
        return 0;
    }else{
        return 1;	/* 01H ���ɹ� */
    }
}
/*
*********************************************************************************************************
*	�� �� ��: MODH_ReadParam_04H
*	����˵��: ��������. ͨ������04Hָ��ʵ�֣�����֮�󣬵ȴ��ӻ�Ӧ��
*	��    ��: ��
*	�� �� ֵ: 1 ��ʾ�ɹ���0 ��ʾʧ�ܣ�ͨ�ų�ʱ�򱻾ܾ���
*********************************************************************************************************
*/
uint8_t MODH_ReadParam_04H(uint8_t _addr,uint16_t _reg, uint16_t _num)
{
    int32_t time1;
    uint8_t i;

    for (i = 0; i < NUM; i++)
    {
        MODH_Send04H (_addr, _reg, _num);
        time1 = bsp_GetRunTime();	/* ��¼����͵�ʱ�� */
//        printf("\r\nRunTime:%d",time1);
        while (1)
        {
            MODH_Poll();

            if (bsp_CheckRunTime(time1) > TIMEOUT)
            {
                break;		/* ͨ�ų�ʱ�� */
            }

            if (g_tModH.fAck04H > 0)
            {
                break;
            }
        }

        if (g_tModH.fAck04H > 0)
        {
            break;
        }
    }

    if (g_tModH.fAck04H == 0)
    {
        return 0;	/* ͨ�ų�ʱ�� */
    }
    else
    {
        return 1;	/* 04H ���ɹ� */
    }
}
