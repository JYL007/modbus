#include "modbus_slave.h"
#include "bsp_rs485.h"
#include "bsp_modbus_crc16.h"
#include "bsp_timer.h"
#include "bsp_dataconvert.h"

#include<stdio.h>
#include<string.h>
#include<stdlib.h>


static uint8_t g_mods_timeout = 0;
MODS_T g_tModS;     //modbusЭ���
VAR_SLAVE g_tVarS;   //����modbus�Ĵ������ò���
/*
*********************************************************************************************************
*	�� �� ��: MODS_RxTimeOut
*	����˵��: ����3.5���ַ�ʱ���ִ�б������� ����ȫ�ֱ��� g_mods_timeout = 1; ֪ͨ������ʼ���롣
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void MODS_RxTimeOut(void)
{
    g_mods_timeout = 1;
	printf("\ntimeout");
}
/*
*********************************************************************************************************
*	�� �� ��: MODS_ReciveNew
*	����˵��: ���ڽ����жϷ���������ñ����������յ�һ���ֽ�ʱ��ִ��һ�α�������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MODS_ReciveNew(uint8_t _byte)
{
    uint32_t timeout;
	printf("%02X ",_byte);
    g_mods_timeout = 0;
    timeout = 36000000 / SBAUD485;			/* ���㳬ʱʱ�䣬��λus 35000000*/
    bsp_StartHardTimer(1, timeout, (void *)MODS_RxTimeOut);
    if (g_tModS.RxCount < S_RX_BUF_SIZE) {
        g_tModS.RxBuf[g_tModS.RxCount++] = _byte;
    }

}

void MODS_SendPacket(uint8_t *_buf, uint16_t _len)
{
	uint8_t i;
//	for(i=0;i<_len;i++)
//	{
//		printf("%02X ",_buf[i]);
//	}
    RS485_Slave_SendBuf(_buf, _len);
}

static void MODS_SendWithCRC(uint8_t *_pBuf, uint8_t _ucLen)
{
    uint16_t crc;
    uint8_t buf[S_TX_BUF_SIZE];

    memcpy(buf, _pBuf, _ucLen);
    crc = CRC16_Modbus(_pBuf, _ucLen);
    buf[_ucLen++] = crc >> 8;
    buf[_ucLen++] = crc;
//    comSendBuf(COM1, buf, _ucLen);
    MODS_SendPacket(buf, _ucLen);



}


/*
*********************************************************************************************************
*	�� �� ��: MODS_SendAckErr
*	����˵��: ���ʹ���Ӧ��
*	��    ��: _ucErrCode : �������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void MODS_SendAckErr(uint8_t _ucErrCode)
{
    uint8_t txbuf[3];

    txbuf[0] = g_tModS.RxBuf[0];					/* 485��ַ */
    txbuf[1] = g_tModS.RxBuf[1] | 0x80;				/* �쳣�Ĺ����� */
    txbuf[2] = _ucErrCode;							/* �������(01,02,03,04) */

    MODS_SendWithCRC(txbuf, 3);
}

/*
*********************************************************************************************************
*	�� �� ��: MODS_SendAckOk
*	����˵��: ������ȷ��Ӧ��.
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void MODS_SendAckOk(void)
{
    uint8_t txbuf[6];
    uint8_t i;

    for (i = 0; i < 6; i++)
    {
        txbuf[i] = g_tModS.RxBuf[i];
    }
    MODS_SendWithCRC(txbuf, 6);
}

/*
*********************************************************************************************************
*	�� �� ��: MODS_ReadRegValue
*	����˵��: ��ȡ���ּĴ�����ֵ
*	��    ��: reg_addr �Ĵ�����ַ
*			  reg_value ��żĴ������
*	�� �� ֵ: 1��ʾOK 0��ʾ����
*********************************************************************************************************
*/
static uint8_t MODS_ReadRegValue(uint16_t reg_addr, uint8_t *reg_value)
{
    uint16_t value;
    value = g_tVarS.DR[reg_addr];
    reg_value[0] = value >> 8;
    reg_value[1] = value;

    return 1;											/* ��ȡ�ɹ� */
}

/*
*********************************************************************************************************
*	�� �� ��: MODS_WriteRegValue
*	����˵��: ��ȡ���ּĴ�����ֵ
*	��    ��: reg_addr �Ĵ�����ַ
*			  reg_value �Ĵ���ֵ
*	�� �� ֵ: 1��ʾOK 0��ʾ����
*********************************************************************************************************
*/
static uint8_t MODS_WriteRegValue(uint16_t reg_addr, uint16_t reg_value)
{

    g_tVarS.DR[reg_addr] = reg_value;
//    STMFLASH_Write(MODBUS_REG_BASE_ADDR+reg_addr,&reg_value,1);

    return 1;		/* ��ȡ�ɹ� */
}
/*
*********************************************************************************************************
*	�� �� ��: MODS_01H
*	����˵��: ��ȡ��Ȧ״̬����ӦԶ�̿���D01/D02/D03��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void MODS_01H(void)
{
    uint16_t reg;
    uint16_t num;
    uint16_t i;
    uint16_t m;
    uint8_t status[10];

    g_tModS.RspCode = RSP_OK;

    if (g_tModS.RxCount != 8)
    {
        g_tModS.RspCode = RSP_ERR_VALUE;				/* ����ֵ����� */
        return;
    }
    reg = BEBufToUint16(&g_tModS.RxBuf[2]); 			/* �Ĵ����� */
    num = BEBufToUint16(&g_tModS.RxBuf[4]);				/* �Ĵ������� */

    m = (num + 7) / 8;

    if ((reg >= REG_D01) && (num > 0) && (reg + num <= REG_DXX + 1))
    {
        for (i = 0; i < m; i++)
        {
            status[i] = 0;
        }
        for (i = 0; i < num; i++)
        {
//			if (bsp_IsLedOn(i + 1 + reg - REG_D01))		/* ��LED��״̬��д��״̬�Ĵ�����ÿһλ */
//			{
//				status[i / 8] |= (1 << (i % 8));
//			}
        }
    }
    else
    {
        g_tModS.RspCode = RSP_ERR_REG_ADDR;				/* �Ĵ�����ַ���� */
    }

    if (g_tModS.RspCode == RSP_OK)						/* ��ȷӦ�� */
    {
        g_tModS.TxCount = 0;
        g_tModS.TxBuf[g_tModS.TxCount++] = g_tModS.RxBuf[0];
        g_tModS.TxBuf[g_tModS.TxCount++] = g_tModS.RxBuf[1];
        g_tModS.TxBuf[g_tModS.TxCount++] = m;			/* �����ֽ��� */

        for (i = 0; i < m; i++)
        {
            g_tModS.TxBuf[g_tModS.TxCount++] = status[i];	/* �̵���״̬ */
        }

        MODS_SendWithCRC(g_tModS.TxBuf, g_tModS.TxCount);
    }
    else
    {
        MODS_SendAckErr(g_tModS.RspCode);				/* ��������������� */
    }
}
/*
*********************************************************************************************************
*	�� �� ��: MODS_02H
*	����˵��: ��ȡ����״̬����ӦK01��K03��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void MODS_02H(void)
{
    uint16_t reg;
    uint16_t num;
    uint16_t i;
    uint16_t m;
    uint8_t status[10];

    g_tModS.RspCode = RSP_OK;

    if (g_tModS.RxCount != 8)
    {
        g_tModS.RspCode = RSP_ERR_VALUE;				/* ����ֵ����� */
        return;
    }

    reg = BEBufToUint16(&g_tModS.RxBuf[2]); 			/* �Ĵ����� */
    num = BEBufToUint16(&g_tModS.RxBuf[4]);				/* �Ĵ������� */

    m = (num + 7) / 8;
    if ((reg >= REG_T01) && (num > 0) && (reg + num <= REG_TXX + 1))
    {
        for (i = 0; i < m; i++)
        {
            status[i] = 0;
        }
        for (i = 0; i < num; i++)
        {
//			if (bsp_GetKeyState((KEY_ID_E)(KID_K1 + reg - REG_T01 + i)))
//			{
//				status[i / 8] |= (1 << (i % 8));
//			}
        }
    }
    else
    {
        g_tModS.RspCode = RSP_ERR_REG_ADDR;				/* �Ĵ�����ַ���� */
    }

    if (g_tModS.RspCode == RSP_OK)						/* ��ȷӦ�� */
    {
        g_tModS.TxCount = 0;
        g_tModS.TxBuf[g_tModS.TxCount++] = g_tModS.RxBuf[0];
        g_tModS.TxBuf[g_tModS.TxCount++] = g_tModS.RxBuf[1];
        g_tModS.TxBuf[g_tModS.TxCount++] = m;			/* �����ֽ��� */

        for (i = 0; i < m; i++)
        {
            g_tModS.TxBuf[g_tModS.TxCount++] = status[i];	/* T01-02״̬ */
        }
        MODS_SendWithCRC(g_tModS.TxBuf, g_tModS.TxCount);
    }
    else
    {
        MODS_SendAckErr(g_tModS.RspCode);				/* ��������������� */
    }
}
/*
*********************************************************************************************************
*	�� �� ��: MODS_03H
*	����˵��: ��ȡ���ּĴ��� ��һ���������ּĴ�����ȡ�õ�ǰ�Ķ�����ֵ
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void MODS_03H(void)
{
	uint16_t reg;
    uint16_t num;
    uint16_t i;
    uint8_t reg_value[64];
	printf("\r\nMODS 03");
    g_tModS.RspCode = RSP_OK;

    if (g_tModS.RxCount != 8)								/* 03H���������8���ֽ� */
    {
        g_tModS.RspCode = RSP_ERR_VALUE;					/* ����ֵ����� */
        goto err_ret;
    }

    reg = BEBufToUint16(&g_tModS.RxBuf[2]); 				/* �Ĵ����� */
    num = BEBufToUint16(&g_tModS.RxBuf[4]);					/* �Ĵ������� */
    if (num > sizeof(reg_value) / 2)
    {
        g_tModS.RspCode = RSP_ERR_VALUE;					/* ����ֵ����� */
        goto err_ret;
    }

    for (i = 0; i < num; i++)
    {
        if (MODS_ReadRegValue(reg, &reg_value[2 * i]) == 0)	/* �����Ĵ���ֵ����reg_value */
        {
            g_tModS.RspCode = RSP_ERR_REG_ADDR;				/* �Ĵ�����ַ���� */
            break;
        }
        reg++;
    }

err_ret:
    if (g_tModS.RspCode == RSP_OK)							/* ��ȷӦ�� */
    {
			printf("\r\nMODS 03 OK");
        g_tModS.TxCount = 0;
        g_tModS.TxBuf[g_tModS.TxCount++] = g_tModS.RxBuf[0];
        g_tModS.TxBuf[g_tModS.TxCount++] = g_tModS.RxBuf[1];
        g_tModS.TxBuf[g_tModS.TxCount++] = num * 2;			/* �����ֽ��� */

        for (i = 0; i < num; i++)
        {
            g_tModS.TxBuf[g_tModS.TxCount++] = reg_value[2 * i];
            g_tModS.TxBuf[g_tModS.TxCount++] = reg_value[2 * i + 1];
        }
        MODS_SendWithCRC(g_tModS.TxBuf, g_tModS.TxCount);	/* ������ȷӦ�� */
    }
    else
    {
        MODS_SendAckErr(g_tModS.RspCode);					/* ���ʹ���Ӧ�� */
    }
}
/*
*********************************************************************************************************
*	�� �� ��: MODS_04H
*	����˵��: ��ȡ����Ĵ�������ӦA01/A02�� SMA
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void MODS_04H(void)
{
	uint16_t reg;
    uint16_t num;
    uint16_t i;
    uint16_t status[10];

    memset(status, 0, 10);

    g_tModS.RspCode = RSP_OK;

    if (g_tModS.RxCount != 8)
    {
        g_tModS.RspCode = RSP_ERR_VALUE;	/* ����ֵ����� */
        goto err_ret;
    }

    reg = BEBufToUint16(&g_tModS.RxBuf[2]); 	/* �Ĵ����� */
    num = BEBufToUint16(&g_tModS.RxBuf[4]);	/* �Ĵ������� */

    if ((reg >= REG_A01) && (num > 0) && (reg + num <= REG_AXX + 1))
    {
        for (i = 0; i < num; i++)
        {
            switch (reg)
            {
            /* ���Բ��� */
            case REG_A01:
                status[i] = g_tVarS.A01;
                break;

            default:
                status[i] = 0;
                break;
            }
            reg++;
        }
    }
    else
    {
        g_tModS.RspCode = RSP_ERR_REG_ADDR;		/* �Ĵ�����ַ���� */
    }

err_ret:
    if (g_tModS.RspCode == RSP_OK)		/* ��ȷӦ�� */
    {
        g_tModS.TxCount = 0;
        g_tModS.TxBuf[g_tModS.TxCount++] = g_tModS.RxBuf[0];
        g_tModS.TxBuf[g_tModS.TxCount++] = g_tModS.RxBuf[1];
        g_tModS.TxBuf[g_tModS.TxCount++] = num * 2;			/* �����ֽ��� */

        for (i = 0; i < num; i++)
        {
            g_tModS.TxBuf[g_tModS.TxCount++] = status[i] >> 8;
            g_tModS.TxBuf[g_tModS.TxCount++] = status[i] & 0xFF;
        }
        MODS_SendWithCRC(g_tModS.TxBuf, g_tModS.TxCount);
    }
    else
    {
        MODS_SendAckErr(g_tModS.RspCode);	/* ��������������� */
    }
}
/*
*********************************************************************************************************
*	�� �� ��: MODS_05H
*	����˵��: ǿ�Ƶ���Ȧ����ӦD01/D02/D03��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void MODS_05H(void)
{
	uint16_t reg;
    uint16_t value;

    g_tModS.RspCode = RSP_OK;

    if (g_tModS.RxCount != 8)
    {
        g_tModS.RspCode = RSP_ERR_VALUE;		/* ����ֵ����� */
        goto err_ret;
    }

    reg = BEBufToUint16(&g_tModS.RxBuf[2]); 	/* �Ĵ����� */
    value = BEBufToUint16(&g_tModS.RxBuf[4]);	/* ���� */

    if (value != 0 && value != 1)
    {
        g_tModS.RspCode = RSP_ERR_VALUE;		/* ����ֵ����� */
        goto err_ret;
    }

    if (reg == REG_D01)
    {
        g_tVarS.D01 = value;
        
    }
    else if (reg == REG_D02)
    {
        g_tVarS.D02 = value;
        
    }
    else if (reg == REG_D03)
    {
        g_tVarS.D03 = value;
        
    }

    else
    {
        g_tModS.RspCode = RSP_ERR_REG_ADDR;		/* �Ĵ�����ַ���� */
    }
err_ret:
    if (g_tModS.RspCode == RSP_OK)				/* ��ȷӦ�� */
    {
        MODS_SendAckOk();
    }
    else
    {
        MODS_SendAckErr(g_tModS.RspCode);		/* ��������������� */
    }
}
/*
*********************************************************************************************************
*	�� �� ��: MODS_06H
*	����˵��: д�����Ĵ���
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void MODS_06H(void)
{
	uint16_t reg;
    uint16_t value;

    g_tModS.RspCode = RSP_OK;

    if (g_tModS.RxCount != 8)
    {
        g_tModS.RspCode = RSP_ERR_VALUE;		/* ����ֵ����� */
        goto err_ret;
    }

    reg = BEBufToUint16(&g_tModS.RxBuf[2]); 	/* �Ĵ����� */
    value = BEBufToUint16(&g_tModS.RxBuf[4]);	/* �Ĵ���ֵ */
    if (MODS_WriteRegValue(reg, value) == 1)	/* �ú������д���ֵ����Ĵ��� */
    {
        
    } else {
        g_tModS.RspCode = RSP_ERR_REG_ADDR;		/* �Ĵ�����ַ���� */
    }
//    if (MODS_WriteRegValue(reg, value) == 1)	/* �ú������д���ֵ����Ĵ��� */
//    {}else{
//        g_tModS.RspCode = RSP_ERR_REG_ADDR;		/* �Ĵ�����ַ���� */
//    }

err_ret:
    if (g_tModS.RspCode == RSP_OK)				/* ��ȷӦ�� */
    {
        MODS_SendAckOk();
    }
    else
    {
        MODS_SendAckErr(g_tModS.RspCode);		/* ��������������� */
    }
}
/*
*********************************************************************************************************
*	�� �� ��: MODS_10H
*	����˵��: ����д����Ĵ���.  �����ڸ�дʱ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void MODS_10H(void)
{
	uint16_t reg_addr;
    uint16_t reg_num;
    uint8_t byte_num;
    uint8_t i;
    uint16_t value;


    g_tModS.RspCode = RSP_OK;

    if (g_tModS.RxCount < 11)
    {
        g_tModS.RspCode = RSP_ERR_VALUE;			/* ����ֵ����� */
        goto err_ret;
    }

    reg_addr = BEBufToUint16(&g_tModS.RxBuf[2]); 	/* �Ĵ����� */
    reg_num = BEBufToUint16(&g_tModS.RxBuf[4]);		/* �Ĵ������� */
    byte_num = g_tModS.RxBuf[6];					/* ������������ֽ��� */
    if (byte_num != 2 * reg_num) {
        ;
    }
    for (i = 0; i < reg_num; i++)
    {
        value = BEBufToUint16(&g_tModS.RxBuf[7 + 2 * i]);	/* �Ĵ���ֵ */
        if (MODS_WriteRegValue(reg_addr + i, value) == 1)
        {
            ;
        }
        else
        {
            g_tModS.RspCode = RSP_ERR_REG_ADDR;		/* �Ĵ�����ַ���� */
            break;
        }
    }
    
err_ret:
    if (g_tModS.RspCode == RSP_OK)					/* ��ȷӦ�� */
    {
        MODS_SendAckOk();
    }
    else
    {
        MODS_SendAckErr(g_tModS.RspCode);			/* ��������������� */
    }
}
/*
*********************************************************************************************************
*	�� �� ��: MODS_AnalyzeApp
*	����˵��: ����Ӧ�ò�Э��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/

static void MODS_AnalyzeApp(void)
{
    switch (g_tModS.RxBuf[1])				/* ��2���ֽ� ������ */
    {
    case 0x01:
        MODS_01H();
        break;
    case 0x02:
        MODS_02H();
        break;
    case 0x03:
        MODS_03H();
        break;
    case 0x04:
        MODS_04H();
        break;
    case 0x05:
        MODS_05H();
        break;
    case 0x06:
        MODS_06H();
        break;
    case 0x10:
        MODS_10H();
        break;
    default:
        g_tModS.RspCode = RSP_ERR_CMD;
        MODS_SendAckErr(g_tModS.RspCode);	/* ��������������� */
        break;
    }
}
/*
*********************************************************************************************************
*	�� �� ��: MODS_Poll
*	����˵��: �������ݰ�. �����������������á�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MODS_Poll(void)
{
    uint16_t addr;
    uint16_t crc1;

    /* ����3.5���ַ�ʱ���ִ��MODH_RxTimeOut()������ȫ�ֱ��� g_rtu_timeout = 1; ֪ͨ������ʼ���� */
    if (g_mods_timeout == 0)
    {
        return;								/* û�г�ʱ���������ա���Ҫ���� g_tModS.RxCount */
    }
    g_mods_timeout = 0;	 					/* ���־ */


    if (g_tModS.RxCount < 4)				/* ���յ�������С��4���ֽھ���Ϊ���� */
    {
        goto err_ret;
    }
    /* ����CRCУ��� */
    crc1 = CRC16_Modbus(g_tModS.RxBuf, g_tModS.RxCount);
    if (crc1 != 0)
    {
        printf("\r\ncrc check failed");
        goto err_ret;
    }
    addr = g_tModS.RxBuf[0];				/* ��1�ֽ� վ�� */
    if (addr != SADDR485) {   /* �ж��������͵������ַ�Ƿ���� */
        goto err_ret;
    }
//    else if(addr==0){
//
//    }
    /* ����Ӧ�ò�Э�� */
    MODS_AnalyzeApp();
err_ret:
    g_tModS.RxCount = 0;					/* ��������������������´�֡ͬ�� */
}

