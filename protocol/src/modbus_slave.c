#include "modbus_slave.h"
#include "bsp_rs485.h"
#include "bsp_modbus_crc16.h"
#include "bsp_timer.h"
#include "bsp_dataconvert.h"

#include<stdio.h>
#include<string.h>
#include<stdlib.h>


static uint8_t g_mods_timeout = 0;
MODS_T g_tModS;     //modbus协议包
VAR_SLAVE g_tVarS;   //网关modbus寄存器配置参数
/*
*********************************************************************************************************
*	函 数 名: MODS_RxTimeOut
*	功能说明: 超过3.5个字符时间后执行本函数。 设置全局变量 g_mods_timeout = 1; 通知主程序开始解码。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void MODS_RxTimeOut(void)
{
    g_mods_timeout = 1;
	printf("\ntimeout");
}
/*
*********************************************************************************************************
*	函 数 名: MODS_ReciveNew
*	功能说明: 串口接收中断服务程序会调用本函数。当收到一个字节时，执行一次本函数。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void MODS_ReciveNew(uint8_t _byte)
{
    uint32_t timeout;
	printf("%02X ",_byte);
    g_mods_timeout = 0;
    timeout = 36000000 / SBAUD485;			/* 计算超时时间，单位us 35000000*/
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
*	函 数 名: MODS_SendAckErr
*	功能说明: 发送错误应答
*	形    参: _ucErrCode : 错误代码
*	返 回 值: 无
*********************************************************************************************************
*/
static void MODS_SendAckErr(uint8_t _ucErrCode)
{
    uint8_t txbuf[3];

    txbuf[0] = g_tModS.RxBuf[0];					/* 485地址 */
    txbuf[1] = g_tModS.RxBuf[1] | 0x80;				/* 异常的功能码 */
    txbuf[2] = _ucErrCode;							/* 错误代码(01,02,03,04) */

    MODS_SendWithCRC(txbuf, 3);
}

/*
*********************************************************************************************************
*	函 数 名: MODS_SendAckOk
*	功能说明: 发送正确的应答.
*	形    参: 无
*	返 回 值: 无
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
*	函 数 名: MODS_ReadRegValue
*	功能说明: 读取保持寄存器的值
*	形    参: reg_addr 寄存器地址
*			  reg_value 存放寄存器结果
*	返 回 值: 1表示OK 0表示错误
*********************************************************************************************************
*/
static uint8_t MODS_ReadRegValue(uint16_t reg_addr, uint8_t *reg_value)
{
    uint16_t value;
    value = g_tVarS.DR[reg_addr];
    reg_value[0] = value >> 8;
    reg_value[1] = value;

    return 1;											/* 读取成功 */
}

/*
*********************************************************************************************************
*	函 数 名: MODS_WriteRegValue
*	功能说明: 读取保持寄存器的值
*	形    参: reg_addr 寄存器地址
*			  reg_value 寄存器值
*	返 回 值: 1表示OK 0表示错误
*********************************************************************************************************
*/
static uint8_t MODS_WriteRegValue(uint16_t reg_addr, uint16_t reg_value)
{

    g_tVarS.DR[reg_addr] = reg_value;
//    STMFLASH_Write(MODBUS_REG_BASE_ADDR+reg_addr,&reg_value,1);

    return 1;		/* 读取成功 */
}
/*
*********************************************************************************************************
*	函 数 名: MODS_01H
*	功能说明: 读取线圈状态（对应远程开关D01/D02/D03）
*	形    参: 无
*	返 回 值: 无
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
        g_tModS.RspCode = RSP_ERR_VALUE;				/* 数据值域错误 */
        return;
    }
    reg = BEBufToUint16(&g_tModS.RxBuf[2]); 			/* 寄存器号 */
    num = BEBufToUint16(&g_tModS.RxBuf[4]);				/* 寄存器个数 */

    m = (num + 7) / 8;

    if ((reg >= REG_D01) && (num > 0) && (reg + num <= REG_DXX + 1))
    {
        for (i = 0; i < m; i++)
        {
            status[i] = 0;
        }
        for (i = 0; i < num; i++)
        {
//			if (bsp_IsLedOn(i + 1 + reg - REG_D01))		/* 读LED的状态，写入状态寄存器的每一位 */
//			{
//				status[i / 8] |= (1 << (i % 8));
//			}
        }
    }
    else
    {
        g_tModS.RspCode = RSP_ERR_REG_ADDR;				/* 寄存器地址错误 */
    }

    if (g_tModS.RspCode == RSP_OK)						/* 正确应答 */
    {
        g_tModS.TxCount = 0;
        g_tModS.TxBuf[g_tModS.TxCount++] = g_tModS.RxBuf[0];
        g_tModS.TxBuf[g_tModS.TxCount++] = g_tModS.RxBuf[1];
        g_tModS.TxBuf[g_tModS.TxCount++] = m;			/* 返回字节数 */

        for (i = 0; i < m; i++)
        {
            g_tModS.TxBuf[g_tModS.TxCount++] = status[i];	/* 继电器状态 */
        }

        MODS_SendWithCRC(g_tModS.TxBuf, g_tModS.TxCount);
    }
    else
    {
        MODS_SendAckErr(g_tModS.RspCode);				/* 告诉主机命令错误 */
    }
}
/*
*********************************************************************************************************
*	函 数 名: MODS_02H
*	功能说明: 读取输入状态（对应K01～K03）
*	形    参: 无
*	返 回 值: 无
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
        g_tModS.RspCode = RSP_ERR_VALUE;				/* 数据值域错误 */
        return;
    }

    reg = BEBufToUint16(&g_tModS.RxBuf[2]); 			/* 寄存器号 */
    num = BEBufToUint16(&g_tModS.RxBuf[4]);				/* 寄存器个数 */

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
        g_tModS.RspCode = RSP_ERR_REG_ADDR;				/* 寄存器地址错误 */
    }

    if (g_tModS.RspCode == RSP_OK)						/* 正确应答 */
    {
        g_tModS.TxCount = 0;
        g_tModS.TxBuf[g_tModS.TxCount++] = g_tModS.RxBuf[0];
        g_tModS.TxBuf[g_tModS.TxCount++] = g_tModS.RxBuf[1];
        g_tModS.TxBuf[g_tModS.TxCount++] = m;			/* 返回字节数 */

        for (i = 0; i < m; i++)
        {
            g_tModS.TxBuf[g_tModS.TxCount++] = status[i];	/* T01-02状态 */
        }
        MODS_SendWithCRC(g_tModS.TxBuf, g_tModS.TxCount);
    }
    else
    {
        MODS_SendAckErr(g_tModS.RspCode);				/* 告诉主机命令错误 */
    }
}
/*
*********************************************************************************************************
*	函 数 名: MODS_03H
*	功能说明: 读取保持寄存器 在一个或多个保持寄存器中取得当前的二进制值
*	形    参: 无
*	返 回 值: 无
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

    if (g_tModS.RxCount != 8)								/* 03H命令必须是8个字节 */
    {
        g_tModS.RspCode = RSP_ERR_VALUE;					/* 数据值域错误 */
        goto err_ret;
    }

    reg = BEBufToUint16(&g_tModS.RxBuf[2]); 				/* 寄存器号 */
    num = BEBufToUint16(&g_tModS.RxBuf[4]);					/* 寄存器个数 */
    if (num > sizeof(reg_value) / 2)
    {
        g_tModS.RspCode = RSP_ERR_VALUE;					/* 数据值域错误 */
        goto err_ret;
    }

    for (i = 0; i < num; i++)
    {
        if (MODS_ReadRegValue(reg, &reg_value[2 * i]) == 0)	/* 读出寄存器值放入reg_value */
        {
            g_tModS.RspCode = RSP_ERR_REG_ADDR;				/* 寄存器地址错误 */
            break;
        }
        reg++;
    }

err_ret:
    if (g_tModS.RspCode == RSP_OK)							/* 正确应答 */
    {
			printf("\r\nMODS 03 OK");
        g_tModS.TxCount = 0;
        g_tModS.TxBuf[g_tModS.TxCount++] = g_tModS.RxBuf[0];
        g_tModS.TxBuf[g_tModS.TxCount++] = g_tModS.RxBuf[1];
        g_tModS.TxBuf[g_tModS.TxCount++] = num * 2;			/* 返回字节数 */

        for (i = 0; i < num; i++)
        {
            g_tModS.TxBuf[g_tModS.TxCount++] = reg_value[2 * i];
            g_tModS.TxBuf[g_tModS.TxCount++] = reg_value[2 * i + 1];
        }
        MODS_SendWithCRC(g_tModS.TxBuf, g_tModS.TxCount);	/* 发送正确应答 */
    }
    else
    {
        MODS_SendAckErr(g_tModS.RspCode);					/* 发送错误应答 */
    }
}
/*
*********************************************************************************************************
*	函 数 名: MODS_04H
*	功能说明: 读取输入寄存器（对应A01/A02） SMA
*	形    参: 无
*	返 回 值: 无
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
        g_tModS.RspCode = RSP_ERR_VALUE;	/* 数据值域错误 */
        goto err_ret;
    }

    reg = BEBufToUint16(&g_tModS.RxBuf[2]); 	/* 寄存器号 */
    num = BEBufToUint16(&g_tModS.RxBuf[4]);	/* 寄存器个数 */

    if ((reg >= REG_A01) && (num > 0) && (reg + num <= REG_AXX + 1))
    {
        for (i = 0; i < num; i++)
        {
            switch (reg)
            {
            /* 测试参数 */
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
        g_tModS.RspCode = RSP_ERR_REG_ADDR;		/* 寄存器地址错误 */
    }

err_ret:
    if (g_tModS.RspCode == RSP_OK)		/* 正确应答 */
    {
        g_tModS.TxCount = 0;
        g_tModS.TxBuf[g_tModS.TxCount++] = g_tModS.RxBuf[0];
        g_tModS.TxBuf[g_tModS.TxCount++] = g_tModS.RxBuf[1];
        g_tModS.TxBuf[g_tModS.TxCount++] = num * 2;			/* 返回字节数 */

        for (i = 0; i < num; i++)
        {
            g_tModS.TxBuf[g_tModS.TxCount++] = status[i] >> 8;
            g_tModS.TxBuf[g_tModS.TxCount++] = status[i] & 0xFF;
        }
        MODS_SendWithCRC(g_tModS.TxBuf, g_tModS.TxCount);
    }
    else
    {
        MODS_SendAckErr(g_tModS.RspCode);	/* 告诉主机命令错误 */
    }
}
/*
*********************************************************************************************************
*	函 数 名: MODS_05H
*	功能说明: 强制单线圈（对应D01/D02/D03）
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void MODS_05H(void)
{
	uint16_t reg;
    uint16_t value;

    g_tModS.RspCode = RSP_OK;

    if (g_tModS.RxCount != 8)
    {
        g_tModS.RspCode = RSP_ERR_VALUE;		/* 数据值域错误 */
        goto err_ret;
    }

    reg = BEBufToUint16(&g_tModS.RxBuf[2]); 	/* 寄存器号 */
    value = BEBufToUint16(&g_tModS.RxBuf[4]);	/* 数据 */

    if (value != 0 && value != 1)
    {
        g_tModS.RspCode = RSP_ERR_VALUE;		/* 数据值域错误 */
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
        g_tModS.RspCode = RSP_ERR_REG_ADDR;		/* 寄存器地址错误 */
    }
err_ret:
    if (g_tModS.RspCode == RSP_OK)				/* 正确应答 */
    {
        MODS_SendAckOk();
    }
    else
    {
        MODS_SendAckErr(g_tModS.RspCode);		/* 告诉主机命令错误 */
    }
}
/*
*********************************************************************************************************
*	函 数 名: MODS_06H
*	功能说明: 写单个寄存器
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void MODS_06H(void)
{
	uint16_t reg;
    uint16_t value;

    g_tModS.RspCode = RSP_OK;

    if (g_tModS.RxCount != 8)
    {
        g_tModS.RspCode = RSP_ERR_VALUE;		/* 数据值域错误 */
        goto err_ret;
    }

    reg = BEBufToUint16(&g_tModS.RxBuf[2]); 	/* 寄存器号 */
    value = BEBufToUint16(&g_tModS.RxBuf[4]);	/* 寄存器值 */
    if (MODS_WriteRegValue(reg, value) == 1)	/* 该函数会把写入的值存入寄存器 */
    {
        
    } else {
        g_tModS.RspCode = RSP_ERR_REG_ADDR;		/* 寄存器地址错误 */
    }
//    if (MODS_WriteRegValue(reg, value) == 1)	/* 该函数会把写入的值存入寄存器 */
//    {}else{
//        g_tModS.RspCode = RSP_ERR_REG_ADDR;		/* 寄存器地址错误 */
//    }

err_ret:
    if (g_tModS.RspCode == RSP_OK)				/* 正确应答 */
    {
        MODS_SendAckOk();
    }
    else
    {
        MODS_SendAckErr(g_tModS.RspCode);		/* 告诉主机命令错误 */
    }
}
/*
*********************************************************************************************************
*	函 数 名: MODS_10H
*	功能说明: 连续写多个寄存器.  进用于改写时钟
*	形    参: 无
*	返 回 值: 无
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
        g_tModS.RspCode = RSP_ERR_VALUE;			/* 数据值域错误 */
        goto err_ret;
    }

    reg_addr = BEBufToUint16(&g_tModS.RxBuf[2]); 	/* 寄存器号 */
    reg_num = BEBufToUint16(&g_tModS.RxBuf[4]);		/* 寄存器个数 */
    byte_num = g_tModS.RxBuf[6];					/* 后面的数据体字节数 */
    if (byte_num != 2 * reg_num) {
        ;
    }
    for (i = 0; i < reg_num; i++)
    {
        value = BEBufToUint16(&g_tModS.RxBuf[7 + 2 * i]);	/* 寄存器值 */
        if (MODS_WriteRegValue(reg_addr + i, value) == 1)
        {
            ;
        }
        else
        {
            g_tModS.RspCode = RSP_ERR_REG_ADDR;		/* 寄存器地址错误 */
            break;
        }
    }
    
err_ret:
    if (g_tModS.RspCode == RSP_OK)					/* 正确应答 */
    {
        MODS_SendAckOk();
    }
    else
    {
        MODS_SendAckErr(g_tModS.RspCode);			/* 告诉主机命令错误 */
    }
}
/*
*********************************************************************************************************
*	函 数 名: MODS_AnalyzeApp
*	功能说明: 分析应用层协议
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/

static void MODS_AnalyzeApp(void)
{
    switch (g_tModS.RxBuf[1])				/* 第2个字节 功能码 */
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
        MODS_SendAckErr(g_tModS.RspCode);	/* 告诉主机命令错误 */
        break;
    }
}
/*
*********************************************************************************************************
*	函 数 名: MODS_Poll
*	功能说明: 解析数据包. 在主程序中轮流调用。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void MODS_Poll(void)
{
    uint16_t addr;
    uint16_t crc1;

    /* 超过3.5个字符时间后执行MODH_RxTimeOut()函数。全局变量 g_rtu_timeout = 1; 通知主程序开始解码 */
    if (g_mods_timeout == 0)
    {
        return;								/* 没有超时，继续接收。不要清零 g_tModS.RxCount */
    }
    g_mods_timeout = 0;	 					/* 清标志 */


    if (g_tModS.RxCount < 4)				/* 接收到的数据小于4个字节就认为错误 */
    {
        goto err_ret;
    }
    /* 计算CRC校验和 */
    crc1 = CRC16_Modbus(g_tModS.RxBuf, g_tModS.RxCount);
    if (crc1 != 0)
    {
        printf("\r\ncrc check failed");
        goto err_ret;
    }
    addr = g_tModS.RxBuf[0];				/* 第1字节 站号 */
    if (addr != SADDR485) {   /* 判断主机发送的命令地址是否符合 */
        goto err_ret;
    }
//    else if(addr==0){
//
//    }
    /* 分析应用层协议 */
    MODS_AnalyzeApp();
err_ret:
    g_tModS.RxCount = 0;					/* 必须清零计数器，方便下次帧同步 */
}

