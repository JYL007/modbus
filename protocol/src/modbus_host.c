#include "modbus_host.h"
#include "bsp_modbus_crc16.h"
#include "bsp_rs485.h"
#include "bsp_timer.h"
#include "bsp_dataconvert.h"

#include<stdio.h>


#define TIMEOUT		500		/* 接收命令超时时间, 单位ms */
#define NUM			1		/* 循环发送次数 */
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
*	函 数 名: MODH_SendPacket
*	功能说明: 发送数据包 COM1口
*	形    参: _buf : 数据缓冲区
*			  _len : 数据长度
*	返 回 值: 无
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
*	函 数 名: MODH_Send01H
*	功能说明: 发送01H指令，查询1个或多个保持寄存器
*	形    参: _addr : 从站地址
*			  _reg : 寄存器编号
*			  _num : 寄存器个数
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_Send01H(uint8_t _addr, uint16_t _reg, uint16_t _num)
{
    g_tModH.TxCount = 0;
    g_tModH.TxBuf[g_tModH.TxCount++] = _addr;		/* 从站地址 */
    g_tModH.TxBuf[g_tModH.TxCount++] = 0x01;		/* 功能码 */
    g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;	/* 寄存器编号 高字节 */
    g_tModH.TxBuf[g_tModH.TxCount++] = _reg;		/* 寄存器编号 低字节 */
    g_tModH.TxBuf[g_tModH.TxCount++] = _num >> 8;	/* 寄存器个数 高字节 */
    g_tModH.TxBuf[g_tModH.TxCount++] = _num;		/* 寄存器个数 低字节 */

    MODH_SendAckWithCRC();		/* 发送数据，自动加CRC */
    g_tModH.fAck01H = 0;		/* 清接收标志 */
    g_tModH.RegNum = _num;		/* 寄存器个数 */
    g_tModH.Reg01H = _reg;		/* 保存03H指令中的寄存器地址，方便对应答数据进行分类 */
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Send02H
*	功能说明: 发送02H指令，读离散输入寄存器
*	形    参: _addr : 从站地址
*			  _reg : 寄存器编号
*			  _num : 寄存器个数
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_Send02H(uint8_t _addr, uint16_t _reg, uint16_t _num)
{
    g_tModH.TxCount = 0;
    g_tModH.TxBuf[g_tModH.TxCount++] = _addr;		/* 从站地址 */
    g_tModH.TxBuf[g_tModH.TxCount++] = 0x02;		/* 功能码 */
    g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;	/* 寄存器编号 高字节 */
    g_tModH.TxBuf[g_tModH.TxCount++] = _reg;		/* 寄存器编号 低字节 */
    g_tModH.TxBuf[g_tModH.TxCount++] = _num >> 8;	/* 寄存器个数 高字节 */
    g_tModH.TxBuf[g_tModH.TxCount++] = _num;		/* 寄存器个数 低字节 */

    MODH_SendAckWithCRC();		/* 发送数据，自动加CRC */
    g_tModH.fAck02H = 0;		/* 清接收标志 */
    g_tModH.RegNum = _num;		/* 寄存器个数 */
    g_tModH.Reg02H = _reg;		/* 保存03H指令中的寄存器地址，方便对应答数据进行分类 */
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Send03H
*	功能说明: 发送03H指令，查询1个或多个保持寄存器
*	形    参: _addr : 从站地址
*			  _reg : 寄存器编号
*			  _num : 寄存器个数
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_Send03H(uint8_t _addr, uint16_t _reg, uint16_t _num)
{
    g_tModH.TxCount = 0;
    g_tModH.TxBuf[g_tModH.TxCount++] = _addr;		/* 从站地址 */
    g_tModH.TxBuf[g_tModH.TxCount++] = 0x03;		/* 功能码 */
    g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;	/* 寄存器编号 高字节 */
    g_tModH.TxBuf[g_tModH.TxCount++] = _reg;		/* 寄存器编号 低字节 */
    g_tModH.TxBuf[g_tModH.TxCount++] = _num >> 8;	/* 寄存器个数 高字节 */
    g_tModH.TxBuf[g_tModH.TxCount++] = _num;		/* 寄存器个数 低字节 */

    MODH_SendAckWithCRC();		/* 发送数据，自动加CRC */
    g_tModH.fAck03H = 0;		/* 清接收标志 */
    g_tModH.RegNum = _num;		/* 寄存器个数 */
    g_tModH.Reg03H = _reg;		/* 保存03H指令中的寄存器地址，方便对应答数据进行分类 */
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Send04H
*	功能说明: 发送04H指令，读输入寄存器
*	形    参: _addr : 从站地址
*			  _reg : 寄存器编号
*			  _num : 寄存器个数
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_Send04H(uint8_t _addr, uint16_t _reg, uint16_t _num)
{
    g_tModH.TxCount = 0;
    g_tModH.TxBuf[g_tModH.TxCount++] = _addr;		/* 从站地址 */
    g_tModH.TxBuf[g_tModH.TxCount++] = 0x04;		/* 功能码 */
    g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;	/* 寄存器编号 高字节 */
    g_tModH.TxBuf[g_tModH.TxCount++] = _reg;		/* 寄存器编号 低字节 */
    g_tModH.TxBuf[g_tModH.TxCount++] = _num >> 8;	/* 寄存器个数 高字节 */
    g_tModH.TxBuf[g_tModH.TxCount++] = _num;		/* 寄存器个数 低字节 */

    MODH_SendAckWithCRC();		/* 发送数据，自动加CRC */
    g_tModH.fAck04H = 0;		/* 清接收标志 */
    g_tModH.RegNum = _num;		/* 寄存器个数 */
    g_tModH.Reg04H = _reg;		/* 保存03H指令中的寄存器地址，方便对应答数据进行分类 */
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Send05H
*	功能说明: 发送05H指令，写强置单线圈
*	形    参: _addr : 从站地址
*			  _reg : 寄存器编号
*			  _value : 寄存器值,2字节
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_Send05H(uint8_t _addr, uint16_t _reg, uint16_t _value)
{
    g_tModH.TxCount = 0;
    g_tModH.TxBuf[g_tModH.TxCount++] = _addr;			/* 从站地址 */
    g_tModH.TxBuf[g_tModH.TxCount++] = 0x05;			/* 功能码 */
    g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;		/* 寄存器编号 高字节 */
    g_tModH.TxBuf[g_tModH.TxCount++] = _reg;			/* 寄存器编号 低字节 */
    g_tModH.TxBuf[g_tModH.TxCount++] = _value >> 8;		/* 寄存器值 高字节 */
    g_tModH.TxBuf[g_tModH.TxCount++] = _value;			/* 寄存器值 低字节 */

    MODH_SendAckWithCRC();		/* 发送数据，自动加CRC */

    g_tModH.fAck05H = 0;		/* 如果收到从机的应答，则这个标志会设为1 */
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Send06H
*	功能说明: 发送06H指令，写1个保持寄存器
*	形    参: _addr : 从站地址
*			  _reg : 寄存器编号
*			  _value : 寄存器值,2字节
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_Send06H(uint8_t _addr, uint16_t _reg, uint16_t _value)
{
    g_tModH.TxCount = 0;
    g_tModH.TxBuf[g_tModH.TxCount++] = _addr;			/* 从站地址 */
    g_tModH.TxBuf[g_tModH.TxCount++] = 0x06;			/* 功能码 */
    g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;		/* 寄存器编号 高字节 */
    g_tModH.TxBuf[g_tModH.TxCount++] = _reg;			/* 寄存器编号 低字节 */
    g_tModH.TxBuf[g_tModH.TxCount++] = _value >> 8;		/* 寄存器值 高字节 */
    g_tModH.TxBuf[g_tModH.TxCount++] = _value;			/* 寄存器值 低字节 */

    MODH_SendAckWithCRC();		/* 发送数据，自动加CRC */

    g_tModH.fAck06H = 0;		/* 如果收到从机的应答，则这个标志会设为1 */
}

/*
*********************************************************************************************************
*	函 数 名: MODH_Send10H
*	功能说明: 发送10H指令，连续写多个保持寄存器. 最多一次支持23个寄存器。
*	形    参: _addr : 从站地址
*			  _reg : 寄存器编号
*			  _num : 寄存器个数n (每个寄存器2个字节) 值域
*			  _buf : n个寄存器的数据。长度 = 2 * n
*	返 回 值: 无
*********************************************************************************************************
*/
void MODH_Send10H(uint8_t _addr, uint16_t _reg, uint8_t _num, uint8_t *_buf)
{
    uint16_t i;

    g_tModH.TxCount = 0;
    g_tModH.TxBuf[g_tModH.TxCount++] = _addr;		/* 从站地址 */
    g_tModH.TxBuf[g_tModH.TxCount++] = 0x10;		/* 从站地址 */
    g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;	/* 寄存器编号 高字节 */
    g_tModH.TxBuf[g_tModH.TxCount++] = _reg;		/* 寄存器编号 低字节 */
    g_tModH.TxBuf[g_tModH.TxCount++] = _num >> 8;	/* 寄存器个数 高字节 */
    g_tModH.TxBuf[g_tModH.TxCount++] = _num;		/* 寄存器个数 低字节 */
    g_tModH.TxBuf[g_tModH.TxCount++] = 2 * _num;	/* 数据字节数 */

    for (i = 0; i < 2 * _num; i++)
    {
        if (g_tModH.TxCount > H_RX_BUF_SIZE - 3)
        {
            return;		/* 数据超过缓冲区超度，直接丢弃不发送 */
        }
        g_tModH.TxBuf[g_tModH.TxCount++] = _buf[i];		/* 后面的数据长度 */
    }

    MODH_SendAckWithCRC();	/* 发送数据，自动加CRC */
}
static void MODH_Read_01H(void)
{
	uint8_t bytes,i;
    uint8_t *p;
	if(g_tModH.RxCount>=7){
		
		bytes = g_tModH.RxBuf[2];	/* 数据长度 字节数 */
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
		
		bytes = g_tModH.RxBuf[2];	/* 数据长度 字节数 */
		for(i=0;i<bytes/2;i++)
		{
			g_tvar_host.DR[i]=BEBufToUint16(p);	//modbus 协议采用大端模式传输数据
			p+=2;
		}
		g_tModH.fAck03H = 1;
	}
}
/*
*********************************************************************************************************
*	函 数 名: MODH_Read_04H
*	功能说明: 分析04H指令的应答数据
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void MODH_Read_04H(void)
{
    uint8_t bytes,i;
    uint8_t *p;
	printf("\nMODH_Read_04H");
    if (g_tModH.RxCount > 0)
    {
        bytes = g_tModH.RxBuf[2];	/* 数据长度 字节数 */

        if(bytes<=100)
        {
            p = &g_tModH.RxBuf[3];
            g_tModH.fAck04H = 1;
        }


    }
}
/*
*********************************************************************************************************
*	函 数 名: MODH_Poll
*	功能说明: 接收控制器指令. 1ms 响应时间。
*	形    参: 无
*	返 回 值: 0 表示无数据 1表示收到正确命令
*********************************************************************************************************
*/
void MODH_Poll(void)
{
    uint16_t crc1;

    if (g_modh_timeout == 0)	/* 超过3.5个字符时间后执行MODH_RxTimeOut()函数。全局变量 g_rtu_timeout = 1 */
    {
        /* 没有超时，继续接收。不要清零 g_tModH.RxCount */
        return ;
    }

    /* 收到命令
    	05 06 00 88 04 57 3B70 (8 字节)
    		05    :  数码管屏的号站，
    		06    :  指令
    		00 88 :  数码管屏的显示寄存器
    		04 57 :  数据,,,转换成 10 进制是 1111.高位在前,
    		3B70  :  二个字节 CRC 码	从05到 57的校验
    */
    g_modh_timeout = 0;

    if (g_tModH.RxCount < 4){
        goto err_ret;
    }

    /* 计算CRC校验和 */
    crc1 = CRC16_Modbus(g_tModH.RxBuf, g_tModH.RxCount);
    if (crc1 != 0){
        goto err_ret;
    }
	printf("crc ok");
    /* 分析应用层协议 */
    MODH_AnalyzeApp();

err_ret:
    g_tModH.RxCount = 0;	/* 必须清零计数器，方便下次帧同步 */
}
void MODH_AnalyzeApp(void)
{
	switch (g_tModH.RxBuf[1])			/* 第2个字节 功能码 */
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
*	函 数 名: MODH_ReadParam_01H
*	功能说明: 单个参数. 通过发送01H指令实现，发送之后，等待从机应答。
*	形    参: 无
*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
*********************************************************************************************************
*/
uint8_t MODH_ReadParam_01H(uint8_t addr,uint16_t _reg, uint16_t _num)
{
	int32_t time1;
    uint8_t i;
	
	for (i = 0; i < NUM; i++)
	{
		MODH_Send01H (addr, _reg, _num);		  /* 发送命令 */
		time1 = bsp_GetRunTime();	/* 记录命令发送的时刻 */
		while(1)
		{
//			bsp_Idle();
			MODH_AnalyzeApp();
			if (bsp_CheckRunTime(time1) > TIMEOUT){
                break;		/* 通信超时了 */
            }
			if (g_tModH.fAck01H > 0){
                break;		/* 接收到应答 */
            }
			
		}
		if (g_tModH.fAck01H > 0){
            break;			/* 循环NUM次，如果接收到命令则break循环 */
        }	
	}
	if (g_tModH.fAck01H == 0){
        return 0;
    }else{
        return 1;	/* 01H 读成功 */
    }
}
/*
*********************************************************************************************************
*	函 数 名: MODH_ReadParam_04H
*	功能说明: 单个参数. 通过发送04H指令实现，发送之后，等待从机应答。
*	形    参: 无
*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
*********************************************************************************************************
*/
uint8_t MODH_ReadParam_04H(uint8_t _addr,uint16_t _reg, uint16_t _num)
{
    int32_t time1;
    uint8_t i;

    for (i = 0; i < NUM; i++)
    {
        MODH_Send04H (_addr, _reg, _num);
        time1 = bsp_GetRunTime();	/* 记录命令发送的时刻 */
//        printf("\r\nRunTime:%d",time1);
        while (1)
        {
            MODH_Poll();

            if (bsp_CheckRunTime(time1) > TIMEOUT)
            {
                break;		/* 通信超时了 */
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
        return 0;	/* 通信超时了 */
    }
    else
    {
        return 1;	/* 04H 读成功 */
    }
}
