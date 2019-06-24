 /************************************************************/
//2014.03.06修改版
/*************************************************************/
//#include "LPC11XX_SPI.h"
//#include "M0_PACK_V101_IO.h"
//#include "Utility.h"
#include "fm175xx.h"
//#include "LPC11xx.h"
#include "stm32f10x.h"
#include "platform_config.h"
#include "system_define.h"
#include <string.h>	
unsigned char FM175XX_IRQ;

#define MI_NOTAGERR 0xEE
#define MAXRLEN 64
#define MaxReLeng 250
/*********************************************************************************************************
** 函数名称：pcd_Init
** 函数描述：MCU初始化函数、包括SPI和UART的初始化
** 输入参数：无
** 返回值  ：无
*********************************************************************************************************/
void pcd_Init(void)
{
    //spi_Init();
    //CD_CfgTPD();                                                        /* 配置复位管脚                 */
	SPI_Configuration();
}

/*********************************************************************************************************
** 函数名称：MCU_TO_PCD_TEST
** 函数描述：MCU与读卡芯片通信测试函数
** 输入参数：无
** 返回值  ：TRUE---测试通过    FALSE----测试失败
** 说明：不同的读卡芯片的选取用于测试的寄存器不一样，需要更具具体芯片而确定目标寄存器
*********************************************************************************************************/

uint8_t MCU_TO_PCD_TEST(void)
{
    volatile uint8_t ucRegVal;

    pcd_RST();                                                      /* 硬件复位                         */
    ucRegVal = spi_GetReg(ControlReg);
	  spi_SetReg(ControlReg, 0x10);               // 启动读写器模式
	  ucRegVal = spi_GetReg(ControlReg);
    spi_SetReg(GsNReg, 0xF0 | 0x04);            // CWGsN = 0xF; ModGsN = 0x4
    ucRegVal = spi_GetReg(GsNReg);
    if(ucRegVal != 0xF4)                        // 验证接口正确
        return FALSE;
		return TRUE;
}

/*********************************************************************************************************
** 函数名称：pcd_RST
** 函数描述：PCD控制PICC硬件复位函数
** 输入参数：无
** 返回值  ：无
** 说明：不同的读卡芯片的复位方式不一样，需要更具具体芯片的复位方式改写函数
*********************************************************************************************************/
void pcd_RST(void)
{	/*
    CD_SetTPD();
    Delay100us(3);
    CD_ClrTPD();                                                        // 复位管脚拉低                 
    Delay100us(5);
    CD_SetTPD();                                                        // 再拉高                       
    Delay100us(10);
   */
    GPIO_SetBits(RC531_RST_PORT, RC531_RST_PIN);
	Delay_Ms(10);
	GPIO_ResetBits(RC531_RST_PORT, RC531_RST_PIN);
	Delay_Ms(10);
	GPIO_SetBits(RC531_RST_PORT, RC531_RST_PIN);
	Delay_Ms(100);
}

/*************************************************************
函数名：	    Read_Reg
功能：	      读寄存器操作
输入参数：	  reg_add，寄存器地址
返回值：	    寄存器数值
*************************************************************/
unsigned char Read_Reg(unsigned char reg_add)
{
unsigned char  reg_value;	   
 reg_value=spi_GetReg(reg_add);
 return reg_value;
}
/*************************************************************
函数名：	    Read_Reg All
功能：	    读64个寄存器操作
输入参数：	无
输出参数		*reg_value，寄存器数值指针
返回值：	    TRUE
************************************************************/
unsigned char Read_Reg_All(unsigned char *reg_value)
{
unsigned char  i;
	for (i=0;i<64;i++)	   
		*(reg_value+i)=spi_GetReg(i);
 return TRUE;
}

/*************************************************************
函数名：	    Write_Reg									 
功能：	      写寄存器操作								 

输入参数：	  reg_add，寄存器地址；reg_value，寄存器数值	 
返回值：	    TRUE											 
				      FALSE										 
*************************************************************/
unsigned char Write_Reg(unsigned char reg_add,unsigned char reg_value)
{
	spi_SetReg(reg_add,reg_value);
	return TRUE;
}
/*************************************************************
函数名：	    Read_FIFO									                   
功能：	    读取FIFO  									                   
															                            
输入参数：	length，读取数据长度					                
输出参数：	*fifo_data，数据存放指针		             			
返回值：	    											                      
*************************************************************/
void Read_FIFO(unsigned char length,unsigned char *fifo_data)
{	 
	//SPIRead_Sequence(length,FIFODataReg,fifo_data);
	return;
}
/*************************************************************/
/*函数名：	    Write_FIFO									 */
/*功能：	    写入FIFO  									 */
/*															 */
/*输入参数：	length，读取数据长度；*fifo_data，数据存放指针*/
/*输出参数：												 */
/*返回值：	    											 */
/*															 */
/*************************************************************/
void Write_FIFO(unsigned char length,unsigned char *fifo_data)
{
	//SPIWrite_Sequence(length,FIFODataReg,fifo_data);
	return;
}
/*************************************************************
函数名：	    Clear_FIFO					 
功能：	      清空FIFO  				
															
输入参数：	  无								
输出参数：										
返回值：	    TRUE						
				      FALSE									
************************************************************/
unsigned char Clear_FIFO(void)
{
	 Set_BitMask(FIFOLevelReg,0x80);//清除FIFO缓冲
	 if (spi_GetReg(FIFOLevelReg)==0)
	 	return TRUE;
	else
		return FALSE;
}
/*************************************************************/
/*函数名：	    Set_BitMask									 */
/*功能：	    置位寄存器操作								 */
/*输入参数：	reg_add，寄存器地址；mask，寄存器置位		 */
/*返回值：	    TRUE											 */
/*				FALSE										 */
/*************************************************************/
unsigned char Set_BitMask(unsigned char reg_add,unsigned char mask)
{
    unsigned char result;
    result=spi_SetReg(reg_add,Read_Reg(reg_add) | mask);  // set bit mask
	return result;
}
/********************************************
函数名：	    Clear_BitMask
功能：	    清除位寄存器操作
				
输入参数：	reg_add，寄存器地址；mask，寄存器清除位
返回值：	    TRUE
				FALSE
********************************************/
unsigned char Clear_BitMask(unsigned char reg_add,unsigned char mask)
{
    unsigned char result;
    result=Write_Reg(reg_add,Read_Reg(reg_add) & ~mask);  // clear bit mask
	return result;
}
/*********************************************/
/*函数名：	    Set_RF
功能：	    设置射频输出
				
输入参数：	mode，射频输出模式
				0，关闭输出
        1，仅打开TX1输出
        2，仅打开TX2输出
				3，TX1，TX2打开输出，TX2为反向输出
返回值：	    TRUE
				FALSE
********************************************/

unsigned char Set_Rf(unsigned char mode)
{
unsigned char result;
if	((Read_Reg(TxControlReg)&0x03)==mode)
	return TRUE;
if (mode==0)
	{
	result=Clear_BitMask(TxControlReg,0x03); //关闭TX1，TX2输出
	}
if (mode==1)
	{
	result=Clear_BitMask(TxControlReg,0x01); //仅打开TX1输出
	}
if (mode==2)
	{
	result=Clear_BitMask(TxControlReg,0x02); //仅打开TX2输出
	}
if (mode==3)
	{
	result=Set_BitMask(TxControlReg,0x03); //打开TX1，TX2输出
	}
    //Delay100us(1000);
	Delay_Ms(20);
return result;
}
/*********************************************
 函数名：	    Pcd_Comm （不利用IRQ管脚的情况）
 功能：	    读卡器通信
				
 输入参数：	Command，通信操作命令；
				pInData，发送数据数组；
				InLenByte，发送数据数组字节长度；
				pOutData，接收数据数组；
				pOutLenBit，接收数据的位长度
 返回值：	    TRUE
				FALSE
 ********************************************/  
unsigned char Pcd_Comm(	unsigned char Command, 
                 		unsigned char *pInData, 
                 		unsigned char InLenByte,
                 		unsigned char *pOutData, 
                 		unsigned int *pOutLenBit)
{
   uint8_t status  = FALSE;
    uint8_t irqEn   = 0x00;
    uint8_t waitFor = 0x00;
    uint8_t lastBits;
    uint8_t n,k;
    uint32_t i;
	  Write_Reg(ComIrqReg, 0x7F);               //清楚IRQ标记
	  Write_Reg(TModeReg,0x80);                 //设置TIMER自动启动
    switch (Command) {

    case MFAuthent:                                                  /* Mifare认证                   */
        irqEn   = 0x12;
        waitFor = 0x10;
    break;
    case Transceive:                                               /* 发送FIFO中的数据到天线，传输后激活接收电路*/
        irqEn   = 0x77;
        waitFor = 0x30;
    break;
    default:
    break;
    }
   
    Write_Reg(ComIEnReg, irqEn | 0x80);
   // Clear_BitMask(ComIrqReg, 0x80);
    Write_Reg(CommandReg, Idle);
    Set_BitMask(FIFOLevelReg, 0x80);
    
    for (i=0; i < InLenByte; i++) {
        Write_Reg(FIFODataReg, pInData[i]);
    }
    Write_Reg(CommandReg, Command);

    if (Command == Transceive) {
        Set_BitMask(BitFramingReg, 0x80);
    }

    i = 30000;                                                            /* 根据时钟频率调整，操作M1卡最大等待时间25ms*/

    do {
        n = Read_Reg(ComIrqReg);
        i--;
    } while ((i != 0) && !(n & 0x01) && !(n & waitFor));         //i==0表示延时到了，n&0x01!=1表示PCDsettimer时间未到
		                                                             //n&waitFor!=1表示指令执行完成

    Clear_BitMask(BitFramingReg, 0x80);

    if (i != 0) {
        if(!(Read_Reg(ErrorReg) & 0x1B)) {
            status = TRUE;
            if (n & irqEn & 0x01) {
                status = MI_NOTAGERR;
            }
            if (Command == Transceive) {
                n = Read_Reg(FIFOLevelReg);
                lastBits = Read_Reg(ControlReg) & 0x07;
                if (lastBits) {
                    *pOutLenBit = (n - 1) * 8 + lastBits;
                } else {
                    *pOutLenBit = n * 8;
                }
                if (n == 0) {
                    n = 1;
                }
                if (n > MAXRLEN) {
                    n = MAXRLEN;
                }
                for (i = 0; i < n; i++) {
                    pOutData[i] = Read_Reg(FIFODataReg);
                }
            }
        } else {
            status = FALSE;
        }
    }
 	Clear_BitMask(BitFramingReg,0x80);//关闭发送
    return status;
}
 /********************************************
 函数名：	    Pcd_SetTimer
 功能：	    设置接收延时
 输入参数：	delaytime，延时时间（单位为毫秒）
 返回值：	    TRUE
 ********************************************/
 unsigned char Pcd_SetTimer(unsigned long delaytime)//设定超时时间（ms）
{
	unsigned long TimeReload;
	unsigned int Prescaler;

	Prescaler=0;
	TimeReload=0;
	while(Prescaler<0xfff)
	{
		TimeReload = ((delaytime*(long)13560)-1)/(Prescaler*2+1);
		
		if( TimeReload<0xffff)
			break;
		Prescaler++;
	}
		TimeReload=TimeReload&0xFFFF;
		Set_BitMask(TModeReg,Prescaler>>8);
		Write_Reg(TPrescalerReg,Prescaler&0xFF);					
		Write_Reg(TReloadMSBReg,TimeReload>>8);
		Write_Reg(TReloadLSBReg,TimeReload&0xFF);
	return TRUE;
}
 /********************************************
 函数名：	    Pcd_ConfigISOType
 功能：	    设置操作协议
 输入参数：	type 0，ISO14443A协议；
 					 1，ISO14443B协议；
 返回值：	    TRUE
 ********************************************/
unsigned char Pcd_ConfigISOType(unsigned char  type)
{
	
   if (type == 0)                     //ISO14443_A
   { 
   		Set_BitMask(ControlReg, 0x10); //ControlReg 0x0C 设置reader模式
		  Set_BitMask(TxAutoReg, 0x40); //TxASKReg 0x15 设置100%ASK有效
	    Write_Reg(TxModeReg, 0x00);  //TxModeReg 0x12 设置TX CRC无效，TX FRAMING =TYPE A
	    Write_Reg(RxModeReg, 0x00); //RxModeReg 0x13 设置RX CRC无效，RX FRAMING =TYPE A
   }
   if (type == 1)                     //ISO14443_B
   	{ 
   		//Write_Reg(ControlReg, 0x10); //ControlReg 0x0C 设置reader模式
	    //Write_Reg(TxModeReg, 0x83); //TxModeReg 0x12 设置TX CRC有效，TX FRAMING =TYPE B
		 // Write_Reg(RxModeReg, 0x83); //RxModeReg 0x13 设置RX CRC有效，RX FRAMING =TYPE B
			//Write_Reg(GsNReg, 0xF4); //GsNReg 0x27 设置ON电导
			//Write_Reg(GsNOffReg, 0xF4); //GsNOffReg 0x23 设置OFF电导
			//Write_Reg(TxAutoReg, 0x00);// TxASKReg 0x15 设置100%ASK无效
			
			Write_Reg(ControlReg,0x10);
			Write_Reg(TxModeReg,0x83);//BIT1~0 = 2'b11:ISO/IEC 14443B
			Write_Reg(RxModeReg,0x83);//BIT1~0 = 2'b11:ISO/IEC 14443B
			Write_Reg(TxAutoReg,0x00);
			Write_Reg(RxThresholdReg,0x55);//p30
			//Write_Reg(RxThresholdReg,0x33);
			
			
			Write_Reg(RFCfgReg,0x48);//?????????????
			Write_Reg(TxBitPhaseReg,0x87);//默认值	
			Write_Reg(GsNReg,0x83);	
			Write_Reg(CWGsPReg,0x10);
			Write_Reg(GsNOffReg,0x38);
			Write_Reg(ModGsPReg,0x07);//10% 2017-04-04
	
	}
	Delay_Ms(2);
   return TRUE;
}
/********************************************
 函数名：	    FM175X_SoftReset
 功能：	    软复位操作
				
 输入参数：	
				

 返回值：	    TRUE
				FALSE
 ********************************************/

unsigned char  FM175X_SoftReset(void)
  {	
	Write_Reg(CommandReg,SoftReset);//
	return	Set_BitMask(ControlReg,0x10);//17520初始值配置
  }

unsigned char FM175X_HardReset(void)
  {	
	/*CD_ClrTPD();
	Delay100us(1);
	CD_SetTPD();
	Delay100us(1);*/

	GPIO_ResetBits(RC531_RST_PORT, RC531_RST_PIN);
	Delay_Ms(2);//1ms
	GPIO_SetBits(RC531_RST_PORT, RC531_RST_PIN);
	Delay_Ms(2);//1ms
	return TRUE;
  }
/*********************************************
 函数名：	    FM175X_SoftPowerdown
 功能：	    软件低功耗操作
 输入参数：	
 返回值：	    TRUE，进入低功耗模式；
 				FALSE，退出低功耗模式；
 ********************************************/
unsigned char FM175X_SoftPowerdown(void)
{
	if (Read_Reg(CommandReg)&0x10)
		{
		Clear_BitMask(CommandReg,0x10);//退出低功耗模式
		return FALSE;
		}
	else
		Set_BitMask(CommandReg,0x10);//进入低功耗模式
		return TRUE;
}
/*********************************************
 函数名：	    FM175X_HardPowerdown
 功能：	    硬件低功耗操作
 输入参数：	
 返回值：	    TRUE，进入低功耗模式；
 				FALSE，退出低功耗模式；
*********************************************/
unsigned char FM175X_HardPowerdown(void)
{	
	//NPD=~NPD;
	//if(NPD==1)
		return TRUE; //进入低功耗模式
//	else
		//return FALSE;//退出低功耗模式
}
/********************************************
 函数名：	    Read_Ext_Reg
 功能：	    读取扩展寄存器
 输入参数：	reg_add，寄存器地址
 返回值：	    寄存器数值
 ********************************************/
unsigned char Read_Ext_Reg(unsigned char reg_add)
{
 	Write_Reg(0x0F,0x80+reg_add);
 	return Read_Reg(0x0F);
}
 /********************************************
 函数名：	    Write_Ext_Reg
 功能：	    写入扩展寄存器
 输入参数：	reg_add，寄存器地址；reg_value，寄存器数值
 返回值：	    TRUE
				FALSE
 ********************************************/
unsigned char Write_Ext_Reg(unsigned char reg_add,unsigned char reg_value)
{
	Write_Reg(0x0F,0x40+reg_add);
	return (Write_Reg(0x0F,0xC0+reg_value));
}
unsigned char Pcd_CommByte(	unsigned char Command, 
                 		unsigned char *pInData, 
                 		unsigned char InLenByte,
                 		unsigned char *pOutData, 
                 		unsigned int *pOutLenByte)
{
   uint8_t status  = FALSE;
    uint8_t irqEn   = 0x00;
    uint8_t waitFor = 0x00;
    uint8_t lastBits;
    uint8_t n,k;
    uint32_t i;
	  Write_Reg(ComIrqReg, 0x7F);               //清楚IRQ标记
	  Write_Reg(TModeReg,0x80);                 //设置TIMER自动启动
    switch (Command) {

    case MFAuthent:                                                  /* Mifare认证                   */
        irqEn   = 0x12;
        waitFor = 0x10;
    break;
    case Transceive:                                               /* 发送FIFO中的数据到天线，传输后激活接收电路*/
        irqEn   = 0x77;
        waitFor = 0x30;
    break;
    default:
    break;
    }
   
    Write_Reg(ComIEnReg, irqEn | 0x80);
   // Clear_BitMask(ComIrqReg, 0x80);
    Write_Reg(CommandReg, Idle);
    Set_BitMask(FIFOLevelReg, 0x80);
    
    for (i=0; i < InLenByte; i++) {
        Write_Reg(FIFODataReg, pInData[i]);
    }
    Write_Reg(CommandReg, Command);

    if (Command == Transceive) {
        Set_BitMask(BitFramingReg, 0x80);
    }

    i = 30000;                                                            /* 根据时钟频率调整，操作M1卡最大等待时间25ms*/

    do {
        n = Read_Reg(ComIrqReg);
        i--;
    } while ((i != 0) && !(n & 0x01) && !(n & waitFor));         //i==0表示延时到了，n&0x01!=1表示PCDsettimer时间未到
		                                                             //n&waitFor!=1表示指令执行完成

    Clear_BitMask(BitFramingReg, 0x80);

    if (i != 0) {
        if(!(Read_Reg(ErrorReg) & 0x1B)) {
            status = TRUE;
            if (n & irqEn & 0x01) {
                status = MI_NOTAGERR;
            }
            if (Command == Transceive) {
                n = Read_Reg(FIFOLevelReg);
                lastBits = Read_Reg(ControlReg) & 0x07;
                if (lastBits) {
                    *pOutLenByte = (n - 1) ;
                } else {
                    *pOutLenByte = n ;
                }
                if (n == 0) {
                    n = 1;
                }
                if (n > MAXRLEN) {
                    n = MAXRLEN;
                }
                for (i = 0; i < n; i++) {
                    pOutData[i] = Read_Reg(FIFODataReg);
                }
            }
        } else {
            status = FALSE;
        }
    }
 	Clear_BitMask(BitFramingReg,0x80);//关闭发送
    return status;
}
unsigned char  AttribSecondID(u8 *buf,u8 len,unsigned int *relen,u8 *savebuf)
{
unsigned char  ret;
		Pcd_SetTimer(20);

		ret=Pcd_CommByte(Transceive,buf,len,savebuf,&(*relen));
		
		if (ret==TRUE)
			return 0;
		else	if(ret==FALSE)
			return 1;
		else 
			return ret;
		  
		/*
		if (ret==TRUE)
			return 0;
		else	
		  return (1);*/
}
void StartSecondID(void)
{
	FM175X_SoftReset();
	Pcd_ConfigISOType(1);
	 Set_Rf(3);
}

/*******************************************************************************
* Function    : THM_SendFrame
* Description : Send Value of Databuffer to PICC	
* Arguments   : buffer  --- send date buffer
				num     --- recieve date length			
* Return      :	
*******************************************************************************/ 
void THM_SendFrame(unsigned char *pInData,unsigned short InLenByte)
{

  uint8_t status  = FALSE;
    uint8_t irqEn   = 0x00;
    uint8_t waitFor = 0x00;
    uint8_t lastBits;
    uint8_t n,k;
    uint16_t j,p,offset=0;
    uint32_t i;
	  Write_Reg(ComIrqReg, 0x7F);               //清楚IRQ标记
	  Write_Reg(TModeReg,0x80);                 //设置TIMER自动启动
        // 发送FIFO中的数据到天线，传输后激活接收电路
        irqEn   = 0x77;
        waitFor = 0x30;

   
    Write_Reg(ComIEnReg, irqEn | 0x80);
   // Clear_BitMask(ComIrqReg, 0x80);
    Write_Reg(CommandReg, Idle);
    Set_BitMask(FIFOLevelReg, 0x80);
    
    if(InLenByte<=48)
    {
    	for(i=0;i<InLenByte;i++)
    		Write_Reg(FIFODataReg, pInData[i]);
    	Write_Reg(CommandReg, Transceive);
        Set_BitMask(BitFramingReg, 0x80);
    }
    else
    {
    	for(i=0;i<48;i++)
    		Write_Reg(FIFODataReg, pInData[i]);
    	Write_Reg(CommandReg, Transceive);
      Set_BitMask(BitFramingReg, 0x80);
    	for(i=48;i<InLenByte;i++)
    	{
    		while(Read_Reg(FIFOLevelReg)>48)	;
    		Write_Reg(FIFODataReg, pInData[i]);
    	}
    }
    i=60000;//防止死机
    while(i)
    {
    	if(Read_Reg(FIFOLevelReg)==0)break;
    	if(Read_Reg(ComIrqReg)&0x40)break;	
    		i--;
    }
   
}
/*******************************************************************************
* Function    : THM_WaitReadFrame
* Description : Wait recieve ended,read data and save to buffer
* Arguments   : buffer  --- recieve date buffer
				len     --- recieve date length			
* Return      :	01      --- recieve end 
				other   --- recieve error
*******************************************************************************/
u8 THM_WaitReadFrame(unsigned short *pOutLenByte, unsigned char *pOutData)
{ 
    uint8_t status  = FALSE;
    uint8_t irqEn   = 0x00;
    uint8_t waitFor = 0x00;
    uint8_t lastBits;
    uint8_t n,k,q,needout;
    uint32_t i;
    uint16_t offset=0;
    i=60000;
    while(i)
    {
    	
    	n = Read_Reg(FIFOLevelReg);
    	if(n>0)
    	{//需要判定是否溢出
    		if((n+offset)>MaxReLeng)
    			n=MaxReLeng-offset;
    		for(k=0;k<n;)
    		{
    			pOutData[offset++]=Read_Reg(FIFODataReg);
    			k+=1;	
    		}
    	}
    	
    	q=Read_Reg(ComIrqReg);
    	if(q&0x03)//有错误
    	{   Write_Reg(CommandReg, Idle);//取消当前指令
    		  Clear_BitMask(BitFramingReg, 0x80);
        	Set_BitMask(ControlReg,0x80);//停止计时
    			return FALSE;
    	}
    	else if(q&0x30)//完成
    	{
    		n = Read_Reg(FIFOLevelReg);
    		if(n>0)
    		{
    			//需要判定是否溢出
    			if((n+offset)>MaxReLeng)
    			n=MaxReLeng-offset;
    		for(k=0;k<n;)
    		{
    			pOutData[offset++]=Read_Reg(FIFODataReg);
    			k+=1;	
    		}
    		}
    		*pOutLenByte=offset;
    		Clear_BitMask(BitFramingReg, 0x80);
        	Set_BitMask(ControlReg,0x80);//停止计时
    		return TRUE;
    	}
    	else;
      i--;
    }
}