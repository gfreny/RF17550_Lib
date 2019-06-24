 /************************************************************/
//2014.03.06�޸İ�
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
** �������ƣ�pcd_Init
** ����������MCU��ʼ������������SPI��UART�ĳ�ʼ��
** �����������
** ����ֵ  ����
*********************************************************************************************************/
void pcd_Init(void)
{
    //spi_Init();
    //CD_CfgTPD();                                                        /* ���ø�λ�ܽ�                 */
	SPI_Configuration();
}

/*********************************************************************************************************
** �������ƣ�MCU_TO_PCD_TEST
** ����������MCU�����оƬͨ�Ų��Ժ���
** �����������
** ����ֵ  ��TRUE---����ͨ��    FALSE----����ʧ��
** ˵������ͬ�Ķ���оƬ��ѡȡ���ڲ��ԵļĴ�����һ������Ҫ���߾���оƬ��ȷ��Ŀ��Ĵ���
*********************************************************************************************************/

uint8_t MCU_TO_PCD_TEST(void)
{
    volatile uint8_t ucRegVal;

    pcd_RST();                                                      /* Ӳ����λ                         */
    ucRegVal = spi_GetReg(ControlReg);
	  spi_SetReg(ControlReg, 0x10);               // ������д��ģʽ
	  ucRegVal = spi_GetReg(ControlReg);
    spi_SetReg(GsNReg, 0xF0 | 0x04);            // CWGsN = 0xF; ModGsN = 0x4
    ucRegVal = spi_GetReg(GsNReg);
    if(ucRegVal != 0xF4)                        // ��֤�ӿ���ȷ
        return FALSE;
		return TRUE;
}

/*********************************************************************************************************
** �������ƣ�pcd_RST
** ����������PCD����PICCӲ����λ����
** �����������
** ����ֵ  ����
** ˵������ͬ�Ķ���оƬ�ĸ�λ��ʽ��һ������Ҫ���߾���оƬ�ĸ�λ��ʽ��д����
*********************************************************************************************************/
void pcd_RST(void)
{	/*
    CD_SetTPD();
    Delay100us(3);
    CD_ClrTPD();                                                        // ��λ�ܽ�����                 
    Delay100us(5);
    CD_SetTPD();                                                        // ������                       
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
��������	    Read_Reg
���ܣ�	      ���Ĵ�������
���������	  reg_add���Ĵ�����ַ
����ֵ��	    �Ĵ�����ֵ
*************************************************************/
unsigned char Read_Reg(unsigned char reg_add)
{
unsigned char  reg_value;	   
 reg_value=spi_GetReg(reg_add);
 return reg_value;
}
/*************************************************************
��������	    Read_Reg All
���ܣ�	    ��64���Ĵ�������
���������	��
�������		*reg_value���Ĵ�����ֵָ��
����ֵ��	    TRUE
************************************************************/
unsigned char Read_Reg_All(unsigned char *reg_value)
{
unsigned char  i;
	for (i=0;i<64;i++)	   
		*(reg_value+i)=spi_GetReg(i);
 return TRUE;
}

/*************************************************************
��������	    Write_Reg									 
���ܣ�	      д�Ĵ�������								 

���������	  reg_add���Ĵ�����ַ��reg_value���Ĵ�����ֵ	 
����ֵ��	    TRUE											 
				      FALSE										 
*************************************************************/
unsigned char Write_Reg(unsigned char reg_add,unsigned char reg_value)
{
	spi_SetReg(reg_add,reg_value);
	return TRUE;
}
/*************************************************************
��������	    Read_FIFO									                   
���ܣ�	    ��ȡFIFO  									                   
															                            
���������	length����ȡ���ݳ���					                
���������	*fifo_data�����ݴ��ָ��		             			
����ֵ��	    											                      
*************************************************************/
void Read_FIFO(unsigned char length,unsigned char *fifo_data)
{	 
	//SPIRead_Sequence(length,FIFODataReg,fifo_data);
	return;
}
/*************************************************************/
/*��������	    Write_FIFO									 */
/*���ܣ�	    д��FIFO  									 */
/*															 */
/*���������	length����ȡ���ݳ��ȣ�*fifo_data�����ݴ��ָ��*/
/*���������												 */
/*����ֵ��	    											 */
/*															 */
/*************************************************************/
void Write_FIFO(unsigned char length,unsigned char *fifo_data)
{
	//SPIWrite_Sequence(length,FIFODataReg,fifo_data);
	return;
}
/*************************************************************
��������	    Clear_FIFO					 
���ܣ�	      ���FIFO  				
															
���������	  ��								
���������										
����ֵ��	    TRUE						
				      FALSE									
************************************************************/
unsigned char Clear_FIFO(void)
{
	 Set_BitMask(FIFOLevelReg,0x80);//���FIFO����
	 if (spi_GetReg(FIFOLevelReg)==0)
	 	return TRUE;
	else
		return FALSE;
}
/*************************************************************/
/*��������	    Set_BitMask									 */
/*���ܣ�	    ��λ�Ĵ�������								 */
/*���������	reg_add���Ĵ�����ַ��mask���Ĵ�����λ		 */
/*����ֵ��	    TRUE											 */
/*				FALSE										 */
/*************************************************************/
unsigned char Set_BitMask(unsigned char reg_add,unsigned char mask)
{
    unsigned char result;
    result=spi_SetReg(reg_add,Read_Reg(reg_add) | mask);  // set bit mask
	return result;
}
/********************************************
��������	    Clear_BitMask
���ܣ�	    ���λ�Ĵ�������
				
���������	reg_add���Ĵ�����ַ��mask���Ĵ������λ
����ֵ��	    TRUE
				FALSE
********************************************/
unsigned char Clear_BitMask(unsigned char reg_add,unsigned char mask)
{
    unsigned char result;
    result=Write_Reg(reg_add,Read_Reg(reg_add) & ~mask);  // clear bit mask
	return result;
}
/*********************************************/
/*��������	    Set_RF
���ܣ�	    ������Ƶ���
				
���������	mode����Ƶ���ģʽ
				0���ر����
        1������TX1���
        2������TX2���
				3��TX1��TX2�������TX2Ϊ�������
����ֵ��	    TRUE
				FALSE
********************************************/

unsigned char Set_Rf(unsigned char mode)
{
unsigned char result;
if	((Read_Reg(TxControlReg)&0x03)==mode)
	return TRUE;
if (mode==0)
	{
	result=Clear_BitMask(TxControlReg,0x03); //�ر�TX1��TX2���
	}
if (mode==1)
	{
	result=Clear_BitMask(TxControlReg,0x01); //����TX1���
	}
if (mode==2)
	{
	result=Clear_BitMask(TxControlReg,0x02); //����TX2���
	}
if (mode==3)
	{
	result=Set_BitMask(TxControlReg,0x03); //��TX1��TX2���
	}
    //Delay100us(1000);
	Delay_Ms(20);
return result;
}
/*********************************************
 ��������	    Pcd_Comm ��������IRQ�ܽŵ������
 ���ܣ�	    ������ͨ��
				
 ���������	Command��ͨ�Ų������
				pInData�������������飻
				InLenByte���������������ֽڳ��ȣ�
				pOutData�������������飻
				pOutLenBit���������ݵ�λ����
 ����ֵ��	    TRUE
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
	  Write_Reg(ComIrqReg, 0x7F);               //���IRQ���
	  Write_Reg(TModeReg,0x80);                 //����TIMER�Զ�����
    switch (Command) {

    case MFAuthent:                                                  /* Mifare��֤                   */
        irqEn   = 0x12;
        waitFor = 0x10;
    break;
    case Transceive:                                               /* ����FIFO�е����ݵ����ߣ�����󼤻���յ�·*/
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

    i = 30000;                                                            /* ����ʱ��Ƶ�ʵ���������M1�����ȴ�ʱ��25ms*/

    do {
        n = Read_Reg(ComIrqReg);
        i--;
    } while ((i != 0) && !(n & 0x01) && !(n & waitFor));         //i==0��ʾ��ʱ���ˣ�n&0x01!=1��ʾPCDsettimerʱ��δ��
		                                                             //n&waitFor!=1��ʾָ��ִ�����

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
 	Clear_BitMask(BitFramingReg,0x80);//�رշ���
    return status;
}
 /********************************************
 ��������	    Pcd_SetTimer
 ���ܣ�	    ���ý�����ʱ
 ���������	delaytime����ʱʱ�䣨��λΪ���룩
 ����ֵ��	    TRUE
 ********************************************/
 unsigned char Pcd_SetTimer(unsigned long delaytime)//�趨��ʱʱ�䣨ms��
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
 ��������	    Pcd_ConfigISOType
 ���ܣ�	    ���ò���Э��
 ���������	type 0��ISO14443AЭ�飻
 					 1��ISO14443BЭ�飻
 ����ֵ��	    TRUE
 ********************************************/
unsigned char Pcd_ConfigISOType(unsigned char  type)
{
	
   if (type == 0)                     //ISO14443_A
   { 
   		Set_BitMask(ControlReg, 0x10); //ControlReg 0x0C ����readerģʽ
		  Set_BitMask(TxAutoReg, 0x40); //TxASKReg 0x15 ����100%ASK��Ч
	    Write_Reg(TxModeReg, 0x00);  //TxModeReg 0x12 ����TX CRC��Ч��TX FRAMING =TYPE A
	    Write_Reg(RxModeReg, 0x00); //RxModeReg 0x13 ����RX CRC��Ч��RX FRAMING =TYPE A
   }
   if (type == 1)                     //ISO14443_B
   	{ 
   		//Write_Reg(ControlReg, 0x10); //ControlReg 0x0C ����readerģʽ
	    //Write_Reg(TxModeReg, 0x83); //TxModeReg 0x12 ����TX CRC��Ч��TX FRAMING =TYPE B
		 // Write_Reg(RxModeReg, 0x83); //RxModeReg 0x13 ����RX CRC��Ч��RX FRAMING =TYPE B
			//Write_Reg(GsNReg, 0xF4); //GsNReg 0x27 ����ON�絼
			//Write_Reg(GsNOffReg, 0xF4); //GsNOffReg 0x23 ����OFF�絼
			//Write_Reg(TxAutoReg, 0x00);// TxASKReg 0x15 ����100%ASK��Ч
			
			Write_Reg(ControlReg,0x10);
			Write_Reg(TxModeReg,0x83);//BIT1~0 = 2'b11:ISO/IEC 14443B
			Write_Reg(RxModeReg,0x83);//BIT1~0 = 2'b11:ISO/IEC 14443B
			Write_Reg(TxAutoReg,0x00);
			Write_Reg(RxThresholdReg,0x55);//p30
			//Write_Reg(RxThresholdReg,0x33);
			
			
			Write_Reg(RFCfgReg,0x48);//?????????????
			Write_Reg(TxBitPhaseReg,0x87);//Ĭ��ֵ	
			Write_Reg(GsNReg,0x83);	
			Write_Reg(CWGsPReg,0x10);
			Write_Reg(GsNOffReg,0x38);
			Write_Reg(ModGsPReg,0x07);//10% 2017-04-04
	
	}
	Delay_Ms(2);
   return TRUE;
}
/********************************************
 ��������	    FM175X_SoftReset
 ���ܣ�	    ��λ����
				
 ���������	
				

 ����ֵ��	    TRUE
				FALSE
 ********************************************/

unsigned char  FM175X_SoftReset(void)
  {	
	Write_Reg(CommandReg,SoftReset);//
	return	Set_BitMask(ControlReg,0x10);//17520��ʼֵ����
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
 ��������	    FM175X_SoftPowerdown
 ���ܣ�	    ����͹��Ĳ���
 ���������	
 ����ֵ��	    TRUE������͹���ģʽ��
 				FALSE���˳��͹���ģʽ��
 ********************************************/
unsigned char FM175X_SoftPowerdown(void)
{
	if (Read_Reg(CommandReg)&0x10)
		{
		Clear_BitMask(CommandReg,0x10);//�˳��͹���ģʽ
		return FALSE;
		}
	else
		Set_BitMask(CommandReg,0x10);//����͹���ģʽ
		return TRUE;
}
/*********************************************
 ��������	    FM175X_HardPowerdown
 ���ܣ�	    Ӳ���͹��Ĳ���
 ���������	
 ����ֵ��	    TRUE������͹���ģʽ��
 				FALSE���˳��͹���ģʽ��
*********************************************/
unsigned char FM175X_HardPowerdown(void)
{	
	//NPD=~NPD;
	//if(NPD==1)
		return TRUE; //����͹���ģʽ
//	else
		//return FALSE;//�˳��͹���ģʽ
}
/********************************************
 ��������	    Read_Ext_Reg
 ���ܣ�	    ��ȡ��չ�Ĵ���
 ���������	reg_add���Ĵ�����ַ
 ����ֵ��	    �Ĵ�����ֵ
 ********************************************/
unsigned char Read_Ext_Reg(unsigned char reg_add)
{
 	Write_Reg(0x0F,0x80+reg_add);
 	return Read_Reg(0x0F);
}
 /********************************************
 ��������	    Write_Ext_Reg
 ���ܣ�	    д����չ�Ĵ���
 ���������	reg_add���Ĵ�����ַ��reg_value���Ĵ�����ֵ
 ����ֵ��	    TRUE
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
	  Write_Reg(ComIrqReg, 0x7F);               //���IRQ���
	  Write_Reg(TModeReg,0x80);                 //����TIMER�Զ�����
    switch (Command) {

    case MFAuthent:                                                  /* Mifare��֤                   */
        irqEn   = 0x12;
        waitFor = 0x10;
    break;
    case Transceive:                                               /* ����FIFO�е����ݵ����ߣ�����󼤻���յ�·*/
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

    i = 30000;                                                            /* ����ʱ��Ƶ�ʵ���������M1�����ȴ�ʱ��25ms*/

    do {
        n = Read_Reg(ComIrqReg);
        i--;
    } while ((i != 0) && !(n & 0x01) && !(n & waitFor));         //i==0��ʾ��ʱ���ˣ�n&0x01!=1��ʾPCDsettimerʱ��δ��
		                                                             //n&waitFor!=1��ʾָ��ִ�����

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
 	Clear_BitMask(BitFramingReg,0x80);//�رշ���
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
	  Write_Reg(ComIrqReg, 0x7F);               //���IRQ���
	  Write_Reg(TModeReg,0x80);                 //����TIMER�Զ�����
        // ����FIFO�е����ݵ����ߣ�����󼤻���յ�·
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
    i=60000;//��ֹ����
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
    	{//��Ҫ�ж��Ƿ����
    		if((n+offset)>MaxReLeng)
    			n=MaxReLeng-offset;
    		for(k=0;k<n;)
    		{
    			pOutData[offset++]=Read_Reg(FIFODataReg);
    			k+=1;	
    		}
    	}
    	
    	q=Read_Reg(ComIrqReg);
    	if(q&0x03)//�д���
    	{   Write_Reg(CommandReg, Idle);//ȡ����ǰָ��
    		  Clear_BitMask(BitFramingReg, 0x80);
        	Set_BitMask(ControlReg,0x80);//ֹͣ��ʱ
    			return FALSE;
    	}
    	else if(q&0x30)//���
    	{
    		n = Read_Reg(FIFOLevelReg);
    		if(n>0)
    		{
    			//��Ҫ�ж��Ƿ����
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
        	Set_BitMask(ControlReg,0x80);//ֹͣ��ʱ
    		return TRUE;
    	}
    	else;
      i--;
    }
}