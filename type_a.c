/*************************************************************/
//2014.03.06�޸İ�
/*************************************************************/
#include "fm175xx.h"
#include "type_a.h"
#include "Utility.h"
unsigned char  picc_atqa[2],picc_uid[15],picc_sak[3];
//ATQA��ֻ�������ֽڣ�UIDȴ��4���ֽڼ�һ��BCCУ��ֵ�������7�ֽ�UID����ײʱ����Ҫ���15�ֽڵ�UID�����
//SAK��һ���ֽڵ�SAK+CRC_A(2�ֽ�)
/****************************************************************************************/
/*���ƣ�TypeA_Request																	*/
/*���ܣ�TypeA_Request��ƬѰ��															*/
/*���룺																				*/
/*       			    			     												*/
/*	       								 												*/
/*�����																			 	*/
/*	       	pTagType[0],pTagType[1] =ATQA                                         		*/					
/*       	TRUE: Ӧ����ȷ                                                              	*/
/*	 		FALSE: Ӧ�����																*/
/****************************************************************************************/
unsigned char TypeA_Request(unsigned char *pTagType)
{
	unsigned char  result,send_buff[1],rece_buff[2];
	unsigned int  rece_bitlen;  
	Clear_BitMask(TxModeReg,0x80);//�ر�TX CRC
	Clear_BitMask(RxModeReg,0x80);//�ر�RX CRC
	Set_BitMask(RxModeReg, 0x08);//�ر�λ����
	Clear_BitMask(Status2Reg,0x08);
	Write_Reg(BitFramingReg,0x07);
	send_buff[0] = 0x26;
   	Pcd_SetTimer(2);
	Clear_FIFO();
	result = Pcd_Comm(Transceive,send_buff,1,rece_buff,&rece_bitlen);
	if ((result == TRUE) && (rece_bitlen == 2*8))
	{    
		*pTagType     = rece_buff[0];
		*(pTagType+1) = rece_buff[1];
	}
  
	return result;
}
/****************************************************************************************/
/*���ƣ�TypeA_WakeUp																	*/
/*���ܣ�TypeA_WakeUp��ƬѰ��															*/
/*���룺																				*/
/*       			    			     												*/
/*	       								 												*/
/*�����																			 	*/
/*	       	pTagType[0],pTagType[1] =ATQA                                         		*/					
/*       	TRUE: Ӧ����ȷ                                                              	*/
/*	 		FALSE: Ӧ�����																*/
/****************************************************************************************/
unsigned char TypeA_WakeUp(unsigned char *pTagType)
{
	unsigned char   result,send_buff[1],rece_buff[2];
	unsigned int   rece_bitlen;  
	Clear_BitMask(TxModeReg,0x80);//�ر�TX CRC
	Clear_BitMask(RxModeReg,0x80);//�ر�RX CRC
	Set_BitMask(RxModeReg, 0x08);//������С��4bit������
	Clear_BitMask(Status2Reg,0x08);
	Write_Reg(BitFramingReg,0x07);
 	send_buff[0] = 0x52;
   	Pcd_SetTimer(2);
	Clear_FIFO();
	result = Pcd_Comm(Transceive,send_buff,1,rece_buff,&rece_bitlen);

	if ((result == TRUE) && (rece_bitlen == 2*8))
	{    
		*pTagType     = rece_buff[0];
		*(pTagType+1) = rece_buff[1];
	}
	return result;
}
/****************************************************************************************/
/*���ƣ�TypeA_Anticollision																*/
/*���ܣ�TypeA_Anticollision��Ƭ����ͻ													*/
/*���룺selcode =0x93��0x95��0x97														*/
/*       			    			     												*/
/*	       								 												*/
/*�����																			 	*/
/*	       	pSnr[0],pSnr[1],pSnr[2],pSnr[3]pSnr[4] =UID                            		*/					
/*       	TRUE: Ӧ����ȷ                                                              	*/
/*	 		FALSE: Ӧ�����																*/
/****************************************************************************************/
unsigned char TypeA_Anticollision(unsigned char selcode,unsigned char *pSnr)
{
    unsigned char   result,i,send_buff[2],rece_buff[5];
    /*volatile*/ unsigned int   rece_bitlen;
	Clear_BitMask(TxModeReg,0x80);
	Clear_BitMask(RxModeReg,0x80);
    Clear_BitMask(Status2Reg,0x08);
    Write_Reg(BitFramingReg,0x00);
    Clear_BitMask(CollReg,0x80);
 
    send_buff[0] = selcode;
    send_buff[1] = 0x20;             //NVB
    Pcd_SetTimer(2);
	Clear_FIFO();
    result = Pcd_Comm(Transceive,send_buff,2,rece_buff,&rece_bitlen);

    if (result == TRUE)
    {
    	 for (i=0; i<5; i++)
         {   
             *(pSnr+i)  = rece_buff[i];
         }
         if (pSnr[4] != (pSnr[0]^pSnr[1]^pSnr[2]^pSnr[3]))     //UID0��UID1��UID2��UID3��BCC(У���ֽ�)
    		result = FALSE;    
    }
  return result;
}
/****************************************************************************************/
/*���ƣ�TypeA_Select																	*/
/*���ܣ�TypeA_Select��Ƭѡ��															*/
/*���룺selcode =0x93��0x95��0x97														*/
/*      pSnr[0],pSnr[1],pSnr[2],pSnr[3]pSnr[4] =UID 			    			     	*/
/*	       								 												*/
/*�����																			 	*/
/*	       	pSak[0],pSak[1],pSak[2] =SAK			                            		*/					
/*       	TRUE: Ӧ����ȷ                                                              	*/
/*	 		FALSE: Ӧ�����																*/
/****************************************************************************************/
unsigned char TypeA_Select(unsigned char selcode,unsigned char *pSnr,unsigned char *pSak)
{
    unsigned char   result,i,send_buff[7],rece_buff[5];
    unsigned int   rece_bitlen;
	Write_Reg(BitFramingReg,0x00);
  	Set_BitMask(TxModeReg,0x80);      //��TX CRC
	Set_BitMask(RxModeReg,0x80);        //�򿪽���RX ��CRCУ��
    Clear_BitMask(Status2Reg,0x08);
	
	send_buff[0] = selcode;        //SEL
    send_buff[1] = 0x70;         //NVB
    
    for (i=0; i<5; i++)
    {
    	send_buff[i+2] = *(pSnr+i);
    }
   	
	Pcd_SetTimer(2);
	Clear_FIFO();
    result = Pcd_Comm(Transceive,send_buff,7,rece_buff,&rece_bitlen);
    
    if (result == TRUE)
    	{ 
		*pSak=rece_buff[0]; 
	 	}
    return result;
}
/****************************************************************************************/
/*���ƣ�TypeA_Halt																		*/
/*���ܣ�TypeA_Halt��Ƭֹͣ																*/
/*���룺																				*/
/*       			    			     												*/
/*	       								 												*/
/*�����																			 	*/
/*	       											                            		*/					
/*       	TRUE: Ӧ����ȷ                                                              	*/
/*	 		FALSE: Ӧ�����																*/
/****************************************************************************************/
unsigned char TypeA_Halt(void)
{
    unsigned char   result,send_buff[2],rece_buff[1];
	unsigned int   rece_bitlen;
    send_buff[0] = 0x50;
    send_buff[1] = 0x00;
   
  /* 	Write_Reg(BitFramingReg,0x00);
  	Set_BitMask(TxModeReg,0x80);
	Set_BitMask(RxModeReg,0x80);
    Clear_BitMask(Status2Reg,0x08);*/
	Pcd_SetTimer(2);
	Clear_FIFO();
    result = Pcd_Comm(Transmit,send_buff,2,rece_buff,&rece_bitlen);
    return result;
}
unsigned char TypeA_Fine(unsigned char type)
{
	unsigned char   result,i,send_buff[7],rece_buff[64];
    unsigned int   rece_bitlen;
	Write_Reg(BitFramingReg,0x00);
  	Set_BitMask(TxModeReg,0x80);      //��TX CRC
	Set_BitMask(RxModeReg,0x80);        //�򿪽���RX ��CRCУ��
    Clear_BitMask(Status2Reg,0x08);
	
	send_buff[0] = type;        //SEL
    send_buff[1] = 0x00;         //NVB
    
   	
	Pcd_SetTimer(5);
	Clear_FIFO();
	result = Pcd_Comm(Transceive,send_buff,2,rece_buff,&rece_bitlen); 
   if (result == TRUE)
   { 
   	if(type==0xba)
   	{
   		if((rece_bitlen==0x10)&&(rece_buff[0]==0xab)&&(rece_buff[1]==0x00))	
   			return TRUE;
   		else
   			return FALSE;
   	}
   	else if(type==0xbb)
   	{
   		if((rece_bitlen==0x10)&&(rece_buff[0]==0xaa)&&(rece_buff[1]==0x00))	
   			return TRUE;
   		else
   			return FALSE;
   	}

	 }
    return FALSE;
}
/****************************************************************************************/
/*���ƣ�TypeA_CardActive																*/
/*���ܣ�TypeA_CardActive��Ƭ����														*/
/*���룺																				*/
/*       			    			     												*/
/*	       								 												*/
/*�����	pTagType[0],pTagType[1] =ATQA 											 	*/
/*	       	pSnr[0],pSnr[1],pSnr[2],pSnr[3]pSnr[4] =UID 		                   		*/
/*	       	pSak[0],pSak[1],pSak[2] =SAK			                            		*/					
/*       	TRUE: Ӧ����ȷ                                                              	*/
/*	 		FALSE: Ӧ�����																*/
/****************************************************************************************/
unsigned char TypeA_CardActive(unsigned char *pTagType,unsigned char *pSnr,unsigned char *pSak)
{
unsigned char   result;
		Set_Rf(3);   //turn on radio
		Pcd_ConfigISOType(0);
		result=TypeA_Request(pTagType);//Ѱ�� Standard	 send request command Standard mode
		
		if (result==FALSE)
		{
			return FALSE;
		}
			if 	((pTagType[0]&0xC0)==0x00)        //M1��,ID��ֻ��4λ
				{
				result=TypeA_Anticollision(0x93,pSnr);
				if (result==FALSE)
					{
						return FALSE;
					}
				result=TypeA_Select(0x93,pSnr,pSak);
				if (result==FALSE)
					{
						return FALSE;
					}	
				}
			if 	((pTagType[0]&0xC0)==0x40)     //ID����7λ
				{
				result=TypeA_Anticollision(0x93,pSnr);
				if (result==FALSE)
					{
						return FALSE;
					}
				result=TypeA_Select(0x93,pSnr,pSak);
				if (result==FALSE)
					{
						return FALSE;
					}
				result=TypeA_Anticollision(0x95,pSnr+5);
				if (result==FALSE)
					{
						return FALSE;
					}
				result=TypeA_Select(0x95,pSnr+5,pSak+1);
				if (result==FALSE)
					{
						return FALSE;
					}
			   	}
			if 	((pTagType[0]&0xC0)==0x80)       //ID����10λ
				{
				result=TypeA_Anticollision(0x93,pSnr);
				if (result==FALSE)
					{
						return FALSE;
					}
				result=TypeA_Select(0x93,pSnr,pSak);
				if (result==FALSE)
					{
						return FALSE;
					}
				result=TypeA_Anticollision(0x95,pSnr+5);
				if (result==FALSE)
					{
						return FALSE;
					}
				result=TypeA_Select(0x95,pSnr+5,pSak+1);
				if (result==FALSE)
					{
						return FALSE;
					}
				result=TypeA_Anticollision(0x97,pSnr+10);
				if (result==FALSE)
					{
						return FALSE;
					}
				result=TypeA_Select(0x97,pSnr+10,pSak+2);
				if (result==FALSE)
					{
						return FALSE;
					}
				}
		return result;
}
unsigned char TypeA_CardActive_OK(unsigned char *pTagType,unsigned char *pSnr,unsigned char *pSak)
{
unsigned char   result;
		Set_Rf(3);   //turn on radio
		Pcd_ConfigISOType(0);
		result=TypeA_Request(pTagType);//Ѱ�� Standard	 send request command Standard mode
		
		if (result!=TRUE)
		{
			return FALSE;
		}
			if 	((pTagType[0]&0xC0)==0x00)        //M1��,ID��ֻ��4λ
				{
				result=TypeA_Anticollision(0x93,pSnr);
				if (result!=TRUE)
					{
						return FALSE;
					}
				result=TypeA_Select(0x93,pSnr,pSak);
				if (result!=TRUE)
					{
						return FALSE;
					}	
				}
			if 	((pTagType[0]&0xC0)==0x40)     //ID����7λ
				{
				result=TypeA_Anticollision(0x93,pSnr);
				if (result!=TRUE)
					{
						return FALSE;
					}
				result=TypeA_Select(0x93,pSnr,pSak);
				if (result!=TRUE)
					{
						return FALSE;
					}
				result=TypeA_Anticollision(0x95,pSnr+5);
				if (result!=TRUE)
					{
						return FALSE;
					}
				result=TypeA_Select(0x95,pSnr+5,pSak+1);
				if (result!=TRUE)
					{
						return FALSE;
					}
			   	}
			if 	((pTagType[0]&0xC0)==0x80)       //ID����10λ
				{
				result=TypeA_Anticollision(0x93,pSnr);
				if (result!=TRUE)
					{
						return FALSE;
					}
				result=TypeA_Select(0x93,pSnr,pSak);
				if (result!=TRUE)
					{
						return FALSE;
					}
				result=TypeA_Anticollision(0x95,pSnr+5);
				if (result!=TRUE)
					{
						return FALSE;
					}
				result=TypeA_Select(0x95,pSnr+5,pSak+1);
				if (result!=TRUE)
					{
						return FALSE;
					}
				result=TypeA_Anticollision(0x97,pSnr+10);
				if (result!=TRUE)
					{
						return FALSE;
					}
				result=TypeA_Select(0x97,pSnr+10,pSak+2);
				if (result!=TRUE)
					{
						return FALSE;
					}
				}
		return result;
}
//---------------
/*unsigned char	Rst_Data[35];
unsigned int	Total_Rec_Lenght;
unsigned char Cpu_Rst(void)
{
	unsigned char  ret,send_byte[3];
	unsigned int  rece_bitlen;
	Pcd_SetTimer(500);
	send_byte[0] = 0xe0;//ppss byte
	send_byte[1] = 0x80;	//pps0 byte
	Clear_FIFO();	
	ret = Pcd_Comm(Transceive,send_byte,2,Rst_Data,&Total_Rec_Lenght );
	//Total_Rec_Lenght=Total_Rec_Lenght/8;
	return ret;		
}*/