#include "fm175xx.h"
#include "type_b.h"
#include "Utility.h"
#include "ISO14443.h"
//#include "cpu_card.h"
#include <string.h>

unsigned char PUPI[4];	


/*****************************************************************************************/
/*���ƣ�TypeB_Halt																		 */
/*���ܣ�����TYPE B��Ƭ����ֹͣ״̬														 */
/*���룺card_sn����Ƭ��PUPI																 */
/*�����																				 */
/*	   	TRUE�������ɹ�																	 */
/*		ERROR������ʧ��																	 */
/*****************************************************************************************/
unsigned char TypeB_Halt(unsigned char *card_sn)
{
	unsigned char  ret,send_byte[5],rece_byte[1],i;
	unsigned int  rece_len;
		Pcd_SetTimer(10);
		send_byte[0] = 0x50;
		for(i = 0; i < 4; i++)
			{
			send_byte[i + 1] =card_sn[i];
			}
		ret=Pcd_Comm(Transceive,send_byte,5,rece_byte,&rece_len);
		return (ret);
}
/*****************************************************************************************/
/*���ƣ�TypeB_WUP																		 */
/*���ܣ�TYPE B��Ƭ����																	 */
/*���룺N/A																				 */
/*�����																				 */
/*		rece_len:��ƬӦ������ݳ��ȣ�buff����ƬӦ�������ָ��							 */
/*		card_sn:��Ƭ��PUPI�ֽ�															 */
/*	   	TRUE�������ɹ�																	 */
/*		ERROR������ʧ��																	 */
/*****************************************************************************************/
unsigned char TypeB_WUP(unsigned int *rece_len,unsigned char *buff,unsigned char *card_sn)
{						
	unsigned char  ret,send_byte[3];
		Pcd_SetTimer(10);
		send_byte[0]=0x05;//APf
		send_byte[1]=0x00;//AFI (00:for all cards)
		send_byte[2]=0x08;//PARAM(WUP,Number of slots =0)
	//	send_byte[2]=0x00;//test
		ret=Pcd_Comm(Transceive,send_byte,3,buff,&(*rece_len));
		if (ret==TRUE)
		{	
			memcpy(card_sn,&buff[1],4);	
			if((buff[10]&0x0f)==0)
			  mypicc[0].Iso14443 = 1;
			else
				mypicc[0].Iso14443 = 0;	//Ĭ��֧��
			}	
		return (ret);
}
/*****************************************************************************************/
/*���ƣ�TypeB_Request																	 */
/*���ܣ�TYPE B��Ƭѡ��																	 */
/*���룺																				 */
/*�����																				 */
/*	   	TRUE�������ɹ�																	 */
/*		ERROR������ʧ��																	 */
/*****************************************************************************************/
unsigned char TypeB_Request(unsigned int *rece_len,unsigned char *buff,unsigned char *card_sn)
{			
	unsigned char  ret,send_byte[5];
		Pcd_SetTimer(10);
		send_byte[0]=0x05;	//APf
		send_byte[1]=0x00;	//AFI (00:for all cards)
		send_byte[2]=0x00;	//PARAM(REQB,Number of slots =0)
		ret=Pcd_Comm(Transceive,send_byte,3,buff,rece_len);
		if (ret==TRUE)	
			memcpy(card_sn,buff+1,4);
		return (ret);
}		
/*****************************************************************************************/
/*���ƣ�TypeB_Select																	 */
/*���ܣ�TYPE B��Ƭѡ��																	 */
/*���룺card_sn����ƬPUPI�ֽڣ�4�ֽڣ�													 */
/*�����																				 */
/*		rece_len��Ӧ�����ݵĳ���														 */
/*		buff��Ӧ�����ݵ�ָ��															 */
/*	   	TRUE�������ɹ�																	 */
/*		ERROR������ʧ��																	 */
/*****************************************************************************************/
unsigned char TypeB_Select(unsigned char *card_sn,unsigned int *rece_len,unsigned char *buff)
{
	unsigned char  ret,send_byte[9],i;
		Pcd_SetTimer(20);
		send_byte[0] = 0x1d;
		for(i = 0; i < 4; i++)
			{
			send_byte[i + 1] =card_sn[i];
			}
		send_byte[5] = 0x00;  //------Param1
		send_byte[6] = 0x08;  //01--24��08--256------Param2
		send_byte[7] = 0x01;  //COMPATIBLE WITH 14443-4------Param3
		send_byte[8] = 0x02;  //CID��01 ------Param4
         
		ret=Pcd_Comm(Transceive,send_byte,9,buff,&(*rece_len));
		return (ret);
}	
//2017-10-21 add by gyf
unsigned char TypeB_WakeUp(unsigned char Afi, unsigned char Param,unsigned int *rece_len,unsigned char *buff,unsigned char *card_sn)
{			
	unsigned char  ret,send_byte[5];
		Pcd_SetTimer(10);
		send_byte[0]=0x05;	//APf
		send_byte[1]=Afi;	//AFI (00:for all cards)
		send_byte[2]=Param;	//PARAM(REQB,Number of slots =0)
		ret=Pcd_Comm(Transceive,send_byte,3,buff,rece_len);
		if (ret==TRUE)	
			memcpy(card_sn,buff+1,4);
		return (ret);
}
unsigned char TypeB_SlotMarker(unsigned char slot,unsigned int *rece_len,unsigned char *buff,unsigned char *card_sn)
{			
	unsigned char  ret,send_byte[5];
		Pcd_SetTimer(10);
		ret=slot;
		send_byte[0]=((ret<<4)&0xf0)+0x05;
		ret=Pcd_Comm(Transceive,send_byte,1,buff,rece_len);
		if (ret==TRUE)	
			memcpy(card_sn,buff+1,4);
		return (ret);
}
/*****************************************************************************************/
/*���ƣ�TypeB_GetUID																	 */
/*���ܣ�����֤ר��ָ��																	 */
/*���룺N/A																				 */
/*�����rece_len���������ݵĳ���														 */
/*		buff���������ݵ�ָ��															 */
/*	   	TRUE�������ɹ�																	 */
/*		ERROR������ʧ��																	 */
/*****************************************************************************************/
unsigned char TypeB_GetUID(unsigned int *rece_len,unsigned char *buff)
{
	unsigned char  ret,send_byte[5];
		Pcd_SetTimer(15);
		send_byte[0]=0x00;
		send_byte[1]=0x36;
		send_byte[2]=0x00;
		send_byte[3]=0x00;
		send_byte[4]=0x08;
		ret=Pcd_Comm(Transceive,send_byte,5,buff,&(*rece_len));
		return (ret);
}	
unsigned char TypeB_Getradom(unsigned int *rece_len,unsigned char *buff)
{
	unsigned char  ret,send_byte[5];
		Pcd_SetTimer(15);
		send_byte[0]=0x00;
		send_byte[1]=0x84;
		send_byte[2]=0x00;
		send_byte[3]=0x00;
		send_byte[4]=0x08;
		ret=Pcd_Comm(Transceive,send_byte,5,buff,&(*rece_len));
		return (ret);
}	
//------------
int TypeB_ADPUNO14443(unsigned char cid,unsigned char *tbuffer,unsigned int length,unsigned char *rbuffer)
{
	unsigned char  ret;
	unsigned int LenByte,n;
		Pcd_SetTimer(20);
		ret=Pcd_CommByte(Transceive,tbuffer,length,rbuffer,&LenByte);
		if(ret==TRUE)
		{				
			return LenByte;		
			}
		else
			return -1;
}	