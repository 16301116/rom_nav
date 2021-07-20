// SDO:send:no myself ;   receive: myself;
// error:send:muself
// noSDO: gps obstable: equal to SDO
// selftest:send:destination   receive:youself(equal)
#include <iostream>
#include <time.h>
#include <chrono>
#include "udp/UDP_Communication.h"
#include "sensor/Analy_Sins_Data.h"


using namespace std;
using namespace chrono;


UDP_Communication::UDP_Communication()
{

}



UDP_Communication::UDP_Communication(int recv_port,int timewait)
{
	sock_recv=socket(AF_INET,SOCK_DGRAM,0);
	if(sock_recv<0)
	{
		perror("socket error\n");
		exit(1);
	}
	memset(&addr_recv,0,sizeof(struct sockaddr_in));
	addr_recv.sin_family = AF_INET;
	addr_recv.sin_port = htons(recv_port);
	addr_recv.sin_addr.s_addr = htonl(INADDR_ANY);
	if(bind(sock_recv,(struct sockaddr*)&addr_recv,sizeof(struct sockaddr_in))<0)
	{
		perror("bind error");
	}

	dest_len = sizeof(struct sockaddr_in);
	//设置超时时间
	if(timewait>0)
   	 {
		struct timeval tv_out;
		tv_out.tv_sec = timewait;
		tv_out.tv_usec = 0;
		setsockopt(sock_recv,SOL_SOCKET,SO_RCVTIMEO,&tv_out,sizeof(tv_out));	
	}

}




UDP_Communication::UDP_Communication(std::string ip,int send_port,int timewait)
{
	sock_send = socket(AF_INET,SOCK_DGRAM,0);
	if(sock_send < 0)
	{ 
		perror("sock_send error\n");
		exit(1); 
	}

	// delete
	memset(&addr_send,0,sizeof(struct sockaddr_in));
	addr_send.sin_family = AF_INET;
	addr_send.sin_port = htons(send_port);
	addr_send.sin_addr.s_addr = inet_addr(ip.c_str());

	dest_len = sizeof(struct sockaddr_in);
	//设置超时时间
	if(timewait>0)
    {
		struct timeval tv_out;
		tv_out.tv_sec = timewait;
		tv_out.tv_usec = 0;
		setsockopt(sock_send,SOL_SOCKET,SO_RCVTIMEO,&tv_out,sizeof(tv_out));
	}

}




UDP_Communication::UDP_Communication(std::string ip,int send_port,int recv_port,int timewait,int isServer)
{
	sock_send = socket(AF_INET,SOCK_DGRAM,0);
	if(sock_send < 0)
	{ 
		perror("sock_send error\n");
		exit(1); 
	}

	// delete
	memset(&addr_send,0,sizeof(struct sockaddr_in));
	addr_send.sin_family = AF_INET;
	addr_send.sin_port = htons(send_port);
	addr_send.sin_addr.s_addr = inet_addr(ip.c_str());

	if(isServer>0) // receive data or not 
    {
		sock_recv = socket(AF_INET,SOCK_DGRAM,0);
		if(sock_recv<0)
		{
			perror("sock_recv error\n");
			exit(1);
		}
		memset(&addr_recv,0,sizeof(struct sockaddr_in));
		addr_recv.sin_family = AF_INET;
		addr_recv.sin_port = htons(recv_port);
		addr_recv.sin_addr.s_addr = htonl(INADDR_ANY);	
	}
	dest_len = sizeof(struct sockaddr_in);
	//设置超时时间
	if(timewait>0)
    {
		struct timeval tv_out;
		tv_out.tv_sec = timewait;
		tv_out.tv_usec = 0;
		setsockopt(sock_send,SOL_SOCKET,SO_RCVTIMEO,&tv_out,sizeof(tv_out));
		if(isServer)	
		{
			setsockopt(sock_recv,SOL_SOCKET,SO_RCVTIMEO,&tv_out,sizeof(tv_out));
		}		
	
	}
	if(bind(sock_recv,(struct sockaddr*)&addr_recv,sizeof(struct sockaddr_in))<0)
	{
		perror("bind error");
		std::cout<<"ip = "<<ip<<std::endl;
		std::cout<<"send_port = "<<send_port<<std::endl;
		std::cout<<"recv_port = "<<recv_port<<std::endl;

	}

}



UDP_Communication::UDP_Communication(std::string ip,int port,int timewait,int isServer)
{
	sock_send = socket(AF_INET,SOCK_DGRAM,0);
	if(sock_send<0)
	{
		perror("socket error\n");
		exit(1);
	}
	memset(&addr_send,0,sizeof(struct sockaddr_in));		
	addr_send.sin_family=AF_INET;
	addr_send.sin_port=htons(port);
	addr_send.sin_addr.s_addr=inet_addr(ip.c_str());

	if(isServer>0) // receive data or not 
   	{
		sock_recv=socket(AF_INET,SOCK_DGRAM,0);
		if(sock_recv<0)
		{
			perror("socket error\n");
			exit(1);
		}
		memset(&addr_recv,0,sizeof(struct sockaddr_in));
		addr_recv.sin_family = AF_INET;
		addr_recv.sin_port = htons(port);
		addr_recv.sin_addr.s_addr = htonl(INADDR_ANY);
		if(bind(sock_recv,(struct sockaddr*)&addr_recv,sizeof(struct sockaddr_in))<0)
		{
			perror("bind error");
			std::cout<<"ip = "<<ip<<std::endl;
			std::cout<<"port = "<<port<<std::endl;
		}
	}
	dest_len = sizeof(struct sockaddr_in);
	//设置超时时间
	if(timewait>0)
   	 {
		struct timeval tv_out;
		tv_out.tv_sec = timewait;
		tv_out.tv_usec = 0;
		setsockopt(sock_send,SOL_SOCKET,SO_RCVTIMEO,&tv_out,sizeof(tv_out));
		if(isServer)		
		{
			setsockopt(sock_recv,SOL_SOCKET,SO_RCVTIMEO,&tv_out,sizeof(tv_out));
		}
			
	}

}






/*
 *函数功能：
 *输入参数：
 *输出参数：
 *备    注：
 *
 */
int UDP_Communication::Send_String(char * message,int size)
{
    
    int Send_Num = sendto(sock_send,message,size,0,(struct sockaddr*)&addr_send,dest_len);
    return(Send_Num);
    
}


/*
 *函数功能：
 *输入参数：
 *输出参数：
 *备    注：
 *
 */
int UDP_Communication::Recv_String(char *recv_buf,int size)
{
	int Rece_Num = recvfrom(sock_recv,recv_buf,size,0,(struct sockaddr*)&addr_recv,&dest_len);
	return(Rece_Num);
}




/*
 *函数功能：
 *输入参数：
 *输出参数：
 *备    注： NavRecorder_Info
 *   
 */
int UDP_Communication::Send_Encoder_SDOData(UDP_Info UDP_Dt)
{

    int Fail_Time = 0;
    for(int i=0;i<SDO_FAILTIME;i++)
    {

		int Send_Num = Send_Encoder(UDP_Dt);   
		if( Send_Num<0 )  { return Send_Num ; }
		bool Recv_Fail_Flag = false;
		double mil_sec = 0;		
		auto start_time = system_clock::now();
		do
		{
			auto end_time = system_clock::now();
			auto duration = duration_cast<microseconds>(end_time - start_time);
			mil_sec = double(1000*duration.count()) * microseconds::period::num / microseconds::period::den;
					
			UDP_Info * UDP_FrameData;
			int Recv_Flag = Recv_Decoder(&UDP_FrameData,Max_UDPFrameNum);  
			if(UDPRECV_ERROR==Recv_Flag)  
			{
				Recv_Fail_Flag = true;
				continue;
			}
			if(DECODER_ERROR==Recv_Flag)
			{
				safe_free((char *)UDP_FrameData);
				Recv_Fail_Flag = true;
				continue;
			}   
			bool Equal_Flag = IsSDOEqual(UDP_FrameData, &UDP_Dt); 
			safe_free((char *)UDP_FrameData);   
			if(Equal_Flag) 
			{
				Fail_Time = 0; 
				Recv_Fail_Flag = false;
				break;
			}
			else
			{
				Recv_Fail_Flag = true;	
			}
											
		}while(mil_sec<SDOWAIT_MIlSEC);	
		
		if(Recv_Fail_Flag)
		{
			Fail_Time++;
		}
		if(Fail_Time==0)
		{
			break;
		}

    }

	
   return Fail_Time;

}



/*
 *函数功能：
 *输入参数：
 *输出参数：norm:>=0
 *备    注： NavRecorder_Info   GPStoCamare
 *   
 */
void UDP_Communication::Send_Encoder_Data(char * data, int num,unsigned short Msg_ID, char Master_ID,char Sub_ID)
{
    
    UDP_Info   UDP_Data;
    static unsigned char Conf_Code ;
    UDP_Data.HeaderData_ID = UDP_HeaderDataID;
    UDP_Data.LoadData_Nums = num;
    UDP_Data.Verf_Code = Conf_Code;   
    UDP_Data.MasterID = Master_ID;
    UDP_Data.SubID = Sub_ID;    
    UDP_Data.Msg_ID = Msg_ID ;   
    UDP_Data.LoadData = data;  
    int Send_Num = Send_Encoder(UDP_Data);   
    if( Send_Num<0 )  
	{ 
		return;
	}
    Conf_Code++; 
    if(MAX_SEQ==Conf_Code)   { Conf_Code = 0x00; }

}




/*
 *函数功能：
 *输入参数：
 *输出参数：norm:>=0
 *备    注： NavRecorder_Info   GPStoCamare
 *   
 */
void UDP_Communication::Send_Encoder_Data_ROV(char * data, int num,unsigned short Msg_ID, char Master_ID,char Sub_ID)
{
    
    UDP_Info_ROV   UDP_Data;
    static unsigned char Conf_Code ;
    UDP_Data.HeaderData_ID = UDP_HeaderDataID;
    UDP_Data.LoadData_Nums = num;
	//ADD
	UDP_Data.PRC = 0X00;
	UDP_Data.PSTATE = 0X00;

    UDP_Data.Verf_Code = Conf_Code;   
    UDP_Data.MasterID = Master_ID;
    UDP_Data.SubID = Sub_ID;    
    UDP_Data.Msg_ID = Msg_ID ;   
	//UDP_Data.Msg_ID = (unsigned short)0x3000;   
    UDP_Data.LoadData = data;  
    int Send_Num = Send_Encoder_ROV(UDP_Data);   
    if( Send_Num<0 )  
	{ 
		return  ;
	}
    Conf_Code++; 
    if(MAX_SEQ==Conf_Code)   
	{ 
		Conf_Code = 0x00; 
	}

}



int UDP_Communication::Send_Encoder(UDP_Info & UDP_Data)
{
	char * Frame_Data = NULL;
	unsigned int Data_Nums = 0;
	Encoder(UDP_Data , &Frame_Data,Data_Nums);
	int Send_Num = Send_String(Frame_Data,Data_Nums);
	safe_free((char *)Frame_Data);
    return (Send_Num);
}




int UDP_Communication::Send_Encoder_ROV(UDP_Info_ROV & UDP_Data)
{
	char * Frame_Data = NULL;
	unsigned int Data_Nums = 0;
	Encoder_ROV(UDP_Data , &Frame_Data,Data_Nums);

	if(debug_flag)
	{
		printf("Data_Nums = %d\n",Data_Nums);
		printf("Frame_Data begin\n");
		for (int i=0;i<Data_Nums;i++)
		{
			printf("0x%.2x,",Frame_Data[i]);
		}
		printf("Frame_Data end\n");
	}

	int Send_Num = Send_String(Frame_Data,Data_Nums);
	safe_free((char *)Frame_Data);
    return (Send_Num);
}



void UDP_Communication::Encoder_ROV(UDP_Info_ROV & UDP_Data, char ** dst, unsigned int & Data_Nums)
{
    
    unsigned int i = 0;
	Data_Nums = NoLoadData_Nums+UDP_Data.LoadData_Nums; // NoLoadData_Nums

	char * FrameData = (char *)malloc(Data_Nums); 
    if(NULL == FrameData)
    {  
        exit(1);
    }
   
    memcpy(FrameData+i,&(UDP_Data.HeaderData_ID),sizeof(UDP_Data.HeaderData_ID));          i+=sizeof(UDP_Data.HeaderData_ID);
    memcpy(FrameData+i,&(UDP_Data.LoadData_Nums),sizeof(UDP_Data.LoadData_Nums));          i+=sizeof(UDP_Data.LoadData_Nums);

    memcpy(FrameData+i,&(UDP_Data.PRC),sizeof(UDP_Data.PRC));                              i+=sizeof(UDP_Data.PRC);
    memcpy(FrameData+i,&(UDP_Data.PSTATE),sizeof(UDP_Data.PSTATE));                        i+=sizeof(UDP_Data.PSTATE);

    memcpy(FrameData+i,&(UDP_Data.Verf_Code),sizeof(UDP_Data.Verf_Code));                  i+=sizeof(UDP_Data.Verf_Code);
    memcpy(FrameData+i,&(UDP_Data.MasterID),sizeof(UDP_Data.MasterID));                    i+=sizeof(UDP_Data.MasterID);
    memcpy(FrameData+i,&(UDP_Data.SubID),sizeof(UDP_Data.SubID));                          i+=sizeof(UDP_Data.SubID);
    memcpy(FrameData+i,&(UDP_Data.Msg_ID),sizeof(UDP_Data.Msg_ID));                        i+=sizeof(UDP_Data.Msg_ID);
    memcpy(FrameData+i,UDP_Data.LoadData,UDP_Data.LoadData_Nums);                          i+=UDP_Data.LoadData_Nums;  

    unsigned char CRC_H = 0;
    unsigned char CRC_L = 0;
    CheckCRC((unsigned char *)FrameData,i,&CRC_H,&CRC_L);

    memcpy(FrameData+i,&CRC_L,sizeof(CRC_L));   i+=sizeof(CRC_L);
    memcpy(FrameData+i,&CRC_H,sizeof(CRC_H));   i+=sizeof(CRC_H);    

    *dst = FrameData;

}




/*
 *函数功能：
 *输入参数：
 *输出参数： IsSDO
 *备   注： 
 *   
 */
char UDP_Communication::FindSDO( unsigned short int Msg_ID )
{
	
	char sdo_flag = NOSDO_FLAG;
				
	switch(Msg_ID)
	{
		/*
		case GPS_Planner_INFO_MSGID:
			sdo_flag = ISSDO_FLAG;
			break;
		case SelfCheck_G_MSGID:
			sdo_flag = ISSDO_FLAG;
			break;
		case SelfCheck_W_MSGID:
			sdo_flag = ISSDO_FLAG;
			break;
		case Nav_Init_INFO_MSGID:
			sdo_flag = ISSDO_FLAG;
			break;
		
		case Nav_Start_MSGID:
			sdo_flag = ISSDO_FLAG;
			break;
		case Nav_END_MSGID:
			sdo_flag = ISSDO_FLAG;
			break;
		default:
			sdo_flag = NOSDO_FLAG;
			break;
		*/	
	}

	return sdo_flag;

}



/*
 *函数功能：
 *输入参数：
 *输出参数：
 *备    注：
 *
 */
bool UDP_Communication::IsSDOEqual(UDP_Info * UDP_DataSDO, UDP_Info * UDP_Data)
{

    if(UDP_DataSDO->MasterID!=UDP_Data->MasterID) 
	{
        	return false;
	}
     if(UDP_DataSDO->SubID!=UDP_Data->SubID) 
	{
        	return false;
	}  
     if(UDP_DataSDO->Msg_ID!=UDP_Data->Msg_ID) 
	{
        	return false;  
	}
     if(SDO_SIZE!=UDP_DataSDO->LoadData_Nums)
	{
       	 	return false;
	}

    for(int i=0;i<SDO_SIZE;i++)
    {
        if(*(UDP_DataSDO->LoadData+i)!=*(SDO_Data+i))  
        {
            return false; 
        }

    }

    return true;

}





/*
 *函数功能：
 *输入参数：
 *输出参数：
 *备    注： free_data
 *
 */  
char UDP_Communication::Recv_Decoder(UDP_Info ** UDP_FrameData,int Size)
{

	char Recv_Buf[Max_UDPFrameNum] = {0};
	int Recv_Num = Recv_String(Recv_Buf,Size);

    if(-1==Recv_Num)
    {
        return UDPRECV_ERROR;
    }
    
    return(Decoder(Recv_Buf , Recv_Num, UDP_FrameData));

}



/*
 *函数功能：
 *输入参数：
 *输出参数：
 *备    注： free_data
 *
 */  
char UDP_Communication::Recv_Decoder_ROV(UDP_Info_ROV ** UDP_FrameData,int Size)
{

	char Recv_Buf[Max_UDPFrameNum] = {0};
	int Recv_Num = Recv_String(Recv_Buf,Size);
	//printf("Recv_Decoder_ROV:Recv_Num = %d\n",Recv_Num);

    if(-1==Recv_Num)
    {
        return UDPRECV_ERROR;
    }
    
    return(Decoder_ROV(Recv_Buf , Recv_Num, UDP_FrameData));

}




/*
 *函数功能：
 *输入参数：
 *输出参数：
 *备    注：
 *
 */
void UDP_Communication::Encoder(UDP_Info & UDP_Data, char ** dst, unsigned int & Data_Nums)
{
    
    unsigned int i = 0;
	Data_Nums = NoLoadData_Nums+UDP_Data.LoadData_Nums;

	char * FrameData = (char *)malloc(Data_Nums); 
    if(NULL == FrameData)
    {  
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
        exit(1);
    }
   
    memcpy(FrameData+i,&(UDP_Data.HeaderData_ID),sizeof(UDP_Data.HeaderData_ID));          i+=sizeof(UDP_Data.HeaderData_ID);
    memcpy(FrameData+i,&(UDP_Data.LoadData_Nums),sizeof(UDP_Data.LoadData_Nums));          i+=sizeof(UDP_Data.LoadData_Nums);
    memcpy(FrameData+i,&(UDP_Data.Verf_Code),sizeof(UDP_Data.Verf_Code));                  i+=sizeof(UDP_Data.Verf_Code);
    memcpy(FrameData+i,&(UDP_Data.MasterID),sizeof(UDP_Data.MasterID));                    i+=sizeof(UDP_Data.MasterID);
    memcpy(FrameData+i,&(UDP_Data.SubID),sizeof(UDP_Data.SubID));                          i+=sizeof(UDP_Data.SubID);
    memcpy(FrameData+i,&(UDP_Data.Msg_ID),sizeof(UDP_Data.Msg_ID));                        i+=sizeof(UDP_Data.Msg_ID);
    memcpy(FrameData+i,UDP_Data.LoadData,UDP_Data.LoadData_Nums);                          i+=UDP_Data.LoadData_Nums;  

    unsigned char CRC_H = 0;
    unsigned char CRC_L = 0;
    CheckCRC((unsigned char *)FrameData,i,&CRC_H,&CRC_L);

    memcpy(FrameData+i,&CRC_L,sizeof(CRC_L));   i+=sizeof(CRC_L);
    memcpy(FrameData+i,&CRC_H,sizeof(CRC_H));   i+=sizeof(CRC_H);    

    *dst = FrameData;

}





/*
 *函数功能：
 *输入参数：
 *输出参数：
 *备    注： Data_Nums
 *
 * Decoder(Recv_Buf , Recv_Num, UDP_FrameData)
 */
char UDP_Communication::Decoder( char * FrameData , unsigned int Data_Nums, UDP_Info ** UDP_FrameData)
{
    unsigned int i = 0;
    
    UDP_Info * UDP_Data = ( UDP_Info*)malloc(sizeof(UDP_Info)); 
    if(NULL == UDP_Data)
    {
        exit(1);    
    }

    * UDP_FrameData = UDP_Data;
    
    if(Data_Nums<MIN_UDPNUM)    
	{ 
		return DECODER_ERROR;
	}  
    memcpy(&(UDP_Data->HeaderData_ID),FrameData+i,sizeof(UDP_Data->HeaderData_ID));   i+=sizeof(UDP_Data->HeaderData_ID);
    if(UDP_HeaderDataID!= UDP_Data->HeaderData_ID) 
	{
		return DECODER_ERROR;
	}
    memcpy(&(UDP_Data->LoadData_Nums),FrameData+i,sizeof(UDP_Data->LoadData_Nums));   i+=sizeof(UDP_Data->LoadData_Nums);
    if(Data_Nums<(MIN_UDPNUM + UDP_Data->LoadData_Nums))    
	{
		return DECODER_ERROR;
	}
    memcpy(&(UDP_Data->Verf_Code),FrameData+i,sizeof(UDP_Data->Verf_Code));           i+=sizeof(UDP_Data->Verf_Code);
    memcpy(&(UDP_Data->MasterID),FrameData+i,sizeof(UDP_Data->MasterID));             i+=sizeof(UDP_Data->MasterID);
    memcpy(&(UDP_Data->SubID),FrameData+i,sizeof(UDP_Data->SubID));                   i+=sizeof(UDP_Data->SubID);   
    memcpy(&(UDP_Data->Msg_ID),FrameData+i,sizeof(UDP_Data->Msg_ID));                 i+=sizeof(UDP_Data->Msg_ID);     
 
    UDP_Data->LoadData = FrameData + i;   i+=UDP_Data->LoadData_Nums;
    
    unsigned char CRC_H = 0;
    unsigned char CRC_L = 0;
    CheckCRC((unsigned char *)FrameData,i,&CRC_H,&CRC_L);

    if(CRC_L==(unsigned char)FrameData[i]&&CRC_H==(unsigned char )FrameData[i+1])    
	{ 
		return DECODER_NORM;
	}
    else 
	{ 
		return DECODER_ERROR;
	}
   
}




/*
 *函数功能：
 *输入参数：
 *输出参数：
 *备    注： Data_Nums
 *
 * Decoder(Recv_Buf , Recv_Num, UDP_FrameData)
 */
char UDP_Communication::Decoder_ROV( char * FrameData , unsigned int Data_Nums, UDP_Info_ROV ** UDP_FrameData)
{
    unsigned int i = 0;
    
    UDP_Info_ROV * UDP_Data = ( UDP_Info_ROV*)malloc(sizeof(UDP_Info_ROV)); 
    if(NULL == UDP_Data)
    {
        exit(1);    
    }

    * UDP_FrameData = UDP_Data;
    if(Data_Nums<ROV_MIN_UDPNUM)    
	{	
		return DECODER_ERROR; 
	} 
    memcpy(&(UDP_Data->HeaderData_ID),FrameData+i,sizeof(UDP_Data->HeaderData_ID));   i+=sizeof(UDP_Data->HeaderData_ID);
    if(UDP_HeaderDataID!= UDP_Data->HeaderData_ID) 
	{
		return DECODER_ERROR;
	}
    memcpy(&(UDP_Data->LoadData_Nums),FrameData+i,sizeof(UDP_Data->LoadData_Nums));   i+=sizeof(UDP_Data->LoadData_Nums);
    if(Data_Nums<(ROV_MIN_UDPNUM + UDP_Data->LoadData_Nums))     
	{
		return DECODER_ERROR;
	}
    memcpy(&(UDP_Data->PRC),FrameData+i,sizeof(UDP_Data->PRC));           i+=sizeof(UDP_Data->PRC);
    memcpy(&(UDP_Data->PSTATE),FrameData+i,sizeof(UDP_Data->PSTATE));     i+=sizeof(UDP_Data->PSTATE);
    memcpy(&(UDP_Data->Verf_Code),FrameData+i,sizeof(UDP_Data->Verf_Code));           i+=sizeof(UDP_Data->Verf_Code);
    memcpy(&(UDP_Data->MasterID),FrameData+i,sizeof(UDP_Data->MasterID));             i+=sizeof(UDP_Data->MasterID);
    memcpy(&(UDP_Data->SubID),FrameData+i,sizeof(UDP_Data->SubID));                   i+=sizeof(UDP_Data->SubID);   
    memcpy(&(UDP_Data->Msg_ID),FrameData+i,sizeof(UDP_Data->Msg_ID));                 i+=sizeof(UDP_Data->Msg_ID);     
    UDP_Data->LoadData = FrameData + i;   i+=UDP_Data->LoadData_Nums;
    
    unsigned char CRC_H = 0;
    unsigned char CRC_L = 0;
    CheckCRC((unsigned char *)FrameData,i,&CRC_H,&CRC_L);

    if(CRC_L==(unsigned char)FrameData[i]&&CRC_H==(unsigned char )FrameData[i+1])    
	{ 
		return DECODER_NORM;
	}
    
    else 
	{ 
		return DECODER_ERROR;
	}
   
}



void UDP_Communication::CheckCRC(unsigned char *buf,int len,unsigned char * CRC_H,unsigned char* CRC_L)
{
	unsigned short i,j,tmp,CRC16;
	CRC16=0xffff;
	for (i=0;i<len;i++)
	{
		CRC16=*buf^CRC16;
		for (j=0;j< 8;j++)
		{
			tmp=CRC16 & 0x0001;
			CRC16 =CRC16 >>1;
			if (tmp)
				CRC16=CRC16 ^ 0xA001;
		}
		buf++;
	}
	CRC_H[0]=CRC16>>8;
	CRC_L[0]=CRC16&0xff;
}






