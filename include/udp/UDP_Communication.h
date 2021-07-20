#include <ctime>
#include <chrono>
#include "lib/common.hpp"
#include "sensor/Analy_Sins_Data.h"

#ifndef __H__UDP_COMMUNICATION__H__
#define __H__UDP_COMMUNICATION__H__


// ADD ROV
#define ROV_NAV_INFO_MSGID   0XF004
#define Nav2Ctrl_ROV_MSGID   0x3009


#define   MAX_SEQ                  0XFF
#define   ISSDO_FLAG               0X01
#define   NOSDO_FLAG               0X00
#define   SDO_SIZE                 1
#define   MIN_UDPNUM               10
#define   ROV_MIN_UDPNUM           12 // add two bytes
#define   STATE_NORM               0
#define   UDPRECV_NORM             0
#define   UDPSEND_NORM             0
#define   DECODER_NORM             0
#define   UDPSEND_ERROR            -1
#define   UDPRECV_ERROR            -2
#define   DECODER_ERROR            -3


#define MAX_GPSPLANNER_NUM          100


#define Landstation_IP          "192.168.1.15"
#define Controller_IP           "192.168.1.10" 
#define Nvidia_AGX_IP           "192.168.1.16" 
#define Nav_MCU_IP              "192.168.1.30" 
#define Nav_Nvidia_IP           "192.168.1.31" 
#define USBL_BASE_IP            "192.168.1.32"
#define Broadcast_IP            "0.0.0.0"


// main port 
#define Landstation_Port        8000
#define Controller_Port         5001
#define Nvidia_AGX_IP           5002
#define Nav_MCU_Port            5003
#define Nav_Nvidia_Port         5004
#define USBL_BASE_Port          5005

#define NAV_Msg_ID                         0x3000
#define SHUTDOWN_Msg_ID        0X3003
#define SETNAVMODE_Msg_ID    0X3006

//#define SHUTDOWN_R_Msg_ID       3002


#define SDOWAIT_MIlSEC          3000
#define SDO_FAILTIME               3

class UDP_Communication{

	public:

    char navigation_master_id = 0x03; //me navigation
    char control_sub_id  = 0x02; // control

	bool debug_flag = false;
	//bool debug_flag = true;

	int sock_recv;
	int sock_send;
	int recv_num;
	socklen_t  dest_len;
	struct sockaddr_in addr_send;
	struct sockaddr_in addr_recv;


	unsigned char seq ;
	unsigned char Max_Verf_Code;
    const unsigned char UDP_HeaderDataID = 0XFE;
   	const unsigned char NoLoadData_Nums= 12;
	const unsigned int  Max_UDPFrameNum= 1000;
     
	const unsigned char Control_Cabin_MasterID= 57; 
	const unsigned char Control_Cabin_SubID= 0;
	const unsigned char Xavier_MasterNavID= 60;
	const unsigned char Xavier_SubNavID= 1;
	const unsigned char Xavier_MasterImgID= 60;
	const unsigned char Xavier_SubImgID= 0;

	unsigned char SDO_Data[SDO_SIZE];

	bool Nav_Work_Flag ;

	//bool Nav_WorkOver_Flag ;
	bool SelfCheck_G_Flag;
	bool SelfCheck_W_Flag;
	bool Nav_Init_Flag;
	bool Nav_Init_Rdy_Flag;
	bool GPS_Rdy_Flag;
	bool Prop_Info_Rdy_Flag;
   	bool Sonnar_Rdy_Flag;
	bool GPSPlanner_Rdy_Flag;
	bool AUVheart_Rdy_Flag;
   	bool GPSnavinit_Rdy_Flag;

	#pragma pack(1)
	struct  UDP_Info
    {      
    	unsigned char HeaderData_ID;  //1
        unsigned short int LoadData_Nums; //1+2
        unsigned char Verf_Code; //1+2+1
        unsigned char MasterID; //1+2+1+1
        unsigned char SubID; //1+2+1+1+1
        unsigned short int Msg_ID; //1+2+1+1+1+2 = 8
        char * LoadData;
    };
		
	UDP_Info UDP_Data;

	struct  UDP_Info_ROV
    {      
    	unsigned char HeaderData_ID;
        unsigned short int LoadData_Nums;
		char PRC = 0;
		char PSTATE = 0;
        unsigned char Verf_Code;
        unsigned char MasterID;
        unsigned char SubID;
        unsigned short int Msg_ID;
        char * LoadData;
    };
		
	UDP_Info_ROV UDP_Data_ROV;
	#pragma pack()


	public:
	UDP_Communication();
	UDP_Communication(int recv_port,int timewait);
	UDP_Communication(std::string ip,int send_port,int timewait);
	UDP_Communication(std::string ip,int port,int timewait,int isServer);
	UDP_Communication(std::string ip,int send_port,int recv_port,int timewait,int isServer);

	char * Begin_Camera_Data = "begincamera";
	char * End_Camera_Data = "endcamera";
	bool Camara_SDOfail_Flag;
	
    int Send_String(char * message,int size);
    int Recv_String(char * recv_buf,int size);
	int Send_Encoder_SDOData(UDP_Info UDP_Dt);
   	char Recv_Encoder_Data(FILE * filestream);
   	bool IsSDOEqual(UDP_Info * UDP_DataSDO, UDP_Info * UDP_Data);
	void CheckCRC(unsigned char *buf,int len,unsigned char * CRC_H,unsigned char* CRC_L);
	char FindSDO( unsigned short int Msg_ID );
	
	// AUV
    void Encoder(UDP_Info & UDP_Data , char ** dst,unsigned int & Data_Nums);
    char Decoder( char * FrameData , unsigned int Data_Nums, UDP_Info ** UDP_FrameData);
	int Send_Encoder(UDP_Info & UDP_Data);
    void Send_Encoder_Data(char * data, int num,unsigned short Msg_ID, char Master_ID,char Sub_ID);
	char Recv_Decoder(UDP_Info ** UDP_FrameData,int Size);

	// ROV
	void Encoder_ROV(UDP_Info_ROV & UDP_Data, char ** dst, unsigned int & Data_Nums);
	char Decoder_ROV( char * FrameData , unsigned int Data_Nums, UDP_Info_ROV ** UDP_FrameData);
	int Send_Encoder_ROV(UDP_Info_ROV & UDP_Data);
	void Send_Encoder_Data_ROV(char * data, int num,unsigned short Msg_ID, char Master_ID,char Sub_ID);
	char Recv_Decoder_ROV(UDP_Info_ROV ** UDP_FrameData,int Size);

};

#endif



