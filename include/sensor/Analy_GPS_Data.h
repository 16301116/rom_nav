
#ifndef  H_Analy_GPS_Data_H
#define  H_Analy_GPS_Data_H

#include<iostream>
#include<string.h>
#include<thread>
#include<mutex>
#include"lib/threadsafe_queue.h"
#include<cmath>
#include<ctime>
#include<chrono>
#include"serial/Serial_Data.h"

using namespace gdface;	
using namespace std;

#define GPS_HEADER1  0xEB
#define GPS_HEADER2  0x90
#define GPS_MSG_ID   0X01

#define GPS_Header_ByNums     2
#define GPS_ID_ByNums         1
#define GPS_CHECK_ByNums      1

#define GPS_Max_Frame_Nums  1000

class Analy_GPS_Data
{

public:

	int min_invalid_star = 4;
	float max_gps_speed = 5;
	int max_display_data_num = 20;
	bool debug_flag = false;	
	//bool debug_flag = true;
	
	std::chrono::time_point<std::chrono::system_clock> start_time;
	std::chrono::time_point<std::chrono::system_clock> end_time;
	double add_second;
	
	enum GPS_Data_State
	{
		no_data_state = 0,
		valid_data_state = 1,
		pos_vel_invalid_state = 2,
		pos_invalid_state = 3,
	};

	#pragma pack(1)
	//惯导输出信息
	struct GPS_Output_Info
	{  
		long int lat;
		long int lon;
		int alt;
		unsigned short int speed;
		unsigned char star;
		unsigned short int pdop;
		unsigned short int hdop;
		unsigned short int vdop;
		unsigned char gnss_state;
		unsigned int timestamp;
	}GPS_Output_Data; 

	struct nmea_utc_info
	{              
		unsigned short int year; //年份
		unsigned char month; //月份
		unsigned char date; //日期
		unsigned char hour;  //小时
		unsigned char min;  //分钟
		unsigned char sec;  //秒钟
		unsigned char milsec;  //毫秒
	}nmea_utc_time;

	struct GNSS_FRAME_Info
	{
		//uint16_t head;   				//帧头
		//uint8_t ID;//帧ID：gps数据帧：0x01
		nmea_utc_info utc;          //UTC时间  
		unsigned long int lat;					//纬度 度扩大1000000倍
		unsigned long int lon;			   	//经度 度扩大1000000倍
		int  alt;			 		//海拔高度，除以1000，单位m	 
		unsigned short int speed;	//水平运动速度，000-999，除以1000，单位km/h(GPVTG)
		unsigned short int forward_speed;			//行进方向运动速度，除以1000，单位km/h(KSXT)
		
		unsigned short int direction_angle;	 //方位角，天线1、天线2连线与正北方向夹角，天线1为方向，天线2为位置，除以100，单位°,0-360
		unsigned short int velocity_angle;	 //速度角，车辆行进方向与正北方向夹角，除以100，单位°，0-360
		
		int east_velocity;		 //东向速度，除以1000，单位km/h
		int north_velocity;		 //北向速度，除以1000，单位km/h
		int diurnal_velocity;	 //天向速度，除以1000，单位km/h
		
		int roll;								//横滚角，除以10000，单位°
		int pitch;							//俯仰角，除以10000，单位°
		int yaw;						//航向角，除以10000，单位°
		unsigned char  star;				//用于定位的卫星数,0~12
		unsigned short int pdop;					//位置精度因子，除以100
		unsigned short int hdop;					//水平精度因子，除以100
		unsigned short int vdop;					   //垂直精度因子，除以100
		unsigned char gnss_state;					//卫星定位状态  0:无效解，1：单点定位解，2：伪距差分，4：固定解，5：浮动解
		unsigned int ppscount;	 				//pps计数
		//unsigned int time_tick_pps;				//pps时间戳
		unsigned long int time_tick_pps;
	}GNSS_SEND_FRAME;


	#pragma pack()	

	double lon_now;
	double lat_now;

	char gpsdata_file_name[100];
	FILE * gps_filestream;

	Analy_GPS_Data();
	~Analy_GPS_Data();
	void Init_Data(void);
	void Analy_GPS_Frame( std::vector<char> & recv_Data,int  & Frame_Num,std::vector<GPS_Output_Info> & output_data);
	void Analy_GPS_Frame( std::vector<char> & recv_Data,int  & Frame_Num,std::vector<GNSS_FRAME_Info> & output_data);
	bool Analy_GPS_Frame(char recv_Data,bool disp_flag);
	char CheckSum( char * nav_data, unsigned int data_len);
	void NavInfo_To_File(GPS_Output_Info * data);
	void NavInfo_To_File(GNSS_FRAME_Info * data);
	void NavInfo_To_File(GNSS_FRAME_Info * data,bool valid_flag);

protected:




private:



};


#endif
