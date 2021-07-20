
#ifndef  H_ANALY_USBL_H
#define  H_ANALY_USBL_H

#include<iostream>
#include<string.h>
#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h> 
#include<unistd.h> 
#include<time.h>
#include<thread>
#include<pthread.h>
#include<mutex>
#include<netinet/in.h>
#include<math.h>
#include<ctime>
#include<chrono>
#include"serial/Serial_Data.h"
#include"lib/threadsafe_queue.h"
#include"lib/CoordTrans.hpp"

#define USBL_HEADER1  0X6B
#define USBL_HEADER2  0XB6

//惯导地址信息
#define Header_ByNums          2
#define CheckSum_ByNums        1

//组合导航工作模式
#define Max_Perframe_Nums      200

//轴数
#define Sins_Axis_Nums         3


#define MAX_SEND_CONFDVLNUM    100
#define MAX_RECV_CONFDVLNUM    2000


class Analy_USBL_Data
{

public:

	int valid_frame_flag = 0;
	int frame_header_num = 0;
	bool recv_frame_flag = false;
	bool debug_flag = false;
	int frame_invaild_num = 0;
	int max_display_data_num = 1;
	enum Analy_Frame_data_State
	{
		header_state = 1,
		load_data_state = 3,
		check_state = 4,
	}analy_frame_data_state;

	int Data_State = header_state;

	std::chrono::time_point<std::chrono::system_clock> start_time;
	std::chrono::time_point<std::chrono::system_clock> end_time;

	char data_file_name[100];
	FILE * filestream;

	enum USBL_Data_State
	{
		no_data_state = 0,
		valid_data_state = 1,
		angle_invalid_state = 2,
		ship_pos_invalid_state = 3,
		enu_pos_invalid_state = 4,
	};

	#pragma pack(1)	
	//惯导输出信息
	struct USBL_Info
	{
		unsigned long int cur_time;
		unsigned long int measurement_time;
		char address;
		float x_ship; //单位:m
		float y_ship; //单位:m
		float z_ship; //单位:m
		float e_ship; //单位:m
		float n_ship; //单位:m
		float u_ship; //单位:m
		float pitch; //单位:deg
		float roll; //单位:deg
		float yaw; //单位:deg
		double lat_gps;
		double lon_gps;		
		unsigned long int propagation_time;
		float rssi;
		float integrity;
		float accuracy;
		unsigned long int timestamp; //精确到 1us
	} USBL_Data;


	struct USBL_Info_Nvidia
	{  
		USBL_Info usbl_data;
		double timestamp_nvidia;
	}USBL_Data_Nvidia;     

	#pragma pack()

	Analy_USBL_Data();
	~Analy_USBL_Data();
	void Init_Data(void);
	bool Analy_Frame( char Receive_Data,bool disp_flag);
	void NavInfo_To_File(USBL_Info_Nvidia * data);
	static  char CheckSum( char * nav_data, unsigned int data_len);

protected:


private:


}; 


#endif


