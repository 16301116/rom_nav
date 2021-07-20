
#ifndef  H_ANALY_PRESSURE_DATA_H
#define  H_ANALY_PRESSURE_DATA_H

#include<iostream>
#include<string.h>
#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h> 
#include<unistd.h> 
#include<time.h>
#include <thread>
#include <pthread.h>
#include <mutex>
#include <netinet/in.h>
#include <math.h>
#include <ctime>
#include <chrono>
#include "serial/Serial_Data.h"
#include "lib/threadsafe_queue.h"
#include "lib/CoordTrans.hpp"


#define PRESSURE_HEADER1  0X8D
#define PRESSURE_HEADER2  0XD8

//惯导地址信息
#define Header_ByNums          2
#define CheckSum_ByNums        1

//组合导航工作模式
#define Max_Perframe_Nums      200

//轴数
#define Sins_Axis_Nums         3


#define MAX_SEND_CONFDVLNUM    100
#define MAX_RECV_CONFDVLNUM    2000


class Analy_Pressure_Data
{

public:

	int valid_frame_flag = 0;
	int frame_header_num = 0;
	bool recv_frame_flag = false;
	bool debug_flag = false;
	int frame_invaild_num = 0;
	int max_display_data_num = 10;
	enum Analy_Frame_data_State
	{
		header_state = 1,
		load_data_state = 3,
		check_state = 4,
	}analy_frame_data_state;

	int Data_State = header_state;

	std::chrono::time_point<std::chrono::system_clock> start_time;
	std::chrono::time_point<std::chrono::system_clock> end_time;

	char pressure_data_file_name[100];
	FILE * pressure_filestream;

	
	#pragma pack(1)	
	//惯导输出信息
	struct Pressure_Sonnar_Info
	{
		float data;
		float temp;
		unsigned long int timestamp;
	} Pressure_Sonnar_Data;

	struct Pressure_Info_Nvidia
	{  
		Pressure_Sonnar_Info pressure_data;
		double timestamp_nvidia;
	}Pressure_Data_Nvidia;     

	#pragma pack()

	Analy_Pressure_Data();
	~Analy_Pressure_Data();
	void Init_Data(void);
	bool Analy_Frame( char Receive_Data,bool disp_flag);
	void NavInfo_To_File(Pressure_Info_Nvidia * data);
	static  char CheckSum( char * nav_data, unsigned int data_len);

protected:


private:


}; 


#endif


