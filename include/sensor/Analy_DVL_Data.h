
#ifndef  H_Analy_DVL_Data_H
#define  H_Analy_DVL_Data_H

#include<iostream>
#include<string.h>
#include"serial/Serial_Data.h"
#include<thread>
#include<mutex>
#include<ctime>
#include<chrono>
#include"lib/threadsafe_queue.h"

using namespace gdface;	
using namespace std;

#define DVL_HEADER1  0x5A
#define DVL_HEADER2  0xA5

#define Max_Perframe_Nums      200


#define DVL_Header_ByNums     2
#define DVL_ID_ByNums         1
#define DVL_CHECK_ByNums      1

#define Header_ByNums         2
#define CheckSum_ByNums       1

#define DVL_Max_Frame_Nums  1000

class Analy_DVL_Data
{

public:

	float max_vel = 8;  // /m/s
	int max_display_data_num = 20;

	enum Analy_Frame_data_State
	{
		header_state = 1,
		load_data_state = 3,
		check_state = 4,
	}analy_frame_data_state;

	int Data_State = header_state;

	// add lost data and mischeck data here==========begin==========
	enum Frame_data_State
	{
		check_right_state = 1,
		check_wrong_state = 2,
		lost_frame_state = 3,
	}frame_data_state;

	enum DvlData_State
	{
		no_data_state = 0,
		valid_data_state = 1,
		vel_bottom_invalid_state = 2,
		vel_water_invalid_state = 3,
	};

	int add_lost_frame_num_max = 10;
	int gap_time_ms_mean = 10; //10ms(100Hz)
	int gap_time_ms_max = 15; //15ms 
	int gap_time_ms_min = 5; //5ms 
	int gap_counter_us= 0;
	int gap_counter_ms = 0;
	double gap_nvidia_s = 0;
	int gap_nvidia_ms = 0;
	int gap_nvidia_us = 0;

	int cur_lost_frame_num = 0;

	bool valid_frame_flag = false;
	bool recv_frame_flag = false;

	int frame_truth_num = 0;
	int frame_header_num = 0;
	int frame_invaild_num = 0;
	int frame_lost_num = 0;
	int frame_check_error_num = 0;

	double invalid_ratio = 0;
	double lost_ratio = 0;
	double check_error_ratio = 0;


	bool debug_flag = false;
	//bool debug_flag = true;	

	bool display_data_flag = true;
	//bool display_data_flag = false;

	// add lost data and mischeck data here==========end==========	

	std::chrono::time_point<std::chrono::system_clock> start_time;
	std::chrono::time_point<std::chrono::system_clock> end_time;
	double add_second;
	
	#pragma pack(1)
	//惯导输出信息
	struct DVL_Output_Info
	{  
		short int vel_btm[3];
		short int vel_ref[3];
		unsigned short int height[4];
		unsigned short int temp_dvl;
		unsigned long int timestamp;
	}DVL_Output_Data; 

	struct DVL_Output_Info_Nvidia
	{  
		DVL_Output_Info dvl_data;
		double timestamp_nvidia;
	}DVL_Output_Data_Nvidia;    

	struct DVL_FilterVel_Info
	{  
		short int dvl_vel[3];
		char dvl_flag;		
		unsigned long int timestamp;	
	};
	DVL_FilterVel_Info DVL_FilterVel_Data;
	DVL_FilterVel_Info old_DVL_FilterVel_Data;	
	#pragma pack()	

	int continue_valid_max_num = 100;
	int continue_valid_cur_num = 0;

	char dvldata_file_name[100];
	FILE * dvl_filestream;

	int dvl_invalid_data_num = 0;

	Analy_DVL_Data();
	~Analy_DVL_Data();
	void Init_Data(void);
	void Analy_DVL_Frame_( std::vector<char> & recv_Data,int  & Frame_Num,std::vector<DVL_Output_Info> & output_data);
	bool Analy_DVL_Frame( char Receive_Data,bool display_flag,bool filter_flag);
	void SimFilter_DVL_Vel(void);
	char CheckSum( char * nav_data, unsigned int data_len);
	void NavInfo_To_File(DVL_Output_Info * data);
	void NavInfo_To_File(DVL_Output_Info_Nvidia * data);

protected:




private:



};


#endif
