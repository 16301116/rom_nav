
#ifndef  H_Analy_MEMS_IMU_Data_H
#define  H_Analy_MEMS_IMU_Data_H

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

#define MTI_G_710_HEADER1  0xFE // attention here MAVLINK HEAD = FE
#define MTI_G_710_HEADER2  0xFA

//惯导地址信息
#define Header_ByNums          2
#define CheckSum_ByNums        1

//组合导航工作模式
#define Max_Perframe_Nums      200

//轴数
#define Sins_Axis_Nums         3


#define MAX_SEND_CONFDVLNUM    100
#define MAX_RECV_CONFDVLNUM    2000


class Analy_MEMS_IMU_Data
{

public:

	int max_display_data_num = 200;
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
	double old_add_second;
	unsigned long int old_timestamp;  // us

	char sins_state = 0; // 2:navigation mode

	std::vector<char> receive_data;
	int max_receive_data = 2000;
	std::vector<char> struct_data_nums ;


	bool down_gps_flag;
	bool navinitover_flag;
	
	char nav_init_flag;	
	int test_queue_data;

	char sinsdata_file_name[100];
	FILE * sins_filestream;

	char error_sinsdata_file_name[100];
	FILE * error_sins_filestream;

	char bad_sinsdata_file_name[100];
	FILE * bad_sins_filestream;

	std::mutex sins_mtx;
	int m_fd_sins;
	
	Serial_Data serial_sins_data;

	bool Find_Sins_Dev;
	bool Sins_Frame_RdyFlag;
    bool Sins_navState_flag;
	
	#pragma pack(1)	
	//惯导输出信息
	struct MEMS_IMU_Output_Info
	{  
		float angle[3];     // 0.01deg
		float gyro_data[3]; // 0.01deg/s
		float acce_data[3]; // 0.01m/s/s
		float temp;
		//unsigned int time_stmap;
		unsigned long int timestamp;
	}MEMS_IMU_Output_Data; // frame_char_num = 130

	struct Mems_IMU_Output_Info_Nvidia
	{  
		MEMS_IMU_Output_Info sins_data;
		double timestamp_nvidia;
	}Mems_IMU_Output_Data_Nvidia;   

	Mems_IMU_Output_Info_Nvidia  Old_Mems_IMU_Output_Data_Nvidia;

	#pragma pack()

	Analy_MEMS_IMU_Data();
	~Analy_MEMS_IMU_Data();
	void Init_Data(void);
	void Analy_Sins_Frame( std::vector<char> & recv_Data,int  & Frame_Num,std::vector<MEMS_IMU_Output_Info> & output_data);
	bool Analy_Sins_Frame( char Receive_Data,int  & Frame_Num);
	bool Analy_Sins_Frame( char Receive_Data);
	bool Sins_Postprocess(char Receive_Data,bool disp_flag,bool add_check_lost_flag);
	void NavInfo_To_File(MEMS_IMU_Output_Info * sins_output_info);
	void NavInfo_To_File(MEMS_IMU_Output_Info * sins_output_info,bool valid_flag);
	void NavInfo_To_File(Mems_IMU_Output_Info_Nvidia * sins_output_info);
	void Nav_Error_Info_To_File(char * nav_data, unsigned int data_len,char data1,char data2);
	char CheckSum( char * nav_data, unsigned int data_len);

protected:


private:


}; 


#endif


