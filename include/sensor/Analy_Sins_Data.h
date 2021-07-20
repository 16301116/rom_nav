
#ifndef  H_Analy_Sins_Data_H
#define  H_Analy_Sins_Data_H

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
#include<ctime>
#include<chrono>
#include"serial/Serial_Data.h"
#include"lib/CoordTrans.hpp"
#include"lib/threadsafe_queue.h"



#define SINS_HEADER1  0x5A
#define SINS_HEADER2  0xA5
#define SINS_TAIL1    0x55
#define SANCHI_SINS_HEADER1  0x55
#define SANCHI_SINS_HEADER2  0xAA


//惯导地址信息
#define Header_ByNums          2
#define State_ByNums           1
#define Frame_ByNums           1
#define Counter_ByNums         1
#define CheckSum_ByNums        1
#define Tail_ByNums            1
#define Backup_ByNums          13

//组合导航工作模式
#define Max_Perframe_Nums      200

#define Sins_Bookbinding_CMD   0X03

#define PureInertia_Mode       0X01

#define Sins_AlignNavig_CMD    0X03
#define Sins_DVL_Mode          0X02

//轴数
#define Sins_Axis_Nums         3

// sins state
#define  FIND_NO_SINS_DEV      0X02

#define    SINS_DEV_ERROR          -1
#define    SINS_BOOK_ALIGN_NORM     1
#define    SINS_DVL_NORM            2


#define MAX_SEND_CONFDVLNUM    100
#define MAX_RECV_CONFDVLNUM    2000

typedef union FLONT_CONV
{
	float f;
	float c[4];

}float_conv;


class Analy_Sins_Data
{

public:
	float min_temp = -40;
	float max_temp = 65;
	float max_gyro_data = 300;// deg/s
	float max_acce_data = 98;  //m/s/s
	int max_display_data_num = 200;
	enum Analy_Frame_data_State
	{
		header_state = 1,
		data_num_state = 2,
		load_data_state = 3,
		check_state = 4,
	}analy_frame_data_state;

	int Sins_Data_State = header_state;

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
	int  old_counter;   // ms

	char struct_data_num = 32;
	char struct_data_num2 = 36+2;
	char sins_state = 0; // 2:navigation mode

	std::vector<char> receive_data;
	int max_receive_data = 2000;
	std::vector<char> struct_data_nums ;

	bool init_or_not = false;

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
	
	enum SINS_Data_State
	{
		standby_state = 0,
		init_state = 1,
		rough_land_state = 2,
		rough_air_state = 3,
		rigid_state = 4,
		ready_state = 5,
	};

	#pragma pack(1)	

	struct IMU_Data_State
	{
		char acce_x_exceed = 0;
		char acce_y_exceed = 1;
		char acce_z_exceed = 2;
		char gyro_x_exceed = 3;
		char gyro_y_exceed = 4;
		char gyro_z_exceed = 5;
		char temp_exceed = 12;
	}imu_state;

	//惯导输出信息
	struct Sins_Output_Info_Sanchi
	{  
		int  counter;  // 1ms
		char gps_invalid_time;
		char state;    //0,1,2,3,4,5
		char nav_mode; // always 2
		char sys_state;
		int angle[3]; // 0.000001deg/LSB
		int fusion_vel[3]; //0.001m/s/LSB
		int fusion_lon; //0.000001deg/LSB
		int fusion_lat; //0.000001deg/LSB
		int fusion_alt; //0.001m/LSB
		int gyro_data[3]; //0.001deg/h/LSB
		int acce_data[3]; //0.00001m/s/s/LSB
		short int gyro_tmp[3]; //0.0625/LSB
		short int acce_tmp[3]; //0.0625/LSB
		int gps_lon;  //0.00001deg/LSB
		int gps_lat;  //0.00001deg/LSB
		short int gps_alt;  //1m/LSB
		int gps_yaw_angle; //0.001deg/LSB
		short int gps_vel; //0.01m/s/LSB
		char gps_time[3];
		char gps_flag;    
		char gps_dop[6];                                             
		short int dvl_vel[3]; //0.001m/s/LSB
		char dvl_flag;
		char packup_char[13];
		//unsigned int timestamp;
		unsigned long int timestamp;
	}Sins_Output_Data_Sanchi; // frame_char_num = 130

	Sins_Output_Info_Sanchi Old_Sins_Output_Data_Sanchi;

	struct Sins_Output_Info_Nvidia
	{ 
		Sins_Output_Info_Sanchi sins_data;
		double timestamp_nvidia;
	}Sins_Output_Data_Nvidia; // frame_char_num = 130

	Sins_Output_Info_Nvidia  Old_Sins_Output_Data_Nvidia;


	struct  SINS_Init_CmdInfo   
	{
		char CMD;
		char FusionMode;
		int Longitude_Data;
		int Latitude_Data;
		short int Altitude_Data;
		short int ENU_VelData[Sins_Axis_Nums];
		char Satellite_Sign;
		char Backup_Data[13];  //备用数据
	}SINS_Init_Cmd;

	#pragma pack()

	Analy_Sins_Data();
	~Analy_Sins_Data();
	void Init_Data(void);
	Analy_Sins_Data(char * dev_name,int baudrate_sins,string parity,bool init_flag);
	static void SinsInit_Thread(Analy_Sins_Data * ptr);
	static void SinsReceCmd_Thread(Analy_Sins_Data * ptr);
	float BLEndianFloat(float value);
	char CheckSum( char * nav_data, unsigned int data_len);
	void Analy_Sins_Frame_Sanchi( std::vector<char> & recv_Data,int  & Frame_Num,std::vector<Sins_Output_Info_Sanchi> & output_data);
	bool Analy_Sins_Frame( char Receive_Data,int  & Frame_Num);
	bool Analy_Sins_Frame( char Receive_Data);
	bool Sins_Postprocess(char Receive_Data,bool disp_flag,bool add_check_lost_flag);
	void NavInfo_To_File(Sins_Output_Info_Sanchi * sins_output_info);
	void NavInfo_To_File(Sins_Output_Info_Sanchi * sins_output_info,bool valid_flag);
	void NavInfo_To_File(Sins_Output_Info_Nvidia * sins_output_info);
	void Nav_Error_Info_To_File(char * nav_data, unsigned int data_len,char data1,char data2);
	char Nav_Init(float Longitude,float Latitude,short int Altitude);
	int Encode_NavInit_Info(SINS_Init_CmdInfo Cmd,char * Nav_Init_FrameData);
	int Sins_Encode( char * nav_data, unsigned char data_len,char * nav_frame_data);

protected:


private:


}; 


#endif


