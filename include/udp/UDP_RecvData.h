#ifndef  __H__UDPRECVDATA__H__
#define  __H__UDPRECVDATA__H__

#include<iostream>
#include<string.h>
#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h> 
#include<unistd.h> 
#include<time.h>
#include<pthread.h>
#include<netinet/in.h>
#include<math.h>
#include<mutex>
#include<chrono>
#include<ctime>
#include<thread>
#include"udp/UDP_Communication.h"
#include"lib/threadsafe_queue.h"
#include"sensor/Analy_MEMS_IMU_Data.h"
#include"sensor/Analy_Sins_Data.h"
#include"sensor/Analy_GPS_Data.h"
#include"sensor/Analy_DVL_Data.h"
#include"sensor/Analy_CTD_Data.h"
#include"sensor/Analy_Pressure_Data.h"
#include"sensor/Analy_Depth_Data.h"
#include"sensor/Analy_USBL_Data.h"

using namespace std;
using namespace gdface;	
using namespace chrono;

#define UDP_RECV_MAX_NUM 1000

class UDP_RecvData
{
	
	public:

	bool udp_nav_mode_flag = false;
	char nav_mode;
	std::mutex mtx_nav_mode; 
	
	char Recv_Buf[UDP_RECV_MAX_NUM] = {0};
	int Recv_Num = 0;

	bool  save_udp_file_flag = false;

	bool debug_flag = false;
	//bool debug_flag = true;	

	bool  disp_all_flag = true;
	bool disp_sins_flag = true;
	bool disp_mems_flag = true;
	bool disp_gps_flag = true;
	bool disp_dvl_flag = true;
	bool disp_depth_flag = true;
	bool disp_pressure_flag = true;
	bool disp_ctd_flag = true;
	bool disp_usbl_flag = true;

	bool sins_frame_flag = false;
	bool mems_sins_frame_flag = false;
	bool gps_frame_flag = false;
	bool dvl_frame_flag = false;
	bool ctd_frame_flag = false;
	bool depth_frame_flag = false;
	bool pressure_frame_flag = false;
	bool usbl_frame_flag = false;

	bool dvl_filter_flag = true;

	const int test_data = 100;
	std::chrono::time_point<std::chrono::system_clock> start_time;
	std::chrono::time_point<std::chrono::system_clock> end_time;
	double add_second;

	char udp_file_name[100];
	FILE * udp_filestream;

	 Analy_Sins_Data get_sins_data;
 	 Analy_MEMS_IMU_Data get_mti_data; 
	 Analy_GPS_Data get_gps_data;
	 Analy_DVL_Data get_dvl_data;
	 Analy_CTD_Data get_ctd_data;
	 Analy_Depth_Data get_depth_data;
	 Analy_Pressure_Data get_pressure_data;
	 Analy_USBL_Data get_usbl_data;

	int max_fog_sins_que = 2;
	int max_mems_sins_que = 2;
	int max_gps_que = 2;
	int max_dvl_que = 2;
	int max_ctd_que = 2;
	int max_depth_que = 2;
	int max_pressure_que = 2;
	int max_usbl_que = 2;

	threadsafe_queue<Analy_Sins_Data::Sins_Output_Info_Nvidia> fog_sins_que;  
	threadsafe_queue<Analy_MEMS_IMU_Data::Mems_IMU_Output_Info_Nvidia> mems_sins_que;  
	threadsafe_queue<Analy_GPS_Data::GNSS_FRAME_Info> gps_que;
	threadsafe_queue<Analy_DVL_Data::DVL_Output_Info_Nvidia> dvl_que;
	threadsafe_queue<Analy_CTD_Data::CTD_Info_Nvidia> ctd_que;  
	threadsafe_queue<Analy_Depth_Data::Depth_Info_Nvidia> depth_que;  
	threadsafe_queue<Analy_Pressure_Data::Pressure_Info_Nvidia> pressure_que;
	threadsafe_queue<Analy_USBL_Data::USBL_Info_Nvidia> usbl_que;

	threadsafe_queue<Analy_DVL_Data::DVL_FilterVel_Info> dvl_filter_que;

	#pragma pack(1)

	#pragma pack()

	std::mutex mtx_nav_info_mode;  
	std::mutex mtx_planning_info;  
	std::mutex mtx_send_ranger;  
	
	UDP_Communication * udprecv;
	UDP_Communication::UDP_Info_ROV * UDP_FrameData;
	
	UDP_RecvData(int recv_port);
	UDP_RecvData(std::string ip,int send_port ,int recv_port);
	~UDP_RecvData();
	static char Recv_Encoder_Data(UDP_RecvData *ptr);
	void NavInfo_To_File(float a1,float a2,float a3,float a4,char b1,char b2);
	void MAVLINK_Recv_Encoder_Data(UDP_Communication::UDP_Info_ROV * UDP_FrameData);

	protected:


	private:


}; 


#endif


