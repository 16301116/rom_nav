#ifndef  H_Planner_H
#define  H_Planner_H


#include "lib/threadsafe_queue.h"
#include "lib/eigen_types.h"
#include "lib/CoordTrans.hpp"
#include "lib/earth_para.hpp"

#include "udp/UDP_Communication.h"
#include "udp/UDP_RecvData.h"
#include "udp/UDP_RecvData.h"

#include "nav/sins_dvl_DR.hpp"

using namespace gdface;	

#define VEL_NUM                   10
#define SONAR_NUM                 8
#define TRAJECTORY_VALID          1
#define TRAJECTORY_INVALID        0
#define MAX_TRAJECTORY_NUM        1000

#define MIN_LINVEL_INDEX          0
#define MAX_LINVEL_INDEX          1
#define MIN_ANGVEL_INDEX          2
#define MAX_ANGVEL_INDEX          3
#define VELWINDOW_NUM             4

#define MAX_SIMNUM                5000
#define filter1_num  6
#define filter2_num  4
#define sonnar_num  3


class Planner
{
public:

	bool first_gps_sins_dvl_mode_flag = false; //在sins/dvl模式下的初始位置(GPS)
	char udp_set_nav_mode = 0;
	int recv_fre = 100; // 100HZ
	int send_ctrl_fre = 50; // 50hz
	int cur_recv_counter = 0;

	UDP_RecvData * udp_recvdata;
	UDP_Communication * udp2ctrol;
	Analy_Sins_Data * aly_sins_data;
	Analy_DVL_Data * aly_dvl_data;
	Analy_GPS_Data * aly_gps_data;
	Analy_USBL_Data * aly_usbl_data;
	sins_dvl_DR *  sins_dvl_dr;

	sins_dvl_DR::Sins_DVl_Gps_USBL_Data real_dr_data;

	bool debug_flag = false;

	std::mutex Nav2CtrlData_mtx;	
	std::mutex mtx; 

	FILE * location_filestream;
	char location_file_name[100];

	FILE * nav2ctrl_filestream;
	char nav2ctrl_file_name[100];

	#pragma pack(1)	

	struct Ref_Pos
	{
		char no_pos = 16;
		char set_pos = 17;
		char gps_pos = 18;
		char usbl_pos = 19;
	};

	struct Ref_Vel
	{
		char no_vel = 20;
		char zero_vel = 21;
		char gps_vel = 22;
		char dvl_vel = 23;
	};

	struct Ref_Alt
	{
		char no_alt = 24;
		char pressure_alt = 25;
		char gps_alt = 26;
		char usbl_alt = 27;
	};

	struct Navigation_Data_State
	{
		char standby_state = 0;
		char init_state = 1;
		char rough_land_state = 2;
		char rough_air_state = 3;
		char rigid_state = 4;
		char pure_sins_state = 5;
		char sins_dvl_state = 6;
		char sins_gps_state = 7;
		char sins_usbl_state = 8;
		char sins_dvl_gps_state = 9;
		char sins_dvl_usbl_state = 10;
		char sins_dvl_usbl_gps_state = 11;
		Ref_Pos ref_pos_state;
		Ref_Vel ref_vel_state;
		Ref_Alt ref_alt_state;
	}navigation_state;

	struct FrameData
	{
		Analy_Sins_Data::Sins_Output_Info_Nvidia  sinsframedata;
		Analy_MEMS_IMU_Data::Mems_IMU_Output_Info_Nvidia mems_sinsframedata;
		Analy_GPS_Data::GNSS_FRAME_Info gpsframedata;
		Analy_DVL_Data::DVL_FilterVel_Info dvl_framedata;
		Analy_CTD_Data::CTD_Info_Nvidia ctd_framedata;
		Analy_Depth_Data::Depth_Info_Nvidia depth_framedata;
		Analy_Pressure_Data::Pressure_Info_Nvidia pressure_framedata;
		Analy_USBL_Data::USBL_Info_Nvidia usbl_framedata;
	}framedata;
	
	Analy_GPS_Data::GNSS_FRAME_Info buf_gpsframedata;
	
	struct Sensor_Ready_Flag
	{
		bool sins_ready_flag = false;
		bool mems_sins_ready_flag = false;
		bool gps_ready_flag = false;
		bool dvl_ready_flag = false;
		bool ctd_ready_flag = false;
		bool depth_ready_flag = false;
		bool pressure_ready_flag = false;
		bool usbl_ready_flag = false;
	}Ready_Flag;

	struct Nav_Output_Info
	{
		double lon;// 单位:deg 保留小数点后 6 位
		double lat; // 单位:deg 保留小数点后 6 位
		float alt; // 单位:m

		float x_ship; // 单位:m,船体坐标系
		float y_ship; // 单位:m,船体坐标系
		float z_ship; // 单位:m,船体坐标系
		float vel[3]; // 单位:m/s
		float angle[3];     // 单位:deg,范围:-180-+180,俯仰,滚转,偏航(north is zero)
		float gyro_data[3]; // 单位:deg/h
		float acce_data[3]; // 单位:m/s/s
		unsigned char imu_state[2];
		unsigned char dvl_state;
		unsigned char gps_state;
		unsigned char usbl_state;
		unsigned char nav_state[4];
		unsigned char packup[4];
		float timestamp ;   //单位:s ,保留小数点后 6 位
	}Nav_Output_Data;

	struct Sensor_State
	{
		char sins_state;
		char mems_sins_state;
		char dvl_state;
		char usbl_state;
		char gps_state;
		char ctd_state;
		char depth_state;
		char pressure_state;
		char nav_state;
	}Mul_Sensor_State;

	struct Continuous_Error_Num
	{
		char sins_num;
		char mems_sins_num;
		char dvl_num;
		char usbl_num;
		char gps_num;
		char ctd_num;
		char depth_num;
		char pressure_num;
	}Sensor_Conti_Error_DataNum;

	struct Max_Continuous_Error_Num
	{
		int sins_num = 100;  //3 second
		int mems_sins_num = 100;//3 second
		int dvl_num = 300; //3 second
		int usbl_num = 500; //5 second
		int gps_num = 200; //3 second
		int ctd_num = 100;//3 second
		int depth_num = 100; //3 second
		int pressure_num = 100; //3 second
	}Max_Conti_Error_Num;	


	struct Cur_Data_Num
	{
		int sins_num ;  
		int mems_sins_num ;
		int dvl_num ; 
		int usbl_num ;
		int gps_num ; 
		int ctd_num ;
		int depth_num ; 
		int pressure_num ; 
	}Cur_Recv_Num ;	
	#pragma pack()	

    CoordTrans  CoordTrans_Tools;

	Planner();
	Planner(bool udp_test_flag);
	~Planner();

	Planner(threadsafe_queue<Analy_Sins_Data::Sins_Output_Info_Nvidia> & fog_sins_que,\
	threadsafe_queue<Analy_MEMS_IMU_Data::Mems_IMU_Output_Info_Nvidia> & mems_sins_que,\
	threadsafe_queue<Analy_GPS_Data::GNSS_FRAME_Info> & gps_que,\
	threadsafe_queue<Analy_DVL_Data::DVL_FilterVel_Info> & dvl_que,\
	threadsafe_queue<Analy_CTD_Data::CTD_Info_Nvidia> & ctd_que,\
	threadsafe_queue<Analy_Depth_Data::Depth_Info_Nvidia> & depth_que,\
	threadsafe_queue<Analy_Pressure_Data::Pressure_Info_Nvidia> & pressure_que,\
	threadsafe_queue<Analy_USBL_Data::USBL_Info_Nvidia> & usbl_que);

	void Init_Data(void);

	static void run(Planner *ptr,threadsafe_queue<Analy_Sins_Data::Sins_Output_Info_Nvidia> & sins_que,\
	threadsafe_queue<Analy_MEMS_IMU_Data::Mems_IMU_Output_Info_Nvidia> & mems_sins_que,\
	threadsafe_queue<Analy_GPS_Data::GNSS_FRAME_Info> & gpsque,\
	threadsafe_queue<Analy_DVL_Data::DVL_FilterVel_Info> & dvl_que,\
	threadsafe_queue<Analy_CTD_Data::CTD_Info_Nvidia> & ctd_que,\
	threadsafe_queue<Analy_Depth_Data::Depth_Info_Nvidia> & depth_que,\
	threadsafe_queue<Analy_Pressure_Data::Pressure_Info_Nvidia> & pressure_que,\
	threadsafe_queue<Analy_USBL_Data::USBL_Info_Nvidia> & usbl_que);

	static void run_test(Planner *ptr);

	void Get_UDP_Data(UDP_RecvData * data);
	void Get_Nav_Data(threadsafe_queue<Analy_Sins_Data::Sins_Output_Info_Nvidia> & nav_que);
	void Get_Mems_SINS_Data(threadsafe_queue<Analy_MEMS_IMU_Data::Mems_IMU_Output_Info_Nvidia> & mems_que);
	void Get_GPS_Data(threadsafe_queue<Analy_GPS_Data::GNSS_FRAME_Info> & gps_que);
	void Get_DVL_Data(threadsafe_queue<Analy_DVL_Data::DVL_FilterVel_Info> & dvl_que);
	void Get_CTD_Data(threadsafe_queue<Analy_CTD_Data::CTD_Info_Nvidia> & ctd_que);
	void Get_Pressure_Data(threadsafe_queue<Analy_Pressure_Data::Pressure_Info_Nvidia> & pressure_que);
	void Get_Depth_Data(threadsafe_queue<Analy_Depth_Data::Depth_Info_Nvidia> & depth_que);
	void Get_USBL_Data(threadsafe_queue<Analy_USBL_Data::USBL_Info_Nvidia> & usbl_que);

	void Get_Nav_State(bool flag);
	void Get_Mems_SINS_State(bool flag);
	void Get_GPS_State(bool flag);
	void Get_DVL_State(bool flag);
	void Get_CTD_State(bool flag);
	void Get_Pressure_State(bool flag);
	void Get_Depth_State(bool flag);
	void Get_USBL_State(bool flag);

	void Init_DR_data(sins_dvl_DR::Sins_DVl_Gps_USBL_Data & nav_data);
	void Init_Fusion_data(const char & nav_mode,sins_dvl_DR::Sins_DVl_Gps_USBL_Data & nav_data);
	void Cal_Nav_Mode(const char &  set_nav_mode);
	void Save_Nav2Ctrl_Data(void);
	void Save_NAV_Data(void);

	static long int Get_Cur_Second(void);
	void Nav2Ctrl_UDP(void);
	void Test_Nav2Ctrl_UDP(void);
	void Test2_Nav2Ctrl_UDP(void);

	void Assembly(void);
	void Get_IMU_State(void);

protected:


private:

}; 


#endif


