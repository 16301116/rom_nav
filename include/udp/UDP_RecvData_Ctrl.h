
#ifndef  __H__UDP_RECVDATA_CTRL__H__
#define  __H__UDP_RECVDATA_CTRL__H__

#include"udp/UDP_Communication.h"
#include"lib/threadsafe_queue.h"
#include<thread>
#include<mutex>

using namespace std;
using namespace gdface;	

class UDP_RecvData_Ctrl
{	
	public:
	
	//bool disp_flag = false;
	bool debug_flag = false;
	bool disp_flag = true;
	//bool debug_flag = true;	

	int nav_data_num = 0;

	UDP_Communication * udprecv;

	FILE * filestream;
	char file_name[100];

	#pragma pack(1)

	struct Nav_Output_Info
	{
		float lon;// 单位:deg 保留小数点后 6 位
		float lat; // 单位:deg 保留小数点后 6 位
		float alt; // 单位:m
		float x_ship; // 单位:m,船体坐标系
		float y_ship; // 单位:m,船体坐标系
		float z_ship; // 单位:m,船体坐标系
		float vel[3]; // 单位:m/s
		float angle[3];     // 单位:deg,范围:-180-+180,俯仰,滚转,偏航
		float gyro_data[3]; // 单位:deg/s
		float acce_data[3]; // 单位:m/s/s
		float timestamp ;   //单位:s ,保留小数点后 6 位
		char packup_char[13];
	}Nav_Output_Data;

	#pragma pack()

	threadsafe_queue<Nav_Output_Info> nav_que;           


	UDP_RecvData_Ctrl(int recv_port);
	static char Recv_Encoder_Data(UDP_RecvData_Ctrl *ptr);
	void Save_Nav_Data(Nav_Output_Info *data);

	protected:

	private:


}; 


#endif


