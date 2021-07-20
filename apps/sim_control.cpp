#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h> 
#include<pthread.h>
#include<semaphore.h>
#include<stdlib.h>
#include<stdio.h>
#include<unistd.h> 
#include<string>
#include<ctime>
#include<time.h>
#include<chrono>
#include<thread>
#include<queue>
#include<math.h>
#include"udp/UDP_RecvData.h"
#include"lib/CoordTrans.hpp"
#include"sensor/Analy_Sins_Data.h"
#include"nav/Planner.h"
#include"udp/UDP_RecvData_Ctrl.h"


using namespace std;
//using namespace chrono;
//using namespace gdface;	



int main( char argc, char ** argv )
{
	
	UDP_RecvData_Ctrl udp_recedata_ctrl(Controller_Port);
	//UDP_RecvData_Ctrl udp_recedata_ctrl(Nav_Nvidia_Port);
	while(1);
	return 0;
}



