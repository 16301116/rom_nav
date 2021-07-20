#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h> 
#include <pthread.h>
#include <semaphore.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h> 
#include <string>
#include <ctime>
#include <time.h>
#include <chrono>
#include <thread>
#include <queue>
#include <math.h>
#include "lib/CoordTrans.hpp"
#include"udp/UDP_RecvData.h"
#include"sensor/Analy_Sins_Data.h"

using namespace std;

#define Middle_IP           "192.168.1.15" 
#define Middle_Port          5017
#define Nav_Port2            9001
#define Nav_Port3            8000

int main( char argc, char ** argv )
{
	bool sins_init_flag = false;
	char * dev_name = "/dev/ttyUSB0";
	int baudrate_sins = 460800;
	string parity = "None";
	Analy_Sins_Data sins_data(dev_name,baudrate_sins,parity,sins_init_flag);		
	while(1);
	return 0;
}



