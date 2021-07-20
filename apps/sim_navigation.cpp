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
#include"udp/UDP_RecvData.h"
#include "lib/CoordTrans.hpp"
#include"sensor/Analy_Sins_Data.h"
#include "nav/Planner.h"

using namespace std;
//using namespace chrono;
//using namespace gdface;	



int main( char argc, char ** argv )
{
	bool udp_test_flag = true;
	//Planner planer(udp_test_flag);
	while(1);
	return 0;
}



