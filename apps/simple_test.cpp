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
//#include "DR/sins_dvl_DR.hpp"

using namespace std;
//using namespace chrono;
//using namespace gdface;	

int main( char argc, char ** argv )
{

	char data_file_name[100];
	FILE * filestream;

	time_t currtime =time(NULL);
	tm *sins_p =localtime(&currtime);
	sprintf(data_file_name,"../data/output_data/simple_test_data%04d%02d%02d%02d%02d%02d.txt",1900+sins_p->tm_year,1+sins_p->tm_mon,sins_p->tm_mday,sins_p->tm_hour,sins_p->tm_min,sins_p->tm_sec);	
	filestream = fopen(data_file_name,"w+");
	if (filestream==NULL)
  	{
		printf("can't create file to save test data!\n");
	}
	else
	{
		printf("successful create file to save test data!\n");
	}
	int num = 0;
	while(1)
	{
		fprintf(filestream,"==================hello world num = %d=========================\n",num);   
		fflush(filestream);				
		usleep(1000000);	 //1s
		num++;
	}
	return 0;
}



