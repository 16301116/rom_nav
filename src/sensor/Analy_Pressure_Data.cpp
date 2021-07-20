#include <stdlib.h>
#include <stdio.h>
#include <mutex>
#include "sensor/Analy_Pressure_Data.h"

using namespace gdface;
using namespace chrono;
using namespace std;


Analy_Pressure_Data::Analy_Pressure_Data()
{
	Init_Data();
	// file name
	time_t currtime =time(NULL);
	tm *sins_p =localtime(&currtime);
	sprintf(pressure_data_file_name,"../data/output_data/pressure-data%04d%02d%02d%02d%02d%02d.txt",1900+sins_p->tm_year,1+sins_p->tm_mon,sins_p->tm_mday,sins_p->tm_hour,sins_p->tm_min,sins_p->tm_sec);	
	pressure_filestream = fopen(pressure_data_file_name,"w+");
	if (pressure_filestream==NULL)
  	{
		printf("can't create file to save Pressure data!\n");
	}
	else
	{
		printf("successful create file to save Pressure data!\n");
	}

}


Analy_Pressure_Data::~Analy_Pressure_Data()
{
		fclose(pressure_filestream);
}


void Analy_Pressure_Data::Init_Data(void)
{
		memset(&Pressure_Sonnar_Data,0,sizeof(Pressure_Sonnar_Data));
		memset(&Pressure_Data_Nvidia,0,sizeof(Pressure_Data_Nvidia));
}


/*
 *函数功能：从串口接收并解析数据
 *输入参数：
 *输出参数：接收到一帧数据标志位sss
 *备    注：一次接收单个字符
 *
 */
bool Analy_Pressure_Data::Analy_Frame( char Receive_Data,bool disp_flag)
{
	static char old_receive_data;
	char cur_receive_data;
	static int cur_datasum;
	static int cur_data_num;
	int perframe_datasum = sizeof(Pressure_Data_Nvidia.pressure_data);

	static char perframe_data[Max_Perframe_Nums]; // 防止内存泄漏!!
	cur_receive_data = Receive_Data;

	valid_frame_flag = false;
	recv_frame_flag = false;

	switch (Data_State)
	{
		case header_state:
			{

				if ((unsigned char)old_receive_data == PRESSURE_HEADER1 && (unsigned char)cur_receive_data ==PRESSURE_HEADER2)
				{
					frame_header_num++;
					if(frame_header_num<=1)
					{
						start_time = system_clock::now();
						Pressure_Data_Nvidia.timestamp_nvidia = 0.0;
					}
					else
					{
						end_time = system_clock::now();
						auto duration = duration_cast<microseconds>(end_time - start_time);
						Pressure_Data_Nvidia.timestamp_nvidia = double(duration.count()) * microseconds::period::num / microseconds::period::den;
					}		
					cur_datasum = Header_ByNums;
					Data_State = load_data_state;
				}
				break;
			};

		case load_data_state:
			{
				cur_datasum++;
				perframe_data[cur_data_num++] = cur_receive_data;
				if (cur_datasum == ( Header_ByNums+ perframe_datasum)) 
				{
					Data_State = check_state;
				}
				break;
			};

			//case SINS_CheckSum:
		case check_state:
			{
				cur_datasum++;	
				if (cur_datasum == ( Header_ByNums + perframe_datasum + CheckSum_ByNums))
				{	
					//sum_check
					recv_frame_flag = true;
					char check_data = CheckSum(perframe_data, cur_data_num);
					if(debug_flag)
					{
					//	printf("check_data = %d cur_receive_data = %d\n",check_data,cur_receive_data);
					}
					if(check_data ==cur_receive_data)
					{	
						valid_frame_flag = true;
						frame_invaild_num++;
						memcpy(&Pressure_Data_Nvidia.pressure_data,perframe_data,perframe_datasum);//把接收到的信息转换成结构体	
						NavInfo_To_File(& Pressure_Data_Nvidia);
						if(disp_flag&&(frame_header_num%max_display_data_num==0))
						{
							printf("=========================Pressure Data Begin====================\n");  
							printf("Pressure = %.6f\n",(float)Pressure_Data_Nvidia.pressure_data.data);  
							printf("temp = %.6f\n",(float)Pressure_Data_Nvidia.pressure_data.temp);  
							printf("timestamp = %.6fS\n",(float)Pressure_Data_Nvidia.pressure_data.timestamp/1000000); 
							printf("timestamp_nvidia = %.6fS\n",Pressure_Data_Nvidia.timestamp_nvidia);  
							printf("=========================Pressure Data End====================\n"); 

						}				
					}
					else
					{

					}		

					cur_datasum = 0;
					cur_data_num= 0;
					memset(perframe_data, 0, Max_Perframe_Nums);
					Data_State = header_state;
				}  

				break;
			};

		default:
			{
				cur_datasum = 0;
				cur_data_num= 0;
				memset(perframe_data, 0, Max_Perframe_Nums);
				Data_State = header_state;
				break;
			};

	}

	old_receive_data = cur_receive_data;
	return recv_frame_flag;

}




void Analy_Pressure_Data::NavInfo_To_File(Pressure_Info_Nvidia * data)
{
	static int fflush_num;
	fflush_num++;

	time_t timep;
	struct tm *p;
	time(&timep);
	p=gmtime(&timep);

	//output time
	fprintf(pressure_filestream,"%04d-",1900+p->tm_year);
	fprintf(pressure_filestream,"%02d-",1+p->tm_mon);
	fprintf(pressure_filestream,"%02d-",p->tm_mday);
	fprintf(pressure_filestream,"%02d-",p->tm_hour);
	fprintf(pressure_filestream,"%02d-",p->tm_min);
	fprintf(pressure_filestream,"%02d    ",p->tm_sec);
	fprintf(pressure_filestream,"%.6f ",(float)data->pressure_data.data); 
	fprintf(pressure_filestream,"%.6f ",(float)data->pressure_data.temp); 
	fprintf(pressure_filestream,"%.6f ",(float)data->pressure_data.timestamp/1000000); 
	fprintf(pressure_filestream,"%.6f ",data->timestamp_nvidia);  

	fprintf(pressure_filestream,"\n");
	if(fflush_num==10)
	{
		fflush_num=0;
		fflush(pressure_filestream);
	}					

}




char Analy_Pressure_Data::CheckSum( char * nav_data, unsigned int data_len)
{
	int data_sum =0;
	char check_data;	

	for(int i=0;i<data_len;i++)
	{
		data_sum+=nav_data[i];	
		//printf("\ndata_sum = %d\n",data_sum);
	}

	check_data = data_sum&0xFF;

	return (check_data);
}

