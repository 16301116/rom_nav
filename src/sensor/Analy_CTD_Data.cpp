#include <stdlib.h>
#include <stdio.h>
#include <mutex>
#include "sensor/Analy_CTD_Data.h"

using namespace gdface;
using namespace chrono;
using namespace std;


Analy_CTD_Data::Analy_CTD_Data()
{
	Init_Data();
	// file name
	time_t currtime =time(NULL);
	tm *sins_p =localtime(&currtime);
	sprintf(ctddata_file_name,"../data/output_data/ctd-data%04d%02d%02d%02d%02d%02d.txt",1900+sins_p->tm_year,1+sins_p->tm_mon,sins_p->tm_mday,sins_p->tm_hour,sins_p->tm_min,sins_p->tm_sec);	
	ctd_filestream = fopen(ctddata_file_name,"w+");
	if (ctd_filestream==NULL)
  	{
		printf("can't create file to save CTD data!\n");
	}
	else
	{
		printf("successful create file to save CTD data!\n");
	}

}


Analy_CTD_Data::~Analy_CTD_Data()
{
		fclose(ctd_filestream);
}



void Analy_CTD_Data::Init_Data(void)
{
		memset(&CTD_Data,0,sizeof(CTD_Data));
		memset(&CTD_Data_Nvidia,0,sizeof(CTD_Data_Nvidia));
}


/*
 *函数功能：从串口接收并解析数据
 *输入参数：
 *输出参数：接收到一帧数据标志位sss
 *备    注：一次接收单个字符
 *
 */
bool Analy_CTD_Data::Analy_Frame( char Receive_Data,bool disp_flag)
{
	static char old_receive_data;
	char cur_receive_data;
	static int cur_datasum;
	static int cur_data_num;
	int perframe_datasum = sizeof(CTD_Data_Nvidia.ctd_data);

	static char perframe_data[Max_Perframe_Nums]; // 防止内存泄漏!!
	cur_receive_data = Receive_Data;

	valid_frame_flag = false;
	recv_frame_flag = false;

	switch (Data_State)
	{
		case header_state:
			{

				if ((unsigned char)old_receive_data == CTD_HEADER1 && (unsigned char)cur_receive_data ==CTD_HEADER2)
				{
					frame_header_num++;
					if(frame_header_num<=1)
					{
						start_time = system_clock::now();
						CTD_Data_Nvidia.timestamp_nvidia = 0.0;
					}
					else
					{
						end_time = system_clock::now();
						auto duration = duration_cast<microseconds>(end_time - start_time);
						CTD_Data_Nvidia.timestamp_nvidia = double(duration.count()) * microseconds::period::num / microseconds::period::den;
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
						memcpy(&CTD_Data_Nvidia.ctd_data,perframe_data,perframe_datasum);//把接收到的信息转换成结构体	
						NavInfo_To_File(&CTD_Data_Nvidia);
						if(disp_flag&&(frame_header_num%max_display_data_num==0))
						{
							printf("=========================CTD Data Begin====================\n");  
							printf("C = %.6f\n",(float)CTD_Data_Nvidia.ctd_data.c);  
							printf("T = %.6f\n",(float)CTD_Data_Nvidia.ctd_data.t);   
							printf("D = %.6f\n",(float)CTD_Data_Nvidia.ctd_data.d); 
							printf("timestamp = %.6fS\n",(float)CTD_Data_Nvidia.ctd_data.timestamp/1000000); 
							printf("timestamp_nvidia = %.6fS\n",CTD_Data_Nvidia.timestamp_nvidia);  
							printf("=========================CTD Data End====================\n"); 

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




void Analy_CTD_Data::NavInfo_To_File(CTD_Info_Nvidia * data)
{
	static int fflush_num;
	fflush_num++;

	time_t timep;
	struct tm *p;
	time(&timep);
	p=gmtime(&timep);

	//output time
	fprintf(ctd_filestream,"%04d-",1900+p->tm_year);
	fprintf(ctd_filestream,"%02d-",1+p->tm_mon);
	fprintf(ctd_filestream,"%02d-",p->tm_mday);
	fprintf(ctd_filestream,"%02d-",p->tm_hour);
	fprintf(ctd_filestream,"%02d-",p->tm_min);
	fprintf(ctd_filestream,"%02d    ",p->tm_sec);
	fprintf(ctd_filestream,"%.6f ",(float)data->ctd_data.c);  
	fprintf(ctd_filestream,"%.6f ",(float)data->ctd_data.t);   
	fprintf(ctd_filestream,"%.6f ",(float)data->ctd_data.d); 
	fprintf(ctd_filestream,"%.6f ",(float)data->ctd_data.timestamp/1000000); 
	fprintf(ctd_filestream,"%.6f ",data->timestamp_nvidia);  
	fprintf(ctd_filestream,"\n");
	if(fflush_num==8)
	{
		fflush_num=0;
		fflush(ctd_filestream);
	}					

}




char Analy_CTD_Data::CheckSum( char * nav_data, unsigned int data_len)
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

