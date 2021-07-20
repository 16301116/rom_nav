#include <stdlib.h>
#include <stdio.h>
#include <mutex>
#include "sensor/Analy_USBL_Data.h"

using namespace gdface;
using namespace chrono;
using namespace std;


Analy_USBL_Data::Analy_USBL_Data()
{
	Init_Data();
	// file name
	time_t currtime =time(NULL);
	tm *sins_p =localtime(&currtime);
	sprintf(data_file_name,"../data/output_data/usbl-data%04d%02d%02d%02d%02d%02d.txt",1900+sins_p->tm_year,1+sins_p->tm_mon,sins_p->tm_mday,sins_p->tm_hour,sins_p->tm_min,sins_p->tm_sec);	
	filestream = fopen(data_file_name,"w+");
	if (filestream==NULL)
  	{
		printf("can't create file to save USBL data!\n");
	}
	else
	{
		printf("successful create file to save USBL data!\n");
	}

}


Analy_USBL_Data::~Analy_USBL_Data()
{
		fclose(filestream);
}



void Analy_USBL_Data::Init_Data(void)
{
		memset(&USBL_Data_Nvidia,0,sizeof(USBL_Data_Nvidia));
		memset(&USBL_Data,0,sizeof(USBL_Data));
}


/*
 *函数功能：从串口接收并解析数据
 *输入参数：
 *输出参数：接收到一帧数据标志位sss
 *备    注：一次接收单个字符
 *
 */
bool Analy_USBL_Data::Analy_Frame( char Receive_Data,bool disp_flag)
{
	static char old_receive_data;
	char cur_receive_data;
	static int cur_datasum;
	static int cur_data_num;
	int perframe_datasum = sizeof(USBL_Data_Nvidia.usbl_data);

	static char perframe_data[Max_Perframe_Nums]; // 防止内存泄漏!!
	cur_receive_data = Receive_Data;

	valid_frame_flag = false;
	recv_frame_flag = false;

	switch (Data_State)
	{
		case header_state:
			{

				if ((unsigned char)old_receive_data == USBL_HEADER1 && (unsigned char)cur_receive_data ==USBL_HEADER2)
				{
					frame_header_num++;
					if(frame_header_num<=1)
					{
						start_time = system_clock::now();
						USBL_Data_Nvidia.timestamp_nvidia = 0.0;
					}
					else
					{
						end_time = system_clock::now();
						auto duration = duration_cast<microseconds>(end_time - start_time);
						USBL_Data_Nvidia.timestamp_nvidia = double(duration.count()) * microseconds::period::num / microseconds::period::den;
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
						memcpy(&USBL_Data_Nvidia.usbl_data,perframe_data,perframe_datasum);//把接收到的信息转换成结构体
						NavInfo_To_File(&USBL_Data_Nvidia);	
						if(disp_flag&&(frame_header_num%max_display_data_num==0))
						{
							printf("=========================USBL Data Begin====================\n");  
							printf("cur_time = %.6f\n",(float)USBL_Data_Nvidia.usbl_data.cur_time/1000000);  
							printf("measure_time = %.6f\n",(float)USBL_Data_Nvidia.usbl_data.measurement_time/1000000);  
							printf("address = %d\n",USBL_Data_Nvidia.usbl_data.address);  							
							printf("x_ship = %.6f\n",(float)USBL_Data_Nvidia.usbl_data.x_ship);  
							printf("y_ship = %.6f\n",(float)USBL_Data_Nvidia.usbl_data.y_ship);   
							printf("z_ship = %.6f\n",(float)USBL_Data_Nvidia.usbl_data.z_ship);
							printf("e_ship = %.6f\n",(float)USBL_Data_Nvidia.usbl_data.e_ship);  
							printf("n_ship = %.6f\n",(float)USBL_Data_Nvidia.usbl_data.n_ship);   
							printf("u_ship = %.6f\n",(float)USBL_Data_Nvidia.usbl_data.u_ship);
							printf("pitch = %.6f\n",(float)USBL_Data_Nvidia.usbl_data.pitch);  
							printf("roll = %.6f\n",(float)USBL_Data_Nvidia.usbl_data.roll);   
							printf("yaw = %.6f\n",(float)USBL_Data_Nvidia.usbl_data.yaw);
							printf("lat_gps = %.8lf\n",(double)USBL_Data_Nvidia.usbl_data.lat_gps);
							printf("lon_gps = %.8lf\n",(double)USBL_Data_Nvidia.usbl_data.lon_gps);							
							printf("propagation_time = %d\n",USBL_Data_Nvidia.usbl_data.propagation_time);
							printf("rssi = %.6f\n",(float)USBL_Data_Nvidia.usbl_data.rssi);
							printf("integrity = %.6f\n",(float)USBL_Data_Nvidia.usbl_data.integrity);
							printf("accuracy = %.6f\n",(float)USBL_Data_Nvidia.usbl_data.accuracy);
							printf("timestamp = %.6fS\n",(float)USBL_Data_Nvidia.usbl_data.timestamp/1000000); 
							printf("timestamp_nvidia = %.6fS\n",USBL_Data_Nvidia.timestamp_nvidia);  
							printf("=========================USBL Data End====================\n"); 

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




void Analy_USBL_Data::NavInfo_To_File(USBL_Info_Nvidia * data)
{
	static int fflush_num;
	fflush_num++;

	time_t timep;
	struct tm *p;
	time(&timep);
	p=gmtime(&timep);

	//output time
	fprintf(filestream,"%04d-",1900+p->tm_year);
	fprintf(filestream,"%02d-",1+p->tm_mon);
	fprintf(filestream,"%02d-",p->tm_mday);
	fprintf(filestream,"%02d-",p->tm_hour);
	fprintf(filestream,"%02d-",p->tm_min);
	fprintf(filestream,"%02d    ",p->tm_sec);

	fprintf(filestream,"%.6f ",(float)data->usbl_data.cur_time/1000000);  
	fprintf(filestream,"%.6f ",(float)data->usbl_data.measurement_time/1000000);  
	fprintf(filestream,"%d ",data->usbl_data.address);  
	fprintf(filestream,"%.6f ",(float)data->usbl_data.x_ship);  
	fprintf(filestream,"%.6f ",(float)data->usbl_data.y_ship);   
	fprintf(filestream,"%.6f ",(float)data->usbl_data.z_ship);
	fprintf(filestream,"%.6f ",(float)data->usbl_data.e_ship); 
	fprintf(filestream,"%.6f ",(float)data->usbl_data.n_ship);   
	fprintf(filestream,"%.6f ",(float)data->usbl_data.u_ship);
	fprintf(filestream,"%.6f ",(float)data->usbl_data.pitch);  
	fprintf(filestream,"%.6f ",(float)data->usbl_data.roll);   
	fprintf(filestream,"%.6f ",(float)data->usbl_data.yaw);
	fprintf(filestream,"%.8lf ",(double)data->usbl_data.lat_gps);
	fprintf(filestream,"%.8lf ",(double)data->usbl_data.lon_gps);
	fprintf(filestream,"%d ",data->usbl_data.propagation_time);
	fprintf(filestream,"%.6f ",(float)data->usbl_data.rssi);
	fprintf(filestream,"%.6f ",(float)data->usbl_data.integrity);
	fprintf(filestream,"%.6f ",(float)data->usbl_data.accuracy);
	fprintf(filestream,"%.6f ",(float)data->usbl_data.timestamp/1000000); 
	fprintf(filestream,"%.6f ",data->timestamp_nvidia);  
	fprintf(filestream,"\n");
	if(fflush_num==1)
	{
		fflush_num=0;
		fflush(filestream);
	}					

}




char Analy_USBL_Data::CheckSum( char * nav_data, unsigned int data_len)
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

