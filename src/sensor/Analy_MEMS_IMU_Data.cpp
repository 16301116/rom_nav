#include <stdlib.h>
#include <stdio.h>
#include <mutex>
#include "sensor/Analy_MEMS_IMU_Data.h"

using namespace gdface;
using namespace chrono;
using namespace std;


Analy_MEMS_IMU_Data::Analy_MEMS_IMU_Data()
{
	Init_Data();
	// file name
	time_t currtime =time(NULL);
	tm *sins_p =localtime(&currtime);
	sprintf(sinsdata_file_name,"../data/output_data/mems-data%04d%02d%02d%02d%02d%02d.txt",1900+sins_p->tm_year,1+sins_p->tm_mon,sins_p->tm_mday,sins_p->tm_hour,sins_p->tm_min,sins_p->tm_sec);	
	sins_filestream = fopen(sinsdata_file_name,"w+");
	if (sins_filestream==NULL)
  {
		printf("can't create file to save mems sins data!\n");
	}
	else
	{
		printf("successful create file to save mems sins data!\n");
	}

/*
	sprintf(error_sinsdata_file_name,"../data/output_data/error-mems-data%04d%02d%02d%02d%02d%02d.txt",1900+sins_p->tm_year,1+sins_p->tm_mon,sins_p->tm_mday,sins_p->tm_hour,sins_p->tm_min,sins_p->tm_sec);	
	error_sins_filestream = fopen(error_sinsdata_file_name,"w+");
	if (error_sins_filestream==NULL)
  {
		printf("can't create file to save error mems sins data!\n");
	}
	else
	{
		printf("successful create file to save error mems sins data!\n");
	}
*/

}


Analy_MEMS_IMU_Data::~Analy_MEMS_IMU_Data()
{
		fclose(sins_filestream);
}




void Analy_MEMS_IMU_Data::Init_Data(void)
{
		memset(&MEMS_IMU_Output_Data,0,sizeof(MEMS_IMU_Output_Data));
		memset(&Mems_IMU_Output_Data_Nvidia,0,sizeof(Mems_IMU_Output_Data_Nvidia));
		memset(&Old_Mems_IMU_Output_Data_Nvidia,0,sizeof(Old_Mems_IMU_Output_Data_Nvidia));
}


/*
 *函数功能：从串口接收并解析数据
 *输入参数：
 *输出参数：接收到一帧数据标志位
 *备    注：一次接收单个字符
 *
 * cur_pos : initial data = 0 or 1
 */
void Analy_MEMS_IMU_Data::Analy_Sins_Frame( std::vector<char> & recv_Data,int  & Frame_Num,std::vector<MEMS_IMU_Output_Info> & output_data)
{
//	printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~come in Analy_Sins_Frame_Sanchi~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
	char counter_num = 0;
	Frame_Num = 0;
	static int bad_frame_data_num = 0;
	static int good_frame_dat_num = 0;
	static int sum_frame_dat_num = 0;
	int frame_char_num = sizeof(MEMS_IMU_Output_Info);
	char perframe_data[Max_Perframe_Nums] = {0}; //防止内存泄漏!!
	if(recv_Data.size()<(Header_ByNums + frame_char_num )) // 52:change here ==========one frame data num :38=============//
	{
		return;
	}
	int max_data_num = recv_Data.size();
	int cur_pos = 0;
	bool over_flag = false;

	while(cur_pos<recv_Data.size())   //attention
 	{
		if(recv_Data[cur_pos]==MTI_G_710_HEADER1&&recv_Data[cur_pos+1]==MTI_G_710_HEADER2) 
		{
			if(cur_pos!=0)
			{
				recv_Data.erase(recv_Data.begin(),recv_Data.begin()+cur_pos);		
			}
			cur_pos = 0; // reset cur_pos 	
			if(recv_Data.size()>=(frame_char_num + Header_ByNums)) // 2+1 = 3
			{
				//analyze one frame 
				recv_Data.erase(recv_Data.begin(),recv_Data.begin() + Header_ByNums);
				for(int i=0;i<(int)frame_char_num;i++)
				{
					perframe_data[i] = recv_Data.front();
					recv_Data.erase(recv_Data.begin());	
				}
				memcpy(&MEMS_IMU_Output_Data,perframe_data,frame_char_num);
				char check_data = CheckSum(perframe_data, frame_char_num);
				//char check_data = 0x01;
				char receive_checksum = recv_Data.front();
				recv_Data.erase(recv_Data.begin());	
				cur_pos = 0; // reset cur_pos 
				sum_frame_dat_num++;
				if(debug_flag)
				{
					printf("\nframe_char_num = %d\n",frame_char_num);
					printf("\ncheck_data = %02x\n",check_data);
					printf("\nreceive_checksum = %02x\n",receive_checksum);
					for(int idx = 0;idx<frame_char_num;idx++)
					{
						printf("%02x ",perframe_data[idx]);
					}
					printf("\n");
				}

				if (check_data == receive_checksum)
				{
					good_frame_dat_num++;
					if(good_frame_dat_num<=1)
					{
							start_time = system_clock::now();
							add_second = 0.0;
					}
					else
					{
							end_time = system_clock::now();
							auto duration = duration_cast<microseconds>(end_time - start_time);
							add_second = double(duration.count()) * microseconds::period::num / microseconds::period::den;
					}

					Frame_Num++;
					NavInfo_To_File(&MEMS_IMU_Output_Data);
					if(good_frame_dat_num%200==0)
					{
						float ratio = (float)good_frame_dat_num/sum_frame_dat_num;
						printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~MTI-G-710 Information Begin~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");	
						printf("good_num = %d bad_num = %d goode_ratio = %.8f\n",good_frame_dat_num,bad_frame_data_num,ratio);	
						printf("angle_x = %.6f \n",(float)MEMS_IMU_Output_Data.angle[0]);  
						printf("angle_y = %.6f \n",(float)MEMS_IMU_Output_Data.angle[1]);   
						printf("angle_z = %.6f \n",(float)MEMS_IMU_Output_Data.angle[2]); 
						printf("gyro_data_x = %.6f \n",(float)MEMS_IMU_Output_Data.gyro_data[0]); 
						printf("gyro_data_y = %.6f \n",(float)MEMS_IMU_Output_Data.gyro_data[1]); 
						printf("gyro_data_z = %.6f \n",(float)MEMS_IMU_Output_Data.gyro_data[2]); 
						printf("acce_data_x = %.6f \n",(float)MEMS_IMU_Output_Data.acce_data[0]); 
						printf("acce_data_y = %.6f \n",(float)MEMS_IMU_Output_Data.acce_data[1]); 
						printf("acce_data_z = %.6f \n",(float)MEMS_IMU_Output_Data.acce_data[2]); 
						printf("temp = %.6f \n",(float)MEMS_IMU_Output_Data.temp); 
						printf("time_stmap_mcu = %.6f \n",(float)MEMS_IMU_Output_Data.timestamp/1000000); 
						printf("time_stmap_nvidia = %.6f \n",add_second); 
						printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~MTI-G-710 Information End~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");	
					}

				}	
				else
				{
					bad_frame_data_num++;
					Nav_Error_Info_To_File(perframe_data,frame_char_num,receive_checksum,check_data);
				}
				
				memset(perframe_data, 0, Max_Perframe_Nums);

			}
			else //left two chars : header1 and header2
			{
				break;
			}
			

		}
		else
		{
			// not have one frame,break from here finally
			cur_pos++;
	    }
																
	}
	//printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~out of Analy_Sins_Frame_Sanchi~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");

}


/*
 *函数功能：从串口接收并解析数据
 *输入参数：
 *输出参数：接收到一帧数据标志位sss
 *备    注：一次接收单个字符
 *
 */
bool Analy_MEMS_IMU_Data::Analy_Sins_Frame( char Receive_Data,int  & Frame_Num)
{
	static char old_receive_data;
	char cur_receive_data;

	static int good_frame_num;
	static int bad_frame_num;
	static int frame_header_num;

	static int Sins_Data_State = 1;
	static int cur_datasum;
	static int cur_data_num;
	int perframe_datasum = sizeof(MEMS_IMU_Output_Info);
	static char perframe_data[Max_Perframe_Nums]; //防止内存泄漏!!

	int checksum_data = 0;
	char check_data = 0x00;

	int output_bytenums = sizeof(MEMS_IMU_Output_Info);
	bool framedata_flag = false;
	cur_receive_data = Receive_Data;

	bool Find_Sins_Dev;
	char Sins_State;
	bool Sins_Frame_Flag;

	static int continue_bad_data = 0;
	int max_continue_bad_data = 1000;

	switch (Sins_Data_State)
	{
		case 1:
			{

				if ((unsigned char)old_receive_data == MTI_G_710_HEADER1 && (unsigned char)cur_receive_data ==MTI_G_710_HEADER2)
				{
					frame_header_num++;
					if(frame_header_num<=1)
					{
						start_time = system_clock::now();
						add_second = 0.0;
						old_add_second = add_second;
					}
					else
					{
						end_time = system_clock::now();
						auto duration = duration_cast<microseconds>(end_time - start_time);
						add_second = double(duration.count()) * microseconds::period::num / microseconds::period::den;
					}

					cur_datasum = Header_ByNums;
					Sins_Data_State = 2;
				}
				break;
			};

		case 2:
			{
				cur_datasum++;
				perframe_data[cur_data_num++] = cur_receive_data;
				if (cur_datasum == ( Header_ByNums+ perframe_datasum)) 
				{
					Sins_Data_State = 3;
				}
				break;
			};

			//case SINS_CheckSum:
		case 3:
			{

				cur_datasum++;	
				if (cur_datasum == ( Header_ByNums + perframe_datasum + CheckSum_ByNums))
				{	
					//sum_check
					char check_data = CheckSum(perframe_data, cur_data_num);
					if(debug_flag)
					{
						printf("check_data = %d cur_receive_data = %d\n",check_data,cur_receive_data);
					}
					if(check_data ==cur_receive_data)
					{	
						framedata_flag = true;
						continue_bad_data = 0;
						good_frame_num++;

						float sucess_ratio = (float)good_frame_num/frame_header_num;		
						memcpy(&MEMS_IMU_Output_Data,perframe_data,output_bytenums);//把接收到的信息转换成结构体
						/**********************************调试信息*****************************/
						if(frame_header_num%200==0&&display_data_flag)
						{  
							printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~MEMS SINS Information Begin~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");	
							printf("good_num = %d bad_num = %d goode_ratio = %.8f\n",good_frame_num,bad_frame_num,sucess_ratio);	
							printf("angle_x = %.6f \n",(float)MEMS_IMU_Output_Data.angle[0]);  
							printf("angle_y = %.6f \n",(float)MEMS_IMU_Output_Data.angle[1]);   
							printf("angle_z = %.6f \n",(float)MEMS_IMU_Output_Data.angle[2]); 
							printf("gyro_data_x = %.6f \n",(float)MEMS_IMU_Output_Data.gyro_data[0]); 
							printf("gyro_data_y = %.6f \n",(float)MEMS_IMU_Output_Data.gyro_data[1]); 
							printf("gyro_data_z = %.6f \n",(float)MEMS_IMU_Output_Data.gyro_data[2]); 
							printf("acce_data_x = %.6f \n",(float)MEMS_IMU_Output_Data.acce_data[0]); 
							printf("acce_data_y = %.6f \n",(float)MEMS_IMU_Output_Data.acce_data[1]); 
							printf("acce_data_z = %.6f \n",(float)MEMS_IMU_Output_Data.acce_data[2]); 
							printf("temp = %.6f \n",(float)MEMS_IMU_Output_Data.temp); 
							printf("time_stmap_mcu = %.6f \n",(float)MEMS_IMU_Output_Data.timestamp/1000000); 
							printf("time_stmap_nvidia = %.6f \n",add_second); 
							printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~MEMS SINS Information End~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");	
						}
					
					}
					else
					{
						if(~framedata_flag) // last data is not right
						{
							continue_bad_data++;
						}
						else // last data is right		
						{
							continue_bad_data = 1; // the first data
						}

						framedata_flag = false;
						bad_frame_num++;

						int  real_gap_time_us = (int)((add_second - old_add_second)*1000000); //us
						MEMS_IMU_Output_Data.timestamp = old_timestamp + real_gap_time_us; // calculate time
					}		

					if(continue_bad_data<=max_continue_bad_data)
					{
							NavInfo_To_File(&MEMS_IMU_Output_Data,framedata_flag);
					}
					else
					{
						
					}

					old_timestamp = MEMS_IMU_Output_Data.timestamp;
					old_add_second = add_second;
					cur_datasum = 0;
					cur_data_num= 0;
					old_receive_data = 0;
					cur_receive_data = 0;
					perframe_datasum = 0;
					memset(perframe_data, 0, Max_Perframe_Nums);
					Sins_Data_State = 1;
					checksum_data = 0;
					check_data = 0x00;

				}  

				break;
			};

		default:
			{
				cur_data_num= 0;
				Sins_Data_State = 1;
				break;
			};

	}

	old_receive_data = cur_receive_data;
	Frame_Num = good_frame_num;
	return framedata_flag;

}






/*
 *函数功能：从串口接收并解析数据
 *输入参数：
 *输出参数：接收到一帧数据标志位sss
 *备    注：一次接收单个字符
 *
 */
bool Analy_MEMS_IMU_Data::Analy_Sins_Frame( char Receive_Data)
{
	static char old_receive_data;
	char cur_receive_data;
	static int cur_datasum;
	static int cur_data_num;
	int perframe_datasum = sizeof(MEMS_IMU_Output_Info);

	static char perframe_data[Max_Perframe_Nums]; // 防止内存泄漏!!
	cur_receive_data = Receive_Data;

	valid_frame_flag = false;
	recv_frame_flag = false;

	switch (Data_State)
	{
		case header_state:
			{

				if ((unsigned char)old_receive_data == MTI_G_710_HEADER1 && (unsigned char)cur_receive_data ==MTI_G_710_HEADER2)
				{
					frame_header_num++;
					if(frame_header_num<=1)
					{
						start_time = system_clock::now();
						Mems_IMU_Output_Data_Nvidia.timestamp_nvidia = 0.0;
					}
					else
					{
						end_time = system_clock::now();
						auto duration = duration_cast<microseconds>(end_time - start_time);
						Mems_IMU_Output_Data_Nvidia.timestamp_nvidia = double(duration.count()) * microseconds::period::num / microseconds::period::den;
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
						memcpy(&Mems_IMU_Output_Data_Nvidia.sins_data,perframe_data,perframe_datasum);//把接收到的信息转换成结构体					
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




bool Analy_MEMS_IMU_Data::Sins_Postprocess(char Receive_Data,bool disp_flag,bool add_check_lost_flag)
{
	bool recv_frame_flag = Analy_Sins_Frame(Receive_Data);
	if(recv_frame_flag&&(frame_invaild_num>=1))
	{
		if(frame_invaild_num==1)
		{
			Old_Mems_IMU_Output_Data_Nvidia = Mems_IMU_Output_Data_Nvidia;
		}

		gap_counter_us = Mems_IMU_Output_Data_Nvidia.sins_data.timestamp - Old_Mems_IMU_Output_Data_Nvidia.sins_data.timestamp;  // ms
		gap_counter_ms = (int)(gap_counter_us/1000);
		gap_nvidia_s = Mems_IMU_Output_Data_Nvidia.timestamp_nvidia - Old_Mems_IMU_Output_Data_Nvidia.timestamp_nvidia;  // s
		gap_nvidia_ms = (int)(gap_nvidia_s*1000);
		gap_nvidia_us = (int)(gap_nvidia_s*1000000);

		if(valid_frame_flag&&(gap_counter_ms<gap_time_ms_max)&&(gap_counter_ms>gap_time_ms_min)) // check bit is right
		{
			if(debug_flag)
			{
				printf("check bit is right============================\n");
			}
			frame_data_state = check_right_state;
			NavInfo_To_File(&Mems_IMU_Output_Data_Nvidia);
		}
		else if(valid_frame_flag&&(gap_counter_ms>gap_time_ms_max)&&(add_check_lost_flag)) //lost data
		{
			if(debug_flag)
			{
				printf("lost data============================\n");
			}
			float mul_num = (float)gap_counter_ms/(float)gap_time_ms_mean;
			cur_lost_frame_num = (round)(mul_num)-1; // attention:one data is valid here
			if((cur_lost_frame_num<=add_lost_frame_num_max)&&(cur_lost_frame_num>=1))
			{
				frame_lost_num+=cur_lost_frame_num;
				for(int i=1;i<=cur_lost_frame_num;i++)
				{
					Mems_IMU_Output_Info_Nvidia  Lost_Sins_Output_Data_Nvidia;
					Lost_Sins_Output_Data_Nvidia = Old_Mems_IMU_Output_Data_Nvidia;
					int gap_time_us_mean = gap_time_ms_mean*1000;
					double gap_time_s_mean = (double)gap_time_ms_mean/(double)1000;					
					Lost_Sins_Output_Data_Nvidia.sins_data.timestamp = Old_Mems_IMU_Output_Data_Nvidia.sins_data.timestamp + i*gap_time_us_mean;
					Lost_Sins_Output_Data_Nvidia.timestamp_nvidia = Old_Mems_IMU_Output_Data_Nvidia.timestamp_nvidia + (double)i*gap_time_s_mean;
					frame_data_state = lost_frame_state; // data is lost
					NavInfo_To_File(&Lost_Sins_Output_Data_Nvidia);
				}
				// have valid data here
				frame_data_state = check_right_state;; // data is invalid
				NavInfo_To_File(&Mems_IMU_Output_Data_Nvidia);
			}
		}
		else if((!valid_frame_flag)&&(gap_nvidia_ms>gap_time_ms_max)&&(add_check_lost_flag)) // checksum data is wrong
		{
			if(debug_flag)
			{
				printf("checksum data is wrong============================\n");
			}
			frame_check_error_num++;
			Mems_IMU_Output_Info_Nvidia  Void_Sins_Output_Data_Nvidia;
			Void_Sins_Output_Data_Nvidia = Mems_IMU_Output_Data_Nvidia;
			Void_Sins_Output_Data_Nvidia.sins_data.timestamp = Mems_IMU_Output_Data_Nvidia.sins_data.timestamp + gap_nvidia_us;
			NavInfo_To_File(&Void_Sins_Output_Data_Nvidia);
		}
		else
		{

		}

		frame_truth_num = frame_invaild_num + frame_lost_num + frame_check_error_num;
		invalid_ratio = (double)frame_invaild_num/(double)frame_truth_num;
		lost_ratio = (double)frame_lost_num/(double)frame_truth_num;
		check_error_ratio = (double)frame_check_error_num/(double)frame_truth_num;
		if(disp_flag&&frame_header_num%max_display_data_num==0)
		{
			printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~MEMS SINS Information Begin~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");	
			printf("num = %d invalid_ratio = %.8f lost_ratio = %.8f check_error_ratio = %.8f\n",frame_truth_num,invalid_ratio,lost_ratio,check_error_ratio);	
			printf("invalid_num = %d lost_num = %d check_error_num = %d\n",frame_invaild_num,frame_lost_num,frame_check_error_num);	
			printf("angle_x = %.6f \n",(float)Mems_IMU_Output_Data_Nvidia.sins_data.angle[0]);  
			printf("angle_y = %.6f \n",(float)Mems_IMU_Output_Data_Nvidia.sins_data.angle[1]);   
			printf("angle_z = %.6f \n",(float)Mems_IMU_Output_Data_Nvidia.sins_data.angle[2]); 
			printf("gyro_data_x = %.6f \n",(float)Mems_IMU_Output_Data_Nvidia.sins_data.gyro_data[0]); 
			printf("gyro_data_y = %.6f \n",(float)Mems_IMU_Output_Data_Nvidia.sins_data.gyro_data[1]); 
			printf("gyro_data_z = %.6f \n",(float)Mems_IMU_Output_Data_Nvidia.sins_data.gyro_data[2]); 
			printf("acce_data_x = %.6f \n",(float)Mems_IMU_Output_Data_Nvidia.sins_data.acce_data[0]); 
			printf("acce_data_y = %.6f \n",(float)Mems_IMU_Output_Data_Nvidia.sins_data.acce_data[1]); 
			printf("acce_data_z = %.6f \n",(float)Mems_IMU_Output_Data_Nvidia.sins_data.acce_data[2]); 
			printf("temp = %.6f \n",(float)Mems_IMU_Output_Data_Nvidia.sins_data.temp); 
			printf("time_stmap_mcu = %.6f \n",(float)Mems_IMU_Output_Data_Nvidia.sins_data.timestamp/1000000); 
			printf("time_stmap_nvidia = %.6f \n",Mems_IMU_Output_Data_Nvidia.timestamp_nvidia); 
			printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~MEMS SINS Information End~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");	
		}

		// update
		Old_Mems_IMU_Output_Data_Nvidia = Mems_IMU_Output_Data_Nvidia;
	}

	return recv_frame_flag;

}




void Analy_MEMS_IMU_Data::NavInfo_To_File(Mems_IMU_Output_Info_Nvidia * sins_output_info)
{
	static int fflush_num;
	fflush_num++;

	time_t timep;
	struct tm *p;
	time(&timep);
	p=gmtime(&timep);


	//output time
	fprintf(sins_filestream,"%04d-",1900+p->tm_year);
	fprintf(sins_filestream,"%02d-",1+p->tm_mon);
	fprintf(sins_filestream,"%02d-",p->tm_mday);
	fprintf(sins_filestream,"%02d-",p->tm_hour);
	fprintf(sins_filestream,"%02d-",p->tm_min);
	fprintf(sins_filestream,"%02d    ",p->tm_sec);
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->sins_data.angle[0]);  
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->sins_data.angle[1]);   
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->sins_data.angle[2]); 
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->sins_data.gyro_data[0]); 
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->sins_data.gyro_data[1]); 
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->sins_data.gyro_data[2]); 
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->sins_data.acce_data[0]); 
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->sins_data.acce_data[1]); 
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->sins_data.acce_data[2]); 
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->sins_data.temp); 
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->sins_data.timestamp/1000000); 
	fprintf(sins_filestream,"%.6f ",sins_output_info->timestamp_nvidia);  
	
	fprintf(sins_filestream,"%d ",frame_data_state);  
	fprintf(sins_filestream,"%d ",frame_truth_num);  
	fprintf(sins_filestream,"%d ",frame_header_num);  
	fprintf(sins_filestream,"%d ",frame_check_error_num);  
	fprintf(sins_filestream,"%d ",frame_lost_num);  

	fprintf(sins_filestream,"%.8f ",invalid_ratio);   
	fprintf(sins_filestream,"%.8f ",lost_ratio);  
	fprintf(sins_filestream,"%.8f ",check_error_ratio);  
	fprintf(sins_filestream,"\n");
	if(fflush_num==100)
	{
		fflush_num=0;
		fflush(sins_filestream);
	}					

}





void Analy_MEMS_IMU_Data::Nav_Error_Info_To_File(char * nav_data, unsigned int data_len,char data1,char data2)
{
	static int fflush_num;
	fflush_num++;

	time_t timep;
	struct tm *p;
	time(&timep);
	p=gmtime(&timep);

	//output time
	fprintf(error_sins_filestream,"%04d-",1900+p->tm_year);
	fprintf(error_sins_filestream,"%02d-",1+p->tm_mon);
	fprintf(error_sins_filestream,"%02d-",p->tm_mday);
	fprintf(error_sins_filestream,"%02d-",p->tm_hour);
	fprintf(error_sins_filestream,"%02d-",p->tm_min);
	fprintf(error_sins_filestream,"%02d    ",p->tm_sec);

	//output time
	for(int i=0;i<data_len;i++)
	{
			fprintf(error_sins_filestream,"0x%02x ",nav_data[i]);
	}
	fprintf(error_sins_filestream,"\n");
	fprintf(error_sins_filestream,"0x%02x ",data1);
	fprintf(error_sins_filestream,"0x%02x ",data2);
	fprintf(error_sins_filestream,"\n");


	if(fflush_num==1)
	{
		fflush_num=0;
		fflush(error_sins_filestream);
	}					

}


void Analy_MEMS_IMU_Data::NavInfo_To_File(MEMS_IMU_Output_Info * sins_output_info)
{
	static int fflush_num;
	fflush_num++;

	time_t timep;
	struct tm *p;
	time(&timep);
	p=gmtime(&timep);

	//output time
	fprintf(sins_filestream,"%04d-",1900+p->tm_year);
	fprintf(sins_filestream,"%02d-",1+p->tm_mon);
	fprintf(sins_filestream,"%02d-",p->tm_mday);
	fprintf(sins_filestream,"%02d-",p->tm_hour);
	fprintf(sins_filestream,"%02d-",p->tm_min);
	fprintf(sins_filestream,"%02d    ",p->tm_sec);
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->angle[0]);  
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->angle[1]);   
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->angle[2]); 
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->gyro_data[0]); 
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->gyro_data[1]); 
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->gyro_data[2]); 
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->acce_data[0]); 
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->acce_data[1]); 
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->acce_data[2]); 
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->temp); 
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->timestamp/1000000); 
	fprintf(sins_filestream,"%.6f ",add_second);  
	fprintf(sins_filestream,"\n");

	if(fflush_num==100)
	{
		fflush_num=0;
		fflush(sins_filestream);
	}					

}



void Analy_MEMS_IMU_Data::NavInfo_To_File(MEMS_IMU_Output_Info * sins_output_info,bool valid_flag)
{
	static int fflush_num;
	fflush_num++;

	time_t timep;
	struct tm *p;
	time(&timep);
	p=gmtime(&timep);

	//output time
	fprintf(sins_filestream,"%04d-",1900+p->tm_year);
	fprintf(sins_filestream,"%02d-",1+p->tm_mon);
	fprintf(sins_filestream,"%02d-",p->tm_mday);
	fprintf(sins_filestream,"%02d-",p->tm_hour);
	fprintf(sins_filestream,"%02d-",p->tm_min);
	fprintf(sins_filestream,"%02d    ",p->tm_sec);
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->angle[0]);  
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->angle[1]);   
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->angle[2]); 
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->gyro_data[0]); 
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->gyro_data[1]); 
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->gyro_data[2]); 
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->acce_data[0]); 
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->acce_data[1]); 
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->acce_data[2]); 
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->temp); 
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->timestamp/1000000); 
	fprintf(sins_filestream,"%.6f ",add_second);  
	fprintf(sins_filestream,"%d ",valid_flag);  
	fprintf(sins_filestream,"\n");

	if(fflush_num==100)
	{
		fflush_num=0;
		fflush(sins_filestream);
	}					

}


char Analy_MEMS_IMU_Data::CheckSum( char * nav_data, unsigned int data_len)
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

