#include <stdio.h>
#include <ctime>
#include <time.h>
#include <unistd.h>
#include <chrono>
#include <pthread.h>
#include "sensor/Analy_DVL_Data.h"

using namespace chrono;
using namespace std;

Analy_DVL_Data::Analy_DVL_Data()
{                                                                                
	// file name
	Init_Data();
	time_t currtime =time(NULL);
	tm *sins_p =localtime(&currtime);
	sprintf(dvldata_file_name,"../data/output_data/dvl_data%04d%02d%02d%02d%02d%02d.txt",1900+sins_p->tm_year,1+sins_p->tm_mon,sins_p->tm_mday,sins_p->tm_hour,sins_p->tm_min,sins_p->tm_sec);	
	dvl_filestream = fopen(dvldata_file_name,"w+");
	if (dvl_filestream==NULL)
  {
		printf("can't create file to save dvl data!\n");
	}
	else
	{
		printf("successful create file to save dvl data!\n");
	}

}



Analy_DVL_Data::~Analy_DVL_Data()
{

	fclose(dvl_filestream);
}


void Analy_DVL_Data::Init_Data(void)
{
	memset(&DVL_Output_Data,0,sizeof(DVL_Output_Data));
	memset(&DVL_Output_Data_Nvidia,0,sizeof(DVL_Output_Data_Nvidia));	
	memset(&DVL_FilterVel_Data,0,sizeof(DVL_FilterVel_Data));	
	memset(&old_DVL_FilterVel_Data,0,sizeof(old_DVL_FilterVel_Data));	
}


void Analy_DVL_Data::Analy_DVL_Frame_( std::vector<char> & recv_Data,int  & Frame_Num,std::vector<DVL_Output_Info> & output_data)
{
//	printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~come in Analy_DVL_Frame_~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
	char msg_id = 0;
	char counter_num = 0;
	Frame_Num = 0;
	static int bad_frame_data_num = 0;
	static int good_frame_dat_num = 0;
	static int sum_frame_dat_num = 0;

	int frame_char_num = sizeof(DVL_Output_Info);
	char perframe_data[DVL_Max_Frame_Nums] = {0}; //防止内存泄漏!!
	if(recv_Data.size()<(frame_char_num+DVL_Header_ByNums + DVL_ID_ByNums + DVL_CHECK_ByNums)) // 52:change here ==========one frame data num :38============= 
	{
		return;
	}
	int max_data_num = recv_Data.size();
	int cur_pos = 0;
	bool over_flag = false;

	while(cur_pos<recv_Data.size())   //attention
 	{
		if(recv_Data[cur_pos]==DVL_HEADER1&&recv_Data[cur_pos+1]==DVL_HEADER2) 
		{
			//printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~dvl DVL_HEADER1 begin~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");	
			if(cur_pos!=0)
			{
				recv_Data.erase(recv_Data.begin(),recv_Data.begin()+cur_pos);		
			}
			cur_pos = 0; // reset cur_pos 	
			if(recv_Data.size()>=(frame_char_num + DVL_Header_ByNums + DVL_ID_ByNums + DVL_CHECK_ByNums)) // 2+1 = 3
			{
				//analyze one frame 
				msg_id = recv_Data[cur_pos + DVL_Header_ByNums + DVL_ID_ByNums-1]; //0x7e	
				for(int idx = 0;idx<DVL_Header_ByNums+DVL_ID_ByNums;idx++)
				{
					perframe_data[idx] = recv_Data[idx]; //check sum
				}

				recv_Data.erase(recv_Data.begin(),recv_Data.begin()+DVL_Header_ByNums+DVL_ID_ByNums);
				for(int i=0;i<(int)frame_char_num;i++)
				{
					perframe_data[i+DVL_Header_ByNums+DVL_ID_ByNums] = recv_Data.front();
					recv_Data.erase(recv_Data.begin());	
				}
				memcpy(&DVL_Output_Data,perframe_data+DVL_Header_ByNums+DVL_ID_ByNums,frame_char_num);
				char check_data = CheckSum(perframe_data, frame_char_num+DVL_Header_ByNums+DVL_ID_ByNums);
				char receive_checksum = recv_Data.front();
				recv_Data.erase(recv_Data.begin());	
				cur_pos = 0; // reset cur_pos 
				sum_frame_dat_num++;
				/*
				printf("max_data_num = %d\n",max_data_num);	
				printf("check_data = %02x\n",check_data);	
				printf("receive_checksum = %02x\n",receive_checksum);	
				printf("frame_char_num = %d\n",frame_char_num);	
				printf("msg_id = %d\n",msg_id);	
				*/
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

					float dvl_y_vel = (float)DVL_Output_Data.vel_btm[1]*0.001;
					if(abs(dvl_y_vel)>8)
					{
						dvl_invalid_data_num++;
					}

					float dvl_invalid_ratio = (float)dvl_invalid_data_num/good_frame_dat_num;

					Frame_Num++;
				//	NavInfo_To_File(&Sins_Output_Data_Sanchi);
					if(good_frame_dat_num%50==0)
					{
						float ratio = (float)good_frame_dat_num/sum_frame_dat_num;
						printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~dvl information begin~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");	
						printf("good_num = %d bad_num = %d goode_ratio = %.8f  = %.6f\n",good_frame_dat_num,bad_frame_data_num,ratio);
						printf("invalid_ratio = %.6f \n",dvl_invalid_ratio); 
						printf("dvl_invalid_data_num = %d \n",dvl_invalid_data_num); 
						printf("vel_btm1 = %.6f m/s\n",(float)DVL_Output_Data.vel_btm[0]*0.001);   //mm/s
						printf("vel_btm2 = %.6f m/s\n",(float)DVL_Output_Data.vel_btm[1]*0.001);  
						printf("vel_btm3 = %.6f m/s\n",(float)DVL_Output_Data.vel_btm[2]*0.001);  
						printf("vel_ref1 = %.6f m/s\n",(float)DVL_Output_Data.vel_ref[0]*0.001);  
						printf("vel_ref2 = %.6f m/s\n",(float)DVL_Output_Data.vel_ref[1]*0.001);  
						printf("vel_ref3 = %.6f m/s\n",(float)DVL_Output_Data.vel_ref[2]*0.001);  
						printf("temp_dvl = %.6f du\n",(float)DVL_Output_Data.temp_dvl*0.01);  
						printf("timestamp1 = %.6f s\n",(float)DVL_Output_Data.timestamp*0.000001);  
						printf("timestamp2 = %.6f s\n",add_second);  
						printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~dvl information end~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");	
					}

					NavInfo_To_File(&DVL_Output_Data);

				}	
				else
				{
					bad_frame_data_num++;
				}
				memset(perframe_data, 0, DVL_Max_Frame_Nums);
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

	//printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~out of Analy_DVL_Frame_~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");	

}


/*
 *函数功能：从串口接收并解析数据
 *输入参数：
 *输出参数：接收到一帧数据标志位sss
 *备    注：一次接收单个字符
 *
 */
bool Analy_DVL_Data::Analy_DVL_Frame( char Receive_Data,bool display_flag,bool filter_flag)
{
	static char old_receive_data;
	char cur_receive_data;
	static int cur_datasum;
	static int cur_data_num;
	int perframe_datasum = sizeof(DVL_Output_Data_Nvidia.dvl_data);

	static char perframe_data[Max_Perframe_Nums]; // 防止内存泄漏!!
	cur_receive_data = Receive_Data;

	valid_frame_flag = false;
	recv_frame_flag = false;

	switch (Data_State)
	{
		case header_state:
			{

				if ((unsigned char)old_receive_data == DVL_HEADER1 && (unsigned char)cur_receive_data ==DVL_HEADER2)
				{
					frame_header_num++;
					if(frame_header_num<=1)
					{
						start_time = system_clock::now();
						DVL_Output_Data_Nvidia.timestamp_nvidia = 0.0;
					}
					else
					{
						end_time = system_clock::now();
						auto duration = duration_cast<microseconds>(end_time - start_time);
						DVL_Output_Data_Nvidia.timestamp_nvidia = double(duration.count()) * microseconds::period::num / microseconds::period::den;
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
						printf("check_data = %d cur_receive_data = %d\n",check_data,cur_receive_data);
					}
					if(check_data ==cur_receive_data)
					{	
						valid_frame_flag = true;
						frame_invaild_num++;
						memcpy(&DVL_Output_Data_Nvidia.dvl_data,perframe_data,perframe_datasum);//把接收到的信息转换成结构体	
						if(filter_flag)  // filter dvl data
						{
							SimFilter_DVL_Vel();
						}
						float dvl_y_vel = (float)DVL_Output_Data_Nvidia.dvl_data.vel_btm[1]*0.001;
						if(abs(dvl_y_vel)<max_vel)
						{
							dvl_invalid_data_num++;
						}
						float dvl_invalid_ratio = (float)dvl_invalid_data_num/(float)frame_invaild_num;
						NavInfo_To_File(&DVL_Output_Data_Nvidia);	
						if(display_flag&&(frame_header_num%max_display_data_num == 0))
						{
							printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~dvl information Begin~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
							printf("frame_invaild_num = %d vel_invaild_num = %d dvl_invalid_ratio = %.6f\n",frame_invaild_num,dvl_invalid_data_num,dvl_invalid_ratio);  	
							printf("vel_btm1 = %.6f m/s\n",(float)DVL_Output_Data_Nvidia.dvl_data.vel_btm[0]*0.001);   //mm/s
							printf("vel_btm2 = %.6f m/s\n",(float)DVL_Output_Data_Nvidia.dvl_data.vel_btm[1]*0.001);  
							printf("vel_btm3 = %.6f m/s\n",(float)DVL_Output_Data_Nvidia.dvl_data.vel_btm[2]*0.001);  
							printf("vel_ref1 = %.6f m/s\n",(float)DVL_Output_Data_Nvidia.dvl_data.vel_ref[0]*0.001);  
							printf("vel_ref2 = %.6f m/s\n",(float)DVL_Output_Data_Nvidia.dvl_data.vel_ref[1]*0.001);  
							printf("vel_ref3 = %.6f m/s\n",(float)DVL_Output_Data_Nvidia.dvl_data.vel_ref[2]*0.001);  
							printf("height1 = %.6f m\n",(float)DVL_Output_Data_Nvidia.dvl_data.height[0]*0.01);  //m 
							printf("height2 = %.6f m\n",(float)DVL_Output_Data_Nvidia.dvl_data.height[1]*0.01);  
							printf("height3 = %.6f m\n",(float)DVL_Output_Data_Nvidia.dvl_data.height[2]*0.01);  							
							printf("height4 = %.6f m\n",(float)DVL_Output_Data_Nvidia.dvl_data.height[3]*0.01);  
							printf("temp_dvl = %.6f du\n",(float)DVL_Output_Data_Nvidia.dvl_data.temp_dvl*0.01);  
							printf("timestamp1 = %.6f s\n",(float)DVL_Output_Data_Nvidia.dvl_data.timestamp*0.000001);  
							printf("timestamp2 = %.6f s\n",DVL_Output_Data_Nvidia.timestamp_nvidia);  
							printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~dvl information end~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");	
							
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



void Analy_DVL_Data::SimFilter_DVL_Vel(void)
{
	if(DVL_Output_Data_Nvidia.dvl_data.vel_btm[0]!=-32768&&DVL_Output_Data_Nvidia.dvl_data.vel_btm[1]!=-32768)
	{
		continue_valid_cur_num = 0;
		DVL_FilterVel_Data.dvl_flag = vel_bottom_invalid_state;
		for(int i=0;i<3;i++)
		{
			DVL_FilterVel_Data.dvl_vel[i] = DVL_Output_Data_Nvidia.dvl_data.vel_btm[i];			
		}
		
	}
	else
	{		
		if(DVL_Output_Data_Nvidia.dvl_data.vel_ref[0]!=-32768&&DVL_Output_Data_Nvidia.dvl_data.vel_ref[1]!=-32768)
		{
			continue_valid_cur_num = 0;
			DVL_FilterVel_Data.dvl_flag = vel_water_invalid_state;
			for(int i=0;i<3;i++)
			{
				DVL_FilterVel_Data.dvl_vel[i] = DVL_Output_Data_Nvidia.dvl_data.vel_ref[i];			
			}
		}
		else
		{
			continue_valid_cur_num++;
			DVL_FilterVel_Data.dvl_flag = old_DVL_FilterVel_Data.dvl_flag;
			for(int i=0;i<3;i++)
			{
				DVL_FilterVel_Data.dvl_vel[i] = old_DVL_FilterVel_Data.dvl_vel[i];			
			}
			
			if(DVL_FilterVel_Data.dvl_vel[0]==-32768||DVL_FilterVel_Data.dvl_vel[1]==-32768)
			{
				DVL_FilterVel_Data.dvl_flag = valid_data_state;
				for(int i=0;i<3;i++)
				{
					DVL_FilterVel_Data.dvl_vel[i] = 0;			
				}			
				
			}
		
		}

	}

	if(DVL_FilterVel_Data.dvl_flag!=0)
	{
		old_DVL_FilterVel_Data = DVL_FilterVel_Data;
	}

	if(continue_valid_cur_num>=continue_valid_max_num)
	{
		DVL_FilterVel_Data.dvl_flag = valid_data_state;
	}

	DVL_FilterVel_Data.timestamp = DVL_Output_Data_Nvidia.dvl_data.timestamp;

}



char Analy_DVL_Data::CheckSum( char * nav_data, unsigned int data_len)
{
	int data_sum =0;
	char check_data;	

	for(int i=0;i<data_len;i++)
	{
		data_sum+=nav_data[i];	
	}

	check_data = data_sum&0xFF;

	return (check_data);
}



void Analy_DVL_Data::NavInfo_To_File(DVL_Output_Info * data)
{
	static int fflush_num;
	fflush_num++;

	time_t timep;
	struct tm *p;
	time(&timep);
	p=gmtime(&timep);

	//output time
	fprintf(dvl_filestream,"%04d-",1900+p->tm_year);
	fprintf(dvl_filestream,"%02d-",1+p->tm_mon);
	fprintf(dvl_filestream,"%02d-",p->tm_mday);
	fprintf(dvl_filestream,"%02d-",p->tm_hour);
	fprintf(dvl_filestream,"%02d-",p->tm_min);
	fprintf(dvl_filestream,"%02d    ",p->tm_sec);
	fprintf(dvl_filestream,"%.6f ",(float)DVL_Output_Data.vel_btm[0]*0.001);   //mm/s
	fprintf(dvl_filestream,"%.6f ",(float)DVL_Output_Data.vel_btm[1]*0.001);  
	fprintf(dvl_filestream,"%.6f ",(float)DVL_Output_Data.vel_btm[2]*0.001);  
	fprintf(dvl_filestream,"%.6f ",(float)DVL_Output_Data.vel_ref[0]*0.001);  
	fprintf(dvl_filestream,"%.6f ",(float)DVL_Output_Data.vel_ref[1]*0.001);  
	fprintf(dvl_filestream,"%.6f ",(float)DVL_Output_Data.vel_ref[2]*0.001);  
	fprintf(dvl_filestream,"%.6f ",(float)DVL_Output_Data.height[0]*0.01);  //m 
	fprintf(dvl_filestream,"%.6f ",(float)DVL_Output_Data.height[1]*0.01);  
	fprintf(dvl_filestream,"%.6f ",(float)DVL_Output_Data.height[2]*0.01);  
	fprintf(dvl_filestream,"%.6f ",(float)DVL_Output_Data.height[3]*0.01);  
	fprintf(dvl_filestream,"%.6f ",(float)DVL_Output_Data.temp_dvl*0.01);  //deg
	fprintf(dvl_filestream,"%.6f ",(float)DVL_Output_Data.timestamp*0.000001);   // s
	fprintf(dvl_filestream,"%.6f ",add_second);   
	fprintf(dvl_filestream,"%d ",dvl_invalid_data_num);    
	fprintf(dvl_filestream,"\n");

	if(fflush_num==6)
	{
		fflush_num=0;
		fflush(dvl_filestream);
	}					

}




void Analy_DVL_Data::NavInfo_To_File(DVL_Output_Info_Nvidia * data)
{
	static int fflush_num;
	fflush_num++;

	time_t timep;
	struct tm *p;
	time(&timep);
	p=gmtime(&timep);

	//output time
	fprintf(dvl_filestream,"%04d-",1900+p->tm_year);
	fprintf(dvl_filestream,"%02d-",1+p->tm_mon);
	fprintf(dvl_filestream,"%02d-",p->tm_mday);
	fprintf(dvl_filestream,"%02d-",p->tm_hour);
	fprintf(dvl_filestream,"%02d-",p->tm_min);
	fprintf(dvl_filestream,"%02d    ",p->tm_sec);
	fprintf(dvl_filestream,"%.6f ",(float)data->dvl_data.vel_btm[0]*0.001);   //mm/s
	fprintf(dvl_filestream,"%.6f ",(float)data->dvl_data.vel_btm[1]*0.001);  
	fprintf(dvl_filestream,"%.6f ",(float)data->dvl_data.vel_btm[2]*0.001);  
	fprintf(dvl_filestream,"%.6f ",(float)DVL_FilterVel_Data.dvl_vel[0]*0.001);   //mm/s
	fprintf(dvl_filestream,"%.6f ",(float)DVL_FilterVel_Data.dvl_vel[1]*0.001);  
	fprintf(dvl_filestream,"%.6f ",(float)DVL_FilterVel_Data.dvl_vel[2]*0.001);  
	fprintf(dvl_filestream,"%d ",	DVL_FilterVel_Data.dvl_flag);  
	fprintf(dvl_filestream,"%.6f ",(float)data->dvl_data.vel_ref[0]*0.001);  
	fprintf(dvl_filestream,"%.6f ",(float)data->dvl_data.vel_ref[1]*0.001);  
	fprintf(dvl_filestream,"%.6f ",(float)data->dvl_data.vel_ref[2]*0.001);  
	fprintf(dvl_filestream,"%.6f ",(float)data->dvl_data.height[0]*0.01);  //m 
	fprintf(dvl_filestream,"%.6f ",(float)data->dvl_data.height[1]*0.01);  
	fprintf(dvl_filestream,"%.6f ",(float)data->dvl_data.height[2]*0.01);  
	fprintf(dvl_filestream,"%.6f ",(float)data->dvl_data.height[3]*0.01);  
	fprintf(dvl_filestream,"%.6f ",(float)data->dvl_data.temp_dvl*0.01);  //deg
	fprintf(dvl_filestream,"%.6f ",(float)data->dvl_data.timestamp*0.000001);   // s
	fprintf(dvl_filestream,"%.6f ",(float)data->timestamp_nvidia);   // s
	fprintf(dvl_filestream,"\n");

	if(fflush_num==6)
	{
		fflush_num=0;
		fflush(dvl_filestream);
	}					

}

