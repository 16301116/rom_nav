#include <stdio.h>
#include <ctime>
#include <time.h>
#include <unistd.h>
#include <chrono>
#include <pthread.h>
#include "sensor/Analy_GPS_Data.h"

using namespace chrono;
using namespace std;

Analy_GPS_Data::Analy_GPS_Data()
{             
	Init_Data();                                                                   
	// file name
	time_t currtime =time(NULL);
	tm *sins_p =localtime(&currtime);
	sprintf(gpsdata_file_name,"../data/output_data/gps_data%04d%02d%02d%02d%02d%02d.txt",1900+sins_p->tm_year,1+sins_p->tm_mon,sins_p->tm_mday,sins_p->tm_hour,sins_p->tm_min,sins_p->tm_sec);	
	gps_filestream = fopen(gpsdata_file_name,"w+");
	if (gps_filestream==NULL)
  {
		printf("can't create file to save gps data!\n");
	}
	else
	{
		printf("successful create file to save gps data!\n");
	}

}



Analy_GPS_Data::~Analy_GPS_Data()
{

	fclose(gps_filestream);
}


void Analy_GPS_Data::Init_Data(void)
{
		memset(&GNSS_SEND_FRAME,0,sizeof(GNSS_SEND_FRAME));
		memset(&nmea_utc_time,0,sizeof(nmea_utc_time));
		memset(&GPS_Output_Data,0,sizeof(GPS_Output_Data));
}


void Analy_GPS_Data::Analy_GPS_Frame( std::vector<char> & recv_Data,int  & Frame_Num,std::vector<GPS_Output_Info> & output_data)
{
//	printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~come in Analy_GPS_Frame_~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
	char msg_id = 0;
	char counter_num = 0;
	Frame_Num = 0;
	static int bad_frame_data_num = 0;
	static int good_frame_dat_num = 0;
	static int sum_frame_dat_num = 0;
	int frame_char_num = sizeof(GPS_Output_Info);
	char perframe_data[GPS_Max_Frame_Nums] = {0}; //防止内存泄漏!!
	if(recv_Data.size()<(frame_char_num + GPS_Header_ByNums + GPS_ID_ByNums + GPS_CHECK_ByNums)) // 52:change here ==========one frame data num :38============= 
	{
		return;
	}
	int max_data_num = recv_Data.size();
	int cur_pos = 0;
	bool over_flag = false;

	while(cur_pos<recv_Data.size())   //attention
 	{
		if(recv_Data[cur_pos]==GPS_HEADER1&&recv_Data[cur_pos+1]==GPS_HEADER2) 
		{
			//printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~GPS_HEADER1 begin~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");	
			if(cur_pos!=0)
			{
				recv_Data.erase(recv_Data.begin(),recv_Data.begin()+cur_pos);		
			}
			cur_pos = 0; // reset cur_pos 	
			if(recv_Data.size()>=(frame_char_num + GPS_Header_ByNums + GPS_ID_ByNums + GPS_CHECK_ByNums)) // 2+1 = 3
			{
				//analyze one frame 
				msg_id = recv_Data[cur_pos + GPS_Header_ByNums + GPS_ID_ByNums-1]; //0x7e	
				for(int idx = 0;idx<GPS_Header_ByNums + GPS_ID_ByNums;idx++)
				{
					perframe_data[idx] = recv_Data[idx]; //check sum
				}

				recv_Data.erase(recv_Data.begin(),recv_Data.begin() + GPS_Header_ByNums + GPS_ID_ByNums);
				for(int i=0;i<(int)frame_char_num;i++)
				{
					perframe_data[i+GPS_Header_ByNums + GPS_ID_ByNums] = recv_Data.front();
					recv_Data.erase(recv_Data.begin());	
				}
				memcpy(&GPS_Output_Data,perframe_data + GPS_Header_ByNums + GPS_ID_ByNums,frame_char_num);
				char check_data = CheckSum(perframe_data, frame_char_num + GPS_Header_ByNums + GPS_ID_ByNums);
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

					Frame_Num++;
				//	NavInfo_To_File(&Sins_Output_Data_Sanchi);
					if(good_frame_dat_num%max_display_data_num==0)
					{
						float ratio = (float)good_frame_dat_num/sum_frame_dat_num;
						printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~gps information begin~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");	
						printf("good_num = %d bad_num = %d goode_ratio = %.8f\n",good_frame_dat_num,bad_frame_data_num,ratio);	
						
						//double lat = (double)GPS_Output_Data.lat*0.0000001;
						//double lon = (double)GPS_Output_Data.lon*0.0000001;
						//lat_now = floor(lat) + (double)(lat - floor(lat))*10.0/6.0;
						//lon_now = floor(lon) + (double)(lon - floor(lon))*10.0/6.0;

						lat_now = (double)GPS_Output_Data.lat*0.0000001;
						lon_now = (double)GPS_Output_Data.lon*0.0000001;
						printf("lat = %.8f\n",lat_now);   
						printf("lon = %.8f\n",lon_now);  
						printf("alt = %.6f\n",(float)GPS_Output_Data.alt*0.001); //m  
						printf("speed = %.6f m/s\n",(float)GPS_Output_Data.speed*0.001/3.6); // m/s  
						printf("star = %d \n",(int)GPS_Output_Data.star);  
						printf("pdop = %.6f\n",(float)GPS_Output_Data.pdop*0.1);  
						printf("hdop = %.6f\n",(float)GPS_Output_Data.hdop*0.1);  
						printf("vdop = %.6f\n",(float)GPS_Output_Data.vdop*0.1);  
						printf("gnss_state = %d\n",GPS_Output_Data.gnss_state);  
						printf("timestamp1 = %.6f s\n",(float)GPS_Output_Data.timestamp*0.000001);   // s
						printf("timestamp2 = %.6f s\n",add_second);   // 
						printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~gps information end~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");	
					}
					NavInfo_To_File(&GPS_Output_Data);

				}	
				else
				{
					bad_frame_data_num++;
				}
				memset(perframe_data, 0, GPS_Max_Frame_Nums);
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

//	printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~out of Analy_GPS_Frame_~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");	

}




void Analy_GPS_Data::Analy_GPS_Frame( std::vector<char> & recv_Data,int  & Frame_Num,std::vector<GNSS_FRAME_Info> & output_data)
{
//	printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~come in Analy_GPS_Frame_~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
	char msg_id = 0;
	char counter_num = 0;
	Frame_Num = 0;
	static int bad_frame_data_num = 0;
	static int good_frame_dat_num = 0;
	static int sum_frame_dat_num = 0;
	int frame_char_num = sizeof(GNSS_SEND_FRAME);
	char perframe_data[GPS_Max_Frame_Nums] = {0}; //防止内存泄漏!!
	if(recv_Data.size()<(frame_char_num + GPS_Header_ByNums + GPS_ID_ByNums + GPS_CHECK_ByNums)) // 52:change here ==========one frame data num :38============= 
	{
		return;
	}
	int max_data_num = recv_Data.size();
	int cur_pos = 0;
	bool over_flag = false;

	while(cur_pos<recv_Data.size())   //attention
 	{
		if(recv_Data[cur_pos]==GPS_HEADER1&&recv_Data[cur_pos+1]==GPS_HEADER2) 
		{
			//printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~GPS_HEADER1 begin~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");	
			if(cur_pos!=0)
			{
				recv_Data.erase(recv_Data.begin(),recv_Data.begin()+cur_pos);		
			}
			cur_pos = 0; // reset cur_pos 	
			if(recv_Data.size()>=(frame_char_num + GPS_Header_ByNums + GPS_ID_ByNums + GPS_CHECK_ByNums)) // 2+1 = 3
			{
				//analyze one frame 
				msg_id = recv_Data[cur_pos + GPS_Header_ByNums + GPS_ID_ByNums-1]; //0x7e	
				for(int idx = 0;idx<GPS_Header_ByNums + GPS_ID_ByNums;idx++)
				{
					perframe_data[idx] = recv_Data[idx]; //check sum
				}

				recv_Data.erase(recv_Data.begin(),recv_Data.begin() + GPS_Header_ByNums + GPS_ID_ByNums);
				for(int i=0;i<(int)frame_char_num;i++)
				{
					perframe_data[i+GPS_Header_ByNums + GPS_ID_ByNums] = recv_Data.front();
					recv_Data.erase(recv_Data.begin());	
				}
				memcpy(&GNSS_SEND_FRAME,perframe_data + GPS_Header_ByNums + GPS_ID_ByNums,frame_char_num);
				char check_data = CheckSum(perframe_data, frame_char_num + GPS_Header_ByNums + GPS_ID_ByNums);
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

					Frame_Num++;
					lat_now = (double)GNSS_SEND_FRAME.lat*0.0000001;
					lon_now = (double)GNSS_SEND_FRAME.lon*0.0000001;
				//	NavInfo_To_File(&Sins_Output_Data_Sanchi);
					if(good_frame_dat_num%max_display_data_num==0)
					{
						float ratio = (float)good_frame_dat_num/sum_frame_dat_num;
						printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~gps information begin~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");	
						printf("good_num = %d bad_num = %d goode_ratio = %.8f\n",good_frame_dat_num,bad_frame_data_num,ratio);	
						printf("%d-%d-%d-%d-%d-%d-%d\n",GNSS_SEND_FRAME.utc.year,GNSS_SEND_FRAME.utc.month,GNSS_SEND_FRAME.utc.date,GNSS_SEND_FRAME.utc.hour,GNSS_SEND_FRAME.utc.min,GNSS_SEND_FRAME.utc.sec,GNSS_SEND_FRAME.utc.milsec);  
						printf("lat = %.8f\n",lat_now);   
						printf("lon = %.8f\n",lon_now);  
						printf("alt = %.6f\n",(float)GNSS_SEND_FRAME.alt*0.001); //m  
						printf("speed = %.6f m/s\n",(float)GNSS_SEND_FRAME.speed*0.001/3.6); // m/s  
						printf("forward_speed = %.6f m/s\n",(float)GNSS_SEND_FRAME.forward_speed*0.001/3.6); // m/s  
						printf("direction_angle = %.6f deg\n",(float)GNSS_SEND_FRAME.direction_angle*0.01); // deg   
						printf("velocity_angle = %.6f deg\n",(float)GNSS_SEND_FRAME.velocity_angle*0.01); // deg
						printf("e_vel = %.6f m/s\n",(float)GNSS_SEND_FRAME.east_velocity*0.001/3.6); // m/s  
						printf("n_vel = %.6f m/s\n",(float)GNSS_SEND_FRAME.north_velocity*0.001/3.6); // m/s  
						printf("u_vel = %.6f m/s\n",(float)GNSS_SEND_FRAME.diurnal_velocity*0.001/3.6); // m/s  
						printf("roll = %.6f deg\n",(float)GNSS_SEND_FRAME.roll*0.001); // deg
						printf("pitch = %.6f deg\n",(float)GNSS_SEND_FRAME.pitch*0.001); // deg  
						printf("yaw = %.6f deg\n",(float)GNSS_SEND_FRAME.yaw*0.001); // deg 
						printf("star = %d \n",(int)GNSS_SEND_FRAME.star);  
						printf("pdop = %.6f\n",(float)GNSS_SEND_FRAME.pdop*0.01);  
						printf("hdop = %.6f\n",(float)GNSS_SEND_FRAME.hdop*0.01);  
						printf("vdop = %.6f\n",(float)GNSS_SEND_FRAME.vdop*0.01);  
						printf("gnss_state = %d\n",GNSS_SEND_FRAME.gnss_state);  
						printf("ppscount = %d\n",GNSS_SEND_FRAME.ppscount);   
						printf("time_tick_pps = %.6f s\n",(float)GNSS_SEND_FRAME.time_tick_pps*0.000001);   // s
						printf("timestamp2 = %.6f s\n",add_second);    // s
						printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~gps information end~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");	
					}
					NavInfo_To_File(&GNSS_SEND_FRAME);

				}	
				else
				{
					bad_frame_data_num++;
				}
				memset(perframe_data, 0, GPS_Max_Frame_Nums);
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

//	printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~out of Analy_GPS_Frame_~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");	

}




/*
 *函数功能：从串口接收并解析数据
 *输入参数：
 *输出参数：接收到一帧数据标志位sss
 *备    注：一次接收单个字符
 *
 */
bool Analy_GPS_Data::Analy_GPS_Frame(char recv_Data,bool disp_flag)
{
	static char old_receive_data;
	char cur_receive_data;

	static int good_frame_num;
	static int bad_frame_num;
	static int frame_num;

	static int State = 1;
	static int cur_datasum;
	static int cur_data_num;
	int perframe_datasum = sizeof(GNSS_FRAME_Info);
	static char perframe_data[GPS_Max_Frame_Nums]; //防止内存泄漏!!

	int checksum_data = 0;
	char check_data = 0x00;
	bool framedata_flag = false;
	cur_receive_data = recv_Data;

	bool Find_Sins_Dev;
	char Sins_State;
	bool Sins_Frame_Flag;

	static int continue_bad_data = 0;
	int max_continue_bad_data = 50;

	switch (State)
	{
		case 1:
			{

				if ((unsigned char)old_receive_data == GPS_HEADER1 && (unsigned char)cur_receive_data ==GPS_HEADER2)
				{
					cur_datasum = GPS_Header_ByNums;
					State = 2;
				}
				break;
			};

		case 2:
			{
				cur_datasum++;
				perframe_data[cur_data_num++] = cur_receive_data;
				if (cur_datasum == ( GPS_Header_ByNums+ perframe_datasum)) 
				{
					State = 3;
				}
				break;
			};

			//case SINS_CheckSum:
		case 3:
			{
				frame_num++;
				cur_datasum++;	
				if (cur_datasum == ( GPS_Header_ByNums + perframe_datasum + GPS_CHECK_ByNums))
				{	
					//sum_check
					char check_data = CheckSum(perframe_data, cur_data_num);
					if(debug_flag)
					{
						printf("check_data = %d cur_receive_data = %d\n",check_data,cur_receive_data);
					}

					if(frame_num<=1)
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
					if(check_data ==cur_receive_data)
					{	
						framedata_flag = true;
						good_frame_num++;
						continue_bad_data = 0;
						float sucess_ratio = (float)good_frame_num/(float)frame_num;		
						memcpy(&GNSS_SEND_FRAME,perframe_data,perframe_datasum);//把接收到的信息转换成结构体
						lat_now = (double)GNSS_SEND_FRAME.lat*(double)0.0000001;
						lon_now = (double)GNSS_SEND_FRAME.lon*(double)0.0000001;
						/**********************************调试信息*****************************/
						if((frame_num%max_display_data_num==0)&&disp_flag)
						{  
							printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~GPS Information Begin~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");	
							printf("good_num = %d bad_num = %d goode_ratio = %.8f\n",good_frame_num,bad_frame_num,sucess_ratio);	
							printf("%d-%d-%d-%d-%d-%d-%d\n",GNSS_SEND_FRAME.utc.year,GNSS_SEND_FRAME.utc.month,GNSS_SEND_FRAME.utc.date,GNSS_SEND_FRAME.utc.hour,GNSS_SEND_FRAME.utc.min,GNSS_SEND_FRAME.utc.sec,GNSS_SEND_FRAME.utc.milsec);  
							printf("lat = %.8f\n",lat_now);   
							printf("lon = %.8f\n",lon_now);  
							printf("alt = %.6f\n",(float)GNSS_SEND_FRAME.alt*0.001); //m  
							printf("speed = %.6f m/s\n",(float)GNSS_SEND_FRAME.speed*0.001/3.6); // m/s  
							printf("forward_speed = %.6f m/s\n",(float)GNSS_SEND_FRAME.forward_speed*0.001/3.6); // m/s  
							printf("direction_angle = %.6f deg\n",(float)GNSS_SEND_FRAME.direction_angle*0.01); // deg   
							printf("velocity_angle = %.6f deg\n",(float)GNSS_SEND_FRAME.velocity_angle*0.01); // deg
							printf("e_vel = %.6f m/s\n",(float)GNSS_SEND_FRAME.east_velocity*0.001/3.6); // m/s  
							printf("n_vel = %.6f m/s\n",(float)GNSS_SEND_FRAME.north_velocity*0.001/3.6); // m/s  
							printf("u_vel = %.6f m/s\n",(float)GNSS_SEND_FRAME.diurnal_velocity*0.001/3.6); // m/s  
							printf("roll = %.6f deg\n",(float)GNSS_SEND_FRAME.roll*0.001); // deg
							printf("pitch = %.6f deg\n",(float)GNSS_SEND_FRAME.pitch*0.001); // deg  
							printf("yaw = %.6f deg\n",(float)GNSS_SEND_FRAME.yaw*0.001); // deg 
							printf("star = %d \n",(int)GNSS_SEND_FRAME.star);  
							printf("pdop = %.6f\n",(float)GNSS_SEND_FRAME.pdop*0.01);  
							printf("hdop = %.6f\n",(float)GNSS_SEND_FRAME.hdop*0.01);  
							printf("vdop = %.6f\n",(float)GNSS_SEND_FRAME.vdop*0.01);  
							printf("gnss_state = %d\n",GNSS_SEND_FRAME.gnss_state);  
							printf("ppscount = %d\n",GNSS_SEND_FRAME.ppscount);   
							printf("time_tick_pps = %.6f s\n",(float)GNSS_SEND_FRAME.time_tick_pps*0.000001);   // s
							printf("timestamp2 = %.6f s\n",add_second);    // s
							printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~GPS Information End~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");	
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
					}		

					if(continue_bad_data<=max_continue_bad_data)
					{
						NavInfo_To_File(&GNSS_SEND_FRAME,framedata_flag);
					}
					else
					{
						
					}	

					cur_datasum = 0;
					cur_data_num= 0;
					old_receive_data = 0;
					cur_receive_data = 0;
					perframe_datasum = 0;
					memset(perframe_data, 0, GPS_Max_Frame_Nums);
					State = 1;
					checksum_data = 0;
					check_data = 0x00;

				}  

				break;
			};

		default:
			{
				break;
			};

	}

	old_receive_data = cur_receive_data;
	return framedata_flag;

}




char Analy_GPS_Data::CheckSum( char * nav_data, unsigned int data_len)
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



void Analy_GPS_Data::NavInfo_To_File(GPS_Output_Info * data)
{
	static int fflush_num;
	fflush_num++;

	time_t timep;
	struct tm *p;
	time(&timep);
	p=gmtime(&timep);

	//output time
	fprintf(gps_filestream,"%04d-",1900+p->tm_year);
	fprintf(gps_filestream,"%02d-",1+p->tm_mon);
	fprintf(gps_filestream,"%02d-",p->tm_mday);
	fprintf(gps_filestream,"%02d-",p->tm_hour);
	fprintf(gps_filestream,"%02d-",p->tm_min);
	fprintf(gps_filestream,"%02d    ",p->tm_sec);

	fprintf(gps_filestream,"%.8f ",lat_now);   
	fprintf(gps_filestream,"%.8f ",lon_now);  
	fprintf(gps_filestream,"%.6f ",(float)data->alt*0.001); //m  
	fprintf(gps_filestream,"%.6f ",(float)data->speed*0.001/3.6); // m/s  
	fprintf(gps_filestream,"%d ",(int)data->star);  
	fprintf(gps_filestream,"%.6f ",(float)data->pdop*0.1);  
	fprintf(gps_filestream,"%.6f ",(float)data->hdop*0.1);  
  fprintf(gps_filestream,"%.6f ",(float)data->vdop*0.1);  

	fprintf(gps_filestream,"%d ",GPS_Output_Data.gnss_state);  
	fprintf(gps_filestream,"%.6f ",(float)data->timestamp*0.000001);   // s
	fprintf(gps_filestream,"%.6f ",add_second);   // 
	fprintf(gps_filestream,"\n");

	if(fflush_num==5)
	{
		fflush_num=0;
		fflush(gps_filestream);
	}					

}



void Analy_GPS_Data::NavInfo_To_File(GNSS_FRAME_Info * data)
{
	static int fflush_num;
	fflush_num++;

	time_t timep;
	struct tm *p;
	time(&timep);
	p=gmtime(&timep);

	//output time
	fprintf(gps_filestream,"%04d-",1900+p->tm_year);
	fprintf(gps_filestream,"%02d-",1+p->tm_mon);
	fprintf(gps_filestream,"%02d-",p->tm_mday);
	fprintf(gps_filestream,"%02d-",p->tm_hour);
	fprintf(gps_filestream,"%02d-",p->tm_min);
	fprintf(gps_filestream,"%02d    ",p->tm_sec);
	fprintf(gps_filestream,"%d-%02d-%02d-%02d-%02d-%02d-%02d ",data->utc.year,data->utc.month,data->utc.date,data->utc.hour,data->utc.min,data->utc.sec,data->utc.milsec);  
	fprintf(gps_filestream,"%.8f ",lat_now);   
	fprintf(gps_filestream,"%.8f ",lon_now);  
	fprintf(gps_filestream,"%.6f ",(float)data->alt*0.001); //m  
	fprintf(gps_filestream,"%.6f ",(float)data->speed*0.001/3.6); // m/s  
	fprintf(gps_filestream,"%.6f ",(float)data->forward_speed*0.001/3.6); // m/s  
	fprintf(gps_filestream,"%.6f ",(float)data->direction_angle*0.01); // deg   
	fprintf(gps_filestream,"%.6f ",(float)data->velocity_angle*0.01); // deg
	fprintf(gps_filestream,"%.6f ",(float)data->east_velocity*0.001/3.6); // m/s  
	fprintf(gps_filestream,"%.6f ",(float)data->north_velocity*0.001/3.6); // m/s  
	fprintf(gps_filestream,"%.6f ",(float)data->diurnal_velocity*0.001/3.6); // m/s  
	fprintf(gps_filestream,"%.6f ",(float)data->roll*0.001); // deg
	fprintf(gps_filestream,"%.6f ",(float)data->pitch*0.001); // deg  
	fprintf(gps_filestream,"%.6f ",(float)data->yaw*0.001); // deg 
	fprintf(gps_filestream,"%d ",(int)data->star);  
	fprintf(gps_filestream,"%.6f ",(float)data->pdop*0.01);  
	fprintf(gps_filestream,"%.6f ",(float)data->hdop*0.01);  
  fprintf(gps_filestream,"%.6f ",(float)data->vdop*0.01);  
	fprintf(gps_filestream,"%d ",data->gnss_state);  
	fprintf(gps_filestream,"%d ",data->ppscount);   
	fprintf(gps_filestream,"%.6f ",(float)data->time_tick_pps*0.000001);   // s
	fprintf(gps_filestream,"%.6f ",add_second);   // 
	fprintf(gps_filestream,"\n");

	if(fflush_num==5)
	{
		fflush_num=0;
		fflush(gps_filestream);
	}					

}




void Analy_GPS_Data::NavInfo_To_File(GNSS_FRAME_Info * data,bool valid_flag)
{
	static int fflush_num;
	fflush_num++;

	time_t timep;
	struct tm *p;
	time(&timep);
	p=gmtime(&timep);

	//output time
	fprintf(gps_filestream,"%04d-",1900+p->tm_year);
	fprintf(gps_filestream,"%02d-",1+p->tm_mon);
	fprintf(gps_filestream,"%02d-",p->tm_mday);
	fprintf(gps_filestream,"%02d-",p->tm_hour);
	fprintf(gps_filestream,"%02d-",p->tm_min);
	fprintf(gps_filestream,"%02d    ",p->tm_sec);
	fprintf(gps_filestream,"%d-%02d-%02d-%02d-%02d-%02d-%02d ",data->utc.year,data->utc.month,data->utc.date,data->utc.hour,data->utc.min,data->utc.sec,data->utc.milsec);  
	fprintf(gps_filestream,"%.8f ",lat_now);   
	fprintf(gps_filestream,"%.8f ",lon_now);  
	fprintf(gps_filestream,"%.6f ",(float)data->alt*0.001); //m  
	fprintf(gps_filestream,"%.6f ",(float)data->speed*0.001/3.6); // m/s  
	fprintf(gps_filestream,"%.6f ",(float)data->forward_speed*0.001/3.6); // m/s  
	fprintf(gps_filestream,"%.6f ",(float)data->direction_angle*0.01); // deg   
	fprintf(gps_filestream,"%.6f ",(float)data->velocity_angle*0.01); // deg
	fprintf(gps_filestream,"%.6f ",(float)data->east_velocity*0.001/3.6); // m/s  
	fprintf(gps_filestream,"%.6f ",(float)data->north_velocity*0.001/3.6); // m/s  
	fprintf(gps_filestream,"%.6f ",(float)data->diurnal_velocity*0.001/3.6); // m/s  
	fprintf(gps_filestream,"%.6f ",(float)data->roll*0.001); // deg
	fprintf(gps_filestream,"%.6f ",(float)data->pitch*0.001); // deg  
	fprintf(gps_filestream,"%.6f ",(float)data->yaw*0.001); // deg 
	fprintf(gps_filestream,"%d ",(int)data->star);  
	fprintf(gps_filestream,"%.6f ",(float)data->pdop*0.01);  
	fprintf(gps_filestream,"%.6f ",(float)data->hdop*0.01);  
  fprintf(gps_filestream,"%.6f ",(float)data->vdop*0.01);  
	fprintf(gps_filestream,"%d ",data->gnss_state);  
	fprintf(gps_filestream,"%d ",data->ppscount);   
	fprintf(gps_filestream,"%.6f ",(float)data->time_tick_pps*0.000001);   // s
	fprintf(gps_filestream,"%.6f ",add_second);  
	fprintf(gps_filestream,"%d ",valid_flag);  
	fprintf(gps_filestream,"\n");

	if(fflush_num==5)
	{
		fflush_num=0;
		fflush(gps_filestream);
	}					

}


