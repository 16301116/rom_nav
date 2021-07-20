#include <stdlib.h>
#include <stdio.h>
#include <mutex>
#include "sensor/Analy_Sins_Data.h"


using namespace gdface;
using namespace chrono;
using namespace std;


Analy_Sins_Data::Analy_Sins_Data()
{
	Init_Data();
	// file name
	time_t currtime =time(NULL);
	tm *sins_p =localtime(&currtime);
	sprintf(sinsdata_file_name,"../data/output_data/sins_data%04d%02d%02d%02d%02d%02d.txt",1900+sins_p->tm_year,1+sins_p->tm_mon,sins_p->tm_mday,sins_p->tm_hour,sins_p->tm_min,sins_p->tm_sec);	
	sins_filestream = fopen(sinsdata_file_name,"w+");
	if (sins_filestream==NULL)
  	{
		printf("can't create file to save sins data!\n");
	}
	else
	{
		printf("successful create file to save sins data!\n");
	}
/*
	sprintf(error_sinsdata_file_name,"../output_data/data/error-sins-data%04d%02d%02d%02d%02d%02d.txt",1900+sins_p->tm_year,1+sins_p->tm_mon,sins_p->tm_mday,sins_p->tm_hour,sins_p->tm_min,sins_p->tm_sec);	
	error_sins_filestream = fopen(error_sinsdata_file_name,"w+");
	if (error_sins_filestream==NULL)
  	{
		printf("can't create file to save error sins data!\n");
	}
	else
	{
		printf("successful create file to save error sins data!\n");
	}
	*/
}

void Analy_Sins_Data::Init_Data(void)
{
		memset(&Sins_Output_Data_Sanchi,0,sizeof(Sins_Output_Data_Sanchi));
		memset(&Old_Sins_Output_Data_Sanchi,0,sizeof(Old_Sins_Output_Data_Sanchi));
		memset(&Sins_Output_Data_Nvidia,0,sizeof(Sins_Output_Data_Nvidia));
		memset(&Old_Sins_Output_Data_Nvidia,0,sizeof(Old_Sins_Output_Data_Nvidia));
}

Analy_Sins_Data::Analy_Sins_Data(char * dev_name,int baudrate_sins,string parity,bool init_flag)		
{
	// file name
	time_t currtime =time(NULL);
	tm *sins_p =localtime(&currtime);
	sprintf(sinsdata_file_name,"../data/output_data/sins_data%04d%02d%02d%02d%02d%02d.txt",1900+sins_p->tm_year,1+sins_p->tm_mon,sins_p->tm_mday,sins_p->tm_hour,sins_p->tm_min,sins_p->tm_sec);
	sins_filestream = fopen(sinsdata_file_name,"w+");
	if (sins_filestream==NULL)
  	{
		printf("can't create file to save sins data!\n");
	}
	else
	{
		printf("successful create file to save sins data!\n");
	}

	m_fd_sins = -1;
	m_fd_sins = serial_sins_data.connectTTY(dev_name ,baudrate_sins,parity);
	if(m_fd_sins>0)
	{
		if(init_flag)
		{
			std::thread sinsinit_id(SinsInit_Thread,this);
			sinsinit_id.join(); // block here
		}
		std::thread sinsrececmd_id(SinsReceCmd_Thread,this);
		sinsrececmd_id.detach();	
	}

}


Analy_Sins_Data::~Analy_Sins_Data()		
{
	fclose(sins_filestream);
}


void Analy_Sins_Data::SinsInit_Thread(Analy_Sins_Data * ptr) 
{
	// beijing position
	float Longitude = (float)116.1234;
	float Latitude = (float)40.1234;
	short int alt = (short int)12.3;
	ptr->sins_mtx.lock();
	ptr->Nav_Init(Longitude,Latitude,alt);
	ptr->sins_mtx.unlock();

}


// read sins data 
void Analy_Sins_Data::SinsReceCmd_Thread(Analy_Sins_Data * ptr)
{

	int sins_read_nums = 150;
	int sins_frame_num = 0;
	static char old_char = 0x00; 
	char cur_char = 0x00;
	while(1)
	{	
		ptr->sins_mtx.lock();		
		int sinsdata_nums = ptr->serial_sins_data.readData(ptr->m_fd_sins , sins_read_nums );
		ptr->sins_mtx.unlock();
		if(ptr->debug_flag)
		{
			printf("sinsdata_nums = %d\n",sinsdata_nums);
		}
		if(sinsdata_nums>0)
		{
			//ptr->start_time = system_clock::now();
			for(int i = 0; i< sinsdata_nums; i++)
			{
				char data;
				data = ptr->serial_sins_data.m_buf[i];
				bool sins_frame_flag = ptr->Analy_Sins_Frame(data,sins_frame_num);
				if(sins_frame_flag&&ptr->Sins_Output_Data_Sanchi.state==5)
				{

				}
			}

			//ptr->end_time = system_clock::now();
			//auto duration = duration_cast<microseconds>(ptr->end_time - ptr->start_time);
			//ptr->add_second = double(duration.count()) * microseconds::period::num / microseconds::period::den;
			//printf("sinsdata_nums = %d add_second = %f\n",sinsdata_nums,ptr->add_second); 

		}

		usleep(8000);  // 8ms

	}

}



/*
 *函数功能：从串口接收并解析数据
 *输入参数：
 *输出参数：接收到一帧数据标志位
 *备    注：一次接收单个字符
 *
 * cur_pos : initial data = 0 or 1
 */
void Analy_Sins_Data::Analy_Sins_Frame_Sanchi( std::vector<char> & recv_Data,int  & Frame_Num,std::vector<Sins_Output_Info_Sanchi> & output_data)
{
//	printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~come in Analy_Sins_Frame_Sanchi~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
	char counter_num = 0;
	Frame_Num = 0;
	static int bad_frame_data_num = 0;
	static int good_frame_dat_num = 0;
	static int sum_frame_dat_num = 0;
	int frame_char_num = sizeof(Sins_Output_Info_Sanchi);
	char perframe_data[Max_Perframe_Nums] = {0}; //防止内存泄漏!!
	if(recv_Data.size()<(frame_char_num+Header_ByNums + State_ByNums + Tail_ByNums)) // 52:change here ==========one frame data num :38============= 
	{
		return;
	}
	int max_data_num = recv_Data.size();
	int cur_pos = 0;
	bool over_flag = false;

	while(cur_pos<recv_Data.size())   //attention
 	{
		if(recv_Data[cur_pos]==SANCHI_SINS_HEADER1&&recv_Data[cur_pos+1]==SANCHI_SINS_HEADER2) 
		{
			if(cur_pos!=0)
			{
				recv_Data.erase(recv_Data.begin(),recv_Data.begin()+cur_pos);		
			}
			cur_pos = 0; // reset cur_pos 	
			if(recv_Data.size()>=(frame_char_num + Header_ByNums + Counter_ByNums + Tail_ByNums)) // 2+1 = 3
			{
				//analyze one frame 
				perframe_data[0] = recv_Data[cur_pos+Header_ByNums+Counter_ByNums-1]; //0x7e	
				recv_Data.erase(recv_Data.begin(),recv_Data.begin()+Header_ByNums+Counter_ByNums);
				for(int i=0;i<(int)frame_char_num;i++)
				{
					perframe_data[i+1] = recv_Data.front();
					recv_Data.erase(recv_Data.begin());	
				}
				memcpy(&Sins_Output_Data_Sanchi,perframe_data+Counter_ByNums,frame_char_num);
				char check_data = CheckSum(perframe_data, frame_char_num+Counter_ByNums);
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
					NavInfo_To_File(&Sins_Output_Data_Sanchi);
					if(good_frame_dat_num%200==0)
					{
						float ratio = (float)good_frame_dat_num/sum_frame_dat_num;
						printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~sins information begin~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");	
						printf("good_num = %d bad_num = %d goode_ratio = %.8f\n",good_frame_dat_num,bad_frame_data_num,ratio);	
						printf("counter = %d\n",Sins_Output_Data_Sanchi.counter); //degree
						printf("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@state = %d\n",(int)Sins_Output_Data_Sanchi.state);  // 0.01 degree/LSB
						printf("nav_mode = %d\n",(int)Sins_Output_Data_Sanchi.nav_mode);  // 0.01 degree/LSB
						printf("angle1 = %.6f deg\n",(float)Sins_Output_Data_Sanchi.angle[0]*0.000001);  // 0.000001deg/LSB
						printf("angle2 = %.6f deg\n",(float)Sins_Output_Data_Sanchi.angle[1]*0.000001);   // 0.000001deg/LSB
						printf("angle3 = %.6f deg\n",(float)Sins_Output_Data_Sanchi.angle[2]*0.000001);    // 0.000001deg/LSB
						printf("fusion_vel_1 = %.6f m/s\n",(float)Sins_Output_Data_Sanchi.fusion_vel[0]*0.001); //0.001m/s/LSB
						printf("fusion_vel_2 = %.6f m/s\n",(float)Sins_Output_Data_Sanchi.fusion_vel[1]*0.001); //0.001m/s/LSB
						printf("fusion_vel_3 = %.6f m/s\n",(float)Sins_Output_Data_Sanchi.fusion_vel[2]*0.001); //0.001m/s/LSB
						printf("fusion_lon = %.6f \n",(float)Sins_Output_Data_Sanchi.fusion_lon*0.000001);  //0.000001deg/LSB
						printf("fusion_lat = %.6f \n",(float)Sins_Output_Data_Sanchi.fusion_lat*0.000001);  //0.000001deg/LSB
						printf("fusion_alt = %.6f m\n",(float)Sins_Output_Data_Sanchi.fusion_alt*0.001);  //0.001m/LSB
						printf("gyro_data1 = %.6f deg/h\n",(float)Sins_Output_Data_Sanchi.gyro_data[0]*0.001);  ////0.001deg/h/LSB
						printf("gyro_data2 = %.6f deg/h\n",(float)Sins_Output_Data_Sanchi.gyro_data[1]*0.001);  ////0.001deg/h/LSB
						printf("gyro_data3 = %.6f deg/h\n",(float)Sins_Output_Data_Sanchi.gyro_data[2]*0.001);  ////0.001deg/h/LSB
						printf("acce_data1 = %.6f m/s/s\n",(float)Sins_Output_Data_Sanchi.acce_data[0]*0.00001);  //0.00001m/s/s/LSB
						printf("acce_data2 = %.6f m/s/s\n",(float)Sins_Output_Data_Sanchi.acce_data[1]*0.00001);  //0.00001m/s/s/LSB
						printf("acce_data3 = %.6f m/s/s\n",(float)Sins_Output_Data_Sanchi.acce_data[2]*0.00001);  //0.00001m/s/s/LSB
						printf("gyro_tmp1 = %.6f \n",(float)Sins_Output_Data_Sanchi.gyro_tmp[0]*0.0625);   //0.0625/LSB
						printf("gyro_tmp2 = %.6f \n",(float)Sins_Output_Data_Sanchi.gyro_tmp[1]*0.0625);   //0.0625/LSB
						printf("gyro_tmp3 = %.6f \n",(float)Sins_Output_Data_Sanchi.gyro_tmp[2]*0.0625);   //0.0625/LSB
						printf("acce_tmp1 = %.6f \n",(float)Sins_Output_Data_Sanchi.acce_tmp[0]*0.0625);   //0.0625/LSB
						printf("acce_tmp2 = %.6f \n",(float)Sins_Output_Data_Sanchi.acce_tmp[1]*0.0625);   //0.0625/LSB
						printf("acce_tmp3 = %.6f \n",(float)Sins_Output_Data_Sanchi.acce_tmp[2]*0.0625);   //0.0625/LSB
						printf("dvl_vel1 = %.6f m/s\n",(float)Sins_Output_Data_Sanchi.dvl_vel[0]*0.001);   //0.001m/s/LSB
						printf("dvl_vel2 = %.6f m/s\n",(float)Sins_Output_Data_Sanchi.dvl_vel[1]*0.001);   //0.001m/s/LSB
						printf("dvl_vel3 = %.6f m/s\n",(float)Sins_Output_Data_Sanchi.dvl_vel[2]*0.001);   //0.001m/s/LSB
						printf("dvl_flag = %d\n",(int)Sins_Output_Data_Sanchi.dvl_flag);   //0.001m/s/LSB
						//printf("timestamp1 = %.6f s\n",(float)Sins_Output_Data_Sanchi.timestamp*0.000001);   // s
						printf("timestamp2 = %.6f s\n",add_second);   // 
						printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~sins information end~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");	
					}

				}	
				else
				{
					bad_frame_data_num++;
					//Nav_Error_Info_To_File(perframe_data, frame_char_num+Counter_ByNums,receive_checksum,check_data);
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
 *输出参数：接收到一帧数据标志位
 *备    注：一次接收单个字符
 *
 */
bool Analy_Sins_Data::Analy_Sins_Frame( char Receive_Data,int  & Frame_Num)
{
	static int lost_frame_num = 0;
	static int continue_bad_data = 0;
	int max_continue_bad_data = 1000;

	static char old_receive_data;
	static int header_sum_num;
	char cur_receive_data;

	static int good_frame_num;
	static int bad_frame_data_num = 0;

	static int cur_datasum;
	static int cur_data_num;
	int perframe_datasum = sizeof(Sins_Output_Data_Sanchi);

	static char perframe_data[Max_Perframe_Nums]; // 防止内存泄漏!!

	int checksum_data = 0;
	char check_data = 0x00;

	static bool framedata_flag = true;
	cur_receive_data = Receive_Data;

	char Sins_State;
	bool Sins_Frame_Flag;

	switch (Sins_Data_State)
	{
		case 1:
			{
				if ((unsigned char)old_receive_data == SANCHI_SINS_HEADER1 && (unsigned char)cur_receive_data ==SANCHI_SINS_HEADER2)
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

					if(debug_flag)
					{
						printf("=====================SANCHI_SINS_HEADER1==================\n");
						printf("header_sum_num = %d\n",header_sum_num);
					}
					header_sum_num++;					
					cur_datasum = Header_ByNums;
					Sins_Data_State = 2;
				}
				break;
			};

		case 2:
			{
				cur_datasum++;
				if (cur_datasum == ( Header_ByNums + Frame_ByNums ))
				{	
					perframe_data[cur_data_num++] = cur_receive_data;
					Sins_Data_State = 3;
				}
				break;
			};

		case 3:
			{
				cur_datasum++ ;
				perframe_data[cur_data_num++] = cur_receive_data;
				if (cur_datasum == ( Header_ByNums + Frame_ByNums + perframe_datasum))
				{
					Sins_Data_State = 4;
				}
				break;
			};

		case 4:
			{
				cur_datasum++;	
				if (cur_datasum == ( Header_ByNums + Frame_ByNums + perframe_datasum + CheckSum_ByNums))
				{	
					char check_data = CheckSum(perframe_data, cur_data_num);
					if(debug_flag)
					{
						printf("check_data = %02x\n",check_data);
						printf("cur_receive_data = %02x\n",cur_receive_data);
					}
					if (check_data ==cur_receive_data)
					{
						framedata_flag =true;
						continue_bad_data = 0;
						good_frame_num++;
						float sucess_ratio = (float)good_frame_num/frame_header_num*100;	
						memcpy(&Sins_Output_Data_Sanchi,perframe_data+Frame_ByNums,perframe_datasum);//把接收到的信息转换成结构体
						if(frame_header_num%200==0&&display_data_flag)
						{  
							printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~sins information begin~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");	
							printf("good_num = %d bad_data = %d goode_ratio = %.8f\n",good_frame_num,bad_frame_data_num,sucess_ratio);	
							printf("counter = %d\n",Sins_Output_Data_Sanchi.counter); //degree
							printf("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@state = %d\n",(int)Sins_Output_Data_Sanchi.state);  // 0.01 degree/LSB
							printf("nav_mode = %d\n",(int)Sins_Output_Data_Sanchi.nav_mode);  // 0.01 degree/LSB
							printf("angle1 = %.6f deg\n",(float)Sins_Output_Data_Sanchi.angle[0]*0.000001);  // 0.000001deg/LSB
							printf("angle2 = %.6f deg\n",(float)Sins_Output_Data_Sanchi.angle[1]*0.000001);   // 0.000001deg/LSB
							printf("angle3 = %.6f deg\n",(float)Sins_Output_Data_Sanchi.angle[2]*0.000001);    // 0.000001deg/LSB
							printf("fusion_vel_1 = %.6f m/s\n",(float)Sins_Output_Data_Sanchi.fusion_vel[0]*0.001); //0.001m/s/LSB
							printf("fusion_vel_2 = %.6f m/s\n",(float)Sins_Output_Data_Sanchi.fusion_vel[1]*0.001); //0.001m/s/LSB
							printf("fusion_vel_3 = %.6f m/s\n",(float)Sins_Output_Data_Sanchi.fusion_vel[2]*0.001); //0.001m/s/LSB
							printf("fusion_lon = %.6f \n",(float)Sins_Output_Data_Sanchi.fusion_lon*0.000001);  //0.000001deg/LSB
							printf("fusion_lat = %.6f \n",(float)Sins_Output_Data_Sanchi.fusion_lat*0.000001);  //0.000001deg/LSB
							printf("fusion_alt = %.6f m\n",(float)Sins_Output_Data_Sanchi.fusion_alt*0.001);  //0.001m/LSB
							printf("gyro_data1 = %.6f deg/h\n",(float)Sins_Output_Data_Sanchi.gyro_data[0]*0.001);  ////0.001deg/h/LSB
							printf("gyro_data2 = %.6f deg/h\n",(float)Sins_Output_Data_Sanchi.gyro_data[1]*0.001);  ////0.001deg/h/LSB
							printf("gyro_data3 = %.6f deg/h\n",(float)Sins_Output_Data_Sanchi.gyro_data[2]*0.001);  ////0.001deg/h/LSB
							printf("acce_data1 = %.6f m/s/s\n",(float)Sins_Output_Data_Sanchi.acce_data[0]*0.00001);  //0.00001m/s/s/LSB
							printf("acce_data2 = %.6f m/s/s\n",(float)Sins_Output_Data_Sanchi.acce_data[1]*0.00001);  //0.00001m/s/s/LSB
							printf("acce_data3 = %.6f m/s/s\n",(float)Sins_Output_Data_Sanchi.acce_data[2]*0.00001);  //0.00001m/s/s/LSB
							printf("gyro_tmp1 = %.6f \n",(float)Sins_Output_Data_Sanchi.gyro_tmp[0]*0.0625);   //0.0625/LSB
							printf("gyro_tmp2 = %.6f \n",(float)Sins_Output_Data_Sanchi.gyro_tmp[1]*0.0625);   //0.0625/LSB
							printf("gyro_tmp3 = %.6f \n",(float)Sins_Output_Data_Sanchi.gyro_tmp[2]*0.0625);   //0.0625/LSB
							printf("acce_tmp1 = %.6f \n",(float)Sins_Output_Data_Sanchi.acce_tmp[0]*0.0625);   //0.0625/LSB
							printf("acce_tmp2 = %.6f \n",(float)Sins_Output_Data_Sanchi.acce_tmp[1]*0.0625);   //0.0625/LSB
							printf("acce_tmp3 = %.6f \n",(float)Sins_Output_Data_Sanchi.acce_tmp[2]*0.0625);   //0.0625/LSB
							printf("dvl_vel1 = %.6f m/s\n",(float)Sins_Output_Data_Sanchi.dvl_vel[0]*0.001);   //0.001m/s/LSB
							printf("dvl_vel2 = %.6f m/s\n",(float)Sins_Output_Data_Sanchi.dvl_vel[1]*0.001);   //0.001m/s/LSB
							printf("dvl_vel3 = %.6f m/s\n",(float)Sins_Output_Data_Sanchi.dvl_vel[2]*0.001);   //0.001m/s/LSB
							printf("dvl_flag = %d\n",(int)Sins_Output_Data_Sanchi.dvl_flag);   //0.001m/s/LSB
							printf("timestamp1 = %.6f s\n",(float)Sins_Output_Data_Sanchi.timestamp*0.000001);   // s
							printf("timestamp2 = %.6f s\n",add_second);   
							printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~sins information end~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");	
						}
						// save data 
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
						bad_frame_data_num++;

						int  real_gap_time_us = (int)((add_second - old_add_second)*1000000); //us
						int real_gap_time_ms = (int)(real_gap_time_us/1000);
						if(real_gap_time_ms>=5&&real_gap_time_ms<=15)
						{
							real_gap_time_ms = 10;
						}
						if(header_sum_num>1)
						{
								Sins_Output_Data_Sanchi.timestamp = old_timestamp + real_gap_time_us; // calculate time ms
								Sins_Output_Data_Sanchi.counter = old_counter + real_gap_time_ms; // calculate time us
						}
						
					}
					if(continue_bad_data<=max_continue_bad_data)
					{
							NavInfo_To_File(&Sins_Output_Data_Sanchi,framedata_flag);
					}
					else
					{

					}	
					old_timestamp = Sins_Output_Data_Sanchi.timestamp;
					old_add_second = add_second;
					old_counter = Sins_Output_Data_Sanchi.counter;

					cur_datasum = 0;
					cur_data_num= 0;
					old_receive_data = 0;
					cur_receive_data = 0;
					memset(perframe_data, 0, Max_Perframe_Nums);
					Sins_Data_State = 1;
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
	Frame_Num = good_frame_num;
	return framedata_flag;

}




/*
 *函数功能：从串口接收并解析数据
 *输入参数：
 *输出参数：接收到一帧数据标志位
 *备    注：一次接收单个字符
 *
 */
bool Analy_Sins_Data::Analy_Sins_Frame( char Receive_Data)
{

	static char old_receive_data;
	char cur_receive_data;
	static int cur_datasum;
	static int cur_data_num;
	int perframe_datasum = sizeof(Sins_Output_Data_Sanchi);

	static char perframe_data[Max_Perframe_Nums]; // 防止内存泄漏!!
	cur_receive_data = Receive_Data;

	valid_frame_flag = false;
	recv_frame_flag = false;

	switch (Sins_Data_State)
	{
		case header_state:
			{
				if ((unsigned char)old_receive_data == SANCHI_SINS_HEADER1 && (unsigned char)cur_receive_data ==SANCHI_SINS_HEADER2)
				{
					frame_header_num++;
					if(frame_header_num<=1)
					{
						start_time = system_clock::now();
						Sins_Output_Data_Nvidia.timestamp_nvidia = 0.0;
					}
					else
					{
						end_time = system_clock::now();
						auto duration = duration_cast<microseconds>(end_time - start_time);
						Sins_Output_Data_Nvidia.timestamp_nvidia = double(duration.count()) * microseconds::period::num / microseconds::period::den;
					}			
					cur_datasum = Header_ByNums;
					Sins_Data_State = data_num_state;
				}
				break;
			};

		case data_num_state:
			{
				cur_datasum++;
				if (cur_datasum == ( Header_ByNums + Frame_ByNums ))
				{	
					perframe_data[cur_data_num++] = cur_receive_data;
					Sins_Data_State = load_data_state;
				}
				break;
			};

		case load_data_state:
			{
				cur_datasum++ ;
				perframe_data[cur_data_num++] = cur_receive_data;
				if (cur_datasum == ( Header_ByNums + Frame_ByNums + perframe_datasum))
				{
					Sins_Data_State = check_state;
				}
				break;
			};

		case check_state:
			{
				cur_datasum++;	
				if (cur_datasum == ( Header_ByNums + Frame_ByNums + perframe_datasum + CheckSum_ByNums))
				{	
					recv_frame_flag = true;
					char check_data = CheckSum(perframe_data, cur_data_num);
					if(debug_flag)
					{
						printf("check_data = %02x\n",check_data);
						printf("cur_receive_data = %02x\n",cur_receive_data);
					}
					if (check_data == cur_receive_data)
					{
						valid_frame_flag = true;
						frame_invaild_num++;
						memcpy(&Sins_Output_Data_Nvidia.sins_data,perframe_data+Frame_ByNums,perframe_datasum);//把接收到的信息转换成结构体
					}
					else
					{

					}
					cur_datasum = 0;
					cur_data_num= 0;
					memset(perframe_data, 0, Max_Perframe_Nums);
					Sins_Data_State = header_state;
				}  

				break;
			};

		default:
		{
			cur_datasum = 0;
			cur_data_num= 0;
			memset(perframe_data, 0, Max_Perframe_Nums);
			Sins_Data_State = header_state;
			break;
		};
	}

	old_receive_data = cur_receive_data;
	return recv_frame_flag;

}



bool Analy_Sins_Data::Sins_Postprocess(char Receive_Data,bool disp_flag,bool add_check_lost_flag)
{
	bool recv_frame_flag = Analy_Sins_Frame(Receive_Data);
	if(recv_frame_flag&&(frame_invaild_num>=1))
	{
		if(frame_invaild_num==1)
		{
			Old_Sins_Output_Data_Nvidia = Sins_Output_Data_Nvidia;
		}
		gap_counter_ms = Sins_Output_Data_Nvidia.sins_data.counter - Old_Sins_Output_Data_Nvidia.sins_data.counter;  // ms
		gap_nvidia_s = Sins_Output_Data_Nvidia.timestamp_nvidia - Old_Sins_Output_Data_Nvidia.timestamp_nvidia;  // s
		gap_nvidia_ms = (int)(gap_nvidia_s*1000);
		gap_nvidia_us = (int)(gap_nvidia_s*1000000);

	//	printf("valid_frame_flag = %d\n",valid_frame_flag);
	//	printf("gap_counter_ms = %d\n",gap_counter_ms);

		if(valid_frame_flag&&(gap_counter_ms<gap_time_ms_max)&&(gap_counter_ms>gap_time_ms_min)) // check bit is right
		{
			//printf("if(valid_frame_flag&&(gap_counter_ms<gap_time_ms_max)&&(gap_counter_ms>gap_time_ms_min))\n");
			frame_data_state = check_right_state;
			NavInfo_To_File(&Sins_Output_Data_Nvidia);
		}
		else if(valid_frame_flag&&(gap_counter_ms>gap_time_ms_max)&&(add_check_lost_flag)) //lost data
		{
			float mul_num = (float)gap_counter_ms/(float)gap_time_ms_mean;
			cur_lost_frame_num = (round)(mul_num)-1; // attention:one data is valid here
			if((cur_lost_frame_num<=add_lost_frame_num_max)&&(cur_lost_frame_num>=1))
			{
				frame_lost_num+=cur_lost_frame_num;
				for(int i=1;i<=cur_lost_frame_num;i++)
				{
					Sins_Output_Info_Nvidia  Lost_Sins_Output_Data_Nvidia;
					Lost_Sins_Output_Data_Nvidia = Old_Sins_Output_Data_Nvidia;
					int gap_time_us_mean = gap_time_ms_mean*1000;
					double gap_time_s_mean = (double)gap_time_ms_mean/(double)1000;					
					Lost_Sins_Output_Data_Nvidia.sins_data.counter = Old_Sins_Output_Data_Nvidia.sins_data.counter + i*gap_time_ms_mean;
					Lost_Sins_Output_Data_Nvidia.sins_data.timestamp = Old_Sins_Output_Data_Nvidia.sins_data.timestamp + i*gap_time_us_mean;
					Lost_Sins_Output_Data_Nvidia.timestamp_nvidia = Old_Sins_Output_Data_Nvidia.timestamp_nvidia + (double)i*gap_time_s_mean;
					frame_data_state = lost_frame_state; // data is lost
					NavInfo_To_File(&Lost_Sins_Output_Data_Nvidia);
				}
				// have valid data here
				frame_data_state = check_right_state;; // data is invalid
				NavInfo_To_File(&Sins_Output_Data_Nvidia);
			}
		}
		else if((!valid_frame_flag)&&(gap_nvidia_ms>gap_time_ms_max)&&(add_check_lost_flag)) // checksum data is wrong
		{
			frame_check_error_num++;
			Sins_Output_Info_Nvidia  Void_Sins_Output_Data_Nvidia;
			Void_Sins_Output_Data_Nvidia = Sins_Output_Data_Nvidia;
			Void_Sins_Output_Data_Nvidia.sins_data.counter = Sins_Output_Data_Nvidia.sins_data.counter + gap_nvidia_ms;
			Void_Sins_Output_Data_Nvidia.sins_data.timestamp = Sins_Output_Data_Nvidia.sins_data.timestamp + gap_nvidia_us;
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
			printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~sins information begin~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");	
			printf("num = %d invalid_ratio = %.8f lost_ratio = %.8f check_error_ratio = %.8f\n",frame_truth_num,invalid_ratio,lost_ratio,check_error_ratio);	
			printf("invalid_num = %d lost_num = %d check_error_num = %d\n",frame_invaild_num,frame_lost_num,frame_check_error_num);	
			printf("counter = %d\n",Sins_Output_Data_Nvidia.sins_data.counter); //degree
			printf("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@STATE = %d\n",(int)Sins_Output_Data_Nvidia.sins_data.state);  // 0.01 degree/LSB
			printf("nav_mode = %d\n",(int)Sins_Output_Data_Nvidia.sins_data.nav_mode);  // 0.01 degree/LSB
			printf("angle1 = %.6f deg\n",(float)Sins_Output_Data_Nvidia.sins_data.angle[0]*0.000001);  // 0.000001deg/LSB
			printf("angle2 = %.6f deg\n",(float)Sins_Output_Data_Nvidia.sins_data.angle[1]*0.000001);   // 0.000001deg/LSB
			printf("angle3 = %.6f deg\n",(float)Sins_Output_Data_Nvidia.sins_data.angle[2]*0.000001);    // 0.000001deg/LSB
			printf("fusion_vel_1 = %.6f m/s\n",(float)Sins_Output_Data_Nvidia.sins_data.fusion_vel[0]*0.001); //0.001m/s/LSB
			printf("fusion_vel_2 = %.6f m/s\n",(float)Sins_Output_Data_Nvidia.sins_data.fusion_vel[1]*0.001); //0.001m/s/LSB
			printf("fusion_vel_3 = %.6f m/s\n",(float)Sins_Output_Data_Nvidia.sins_data.fusion_vel[2]*0.001); //0.001m/s/LSB
			printf("fusion_lon = %.6f \n",(float)Sins_Output_Data_Nvidia.sins_data.fusion_lon*0.000001);  //0.000001deg/LSB
			printf("fusion_lat = %.6f \n",(float)Sins_Output_Data_Nvidia.sins_data.fusion_lat*0.000001);  //0.000001deg/LSB
			printf("fusion_alt = %.6f m\n",(float)Sins_Output_Data_Nvidia.sins_data.fusion_alt*0.001);  //0.001m/LSB
			printf("gyro_data1 = %.6f deg/h\n",(float)Sins_Output_Data_Nvidia.sins_data.gyro_data[0]*0.001);  ////0.001deg/h/LSB
			printf("gyro_data2 = %.6f deg/h\n",(float)Sins_Output_Data_Nvidia.sins_data.gyro_data[1]*0.001);  ////0.001deg/h/LSB
			printf("gyro_data3 = %.6f deg/h\n",(float)Sins_Output_Data_Nvidia.sins_data.gyro_data[2]*0.001);  ////0.001deg/h/LSB
			printf("acce_data1 = %.6f m/s/s\n",(float)Sins_Output_Data_Nvidia.sins_data.acce_data[0]*0.00001);  //0.00001m/s/s/LSB
			printf("acce_data2 = %.6f m/s/s\n",(float)Sins_Output_Data_Nvidia.sins_data.acce_data[1]*0.00001);  //0.00001m/s/s/LSB
			printf("acce_data3 = %.6f m/s/s\n",(float)Sins_Output_Data_Nvidia.sins_data.acce_data[2]*0.00001);  //0.00001m/s/s/LSB
			printf("gyro_tmp1 = %.6f \n",(float)Sins_Output_Data_Nvidia.sins_data.gyro_tmp[0]*0.0625);   //0.0625/LSB
			printf("gyro_tmp2 = %.6f \n",(float)Sins_Output_Data_Nvidia.sins_data.gyro_tmp[1]*0.0625);   //0.0625/LSB
			printf("gyro_tmp3 = %.6f \n",(float)Sins_Output_Data_Nvidia.sins_data.gyro_tmp[2]*0.0625);   //0.0625/LSB
			printf("acce_tmp1 = %.6f \n",(float)Sins_Output_Data_Nvidia.sins_data.acce_tmp[0]*0.0625);   //0.0625/LSB
			printf("acce_tmp2 = %.6f \n",(float)Sins_Output_Data_Nvidia.sins_data.acce_tmp[1]*0.0625);   //0.0625/LSB
			printf("acce_tmp3 = %.6f \n",(float)Sins_Output_Data_Nvidia.sins_data.acce_tmp[2]*0.0625);   //0.0625/LSB
			printf("dvl_vel1 = %.6f m/s\n",(float)Sins_Output_Data_Nvidia.sins_data.dvl_vel[0]*0.001);   //0.001m/s/LSB
			printf("dvl_vel2 = %.6f m/s\n",(float)Sins_Output_Data_Nvidia.sins_data.dvl_vel[1]*0.001);   //0.001m/s/LSB
			printf("dvl_vel3 = %.6f m/s\n",(float)Sins_Output_Data_Nvidia.sins_data.dvl_vel[2]*0.001);   //0.001m/s/LSB
			printf("dvl_flag = %d\n",(int)Sins_Output_Data_Nvidia.sins_data.dvl_flag);   //0.001m/s/LSB
			printf("timestamp1 = %.6f s\n",(float)Sins_Output_Data_Nvidia.sins_data.timestamp*0.000001);   // s
			printf("timestamp2 = %.6f s\n",Sins_Output_Data_Nvidia.timestamp_nvidia);   
			printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~sins information end~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");	
		}

		// update
		Old_Sins_Output_Data_Nvidia = Sins_Output_Data_Nvidia;
	}

	return recv_frame_flag;
}




void Analy_Sins_Data::NavInfo_To_File(Sins_Output_Info_Nvidia * sins_output_info)
{
	//printf("NavInfo_To_File========================================================================================================\n");
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

	fprintf(sins_filestream,"%d ",sins_output_info->sins_data.counter); //degree
	fprintf(sins_filestream,"%d ",(int)sins_output_info->sins_data.state);
	fprintf(sins_filestream,"%d ",(int)sins_output_info->sins_data.nav_mode);  // 0.01 degree/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->sins_data.angle[0]*0.000001);  // 0.000001deg/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->sins_data.angle[1]*0.000001);   // 0.000001deg/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->sins_data.angle[2]*0.000001);    // 0.000001deg/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->sins_data.fusion_vel[0]*0.001); //0.001m/s/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->sins_data.fusion_vel[1]*0.001); //0.001m/s/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->sins_data.fusion_vel[2]*0.001); //0.001m/s/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->sins_data.fusion_lon*0.000001);  //0.000001deg/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->sins_data.fusion_lat*0.000001);  //0.000001deg/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->sins_data.fusion_alt*0.001);  //0.001m/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->sins_data.gyro_data[0]*0.001);  ////0.001deg/h/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->sins_data.gyro_data[1]*0.001);  ////0.001deg/h/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->sins_data.gyro_data[2]*0.001);  ////0.001deg/h/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->sins_data.acce_data[0]*0.00001);  //0.00001m/s/s/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->sins_data.acce_data[1]*0.00001);  //0.00001m/s/s/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->sins_data.acce_data[2]*0.00001);  //0.00001m/s/s/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->sins_data.gyro_tmp[0]*0.0625);   //0.0625/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->sins_data.gyro_tmp[1]*0.0625);   //0.0625/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->sins_data.gyro_tmp[2]*0.0625);   //0.0625/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->sins_data.acce_tmp[0]*0.0625);   //0.0625/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->sins_data.acce_tmp[1]*0.0625);   //0.0625/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->sins_data.acce_tmp[2]*0.0625);   //0.0625/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->sins_data.dvl_vel[0]*0.001);   //0.001m/s/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->sins_data.dvl_vel[1]*0.001);   //0.001m/s/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->sins_data.dvl_vel[2]*0.001);   //0.001m/s/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->sins_data.timestamp*0.000001);   // s
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



void Analy_Sins_Data::Nav_Error_Info_To_File(char * nav_data, unsigned int data_len,char data1,char data2)
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



void Analy_Sins_Data::NavInfo_To_File(Sins_Output_Info_Sanchi * sins_output_info)
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

	fprintf(sins_filestream,"%d ",sins_output_info->counter); //degree
	fprintf(sins_filestream,"%d ",(int)sins_output_info->state);
	fprintf(sins_filestream,"%d ",(int)sins_output_info->nav_mode);  // 0.01 degree/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->angle[0]*0.000001);  // 0.000001deg/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->angle[1]*0.000001);   // 0.000001deg/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->angle[2]*0.000001);    // 0.000001deg/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->fusion_vel[0]*0.001); //0.001m/s/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->fusion_vel[1]*0.001); //0.001m/s/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->fusion_vel[2]*0.001); //0.001m/s/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->fusion_lon*0.000001);  //0.000001deg/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->fusion_lat*0.000001);  //0.000001deg/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->fusion_alt*0.001);  //0.001m/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->gyro_data[0]*0.001);  ////0.001deg/h/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->gyro_data[1]*0.001);  ////0.001deg/h/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->gyro_data[2]*0.001);  ////0.001deg/h/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->acce_data[0]*0.00001);  //0.00001m/s/s/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->acce_data[1]*0.00001);  //0.00001m/s/s/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->acce_data[2]*0.00001);  //0.00001m/s/s/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->gyro_tmp[0]*0.0625);   //0.0625/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->gyro_tmp[1]*0.0625);   //0.0625/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->gyro_tmp[2]*0.0625);   //0.0625/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->acce_tmp[0]*0.0625);   //0.0625/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->acce_tmp[1]*0.0625);   //0.0625/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->acce_tmp[2]*0.0625);   //0.0625/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->dvl_vel[0]*0.001);   //0.001m/s/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->dvl_vel[1]*0.001);   //0.001m/s/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->dvl_vel[2]*0.001);   //0.001m/s/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->timestamp*0.000001);   // s
	fprintf(sins_filestream,"%.6f ",add_second);   

	fprintf(sins_filestream,"\n");

	if(fflush_num==100)
	{
		fflush_num=0;
		fflush(sins_filestream);
	}					

}



void Analy_Sins_Data::NavInfo_To_File(Sins_Output_Info_Sanchi * sins_output_info,bool valid_flag)
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

	fprintf(sins_filestream,"%d ",sins_output_info->counter); //degree
	fprintf(sins_filestream,"%d ",(int)sins_output_info->state);
	fprintf(sins_filestream,"%d ",(int)sins_output_info->nav_mode);  // 0.01 degree/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->angle[0]*0.000001);  // 0.000001deg/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->angle[1]*0.000001);   // 0.000001deg/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->angle[2]*0.000001);    // 0.000001deg/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->fusion_vel[0]*0.001); //0.001m/s/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->fusion_vel[1]*0.001); //0.001m/s/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->fusion_vel[2]*0.001); //0.001m/s/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->fusion_lon*0.000001);  //0.000001deg/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->fusion_lat*0.000001);  //0.000001deg/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->fusion_alt*0.001);  //0.001m/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->gyro_data[0]*0.001);  ////0.001deg/h/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->gyro_data[1]*0.001);  ////0.001deg/h/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->gyro_data[2]*0.001);  ////0.001deg/h/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->acce_data[0]*0.00001);  //0.00001m/s/s/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->acce_data[1]*0.00001);  //0.00001m/s/s/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->acce_data[2]*0.00001);  //0.00001m/s/s/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->gyro_tmp[0]*0.0625);   //0.0625/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->gyro_tmp[1]*0.0625);   //0.0625/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->gyro_tmp[2]*0.0625);   //0.0625/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->acce_tmp[0]*0.0625);   //0.0625/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->acce_tmp[1]*0.0625);   //0.0625/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->acce_tmp[2]*0.0625);   //0.0625/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->dvl_vel[0]*0.001);   //0.001m/s/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->dvl_vel[1]*0.001);   //0.001m/s/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->dvl_vel[2]*0.001);   //0.001m/s/LSB
	fprintf(sins_filestream,"%.6f ",(float)sins_output_info->timestamp*0.000001);   // s
	fprintf(sins_filestream,"%.6f ",add_second);  
	fprintf(sins_filestream,"%d ",valid_flag); 

	fprintf(sins_filestream,"\n");

	if(fflush_num==100)
	{
		fflush_num=0;
		fflush(sins_filestream);
	}					

}



/*
 *函数功能：
 *输入参数：
 *输出参数： 
 *备    注：  Serial_Sins_Data
 *
 */
char Analy_Sins_Data::Nav_Init(float Longitude,float Latitude,short int Altitude)
{

	int sins_data_num;
	unsigned int us_delay = 100;
	//char t[2] = {0x55,0xAA};
	char Sins_Binding_Data[100]={0x00}; 
	if(-1==m_fd_sins)  
	{
		return -1;		
	}

	SINS_Init_Cmd.CMD = Sins_AlignNavig_CMD;
	SINS_Init_Cmd.FusionMode = Sins_DVL_Mode; //Sins_DVL_Mode
	SINS_Init_Cmd.Longitude_Data = (int)(Longitude*1000000);
	SINS_Init_Cmd.Latitude_Data = (int)(Latitude*1000000);
	SINS_Init_Cmd.Altitude_Data = (short int )Altitude;	
	SINS_Init_Cmd.Satellite_Sign = 0x00;
	for(int i=0;i<3;i++)
	{
		SINS_Init_Cmd.ENU_VelData[i] = 0;
	}

	for(int i=0;i<13;i++)
	{
		SINS_Init_Cmd.Backup_Data[i] = 0;
	}
		
	sins_data_num = Encode_NavInit_Info(SINS_Init_Cmd,Sins_Binding_Data); //Sins_DVL_Mode
	for(int i=0;i<2;i++)
	{
		//char t[] = {0x55,0xAA,0x20,0x03,0x02,0x08,0xE7,0xEB,0x06,0x08,0x3C,0x64,0x02,0x7B,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x2A};
		printf("\n");
		printf("sins_data_num = %d\n",sins_data_num);
		for(int j=0;j<sins_data_num;j++)
		{
			//Sins_Binding_Data[i] =  t[i];
			printf("%02x ",(unsigned char)Sins_Binding_Data[j]);
		}
		printf("\n");

		serial_sins_data.SendString(m_fd_sins ,Sins_Binding_Data, sins_data_num ,us_delay);
		printf("=================================Nav_Init========================\n");
		sleep(1);
	}

	return 0x01;	

}



/*
 *函数功能:
 *输入参数：
 *输出参数：bytes of framedata 
 *备    注： add analy
 *
 */
int Analy_Sins_Data::Encode_NavInit_Info(SINS_Init_CmdInfo Cmd,char * Nav_Init_FrameData)
{	

	unsigned int cur_index = 0;
	int data_bytes=sizeof(Cmd); 

	char * nav_init_data =(char *)malloc(sizeof(char)*data_bytes);
	if(NULL!=nav_init_data)
	{
		memcpy(nav_init_data,(char *)&Cmd,data_bytes);

	}
	else
	{
		return -1;
	}

	int frame_datanum = Sins_Encode( nav_init_data,data_bytes,Nav_Init_FrameData);
	if(NULL!=nav_init_data)
	{
		free(nav_init_data);
	}

	return (frame_datanum);

}




/*
 *函数功能：
 *输入参数：
 *输出参数：bytes of framedata 
 *备    注：struct Calib_Nav_Info
 *
 */
int Analy_Sins_Data::Sins_Encode( char * nav_data, unsigned char data_len,char * nav_frame_data)
{

	unsigned int cur_index = 0;
	nav_frame_data[cur_index++] = 0X55;  
	nav_frame_data[cur_index++] = 0XAA;  
	nav_frame_data[cur_index++] = data_len;  

	for(int i=0;i<data_len;i++)
	{
		nav_frame_data[cur_index++] = nav_data[i];  	
	}



	nav_frame_data[cur_index++] = CheckSum(nav_frame_data+2,data_len+1);	

	nav_frame_data[13] = 0x7B;
	nav_frame_data[35] = 0x2A;

	return (cur_index);
}


char Analy_Sins_Data::CheckSum( char * nav_data, unsigned int data_len)
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




float Analy_Sins_Data::BLEndianFloat(float value)
{
	float_conv d1,d2;
	d1.f= value;
	d2.c[0] = d1.c[3];
	d2.c[1] = d1.c[2];
	d2.c[2] = d1.c[1];
	d2.c[3] = d1.c[0];

	return (d2.f);
}

