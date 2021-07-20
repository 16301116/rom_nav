#include"udp/UDP_RecvData.h"

UDP_RecvData::UDP_RecvData(std::string ip,int send_port ,int recv_port)
{
	const int timewait = 5;
	const int isServer = 1;
	if(save_udp_file_flag)
	{
		time_t currtime =time(NULL);
		tm *sins_p =localtime(&currtime);
		sprintf(udp_file_name,"../data/output_data/udp-data%04d%02d%02d%02d%02d%02d.txt",1900+sins_p->tm_year,1+sins_p->tm_mon,sins_p->tm_mday,sins_p->tm_hour,sins_p->tm_min,sins_p->tm_sec);	
		udp_filestream = fopen(udp_file_name,"w+");
		if (udp_filestream==NULL)
		{
			printf("can't create file to save udp data!\n");
		}
		else
		{
			printf("successful create file to save udp data!\n");
		}

	}

	udprecv =  new UDP_Communication(ip,send_port,recv_port,timewait,isServer);
	std::thread t1(Recv_Encoder_Data,this);
	t1.detach();
}


UDP_RecvData::UDP_RecvData(int recv_port)
{
	const int timewait = 5;
	const int isServer = 1;
	if(save_udp_file_flag)
	{
		time_t currtime =time(NULL);
		tm *sins_p =localtime(&currtime);
		sprintf(udp_file_name,"../data/output_data/udp-data%04d%02d%02d%02d%02d%02d.txt",1900+sins_p->tm_year,1+sins_p->tm_mon,sins_p->tm_mday,sins_p->tm_hour,sins_p->tm_min,sins_p->tm_sec);	
		udp_filestream = fopen(udp_file_name,"w+");
		if (udp_filestream==NULL)
		{
			printf("can't create file to save udp data!\n");
		}
		else
		{
			printf("successful create file to save udp data!\n");
		}

	}

	
	udprecv =  new UDP_Communication(recv_port,timewait);
	std::thread t1(Recv_Encoder_Data,this);
	t1.detach();
}




UDP_RecvData::~UDP_RecvData()
{
	delete udprecv;
	fclose(udp_filestream);
}



char UDP_RecvData::Recv_Encoder_Data(UDP_RecvData * ptr)
{
	if(ptr->disp_all_flag)
	{
		ptr->disp_sins_flag = true;
		ptr->disp_mems_flag = true;
		ptr->disp_gps_flag = true;
		ptr->disp_dvl_flag = true;
		ptr->disp_depth_flag = true;
		ptr->disp_pressure_flag = true;
		ptr->disp_ctd_flag = true;
		ptr->disp_usbl_flag = true;
	}
	else
	{
		ptr->disp_sins_flag = false;
		ptr->disp_mems_flag = false;
		ptr->disp_gps_flag = false;
		ptr->disp_dvl_flag = false;
		ptr->disp_depth_flag = false;
		ptr->disp_pressure_flag = false;
		ptr->disp_ctd_flag = false;
		ptr->disp_usbl_flag = false;
	}

  while(1)
  {
		static int Frame_Num = 0;
		memset(ptr->Recv_Buf,0,UDP_RECV_MAX_NUM);	
		ptr->Recv_Num = ptr->udprecv->Recv_String(ptr->Recv_Buf,UDP_RECV_MAX_NUM);
		/*
		printf("Recv_Num = %d==============\n",ptr->Recv_Num);
		printf("\n");
		for(int i=0;i<ptr->Recv_Num;i++)
		{
			printf("0x%02x ",(unsigned char)ptr->Recv_Buf[i]);
		}
		printf("\n");
		*/
		if(ptr->Recv_Num>0)
		{
			Frame_Num++;
			if(ptr->Recv_Buf[0]==ptr->udprecv->UDP_HeaderDataID) //MAVLINK
			{ 
				ptr->MAVLINK_Recv_Encoder_Data(ptr->UDP_FrameData);
			}
			
			else if(ptr->Recv_Buf[0]==SANCHI_SINS_HEADER1 && ptr->Recv_Buf[1]==SANCHI_SINS_HEADER2)  //sins data
			{
				if(ptr->debug_flag)
				{
					printf("UDP:SINS DATA==============\n");
				}
				int Frame_Num = 0;
				bool add_check_lost_flag = true;
				for(int i=0;i<ptr->Recv_Num;i++)
				{
					//ptr->get_sins_data.Analy_Sins_Frame(ptr->Recv_Buf[i],Frame_Num);
					ptr->sins_frame_flag = ptr->get_sins_data.Sins_Postprocess(ptr->Recv_Buf[i], ptr->disp_sins_flag, add_check_lost_flag);
					if(ptr->sins_frame_flag)
					{
						ptr->fog_sins_que.push(ptr->get_sins_data.Sins_Output_Data_Nvidia); 
						if(ptr->fog_sins_que.size()>ptr->max_fog_sins_que)
						{
							Analy_Sins_Data::Sins_Output_Info_Nvidia data_buf;
							ptr->fog_sins_que.try_pop(data_buf);				
						}

					}
				}
				
			}
			
			else if(ptr->Recv_Buf[0]==MTI_G_710_HEADER1 && ptr->Recv_Buf[1]==MTI_G_710_HEADER2) //MTI-G-710 data
			{
				if(ptr->debug_flag)
				{
					printf("UDP:MEMS SINS DATA==============\n");
				}
				int Frame_Num = 0;
				bool add_check_lost_flag = true;
				for(int i=0;i<ptr->Recv_Num;i++)
				{
					ptr->mems_sins_frame_flag = ptr->get_mti_data.Sins_Postprocess(ptr->Recv_Buf[i], ptr->disp_mems_flag, add_check_lost_flag);
					if(ptr->mems_sins_frame_flag) //receive one frame data 
					{
						ptr->mems_sins_que.push(ptr->get_mti_data.Mems_IMU_Output_Data_Nvidia); 
						if(ptr->mems_sins_que.size()>ptr->max_mems_sins_que)
						{
							Analy_MEMS_IMU_Data::Mems_IMU_Output_Info_Nvidia data_buf;
							ptr->mems_sins_que.try_pop(data_buf);				
						}
					}				
					//ptr->get_mti_data.Analy_Sins_Frame(ptr->Recv_Buf[i],Frame_Num); 
				}

			}
			
			else if(ptr->Recv_Buf[0]==GPS_HEADER1 && ptr->Recv_Buf[1]==GPS_HEADER2) //gps data
			{
				for(int i=0;i<ptr->Recv_Num;i++)
				{
					ptr->gps_frame_flag = ptr->get_gps_data.Analy_GPS_Frame(ptr->Recv_Buf[i],ptr->disp_gps_flag); 
					if(ptr->gps_frame_flag)
					{
						ptr->gps_que.push(ptr->get_gps_data.GNSS_SEND_FRAME); 
						if(ptr->gps_que.size()>ptr->max_gps_que)
						{
							Analy_GPS_Data::GNSS_FRAME_Info data_buf;
							ptr->gps_que.try_pop(data_buf);				
						}
					}
				}
			}	
				
			else if(ptr->Recv_Buf[0]==DVL_HEADER1 && ptr->Recv_Buf[1]==DVL_HEADER2) //dvl data 
			{
				
				if(ptr->debug_flag)
				{
					printf("UDP:DVL DATA==============\n");
				}
				for(int i=0;i<ptr->Recv_Num;i++)
				{
					ptr->dvl_frame_flag = ptr->get_dvl_data.Analy_DVL_Frame(ptr->Recv_Buf[i],ptr->disp_dvl_flag,\
					ptr->dvl_filter_flag);
					if(ptr->dvl_frame_flag)
					{
						/*
						ptr->dvl_que.push(ptr->get_dvl_data.DVL_Output_Data_Nvidia); 
						if(ptr->dvl_que.size()>ptr->max_dvl_que)
						{
							Analy_DVL_Data::DVL_Output_Info_Nvidia data_buf;
							ptr->dvl_que.try_pop(data_buf);				
						}
						*/
						// filter dvl data
						ptr->dvl_filter_que.push(ptr->get_dvl_data.DVL_FilterVel_Data); 
						if(ptr->dvl_filter_que.size()>ptr->max_dvl_que)
						{
							Analy_DVL_Data::DVL_FilterVel_Info data_buf;
							ptr->dvl_filter_que.try_pop(data_buf);				
						}

					}
				}

			}
			
			else if(ptr->Recv_Buf[0]==DEPTH_HEADER1 && ptr->Recv_Buf[1]==DEPTH_HEADER2) // depth data
			{
				for(int i=0;i<ptr->Recv_Num;i++)
				{
					ptr->depth_frame_flag = ptr->get_depth_data.Analy_Frame(ptr->Recv_Buf[i],ptr->disp_depth_flag); 
					if(ptr->depth_frame_flag)
					{
						ptr->depth_que.push(ptr->get_depth_data.Depth_Data_Nvidia); 
						if(ptr->depth_que.size()>ptr->max_depth_que)
						{
							Analy_Depth_Data::Depth_Info_Nvidia data_buf;
							ptr->depth_que.try_pop(data_buf);				
						}
					}
				}

			}

			else if(ptr->Recv_Buf[0]==PRESSURE_HEADER1 && ptr->Recv_Buf[1]==PRESSURE_HEADER2) // pressure data
			{
				for(int i=0;i<ptr->Recv_Num;i++)
				{
					ptr->pressure_frame_flag = ptr->get_pressure_data.Analy_Frame(ptr->Recv_Buf[i],ptr->disp_pressure_flag); 
					if(ptr->pressure_frame_flag)
					{
						ptr->pressure_que.push(ptr->get_pressure_data.Pressure_Data_Nvidia); 
						if(ptr->pressure_que.size()>ptr->max_pressure_que)
						{
							Analy_Pressure_Data::Pressure_Info_Nvidia data_buf;
							ptr->pressure_que.try_pop(data_buf);				
						}
					}
				}

			}
			
			else if(ptr->Recv_Buf[0]==CTD_HEADER1 && ptr->Recv_Buf[1]==CTD_HEADER2) // ctd data
			{
				if(ptr->debug_flag)
				{
					printf("UDP:CTD DATA==============\n");
				}
				for(int i=0;i<ptr->Recv_Num;i++)
				{
					ptr->ctd_frame_flag = ptr->get_ctd_data.Analy_Frame(ptr->Recv_Buf[i],ptr->disp_ctd_flag); 
					if(ptr->ctd_frame_flag)
					{
						ptr->ctd_que.push(ptr->get_ctd_data.CTD_Data_Nvidia); 
						if(ptr->ctd_que.size()>ptr->max_ctd_que)
						{
							Analy_CTD_Data::CTD_Info_Nvidia data_buf;
							ptr->ctd_que.try_pop(data_buf);				
						}
					}
				}

			}

			else if((unsigned char)ptr->Recv_Buf[0]==USBL_HEADER1 && (unsigned char)ptr->Recv_Buf[1]==USBL_HEADER2) // usbl data
			{
				if(ptr->debug_flag)
				{
					printf("UDP:USBL DATA==============\n");
				}
				for(int i=0;i<ptr->Recv_Num;i++)
				{
					ptr->usbl_frame_flag = ptr->get_usbl_data.Analy_Frame(ptr->Recv_Buf[i],ptr->disp_usbl_flag); 
					if(ptr->usbl_frame_flag)
					{
						ptr->usbl_que.push(ptr->get_usbl_data.USBL_Data_Nvidia); 
						if(ptr->usbl_que.size()>ptr->max_usbl_que)
						{
							Analy_USBL_Data::USBL_Info_Nvidia data_buf;
							ptr->usbl_que.try_pop(data_buf);				
						}
					}
				}

			}
			else
			{

			}
			
		}
		usleep(2000);	 //2ms
		
	}

}



void UDP_RecvData::MAVLINK_Recv_Encoder_Data(UDP_Communication::UDP_Info_ROV * UDP_FrameData)
{
	//char Recv_Flag = udprecv->Recv_Decoder_ROV(&UDP_FrameData,1000);
	char Recv_Flag = udprecv->Decoder_ROV(Recv_Buf , Recv_Num, &UDP_FrameData);
	static unsigned char Conf_Flag  = 0x00;
	char Flag  = Recv_Flag;
	if(DECODER_NORM==Recv_Flag)  
	{
		switch(UDP_FrameData->Msg_ID) 
		{
			case SHUTDOWN_Msg_ID:  //receive and no sdo
			{
				char power_cmd = UDP_FrameData->LoadData[0];
				if(power_cmd==0x01) 
				{
					printf("============================Power Off Now=========================\n");
					system("sudo shutdown -h now");
				}
				else if(power_cmd==0x02)
				{
					printf("============================Restart Now=========================\n");
					system("sudo shutdown -r now");
				}

				break;
			}
			case SETNAVMODE_Msg_ID:  // set navigation mode
			{
				mtx_nav_mode.lock();
				nav_mode = UDP_FrameData->LoadData[0];
				printf("======================UDP:set navigation  mode  = %d=======================\n",nav_mode);
				udp_nav_mode_flag = true;
				mtx_nav_mode.unlock();
				break;
			}
			default:
			{
				break;
			}

		}
				 
	}         

}



void UDP_RecvData::NavInfo_To_File(float a1,float a2,float a3,float a4,char b1,char b2)
{
	static int fflush_num;
	fflush_num++;

	time_t timep;
	struct tm *p;
	time(&timep);
	p=gmtime(&timep);

	//output time
	fprintf(udp_filestream,"%04d-",1900+p->tm_year);
	fprintf(udp_filestream,"%02d-",1+p->tm_mon);
	fprintf(udp_filestream,"%02d-",p->tm_mday);
	fprintf(udp_filestream,"%02d-",p->tm_hour);
	fprintf(udp_filestream,"%02d-",p->tm_min);
	fprintf(udp_filestream,"%02d    ",p->tm_sec);

	fprintf(udp_filestream,"%.6f ",a1); 
	fprintf(udp_filestream,"%.6f ",a2);  
	fprintf(udp_filestream,"%.6f ",a3);  
	fprintf(udp_filestream,"%.6f ",a4);  
	fprintf(udp_filestream,"0x%02x ",b1);  
	fprintf(udp_filestream,"0x%02x ",b2);  
	fprintf(udp_filestream,"\n");

	if(fflush_num==100)
	{
		fflush_num=0;
		fflush(udp_filestream);
	}					

}
