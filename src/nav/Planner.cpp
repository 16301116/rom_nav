#include <math.h>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Dense> 
#include <cstring>
#include <math.h>
#include "nav/Planner.h"


Planner::Planner()
{
	time_t currtime =time(NULL);
	tm *sins_p =localtime(&currtime);
	sprintf(location_file_name,"../data/output_data/location_data%04d%02d%02d%02d%02d%02d.txt",1900+sins_p->tm_year,1+sins_p->tm_mon,sins_p->tm_mday,sins_p->tm_hour,sins_p->tm_min,sins_p->tm_sec);	
	location_filestream = fopen(location_file_name,"w+");
	if (location_filestream==NULL)
	{
		printf("can't create file to save location data!\n");
	}
	else
	{
		printf("successful create file to save location data!\n");
	}


	sprintf(nav2ctrl_file_name,"../data/output_data/nav2ctrl_data%04d%02d%02d%02d%02d%02d.txt",1900+sins_p->tm_year,1+sins_p->tm_mon,sins_p->tm_mday,sins_p->tm_hour,sins_p->tm_min,sins_p->tm_sec);	
	nav2ctrl_filestream = fopen(nav2ctrl_file_name,"w+");
	if (nav2ctrl_filestream==NULL)
	{
		printf("can't create file to save nav2ctrl data!\n");
	}
	else
	{
		printf("successful create file to save nav2ctrl data!\n");
	}

	Init_Data();

	udp_recvdata = new UDP_RecvData(Nav_Nvidia_Port);
	int timewait = 2;
	udp2ctrol = new UDP_Communication(Controller_IP,Controller_Port,timewait); // send navigation data to control 

	std::thread t1(run,this,std::ref(udp_recvdata->fog_sins_que),\
	std::ref(udp_recvdata->mems_sins_que),\
	std::ref(udp_recvdata->gps_que),\
	std::ref(udp_recvdata->dvl_filter_que),\
	std::ref(udp_recvdata->ctd_que),\
	std::ref(udp_recvdata->depth_que),\
	std::ref(udp_recvdata->pressure_que),\
	std::ref(udp_recvdata->usbl_que));
	t1.detach();
}

Planner::Planner(bool udp_test_flag)
{
	time_t currtime =time(NULL);
	tm *sins_p =localtime(&currtime);
	sprintf(location_file_name,"../data/output_data/location_data%04d%02d%02d%02d%02d%02d.txt",1900+sins_p->tm_year,1+sins_p->tm_mon,sins_p->tm_mday,sins_p->tm_hour,sins_p->tm_min,sins_p->tm_sec);	
	location_filestream = fopen(location_file_name,"w+");
	if (location_filestream==NULL)
	{
		printf("can't create file to save location data!\n");
	}
	else
	{
		printf("successful create file to save location data!\n");
	}

	sprintf(nav2ctrl_file_name,"../data/output_data/nav2ctrl_data%04d%02d%02d%02d%02d%02d.txt",1900+sins_p->tm_year,1+sins_p->tm_mon,sins_p->tm_mday,sins_p->tm_hour,sins_p->tm_min,sins_p->tm_sec);	
	nav2ctrl_filestream = fopen(nav2ctrl_file_name,"w+");
	if (nav2ctrl_filestream==NULL)
	{
		printf("can't create file to save nav2ctrl data!\n");
	}
	else
	{
		printf("successful create file to save nav2ctrl data!\n");
	}

	Init_Data();

	udp_recvdata = new UDP_RecvData(Nav_Nvidia_Port);
	if(udp_test_flag)
	{
		int timewait = 2;
		udp2ctrol = new UDP_Communication(Controller_IP,Controller_Port,timewait); // send navigation data to control 
		
		std::thread t1(run_test,this);
		t1.detach();
	}


}


Planner::Planner(threadsafe_queue<Analy_Sins_Data::Sins_Output_Info_Nvidia> & fog_sins_que,\
threadsafe_queue<Analy_MEMS_IMU_Data::Mems_IMU_Output_Info_Nvidia> & mems_sins_que,\
threadsafe_queue<Analy_GPS_Data::GNSS_FRAME_Info> & gps_que,\
threadsafe_queue<Analy_DVL_Data::DVL_FilterVel_Info> & dvl_que,\
threadsafe_queue<Analy_CTD_Data::CTD_Info_Nvidia> & ctd_que,\
threadsafe_queue<Analy_Depth_Data::Depth_Info_Nvidia> & depth_que,\
threadsafe_queue<Analy_Pressure_Data::Pressure_Info_Nvidia> & pressure_que,\
threadsafe_queue<Analy_USBL_Data::USBL_Info_Nvidia> & usbl_que)
{

	time_t currtime =time(NULL);
	tm *sins_p =localtime(&currtime);
	sprintf(location_file_name,"../data/output_data/location_data%04d%02d%02d%02d%02d%02d.txt",1900+sins_p->tm_year,1+sins_p->tm_mon,sins_p->tm_mday,sins_p->tm_hour,sins_p->tm_min,sins_p->tm_sec);	
	location_filestream = fopen(location_file_name,"w+");
	if (location_filestream==NULL)
	{
		printf("can't create file to save location data!\n");
	}
	else
	{
		printf("successful create file to save location data!\n");
	}

	sprintf(nav2ctrl_file_name,"../output_data/data/location_data%04d%02d%02d%02d%02d%02d.txt",1900+sins_p->tm_year,1+sins_p->tm_mon,sins_p->tm_mday,sins_p->tm_hour,sins_p->tm_min,sins_p->tm_sec);	
	nav2ctrl_filestream = fopen(nav2ctrl_file_name,"w+");
	if (nav2ctrl_filestream==NULL)
	{
		printf("can't create file to save nav2ctrl data!\n");
	}
	else
	{
		printf("successful create file to save nav2ctrl data!\n");
	}

	Init_Data();
	
	std::thread t1(run,this,std::ref(udp_recvdata->fog_sins_que),\
	std::ref(udp_recvdata->mems_sins_que),\
	std::ref(udp_recvdata->gps_que),\
	std::ref(udp_recvdata->dvl_filter_que),\
	std::ref(udp_recvdata->ctd_que),\
	std::ref(udp_recvdata->depth_que),\
	std::ref(udp_recvdata->pressure_que),\
	std::ref(udp_recvdata->usbl_que));
	t1.detach();

}


Planner::~Planner()
{
	delete udp_recvdata;
	delete udp2ctrol;
	delete aly_dvl_data;
	delete aly_gps_data;	
	delete aly_usbl_data;
	delete aly_sins_data;
	delete sins_dvl_dr;
	fclose(location_filestream);
	fclose(nav2ctrl_filestream);
	
}


void Planner::Init_Data(void)
{
	memset(&Nav_Output_Data,0,sizeof(Nav_Output_Data));
	memset(&Mul_Sensor_State,0,sizeof(Mul_Sensor_State));
	memset(&Sensor_Conti_Error_DataNum,0,sizeof(Sensor_Conti_Error_DataNum));	
	memset(&Cur_Recv_Num,0,sizeof(Cur_Recv_Num));	
	memset(&framedata,0,sizeof(framedata));
	aly_sins_data = new Analy_Sins_Data();
	aly_dvl_data = new Analy_DVL_Data();
	aly_gps_data = new Analy_GPS_Data();
	aly_usbl_data = new Analy_USBL_Data();
	sins_dvl_dr = new sins_dvl_DR();
}

void Planner::run(Planner *ptr,threadsafe_queue<Analy_Sins_Data::Sins_Output_Info_Nvidia> & sins_que,\
threadsafe_queue<Analy_MEMS_IMU_Data::Mems_IMU_Output_Info_Nvidia> & mems_sins_que,\
threadsafe_queue<Analy_GPS_Data::GNSS_FRAME_Info> & gpsque,\
threadsafe_queue<Analy_DVL_Data::DVL_FilterVel_Info> & dvl_que,\
threadsafe_queue<Analy_CTD_Data::CTD_Info_Nvidia> & ctd_que,\
threadsafe_queue<Analy_Depth_Data::Depth_Info_Nvidia> & depth_que,\
threadsafe_queue<Analy_Pressure_Data::Pressure_Info_Nvidia> & pressure_que,\
threadsafe_queue<Analy_USBL_Data::USBL_Info_Nvidia> & usbl_que)
{
	unsigned int  recv_u_second = round(1000000.0/ptr->recv_fre);
	int mult_num = round(ptr->recv_fre/ptr->send_ctrl_fre);
	while(1)
	{
		ptr->Get_UDP_Data(ptr->udp_recvdata);
        ptr->Get_Nav_Data(sins_que);  
		if(!ptr->Ready_Flag.sins_ready_flag)
        {
            continue;
        }
		ptr->cur_recv_counter++;
		ptr->Get_Mems_SINS_Data(mems_sins_que);
		ptr->Get_GPS_Data(gpsque);
		ptr->Get_DVL_Data(dvl_que);
		ptr->Get_CTD_Data(ctd_que);
		ptr->Get_Pressure_Data(pressure_que);
		ptr->Get_Depth_Data(depth_que);
		ptr->Get_USBL_Data(usbl_que);
		ptr->Save_Nav2Ctrl_Data();
		ptr->Assembly();
		ptr->Save_NAV_Data();
		if(ptr->cur_recv_counter%mult_num==0)
		{
			ptr->Nav2Ctrl_UDP(); // send data to controler 
		}
		usleep(recv_u_second);//100Hz(10ms)
    }

}


void Planner::run_test(Planner *ptr)
{
	while(1)
	{
		ptr->Test2_Nav2Ctrl_UDP();
        usleep(1000000);//1s  
    }

}


void Planner::Get_UDP_Data(UDP_RecvData * data)
{
	data->mtx_nav_mode.lock();
	if(data->udp_nav_mode_flag)
	{
		udp_set_nav_mode = data->nav_mode;
		data->nav_mode = 0x00;
		if(udp_set_nav_mode==navigation_state.sins_dvl_state) //sins/dvl
		{
			first_gps_sins_dvl_mode_flag = true;
		}
		data->udp_nav_mode_flag = false;
	}
	data->mtx_nav_mode.unlock();


}





void Planner::Get_Nav_Data(threadsafe_queue<Analy_Sins_Data::Sins_Output_Info_Nvidia> & nav_que)
{
    if(nav_que.size()>0)
    {	
		Cur_Recv_Num.sins_num++;
		if(debug_flag)
		{
			printf(" planner receive sins data ================\n");	
		}
        nav_que.try_pop(framedata.sinsframedata);
		Sensor_Conti_Error_DataNum.sins_num = 0;
    }
    else
    {
		Sensor_Conti_Error_DataNum.sins_num++;
    }

	if((Cur_Recv_Num.sins_num>=1)&&(Sensor_Conti_Error_DataNum.sins_num<Max_Conti_Error_Num.sins_num))
	{
		Ready_Flag.sins_ready_flag = true;
	}
	else
	{
		Ready_Flag.sins_ready_flag = false;
	}

	Get_Nav_State(Ready_Flag.sins_ready_flag);
}

void Planner::Get_Nav_State(bool flag)
{
	if(flag)
	{
		Mul_Sensor_State.sins_state = (char)framedata.sinsframedata.sins_data.state;
	}
	else
	{
		Mul_Sensor_State.sins_state = Analy_Sins_Data::standby_state;		
	}

}



void Planner::Get_Mems_SINS_Data(threadsafe_queue<Analy_MEMS_IMU_Data::Mems_IMU_Output_Info_Nvidia> & mems_que)
{
	//old_DR_data = new_DR_data;
    if(mems_que.size()>0)
    {
		Cur_Recv_Num.mems_sins_num++;
		if(debug_flag)
		{
			printf(" planner receive mems sins data ================\n");	
		}
		mems_que.try_pop(framedata.mems_sinsframedata);
		Sensor_Conti_Error_DataNum.mems_sins_num = 0;
    }
    else
    {
		Sensor_Conti_Error_DataNum.mems_sins_num++;
    }

	if((Cur_Recv_Num.mems_sins_num>=1)&&(Sensor_Conti_Error_DataNum.mems_sins_num<Max_Conti_Error_Num.mems_sins_num))
	{
		Ready_Flag.mems_sins_ready_flag = true;
	}
	else
	{
		Ready_Flag.mems_sins_ready_flag = false;
	}

	Get_Mems_SINS_State(Ready_Flag.mems_sins_ready_flag);

}



void Planner::Get_Mems_SINS_State(bool flag)
{
	if(flag)
	{
		Mul_Sensor_State.mems_sins_state = 0x01;
	}
	else
	{
		Mul_Sensor_State.mems_sins_state = 0x00;
	}
}


void Planner::Get_GPS_Data(threadsafe_queue<Analy_GPS_Data::GNSS_FRAME_Info> & gps_que)
{
    if(gps_que.size()>0)
    {
		Cur_Recv_Num.gps_num++;
		if(debug_flag)
		{
			printf(" planner receive GPS data ================\n");	
		}
		//gps_que.try_pop(framedata.gpsframedata);
		gps_que.try_pop(buf_gpsframedata);	
		Sensor_Conti_Error_DataNum.gps_num = 0;
    }
    else
    {
		Sensor_Conti_Error_DataNum.gps_num++;
    }

	if((Cur_Recv_Num.gps_num>=1)&&(Sensor_Conti_Error_DataNum.gps_num<Max_Conti_Error_Num.gps_num))
	{
		Ready_Flag.gps_ready_flag = true;
	}
	else
	{
		Ready_Flag.gps_ready_flag = false;
	}

	Get_GPS_State(Ready_Flag.gps_ready_flag);
}


void Planner::Get_GPS_State(bool flag)
{
	if(flag) // have data
	{
		double vel = (float)buf_gpsframedata.speed*0.001/3.6;
		double star = (int)buf_gpsframedata.star;
		double lat = (double)buf_gpsframedata.lat*0.0000001;
		double lon = (double)buf_gpsframedata.lon*0.0000001;
		if((lon!=0)&&(lat!=0)&&(star>=aly_gps_data->min_invalid_star)) // invalid
		{
			if(abs(vel)<=aly_gps_data->max_gps_speed) // invalid vel and pos
			{
				Mul_Sensor_State.gps_state = aly_gps_data->pos_vel_invalid_state;
			}
			else // only invalid pos
			{
				Mul_Sensor_State.gps_state = aly_gps_data->pos_invalid_state;
			}
			framedata.gpsframedata = buf_gpsframedata;
		}
		else // valid data
		{
			Mul_Sensor_State.gps_state = aly_gps_data->valid_data_state;
		}

	}
	else // no data
	{
		Mul_Sensor_State.gps_state = aly_gps_data->no_data_state;
	}
}



void Planner::Get_DVL_Data(threadsafe_queue<Analy_DVL_Data::DVL_FilterVel_Info> & dvl_que)
{
    if(dvl_que.size()>0)
    {
		Cur_Recv_Num.dvl_num++;
		if(debug_flag)
		{
			printf(" planner receive DVL data ================\n");	
		}
		dvl_que.try_pop(framedata.dvl_framedata);
		Sensor_Conti_Error_DataNum.dvl_num = 0;
    }
    else
    {
		Sensor_Conti_Error_DataNum.dvl_num++;
    }

	if((Cur_Recv_Num.dvl_num>=1)&&(Sensor_Conti_Error_DataNum.dvl_num<Max_Conti_Error_Num.dvl_num))
	{
		Ready_Flag.dvl_ready_flag = true;
	}
	else
	{
		Ready_Flag.dvl_ready_flag = false;
	}
	Get_DVL_State(Ready_Flag.dvl_ready_flag );
}



void Planner::Get_DVL_State(bool flag)
{
	if(flag)
	{
		Mul_Sensor_State.dvl_state = framedata.dvl_framedata.dvl_flag;
	}
	else
	{
		Mul_Sensor_State.dvl_state = aly_dvl_data->no_data_state ;
	}
}



void Planner::Get_CTD_Data(threadsafe_queue<Analy_CTD_Data::CTD_Info_Nvidia> & ctd_que)
{
    if(ctd_que.size()>0)
    {
		Cur_Recv_Num.ctd_num++;
		if(debug_flag)
		{
			printf(" planner receive CTD data ================\n");	
		}
		ctd_que.try_pop(framedata.ctd_framedata);
		Sensor_Conti_Error_DataNum.ctd_num = 0;
    }
    else
    {
		Sensor_Conti_Error_DataNum.ctd_num++;
    }

	if((Cur_Recv_Num.ctd_num>=1)&&(Sensor_Conti_Error_DataNum.ctd_num<Max_Conti_Error_Num.ctd_num))
	{
		Ready_Flag.ctd_ready_flag = true;
	}
	else
	{
		Ready_Flag.ctd_ready_flag = false;
	}

}


void Planner::Get_CTD_State(bool flag)
{
	if(flag)
	{
		Mul_Sensor_State.ctd_state = 0x01;
	}
	else
	{
		Mul_Sensor_State.ctd_state = 0x00;
	}
}


void Planner::Get_Depth_Data(threadsafe_queue<Analy_Depth_Data::Depth_Info_Nvidia> & depth_que)
{
    if(depth_que.size()>0)
    {
		Cur_Recv_Num.depth_num++;
		if(debug_flag)
		{
			printf(" planner receive Depth data ================\n");	
		}
		depth_que.try_pop(framedata.depth_framedata);
		Sensor_Conti_Error_DataNum.depth_num = 0;
    }
    else
    {
		Sensor_Conti_Error_DataNum.depth_num++;
    }

	if((Cur_Recv_Num.depth_num>=1)&&(Sensor_Conti_Error_DataNum.depth_num<Max_Conti_Error_Num.depth_num))
	{
		Ready_Flag.depth_ready_flag = true;
	}
	else
	{
		Ready_Flag.depth_ready_flag = false;
	}

}

void Planner::Get_Depth_State(bool flag)
{
	if(flag)
	{
		Mul_Sensor_State.depth_state = 0x01;
	}
	else
	{
		Mul_Sensor_State.depth_state = 0x00;
	}
}

void Planner::Get_Pressure_Data(threadsafe_queue<Analy_Pressure_Data::Pressure_Info_Nvidia> & pressure_que)
{
    if(pressure_que.size()>0)
    {
		Cur_Recv_Num.pressure_num++;
		if(debug_flag)
		{
			printf(" planner receive Pressure data ================\n");	
		}
		pressure_que.try_pop(framedata.pressure_framedata);
		Sensor_Conti_Error_DataNum.pressure_num = 0;
    }
    else
    {
		Sensor_Conti_Error_DataNum.pressure_num++;
    }

	if((Cur_Recv_Num.pressure_num>=1)&&(Sensor_Conti_Error_DataNum.pressure_num<Max_Conti_Error_Num.pressure_num))
	{
		Ready_Flag.pressure_ready_flag = true;
	}
	else
	{
		Ready_Flag.pressure_ready_flag = false;
	}

}

void Planner::Get_Pressure_State(bool flag)
{
	if(flag)
	{
		Mul_Sensor_State.pressure_state = 0x01;
	}
	else
	{
		Mul_Sensor_State.pressure_state = 0x00;
	}
}

void Planner::Get_USBL_Data(threadsafe_queue<Analy_USBL_Data::USBL_Info_Nvidia> & usbl_que)
{
    if(usbl_que.size()>0)
    {
		Cur_Recv_Num.usbl_num++;
		if(debug_flag)
		{
			printf(" planner receive USBL data ================\n");	
		}
		usbl_que.try_pop(framedata.usbl_framedata);
		Sensor_Conti_Error_DataNum.usbl_num = 0;
    }
    else
    {
		Sensor_Conti_Error_DataNum.usbl_num++;
    }

	if((Cur_Recv_Num.usbl_num>=1)&&(Sensor_Conti_Error_DataNum.usbl_num<Max_Conti_Error_Num.usbl_num))
	{
		Ready_Flag.usbl_ready_flag = true;
	}
	else
	{
		Ready_Flag.usbl_ready_flag = false;
	}

}


void Planner::Get_USBL_State(bool flag)
{
	if(flag)
	{
		Mul_Sensor_State.pressure_state = aly_usbl_data->enu_pos_invalid_state;
	}
	else
	{
		Mul_Sensor_State.pressure_state = aly_usbl_data->no_data_state;
	}
}


void Planner::Get_IMU_State(void)
{
	for(int i=0;i<2;i++)
	{
		Nav_Output_Data.imu_state[i] = 0x00;
	}

	float gyro_data_x = (float)framedata.sinsframedata.sins_data.gyro_data[0]*0.001/3600;  //deg/s
	float gyro_data_y = (float)framedata.sinsframedata.sins_data.gyro_data[1]*0.001/3600;  //deg/s
	float gyro_data_z = (float)framedata.sinsframedata.sins_data.gyro_data[2]*0.001/3600;  //deg/s
	float acce_data_x = (float)framedata.sinsframedata.sins_data.acce_data[0]*0.00001; // m/s/s
	float acce_data_y = (float)framedata.sinsframedata.sins_data.acce_data[1]*0.00001; // m/s/s
	float acce_data_z = (float)framedata.sinsframedata.sins_data.acce_data[2]*0.00001; // m/s/s

	float gyro_tmp_x = (float)framedata.sinsframedata.sins_data.gyro_tmp[0]*0.0625;
	float gyro_tmp_y = (float)framedata.sinsframedata.sins_data.gyro_tmp[1]*0.0625;
	float gyro_tmp_z = (float)framedata.sinsframedata.sins_data.gyro_tmp[2]*0.0625;
	float acce_tmp = (float)framedata.sinsframedata.sins_data.acce_tmp[0]*0.0625;

	if(abs(gyro_data_x)>=aly_sins_data->max_gyro_data)
	{
		Nav_Output_Data.imu_state[0] |= (0x01<<(aly_sins_data->imu_state.gyro_x_exceed%8));
	}
	if(abs(gyro_data_y)>=aly_sins_data->max_gyro_data)
	{
		Nav_Output_Data.imu_state[0] |= (0x01<<(aly_sins_data->imu_state.gyro_y_exceed%8));
	}
	if(abs(gyro_data_z)>=aly_sins_data->max_gyro_data)
	{
		Nav_Output_Data.imu_state[0] |= (0x01<<(aly_sins_data->imu_state.gyro_z_exceed%8));
	}
	if(abs(acce_data_x)>=aly_sins_data->max_acce_data)
	{
		Nav_Output_Data.imu_state[0] |= (0x01<<(aly_sins_data->imu_state.acce_x_exceed%8));
	}
	if(abs(acce_data_y)>=aly_sins_data->max_acce_data)
	{
		Nav_Output_Data.imu_state[0] |= (0x01<<(aly_sins_data->imu_state.acce_y_exceed%8));
	}
	if(abs(acce_data_z)>=aly_sins_data->max_acce_data)
	{
		Nav_Output_Data.imu_state[0] |= (0x01<<(aly_sins_data->imu_state.acce_z_exceed%8));
	}

	if(gyro_tmp_x>aly_sins_data->max_temp||\
		gyro_tmp_y>aly_sins_data->max_temp||\
		gyro_tmp_z>aly_sins_data->max_temp||\
		acce_tmp>aly_sins_data->max_temp
	)
	{
		Nav_Output_Data.imu_state[1] |= (0x01<<(aly_sins_data->imu_state.temp_exceed%8));
	}

}


void Planner::Assembly(void)
{
	for(int i=0;i<4;i++)
	{  
		Nav_Output_Data.packup[i] = 0x00;
	}

	if(Ready_Flag.pressure_ready_flag) // alt data
	{
		Nav_Output_Data.nav_state[3] = 0x00;
		Nav_Output_Data.nav_state[3] |= (0x01<<(navigation_state.ref_alt_state.pressure_alt%8));
		Nav_Output_Data.alt = framedata.pressure_framedata.pressure_data.data*10;  // m
	}
	
	Cal_Nav_Mode(udp_set_nav_mode);

	if(Ready_Flag.sins_ready_flag) // imu data is invalid
	{
		for(int i=0;i<3;i++)
		{
			Nav_Output_Data.angle[i] = (float)framedata.sinsframedata.sins_data.angle[i]*0.000001;	
			Nav_Output_Data.gyro_data[i] = (float)framedata.sinsframedata.sins_data.gyro_data[i]*0.001;	
			Nav_Output_Data.acce_data[i] = (float)framedata.sinsframedata.sins_data.acce_data[i]*0.00001;
		}	
		Nav_Output_Data.timestamp = (float)framedata.sinsframedata.sins_data.timestamp*0.000001;
	}
	// analy imu data
	Get_IMU_State();

	Nav_Output_Data.dvl_state = Mul_Sensor_State.dvl_state;
	Nav_Output_Data.gps_state = Mul_Sensor_State.gps_state;
	Nav_Output_Data.usbl_state = Mul_Sensor_State.usbl_state;

	if(Mul_Sensor_State.sins_state==aly_sins_data->ready_state) // invalid sins 
	{ 
		Init_Fusion_data(udp_set_nav_mode,real_dr_data);
		sins_dvl_dr->Cal_Vel_n(real_dr_data);
		for(int i=0;i<3;i++)
		{
			Nav_Output_Data.vel[i] = sins_dvl_dr->dvl_vel_n[i];  // m/s
		}
		sins_dvl_dr->Cal_Pos_n(real_dr_data);
		Nav_Output_Data.lon = sins_dvl_dr->dr_pos_lon_lat[0];
		Nav_Output_Data.lat = sins_dvl_dr->dr_pos_lon_lat[1];
	}

}






void Planner::Init_DR_data(sins_dvl_DR::Sins_DVl_Gps_USBL_Data & nav_data)
{
	nav_data.timestamp =(float)framedata.sinsframedata.sins_data.timestamp*0.000001; ;
	for(int i=0;i<3;i++)
	{
		nav_data.angle_data[i] = (float)framedata.sinsframedata.sins_data.angle[i]*0.000001; //deg
		nav_data.dvl_data[i] = (float)framedata.dvl_framedata.dvl_vel[i]*0.001; // m/s;
	}
	nav_data.gps_lon_data = (float)framedata.gpsframedata.lon*0.0000001;
	nav_data.gps_lat_data = (float)framedata.gpsframedata.lat*0.0000001;
	nav_data.gps_vel_data = (float)framedata.gpsframedata.speed*0.001/3.6 ;	
	nav_data.gps_star_data = framedata.gpsframedata.star;	
	nav_data.gps_flag = Mul_Sensor_State.gps_state;	
	nav_data.dvl_flag = Mul_Sensor_State.dvl_state;
	nav_data.lon_lat_usbl[0] = (double)framedata.usbl_framedata.usbl_data.lon_gps;
	nav_data.lon_lat_usbl[1] = (double)framedata.usbl_framedata.usbl_data.lat_gps;
	nav_data.usbl_flag= Mul_Sensor_State.usbl_state;
}


void Planner::Init_Fusion_data(const char & nav_mode,sins_dvl_DR::Sins_DVl_Gps_USBL_Data & nav_data)
{
	nav_data.timestamp =(float)framedata.sinsframedata.sins_data.timestamp*0.000001; ;
	for(int i=0;i<3;i++)
	{
		nav_data.angle_data[i] = (float)framedata.sinsframedata.sins_data.angle[i]*0.000001; //deg
		nav_data.dvl_data[i] = (float)framedata.dvl_framedata.dvl_vel[i]*0.001; // m/s;
	}
	nav_data.gps_lon_data = (float)framedata.gpsframedata.lon*0.0000001;
	nav_data.gps_lat_data = (float)framedata.gpsframedata.lat*0.0000001;
	nav_data.gps_vel_data = (float)framedata.gpsframedata.speed*0.001/3.6 ;	
	nav_data.gps_star_data = framedata.gpsframedata.star;	
	nav_data.lon_lat_usbl[0] = (double)framedata.usbl_framedata.usbl_data.lon_gps;
	nav_data.lon_lat_usbl[1] = (double)framedata.usbl_framedata.usbl_data.lat_gps;

	//set  gps flag
	if(nav_mode==navigation_state.sins_dvl_state) // sins/dvl
	{
		if(first_gps_sins_dvl_mode_flag)
		{
			nav_data.gps_flag = Mul_Sensor_State.gps_state;	
		}
		else
		{
			nav_data.gps_flag = aly_gps_data->valid_data_state;	 	
		}
		if((first_gps_sins_dvl_mode_flag)&&
		(Mul_Sensor_State.gps_state==aly_gps_data->pos_vel_invalid_state||
		Mul_Sensor_State.gps_state==aly_gps_data->pos_invalid_state))
		{
			first_gps_sins_dvl_mode_flag = false;
		}

	}
	else if(nav_mode==navigation_state.pure_sins_state||
				 nav_mode==navigation_state.sins_usbl_state||
				 nav_mode==navigation_state.sins_dvl_usbl_state)
	{
		nav_data.gps_flag = aly_gps_data->valid_data_state;
	}
	else
	{
		nav_data.gps_flag = Mul_Sensor_State.gps_state;	
	}

	//set  usbl flag
	if(nav_mode==navigation_state.pure_sins_state||
		nav_mode==navigation_state.sins_dvl_state||
		nav_mode==navigation_state.sins_gps_state||
		nav_mode==navigation_state.sins_dvl_gps_state)
	{
		nav_data.usbl_flag = aly_usbl_data->valid_data_state;
	}
	else
	{
		nav_data.usbl_flag = Mul_Sensor_State.usbl_state;
	}

	//set  dvl flag
	if(nav_mode==navigation_state.pure_sins_state||
		nav_mode==navigation_state.sins_gps_state||
		nav_mode==navigation_state.sins_usbl_state) 
	{
		nav_data.dvl_flag = aly_dvl_data->valid_data_state;
	}
	else
	{
		nav_data.dvl_flag = Mul_Sensor_State.dvl_state;
	}

}



void Planner::Cal_Nav_Mode(const char &  set_nav_mode)
{
	if(Ready_Flag.sins_ready_flag==false)
	{
		return;
	}
	for(int i=0;i<4;i++)
	{
		Nav_Output_Data.nav_state[i] = 0x00;
	}

	if(Mul_Sensor_State.sins_state<aly_sins_data->ready_state) // sins is not ready
	{
		
		Nav_Output_Data.nav_state[0] = 0x00;
		Nav_Output_Data.nav_state[0] |= (0x01<<(Mul_Sensor_State.sins_state%8));
		Mul_Sensor_State.nav_state = Mul_Sensor_State.sins_state;
	}
	else if(Mul_Sensor_State.sins_state==aly_sins_data->ready_state)
	{
		// init_state = 0x00
		if(set_nav_mode==0x00||set_nav_mode==navigation_state.sins_dvl_usbl_gps_state) // sins/dvl/usbl/gps
		{
			Nav_Output_Data.nav_state[0] = 0x00;
			Nav_Output_Data.nav_state[0] |= (0x01<<(navigation_state.sins_dvl_usbl_gps_state%8));
			Mul_Sensor_State.nav_state = navigation_state.sins_dvl_usbl_gps_state;
		}
		else if(set_nav_mode==navigation_state.pure_sins_state) //sins
		{
			Nav_Output_Data.nav_state[0] = 0x00;
			Nav_Output_Data.nav_state[0] |= (0x01<<(navigation_state.pure_sins_state%8));
			Mul_Sensor_State.nav_state = navigation_state.pure_sins_state;
		}
		else if(set_nav_mode==navigation_state.sins_dvl_state) //sins/dvl
		{
			Nav_Output_Data.nav_state[0] = 0x00;
			Nav_Output_Data.nav_state[0] |= (0x01<<(navigation_state.sins_dvl_state%8));
			Mul_Sensor_State.nav_state = navigation_state.sins_dvl_state;
		}
		else if(set_nav_mode==navigation_state.sins_gps_state) //sins/gps
		{
			Nav_Output_Data.nav_state[0] = 0x00;
			Nav_Output_Data.nav_state[0] |= (0x01<<(navigation_state.sins_gps_state%8));
			Mul_Sensor_State.nav_state = navigation_state.sins_gps_state;
		}
		else if(set_nav_mode==navigation_state.sins_usbl_state) //sins/usbl
		{
			Nav_Output_Data.nav_state[0] = 0x00;
			Nav_Output_Data.nav_state[0] |= (0x01<<(navigation_state.sins_usbl_state%8));
			Mul_Sensor_State.nav_state = navigation_state.sins_usbl_state;
		}
		else if(set_nav_mode==navigation_state.sins_dvl_gps_state) //sins/dvl/gps
		{
			Nav_Output_Data.nav_state[0] = 0x00;
			Nav_Output_Data.nav_state[0] |= (0x01<<(navigation_state.sins_dvl_gps_state%8));
			Mul_Sensor_State.nav_state = navigation_state.sins_dvl_gps_state;
		}
		else if(set_nav_mode==navigation_state.sins_dvl_usbl_state) //sins/dvl/usbl
		{
			Nav_Output_Data.nav_state[0] = 0x00;
			Nav_Output_Data.nav_state[0] |= (0x01<<(navigation_state.sins_dvl_usbl_state%8));
			Mul_Sensor_State.nav_state = navigation_state.sins_dvl_usbl_state;
		}	
		else // sins/dvl/usbl/gps
		{
			Nav_Output_Data.nav_state[0] = 0x00;
			Nav_Output_Data.nav_state[0] |= (0x01<<(navigation_state.sins_dvl_usbl_gps_state%8));
			Mul_Sensor_State.nav_state = navigation_state.sins_dvl_usbl_gps_state;
		}

	}

}





void Planner::Nav2Ctrl_UDP(void)
{	
		udp2ctrol->Send_Encoder_Data_ROV((char*)&Nav_Output_Data,sizeof(Nav_Output_Data),NAV_Msg_ID,\
		udp2ctrol->navigation_master_id,udp2ctrol->control_sub_id);
}



void Planner::Test_Nav2Ctrl_UDP(void)
{	
	static int counter_num = 0;

	Nav2CtrlData_mtx.lock();
	Nav_Output_Data.lon = (float)framedata.sinsframedata.sins_data.fusion_lon*0.000001;
	Nav_Output_Data.lat = (float)framedata.sinsframedata.sins_data.fusion_lat*0.000001;
	Nav_Output_Data.alt = (float)framedata.sinsframedata.sins_data.fusion_alt*0.001;
	Nav_Output_Data.x_ship = 0;
	Nav_Output_Data.y_ship = 0;
	Nav_Output_Data.z_ship = 0;
	for(int i=0;i<3;i++)
	{
		Nav_Output_Data.vel[i] = (float)framedata.sinsframedata.sins_data.fusion_vel[i]*0.001;
		Nav_Output_Data.angle[i] = (float)framedata.sinsframedata.sins_data.angle[i]*0.000001;	
		Nav_Output_Data.gyro_data[i] = (float)framedata.sinsframedata.sins_data.gyro_data[i]*0.001;	
		Nav_Output_Data.acce_data[i] = (float)framedata.sinsframedata.sins_data.acce_data[i]*0.00001;
	}	
	Nav_Output_Data.timestamp = (float)framedata.sinsframedata.sins_data.timestamp*0.000001;

	for(int i=0;i<13;i++)
	{
		Nav_Output_Data.packup[i] = 1;
	}
	Nav2CtrlData_mtx.unlock();

	counter_num++;
	if(counter_num==1)
	{
		counter_num = 0;
    	char master_id = 0x03; //me navigation
    	char sub_id  = 0x02; // control
		udp2ctrol->Send_Encoder_Data_ROV((char*)&Nav_Output_Data,sizeof(Nav_Output_Data),NAV_Msg_ID,master_id,sub_id);
	}
}


void Planner::Test2_Nav2Ctrl_UDP(void)
{	
	printf("==========================Test2_Nav2Ctrl_UDP====================\n");
	static int counter_num = 0;
	if(counter_num%2==0)
	{
		Nav2CtrlData_mtx.lock();
		Nav_Output_Data.lon = 116;
		Nav_Output_Data.lat = 40;
		Nav_Output_Data.alt = 0;
		Nav_Output_Data.x_ship = 100;
		Nav_Output_Data.y_ship = 200;
		Nav_Output_Data.z_ship = 0;
		for(int i=0;i<3;i++)
		{
			Nav_Output_Data.vel[i] = 1;
			Nav_Output_Data.angle[i] = 10;	
			Nav_Output_Data.gyro_data[i] = 100;	
			Nav_Output_Data.acce_data[i] = 9.8;
		}	
		Nav_Output_Data.timestamp = 0;
		for(int i=0;i<13;i++)
		{
			Nav_Output_Data.packup[i] = 1;
		}
		Nav2CtrlData_mtx.unlock();
	}
	else
	{
		Nav2CtrlData_mtx.lock();
		Nav_Output_Data.lon = 120;
		Nav_Output_Data.lat = 30;
		Nav_Output_Data.alt = 0;
		Nav_Output_Data.x_ship = 200;
		Nav_Output_Data.y_ship = 100;
		Nav_Output_Data.z_ship = 0;
		for(int i=0;i<3;i++)
		{
			Nav_Output_Data.vel[i] = 2;
			Nav_Output_Data.angle[i] = 20;	
			Nav_Output_Data.gyro_data[i] = 300;	
			Nav_Output_Data.acce_data[i] = 9.7;
		}	
		Nav_Output_Data.timestamp = 1;
		for(int i=0;i<13;i++)
		{
			Nav_Output_Data.packup[i] = 0;
		}
		Nav2CtrlData_mtx.unlock();
	}
	counter_num++;
	if(counter_num%1==0)
	{
    	char master_id = 0x03; //me navigation
    	char sub_id  = 0x02; // control
		printf("@@@@@@@@@@@@@@@@@@@@@@@@@@@Send_Encoder_Data_ROV@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@counter_num = %d\n",counter_num);
		udp2ctrol->Send_Encoder_Data_ROV((char*)&Nav_Output_Data,sizeof(Nav_Output_Data),NAV_Msg_ID,master_id,sub_id);
	}
}




long int Planner::Get_Cur_Second(void)
{
    struct timeval tv;
    gettimeofday(&tv,NULL);
    long int cur_time = tv.tv_sec;
    return(cur_time);
}


void Planner::Save_NAV_Data(void)
{
	static int counter_num = 0;
    time_t timep;
    struct tm *p;
    time(&timep);
    p = gmtime(&timep);
    fprintf(location_filestream,"%04d-",1900+p->tm_year);
    fprintf(location_filestream,"%02d-",1+p->tm_mon);
    fprintf(location_filestream,"%02d-",p->tm_mday);
    fprintf(location_filestream,"%02d-",p->tm_hour);
    fprintf(location_filestream,"%02d-",p->tm_min);
    fprintf(location_filestream,"%02d ",p->tm_sec);

	fprintf(location_filestream,"%d ",Mul_Sensor_State.sins_state); 
	fprintf(location_filestream,"%d  ",Mul_Sensor_State.mems_sins_state); 
	fprintf(location_filestream,"%d  ",Mul_Sensor_State.dvl_state); 
	fprintf(location_filestream,"%d  ",Mul_Sensor_State.usbl_state); 
	fprintf(location_filestream,"%d  ",Mul_Sensor_State.gps_state); 
	fprintf(location_filestream,"%d  ",Mul_Sensor_State.ctd_state); 
	fprintf(location_filestream,"%d  ",Mul_Sensor_State.depth_state); 
	fprintf(location_filestream,"%d  ",Mul_Sensor_State.pressure_state); 	
	fprintf(location_filestream,"%d  ",Mul_Sensor_State.nav_state); 	
	fprintf(location_filestream,"%d  ",first_gps_sins_dvl_mode_flag); 

	
	fprintf(location_filestream,"\n"); 

	counter_num++;
	if(counter_num==100)
	{
		counter_num = 0;
		fflush(location_filestream);
	}
	fflush(location_filestream);
}



void Planner::Save_Nav2Ctrl_Data(void)
{
	static int counter_num = 0;
    time_t timep;
    struct tm *p;
    time(&timep);
    p = gmtime(&timep);
    fprintf(nav2ctrl_filestream,"%04d-",1900+p->tm_year);
    fprintf(nav2ctrl_filestream,"%02d-",1+p->tm_mon);
    fprintf(nav2ctrl_filestream,"%02d-",p->tm_mday);
    fprintf(nav2ctrl_filestream,"%02d-",p->tm_hour);
    fprintf(nav2ctrl_filestream,"%02d-",p->tm_min);
    fprintf(nav2ctrl_filestream,"%02d ",p->tm_sec);
	fprintf(nav2ctrl_filestream,"%.8lf ",(double)Nav_Output_Data.lon); 
	fprintf(nav2ctrl_filestream,"%.8lf ",(double)Nav_Output_Data.lat); 
	fprintf(nav2ctrl_filestream,"%.6f ",(float)Nav_Output_Data.alt); 
	fprintf(nav2ctrl_filestream,"%.6f ",(float)Nav_Output_Data.x_ship); 
	fprintf(nav2ctrl_filestream,"%.6f ",(float)Nav_Output_Data.y_ship);
	fprintf(nav2ctrl_filestream,"%.6f ",(float)Nav_Output_Data.z_ship);	 
	fprintf(nav2ctrl_filestream,"%.6f ",Nav_Output_Data.vel[0]); 
	fprintf(nav2ctrl_filestream,"%.6f ",Nav_Output_Data.vel[1]); 
	fprintf(nav2ctrl_filestream,"%.6f ",Nav_Output_Data.vel[2]); 
	fprintf(nav2ctrl_filestream,"%.6f ",Nav_Output_Data.angle[0]); 
	fprintf(nav2ctrl_filestream,"%.6f ",Nav_Output_Data.angle[1]); 
	fprintf(nav2ctrl_filestream,"%.6f ",Nav_Output_Data.angle[2]); 
	fprintf(nav2ctrl_filestream,"%.6f ",Nav_Output_Data.gyro_data[0]); 
	fprintf(nav2ctrl_filestream,"%.6f ",Nav_Output_Data.gyro_data[1]); 
	fprintf(nav2ctrl_filestream,"%.6f ",Nav_Output_Data.gyro_data[2]); 
	fprintf(nav2ctrl_filestream,"%.6f ",Nav_Output_Data.acce_data[0]); 
	fprintf(nav2ctrl_filestream,"%.6f ",Nav_Output_Data.acce_data[1]); 
	fprintf(nav2ctrl_filestream,"%.6f ",Nav_Output_Data.acce_data[2]); 
	fprintf(nav2ctrl_filestream,"%.6f ",Nav_Output_Data.timestamp); 
	fprintf(nav2ctrl_filestream,"\n"); 

	counter_num++;
	if(counter_num==50)
	{
		counter_num = 0;
		fflush(nav2ctrl_filestream);
	}
	fflush(nav2ctrl_filestream);
}