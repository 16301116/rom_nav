#include "nav/sins_dvl_DR.hpp"

using namespace  std;

sins_dvl_DR::sins_dvl_DR()
{
	aly_dvl_data = new Analy_DVL_Data();
	aly_gps_data = new Analy_GPS_Data();

	time_t currtime =time(NULL);
	tm *sins_p =localtime(&currtime);
	sprintf(DR_file_name,"../data/output_data/DR_data%04d%02d%02d%02d%02d%02d.txt",1900+sins_p->tm_year,1+sins_p->tm_mon,sins_p->tm_mday,sins_p->tm_hour,sins_p->tm_min,sins_p->tm_sec);	
	DR_filestream = fopen(DR_file_name,"w+");
	if (DR_filestream==NULL)
	{
		printf("can't create file to save DR data!\n");
	}
	else
	{
		printf("successful create file to save DR data!\n");
	}
	
}


void sins_dvl_DR::Init_Data(void)
{
	memset(&new_dr_data,0,sizeof(new_dr_data));
	memset(&old_dr_data,0,sizeof(old_dr_data));
	old_dvl_vel_n<<0,0,0;
	old_dvl_vel_b<<0,0,0;
	dvl_vel_n<<0,0,0;
	dvl_vel_b<<0,0,0;
}

sins_dvl_DR::~sins_dvl_DR()
{
	delete aly_dvl_data;
	delete aly_gps_data;
	fclose(DR_filestream);
}

void sins_dvl_DR::Cal_Vel_n(const Sins_DVl_Gps_USBL_Data & nav_data)
{
	float dvl_k = 1;
	float mount_angle = 0;
	Mat33f Cb2n;
	float roll_angle,roll_raddata,pitch_angle,pitch_raddata,yaw_angle,yaw_raddata;

	char dvl_flag = nav_data.dvl_flag;
	pitch_angle = nav_data.angle_data[0];
    pitch_raddata = pitch_angle / earth_para::RAD;
    roll_angle =  nav_data.angle_data[1];
    roll_raddata = roll_angle / earth_para::RAD;
	yaw_angle = nav_data.angle_data[2];
    yaw_angle = CoordTrans::CalEndAngle_Zcoord(yaw_angle,mount_angle) ;//mount_yaw_angle;
    yaw_raddata = yaw_angle / earth_para::RAD;
	Cb2n = CoordTrans::a2mat(pitch_raddata,roll_raddata,yaw_raddata);
    dvl_vel_b[0] = nav_data.dvl_data[0]*dvl_k;   //探底数据
    dvl_vel_b[1] = nav_data.dvl_data[1]*dvl_k;
    dvl_vel_b[2] = nav_data.dvl_data[2]*dvl_k;

	if(dvl_flag==aly_dvl_data->vel_bottom_invalid_state||dvl_flag==aly_dvl_data->vel_water_invalid_state)   //修改 
	{	
		dvl_num_continue++;
		dvl_vel_n = Cb2n*dvl_vel_b;
	}
	else
	{
		dvl_vel_b = old_dvl_vel_b;
		dvl_vel_n = old_dvl_vel_n;
	}
     
	//update velocity
	float abs_dvl_vel_n_x =abs(dvl_vel_n[0]);
	float abs_dvl_vel_n_y = abs(dvl_vel_n[1]);
	float abs_dvl_vel_n_z = abs(dvl_vel_n[2]);
	if(abs_dvl_vel_n_x>=aly_dvl_data->max_vel) //abnormal situation
	{
		dvl_vel_n[0] = old_dvl_vel_n[0];
	}	
	if(abs_dvl_vel_n_y>=aly_dvl_data->max_vel) //abnormal situation
	{
		dvl_vel_n[1] = old_dvl_vel_n[1];
	}	
	if(abs_dvl_vel_n_z>=aly_dvl_data->max_vel) //abnormal situation
	{
		dvl_vel_n[2] = old_dvl_vel_n[2];
	}

	if(dvl_flag==aly_dvl_data->vel_bottom_invalid_state||dvl_flag==aly_dvl_data->vel_water_invalid_state)   
	{
		old_dvl_vel_n = dvl_vel_n;
		old_dvl_vel_b = dvl_vel_b;
	}
}



void sins_dvl_DR::Cal_Pos_n(const Sins_DVl_Gps_USBL_Data & nav_data)
{
    double nav_time_index = nav_data.timestamp;
	float lon =  nav_data.gps_lon_data;
	float lat =  nav_data.gps_lat_data;
	char gps_flag = nav_data.gps_flag;
	char usbl_flag= nav_data.usbl_flag;

	// usbl data is invalid
	if(usbl_flag==aly_usbl_data->enu_pos_invalid_state)
	{
		dr_pos_m[0] = 0; // reset
		dr_pos_m[1] = 0; // reset
		dr_pos_lon_lat[0] = (double)nav_data.lon_lat_usbl[0];
	 	dr_pos_lon_lat[1] = (double)nav_data.lon_lat_usbl[1];
		 return;
	}

	if(gps_flag==aly_gps_data->pos_vel_invalid_state||gps_flag==aly_gps_data->pos_invalid_state) 
	{
		gps_num++;
		first_gps_flag = true; // the first gps data
		last_new_gps[0] = lon;
	 	last_new_gps[1] = lat;
		dr_pos_m[0] = 0; // reset
		dr_pos_m[1] = 0; // reset
		dr_pos_lon_lat[0] = lon;
	 	dr_pos_lon_lat[1] = lat;
		dvl_num_continue = 0;
	}
	if(first_gps_flag = false)
	{
		dr_pos_m[0] = 0; 
		dr_pos_m[1] = 0; 
		dr_pos_lon_lat[0] = 0;
	 	dr_pos_lon_lat[1] = 0;
	}

	if(dvl_num_continue==0)
	{
		old_nav_time_index = nav_time_index;
	}
	T = abs(nav_time_index-old_nav_time_index); // s
	old_nav_time_index = nav_time_index;
	if(abs(T)>=T_Max) //abnormal situation
	{
		T = 0;
	}
	
	if((gps_flag!=aly_gps_data->pos_vel_invalid_state)&&(gps_flag!=aly_gps_data->pos_invalid_state)) //not have gps data
	{
		float add_pos_dr_x = dvl_vel_n[0]*T;
		float add_pos_dr_y = dvl_vel_n[1]*T;
		dr_pos_m[0]+=add_pos_dr_x;
		dr_pos_m[1]+=add_pos_dr_y;

		double end_lon_lat[2];
		CoordTrans::GetAddPos2(last_new_gps,dr_pos_m[0],dr_pos_m[1],end_lon_lat);
		dr_pos_lon_lat[0] = end_lon_lat[0];
		dr_pos_lon_lat[1] = end_lon_lat[1];
	}

	Save_DR_Data(nav_data);
}


// return DR_Pos
void sins_dvl_DR::DR_Realtime(const Sins_DVl_Gps_USBL_Data & nav_data)
{
    double nav_time_index = nav_data.timestamp;
	float lon =  nav_data.gps_lon_data;
	float lat =  nav_data.gps_lat_data;
	char gps_flag = nav_data.gps_flag;

	if(gps_flag==aly_gps_data->pos_vel_invalid_state||gps_flag==aly_gps_data->pos_invalid_state) 
	{
		gps_num++;
		first_gps_flag = true; // the first gps data
		last_new_gps[0] = lon;
	 	last_new_gps[1] = lat;
		dr_pos_m[0] = 0; // reset
		dr_pos_m[1] = 0; // reset
		dr_pos_lon_lat[0] = lon;
	 	dr_pos_lon_lat[1] = lat;
		dvl_num_continue = 0;
	}
	if(first_gps_flag = false)
	{
		dr_pos_m[0] = 0; 
		dr_pos_m[1] = 0; 
		dr_pos_lon_lat[0] = 0;
	 	dr_pos_lon_lat[1] = 0;
	}
	
	Cal_Vel_n(nav_data);

	if(dvl_num_continue==0)
	{
		old_nav_time_index = nav_time_index;
	}
	T = abs(nav_time_index-old_nav_time_index); // s
	if(abs(T)>=T_Max) //abnormal situation
	{
		T = 0;
	}
	
	if(gps_flag!=aly_gps_data->pos_vel_invalid_state&&gps_flag!=aly_gps_data->pos_invalid_state) //not have gps data
	{
		double add_pos_dr_x = dvl_vel_n[0]*T;
		double add_pos_dr_y = dvl_vel_n[1]*T;
		dr_pos_m[0]+=add_pos_dr_x;
		dr_pos_m[1]+=add_pos_dr_y;

		double end_lon_lat[2];
		CoordTrans::GetAddPos2(last_new_gps,dr_pos_m[0],dr_pos_m[1],end_lon_lat);
		dr_pos_lon_lat[0] = end_lon_lat[0];
		dr_pos_lon_lat[1] = end_lon_lat[1];
	}

	Save_DR_Data(nav_data);
}



// return DR_Pos
void sins_dvl_DR::DR_Realtime(const DR_Data & nav_data)
{
	Mat33f Cb2n;
	float roll_angle,roll_raddata,pitch_angle,pitch_raddata,yaw_angle,yaw_raddata;
    float dvl_k = 1;
	float mount_angle = 0;

    double nav_time_index = nav_data.timestamp;
	float lon =  nav_data.last_lon_data;
	float lat =  nav_data.last_lat_data;
	
	last_new_gps[0] = nav_data.last_lon_data;
	last_new_gps[1] = nav_data.last_lat_data;
    pitch_angle = nav_data.angle_data[0];
    pitch_raddata = pitch_angle / earth_para::RAD;
    roll_angle =  nav_data.angle_data[1];
    roll_raddata = roll_angle / earth_para::RAD;
	yaw_angle = nav_data.angle_data[2];
    yaw_angle = CoordTrans::CalEndAngle_Zcoord(yaw_angle,mount_angle) ;//mount_yaw_angle;
    yaw_raddata = yaw_angle / earth_para::RAD;
	Cb2n = CoordTrans::a2mat(pitch_raddata,roll_raddata,yaw_raddata);
    dvl_vel_b[0] = nav_data.dvl_data[0]*dvl_k;   //探底数据
    dvl_vel_b[1] = nav_data.dvl_data[1]*dvl_k;
    dvl_vel_b[2] = nav_data.dvl_data[2]*dvl_k;
	char dvl_flag = nav_data.dvl_flag;
	if(dvl_num_continue==0)
	{
		old_nav_time_index = nav_time_index;
	}
	T = abs(nav_time_index-old_nav_time_index); // s
	if(abs(T)>=T_Max) //abnormal situation
	{
		T = 0;
	}	
	if(dvl_flag==aly_dvl_data->vel_bottom_invalid_state||dvl_flag==aly_dvl_data->vel_water_invalid_state)   //修改 
	{	
		dvl_num_continue++;
		dvl_vel_n = Cb2n*dvl_vel_b;
    	dvl_vel_n[2] = 0;      //二维空间中运动 
	}
	else
	{
		dvl_vel_b = old_dvl_vel_b;
		dvl_vel_n = old_dvl_vel_n;
	}
     
	//update velocity
	float abs_dvl_vel_n_x =abs(dvl_vel_n[0]);
	float abs_dvl_vel_n_y = abs(dvl_vel_n[1]);
	if(abs_dvl_vel_n_x>=aly_dvl_data->max_vel) //abnormal situation
	{
		dvl_vel_n[0] = old_dvl_vel_n[0];
	}	
	if(abs_dvl_vel_n_y>=aly_dvl_data->max_vel) //abnormal situation
	{
		dvl_vel_n[1] = old_dvl_vel_n[1];
	}	

	float add_pos_dr_x = dvl_vel_n[0]*T;
	float add_pos_dr_y = dvl_vel_n[1]*T;

	CoordTrans::GetAddPos2(last_new_gps,add_pos_dr_x,add_pos_dr_y,dr_pos_lon_lat);

	if(dvl_flag==aly_dvl_data->vel_bottom_invalid_state||dvl_flag==aly_dvl_data->vel_water_invalid_state)   
	{
		old_dvl_vel_n = dvl_vel_n;
		old_dvl_vel_b = dvl_vel_b;
	}

	//Save_DR_Data(nav_data);
}


void sins_dvl_DR::Save_DR_Data(const Sins_DVl_Gps_USBL_Data & nav_data) const
{
	
	time_t timep;
	struct tm *p;
	time(&timep);
	p=gmtime(&timep);

	//output time
	fprintf(DR_filestream,"%04d-",1900+p->tm_year);
	fprintf(DR_filestream,"%02d-",1+p->tm_mon);
	fprintf(DR_filestream,"%02d-",p->tm_mday);
	fprintf(DR_filestream,"%02d-",p->tm_hour);
	fprintf(DR_filestream,"%02d-",p->tm_min);
	fprintf(DR_filestream,"%02d ",p->tm_sec);
	
	fprintf(DR_filestream,"%f ",(double)nav_data.timestamp);
	fprintf(DR_filestream,"%.4f ",nav_data.angle_data[0]);
	fprintf(DR_filestream,"%.4f ",nav_data.angle_data[1]);
	fprintf(DR_filestream,"%.4f ",nav_data.angle_data[2]);
	fprintf(DR_filestream,"%.6f ",nav_data.gps_lon_data);
	fprintf(DR_filestream,"%.6f ",nav_data.gps_lat_data);
	fprintf(DR_filestream,"%.4f ",nav_data.gps_vel_data);
	fprintf(DR_filestream,"%d ",(int)nav_data.gps_star_data);
	fprintf(DR_filestream,"%d ",nav_data.gps_flag);
	fprintf(DR_filestream,"%.4f ",nav_data.dvl_data[0]);
	fprintf(DR_filestream,"%.4f ",nav_data.dvl_data[1]);
	fprintf(DR_filestream,"%.4f ",nav_data.dvl_data[2]);
	fprintf(DR_filestream,"%d ",nav_data.dvl_flag);

	fprintf(DR_filestream,"%.6f ",dvl_vel_n[0]);
	fprintf(DR_filestream,"%.6f ",dvl_vel_n[1]);
	fprintf(DR_filestream,"%.6f ",dvl_vel_n[2]);

	fprintf(DR_filestream,"%.4f ",dr_pos_m[0]);
	fprintf(DR_filestream,"%.4f ",dr_pos_m[1]);
	fprintf(DR_filestream,"%.6f ",dr_pos_lon_lat[0]);
	fprintf(DR_filestream,"%.6f ",dr_pos_lon_lat[1]);

	fprintf(DR_filestream,"%.6f ",nav_data.lon_lat_usbl[0]);
	fprintf(DR_filestream,"%.6f ",nav_data.lon_lat_usbl[1]);
	fprintf(DR_filestream,"%d ",nav_data.usbl_flag);

	fprintf(DR_filestream,"%.6f ",T);
	
	fprintf(DR_filestream,"\n");
	fflush(DR_filestream);
}







