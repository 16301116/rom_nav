#ifndef  H_SINS_DVL_DR_H
#define  H_SINS_DVL_DR_H

#include <iostream>
#include <vector>
#include <fstream>
#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "lib/earth_para.hpp"
#include "lib/eigen_types.h"
#include "lib/CoordTrans.hpp"
#include "lib/earth_para.hpp"
#include "lib/file_utilities.hpp"
#include "sensor/Analy_DVL_Data.h"
#include "sensor/Analy_GPS_Data.h"
#include "sensor/Analy_USBL_Data.h"

using namespace std;

class sins_dvl_DR
{

public:
	Analy_DVL_Data * aly_dvl_data;
	Analy_GPS_Data * aly_gps_data;
	Analy_USBL_Data * aly_usbl_data;

	Vec3f old_dvl_vel_n,old_dvl_vel_b,dvl_vel_n,dvl_vel_b;
	double old_nav_time_index = 0;
	double T_Max = 20 ; //s
	double T = 0;
	double dr_pos_lon_lat[2] = {0};
	double dr_pos_m[2] = {0};
	bool first_gps_flag = false;
	int gps_num = 0;
	int dvl_num_continue = 0;
	double last_new_gps[2] = {0};

	char DR_file_name[100];
	FILE * DR_filestream;

	#pragma pack(1)
	struct Real_DR_Data 
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		double timestamp; 
		Eigen::Vector3d angle_data;
		float gps_lon_data;
		float gps_lat_data;
		float gps_vel_data;
		float gps_star_data;
		char gps_flag;
		Eigen::Vector3d dvl_data;
		char dvl_flag;
	};

	struct Sins_DVl_Gps_USBL_Data 
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		double timestamp; 
		Eigen::Vector3d angle_data;
		float gps_lon_data;
		float gps_lat_data;
		float gps_vel_data;
		float gps_star_data;
		char gps_flag;
		Eigen::Vector3d dvl_data;
		char dvl_flag;
		Eigen::Vector3d lon_lat_usbl;	
		char usbl_flag;
	};

	Sins_DVl_Gps_USBL_Data sins_dvl_gp_usbl_data;
	Sins_DVl_Gps_USBL_Data old_sins_dvl_gp_usbl_data;	

	struct DR_Data 
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		double timestamp; 
		Eigen::Vector3d angle_data;
		float last_lon_data;  //上一个数据
		float last_lat_data;   //上一个数据
		Eigen::Vector3d dvl_data;
		char dvl_flag;
	};
	Real_DR_Data new_dr_data;
	Real_DR_Data old_dr_data;
	#pragma pack(1)

    sins_dvl_DR();
	~sins_dvl_DR();
	void Init_Data(void);
	void Cal_Vel_n(const Sins_DVl_Gps_USBL_Data & nav_data);
	void Cal_Pos_n(const Sins_DVl_Gps_USBL_Data & nav_data);
	void DR_Realtime(const Sins_DVl_Gps_USBL_Data & nav_data);
	void DR_Realtime(const DR_Data & nav_data);
	void Save_DR_Data(const Sins_DVl_Gps_USBL_Data & nav_data) const;



private:

    

protected:

};



#endif
