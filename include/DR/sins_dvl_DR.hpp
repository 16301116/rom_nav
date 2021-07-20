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


using namespace std;

class sins_dvl_DR
{

public:

	Vec3f old_dvl_vel_n,old_dvl_vel_b,dvl_vel_n,dvl_vel_b,cur_vel_DR,old_vel_DR;
	Vec2f DR_Pos;
	float max_vel = 5;
	bool first_gps_flag = false;
	int gps_num = 0;
	int dvl_num = 0;
	int dvl_num_continue = 0;
	float last_new_gps[2] = {0};

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
	#pragma pack(1)

    sins_dvl_DR();
	~sins_dvl_DR();
    vector<vector<float>> DR(const std::vector<Old_SINS_DVL_Data> & sins_dvl_data,const VecXf & set_para);
	vector<vector<float>> DR(const std::vector<New_SINS_DVL_Data> & sins_dvl_data,const VecXf & set_para);
	Vec2f DR_Realtime(const Real_DR_Data & nav_data,const VecXf & set_para);
	Vec2f DR_Realtime2(const Real_DR_Data & nav_data,const VecXf & set_para);
	Vec2f DR_Realtime_ROV(const Real_DR_Data & nav_data,const VecXf & set_para);
	void Save_DR_Data(const Real_DR_Data & nav_data) const;

 	Real_DR_Data new_dr_data;
	Real_DR_Data old_dr_data;

private:

    

protected:

};



#endif
