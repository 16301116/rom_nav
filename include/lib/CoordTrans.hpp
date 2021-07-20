#ifndef __H__COORDTRANS__H__
#define __H__COORDTRANS__H__


#include <Eigen/Core>
#include <Eigen/Dense>
#include "lib/eigen_types.h"
/*
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
*/


#define   LonCoord_INDEX    0
#define   LatCoord_INDEX    1
#define   ALTITUDE_INDEX    2
#define   ECoord_INDEX      0
#define   NCoord_INDEX      1
#define   UCoord_INDEX      2
#define   SHIPX_INDEX       0
#define   SHIPY_INDEX       1
#define   SHIPZ_INDEX       2
#define   Coord_NUMS        2

#define   PI                3.14159265358979
#define   iPI               0.0174532925199433


struct Polar_Coord
{
	double Mag;  //与极点的距离
	double Rad;        //角度
};
 
struct Cart_Coord
{
	double x;
	double y;
};

class CoordTrans{
	
	public:
		
	CoordTrans();
	~CoordTrans();

    static float Transf_ECoordToNCoord(float Raw_Angle);
	static void ENcoordToLonLat(float E_Pos, float N_Pos, float * lonlat_coord);
	static void LonLatToENcoord(float longitude, float latitude, float *ENcoord);
	static void Cal_TwoDots_Info(float *Start_Dot, float *End_Dot, float & dis ,float & raw_rad_enu);
    static void Cal_TwoDots_Dis(float *Start_Dot, float *End_Dot, float & dis);
    static void Cal_TwoDots_Sita(float *Start_Dot, float *End_Dot,float & raw_rad_enu);
    void Cal_EndCoord(float *Start_Dot, float dis,float goal_rad,float * End_Dot);
    void Cal_SubEndCoord(float *Start_Dot, float * End_Dot,float dis,float * Sub_End_Dot);

	static float toRadian(float degree);
	static float toDegree(float radian);
	void Cart_To_Polar(Cart_Coord Cart,Polar_Coord & Polar);
    int Round(float r);

    static Eigen::Matrix<float,3,3> x_rot_matrix(float sita);
    static Eigen::Matrix<float,3,3> y_rot_matrix(float sita);
    static Eigen::Matrix<float,3,3> z_rot_matrix(float sita);
    static Eigen::Matrix<float,3,3> zxy_mutiple_matrix(float x_sita,float y_sita,float z_sita);
    static Eigen::Matrix<float,3,3> yxz_mutiple_matrix(float x_sita,float y_sita,float z_sita);

    static float CalEndRad_Zcoord(float begin_rad,float add_rad);
    static float CalEndAngle_Zcoord(float begin_angle,float add_angle);
    static float CalAddRad_Zcoord(float begin_rad,float end_rad);
    static float CalAddAngle_Zcoord(float begin_angle,float end_angle);
    static float CalAverRad_Zcoord(float rad1,float rad2);
    static void GetAddPos(float * begin_lonlat,float e_add_pos,float n_add_pos,float * end_lonlat);
    static float Yaw_Rad_N2ECoord(float Yaw_Rad);
    static float Yaw_Angle_N2ECoord(float Yaw_Angle);

    static Eigen::Matrix<float,3,3> a2mat(float pitch,float roll,float yaw);
    static void Lonlat_Add_M(float * begin_lonlat,float * end_lonlat,float * add_m);
	

	//add 
	static float Cal_SideError(float set_yaw_rad,float cur_yaw_rad,float dis);

	static double compress_earth;
    static int Re;
    static float RAD;
    static double Rm(float Lat);
    static double Rn(float Lat);

	static void Cal_TwoDots_Info2(float *Start_Dot, float *End_Dot, float & dis ,float & raw_rad_enu );
	static void Lonlat_Add_M2(float * begin_lonlat,float * end_lonlat,float * add_m);
	static void GetAddPos2(float * begin_lonlat,float e_add_pos,float n_add_pos,float * end_lonlat);
    static void GetAddPos2(double * begin_lonlat,double e_add_pos,double n_add_pos,double * end_lonlat);
    static void Cal_TwoDots_Dis2(float *Start_Dot, float *End_Dot, float & dis);
    static void Cal_TwoDots_Sita2(float *Start_Dot, float *End_Dot,float & raw_rad_enu);

    static void change_data(int & data1,int & data2);
    static std::vector<int> Sort_topK(std::vector<int> & data, int K);
    static void Polar2Cart(Polar_Coord Polar, Cart_Coord & Cart);
   // static void MatXXf2Mat3(MatXXf & src,cv::Mat & dst,int type);

	private:

	protected:
	

};

#endif


