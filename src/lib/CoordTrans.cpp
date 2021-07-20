/*文件描述：坐标转换
 *
 *
 *
 * 
 */

#include <cmath>
#include <stdio.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "lib/CoordTrans.hpp"

CoordTrans::CoordTrans()
{

}


CoordTrans::~CoordTrans()
{

}




/*
 *函数功能：根据高斯投影由大地坐标反算经纬度
 *输入参数：大地坐标（E_Pos,N_Pos,U_Pos轴对应东北天坐标系）,单位m
 *输出参数：
 *备    注：
 *示    例：
 *          
 */
void CoordTrans::ENcoordToLonLat(float E_Pos, float N_Pos, float * lonlat_coord)
{
	float a,f,longitude0,E_Pos0,yval,longitude,latitude;
	float E_Posval,e2,e1,ee,M,u,fai,C,T,NN,R,D,longitude1,latitude1 ;
	int zoneWide,ProjNo,Y0;

	// a = 6378245.0;f = 1.0/298.3;    // 54年北京坐标系参数    
	a = 6378140.0; f = 1.0 / 298.257;   // 80年西安坐标系参数   
	zoneWide = 6;
	ProjNo = (int)(E_Pos / 1000000.0); // 查找带号
	longitude0 = (ProjNo - 1) * zoneWide + zoneWide / 2;
	longitude0 = longitude0 * iPI; // 中央经线
	E_Pos0 = ProjNo * 1000000 + 500000;
	Y0 = 0;
	E_Posval = E_Pos - E_Pos0;
	yval = N_Pos - Y0;  // 带入大地坐标系
	e2 = 2 * f - f * f;
	e1 = (1.0 - sqrt(1 - e2)) / (1.0 + sqrt(1 - e2));
	ee = e2 / (1 - e2);
	M = yval;
	u = M / (a * (1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2 * e2 * e2 / 256));
	fai = u + (3 * e1 / 2 - 27 * e1 * e1 * e1 / 32) * sin(2 * u) + (21 * e1 * e1 / 16 - 55 * e1 * e1 * e1 * e1 / 32) * sin(4 * u) + (151 * e1 * e1 * e1 / 96) * sin(6 * u) + (1097 * e1 * e1 * e1 * e1 / 512) * sin(8 * u);
	C = ee * cos(fai) * cos(fai);
	T = tan(fai) * tan(fai);
	NN = a / sqrt(1.0 - e2 * sin(fai) * sin(fai));
	R = a * (1 - e2) / sqrt((1 - e2 * sin(fai) * sin(fai)) * (1 - e2 * sin(fai) * sin(fai)) * (1 - e2 * sin(fai) * sin(fai)));
	D = E_Posval / NN;
	// 计算经(Longitude) 纬度(Latitude)
	longitude1 = longitude0 + (D - (1 + 2 * T + C) * D * D * D / 6 + (5 - 2 * C + 28 * T - 3 * C * C + 8 * ee + 24 * T * T) * D * D * D * D * D / 120) / cos(fai);
	latitude1 = fai - (NN * tan(fai) / R) * (D * D / 2 - (5 + 3 * T + 10 * C - 4 * C * C - 9 * ee) * D * D * D * D / 24 + (61 + 90 * T + 298 * C + 45 * T * T - 256 * ee - 3 * C * C) * D * D * D * D * D * D / 720);
	// 转换为度 DD
	longitude = round((longitude1 / iPI) * 1000000) / 1000000.0;
	latitude = round((latitude1 / iPI) * 1000000) / 1000000.0;

	lonlat_coord[LonCoord_INDEX] = longitude;
	lonlat_coord[LatCoord_INDEX] = latitude;

}



/*
 *函数功能：根据高斯投影由经纬度推算大地坐标
 *输入参数：
 *输出参数：
 *备    注：
 *示    例：
 *          
 *
 */
void CoordTrans::LonLatToENcoord(float longitude, float latitude, float *ENcoord)
{
	int ProjNo = 0;
	int ZoneWide; //带宽  , latitude0
	float longitude1, latitude1, longitude0, E_Pos0, Y0, E_Posval, yval;
	float a, f, e2, ee, NN, T, C, A, M;
	ZoneWide = 6; 		    //6度带宽
	//a = 6378245.0;
	//f = 1.0 / 298.3;   //54年北京坐标系参数
	a=6378140.0; f=1/298.257; //80年西安坐标系参数
	ProjNo = (int)(longitude / ZoneWide);
	longitude0 = ProjNo * ZoneWide + ZoneWide / 2;
	longitude0 = longitude0 * iPI;
	//latitude0  = 0;
	longitude1 = longitude * iPI; //经度转换位弧度
	latitude1 = latitude * iPI; //纬度转换为弧度
	e2 = 2 * f - f * f;
	ee = e2 * (1.0 - e2);
	NN = a / sqrt(1.0 - e2 * sin(latitude1) * sin(latitude1));
	T = tan(latitude1)*tan(latitude1);
	C = ee * cos(latitude1) * cos(latitude1);
	A = (longitude1 - longitude0) * cos(latitude1);
	M = a * ((1 - e2 / 4 - 3 * e2*e2 / 64 - 5 * e2*e2*e2 / 256)*latitude1 - (3 * e2 / 8 + 3 * e2*e2 / 32
				+ 45 * e2*e2*e2 / 1024)*sin(2 * latitude1) + (15 * e2*e2 / 256 + 45 * e2*e2*e2 / 1024)*sin(4 * latitude1) - (35 * e2*e2*e2 / 3072)*sin(6 * latitude1));
	E_Posval = NN * (A + (1 - T + C)*A*A*A / 6 + (5 - 18 * T + T * T + 72 * C - 58 * ee)*A*A*A*A*A / 120);
	yval = M + NN * tan(latitude1)*(A*A / 2 + (5 - T + 9 * C + 4 * C*C)*A*A*A*A / 24
			+ (61 - 58 * T + T * T + 600 * C - 330 * ee)*A*A*A*A*A*A / 720);
	E_Pos0 = 1000000L * (ProjNo+ 1) + 500000L;
	Y0 = 0;
	E_Posval = E_Posval + E_Pos0;
	yval = yval + Y0;
	ENcoord[ECoord_INDEX] = E_Posval;
	ENcoord[NCoord_INDEX] = yval;
}



/*
 *函数功能：求两个经纬度之间的距离和方位角
 *输入参数：
 *输出参数：
 *备    注：
 *          
 *
 */
void CoordTrans::Cal_TwoDots_Info(float *Start_Dot, float *End_Dot, float & dis ,float & raw_rad_enu )
{	
	float Start_ENcoord[Coord_NUMS],End_ENcoord[Coord_NUMS];
	float  ECoord_Dis,NCoord_Dis;

	LonLatToENcoord(Start_Dot[LonCoord_INDEX],Start_Dot[LatCoord_INDEX],Start_ENcoord);
	LonLatToENcoord(End_Dot[LonCoord_INDEX],End_Dot[LatCoord_INDEX],End_ENcoord);
	ECoord_Dis= End_ENcoord[ECoord_INDEX]-Start_ENcoord[ECoord_INDEX];
	NCoord_Dis= End_ENcoord[NCoord_INDEX]-Start_ENcoord[NCoord_INDEX];
	dis=sqrt(pow(ECoord_Dis,2)+pow(NCoord_Dis,2));
	raw_rad_enu=atan2(NCoord_Dis,ECoord_Dis); // 转换为东北天坐标系，正东方向为0°;

}







/*
 *函数功能：求两个经纬度之间的距离
 *输入参数：
 *输出参数：
 *备    注：
 *
 *
 */
void CoordTrans::Cal_TwoDots_Dis(float *Start_Dot, float *End_Dot, float & dis)
{
    float Start_ENcoord[Coord_NUMS],End_ENcoord[Coord_NUMS];
    float  ECoord_Dis,NCoord_Dis;

    LonLatToENcoord(Start_Dot[LonCoord_INDEX],Start_Dot[LatCoord_INDEX],Start_ENcoord);
    LonLatToENcoord(End_Dot[LonCoord_INDEX],End_Dot[LatCoord_INDEX],End_ENcoord);
    ECoord_Dis= End_ENcoord[ECoord_INDEX]-Start_ENcoord[ECoord_INDEX];
    NCoord_Dis= End_ENcoord[NCoord_INDEX]-Start_ENcoord[NCoord_INDEX];
    dis=sqrt(pow(ECoord_Dis,2)+pow(NCoord_Dis,2));

}









/*
 *函数功能：求两个经纬度之间的方位角
 *输入参数：
 *输出参数：
 *备    注：
 *
 *
 */
void CoordTrans::Cal_TwoDots_Sita(float *Start_Dot, float *End_Dot,float & raw_rad_enu )
{
    float Start_ENcoord[Coord_NUMS],End_ENcoord[Coord_NUMS];
    float  ECoord_Dis,NCoord_Dis;

    LonLatToENcoord(Start_Dot[LonCoord_INDEX],Start_Dot[LatCoord_INDEX],Start_ENcoord);
    LonLatToENcoord(End_Dot[LonCoord_INDEX],End_Dot[LatCoord_INDEX],End_ENcoord);
    ECoord_Dis= End_ENcoord[ECoord_INDEX]-Start_ENcoord[ECoord_INDEX];
    NCoord_Dis= End_ENcoord[NCoord_INDEX]-Start_ENcoord[NCoord_INDEX];
    raw_rad_enu=atan2(NCoord_Dis,ECoord_Dis);

}







/*
 *函数功能：
 *输入参数：
 *输出参数：
 *备    注：
 *
 *
 */
void CoordTrans::Cal_EndCoord(float * Start_Dot, float dis,float goal_rad,float * End_Dot)
{
    float begin_lon = Start_Dot[0];
    float begin_lat = Start_Dot[1];
    float begin_coord[2];
    LonLatToENcoord(begin_lon, begin_lat, begin_coord);
    float add_coord[2];
    float end_coord[2];
    add_coord[0] = dis*cos(goal_rad);
    add_coord[1] = dis*sin(goal_rad);
    for(int i=0;i<2;i++)
    {
        end_coord[i]=begin_coord[i] + add_coord[i];
    }
    ENcoordToLonLat(end_coord[0], end_coord[1], End_Dot);

}



/*
 *函数功能：
 *输入参数：
 *输出参数：
 *备    注：
 *
 *
 */
void CoordTrans::Cal_SubEndCoord(float *Start_Dot, float * End_Dot,float dis,float * Sub_End_Dot)
{
    float goal_rad_enu;
    Cal_TwoDots_Sita(Start_Dot,End_Dot,goal_rad_enu);
    Cal_EndCoord(Start_Dot, dis,goal_rad_enu,Sub_End_Dot);
}





/*
 *函数功能：
 *输入参数：
 *输出参数：
 *备    注：
 *          
 *
 */
float CoordTrans::Transf_ECoordToNCoord(float Raw_Angle)
{

	float raw_raddata,raw_angle_enu;

	raw_raddata=Raw_Angle/180*PI;
	raw_angle_enu = atan2(-cos(raw_raddata),sin(raw_raddata))/PI*180; 
	return (raw_angle_enu);


}


//角度转换为弧度
float CoordTrans::toRadian(float degree)
{
	float radian = degree/180*PI;
	return radian;
}



//弧度转换为角度
float CoordTrans::toDegree(float radian)
{
	float degree = radian/PI*180;
	return degree;
}



/*
 *函数功能：笛卡尔坐标系转换为极坐标系
 *输入参数：
 *输出参数：
 *备    注：
 *          
 *
 */
void CoordTrans::Cart_To_Polar(Cart_Coord Cart,Polar_Coord & Polar)
{

	Polar.Mag = sqrt(pow(Cart.x,2)+pow(Cart.y,2));
	Polar.Rad = atan2(Cart.y, Cart.x);

}




/*
 *函数功能：
 *输入参数：
 *输出参数：
 *备    注：
 * A = [cos(90),-sin(90),0;sin(90),cos(90),0;0,0,1]
 * B = [cos(sita),-sin(sita),0;sin(sita),cos(sita),0;0,0,1]
 * C = A*B
 * angle = atan2(C(1,0),C(0,0));
 *
 */
float CoordTrans::Yaw_Angle_N2ECoord(float Yaw_Angle)
{
    float yaw_rad,yaw_angle_enu;
    yaw_rad=toRadian(Yaw_Angle);
    yaw_angle_enu = toDegree(atan2(cos(yaw_rad),-sin(yaw_rad))); // 转换为东北天坐标系，正东方向为0°
    return (yaw_angle_enu);
}




/*
 *函数功能：
 *输入参数：
 *输出参数：
 *备    注：
 * A = [cos(90),-sin(90),0;sin(90),cos(90),0;0,0,1]
 * B = [cos(sita),-sin(sita),0;sin(sita),cos(sita),0;0,0,1]
 * C = A*B
 * angle = atan2(C(1,0),C(0,0));
 *
 */
float CoordTrans::Yaw_Rad_N2ECoord(float Yaw_Rad)
{
    float yaw_rad_enu = atan2(cos(Yaw_Rad),-sin(Yaw_Rad)); // 转换为东北天坐标系，正东方向为0°
    return (yaw_rad_enu);
}



Eigen::Matrix<float,3,3> CoordTrans::x_rot_matrix(float sita)
{
    Eigen::Matrix<float,3,3> matrix;
    matrix<< 1,        0,         0,
            0,cos(sita),-sin(sita),
            0,sin(sita), cos(sita);
    return matrix;
}



Eigen::Matrix<float,3,3> CoordTrans::y_rot_matrix(float sita)
{
    Eigen::Matrix<float,3,3> matrix;
    matrix << cos(sita),0,sin(sita),
            0,1,        0,
            -sin(sita),0,cos(sita);
    return matrix;
}



Eigen::Matrix<float,3,3> CoordTrans::z_rot_matrix(float sita)
{
    Eigen::Matrix<float,3,3> matrix;
    matrix << cos(sita),-sin(sita),0,
            sin(sita), cos(sita),0,
            0,         0,1;
    return matrix;
}




Eigen::Matrix<float,3,3> CoordTrans::zxy_mutiple_matrix(float x_sita,float y_sita,float z_sita)
{
    Eigen::Matrix<float,3,3> rot_matrix;
    rot_matrix = z_rot_matrix(z_sita) *x_rot_matrix(x_sita) * y_rot_matrix(y_sita);
    return rot_matrix;
}




Eigen::Matrix<float,3,3> CoordTrans::yxz_mutiple_matrix(float x_sita,float y_sita,float z_sita)
{
    Eigen::Matrix<float,3,3> rot_matrix;
    rot_matrix = y_rot_matrix(y_sita)*x_rot_matrix(x_sita)*z_rot_matrix(z_sita);
    return rot_matrix;
}


/*
 *函数功能：
 *输入参数：
 *输出参数：
 *备   注：逆时针为正方向
 */
float CoordTrans::CalEndRad_Zcoord(float begin_rad,float add_rad)
{
    float end_rad;
    Eigen::AngleAxisd add_vector (add_rad, Eigen::Vector3d ( 0,0,1 ) ); //沿 Z 轴旋转
    Eigen::Matrix3d add_matrix  = add_vector.toRotationMatrix();
    Eigen::Vector3d begin_xy (cos(begin_rad),sin(begin_rad),0);
    Eigen::Vector3d end_xy = add_matrix*begin_xy;
    end_rad = atan2(end_xy(1),end_xy(0));

    return(end_rad);
}


float CoordTrans::CalEndAngle_Zcoord(float begin_angle,float add_angle)
{
    float begin_rad = toRadian(begin_angle);
    float add_rad = toRadian(add_angle);
    Eigen::AngleAxisd add_vector (add_rad, Eigen::Vector3d ( 0,0,1 ) ); //沿 Z 轴旋转
    Eigen::Matrix3d add_matrix  = add_vector.toRotationMatrix();
    Eigen::Vector3d begin_xy (cos(begin_rad),sin(begin_rad),0);
    Eigen::Vector3d end_xy = add_matrix*begin_xy;
    float end_rad = atan2(end_xy(1),end_xy(0));
    float end_angle = toDegree(end_rad);

    return(end_angle);
}




/*
 *函数功能：
 *输入参数：
 *输出参数：
 *备    注：逆时针为正方向
 */
float CoordTrans::CalAddRad_Zcoord(float begin_rad,float end_rad)
{
    float add_rad;
    Eigen::AngleAxisd begin_vector(begin_rad, Eigen::Vector3d(0, 0, 1)); //沿 Z 轴旋转
    Eigen::Matrix3d begin_matrix = begin_vector.toRotationMatrix();
    Eigen::Vector3d end_xy(cos(end_rad), sin(end_rad), 0);
    Eigen::Vector3d add_xy = begin_matrix.transpose() * end_xy;
    add_rad = atan2(add_xy(1), add_xy(0));

    return(add_rad);
}



float CoordTrans::CalAddAngle_Zcoord(float begin_angle,float end_angle)
{
    float begin_rad = toRadian(begin_angle);
    float end_rad = toRadian(end_angle);
    Eigen::AngleAxisd begin_vector(begin_rad, Eigen::Vector3d(0, 0, 1)); //沿 Z 轴旋转
    Eigen::Matrix3d begin_matrix = begin_vector.toRotationMatrix();
    Eigen::Vector3d end_xy(cos(end_rad), sin(end_rad), 0);
    Eigen::Vector3d add_xy = begin_matrix.transpose() * end_xy;
    float add_rad = atan2(add_xy(1), add_xy(0));
    float add_angle = toDegree(add_rad);

    return(add_angle);
}



/*
 *函数功能：
 *输入参数：
 *输出参数：
 *备    注：逆时针为正方向
 */
float CoordTrans::CalAverRad_Zcoord(float rad1,float rad2)
{

    float add_rad = CalAddRad_Zcoord(rad1,rad2);
    float aver_rad = CalEndRad_Zcoord(rad1,add_rad/2);
    return(aver_rad);
}



//函数功能：
int CoordTrans::Round(float r)
{
    return (r > 0.0) ? floor(r + 0.5) : ceil(r - 0.5);
}



//ENU Coord
void CoordTrans::GetAddPos(float * begin_lonlat,float e_add_pos,float n_add_pos,float * end_lonlat)
{
    float begin_lon = begin_lonlat[0];
    float begin_lat = begin_lonlat[1];
    float begin_coord[2];
    float end_coord[2];
    LonLatToENcoord(begin_lon, begin_lat,begin_coord);
    end_coord[0] = begin_coord[0] + e_add_pos;
    end_coord[1] = begin_coord[1] + n_add_pos;
    ENcoordToLonLat(end_coord[0], end_coord[1], end_lonlat);

}


//ENU Coord
void CoordTrans::Lonlat_Add_M(float * begin_lonlat,float * end_lonlat,float * add_m)
{
    float begin_geo[2];
    float end_geo[2];
    LonLatToENcoord(begin_lonlat[0], begin_lonlat[1],begin_geo);
    LonLatToENcoord(end_lonlat[0], end_lonlat[1],end_geo);
    add_m[0] = end_geo[0] - begin_geo[0];
    add_m[1] = end_geo[1] - begin_geo[1];
}



// Prototype: Cnb = a2mat(att)
// Input: att - att=[pitch; roll; yaw] in radians
// Output: Cnb - DCM from body-frame to navigation-frame
Eigen::Matrix<float,3,3> CoordTrans::a2mat(float pitch,float roll,float yaw)
{
    Eigen::Matrix<float,3,3> mat;
    float si = sin(pitch);
    float sj = sin(roll);
    float sk = sin(yaw);

    float ci = cos(pitch);
    float cj = cos(roll);
    float ck = cos(yaw);

    mat<< cj*ck-si*sj*sk, -ci*sk,  sj*ck+si*cj*sk,
          cj*sk+si*sj*ck,  ci*ck,  sj*sk-si*cj*ck,
                  -ci*sj,     si,           ci*cj;

    return mat;
}


// output: + or -
float CoordTrans::Cal_SideError(float set_yaw_rad,float cur_yaw_rad,float dis)
{
	float add_rad = CalAddRad_Zcoord(cur_yaw_rad,set_yaw_rad);
	float side_dis = dis*sin(add_rad);
    return side_dis;
}



double CoordTrans::compress_earth = 1/298.257;
int CoordTrans::Re = 6378137;
float CoordTrans::RAD = 180/PI;

double CoordTrans::Rm(float Lat)
{
   return(Re*(1-2*compress_earth+3*compress_earth*pow(sin(Lat),2))); //子午圈半径
}

double CoordTrans::Rn(float Lat)
{
    return(Re*(1+compress_earth*pow(sin(Lat),2))); //卯酉圈半径
}



void CoordTrans::Cal_TwoDots_Info2(float *Start_Dot, float *End_Dot, float & dis ,float & raw_rad_enu )
{	
	float pos1_rad[2];
	float pos2_rad[2]; 
	
	pos1_rad[0] = Start_Dot[0]/RAD; //lon
	pos1_rad[1] = Start_Dot[1]/RAD; //lat
	pos2_rad[0] = End_Dot[0]/RAD; //lon
	pos2_rad[1] = End_Dot[1]/RAD; //lat
	float x_add = (pos2_rad[0] - pos1_rad[0])*Rn(pos1_rad[1])*cos(pos1_rad[1]);   //lon
	float y_add = (pos2_rad[1] - pos1_rad[1])*Rm(pos1_rad[1]);     //lat

	dis = sqrt(pow(x_add,2)+pow(y_add,2));
	raw_rad_enu = atan2(y_add,x_add); // 转换为东北天坐标系，正东方向为0°;

}


void CoordTrans::Cal_TwoDots_Dis2(float *Start_Dot, float *End_Dot, float & dis)
{
	float pos1_rad[2];
	float pos2_rad[2]; 
	
	pos1_rad[0] = Start_Dot[0]/RAD; //lon
	pos1_rad[1] = Start_Dot[1]/RAD; //lat
	pos2_rad[0] = End_Dot[0]/RAD; //lon
	pos2_rad[1] = End_Dot[1]/RAD; //lat
	float x_add = (pos2_rad[0] - pos1_rad[0])*Rn(pos1_rad[1])*cos(pos1_rad[1]);   //lon
	float y_add = (pos2_rad[1] - pos1_rad[1])*Rm(pos1_rad[1]);     //lat

	dis = sqrt(pow(x_add,2)+pow(y_add,2));
}



void CoordTrans::Lonlat_Add_M2(float * begin_lonlat,float * end_lonlat,float * add_m)
{
	float pos1_rad[2];
	float pos2_rad[2]; 
	
	pos1_rad[0] = begin_lonlat[0]/RAD; //lon
	pos1_rad[1] = begin_lonlat[1]/RAD; //lat
	pos2_rad[0] = end_lonlat[0]/RAD; //lon
	pos2_rad[1] = end_lonlat[1]/RAD; //lat
	add_m[0] = (pos2_rad[0] - pos1_rad[0])*Rn(pos1_rad[1])*cos(pos1_rad[1]);   //lon
	add_m[1] = (pos2_rad[1] - pos1_rad[1])*Rm(pos1_rad[1]);     //lat
}



void CoordTrans::Cal_TwoDots_Sita2(float *Start_Dot, float *End_Dot,float & raw_rad_enu )
{
	float pos1_rad[2];
	float pos2_rad[2]; 
	
	pos1_rad[0] = Start_Dot[0]/RAD; //lon
	pos1_rad[1] = Start_Dot[1]/RAD; //lat
	pos2_rad[0] = End_Dot[0]/RAD; //lon
	pos2_rad[1] = End_Dot[1]/RAD; //lat
	float x_add = (pos2_rad[0] - pos1_rad[0])*Rn(pos1_rad[1])*cos(pos1_rad[1]);   //lon
	float y_add = (pos2_rad[1] - pos1_rad[1])*Rm(pos1_rad[1]);     //lat

	raw_rad_enu = atan2(y_add,x_add); // 转换为东北天坐标系，正东方向为0°;

}


//ENU Coord
void CoordTrans::GetAddPos2(float * begin_lonlat,float e_add_pos,float n_add_pos,float * end_lonlat)
{
    float lon0 = begin_lonlat[0];
    float lat0 = begin_lonlat[1];
	float lat0_rad = lat0/RAD;

	double x_add_rad = e_add_pos/(Rn(lat0_rad)*cos(lat0_rad));
	double y_add_rad = n_add_pos/Rm(lat0_rad);
	
	end_lonlat[0] = begin_lonlat[0] + x_add_rad*RAD;
	end_lonlat[1] = begin_lonlat[1] + y_add_rad*RAD;
}

void CoordTrans::GetAddPos2(double * begin_lonlat,double e_add_pos,double n_add_pos,double * end_lonlat)
{
    double lon0 = begin_lonlat[0];
    double lat0 = begin_lonlat[1];
	double lat0_rad = lat0/RAD;

	double x_add_rad = e_add_pos/(Rn(lat0_rad)*cos(lat0_rad));
	double y_add_rad = n_add_pos/Rm(lat0_rad);
	
	end_lonlat[0] = begin_lonlat[0] + x_add_rad*RAD;
	end_lonlat[1] = begin_lonlat[1] + y_add_rad*RAD;
}


//sort topK data 
// output: signal index of data 
std::vector<int> CoordTrans::Sort_topK(std::vector<int> & data, int K)
{
	std::vector<int> topK_index;
	int data_num = data.size();
	std::vector<int> sign_index(data_num,0);
	if(K>data_num)
	{
		K = data_num;
	}
	for(int i=0;i<K;i++)
	{
		int max_data = -65536;
		int max_idx = 0;		
		for(int j = 0;j<data_num;j++)
		{
			if(data[j]>max_data&&sign_index[j]==0)
			{
				max_data = data[j];	
				max_idx = j;
			}	
		}
		sign_index[max_idx] = i+1;

		//change_data(data[i],data[max_idx]); // change data 			
		//topK_index.push_back(max_idx);
	}

	return sign_index;
}


void CoordTrans::change_data(int & data1,int & data2)
{
	if(data1==data2)
	{
		return;
	}
	int data_buf;
	data_buf = data1;
	data1 = data2;
	data2 = data_buf;	

}


/*
 *函数功能：极坐标系转换为笛卡尔坐标系
 *输入参数：
 *输出参数：
 *备    注：
 *          
 *
 */
void CoordTrans::Polar2Cart(Polar_Coord Polar, Cart_Coord & Cart)
{
	Cart.x = Polar.Mag*cos(Polar.Rad); 
	Cart.y = Polar.Mag*sin(Polar.Rad); 
}


/*
void CoordTrans::MatXXf2Mat3(MatXXf & src,cv::Mat & dst,int type)
{
	int rows = src.rows();
	int cols = src.cols();
    cv::Mat temp(rows,cols,CV_8UC3);
    for(int i=0; i<rows; ++i)
	{
       for(int j=0; j<cols; ++j)
       {
			
       		temp.at<cv::Vec3b>(i, j)[0] = (char)src(i,j)*2;
			temp.at<cv::Vec3b>(i, j)[1] = 40; //=============attention here==============
			temp.at<cv::Vec3b>(i, j)[2] = 40; //=============attention here==============
       }

	}
    temp.copyTo(dst);
}
*/


