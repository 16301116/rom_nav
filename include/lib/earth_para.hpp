//
//
//

#ifndef _H_EARTH_PARAMETER_H
#define _H_EARTH_PARAMETER_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <vector>
#include <fstream>
#include <math.h>


#ifndef PI
	#define PI 3.1415926
#endif

class earth_para
{

private:


protected:

public:
    static double compress_earth;
    static int Re;
    static float RAD;

    static double Rm(float Lat);
    static double Rn(float Lat);


};


#endif


