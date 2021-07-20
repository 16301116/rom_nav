//
//
//
#include "lib/earth_para.hpp"

double earth_para::compress_earth = 1/298.257;
int earth_para::Re = 6378137;
float earth_para::RAD = 180/PI;

double earth_para::Rm(float Lat)
{
   return(Re*(1-2*compress_earth+3*compress_earth*pow(sin(Lat),2))); //子午圈半径
}

double earth_para::Rn(float Lat)
{
    return(Re*(1+compress_earth*pow(sin(Lat),2))); //卯酉圈半径
}
