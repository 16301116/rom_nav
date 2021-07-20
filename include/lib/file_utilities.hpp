//
//
//

#ifndef _H_FILE_UTILITIES_H_
#define _H_FILE_UTILITIES_H_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <vector>
#include <fstream>
#include "lib/eigen_types.h"

// save 3d points to file
void save_points(std::string filename, std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > points);

// save 3d points and it's obs in image
void save_features(std::string filename,
                   std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > points,
                   std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > features);

// save line obs
void save_lines(std::string filename,
                std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > features);

//void Load_Scan_Sonnar_Data(std::string filename, std::vector<Sonnar_Frame_Info> & sonnar_data);

//void Load_Scan_Sonnar_Data(std::string filename, std::vector<Sonnar_Frame_Info> & sonnar_data,int begin_idx,int end_idx);

#endif //IMUSIMWITHPOINTLINE_UTILITIES_H


