#include <iomanip>
#include <iostream>
#include "lib/file_utilities.hpp"

using namespace std;


void save_points(std::string filename, std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > points)
{
    std::ofstream save_points;
    save_points.open(filename.c_str());
    for (int i = 0; i < points.size(); ++i) {
        Eigen::Vector4d p = points[i];

        save_points<<p(0)<<" "
                   <<p(1)<<" "
                   <<p(2)<<" "
                   <<p(3)<<std::endl;
    }
}



void save_features(std::string filename,
                   std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > points,
                   std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > features)
{
    std::ofstream save_points;
    save_points.open(filename.c_str());

    for (int i = 0; i < points.size(); ++i) {
        Eigen::Vector4d p = points[i];
        Eigen::Vector2d f = features[i];
        save_points<<p(0)<<" "
                   <<p(1)<<" "
                   <<p(2)<<" "
                   <<p(3)<<" "
                   <<f(0)<<" "
                   <<f(1)<<" "
                   <<std::endl;
    }
}



void save_lines(std::string filename,
                std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > features)
{
    std::ofstream save_points;
    save_points.open(filename.c_str());

    for (int i = 0; i < features.size(); ++i) {
        Eigen::Vector4d f = features[i];
        save_points<<f(0)<<" "
                   <<f(1)<<" "
                   <<f(2)<<" "
                   <<f(3)<<" "
                   <<std::endl;
    }
}


