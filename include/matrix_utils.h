#ifndef MATRIX_UTILS_H
#define MATRIX_UTILS_H

#include <iostream>
#include <string>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

// using namespace cv;
using namespace std;
using namespace Eigen;

template <class T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> real_to_homo_coord(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &pts_in);

template <class T> // though vector can be casted into matrix, to make output clear to be vector, it is better to define a new function.
Eigen::Matrix<T, Eigen::Dynamic, 1> homo_to_real_coord_vec(const Eigen::Matrix<T, Eigen::Dynamic, 1> &pts_homo_in);

template <class T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> homo_to_real_coord(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &pts_homo_in);

// make sure column size is given. no checks here. row will be adjusted automatically. if more cols given, will be zero.
template <class T>
bool read_all_number_txt(const std::string txt_file_name, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &read_number_mat);


template <class T>
Eigen::Matrix<T, 3, 3> euler_zyx_to_rot(const T &roll, const T &pitch, const T &yaw);

template <class T>
void quat_to_euler_zyx(const Eigen::Quaternion<T> &q, T &roll, T &pitch, T &yaw);

Eigen::Quaterniond zyx_euler_to_quat(const double &roll, const double &pitch, const double &yaw);


bool read_obj_detection_txt(const std::string txt_file_name, Eigen::MatrixXd &read_number_mat, std::vector<std::string> &all_strings);

void read_yaml(const std::string &path_to_yaml, Eigen::Matrix3d & Kalib, float& depth_scale);

void LoadFileName(const std::string &strFile, std::vector<std::string> &vstrImageFilenames, std::vector<double> &vTimestamps);

// float bboxOverlapratio(const Eigen::Vector4d &bbox_vec_1, const Eigen::Vector4d &bbox_vec_2);
float bboxOverlapratio(const cv::Rect &rect1, const cv::Rect &rect2);

#endif //MATRIX_UTILS_H
