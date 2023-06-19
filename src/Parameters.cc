/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/
#include "Parameters.h"

namespace ORB_SLAM2
{

std::string dataset_name;
std::string truth_camera_file;
std::string rgb_list_file;
std::string bbox_2d_list_file;
std::string truth_cuboid_file;
std::string offline_cuboid_list;
std::string offline_plane_list;
std::string association_file;

bool whether_detect_object = false;
bool whether_read_offline_cuboidtxt = false;

bool whether_detect_plane = false;
bool whether_read_offline_planetxt = false;

bool associate_point_with_object = false;
bool associate_cuboid_with_classname = false;

bool optimize_with_cuboid_plane = false;
bool optimize_with_plane_3d = false;
bool optimize_with_cuboid_2d = false;
bool optimize_with_cuboid_3d = false;
bool optimize_with_corners_2d = false;
bool optimize_with_pt_obj_3d = false;

bool enable_ground_height_scale = false;
bool build_worldframe_on_ground = false;

// // for camera pose
// double cam_height = 1.5; // first camera height: living1/2/3: 1.17  living0:1.39  office1/2/3: 1.5/1.51/1.51   office0: 1.96
// double camera_pose_var = 2.5; //- truth_frame_poses(0,3); // should be 2.5 or 2.25

// for BA
double ba_weight_bbox = 1.0;
double ba_weight_corner = 1.0;
double ba_weight_SE3 = 1.0;
double ba_weight_pt_obj = 1.0;

double thHuberBbox2d = 80.0;
double thHuberConer2d = 10.0;
double thHuberSE3 = 900.0;
double thHuberPtObj = 10.0;

double plane_angle_info = 1.0;
double plane_dist_info = 100.0;
double plane_chi = 500.0;

double cuboid_plane_angle_info = 2.0;
double cuboid_plane_dist_info = 100.0;
double cuboid_plane_chi = 500.0;

// for debug
int Image_num = 20;
int Stop_time = 1e4;

} // namespace ORB_SLAM2
