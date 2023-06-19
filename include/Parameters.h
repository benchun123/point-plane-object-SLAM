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
#pragma once

#ifndef PARAMETERS_H
#define PARAMETERS_H
#include <string>

namespace ORB_SLAM2
{

extern std::string dataset_name;
extern std::string truth_camera_file;
extern std::string rgb_list_file;
extern std::string bbox_2d_list_file;
extern std::string truth_cuboid_file;
extern std::string offline_cuboid_list;
extern std::string offline_plane_list;
extern std::string association_file;

extern bool whether_detect_object;
extern bool whether_read_offline_cuboidtxt;
extern bool whether_detect_plane;
extern bool whether_read_offline_planetxt;
extern bool associate_point_with_object;
extern bool associate_cuboid_with_classname;

extern bool optimize_with_plane_3d;
extern bool optimize_with_cuboid_plane;
extern bool optimize_with_cuboid_2d;
extern bool optimize_with_cuboid_3d;
extern bool optimize_with_corners_2d;
extern bool optimize_with_pt_obj_3d;

extern bool enable_ground_height_scale;
extern bool build_worldframe_on_ground;

// // for camera pose
// extern double cam_height;  // first camera height: living1/2/3: 1.17  living0:1.39  office1/2/3: 1.5/1.51/1.51   office0: 1.96
// extern double camera_pose_var; //- truth_frame_poses(0,3); // should be 2.5 or 2.25

// for BA
extern double ba_weight_bbox;
extern double ba_weight_corner;
extern double ba_weight_SE3;
extern double ba_weight_pt_obj;

extern double thHuberBbox2d;
extern double thHuberConer2d;
extern double thHuberSE3;
extern double thHuberPtObj;

extern double plane_angle_info;
extern double plane_dist_info;
extern double plane_chi;

extern double cuboid_plane_angle_info;
extern double cuboid_plane_dist_info;
extern double cuboid_plane_chi;

// for debug
extern int Image_num;
extern int Stop_time;

} // namespace ORB_SLAM2

#endif // PARAMETERS_H
