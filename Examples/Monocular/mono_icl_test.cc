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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>

#include<opencv2/core/core.hpp>

#include "System.h"
#include "Parameters.h"
#include "tictoc_profiler/profiler.hpp"

using namespace std;

void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);
void LoadParameters(const string &data_folder, const string &strSettingFile);

int main(int argc, char **argv)
{
    if(argc != 2)
    {
        cerr << endl << "Usage: ./mono_kitti path_to_sequence" << endl;
        return 1;
    }

    ca::Profiler::enable();

    cout << "\033[1;31m change print color examples \033[0m "<< endl; // https://stackoverflow.com/questions/2616906/how-do-i-output-coloured-text-to-a-linux-terminal
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
    std::string data_folder = std::string(argv[1]);
    std::string strSettingsFile = data_folder + "/ICL.yaml";
    std::string strVocFile = data_folder + "/ORBvoc.bin";
    LoadParameters(data_folder, strSettingsFile);
    std::string rgb_list_file = ORB_SLAM2::rgb_list_file;

    // // Retrieve paths to images
    std::vector<std::string> vstrRGBImg;
    std::vector<double> vTimestamps;
    LoadImages(rgb_list_file, vstrRGBImg, vTimestamps);

    bool bUseViewer = true;
    ORB_SLAM2::System SLAM(strVocFile, strSettingsFile, ORB_SLAM2::System::MONOCULAR, bUseViewer);

    // Main loop
    int nImages = vstrRGBImg.size();
    nImages = ORB_SLAM2::Image_num;
    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        std::cout << "------------------------frame index "  << ni  << "-------------------" << std::endl;
        ca::Profiler::tictoc("time single frame");
        // im = cv::imread(data_folder+"/"+vstrRGBImg[ni],CV_LOAD_IMAGE_UNCHANGED);
        im = cv::imread(data_folder+"/"+vstrRGBImg[ni],CV_LOAD_IMAGE_COLOR);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrRGBImg[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;
        int stop_time = ORB_SLAM2::Stop_time; 

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*stop_time); //1e4->opti only once, 1e6->opti every keyframe
            // usleep((T-ttrack)*1e6);
        ca::Profiler::tictoc("time single frame");
    }

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");    
    SLAM.SaveCuboidOptimized("CuboidPose.txt");    
    SLAM.SavePlaneOptimized("PlanePose.txt");    

    char bStop;

    cerr << "MonoICL: Please type 'x', if you want to shutdown windows." << endl;

    while (bStop != 'x'){
        bStop = getchar();
    }

    ca::Profiler::print_aggregated(std::cout);
    // Stop all threads
    SLAM.Shutdown();


    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    // std::cout << "strPathToSequence: " << strPathToSequence << std::endl;
    ifstream f;
    f.open(strPathToSequence.c_str());

    string s0;
    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
        }
    }
}

void LoadParameters(const string &data_folder, const string &strSettingFile)
{
    cv::FileStorage fSettings(strSettingFile, cv::FileStorage::READ);

    ORB_SLAM2::dataset_name = std::string(fSettings["dataset_name"]);
    ORB_SLAM2::truth_camera_file = data_folder +"/"+ std::string(fSettings["truth_camera_file"]);
    ORB_SLAM2::rgb_list_file = data_folder +"/"+ std::string(fSettings["rgb_list_file"]);
    ORB_SLAM2::bbox_2d_list_file = data_folder +"/"+ std::string(fSettings["bbox_2d_list_file"]);
    ORB_SLAM2::offline_cuboid_list = data_folder +"/"+ std::string(fSettings["offline_cuboid_list"]);
    ORB_SLAM2::offline_plane_list = data_folder +"/"+ std::string(fSettings["offline_plane_list"]);
    ORB_SLAM2::truth_cuboid_file = data_folder+"/"+std::string(fSettings["truth_cuboid_file"]); // define in Parameters.h, read from setting file

    ORB_SLAM2::build_worldframe_on_ground = int(fSettings["build_worldframe_on_ground"]);
    ORB_SLAM2::enable_ground_height_scale = int(fSettings["enable_ground_height_scale"]);

    ORB_SLAM2::whether_detect_object = int(fSettings["whether_detect_object"]);
    ORB_SLAM2::whether_read_offline_cuboidtxt = int(fSettings["whether_read_offline_cuboidtxt"]);
    ORB_SLAM2::whether_detect_plane = int(fSettings["whether_detect_plane"]);
    ORB_SLAM2::whether_read_offline_planetxt = int(fSettings["whether_read_offline_planetxt"]);

    ORB_SLAM2::associate_point_with_object = int(fSettings["associate_point_with_object"]);
    ORB_SLAM2::associate_cuboid_with_classname = int(fSettings["associate_cuboid_with_classname"]);

    ORB_SLAM2::optimize_with_plane_3d = int(fSettings["optimize_with_plane_3d"]);
    ORB_SLAM2::optimize_with_cuboid_plane = int(fSettings["optimize_with_cuboid_plane"]);
    ORB_SLAM2::optimize_with_cuboid_2d = int(fSettings["optimize_with_cuboid_2d"]);
    ORB_SLAM2::optimize_with_corners_2d = int(fSettings["optimize_with_corners_2d"]);
    ORB_SLAM2::optimize_with_cuboid_3d = int(fSettings["optimize_with_cuboid_3d"]);
    ORB_SLAM2::optimize_with_pt_obj_3d = int(fSettings["optimize_with_pt_obj_3d"]);

    ORB_SLAM2::ba_weight_bbox = double(fSettings["ba_weight_bbox"]);
    ORB_SLAM2::ba_weight_corner = double(fSettings["ba_weight_corner"]);
    ORB_SLAM2::ba_weight_SE3 = double(fSettings["ba_weight_SE3"]);
    ORB_SLAM2::ba_weight_pt_obj = double(fSettings["ba_weight_pt_obj"]);
    ORB_SLAM2::thHuberBbox2d = double(fSettings["thHuberBbox2d"]);
    ORB_SLAM2::thHuberConer2d = double(fSettings["thHuberConer2d"]);
    ORB_SLAM2::thHuberSE3 = double(fSettings["thHuberSE3"]);
    ORB_SLAM2::thHuberPtObj = double(fSettings["thHuberPtObj"]);

    ORB_SLAM2::plane_angle_info = double(fSettings["Plane.AngleInfo"]);
    ORB_SLAM2::plane_dist_info = double(fSettings["Plane.DisInfo"]);
    ORB_SLAM2::plane_chi = double(fSettings["Plane.Chi"]);

    ORB_SLAM2::cuboid_plane_angle_info = double(fSettings["CuboidPlane.AngleInfo"]);
    ORB_SLAM2::cuboid_plane_dist_info = double(fSettings["CuboidPlane.DisInfo"]);
    ORB_SLAM2::cuboid_plane_chi = double(fSettings["CuboidPlane.Chi"]);

    ORB_SLAM2::Image_num = int(fSettings["Debug.Image_num"]);
    ORB_SLAM2::Stop_time = int(fSettings["Debug.Stop_time"]);

}