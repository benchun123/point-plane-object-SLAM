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

#include<opencv2/core/core.hpp>

#include<System.h>
#include "Parameters.h"

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);
void LoadParameters(const string &data_folder, const string &strSettingFile);

int main(int argc, char **argv)
{
    if(argc != 2)
    {
        cerr << endl << "Usage: ./rgbd_icl path_to_sequence" << endl;
        return 1;
    }

    std::string data_folder = std::string(argv[1]);
    std::string strSettingsFile = data_folder + "/ICL.yaml";
    std::string strVocFile = "/home/benchun/dataset/ORBvoc.bin";
    LoadParameters(data_folder, strSettingsFile);
    string strAssociationFilename = ORB_SLAM2::association_file;

    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    // string strAssociationFilename = string(argv[4]);
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);
    ORB_SLAM2::System SLAM(strVocFile, strSettingsFile, ORB_SLAM2::System::RGBD, true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imRGB, imD;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image and depthmap from file
        // imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
        // imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);
        imRGB = cv::imread(data_folder+"/"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
        imD = cv::imread(data_folder+"/"+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGB,imD,tframe);

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
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");   

    char bStop;
    cerr << "MonoICL: Please type 'x', if you want to shutdown windows." << endl;

    while (bStop != 'x'){
        bStop = getchar();
    }

    // Stop all threads
    SLAM.Shutdown();

    return 0;
}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}

void LoadParameters(const string &data_folder, const string &strSettingFile)
{
    cv::FileStorage fSettings(strSettingFile, cv::FileStorage::READ);

    ORB_SLAM2::dataset_name = std::string(fSettings["dataset_name"]);
    ORB_SLAM2::truth_camera_file = data_folder +"/"+ std::string(fSettings["truth_camera_file"]);
    ORB_SLAM2::association_file = data_folder +"/"+ std::string(fSettings["association_file"]);
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