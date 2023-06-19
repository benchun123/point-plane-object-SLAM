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


#include "Tracking.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"ORBmatcher.h"
#include"FrameDrawer.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"

#include"Optimizer.h"
#include"PnPsolver.h"

#include<iostream>

#include<mutex>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Parameters.h>
#include "matrix_utils.h" // read offline object
#include "g2o_cuboid.h" // read offline object
// #include "MapCuboid.h"
#include "tictoc_profiler/profiler.hpp"


using namespace std;

namespace ORB_SLAM2
{

// [orb-cuboid-slam] add initial setting
Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor):
    mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
    mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys), mpViewer(NULL),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0)
{
    // Load camera parameters from settings file

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::MONOCULAR)
        mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    if(sensor==System::STEREO || sensor==System::RGBD)
    {
        mThDepth = mbf*(float)fSettings["ThDepth"]/fx;
        cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
    }

    if(sensor==System::RGBD)
    {
        mDepthMapFactor = fSettings["DepthMapFactor"];
        if(fabs(mDepthMapFactor)<1e-5)
            mDepthMapFactor=1;
        else
            mDepthMapFactor = 1.0f/mDepthMapFactor;
    }

    mImageWidth = fSettings["Camera.width"];
    mImageHeight = fSettings["Camera.height"];
    double camera_height = double(fSettings["camera_height"]);

    mDepthMapFactor = fSettings["DepthMapFactor"];
    if(fabs(mDepthMapFactor)<1e-5)
        mDepthMapFactor=1;
    else
        mDepthMapFactor = 1.0f/mDepthMapFactor;

    // [orb-cuboid-slam] add initial setting
    // std::string data_folder = strSettingPath.substr(0, strSettingPath.find_last_of("/"));
    data_folder = strSettingPath.substr(0, strSettingPath.find_last_of("/")); // define as tracking::data_folder
    // Retrieve bbox yolo name
    // std::vector<std::string> vstrBbox; // define as tracking param
    // std::string bbox_2d_list_file = data_folder+"/yolov3_bbox.txt"; // define in Parameters.h, read from setting file
    std::vector<double> vframe_id;
    LoadFileName(bbox_2d_list_file, vstrBbox, vframe_id);
    LoadFileName(offline_cuboid_list, vstrCuboidFile, vframe_id);

    // read truth cuboid
    // std::string truth_cuboid_file = data_folder+"/cuboid_list.txt"; // define in Parameters.h, read from setting file
    std::vector<std::string> truth_class;
    Eigen::MatrixXd truth_cuboid_list(6,9); // except class, xyz, rot, scale
    if (!read_obj_detection_txt(truth_cuboid_file, truth_cuboid_list, truth_class))
       exit(-1);
    std::cout << "truth_cuboid_list: \n" << truth_cuboid_list << std::endl;
    mpMapDrawer->truth_cuboid_list.resize(truth_cuboid_list.rows(), truth_cuboid_list.cols());
    mpMapDrawer->truth_cuboid_list = truth_cuboid_list;

    // read odom, convert to camera pose
    // std::string truth_camera_file = data_folder + "/odom.txt"; // define in Parameters.h, read from setting file
    // Eigen::MatrixXd truth_frame_poses(100,8);// time, xyz, qwxyz
    truth_frame_poses.resize(100,8);
    if (!read_all_number_txt(truth_camera_file,truth_frame_poses))
       exit(-1);
    std::cout<< "truth_camera_pose file: " << truth_camera_file << " data size:  "<< truth_frame_poses.rows()<<std::endl;
    double camera_pose_var = - truth_frame_poses(0,3); // should be 2.5 or 2.25
    
    std::cout << "dataset: icl nuim, change camera pose coordinate ... " << std::endl;
    for (unsigned i = 0; i < truth_frame_poses.rows(); i++)
    {
        Eigen::MatrixXd cam_pose_origin = truth_frame_poses.row(i).tail<7>(); // xyz, q1234
        Eigen::Matrix<double,4,4> projection;
        projection.setIdentity();
        projection.block(0,0,3,3) = Eigen::Quaterniond(cam_pose_origin(6),cam_pose_origin(3),cam_pose_origin(4),cam_pose_origin(5)).toRotationMatrix();
        projection.col(3).head(3) = Eigen::Vector3d(cam_pose_origin(0), cam_pose_origin(1), cam_pose_origin(2));
        // first camera height: living1/2/3: 1.17  living0:1.39  office1/2/3: 1.5/1.51/1.51   office0: 1.96
        Eigen::Matrix4d left_matrix, right_matrix;  // change coordinate just for living room dataset
        left_matrix <<  1, 0, 0, 0, 
                        0, 0, 1, camera_pose_var, // 2.5, 2.25, negative value in first pose
                        0, 1, 0, camera_height, // camera height in first frame (where it comes from)
                        0, 0, 0, 1;
        right_matrix << 1, 0, 0, 0, 
                        0, -1, 0, 0, 
                        0,  0, 1, 0, 
                        0,  0, 0, 1;
        projection = left_matrix * projection * right_matrix;
        // after change coordinate, transfer back to camera pose for publication
        Eigen::Matrix3d R = projection.block(0,0,3,3);
        Eigen::Quaterniond qxyzw = Eigen::Quaterniond( R );
        Eigen::Vector3d trans = projection.col(3).head(3);
        truth_frame_poses(i, 0) = truth_frame_poses(i,0);
        truth_frame_poses(i, 1) = trans (0);
        truth_frame_poses(i, 2) = trans (1);
        truth_frame_poses(i, 3) = trans (2);
        truth_frame_poses(i, 4) = qxyzw.x();
        truth_frame_poses(i, 5) = qxyzw.y();
        truth_frame_poses(i, 6) = qxyzw.z();
        truth_frame_poses(i, 7) = qxyzw.w();
    }
	mpMapDrawer->truth_poses.resizeLike(truth_frame_poses);
	mpMapDrawer->truth_poses = truth_frame_poses;

    // Eigen::MatrixXd truth_frame_poses_plot; // read truth pose 
    // truth_frame_poses_plot.resizeLike(truth_frame_poses);
    // truth_frame_poses_plot = truth_frame_poses;
    // std::cout << "dataset: icl nuim, change camera pose coordinate ... " << std::endl;
    // for (unsigned i = 0; i < truth_frame_poses.rows(); i++)
    // {
    //     Eigen::MatrixXd cam_pose_origin = truth_frame_poses.row(i).tail<7>(); // xyz, q1234
    //     Eigen::Matrix<double,4,4> projection;
    //     projection.setIdentity();
    //     projection.block(0,0,3,3) = Eigen::Quaterniond(cam_pose_origin(6),cam_pose_origin(3),cam_pose_origin(4),cam_pose_origin(5)).toRotationMatrix();
    //     projection.col(3).head(3) = Eigen::Vector3d(cam_pose_origin(0), cam_pose_origin(1), cam_pose_origin(2));
    //     // first camera height: living1/2/3: 1.17  living0:1.39  office1/2/3: 1.5/1.51/1.51   office0: 1.96
    //     Eigen::Matrix4d left_matrix, right_matrix;  // change coordinate just for living room dataset
    //     left_matrix <<  1, 0, 0, 0, 
    //                     0, 0, 1, camera_pose_var, // 2.5, 2.25, negative value in first pose
    //                     0, 1, 0, camera_height, // camera height in first frame (where it comes from)
    //                     0, 0, 0, 1;
    //     right_matrix << 1, 0, 0, 0, 
    //                     0, -1, 0, 0, 
    //                     0,  0, 1, 0, 
    //                     0,  0, 0, 1;
    //     projection = left_matrix * projection * right_matrix;
    //     // after change coordinate, transfer back to camera pose for publication
    //     Eigen::Matrix3d R = projection.block(0,0,3,3);
    //     Eigen::Quaterniond qxyzw = Eigen::Quaterniond( R );
    //     Eigen::Vector3d trans = projection.col(3).head(3);
    //     truth_frame_poses_plot(i, 0) = truth_frame_poses(i,0);
    //     truth_frame_poses_plot(i, 1) = trans (0);
    //     truth_frame_poses_plot(i, 2) = trans (1);
    //     truth_frame_poses_plot(i, 3) = trans (2);
    //     truth_frame_poses_plot(i, 4) = qxyzw.x();
    //     truth_frame_poses_plot(i, 5) = qxyzw.y();
    //     truth_frame_poses_plot(i, 6) = qxyzw.z();
    //     truth_frame_poses_plot(i, 7) = qxyzw.w();
    // }
    // truth_frame_poses = truth_frame_poses_plot;
	// mpMapDrawer->truth_poses.resizeLike(truth_frame_poses);
	// mpMapDrawer->truth_poses = truth_frame_poses_plot;

    // the init pose should equal camera truth pose
	Eigen::MatrixXd init_pose_truth = truth_frame_poses.row(0).tail<7>(); // xyz, q1234
	Eigen::Matrix4d Twc;
	Twc.setIdentity();
	Twc.block(0,0,3,3) = Eigen::Quaterniond(init_pose_truth(6),init_pose_truth(3),init_pose_truth(4),init_pose_truth(5)).toRotationMatrix();
	Twc.col(3).head(3) = Eigen::Vector3d(init_pose_truth(0), init_pose_truth(1), init_pose_truth(2));
    std::cout << "Twc: \n " << Twc << std::endl;
	InitToGround = cv::Mat::eye(4, 4, CV_32F);
	for (int row = 0; row < 4; row++)
		for (int col = 0; col < 4; col++)
			InitToGround.at<float>(row, col) = float(Twc(row, col));
    std::cout << " InitToGround: \n" << InitToGround << std::endl;

    cuboid_num_without_delete = 0;
}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}


cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp)
{
    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGR2GRAY);
        }
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGBA2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGRA2GRAY);
        }
    }

    mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp)
{
    mImGray = imRGB;
    cv::Mat imDepth = imD;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if((fabs(mDepthMapFactor-1.0f)>1e-5) || imDepth.type()!=CV_32F)
        imDepth.convertTo(imDepth,CV_32F,mDepthMapFactor);

    mCurrentFrame = Frame(mImGray,imDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
{
    mImGray = im;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
    else
        mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
	// initial and track use different orb extractor, inital is 2000 features, track is 1000
    // std::cout << "Tracking: keypoints: " << mCurrentFrame.mvKeys.size() << std::endl;

    Track();

    return mCurrentFrame.mTcw.clone();
}

void Tracking::Track()
{
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

    // std::cout << "Tracking: statue: " << mState << " >> 2 is okay" << std::endl;

    if(mState==NOT_INITIALIZED)
    {
        if(mSensor==System::STEREO || mSensor==System::RGBD)
            StereoInitialization();
        else
            MonocularInitialization();

        // comment out due to plot cuboid in keyframe
        // update after keyframe, because cuboid detection happens in keyframe. 
        // mpFrameDrawer->Update(this);

        if(mState!=OK)
            return;
    }
    else
    {
        // System is initialized. Track Frame.
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if(!mbOnlyTracking)
        {
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.

            if(mState==OK)
            {
                // Local Mapping might have changed some MapPoints tracked in last frame
                CheckReplacedInLastFrame();

                if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                {
                    bOK = TrackReferenceKeyFrame();
                }
                else
                {
                    bOK = TrackWithMotionModel();
                    if(!bOK)
                        bOK = TrackReferenceKeyFrame();
                }
            }
            else
            {
                bOK = Relocalization();
            }
        }
        else
        {
            // Localization Mode: Local Mapping is deactivated

            if(mState==LOST)
            {
                bOK = Relocalization();
            }
            else
            {
                if(!mbVO)
                {
                    // In last frame we tracked enough MapPoints in the map

                    if(!mVelocity.empty())
                    {
                        bOK = TrackWithMotionModel();
                    }
                    else
                    {
                        bOK = TrackReferenceKeyFrame();
                    }
                }
                else
                {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is sucessfull we choose that solution, otherwise we retain
                    // the "visual odometry" solution.

                    bool bOKMM = false;
                    bool bOKReloc = false;
                    vector<MapPoint*> vpMPsMM;
                    vector<bool> vbOutMM;
                    cv::Mat TcwMM;
                    if(!mVelocity.empty())
                    {
                        bOKMM = TrackWithMotionModel();
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.mTcw.clone();
                    }
                    bOKReloc = Relocalization();

                    if(bOKMM && !bOKReloc)
                    {
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;

                        if(mbVO)
                        {
                            for(int i =0; i<mCurrentFrame.N; i++)
                            {
                                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                {
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                }
                            }
                        }
                    }
                    else if(bOKReloc)
                    {
                        mbVO = false;
                    }

                    bOK = bOKReloc || bOKMM;
                }
            }
        }

        mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(!mbOnlyTracking)
        {
            if(bOK)
                bOK = TrackLocalMap();
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if(bOK && !mbVO)
                bOK = TrackLocalMap();
        }

        if(bOK)
            mState = OK;
        else
            mState=LOST;

        // comment out, set keyframe first and update
        // // Update drawer
        // mpFrameDrawer->Update(this);

        // If tracking were good, check if we insert a keyframe
        if(bOK)
        {
            // Update motion model
            if(!mLastFrame.mTcw.empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;
            }
            else
                mVelocity = cv::Mat();

            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            // Clean VO matches
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }

            // Delete temporal MapPoints
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();

            // Check if we need to insert a new keyframe
            if(NeedNewKeyFrame())
                CreateNewKeyFrame();

            // // treat every frame as key frame
            // CreateNewKeyFrame();

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
        }

        // move here after keyframe set up
        mpFrameDrawer->Update(this); // I put here so that frame drawer syncs with new keyframe.

        // Reset if the camera get lost soon after initialization
        if(mState==LOST)
        {
            if(mpMap->KeyFramesInMap()<=5)
            {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                mpSystem->Reset();
                return;
            }
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame);
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    if(!mCurrentFrame.mTcw.empty())
    {
        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        mlbLost.push_back(mState==LOST);
    }
    else
    {
        // This can happen if tracking is lost
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState==LOST);
    }

}


void Tracking::StereoInitialization()
{
    if(mCurrentFrame.N>500)
    {
        // Set Frame pose to the origin
        // mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
		if (build_worldframe_on_ground) // transform initial pose and map to ground frame
        {
        cv::Mat R = InitToGround.rowRange(0, 3).colRange(0, 3);
        cv::Mat t = InitToGround.rowRange(0, 3).col(3);
        cv::Mat Rinv = R.t();
        cv::Mat Ow = -Rinv * t;
        cv::Mat GroundToInit = cv::Mat::eye(4, 4, CV_32F);
        Rinv.copyTo(GroundToInit.rowRange(0, 3).colRange(0, 3));
        Ow.copyTo(GroundToInit.rowRange(0, 3).col(3));
        std::cout << "GroundToInit \n" << GroundToInit << std::endl;
			mCurrentFrame.SetPose(GroundToInit);

        }
        else
            mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));

        // Create KeyFrame
        KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

        // Insert KeyFrame in the map
        mpMap->AddKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
        for(int i=0; i<mCurrentFrame.N;i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpMap);
                pNewMP->AddObservation(pKFini,i);
                pKFini->AddMapPoint(pNewMP,i);
                pNewMP->ComputeDistinctiveDescriptors();
                pNewMP->UpdateNormalAndDepth();
                mpMap->AddMapPoint(pNewMP);

                mCurrentFrame.mvpMapPoints[i]=pNewMP;
            }
        }

        cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl;

        mpLocalMapper->InsertKeyFrame(pKFini);

        mLastFrame = Frame(mCurrentFrame);
        mnLastKeyFrameId=mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;

        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints=mpMap->GetAllMapPoints();
        mpReferenceKF = pKFini;
        mCurrentFrame.mpReferenceKF = pKFini;

        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        mpMap->mvpKeyFrameOrigins.push_back(pKFini);

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

        mState=OK;
    }
}

void Tracking::MonocularInitialization()
{

    if(!mpInitializer)
    {
        // Set Reference Frame
        std::cout << "Tracking: monocular initialization: set reference frame 0" << std::endl;
        if(mCurrentFrame.mvKeys.size()>100)
        {
            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            if(mpInitializer)
                delete mpInitializer;

            mpInitializer =  new Initializer(mCurrentFrame,1.0,200);

            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            return;
        }
    }
    else
    {
        // Try to initialize
        std::cout << "Tracking: monocular initialization: set current frame " << mCurrentFrame.mnId << std::endl;
        if((int)mCurrentFrame.mvKeys.size()<=100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

        // Check if there are enough correspondences
        if(nmatches<100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
        {
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame.SetPose(Tcw);

            CreateInitialMapMonocular();
        }
    }
}

// [orb-cuboid-slam] convert to world coordinate using first camera pose
void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpMap,mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

	std::cout << "\033[33m Created Mono Initial Map using 1st keyframe " << pKFini->mnId << " normal frame ID  " << pKFini->mnFrameId << "\033[0m" << std::endl;
	std::cout << "\033[33m Created Mono Initial Map using 2rd keyframe " << pKFcur->mnId << " normal frame ID  " << pKFcur->mnFrameId << "\033[0m" << std::endl;

    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);

        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpMap);

        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpMap->AddMapPoint(pMP);
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    cout << "Tracking: New map created with " << mpMap->MapPointsInMap() << " points" << endl;

    std::cout << "Optimizer: Tracking, GlobalBundleAdjustemnt CreateInitialMapMonocular" << endl;
    Optimizer::GlobalBundleAdjustemnt(mpMap,20);

    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;

    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<100)
    {
        cout << "Wrong initialization, reseting..." << endl;
        Reset();
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
        }
    }

    // [orb-cuboid-slam] convert to world coordinate using first camera pose
    // bool build_worldframe_on_ground = true; // defined in Parameters.h, read from setting file
    if(build_worldframe_on_ground)
    {

        float scaling_ratio = 1.0f; // not scale in the beginning

        // rotate the world coordinate to the initial frame
        // only use the groundtruth of the first frame.
        // cv::Mat InitToGround = mInitialFrame.InitToGround;
        // cv::Mat InitToGround = (cv::Mat_<float>(4,4) <<
        //                         1.0, 0.0, 0.0, 0.6,
        //                         0.0, 0.0, 1.0, -0.1,
        //                         0.0, -1.0, 0.0, 0.4,
        //                         0.0, 0.0, 0.0, 1.0);
        std::cout << "InitToGround \n" << InitToGround << std::endl;
        cv::Mat R = InitToGround.rowRange(0, 3).colRange(0, 3);
        cv::Mat t = InitToGround.rowRange(0, 3).col(3);
        cv::Mat Rinv = R.t();
        cv::Mat Ow = -Rinv * t;
        cv::Mat GroundToInit = cv::Mat::eye(4, 4, CV_32F);
        Rinv.copyTo(GroundToInit.rowRange(0, 3).colRange(0, 3));
        Ow.copyTo(GroundToInit.rowRange(0, 3).col(3));
        std::cout << "GroundToInit \n" << GroundToInit << std::endl;

        pKFini->SetPose(pKFini->GetPose() * GroundToInit);
        pKFcur->SetPose(pKFcur->GetPose() * GroundToInit);

        for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++)
        {
            if (vpAllMapPoints[iMP])
            {
                MapPoint *pMP = vpAllMapPoints[iMP];
                pMP->SetWorldPos(pMP->GetWorldPos()*scaling_ratio);
                pMP->SetWorldPos(InitToGround.rowRange(0, 3).colRange(0, 3) * pMP->GetWorldPos() + InitToGround.rowRange(0, 3).col(3));
            }
        }
    }
    // bool whether_detect_object = true; // define in Parameters.h, read from setting file
    if (whether_detect_object)
    {
        DetectCuboid(pKFini);
        AssociateCuboids(pKFini);
        DetectCuboid(pKFcur);
        AssociateCuboids(pKFcur);
    }
    // bool whether_detect_plane = true; // define in Parameters.h, read from setting file
    if(whether_detect_plane)
    {
        DetectPlane(pKFini);
        AssociatePlanes(pKFini);
        DetectPlane(pKFcur);
        AssociatePlanes(pKFcur);
    }

    bool whether_associate_plane_cuboids = true; // define in Parameters.h, read from setting file
    if(whether_associate_plane_cuboids)
    {
        AssociatePlanesAndCuboids(pKFini);
        AssociatePlanesAndCuboids(pKFcur);
    }

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints=mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    mLastFrame = Frame(mCurrentFrame);

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mState=OK;
}

void Tracking::CheckReplacedInLastFrame()
{
    for(int i =0; i<mLastFrame.N; i++)
    {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(pMP)
        {
            MapPoint* pRep = pMP->GetReplaced();
            if(pRep)
            {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}


bool Tracking::TrackReferenceKeyFrame()
{
    // Compute Bag of Words vector
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<MapPoint*> vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);

    if(nmatches<15)
        return false;

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.mTcw);

    Optimizer::PoseOptimization(&mCurrentFrame);
    std::cout << "Optimizer: Trackin, PoseOptimization, TrackReferenceKeyFrame" << std::endl;

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    return nmatchesMap>=10;
}

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();

    mLastFrame.SetPose(Tlr*pRef->GetPose());

    if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR || !mbOnlyTracking)
        return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector<pair<float,int> > vDepthIdx;
    vDepthIdx.reserve(mLastFrame.N);
    for(int i=0; i<mLastFrame.N;i++)
    {
        float z = mLastFrame.mvDepth[i];
        if(z>0)
        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    if(vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint* pMP = mLastFrame.mvpMapPoints[i];
        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1)
        {
            bCreateNew = true;
        }

        if(bCreateNew)
        {
            cv::Mat x3D = mLastFrame.UnprojectStereo(i);
            MapPoint* pNewMP = new MapPoint(x3D,mpMap,&mLastFrame,i);

            mLastFrame.mvpMapPoints[i]=pNewMP;

            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }

        if(vDepthIdx[j].first>mThDepth && nPoints>100)
            break;
    }
}

bool Tracking::TrackWithMotionModel()
{
    ORBmatcher matcher(0.9,true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();

    mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);

    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    int th;
    if(mSensor!=System::STEREO)
        th=15;
    else
        th=7;
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR);

    // If few matches, uses a wider window search
    if(nmatches<20)
    {
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR);
    }

    if(nmatches<20)
        return false;

    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(&mCurrentFrame);
    std::cout << "Optimizer: Tracking, PoseOptimization, TrackWithMotionModel" << std::endl;

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    if(mbOnlyTracking)
    {
        mbVO = nmatchesMap<10;
        return nmatches>20;
    }

    return nmatchesMap>=10;
}

bool Tracking::TrackLocalMap()
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    UpdateLocalMap();

    SearchLocalPoints();

    // Optimize Pose
    Optimizer::PoseOptimization(&mCurrentFrame);
    std::cout << "Optimizer: Tracking, PoseOptimization, TrackLocalMap" << std::endl;
    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(!mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if(!mbOnlyTracking)
                {
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
                }
                else
                    mnMatchesInliers++;
            }
            else if(mSensor==System::STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);

        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
        return false;

    if(mnMatchesInliers<30)
        return false;
    else
        return true;
}


bool Tracking::NeedNewKeyFrame()
{
    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    const int nKFs = mpMap->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
        return false;

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose= 0;
    if(mSensor!=System::MONOCULAR)
    {
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;
            }
        }
    }

    bool bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);

    // Thresholds
    float thRefRatio = 0.75f;
    if(nKFs<2)
        thRefRatio = 0.4f;

    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    const bool c1c =  mSensor!=System::MONOCULAR && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| bNeedToInsertClose) && mnMatchesInliers>15);

    if((c1a||c1b||c1c)&&c2)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR)
            {
                if(mpLocalMapper->KeyframesInQueue()<3)
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
    }
    else
        return false;
}

void Tracking::CreateNewKeyFrame()
{
    if(!mpLocalMapper->SetNotStop(true))
        return;

    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

	std::cout << "\033[33m Tracking: Created new keyframe!  " << pKF->mnId //<< " local cuboid " << pKF->local_cuboids.size()
            << " normal frame ID  " << pKF->mnFrameId << "\033[0m" << std::endl;

    // [orb-cuboid-slam] add object detection (read offline)
    // bool whether_detect_object = true;// define in Parameters.h, read from setting file
    if (whether_detect_object)
    {
        ca::Profiler::tictoc("time object detection");
        DetectCuboid(pKF);
        AssociateCuboids(pKF);
        ca::Profiler::tictoc("time object detection");
    }
    // bool whether_detect_plane = true; // define in Parameters.h, read from setting file
    if(whether_detect_plane)
    {
        ca::Profiler::tictoc("time plane estimation");
        DetectPlane(pKF);
        AssociatePlanes(pKF);
        ca::Profiler::tictoc("time plane estimation");
    }

    ca::Profiler::tictoc("time plane object association");
    bool whether_associate_plane_cuboids = true; // define in Parameters.h, read from setting file
    if(whether_associate_plane_cuboids)
        AssociatePlanesAndCuboids(pKF);
    ca::Profiler::tictoc("time plane object association");

    // bool enable_ground_height_scale = true;// define in tracking.h, read from setting file
    if (enable_ground_height_scale)
	{
        int ground_everyKFs = 10; // estimate ground plane every 10 keyframes// define in tracking.h, read from setting file
        if (pKF->mnId % ground_everyKFs == 0)
        {
            float scaling_ratio = 1.0f;
            if (dataset_name == "icl_nuim")
            {
                if (pKF->mnId == 10) // key frame = 10, the scale can be estimated by depth image
                    scaling_ratio = 3.14f; // truth scale = 3.14, compare orb with gt, todo: use depth image to imit
            }
        
            std::vector<KeyFrame *> ground_potential_KFs;
            ground_potential_KFs.clear();
            long unsigned int start_frame_id = (pKF->mnId / ground_everyKFs -1) * ground_everyKFs; // from 0-10, 10-20, ....
            for (size_t ii = 0; ii < mvpLocalKeyFrames.size(); ii++) // local key frame may not contain current keyfram
            {
                KeyFrame *pKFi = mvpLocalKeyFrames[ii];
                if (pKFi->mnId >= start_frame_id) // from 0-10, 10-20, ....
                {
                    ground_potential_KFs.push_back(pKFi);
                    pKFi->mnGroundFittingForKF = pKF->mnId;
                }
            }
            ground_potential_KFs.push_back(pKF); // add current keyframe
            std::cout << "Ground: check_plane_potential_keyframe, num: "<< ground_potential_KFs.size() << std::endl;

            // scale all map points
            // transform to anchor frame
            KeyFrame *anchor_frame = ground_potential_KFs[0]; // first_keyframe  median_keyframe  pKF;
            cv::Mat anchor_Tcw = anchor_frame->GetPose();
            cv::Mat anchor_Twc = anchor_frame->GetPoseInverse();

            for (size_t iMP = 0; iMP < mvpLocalMapPoints.size(); iMP++) // approximatedly. actually mvpLocalMapPoints has much more points
            {
                cv::Mat local_pt = anchor_Tcw.rowRange(0, 3).colRange(0, 3) * mvpLocalMapPoints[iMP]->GetWorldPos() + anchor_Tcw.rowRange(0, 3).col(3);
                cv::Mat scaled_global_pt = anchor_Twc.rowRange(0, 3).colRange(0, 3) * (local_pt * scaling_ratio) + anchor_Twc.rowRange(0, 3).col(3);
                mvpLocalMapPoints[iMP]->SetWorldPos(scaled_global_pt);
            }
            for (size_t iKF = 0; iKF < ground_potential_KFs.size(); iKF++)
            {
                cv::Mat anchor_to_pose = ground_potential_KFs[iKF]->GetPose() * anchor_Twc;
                anchor_to_pose.col(3).rowRange(0, 3) = anchor_to_pose.col(3).rowRange(0, 3) * scaling_ratio;
                ground_potential_KFs[iKF]->SetPose(anchor_to_pose * anchor_Tcw);
            }
            cv::Mat anchor_to_pose = mLastFrame.mTcw * anchor_Twc;
            anchor_to_pose.col(3).rowRange(0, 3) = anchor_to_pose.col(3).rowRange(0, 3) * scaling_ratio;
            mLastFrame.SetPose(anchor_to_pose * anchor_Tcw);
            mCurrentFrame.SetPose(pKF->GetPose());
            mVelocity.col(3).rowRange(0, 3) = mVelocity.col(3).rowRange(0, 3) * scaling_ratio;
        }

        if (pKF->mnId < 10)
        {
            std::cout << "\033[31mTracking: KeyFrame size < 10, clear object  \033[0m" << std::endl;
            pKF->local_cuboids.clear(); // don't do object when in initial stage...
            pKF->keypoint_associate_cuboid_id.clear();
        }
    }// loop enable_ground_height_scale

    if(mSensor!=System::MONOCULAR)
    {
        mCurrentFrame.UpdatePoseMatrices();

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        vector<pair<float,int> > vDepthIdx;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        if(!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(),vDepthIdx.end());

            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }

                if(bCreateNew)
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpMap);
                    pNewMP->AddObservation(pKF,i);
                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                if(vDepthIdx[j].first>mThDepth && nPoints>100)
                    break;
            }
        }
    }

    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
            }
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }

    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor==System::RGBD)
            th=3;
        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;
        matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
    }
}

void Tracking::UpdateLocalMap()
{
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}

void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();

    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}


void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter;
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(!pMP->isBad())
            {
                const map<KeyFrame*,size_t> observations = pMP->GetObservations();
                for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
            }
        }
    }

    if(keyframeCounter.empty())
        return;

    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80)
            break;

        KeyFrame* pKF = *itKF;

        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }

    }

    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

bool Tracking::Relocalization()
{
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

    if(vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint*> sFound;

                const int np = vbInliers.size();

                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if(nGood<10)
                    continue;

                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        return false;
    }
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }

}

void Tracking::Reset()
{

    cout << "System Reseting" << endl;
    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    // Reset Local Mapping
    cout << "Reseting Local Mapper...";
    mpLocalMapper->RequestReset();
    cout << " done" << endl;

    // Reset Loop Closing
    cout << "Reseting Loop Closing...";
    mpLoopClosing->RequestReset();
    cout << " done" << endl;

    // Clear BoW Database
    cout << "Reseting Database...";
    mpKeyFrameDB->clear();
    cout << " done" << endl;

    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    if(mpViewer)
        mpViewer->Release();
}

void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}

// [orb-cuboid-slam] add cuboid detection function, here offline
void Tracking::DetectCuboid(KeyFrame *pKF)
{
    // bool whether_read_offline_cuboidtxt = true;  // define in Parameters.h, read from setting file
    std::cout << "Tracking: whether_read_offline_cuboidtxt: " << whether_read_offline_cuboidtxt << std::endl;
	if (whether_read_offline_cuboidtxt) // check offline object format, here global pose (world)
	{
        int frame_index = pKF->mnFrameId; // not keyframe index, but frame index
        char frame_index_c[256];
        sprintf(frame_index_c, "%04d", frame_index); // format into 4 digit
        // // // // read offline cuboid: each row: [cuboid_center(3), yaw, cuboid_scale(3), [x1 y1 w h]], prob]
        // Eigen::MatrixXd pred_frame_obj_txts(10,12);  // 3d cuboid [xyz, roll,pitch,yaw, dimension]
        // pred_frame_obj_txts.setZero();
        // if (!read_all_number_txt(data_folder+"/pred_3d_obj_matched_txt/"+frame_index_c+"_3d_cuboids.txt", pred_frame_obj_txts))
        // {    std::cout << "\033[31m bbox_offline_yolo: read error\033[0m" << std::endl;
        //     exit(-1);}
        // // std::cout << "pred_frame_obj_txts: " << pred_frame_obj_txts << std::endl;

        // pKF->local_cuboids.clear();
        // for(int obj_id = 0; obj_id < pred_frame_obj_txts.rows(); obj_id++)
        // {
        //     Eigen::Vector4d bbox_offline = pred_frame_obj_txts.row(obj_id).segment<4>(7); // x1, y1, x2, y2
        //     Eigen::Vector2d rect_center = (bbox_offline.tail<2>()/2.0) + bbox_offline.head<2>();
        //     Eigen::Vector2d widthheight = bbox_offline.tail<2>();
        //     Eigen::Vector4d bbox_value = Eigen::Vector4d(rect_center(0), rect_center(1), widthheight(0), widthheight(1)); // cx, cy, width, height
        //     // transfer global value to local value as edge measurement
        //     Eigen::VectorXd measure_data(9);
        //     measure_data << pred_frame_obj_txts(obj_id,0), pred_frame_obj_txts(obj_id,1), pred_frame_obj_txts(obj_id,2),
        //      0, 0, pred_frame_obj_txts(obj_id,3), pred_frame_obj_txts(obj_id,4), pred_frame_obj_txts(obj_id,5), pred_frame_obj_txts(obj_id,6);
        //     std::cout << "measure_data: " << measure_data.transpose() << std::endl;
        //     g2o::cuboid cube_ground_value;// offline cuboid txt in local ground frame.  [x y z yaw l w h]
        //     cube_ground_value.fromMinimalVector(measure_data);
		//     cv::Mat frame_pose_to_ground = InitToGround.clone(); // for kitti, I used InitToGround to pop offline.
		// 	g2o::cuboid curr_cuboid_local_pose = cube_ground_value.transform_to(Converter::toSE3Quat(frame_pose_to_ground));
        //     std::cout << "frame_pose_to_ground: " << frame_pose_to_ground << std::endl;
        //     std::cout << "curr_cuboid_local_pose: " << curr_cuboid_local_pose.toMinimalVector().transpose() << std::endl;

        //     // measurement in local camera frame! important
        //     MapCuboid *newcuboid = new MapCuboid(mpMap, false); // init mnId
        //     cv::Mat curr_pose_to_ground = pKF->GetPoseInverse(); // camera to init, not always ground.
        //     // g2o::SE3Quat frame_pose_to_init = Converter::toSE3Quat(InitToGround.inv()*curr_pose_to_ground); // camera to init, not always ground.
        //     g2o::SE3Quat frame_pose_to_init = Converter::toSE3Quat(curr_pose_to_ground); // camera to init, not always ground.
        //     g2o::cuboid global_obj_pose_to_init = curr_cuboid_local_pose.transform_from(frame_pose_to_init);
        //     std::cout << "frame_pose_to_init: " << frame_pose_to_init.toMinimalVector().transpose() << std::endl;
        //     std::cout << "global_obj_pose_to_init: " << global_obj_pose_to_init.toMinimalVector().transpose() << std::endl;
        //     newcuboid->cuboid_global_data = global_obj_pose_to_init; // curr_cuboid_global_pose
        //     newcuboid->cuboid_local_meas = curr_cuboid_local_pose;
        //     newcuboid->object_id = obj_id;
        //     newcuboid->object_graph_id = -1; // not assign yet
        //     newcuboid->object_classname = "any";
        //     newcuboid->bbox_vec = bbox_value; // center_x, center_y, width, height
        //     newcuboid->bbox_2d = cv::Rect(bbox_offline(0), bbox_offline(1), widthheight(0), widthheight(1));// x1, x2, width, height
        //     newcuboid->meas_quality = 0.7; // suppose
        //     newcuboid->SetReferenceKeyFrame(pKF);
        //     newcuboid->SetWorldPos(Converter::toCvMat(global_obj_pose_to_init.pose));
        //     pKF->local_cuboids.push_back(newcuboid);
		// 	if (1)
		// 	{
		// 		double obj_cam_dist = std::min(std::max(newcuboid->cuboid_local_meas.translation()(2), 10.0), 30.0); // cut into [a,b]
		// 		double obj_meas_quality = (60.0 - obj_cam_dist) / 40.0;
		// 		newcuboid->meas_quality = obj_meas_quality;
		// 	}
		// 	else
		// 		newcuboid->meas_quality = 1.0;
        // }


        // // read offline bbox: each row:  [x1y1wh]
        std::vector<std::string> bbox_name_yolo; // use bbox for object association
        Eigen::MatrixXd bbox_offline_yolo(10,5);  // 3d cuboid [x1y1x2y2, error]
        bbox_offline_yolo.setZero();
        if (!read_obj_detection_txt(data_folder+"/"+vstrBbox[frame_index], bbox_offline_yolo, bbox_name_yolo))
        {    std::cout << "\033[31m bbox_offline_yolo: \033[0m" << bbox_offline_yolo << "\033[0m" << std::endl;
            exit(-1);}
        std::cout << "bbox_offline_yolo: " << bbox_offline_yolo << std::endl;

        // // read offline cuboid: each row:  [cuboid_center(3), rotation(3), cuboid_scale(3)
        std::vector<std::string> bbox_name; // use bbox for object association
        Eigen::MatrixXd pred_frame_obj_txts(10,9);  // 3d cuboid [xyz, roll,pitch,yaw, dimension]
        pred_frame_obj_txts.setZero();
        if (!read_obj_detection_txt(data_folder+"/"+vstrCuboidFile[frame_index], pred_frame_obj_txts, bbox_name))
        {    std::cout << "\033[31m pred_frame_obj_txts: \033[0m" << pred_frame_obj_txts << "\033[0m" << std::endl;
            exit(-1);}
        std::cout << "pred_frame_obj_txts: " << pred_frame_obj_txts << std::endl;

        // for every object, add to keyframe
        pKF->local_cuboids.clear();
        for(int obj_id = 0; obj_id < pred_frame_obj_txts.rows(); obj_id++)
        {
            // transfer global value to local value as edge measurement
            Eigen::VectorXd measure_data = pred_frame_obj_txts.row(obj_id).tail(9);
            // std::cout << "measure_data: " << measure_data.transpose() << std::endl;
            g2o::cuboid curr_cuboid_global_pose;
            curr_cuboid_global_pose.fromMinimalVector(measure_data);
            g2o::SE3Quat curr_cam_true_Twc; // here need ground truth to convert global pose to local pose
            curr_cam_true_Twc.fromVector(truth_frame_poses.row(frame_index).tail(7));
            std::cout << "curr_cam_true_Twc: " << curr_cam_true_Twc.toMinimalVector().transpose() << std::endl;
            Eigen::Matrix3d Kalib;
            for (size_t ttt = 0; ttt < 3; ttt++)
                for (size_t zzz = 0; zzz < 3; zzz++)
                    Kalib(ttt,zzz) = mK.at<float>(ttt,zzz);

            // here we do not use yolo detect bbox, because we think it is not accurate, we instead project cuboid on image
            // Eigen::Vector4d bbox_offline = bbox_offline_yolo.row(obj_id).head(4); // x1, y1, x2, y2
            // Eigen::Vector2d rect_center = (bbox_offline.tail<2>()/2.0) + bbox_offline.head<2>();
            // Eigen::Vector2d widthheight = bbox_offline.tail<2>();
            // Eigen::Vector4d bbox_value = Eigen::Vector4d(rect_center(0), rect_center(1), widthheight(0), widthheight(1)); // cx, cy, width, height
            g2o::cuboid curr_cuboid_local_pose = curr_cuboid_global_pose.transform_to(curr_cam_true_Twc);
            Eigen::Matrix2Xd corners_2d = curr_cuboid_global_pose.projectOntoImage(curr_cam_true_Twc.inverse(), Kalib); // ptx, pty from 1-8
            // Eigen::Vector4d bbox_rect = curr_cuboid_global_pose.projectOntoImageRect(curr_cam_true_Twc.inverse(), Kalib); // cx, cy, width, height
            Eigen::Vector4d bbox_value = curr_cuboid_global_pose.projectOntoImageBbox(curr_cam_true_Twc.inverse(), Kalib); // cx, cy, width, height
            std::cout << "curr_cuboid_local_pose: " << curr_cuboid_local_pose.toMinimalVector().transpose() << std::endl;
            std::cout << "bbox_value:\n " << bbox_value.transpose() << std::endl;
            std::cout << "corners_2d:\n " << corners_2d << std::endl;

            // measurement in local camera frame! important
            MapCuboid *newcuboid = new MapCuboid(mpMap, false); // init mnId
            // g2o::SE3Quat frame_pose_to_init = Converter::toSE3Quat(pKF->GetPoseInverse()); // camera to init, not always ground.
            // g2o::cuboid global_obj_pose_to_init = curr_cuboid_local_pose.transform_from(frame_pose_to_init);
            // std::cout << "global_obj_pose_to_init: " << global_obj_pose_to_init.toMinimalVector().transpose() << std::endl;
            newcuboid->cuboid_global_data = curr_cuboid_global_pose;
            newcuboid->cuboid_local_meas = curr_cuboid_local_pose;
            newcuboid->box_corners_2d = corners_2d;
            newcuboid->object_id = obj_id;
            newcuboid->object_graph_id = -1; // not assign yet
            newcuboid->object_classname = bbox_name[obj_id];
            newcuboid->bbox_vec = bbox_value; // center_x, center_y, width, height
            newcuboid->bbox_2d = cv::Rect(bbox_value(0)-bbox_value(2)/2.0, bbox_value(1)-bbox_value(3)/2.0, bbox_value(2), bbox_value(3));// x1, x2, width, height
            newcuboid->meas_quality = 0.7; // suppose
            newcuboid->SetReferenceKeyFrame(pKF);
            newcuboid->SetWorldPos(Converter::toCvMat(curr_cuboid_global_pose.pose));
            pKF->local_cuboids.push_back(newcuboid);
            // mpMap->AddMapCuboid(newcuboid);
        }

    }
	else
	{
        std::cout << "\033[31m online detection is disable ... \033[0m" << std::endl;
	}
	std::cout << "Tracking: detect cuboid for key frame id: " << pKF->mnId << "  frame id: " << pKF->mnFrameId << "  numObj: " << pKF->local_cuboids.size() << std::endl;

    // for static object, associate points to cuboid based on 2d overlap... could also use instance segmentation
    // here assign a cuboid id, and in map point, add object observation, and in local mapping, update
	std::cout << "Tracking: associate_point_with_object: " << associate_point_with_object << std::endl;
    // bool associate_point_with_object = true;// define in Parameters.h, read from setting file
	if (associate_point_with_object)
	{
        // std::vector<bool> overlapped(pKF->local_cuboids.size(), false);
        // for (size_t i = 0; i < pKF->local_cuboids.size(); i++)
        //     for (size_t j = i + 1; j < pKF->local_cuboids.size(); j++)
        //         if (!overlapped[i] && !overlapped[j])
        //         {
        //             float iou_ratio = bboxOverlapratio(pKF->mvpMapCuboid[i]->bbox_2d, pKF->mvpMapCuboid[j]->bbox_2d);
        //             if (iou_ratio > 0.15)
        //             {
        //                 overlapped[i] = true;
        //                 overlapped[j] = true;
        //             }
        //         }
        pKF->keypoint_associate_cuboid_id = std::vector<int>(pKF->mvKeys.size(), -1);
        for (size_t i = 0; i < pKF->mvKeys.size(); i++)
        {
            int associated_times = 0;
            for (size_t j = 0; j < pKF->local_cuboids.size(); j++)
                if (pKF->local_cuboids[j]->bbox_2d.contains(pKF->mvKeys[i].pt))
                {
                    // if (!overlapped[j])
                    // {
                        associated_times++;
                        if (associated_times == 1)
                            pKF->keypoint_associate_cuboid_id[i] = j;
                        else
                            pKF->keypoint_associate_cuboid_id[i] = -1;
                    // }
                }
        }
        if (pKF->mnId < 2)
        {
            std::cout << "\033[31mTracking: init, clear object  \033[0m" << std::endl;
        	pKF->local_cuboids.clear(); // don't do object when in initial stage...
        	pKF->keypoint_associate_cuboid_id.clear();
        }
	}

	// std::cout << "Tracking: check the objects become candidate, keyframe num:" << mvpLocalKeyFrames.size() << std::endl;
    // // using key frame or local key frame?
	// for (size_t i = 0; i < mvpLocalKeyFrames.size(); i++)
	// {
	// 	KeyFrame *kfs = mvpLocalKeyFrames[i];
	// 	for (size_t j = 0; j < kfs->local_cuboids.size(); j++)
	// 	{
	// 		MapCuboid *mMC = kfs->local_cuboids[j];
	// 		if (!mMC->become_candidate)
	// 		{
	// 			// points number maybe increased when later triangulated
    //             int  object_own_point_threshold = 10; // todo ? why potential points is zero
	// 			mMC->check_enough_map_points(object_own_point_threshold);
	// 		}
    //         // std::cout << "Tracking: check mvpLocalKeyFramesid: " << kfs->mnId  <<" cuboid id "<< mMC->object_id << std::endl;
	// 	}
	// }
    // for (size_t j = 0; j < pKF->local_cuboids.size(); j++)
    // {
    //     MapCuboid *mMC = pKF->local_cuboids[j];
    //     if (!mMC->become_candidate)
    //     {
    //         // points number maybe increased when later triangulated
    //         int  object_own_point_threshold = -1; // todo ? why potential points is zero
    //         mMC->check_enough_map_points(object_own_point_threshold);
    //     }
    // }
}

// [orb-cuboid-slam] add cuboid associate function, here using classname
void Tracking::AssociateCuboids(KeyFrame *pKF)
{
    std::cout << "Association: start association between cuboid and cuboid" << std::endl;
    // loop over current KF's objects, check with all past objects (or local objects), compare the associated object map points.
    // (if a local object is not associated, could re-check here as frame-object-point might change overtime, especially due to triangulation.)
    std::vector<MapCuboid *> LocalObjectsCandidates;
    std::vector<MapCuboid *> LocalObjectsLandmarks;
    // keypoint might not added to frame observation yet, so object might not have many associated points yet....
    // method 1: just check current frame's object, using existing map point associated objects.
    // same as plane association, don't just check current frame, but check all recent keyframe's unmatched objects...
    int  object_own_point_threshold = 20;
    for (size_t i = 0; i < mvpLocalKeyFrames.size(); i++) // pKF is not in mvpLocalKeyFrames yet
    {
        KeyFrame *kfs = mvpLocalKeyFrames[i];
        for (size_t j = 0; j < kfs->local_cuboids.size(); j++)
        {
            MapCuboid *mPC = kfs->local_cuboids[j];
			if (!mPC->become_candidate)
				mPC->check_enough_map_points(object_own_point_threshold); // own points come from MapPoint::AddObservation and from local mapping
            if (mPC->become_candidate && (!mPC->already_associated))
                LocalObjectsCandidates.push_back(kfs->local_cuboids[j]);
        }
        // mvpMapCuboid save the pointer, every frame is same, just need to change one, and others also change.
        for (size_t j = 0; j < kfs->mvpMapCuboid.size(); j++)
            if (kfs->mvpMapCuboid[j]) // might be deleted due to badFlag()
                if (!kfs->mvpMapCuboid[j]->isBad())
                    if (kfs->mvpMapCuboid[j]->association_refid_in_tracking != pKF->mnId) // could also use set to avoid duplicates
                    {
                        // std::cout << "object landmark mvpMapCuboid before id " << kfs->mvpMapCuboid[j]->association_refid_in_tracking << std::endl;
                        LocalObjectsLandmarks.push_back(kfs->mvpMapCuboid[j]);
                        kfs->mvpMapCuboid[j]->association_refid_in_tracking = pKF->mnId;
                        // std::cout << "object landmark mvpMapCuboid after id " << kfs->mvpMapCuboid[j]->association_refid_in_tracking << std::endl;
                    }
        // std::cout << "Association: key frame id" << kfs->mnId <<" mvpMapCuboid.size()"<< kfs->mvpMapCuboid.size()
        //     << " #landmarks " << LocalObjectsLandmarks.size() << std::endl;
    }

    std::cout << "Association: before Association, cuboids #candidate: " << LocalObjectsCandidates.size() << " #landmarks " << LocalObjectsLandmarks.size()
            << " #localKFs " << mvpLocalKeyFrames.size() << std::endl;

    // bool associate_cuboid_with_classname = true;// define in Parameters.h, read from setting file
    if (associate_cuboid_with_classname) // find associate id based on tracket id.
    {
        // sort landmarks classname
        // todo: when there are two candidata with same classname, but both not exist in landmark, with creat landmark twice ....
        // todo: maybe will be regarded as outlier and be removed out.
        std::vector<std::string> cuboid_name_list; // use for association
        for (size_t i = 0; i < LocalObjectsLandmarks.size(); i++)
        {
            MapCuboid *cuboid_tmp = LocalObjectsLandmarks[i];
            cuboid_name_list.push_back(cuboid_tmp->object_classname);
            std::cout << "Association: name_list: " << cuboid_tmp->mnId << " : "<< cuboid_name_list[i] << std::endl;
        }
        for (size_t i = 0; i < LocalObjectsCandidates.size(); i++)
        {
            // find existing object landmarks which has the same name
            MapCuboid *candidateObject = LocalObjectsCandidates[i];
            std::string candidate_name = candidateObject->object_classname;
            auto it = std::find(cuboid_name_list.begin(), cuboid_name_list.end(), candidate_name); // find the name where first occur
            if (it == cuboid_name_list.end()) //name not exist, add to name
            {
                // int cuboid_idx = cuboid_name_list.size();
                candidateObject->mnId = cuboid_num_without_delete;//mpMap->MapCuboidsInMap();
                candidateObject->object_graph_id = candidateObject->mnId; // same as mnId
                candidateObject->already_associated = true; // must be put before SetAsLandmark();
                candidateObject->SetAsLandmark(); // after associate, add potential map points to object

                KeyFrame *refframe = candidateObject->GetReferenceKeyFrame(); // reference frame = key frame
                candidateObject->addObservation(refframe, candidateObject->object_id); // add to frame observation
                refframe->mvpMapCuboid.push_back(candidateObject); // add local to map
                mpMap->AddMapCuboid(candidateObject); // add to  mpMap->mspMapCuboids
                cuboid_num_without_delete ++;
                std::cout << "Association: new cuboid! candidate "<< i <<" added to keyframe " << refframe->mnId << std::endl;
            }
            else // find existing name, update
            {
                int cuboid_idx = std::distance(cuboid_name_list.begin(), it);
                MapCuboid* existing_cuboid = LocalObjectsLandmarks[cuboid_idx];

                candidateObject->object_graph_id = existing_cuboid->object_graph_id;
                candidateObject->mnId = existing_cuboid->mnId;
                candidateObject->already_associated = true; // must be put before SetAsLandmark();

                KeyFrame *refframe = candidateObject->GetReferenceKeyFrame();
                existing_cuboid->addObservation(refframe, candidateObject->object_id);
                refframe->mvpMapCuboid.push_back(existing_cuboid);
                existing_cuboid->MergeIntoLandmark(candidateObject); // after associate, add candidate potential map points to landmark
                std::cout << "Association: existing! candidate "<< i <<"  update keyframe " << refframe->mnId << std::endl;
            }
        }
    } // loop associate_cuboid_with_classname

    else // find existing object landmarks which share most points with this object
    {
        int largest_shared_num_points_thres = 5;
        for (size_t i = 0; i < LocalObjectsCandidates.size(); i++)
        {
            MapCuboid *candidateObject = LocalObjectsCandidates[i];
            std::vector<MapPoint *> object_owned_pts = candidateObject->GetPotentialMapPoints();
            std::cout << "Association: LocalObjectsCandidates id " << i << " object_owned_pts " << object_owned_pts.size() << std::endl;

            MapCuboid *largest_shared_objectlandmark = nullptr;
            if (LocalObjectsLandmarks.size() > 0)
            {
                map<MapCuboid *, int> LandmarkObserveCounter;
                for (size_t j = 0; j < object_owned_pts.size(); j++)
                    for (map<MapCuboid *, int>::iterator mit = object_owned_pts[j]->MapObjObservations.begin(); mit != object_owned_pts[j]->MapObjObservations.end(); mit++)
                        LandmarkObserveCounter[mit->first]++;

                int largest_shared_num_points = largest_shared_num_points_thres;
                for (size_t j = 0; j < LocalObjectsLandmarks.size(); j++)
                {
                    MapCuboid *pMC = LocalObjectsLandmarks[j];
                    if (!pMC->isBad())
                        if (LandmarkObserveCounter.count(pMC))
                        {
                            if (LandmarkObserveCounter[pMC] >= largest_shared_num_points)
                            {
                                largest_shared_num_points = LandmarkObserveCounter[pMC];
                                largest_shared_objectlandmark = pMC;
                            }
                        }
                    // std::cout << "Association: LocalObjectsLandmarks " << j << " id " << pMC->mnId << " common pts: " << LandmarkObserveCounter[pMC] << std::endl;
                }
            }

            // if not found associated cuboid landmark, create as new landmark.  either using original local pointer, or initialize as new
            if (largest_shared_objectlandmark == nullptr)
            {
                candidateObject->already_associated = true; // must be put before SetAsLandmark();
                candidateObject->mnId = mpMap->MapCuboidsInMap();
                // candidateObject->mnId = cuboid_num_without_delete;//MapCuboid::getIncrementedIndex(); //mpMap->MapObjectsInMap();
                candidateObject->object_graph_id = candidateObject->mnId; //mpMap->MapObjectsInMap();
                // add cuboid to referece frame
                KeyFrame *refframe = candidateObject->GetReferenceKeyFrame();
                candidateObject->addObservation(refframe, candidateObject->object_id); // add to frame observation
                refframe->mvpMapCuboid.push_back(candidateObject);
                candidateObject->SetAsLandmark(); // after associate, add potential map points to object
                mpMap->AddMapCuboid(candidateObject);// add to  mpMap->mspMapCuboids
                cuboid_num_without_delete ++;
                std::cout << "Association: new cuboid! candidate "<< i <<" id " << candidateObject->mnId <<" added to keyframe " << refframe->mnId << std::endl;
            }
            else // if found associated cuboid, then update observation.
            {
                candidateObject->already_associated = true; // must be put before SetAsLandmark();
                candidateObject->mnId = largest_shared_objectlandmark->mnId; //mpMap->MapObjectsInMap();  // needs to manually set
                candidateObject->object_graph_id = largest_shared_objectlandmark->object_graph_id;

                KeyFrame *refframe = candidateObject->GetReferenceKeyFrame();
                largest_shared_objectlandmark->addObservation(refframe, candidateObject->object_id);
                refframe->mvpMapCuboid.push_back(largest_shared_objectlandmark);
                largest_shared_objectlandmark->MergeIntoLandmark(candidateObject); // after associate, add candidate potential map points to landmark
                std::cout << "Association: existing! candidate "<< i << " id " << candidateObject->mnId <<"  update keyframe " << refframe->mnId << std::endl;
            }
        }

    }

    // remove outlier objects....
    bool remove_object_outlier = true;
    bool check_object_points = false; // not use
    int minimum_object_observation = 3;

    if (remove_object_outlier)
    {
        vector<MapCuboid *> all_cuboids = mpMap->GetAllMapCuboids();
        for (size_t i = 0; i < all_cuboids.size(); i++)
        {
            MapCuboid *pMCuboid = all_cuboids[i];
            if ((!pMCuboid->isBad()) && (!pMCuboid->isGood))	// if not determined good or bad yet.
                // if ((int)pMCuboid->GetLatestKeyFrame()->mnId < (int)pKF->mnId - 15) //20
                if ((int)pMCuboid->GetReferenceKeyFrame()->mnId < (int)pKF->mnId - 15) //20
                {
                    // if not recently observed, and not enough observations.  NOTE if point-object not used in BA, filtered size will be zero...
                    // bool no_enough_inlier_pts = check_object_points && (pMCuboid->NumUniqueMapPoints() > 20) && (pMCuboid->used_points_in_BA_filtered.size() < 10) && (pMCuboid->point_object_BA_counter > -1);
                    if (pMCuboid->Observations() < minimum_object_observation)
                    {
                        pMCuboid->SetBadFlag();
                        cout << "Found one bad object !!!!!!!!!!!!  cuboid id" << pMCuboid->mnId << " observation times:  " << pMCuboid->Observations() << endl;
                    }
                    else
                    {
                        pMCuboid->isGood = true;
                    }
                }
        }
    }

    bool debug_info = false;
    if(debug_info)
    {
        std::cout << "Tracking: check the objects index" << std::endl;
        for (size_t i = 0; i < mvpLocalKeyFrames.size(); i++)
        {
            KeyFrame *kfs = mvpLocalKeyFrames[i];
            // for (size_t j = 0; j < kfs->local_cuboids.size(); j++)
            // {
            //     MapCuboid *mMC = kfs->local_cuboids[j];
            //     std::cout << "Tracking: mvpLocalKeyFrames " << kfs->mnId << " localcuboids.size() " << kfs->local_cuboids.size()
            //             << " id " << mMC->object_id << " graph_id: " << mMC->object_graph_id << std::endl;
            // }
            for (size_t j = 0; j < kfs->mvpMapCuboid.size(); j++)
            {
                // std::cout << "try error " << std::endl;
                // std::cout << "isGood " <<mMC->isGood << " isbad "<<mMC->isBad() << std::endl;
                if(kfs->mvpMapCuboid[j]!=NULL) // maybe deteleted
                {
                    MapCuboid *mMC = kfs->mvpMapCuboid[j];
                    std::cout << "Tracking: mvpLocalKeyFrames " << kfs->mnId << " mvpMapCuboid.size() " << kfs->mvpMapCuboid.size()
                            << " id " << mMC->mnId << " graph_id: " << mMC->object_graph_id << std::endl;
                }
            }
        }
    }


}


void Tracking::DetectPlane(KeyFrame *pKF)
{
    // bool whether_read_offline_planetxt = true;  // define in Parameters.h, read from setting file
    std::cout << "Tracking: whether_read_offline_planetxt: " << whether_read_offline_planetxt << std::endl;
	if (whether_read_offline_planetxt) // check offline object format, here global pose (world)
	{
        int frame_index = pKF->mnFrameId; // not keyframe index, but frame index
        // // read offline plane: each row:  [id normal(4) centroid(3) num]
        Eigen::MatrixXd pred_frame_plane_txts(10,9);  // plane param [nx, ny, nz, d, x y z]
        pred_frame_plane_txts.setZero();
        if (!read_all_number_txt(data_folder+"/plane_seg/"+std::to_string(frame_index)+"_offline_plane_multiplane.txt", pred_frame_plane_txts))
        {    std::cout << "\033[31m pred_frame_plane_txts: \033[0m" << pred_frame_plane_txts << "\033[0m" << std::endl;
            exit(-1);}
        std::cout << "pred_frame_plane_txts: \n" << pred_frame_plane_txts << std::endl;
        
        // for every plane, add to keyframe
        pKF->mvPlaneCoefficients.clear();
        for(int plane_id = 0; plane_id < pred_frame_plane_txts.rows(); plane_id++)
        {
            // transfer global value to local value as edge measurement
            Eigen::VectorXd measure_data = pred_frame_plane_txts.row(plane_id).tail(8);
            std::cout << "measure_data: " << measure_data.transpose() << std::endl;
            cv::Mat coef = (cv::Mat_<float>(4,1) << measure_data(0), measure_data(1),
                    measure_data(2), measure_data(3));
            if(coef.at<float>(3) < 0)
                coef = -coef;
            // if(!PlaneNotSeen(coef)){
            //     continue;
            // }
            cv::Mat centroid = (cv::Mat_<float>(4,1) << measure_data(4), measure_data(5), measure_data(6), 1.0f);
            pKF->mvPlaneCoefficients.push_back(coef);
            pKF->mvPlaneCentroid.push_back(centroid);
        }
        pKF->mnPlaneNum = pKF->mvPlaneCoefficients.size();
        pKF->mvpMapPlanes = vector<MapPlane*>(pKF->mnPlaneNum,static_cast<MapPlane*>(nullptr));
        pKF->mvpParallelPlanes = vector<MapPlane*>(pKF->mnPlaneNum,static_cast<MapPlane*>(nullptr));
        pKF->mvpVerticalPlanes = vector<MapPlane*>(pKF->mnPlaneNum,static_cast<MapPlane*>(nullptr));
        // pKf->plane_assco_cuboid_id.resize(pKF->mnPlaneNum);
        // pKf->plane_assco_cuboid_meas.resize(pKF->mnPlaneNum);
    }
    else
	{
        std::cout << "\033[31m online detection is disable ... \033[0m" << std::endl;
        int frame_index = pKF->mnFrameId; // not keyframe index, but frame index
        std::string depth_file_name = data_folder+"/depth/"+std::to_string(frame_index)+".png";
        cv::Mat imDepth = cv::imread(depth_file_name, CV_LOAD_IMAGE_UNCHANGED);
        if (imDepth.empty() || imDepth.depth() != CV_16U)
        {
            cout << "WARNING: cannot read depth image. No such a file, or the image format is not 16UC1" << endl;
        }
        if (mDepthMapFactor == 0)
            cout << "WARNING: should read intrinsic param firstly" << endl;
        if((fabs(mDepthMapFactor-1.0f)>1e-5) || imDepth.type()!=CV_32F)
            imDepth.convertTo(imDepth,CV_32F,mDepthMapFactor);

        typedef pcl::PointXYZRGB PointT;
        typedef pcl::PointCloud<PointT> PointCloud;
        PointCloud::Ptr inputCloud( new PointCloud() );
        int cloudDis = 3;//Config::Get<int>("Cloud.Dis");
        float fx = pKF->mK.at<float>(0,0);
        float fy = pKF->mK.at<float>(1,1);
        float cx = pKF->mK.at<float>(0,2);
        float cy = pKF->mK.at<float>(1,2);
        for ( int m=0; m<imDepth.rows; m+=cloudDis )
            for ( int n=0; n<imDepth.cols; n+=cloudDis )
        {
                float d = imDepth.ptr<float>(m)[n];
                PointT p;
                p.z = d;
                p.x = ( n - cx) * p.z / fx;
                p.y = ( m - cy) * p.z / fy;
                p.r = 0;
                p.g = 0;
                p.b = 250;
                inputCloud->points.push_back(p);
        }
        inputCloud->height = ceil(imDepth.rows/float(cloudDis));
        inputCloud->width = ceil(imDepth.cols/float(cloudDis));

        int min_plane = 1000;//MultiPlane_SizeMin;//300;
        float AngTh = 3.0;// MultiPlane_AngleThre;//2.0;
        float DisTh = 0.05;// MultiPlane_DistThre;//0.02;
        std::cout << "min_plane " << min_plane << std::endl;
        std::cout << "AngTh " << AngTh << std::endl;
        std::cout << "DisTh " << DisTh << std::endl;
        // // firstly, compute normal
        // pcl::PointCloud<PointT>::Ptr  input_cloud=rgbd_cloud.makeShared();
        pcl::PointCloud<PointT>::Ptr  input_cloud=inputCloud;
        pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
        ne.setMaxDepthChangeFactor(0.05f); // 0.05
        ne.setNormalSmoothingSize(10.0f); // 10.0
        ne.setInputCloud(input_cloud);
        ne.compute(*cloud_normals);

        // secondly, compute region, label, coefficient, inliners, ...
        pcl::OrganizedMultiPlaneSegmentation< PointT, pcl::Normal, pcl::Label > mps;
        pcl::PointCloud<pcl::Label>::Ptr labels ( new pcl::PointCloud<pcl::Label> );
        vector<pcl::ModelCoefficients> coefficients;
        vector<pcl::PointIndices> inliers;
        vector<pcl::PointIndices> label_indices;
        vector<pcl::PointIndices> boundary;
        std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT>>> regions;
        mps.setMinInliers(min_plane);	//int min_plane = 1000;
        mps.setAngularThreshold (0.017453 * AngTh); //float AngleThreshold = 3.0 (0.017453=pi/180)
        mps.setDistanceThreshold (DisTh); // float DistanceThreshold = 0.05
        mps.setInputNormals (cloud_normals);
        mps.setInputCloud (input_cloud);
        mps.segmentAndRefine (regions, coefficients, inliers, labels, label_indices, boundary);

        // // thirdly, exact and filter point cloud
        for (int i = 0; i < inliers.size(); ++i)
        {
            cv::Mat coef = (cv::Mat_<float>(4,1) << coefficients[i].values[0], 
                            coefficients[i].values[1], 
                            coefficients[i].values[2], 
                            coefficients[i].values[3]);
            if(coef.at<float>(3) < 0)
                    coef = -coef;
            std::cout << "plane: " << i << coef.t() << std::endl;

            pcl::ExtractIndices<PointT> extract;
            extract.setInputCloud(input_cloud);
            extract.setNegative(false);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            extract.setIndices(boost::make_shared<pcl::PointIndices>(inliers[i])); // #include <boost/make_shared.hpp>
            extract.filter(*planeCloud);

            // add color
            Eigen::Matrix<double, 6, 3> color_list;
            color_list << 0,0,255,  0,255,0,  255,0,0,
                    0,255,255,  177,50,200,  255,0,255;
            for (size_t pt_id = 0; pt_id < planeCloud->size(); pt_id++)
            {
                planeCloud->points[pt_id].r = color_list(i,0);
                planeCloud->points[pt_id].g = color_list(i,1);
                planeCloud->points[pt_id].b = color_list(i,2);
            }

            bool merge_pcl = false;
            for (int j = 0; j < pKF->mvPlaneCoefficients.size(); ++j) 
            {
                cv::Mat pM = pKF->mvPlaneCoefficients[j];
                float d = pM.at<float>(3,0) - coef.at<float>(3,0);
                float angle = pM.at<float>(0,0) * coef.at<float>(0,0) +
                                pM.at<float>(1,0) * coef.at<float>(1,0) +
                                pM.at<float>(2,0) * coef.at<float>(2,0);
                if((d < 0.2 && d > -0.2) && (angle > 0.965 || angle < -0.965)) // associate plane
                {
                    // model 1: SACSegmentation // simplest, only single plane
                    pcl::PointCloud<PointT> old_points = pKF->mvPlanePoints[j];
                    for (auto &p : old_points.points) {
                        p.r = color_list(i,0);
                        p.g = color_list(i,1);
                        p.b = color_list(i,2);
                    }
                    pcl::PointCloud<PointT>::Ptr  new_cloud=old_points.makeShared();
                    *planeCloud += *new_cloud;
                    pcl::SACSegmentation<PointT> *seg = new pcl::SACSegmentation<PointT>();
                    pcl::ModelCoefficients coefficients;
                    pcl::PointIndices inliers;
                    seg->setOptimizeCoefficients(true); // optional
                    seg->setModelType(pcl::SACMODEL_PLANE); // required
                    seg->setMethodType(pcl::SAC_RANSAC);  // required
                    seg->setDistanceThreshold(0.01); // required 0.01m
                    seg->setInputCloud(planeCloud); // required ptr or setInputCloud(cloud.makeShared());
                    seg->segment(inliers, coefficients);  // seg.segment(*inliers, *coefficients);
                    if (inliers.indices.size() == 0) {
                        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
                        // return false;
                    }
                    Eigen::Vector4f local_fitted_plane(coefficients.values[0], coefficients.values[1], coefficients.values[2], coefficients.values[3]);
                    std::cout << "local_fitted_plane new: " << local_fitted_plane.transpose() << std::endl;
                    cv::Mat coef_new = (cv::Mat_<float>(4,1) << coefficients.values[0], 
                                    coefficients.values[1], 
                                    coefficients.values[2], 
                                    coefficients.values[3]);
                    // mvPlanePoints[j] = *planeCloud;
                    // mvPlaneCoefficients[j] = coef_new;
                    pKF->mvPlanePoints[j] = *planeCloud;
                    pKF->mvPlaneCoefficients[j] = coef_new;
                    pcl::PointXYZRGB centroid;
                    pcl::computeCentroid(*planeCloud, centroid); // recalculate
                    cv::Mat centroid_mat = (cv::Mat_<float>(4,1) << centroid.x, centroid.y, centroid.z, 1.0f);
                    std::cout << "plane: " << i << " centroid: " << centroid_mat.t() << std::endl;
                    pKF->mvPlaneCentroid[j] = centroid_mat;
                    merge_pcl = true;
                    // PointCloud::Ptr boundaryPoints(new PointCloud());
                    // boundaryPoints->points = regions[i].getContour();
                    // mvBoundaryPoints[j] += *boundaryPoints;
                    break;
                }
            }
            if(merge_pcl == false)
            {
                // mvPlanePoints.push_back(*planeCloud);
                // mvPlaneCoefficients.push_back(coef);
                // PointCloud::Ptr boundaryPoints(new PointCloud());
                // boundaryPoints->points = regions[i].getContour();
                // mvBoundaryPoints.push_back(*boundaryPoints);

                pKF->mvPlanePoints.push_back(*planeCloud);
                pKF->mvPlaneCoefficients.push_back(coef);
                pcl::PointXYZRGB centroid;
                pcl::computeCentroid(*planeCloud, centroid); // recalculate
                cv::Mat centroid_mat = (cv::Mat_<float>(4,1) << centroid.x, centroid.y, centroid.z, 1.0f);
                pKF->mvPlaneCentroid.push_back(centroid_mat);
                std::cout << "plane: " << i << " centroid: " << centroid_mat.t() << std::endl;
            }
        }

        std::cout << "plane num: " << pKF->mvPlaneCoefficients.size() << std::endl;
        for (size_t i = 0; i < pKF->mvPlanePoints.size(); i++)
        {
            Eigen::Matrix2Xi plane_img;
            plane_img.resize(2, pKF->mvPlanePoints[i].points.size());
            for (size_t pt_id = 0; pt_id < pKF->mvPlanePoints[i].points.size(); pt_id++)
            {
                double x = pKF->mvPlanePoints[i].points[pt_id].x;
                double y = pKF->mvPlanePoints[i].points[pt_id].y;
                double z = pKF->mvPlanePoints[i].points[pt_id].z;
                if (z != 0 )
                {
                    int pixel_x = int(x/z * fx + cx);
                    int pixel_y = int(y/z * fy + cy);
                    plane_img(0, pt_id) = pixel_x;
                    plane_img(1, pt_id) = pixel_y;
                }
            }
            pKF->mvPlanePoints_img.push_back(plane_img);
        }
        
        pKF->mnPlaneNum = pKF->mvPlaneCoefficients.size();
        pKF->mvpMapPlanes = vector<MapPlane*>(pKF->mnPlaneNum,static_cast<MapPlane*>(nullptr));
        pKF->mvpParallelPlanes = vector<MapPlane*>(pKF->mnPlaneNum,static_cast<MapPlane*>(nullptr));
        pKF->mvpVerticalPlanes = vector<MapPlane*>(pKF->mnPlaneNum,static_cast<MapPlane*>(nullptr));
	} // loop online mode
}

void Tracking::AssociatePlanes(KeyFrame *pKF)
{
    std::cout << "Association: start association between plane and plane" << std::endl;
    cout << "Association: Keyframe ID : " << pKF->mnId << " num of Plane: "  << pKF->mvPlaneCoefficients.size() << " all plane in map: " << mpMap->mspMapPlanes.size() << endl;
    // loop over current KF's plane, check with all planes, compare the plane in maps.
	for (size_t i = 0; i < pKF->mvPlaneCoefficients.size(); i++)
	{
		// cv::Mat plane_local_value = pKF->mvPlaneCoefficients[i]; // local?
		cv::Mat plane_in_frame = pKF->ComputePlaneWorldCoeff(i); // global
        if(plane_in_frame.at<float>(3) < 0)
            plane_in_frame = -plane_in_frame;
		float dist_thre = 0.4; // 
		float angle_thre = 0.8; // cos theta // 
        float angle_thre_ver = 0.08716;// # 85 degree mfVerTh;
        float angle_thre_par = 0.9962;// # 5 degree mfParTh;
        bool plane_existing = false;
        // std::cout << "Association: plane_in_frame " << plane_in_frame.t() << std::endl;
        for(set<MapPlane*>::iterator sit=mpMap->mspMapPlanes.begin(), send=mpMap->mspMapPlanes.end(); sit!=send; sit++)
        {
            cv::Mat plane_in_map = (*sit)->GetWorldPos();
            float angle = plane_in_frame.at<float>(0, 0) * plane_in_map.at<float>(0, 0) +
                        plane_in_frame.at<float>(1, 0) * plane_in_map.at<float>(1, 0) +
                        plane_in_frame.at<float>(2, 0) * plane_in_map.at<float>(2, 0);
	        float distance = plane_in_frame.at<float>(3,0) - plane_in_map.at<float>(3,0);
            // std::cout << "Association: plane_in_map " << plane_in_map.t() << std::endl;
            // std::cout << "Association: angle " << angle << " distance: " << distance << std::endl;
            
            plane_existing = (distance < dist_thre && distance > -dist_thre) && (angle > angle_thre || angle < -angle_thre); // associate plane 
            // plane_existing = CheckPlaneAssociation(plane_in_frame, plane_in_map, dist_thre, angle_thre);
		    if(plane_existing)
            {
                dist_thre = abs(distance);
                std::cout << "Association: existing! plane id "<< i <<"  match to plane in map " << (*sit)->mnId << std::endl;
                pKF->mvpMapPlanes[i] = static_cast<MapPlane*>(nullptr);
                pKF->mvpMapPlanes[i] = (*sit);
                continue;
            }

            // check vertical and parallel
            if (angle < angle_thre_ver && angle > -angle_thre_ver) {
                // std::cout << "Association: vertical! plane id "<< i <<"  match to plane in map " << (*sit)->mnId << std::endl;
                angle_thre_ver = abs(angle);
                pKF->mvpVerticalPlanes[i] = static_cast<MapPlane*>(nullptr);
                pKF->mvpVerticalPlanes[i] = (*sit);
                // continue;
            }
            if ((angle > angle_thre_par || angle < -angle_thre_par)) {
                // std::cout << "Association: parallel! plane id "<< i <<"  match to plane in map " << (*sit)->mnId << std::endl;
                angle_thre_par = abs(angle);
                pKF->mvpParallelPlanes[i] = static_cast<MapPlane*>(nullptr);
                pKF->mvpParallelPlanes[i] = (*sit);
            }
        }
        // if(plane_existing == false)
        // {
        //     std::cout << "Association: plane id "<< i <<" could not find match" << std::endl;
        //     MapPlane* pNewMP = new MapPlane(plane_in_frame, pKF, i, mpMap);
        //     mpMap->AddMapPlane(pNewMP);// add to  mpMap->mspMapPlanes
        //     pKF->AddMapPlane(pNewMP, i);
        //     // pKF->mvpMapPlanes.push_back(pNewMP); // add local to map
        //     // pKF->mvpMapPlanes[i] = static_cast<MapPlane*>(nullptr);
        //     // pKF->mvpMapPlanes[i] = pNewMP;
        // }
        
    } // loop plane size

    // after association, add observation
	for (size_t i = 0; i < pKF->mvPlaneCoefficients.size(); i++)
    {
        if(pKF->mvpParallelPlanes[i])
        {
            pKF->mvpParallelPlanes[i]->AddParObservation(pKF,i);
        }
        if(pKF->mvpVerticalPlanes[i])
        {
            pKF->mvpVerticalPlanes[i]->AddVerObservation(pKF,i);
        }
        if(pKF->mvpMapPlanes[i]) 
        {
            pKF->mvpMapPlanes[i]->AddObservation(pKF, i);
            if(!pKF->mvpMapPlanes[i]->mbSeen) {
                pKF->mvpMapPlanes[i]->mbSeen = true;
                mpMap->AddMapPlane(pKF->mvpMapPlanes[i]);
            }
            continue;
        }
        cv::Mat p3D = pKF->ComputePlaneWorldCoeff(i);
        cout << "Association: create new plane:" << i << "  " << pKF->mvPlaneCoefficients[i].t() << endl;
        // MapPlane* pNewMP = new MapPlane(p3D, pKF, i);
        MapPlane* pNewMP = new MapPlane(p3D, pKF, i, mpMap);
        pNewMP->asso_cuboid_id = 999;
        pNewMP->center_world = pKF->ComputePlaneCenterToWorld(i);
        mpMap->AddMapPlane(pNewMP);
        pKF->AddMapPlane(pNewMP, i);
    }

    cout << "Association: New map created with " << mpMap->MapPlanesInMap() << " planes" << endl;
}

void Tracking::AssociatePlanesAndCuboids(KeyFrame *pKF)
{
    std::cout << "Association: start association between planes and cuboids. " << std::endl;
    // std::cout << "Association: start association between planes and cuboids. plane: " << pKF->mvpMapPlanes.size() << " cuboid " << pKF->local_cuboids.size() << std::endl;
    for (size_t i = 0; i < mvpLocalKeyFrames.size(); i++) // pKF is not in mvpLocalKeyFrames yet
    {
        KeyFrame *kfs = mvpLocalKeyFrames[i];
        std::cout << "Association keyframe id " << kfs->mnId << " plane " << kfs->mvpMapPlanes.size() << " cuboid " << kfs->mvpMapCuboid.size() << std::endl;
        if(kfs->mvpMapPlanes.size()== 0 || kfs->mvpMapCuboid.size()==0)
            continue;
        for (size_t xx = 0; xx < kfs->mvpMapPlanes.size(); xx++)
        {
            MapPlane *map_plane = kfs->mvpMapPlanes[xx];
            if(!map_plane)
                continue;
            cv::Mat plane_coef = map_plane->GetWorldPos();
            bool find_cuboid_plane_asso = false;
            float dist_thre = 0.2;
            float angle_thre = 0.9397;

            for (size_t yy = 0; yy < kfs->mvpMapCuboid.size(); yy++)
            {
                MapCuboid *map_cuboid = kfs->mvpMapCuboid[yy];
                if(map_cuboid==NULL) // maybe deteleted
                    continue;
                // g2o::cuboid cuboid_asso = map_cuboid->cuboid_local_meas; // local pose
                g2o::cuboid cuboid_asso = map_cuboid->cuboid_global_data; // global pose
                Eigen::VectorXd cuboid_vec = cuboid_asso.pose.toVector(); // 7d, xyz, qwxyz
                Eigen::Matrix4d cuboid_mat;
                cuboid_mat.setIdentity();
                cuboid_mat.block(0,0,3,3) = Eigen::Quaterniond(cuboid_vec(6),cuboid_vec(3),cuboid_vec(4),cuboid_vec(5)).toRotationMatrix();
                cuboid_mat.col(3).head(3) = cuboid_vec.head(3);
                Eigen::MatrixXd cuboid_corners = cuboid_asso.compute3D_BoxCorner();
                // exact cuboid to 6 planes
                std::vector<cv::Mat> cuboid_coef;
                for (size_t k = 0; k < 6; k++)
                {
                    // n123 = R(theta).transpose()
                    float a = cuboid_mat(0, k%3);
                    float b = cuboid_mat(1, k%3);
                    float c = cuboid_mat(2, k%3);
                    float v = cuboid_mat.col(k%3).norm();
                    float d = a*cuboid_corners(0,0) + b*cuboid_corners(1,0)+ c*cuboid_corners(2,0);
                    if(k >= 3) // use the first or last corner to calculate d
                        d = a*cuboid_corners(0,6) + b*cuboid_corners(1,6)+ c*cuboid_corners(2,6);
                    cv::Mat coef = (cv::Mat_<float>(4,1) << a/v, b/v, c/v, -d/v);
                    if(coef.at<float>(3) < 0)
                        coef = -coef;
                    cuboid_coef.push_back(coef);
                }

                for (size_t zz = 0; zz < cuboid_coef.size(); zz++)
                {
                    cv::Mat coef = cuboid_coef[zz];
                    float dist = coef.at<float>(3,0) - plane_coef.at<float>(3,0);
                    float angle = coef.at<float>(0,0) * plane_coef.at<float>(0,0) +
                                    coef.at<float>(1,0) * plane_coef.at<float>(1,0) +
                                    coef.at<float>(2,0) * plane_coef.at<float>(2,0);
                    if((dist < dist_thre && dist > -dist_thre) && (angle > angle_thre || angle < -angle_thre)) // associate plane
                    {
                        find_cuboid_plane_asso = true;
                        dist_thre = dist;
                        g2o::Plane3D cuboid_temp(Eigen::Vector4d(coef.at<float>(0,0), coef.at<float>(1,0), 
                                                    coef.at<float>(2,0), coef.at<float>(3,0)));
                        g2o::Plane3D plane_temp(Eigen::Vector4d(plane_coef.at<float>(0,0), plane_coef.at<float>(1,0), 
                                                    plane_coef.at<float>(2,0), plane_coef.at<float>(3,0)));
                        Eigen::Vector3d measurement = cuboid_temp.ominus(plane_temp);
                        map_plane->asso_cuboid_id = map_cuboid->mnId;
                        map_plane->asso_cuboid_meas = measurement;
                        std::cout << "Association: local plane " << xx << " match to map cuboid " << map_cuboid->mnId << " e: " << measurement.transpose() << std::endl;
                        std::cout << "Association: local plane " << plane_coef.t() << " cuboid " << cuboid_asso.toMinimalVector().transpose() << std::endl;
                        // std::cout << "Association: dist " << dist << " angle " << angle << std::endl;
                    }
                } // loop cuboid
            } // loop plane

            if(!find_cuboid_plane_asso)
            {
                std::cout << "Association: local plane: " << xx << " can not match map cuboids" << std::endl;
                // int un_associate = 999;
                // Eigen::Vector3d measurement = Eigen::Vector3d(0.0, 0.0, 0.0);
                // map_plane->asso_cuboid_id = un_associate;
                // map_plane->asso_cuboid_meas = measurement;
            }

        }
    }

}


bool Tracking::CheckPlaneAssociation(cv::Mat& coef, cv::Mat& plane, float& dist_thre, float& angle_thre)
{
	float d = plane.at<float>(3,0) - coef.at<float>(3,0);
	float angle = plane.at<float>(0,0) * coef.at<float>(0,0) +
					plane.at<float>(1,0) * coef.at<float>(1,0) +
					plane.at<float>(2,0) * coef.at<float>(2,0);
	// float dist_thre = 0.2; // 
	// float angle_thre = 0.965; // cos theta // 
	if((d < dist_thre && d > -dist_thre) && (angle > angle_thre || angle < -angle_thre)) // associate plane
		return true;
	else
		return false;
}


} //namespace ORB_SLAM
