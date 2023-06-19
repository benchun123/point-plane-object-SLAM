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

#include "FrameDrawer.h"
#include "Tracking.h"
#include "Parameters.h"
#include "MapCuboid.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<mutex>

namespace ORB_SLAM2
{

FrameDrawer::FrameDrawer(Map* pMap):mpMap(pMap)
{
    mState=Tracking::SYSTEM_NOT_READY;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
}

cv::Mat FrameDrawer::DrawFrame()
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    int state; // Tracking state

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
        }
    } // destroy scoped mutex -> release mutex

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,CV_GRAY2BGR);

    //Draw
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                        cv::Scalar(0,255,0));
            }
        }        
    }
    else if(state==Tracking::OK) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        const int n = vCurrentKeys.size();
        for(int i=0;i<n;i++)
        {
            if(vbVO[i] || vbMap[i])
            {
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;

                // This is a match to a MapPoint in the map
                if(vbMap[i])
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,255,0),-1);
                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,0),-1);
                    mnTrackedVO++;
                }
            }
        }
    }

    // Draw cuboid
    if (whether_detect_object) //draw object box
    {                          // better to write some warning that if it is keyframe or not, because I only detect cuboid for keyframes.
        for (size_t i = 0; i < bbox_2ds.size(); i++)
        {
            cv::rectangle(im, bbox_2ds[i], cv::Scalar(177, 177, 0), 1); // 2d bounding box.
            if (box_corners_2ds[i].cols() > 0)    // for most offline read data, usually cannot read it, could use rviz.
            {
                plot_image_with_cuboid_edges(im, box_corners_2ds[i]); // eight corners.
            }
        }
    }
    // Draw plane
    if(whether_detect_plane)
    {
        Eigen::Matrix<double, 6, 3> color_list;
        color_list << 0,0,255,  0,255,0,  255,0,0,
                    0,255,255,  255,255,0,  255,0,255;
        for (size_t i = 0; i < plane_pts.size(); i++)
        {
            Eigen::Matrix2Xi pts = plane_pts[i];
            for (size_t j = 0; j < pts.cols(); j++)
            {
                int pixel_x = pts(0,j);
                int pixel_y = pts(1,j);
                cv::Vec3b color(color_list(i%6,0),color_list(i%6,1),color_list(i%6,2));
                im.at<cv::Vec3b>(cv::Point(pixel_x,pixel_y)) = color;
            }
        }
    }


    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);

    return imWithInfo;
}


void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::OK)
    {
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nKFs = mpMap->KeyFramesInMap();
        int nMPs = mpMap->MapPointsInMap();
        s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
        if(mnTrackedVO>0)
            s << ", + VO matches: " << mnTrackedVO;
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

}

void FrameDrawer::Update(Tracking *pTracker)
{
    unique_lock<mutex> lock(mMutex);
    pTracker->mImGray.copyTo(mIm);
    mvCurrentKeys=pTracker->mCurrentFrame.mvKeys;
    N = mvCurrentKeys.size();
    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);
    mbOnlyTracking = pTracker->mbOnlyTracking;


    if(pTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
    {
        mvIniKeys=pTracker->mInitialFrame.mvKeys;
        mvIniMatches=pTracker->mvIniMatches;
    }
    else if(pTracker->mLastProcessedState==Tracking::OK)
    {
        for(int i=0;i<N;i++)
        {
            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pTracker->mCurrentFrame.mvbOutlier[i])
                {
                    if(pMP->Observations()>0)
                        mvbMap[i]=true;
                    else
                        mvbVO[i]=true;
                }
            }
        }
    }
    mState=static_cast<int>(pTracker->mLastProcessedState);

    if(whether_detect_object) // copy some object data for visualization
    {
        bbox_2ds.clear();
        box_corners_2ds.clear();
        if (pTracker->mCurrentFrame.mpReferenceKF != NULL)      // mCurrentFrame.mpReferenceKF
            if ((pTracker->mCurrentFrame.mnId - pTracker->mCurrentFrame.mpReferenceKF->mnFrameId) < 1) // if current frame is a keyframe
                for (const MapCuboid *cuboid_tmp : pTracker->mCurrentFrame.mpReferenceKF->local_cuboids)
                {
                    bbox_2ds.push_back(cuboid_tmp->bbox_2d);
                    box_corners_2ds.push_back(cuboid_tmp->box_corners_2d);
                }
    }// loop whether detect object

    if(whether_detect_plane)
    {
        plane_pts.clear();
        if (pTracker->mCurrentFrame.mpReferenceKF != NULL)      // mCurrentFrame.mpReferenceKF
            if ((pTracker->mCurrentFrame.mnId - pTracker->mCurrentFrame.mpReferenceKF->mnFrameId) < 1) // if current frame is a keyframe
            {
               plane_pts=pTracker->mCurrentFrame.mpReferenceKF->mvPlanePoints_img;
            }
    }
}

void FrameDrawer::plot_image_with_cuboid_edges(cv::Mat &img, Eigen::Matrix2Xd& corner_img)
{
    Eigen::MatrixXd edge_pt_ids(2,14); // for 12 body edge (1-2, 2-3, ...) and 2 front line
    edge_pt_ids << 1,2,3,4, 1,2,6,5, 1,4,8,5, 1,2,
                    5,6,7,8, 4,3,7,8, 2,3,7,6, 6,5;
    edge_pt_ids.array()-=1; // transfer 1-8 to 0-7
    for (int pt_id = 0; pt_id < 12; pt_id++) // 12 body edge
    {
        int i = edge_pt_ids(0, pt_id);
        int j = edge_pt_ids(1, pt_id);
        cv::Point pt1 = cv::Point(corner_img(0,i), corner_img(1,i));
        cv::Point pt2 = cv::Point(corner_img(0,j), corner_img(1,j));
        cv::line(img, pt1, pt2, cv::Scalar(0, 225, 0), 1, CV_AA, 0);
    }
    for (int pt_id = 0; pt_id < 2; pt_id++) // 2 front edge
    {
        int i = edge_pt_ids(0, 12+pt_id);
        int j = edge_pt_ids(1, 12+pt_id);
        cv::Point pt1 = cv::Point(corner_img(0,i), corner_img(1,i));
        cv::Point pt2 = cv::Point(corner_img(0,j), corner_img(1,j));
        cv::line(img, pt1, pt2, cv::Scalar(255, 0, 0), 1, CV_AA, 0);
    }

    for (int i = 0; i < 8; i++) // plot 8 corners
    {
        cv::Point pt = cv::Point(corner_img(0,i), corner_img(1,i));
        cv::circle(img, pt, i, CV_RGB(255, 0, 0), 2);
    }
}

} //namespace ORB_SLAM
