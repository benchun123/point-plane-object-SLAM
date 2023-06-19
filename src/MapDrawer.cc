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

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>

#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include "g2o_cuboid.h"
#include "Parameters.h"
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

namespace ORB_SLAM2
{


MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath):mpMap(pMap)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

	box_colors.push_back(Vector3f(230, 0, 0) / 255.0);	 // red  0
	box_colors.push_back(Vector3f(60, 180, 75) / 255.0);   // green  1
	box_colors.push_back(Vector3f(0, 0, 255) / 255.0);	 // blue  2
	box_colors.push_back(Vector3f(255, 0, 255) / 255.0);   // Magenta  3
	box_colors.push_back(Vector3f(255, 165, 0) / 255.0);   // orange 4
	box_colors.push_back(Vector3f(128, 0, 128) / 255.0);   //purple 5
	box_colors.push_back(Vector3f(0, 255, 255) / 255.0);   //cyan 6
	box_colors.push_back(Vector3f(210, 245, 60) / 255.0);  //lime  7
	box_colors.push_back(Vector3f(250, 190, 190) / 255.0); //pink  8
	box_colors.push_back(Vector3f(0, 128, 128) / 255.0);   //Teal  9
	box_colors.push_back(Vector3f(255, 255, 255) / 255.0);   //Teal  10
}

void MapDrawer::DrawMapPoints()
{
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();
    // std::cout << "MapDrawer:: DrawMapPoints: "<< vpMPs.size() << " ref points: " << vpRefMPs.size() << std::endl;

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

    }

    glEnd();
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    // std::cout << "MapDrawer:: KeyFrame: "<< vpKFs.size() << std::endl;

    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetPoseInverse().t();

            glPushMatrix();

            glMultMatrixf(Twc.ptr<GLfloat>(0));

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,0.0f,1.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();
        }
    }

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                }
            }

            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                cv::Mat Owp = pParent->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }

            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
            }
        }

        glEnd();
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}


void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}

void MapDrawer::DrawMapPointsInCuboid()
{
	// draw the object points
    bool associate_point_with_object = true;
	if (associate_point_with_object)
	{
		glPointSize(mPointSize * 5);
		glBegin(GL_POINTS);
		const vector<MapCuboid *> all_Map_objs = mpMap->GetAllMapCuboids();
		for (size_t object_id = 0; object_id < all_Map_objs.size(); object_id++)
		{
			MapCuboid *obj_landmark = all_Map_objs[object_id];
			if (obj_landmark->isBad())
				continue;
			vector<MapPoint *> owned_mappoints;
			owned_mappoints = obj_landmark->used_points_in_BA_filtered; // points really used in BA
			// owned_mappoints = obj_landmark->GetUniqueMapPoints(); // points really used in BA

			Vector3f box_color = box_colors[obj_landmark->mnId % box_colors.size()];
			glColor4f(box_color(0), box_color(1), box_color(2), 1.0f);
			for (size_t pt_id = 0; pt_id < owned_mappoints.size(); pt_id++)
			{
				MapPoint *mpt = owned_mappoints[pt_id];
				if (!mpt->isBad())
				{
					cv::Mat pos;
                    pos = mpt->GetWorldPos();
                    // if (obj_landmark->obj_been_optimized)
					// 		pos = mpt->GetWorldPosBA();
					if (pos.rows == 0)
						continue;
					glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
                    // std::cout << "point xyz:" << pos << std::endl;
				}
			}
		}
		glEnd();
	}

}

void MapDrawer::DrawFrameMapCuboids() // draw original map cuboid 
{
	// make sure frame cuboid is in init world frame
    std::vector<MapCuboid *> all_frame_cuboid;
    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* kfs = vpKFs[i];
        for (size_t j = 0; j < kfs->local_cuboids.size(); j++)
            all_frame_cuboid.push_back(kfs->local_cuboids[j]);
    }

    Eigen::MatrixXd all_edge_pt_ids;
	all_edge_pt_ids.resize(14, 2); // draw 12 edges + front
	all_edge_pt_ids << 1, 2, 2, 3, 3, 4, 4, 1, 5, 6, 6, 7, 7, 8, 8, 5, 1, 5, 2, 6, 3, 7, 4, 8, 1, 6, 2, 5;
	all_edge_pt_ids.array() -= 1;
    // std::cout << "MapDrawer:: DrawMapCuboids: "<< all_frame_cuboid.size() << std::endl;
	for (size_t object_id = 0; object_id < all_frame_cuboid.size(); object_id++)
	{
		MapCuboid *plot_cuboid = all_frame_cuboid[object_id];
        // std::cout << "MapDrawer:: DrawMapCuboids: "<< all_frame_cuboid.size() << std::endl;
		if (plot_cuboid->isBad()) // some good, some bad, some not determined
			continue;

        Eigen::VectorXd plot_cuboid_data = plot_cuboid->cuboid_global_data.toMinimalVector();
        Eigen::Vector3d loc_world = Eigen::Vector3d(plot_cuboid_data(0), plot_cuboid_data(1), plot_cuboid_data(2));
        Eigen::Vector3d dim_world = Eigen::Vector3d(plot_cuboid_data(6), plot_cuboid_data(7), plot_cuboid_data(8));
        double yaw_world = plot_cuboid_data(5);
        Eigen::MatrixXd cuboid_corners(3, 8);
        Eigen::MatrixXd corners_body(3, 8);
        corners_body << 1, 1, -1, -1, 1, 1, -1, -1,
                        1, -1, -1, 1, 1, -1, -1, 1,
                        1, 1, 1, 1, -1, -1, -1, -1;
        Eigen::Matrix3d scale_mat = dim_world.asDiagonal();
        Eigen::Matrix3d rot;
        rot << cos(yaw_world), -sin(yaw_world), 0,
            sin(yaw_world), cos(yaw_world), 0,
            0, 0, 1;                          // rotation around z (front), how to define xyz
        Eigen::MatrixXd corners_without_center = rot * scale_mat * corners_body;
        for (size_t i = 0; i < 8; i++)
        {
            cuboid_corners(0,i) = corners_without_center(0,i) + loc_world(0);
            cuboid_corners(1,i) = corners_without_center(1,i) + loc_world(1);
            cuboid_corners(2,i) = corners_without_center(2,i) + loc_world(2);
        }

		// draw cuboid
        glLineWidth(mGraphLineWidth * 2);
        glBegin(GL_LINES);
        Eigen::Vector3f box_color = Eigen::Vector3f(233, 0, 0) / 255.0;
		// Vector3f box_color = box_colors[plot_cuboid->mnId % box_colors.size()];
		glColor4f(box_color(0), box_color(1), box_color(2), 1.0f); // draw all edges  cyan
		for (int line_id = 0; line_id < all_edge_pt_ids.rows(); line_id++)
		{
			glVertex3f(cuboid_corners(0, all_edge_pt_ids(line_id, 0)), cuboid_corners(1, all_edge_pt_ids(line_id, 0)), cuboid_corners(2, all_edge_pt_ids(line_id, 0)));
			glVertex3f(cuboid_corners(0, all_edge_pt_ids(line_id, 1)), cuboid_corners(1, all_edge_pt_ids(line_id, 1)), cuboid_corners(2, all_edge_pt_ids(line_id, 1)));
		}
		glEnd();
	}
}

void MapDrawer::DrawMapCuboids() // draw map cuboid after optimization
{
    DrawMapPointsInCuboid();
	// make sure final cuboid is in init world frame.
	const vector<MapCuboid *> all_Map_cubs = mpMap->GetAllMapCuboids();
    Eigen::MatrixXd all_edge_pt_ids;
	all_edge_pt_ids.resize(14, 2); // draw 12 edges + front
	all_edge_pt_ids << 1, 2, 2, 3, 3, 4, 4, 1, 5, 6, 6, 7, 7, 8, 8, 5, 1, 5, 2, 6, 3, 7, 4, 8, 1, 6, 2, 5;
	all_edge_pt_ids.array() -= 1;
    // std::cout << "MapDrawer:: DrawMapCuboids: "<< all_Map_cubs.size() << std::endl;
	for (size_t object_id = 0; object_id < all_Map_cubs.size(); object_id++)
	{
		MapCuboid *plot_cuboid = all_Map_cubs[object_id];
        // std::cout << "MapDrawer:: DrawMapCuboids: "<< all_Map_cubs.size() << std::endl;
		if (plot_cuboid->isBad()) // some good, some bad, some not determined
			continue;
        if (!plot_cuboid->obj_been_optimized) // show objects that being optimized!
            continue;

		// // show objects that being optimized! for kitti fix scale, this will make map visualization better.
		// if (bundle_object_opti)
		// {
		// 	if (!plot_cuboid->obj_been_optimized)
		// 	{
		// 		continue;
		// 	}
		// }
		// Eigen::MatrixXd cuboid_corners;
        // cuboid_corners = plot_cuboid->cuboid_global_data.compute3D_BoxCorner();

        Eigen::VectorXd plot_cuboid_data = plot_cuboid->cuboid_global_opti.toMinimalVector();
        Eigen::Vector3d loc_world = Eigen::Vector3d(plot_cuboid_data(0), plot_cuboid_data(1), plot_cuboid_data(2));
        Eigen::Vector3d dim_world = Eigen::Vector3d(plot_cuboid_data(6), plot_cuboid_data(7), plot_cuboid_data(8));
        double yaw_world = plot_cuboid_data(5);
        Eigen::MatrixXd cuboid_corners(3, 8);
        Eigen::MatrixXd corners_body(3, 8);
        corners_body << 1, 1, -1, -1, 1, 1, -1, -1,
                        1, -1, -1, 1, 1, -1, -1, 1,
                        1, 1, 1, 1, -1, -1, -1, -1;
        Eigen::Matrix3d scale_mat = dim_world.asDiagonal();
        Eigen::Matrix3d rot;
        rot << cos(yaw_world), -sin(yaw_world), 0,
            sin(yaw_world), cos(yaw_world), 0,
            0, 0, 1;                          // rotation around z (front), how to define xyz
        Eigen::MatrixXd corners_without_center = rot * scale_mat * corners_body;
        for (size_t i = 0; i < 8; i++)
        {
            cuboid_corners(0,i) = corners_without_center(0,i) + loc_world(0);
            cuboid_corners(1,i) = corners_without_center(1,i) + loc_world(1);
            cuboid_corners(2,i) = corners_without_center(2,i) + loc_world(2);
        }

		// draw cuboid
        glLineWidth(mGraphLineWidth * 2);
        glBegin(GL_LINES);
        // Eigen::Vector3f box_color = Eigen::Vector3f(233, 0, 0) / 255.0;
		Vector3f box_color = box_colors[plot_cuboid->mnId % box_colors.size()];
		glColor4f(box_color(0), box_color(1), box_color(2), 1.0f); // draw all edges  cyan
		for (int line_id = 0; line_id < all_edge_pt_ids.rows(); line_id++)
		{
			glVertex3f(cuboid_corners(0, all_edge_pt_ids(line_id, 0)), cuboid_corners(1, all_edge_pt_ids(line_id, 0)), cuboid_corners(2, all_edge_pt_ids(line_id, 0)));
			glVertex3f(cuboid_corners(0, all_edge_pt_ids(line_id, 1)), cuboid_corners(1, all_edge_pt_ids(line_id, 1)), cuboid_corners(2, all_edge_pt_ids(line_id, 1)));
		}
		glEnd();
	}
}

void MapDrawer::DrawMapTruthCuboids() // ideally this should be draw map cuboids.
{
    Eigen::MatrixXd all_edge_pt_ids;
	all_edge_pt_ids.resize(14, 2); // draw 12 edges + front
	all_edge_pt_ids << 1, 2, 2, 3, 3, 4, 4, 1, 5, 6, 6, 7, 7, 8, 8, 5, 1, 5, 2, 6, 3, 7, 4, 8, 1, 6, 2, 5;
	all_edge_pt_ids.array() -= 1;

	// make sure final cuboid is in init world frame.
    
    // read truth cuboid in tracking.h
    // Eigen::MatrixXd truth_cuboid_list(4,9);
    // truth_cuboid_list <<  0.98, 1.28, 0.18, 0, 0, 0, 0.25, 0.08, 0.18,
    //                 2.15, 1.28, 0.20, 0, 0, 0, 0.22, 0.10, 0.20,
    //                 3.30, 1.28, 0.20, 0, 0, 0, 0.14, 0.08, 0.20,
    //                 4.15, 1.28, 0.13, 0, 0, 0, 0.18, 0.13, 0.13;

    // draw cuboid
    // compute 3d world points,convert to camera coordinate
    // cv::Mat Twc_test;
    // Twc_test = (cv::Mat_<double>(3,4) <<
    //                 1,    0.0,    0.0,     0.6,
    //                -0.0,    0.0,      1,    -0.1,
    //                 0.0,     -1,    0.0,  0.4);
    // glPushMatrix();
    // glMultMatrixf(Twc_test.ptr<GLfloat>(0));
    for (int i = 0; i < truth_cuboid_list.rows(); i++)
    {
        Eigen::Vector3d loc_world = Eigen::Vector3d(truth_cuboid_list(i,0), truth_cuboid_list(i,1), truth_cuboid_list(i,2));
        Eigen::Vector3d dim_world = Eigen::Vector3d(truth_cuboid_list(i,6), truth_cuboid_list(i,7), truth_cuboid_list(i,8));
        double yaw_world = truth_cuboid_list(i,5);
        // Eigen::MatrixXd cuboid_corners = compute3D_BoxCorner_in_world(loc_world, dim_world, yaw_world);
        Eigen::MatrixXd cuboid_corners(3, 8);
        Eigen::MatrixXd corners_body(3, 8);
        corners_body << 1, 1, -1, -1, 1, 1, -1, -1,
                        1, -1, -1, 1, 1, -1, -1, 1,
                        1, 1, 1, 1, -1, -1, -1, -1;
        Eigen::Matrix3d scale_mat = dim_world.asDiagonal();
        Eigen::Matrix3d rot;
        rot << cos(yaw_world), -sin(yaw_world), 0,
            sin(yaw_world), cos(yaw_world), 0,
            0, 0, 1;                          // rotation around z (front), how to define xyz
        Eigen::MatrixXd corners_without_center = rot * scale_mat * corners_body;
        for (size_t i = 0; i < 8; i++)
        {
            cuboid_corners(0,i) = corners_without_center(0,i) + loc_world(0);
            cuboid_corners(1,i) = corners_without_center(1,i) + loc_world(1);
            cuboid_corners(2,i) = corners_without_center(2,i) + loc_world(2);
        }
        // // std::cout << "cuboid_corners: \n" << cuboid_corners << std::endl;
        // Eigen::MatrixXd cuboid_corners_4x8(4,8);
        // for (size_t j = 0; j < 8; j++)
        //     cuboid_corners_4x8(3,j) = 1.0;
        // cuboid_corners_4x8.block(0,0,3,8) = cuboid_corners;

        // Eigen::Matrix4d Twc;   // transfrom from world to camera    
        // Twc <<        1,    0.0,    0.0,    0.6,
        //            -0.0,    0.0,      1,    -0.1,
        //             0.0,     -1,    0.0,    0.4,
        //             0.0,    0.0,    0.0,      1;
        // Eigen::MatrixXd cuboid_local_cor =  Twc.inverse().topRows(3) * cuboid_corners_4x8;   
        // // std::cout << "cuboid_local_cor: \n" << cuboid_local_cor << std::endl;

        glLineWidth(mGraphLineWidth * 2);
        glBegin(GL_LINES);
        Eigen::Vector3f box_color = Eigen::Vector3f(0, 230, 0) / 255.0;
        glColor4f(box_color(0), box_color(1), box_color(2), 1.0f); // draw all edges  cyan
        for (int line_id = 0; line_id < all_edge_pt_ids.rows(); line_id++)
        {
            glVertex3f(cuboid_corners(0, all_edge_pt_ids(line_id, 0)), cuboid_corners(1, all_edge_pt_ids(line_id, 0)), cuboid_corners(2, all_edge_pt_ids(line_id, 0)));
            glVertex3f(cuboid_corners(0, all_edge_pt_ids(line_id, 1)), cuboid_corners(1, all_edge_pt_ids(line_id, 1)), cuboid_corners(2, all_edge_pt_ids(line_id, 1)));
        }
        glEnd();
        glPopMatrix();
    }
}

void MapDrawer::DrawMapPlanes() // draw map planes after optimization
{
	const vector<MapPlane *> all_Map_Planes = mpMap->GetAllMapPlanes();
    glPointSize(mPointSize * 1);
    glBegin(GL_POINTS);
    pcl::VoxelGrid<pcl::PointXYZRGB>  voxel;
    voxel.setLeafSize( 0.05, 0.05, 0.05);
	for (size_t plane_id = 0; plane_id < all_Map_Planes.size(); plane_id++)
	{
        MapPlane *plot_plane = all_Map_Planes[plane_id];
        map<KeyFrame*, int> observations = plot_plane->GetObservations();
        cv::Mat coef = plot_plane->GetWorldPos();
        float ir = plot_plane->mRed;
        float ig = plot_plane->mGreen;
        float ib = plot_plane->mBlue;
        float norm = sqrt(ir*ir + ig*ig + ib*ib);
        // glColor3f(ir/norm, ig/norm, ib/norm);
        glColor3f(box_colors[9](0), box_colors[9](1), box_colors[9](2));
        glColor3f(box_colors[plot_plane->mnId](0), box_colors[plot_plane->mnId](1), box_colors[plot_plane->mnId](2));
        if(plot_plane->asso_cuboid_id!=999)
        {
			Eigen::Vector3f asso_color = box_colors[plot_plane->asso_cuboid_id % box_colors.size()];
            glColor3f(asso_color(0), asso_color(1), asso_color(2));
        }
        if(plot_plane->asso_cuboid_id==999) // when not associated, draw a fix color
            continue;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr allCloudPoints(new pcl::PointCloud<pcl::PointXYZRGB>() );
        for(auto mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
        {
            KeyFrame* frame = mit->first;
            int id = mit->second;
            Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( frame->GetPose() );
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>() );
            pcl::transformPointCloud( frame->mvPlanePoints[id], *cloud, T.inverse().matrix());
            *allCloudPoints += *cloud;
        }
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>());
        voxel.setInputCloud( allCloudPoints );
        voxel.filter( *tmp );
        for(auto& p : tmp->points)
        {
            glVertex3f(p.x, p.y, p.z);
        }
    } // loop plane id 
    glEnd();
}

// void MapDrawer::DrawMapPlanes() // draw map planes after optimization
// {
// 	const vector<MapPlane *> all_Map_Planes = mpMap->GetAllMapPlanes();
//     glPointSize(mPointSize * 5);
//     glBegin(GL_POINTS);
// 	for (size_t plane_id = 0; plane_id < all_Map_Planes.size(); plane_id++)
// 	{
//         MapPlane *plot_plane = all_Map_Planes[plane_id];
//         cv::Mat coef = plot_plane->GetWorldPos();
//         cv::Mat centroid = plot_plane->GetWorldCentroid();
//         // Vector3f box_color = box_colors[plot_plane->mnId % box_colors.size()];
//         Vector3f box_color = box_colors[9];
//         // std::cout << "MapDrawer: plot_plane->asso_cuboid_id " << plot_plane->asso_cuboid_id << std::endl;
//         if(plot_plane->asso_cuboid_id!=999)
//         {
// 			box_color = box_colors[plot_plane->asso_cuboid_id % box_colors.size()];
//         }
//         glColor4f(box_color(0), box_color(1), box_color(2), 1.0f);
//         if (abs(coef.at<float>(2)) > abs(coef.at<float>(0)) && abs(coef.at<float>(2)) > abs(coef.at<float>(1)))
//         {
//             for(float ii = -0.25; ii < 0.25; ii = ii + 0.01)
//                 for(float jj = -0.25; jj < 0.25; jj = jj + 0.01)
//                 {
//                     float px = centroid.at<float>(0) + ii;
//                     float py = centroid.at<float>(1) + jj;
//                     float pz = (coef.at<float>(0)*px + coef.at<float>(1)*py + coef.at<float>(3)) / (-coef.at<float>(2));
// 					glVertex3f(px, py, pz);
//                 }
//         }
//         else if (abs(coef.at<float>(1)) > abs(coef.at<float>(0)) && abs(coef.at<float>(1)) > abs(coef.at<float>(2)))
//         {
//             for(float ii = -0.25; ii < 0.25; ii = ii + 0.01)
//                 for(float jj = -0.25; jj < 0.25; jj = jj + 0.01)
//                 {
//                     float px = centroid.at<float>(0) + ii;
//                     float pz = centroid.at<float>(2) + jj;
//                     float py = (coef.at<float>(0)*px + coef.at<float>(2)*pz + coef.at<float>(3)) / (-coef.at<float>(1));
// 					glVertex3f(px, py, pz);
//                 }
//         }
//         else if (abs(coef.at<float>(0)) > abs(coef.at<float>(1)) && abs(coef.at<float>(0)) > abs(coef.at<float>(2)))
//         {
//             for(float ii = -0.25; ii < 0.25; ii = ii + 0.01)
//                 for(float jj = -0.25; jj < 0.25; jj = jj + 0.01)
//                 {
//                     float py = centroid.at<float>(1) + ii;
//                     float pz = centroid.at<float>(2) + jj;
//                     float px = (coef.at<float>(1)*py + coef.at<float>(2)*pz + coef.at<float>(3)) / (-coef.at<float>(0));
// 					glVertex3f(px, py, pz);
//                 }
//         } // loop if coef max
//     } // loop plane id 
//     glEnd();
// }

void MapDrawer::DrawMapTruthCameraPose()
{
    // read truth pose in tracking.h
    // Eigen::MatrixXd truth_poses(4,8); 
    // truth_poses << 
    // 1617728169.41141939, 0.010638, -0.000009, 0.000000, 0.000000, 0.000000, -0.000494, 1.000000,
    // 1617728176.45573926, 0.803737, -0.000614, 0.000000, 0.000000, 0.000000, -0.000199, 1.000000,
    // 1617728192.57064152, 3.331697, -0.003343, 0.000000, 0.000000, 0.000000, -0.000105, 1.000000,
    // 1617728192.57064152, 3.361697, -0.003343, 0.000000, 0.000000, 0.000000, -0.000105, 1.000000;

    if (truth_poses.rows() > 0)
    {
        glLineWidth(mGraphLineWidth * 2);
        glBegin(GL_LINE_STRIP); // line strip connects adjacent points
        glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
        for (int pt_id = 0; pt_id < truth_poses.rows(); pt_id++)
        {
            // glVertex3f(truth_poses(pt_id, 0), truth_poses(pt_id, 1), truth_poses(pt_id, 2));
            glVertex3f(truth_poses(pt_id, 1), truth_poses(pt_id, 2), truth_poses(pt_id, 3));
        }
        glEnd();
    }
}

} //namespace ORB_SLAM
