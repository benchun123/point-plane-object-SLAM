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

#include "Optimizer.h"
#include "MapCuboid.h"
#include "g2o_cuboid.h"
#include "Parameters.h"
#include "G2O_Plane3D.h"
#include "MapPlane.h"

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

#include<Eigen/StdVector>

#include "Converter.h"

#include<mutex>

namespace ORB_SLAM2
{


void Optimizer::GlobalBundleAdjustemnt(Map* pMap, int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
    vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    vector<MapPoint*> vpMP = pMap->GetAllMapPoints();
    BundleAdjustment(vpKFs,vpMP,nIterations,pbStopFlag, nLoopKF, bRobust);
}


void Optimizer::BundleAdjustment(const vector<KeyFrame *> &vpKFs, const vector<MapPoint *> &vpMP,
                                 int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
    vector<bool> vbNotIncludedMP;
    vbNotIncludedMP.resize(vpMP.size());

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    long unsigned int maxKFid = 0;

    // Set KeyFrame vertices
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKF->GetPose()));
        vSE3->setId(pKF->mnId);
        vSE3->setFixed(pKF->mnId==0);
        optimizer.addVertex(vSE3);
        if(pKF->mnId>maxKFid)
            maxKFid=pKF->mnId;
    }

    const float thHuber2D = sqrt(5.99);
    const float thHuber3D = sqrt(7.815);

    // Set MapPoint vertices
    for(size_t i=0; i<vpMP.size(); i++)
    {
        MapPoint* pMP = vpMP[i];
        if(pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        const int id = pMP->mnId+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

       const map<KeyFrame*,size_t> observations = pMP->GetObservations();

        int nEdges = 0;
        //SET EDGES
        for(map<KeyFrame*,size_t>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
        {

            KeyFrame* pKF = mit->first;
            if(pKF->isBad() || pKF->mnId>maxKFid)
                continue;

            nEdges++;

            const cv::KeyPoint &kpUn = pKF->mvKeysUn[mit->second];

            if(pKF->mvuRight[mit->second]<0)
            {
                Eigen::Matrix<double,2,1> obs;
                obs << kpUn.pt.x, kpUn.pt.y;

                g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                if(bRobust)
                {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber2D);
                }

                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;

                optimizer.addEdge(e);
            }
            else
            {
                Eigen::Matrix<double,3,1> obs;
                const float kp_ur = pKF->mvuRight[mit->second];
                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                e->setInformation(Info);

                if(bRobust)
                {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber3D);
                }

                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;
                e->bf = pKF->mbf;

                optimizer.addEdge(e);
            }
        }

        if(nEdges==0)
        {
            optimizer.removeVertex(vPoint);
            vbNotIncludedMP[i]=true;
        }
        else
        {
            vbNotIncludedMP[i]=false;
        }
    }

    std::cout << "before optimization" << std::endl;
    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);
    std::cout << "after optimization" << std::endl;

    // Recover optimized data

    //Keyframes
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        if(nLoopKF==0)
        {
            pKF->SetPose(Converter::toCvMat(SE3quat));
        }
        else
        {
            pKF->mTcwGBA.create(4,4,CV_32F);
            Converter::toCvMat(SE3quat).copyTo(pKF->mTcwGBA);
            pKF->mnBAGlobalForKF = nLoopKF;
        }
    }

    //Points
    for(size_t i=0; i<vpMP.size(); i++)
    {
        if(vbNotIncludedMP[i])
            continue;

        MapPoint* pMP = vpMP[i];

        if(pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));

        if(nLoopKF==0)
        {
            pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
            pMP->UpdateNormalAndDepth();
        }
        else
        {
            pMP->mPosGBA.create(3,1,CV_32F);
            Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
            pMP->mnBAGlobalForKF = nLoopKF;
        }
    }
    std::cout << "overall ends" << std::endl;

}

int Optimizer::PoseOptimization(Frame *pFrame)
{
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    int nInitialCorrespondences=0;

    // Set Frame vertex
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);

    // Set MapPoint vertices
    const int N = pFrame->N;

    vector<g2o::EdgeSE3ProjectXYZOnlyPose*> vpEdgesMono;
    vector<size_t> vnIndexEdgeMono;
    vpEdgesMono.reserve(N);
    vnIndexEdgeMono.reserve(N);

    vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose*> vpEdgesStereo;
    vector<size_t> vnIndexEdgeStereo;
    vpEdgesStereo.reserve(N);
    vnIndexEdgeStereo.reserve(N);

    const float deltaMono = sqrt(5.991);
    const float deltaStereo = sqrt(7.815);


    {
    unique_lock<mutex> lock(MapPoint::mGlobalMutex);

    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = pFrame->mvpMapPoints[i];
        if(pMP)
        {
            // Monocular observation
            if(pFrame->mvuRight[i]<0)
            {
                nInitialCorrespondences++;
                pFrame->mvbOutlier[i] = false;

                Eigen::Matrix<double,2,1> obs;
                const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                obs << kpUn.pt.x, kpUn.pt.y;

                g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                e->setMeasurement(obs);
                const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(deltaMono);

                e->fx = pFrame->fx;
                e->fy = pFrame->fy;
                e->cx = pFrame->cx;
                e->cy = pFrame->cy;
                cv::Mat Xw = pMP->GetWorldPos();
                e->Xw[0] = Xw.at<float>(0);
                e->Xw[1] = Xw.at<float>(1);
                e->Xw[2] = Xw.at<float>(2);

                optimizer.addEdge(e);

                vpEdgesMono.push_back(e);
                vnIndexEdgeMono.push_back(i);
            }
            else  // Stereo observation
            {
                nInitialCorrespondences++;
                pFrame->mvbOutlier[i] = false;

                //SET EDGE
                Eigen::Matrix<double,3,1> obs;
                const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                const float &kp_ur = pFrame->mvuRight[i];
                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                e->setMeasurement(obs);
                const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                e->setInformation(Info);

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(deltaStereo);

                e->fx = pFrame->fx;
                e->fy = pFrame->fy;
                e->cx = pFrame->cx;
                e->cy = pFrame->cy;
                e->bf = pFrame->mbf;
                cv::Mat Xw = pMP->GetWorldPos();
                e->Xw[0] = Xw.at<float>(0);
                e->Xw[1] = Xw.at<float>(1);
                e->Xw[2] = Xw.at<float>(2);

                optimizer.addEdge(e);

                vpEdgesStereo.push_back(e);
                vnIndexEdgeStereo.push_back(i);
            }
        }

    }
    }


    if(nInitialCorrespondences<3)
        return 0;

    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    const float chi2Mono[4]={5.991,5.991,5.991,5.991};
    const float chi2Stereo[4]={7.815,7.815,7.815, 7.815};
    const int its[4]={10,10,10,10};

    int nBad=0;
    for(size_t it=0; it<4; it++)
    {

        vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
        optimizer.initializeOptimization(0);
        optimizer.optimize(its[it]);

        nBad=0;
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
        {
            g2o::EdgeSE3ProjectXYZOnlyPose* e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];

            if(pFrame->mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2Mono[it])
            {
                pFrame->mvbOutlier[idx]=true;
                e->setLevel(1);
                nBad++;
            }
            else
            {
                pFrame->mvbOutlier[idx]=false;
                e->setLevel(0);
            }

            if(it==2)
                e->setRobustKernel(0);
        }

        for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++)
        {
            g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = vpEdgesStereo[i];

            const size_t idx = vnIndexEdgeStereo[i];

            if(pFrame->mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2Stereo[it])
            {
                pFrame->mvbOutlier[idx]=true;
                e->setLevel(1);
                nBad++;
            }
            else
            {
                e->setLevel(0);
                pFrame->mvbOutlier[idx]=false;
            }

            if(it==2)
                e->setRobustKernel(0);
        }

        if(optimizer.edges().size()<10)
            break;
    }

    // Recover optimized pose and return number of inliers
    g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    cv::Mat pose = Converter::toCvMat(SE3quat_recov);
    pFrame->SetPose(pose);

    return nInitialCorrespondences-nBad;
}

void Optimizer::LocalBundleAdjustment(KeyFrame *pKF, bool* pbStopFlag, Map* pMap)
{    
    // Local KeyFrames: First Breath Search from Current Keyframe
    list<KeyFrame*> lLocalKeyFrames;

    lLocalKeyFrames.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;

    const vector<KeyFrame*> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
    for(int i=0, iend=vNeighKFs.size(); i<iend; i++)
    {
        KeyFrame* pKFi = vNeighKFs[i];
        pKFi->mnBALocalForKF = pKF->mnId;
        if(!pKFi->isBad())
            lLocalKeyFrames.push_back(pKFi);
    }

    // Local MapPoints seen in Local KeyFrames
    list<MapPoint*> lLocalMapPoints;
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin() , lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        vector<MapPoint*> vpMPs = (*lit)->GetMapPointMatches();
        for(vector<MapPoint*>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
        {
            MapPoint* pMP = *vit;
            if(pMP)
                if(!pMP->isBad())
                    if(pMP->mnBALocalForKF!=pKF->mnId)
                    {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF=pKF->mnId;
                    }
        }
    }

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    list<KeyFrame*> lFixedCameras;
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        map<KeyFrame*,size_t> observations = (*lit)->GetObservations();
        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(pKFi->mnBALocalForKF!=pKF->mnId && pKFi->mnBAFixedForKF!=pKF->mnId)
            {                
                pKFi->mnBAFixedForKF=pKF->mnId;
                if(!pKFi->isBad())
                    lFixedCameras.push_back(pKFi);
            }
        }
    }

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    unsigned long maxKFid = 0;

    // Set Local KeyFrame vertices
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(pKFi->mnId==0);
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
    }

    // Set Fixed KeyFrame vertices
    for(list<KeyFrame*>::iterator lit=lFixedCameras.begin(), lend=lFixedCameras.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
    }

    // Set MapPoint vertices
    const int nExpectedSize = (lLocalKeyFrames.size()+lFixedCameras.size())*lLocalMapPoints.size();

    vector<g2o::EdgeSE3ProjectXYZ*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);

    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        int id = pMP->mnId+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const map<KeyFrame*,size_t> observations = pMP->GetObservations();

        //Set edges
        for(map<KeyFrame*,size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(!pKFi->isBad())
            {                
                const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];

                // Monocular observation
                if(pKFi->mvuRight[mit->second]<0)
                {
                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;

                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);
                }
                else // Stereo observation
                {
                    Eigen::Matrix<double,3,1> obs;
                    const float kp_ur = pKFi->mvuRight[mit->second];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                    e->setInformation(Info);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;
                    e->bf = pKFi->mbf;

                    optimizer.addEdge(e);
                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKFi);
                    vpMapPointEdgeStereo.push_back(pMP);
                }
            }
        }
    }

    if(pbStopFlag)
        if(*pbStopFlag)
            return;

    optimizer.initializeOptimization();
    optimizer.optimize(5);

    bool bDoMore= true;

    if(pbStopFlag)
        if(*pbStopFlag)
            bDoMore = false;

    if(bDoMore)
    {

    // Check inlier observations
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            e->setLevel(1);
        }

        e->setRobustKernel(0);
    }

    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
        MapPoint* pMP = vpMapPointEdgeStereo[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>7.815 || !e->isDepthPositive())
        {
            e->setLevel(1);
        }

        e->setRobustKernel(0);
    }

    // Optimize again without the outliers

    optimizer.initializeOptimization(0);
    optimizer.optimize(10);

    }

    vector<pair<KeyFrame*,MapPoint*> > vToErase;
    vToErase.reserve(vpEdgesMono.size()+vpEdgesStereo.size());

    // Check inlier observations       
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }

    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
        MapPoint* pMP = vpMapPointEdgeStereo[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>7.815 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }

    // Get Map Mutex
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    if(!vToErase.empty())
    {
        for(size_t i=0;i<vToErase.size();i++)
        {
            KeyFrame* pKFi = vToErase[i].first;
            MapPoint* pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }

    // Recover optimized data

    //Keyframes
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKF = *lit;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        pKF->SetPose(Converter::toCvMat(SE3quat));
    }

    //Points
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
    }
}


void Optimizer::OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections, const bool &bFixScale)
{
    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    g2o::BlockSolver_7_3::LinearSolverType * linearSolver =
           new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
    g2o::BlockSolver_7_3 * solver_ptr= new g2o::BlockSolver_7_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    solver->setUserLambdaInit(1e-16);
    optimizer.setAlgorithm(solver);

    const vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    const vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();

    const unsigned int nMaxKFid = pMap->GetMaxKFid();

    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vScw(nMaxKFid+1);
    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vCorrectedSwc(nMaxKFid+1);
    vector<g2o::VertexSim3Expmap*> vpVertices(nMaxKFid+1);

    const int minFeat = 100;

    // Set KeyFrame vertices
    for(size_t i=0, iend=vpKFs.size(); i<iend;i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

        const int nIDi = pKF->mnId;

        LoopClosing::KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pKF);

        if(it!=CorrectedSim3.end())
        {
            vScw[nIDi] = it->second;
            VSim3->setEstimate(it->second);
        }
        else
        {
            Eigen::Matrix<double,3,3> Rcw = Converter::toMatrix3d(pKF->GetRotation());
            Eigen::Matrix<double,3,1> tcw = Converter::toVector3d(pKF->GetTranslation());
            g2o::Sim3 Siw(Rcw,tcw,1.0);
            vScw[nIDi] = Siw;
            VSim3->setEstimate(Siw);
        }

        if(pKF==pLoopKF)
            VSim3->setFixed(true);

        VSim3->setId(nIDi);
        VSim3->setMarginalized(false);
        VSim3->_fix_scale = bFixScale;

        optimizer.addVertex(VSim3);

        vpVertices[nIDi]=VSim3;
    }


    set<pair<long unsigned int,long unsigned int> > sInsertedEdges;

    const Eigen::Matrix<double,7,7> matLambda = Eigen::Matrix<double,7,7>::Identity();

    // Set Loop edges
    for(map<KeyFrame *, set<KeyFrame *> >::const_iterator mit = LoopConnections.begin(), mend=LoopConnections.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        const long unsigned int nIDi = pKF->mnId;
        const set<KeyFrame*> &spConnections = mit->second;
        const g2o::Sim3 Siw = vScw[nIDi];
        const g2o::Sim3 Swi = Siw.inverse();

        for(set<KeyFrame*>::const_iterator sit=spConnections.begin(), send=spConnections.end(); sit!=send; sit++)
        {
            const long unsigned int nIDj = (*sit)->mnId;
            if((nIDi!=pCurKF->mnId || nIDj!=pLoopKF->mnId) && pKF->GetWeight(*sit)<minFeat)
                continue;

            const g2o::Sim3 Sjw = vScw[nIDj];
            const g2o::Sim3 Sji = Sjw * Swi;

            g2o::EdgeSim3* e = new g2o::EdgeSim3();
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            e->setMeasurement(Sji);

            e->information() = matLambda;

            optimizer.addEdge(e);

            sInsertedEdges.insert(make_pair(min(nIDi,nIDj),max(nIDi,nIDj)));
        }
    }

    // Set normal edges
    for(size_t i=0, iend=vpKFs.size(); i<iend; i++)
    {
        KeyFrame* pKF = vpKFs[i];

        const int nIDi = pKF->mnId;

        g2o::Sim3 Swi;

        LoopClosing::KeyFrameAndPose::const_iterator iti = NonCorrectedSim3.find(pKF);

        if(iti!=NonCorrectedSim3.end())
            Swi = (iti->second).inverse();
        else
            Swi = vScw[nIDi].inverse();

        KeyFrame* pParentKF = pKF->GetParent();

        // Spanning tree edge
        if(pParentKF)
        {
            int nIDj = pParentKF->mnId;

            g2o::Sim3 Sjw;

            LoopClosing::KeyFrameAndPose::const_iterator itj = NonCorrectedSim3.find(pParentKF);

            if(itj!=NonCorrectedSim3.end())
                Sjw = itj->second;
            else
                Sjw = vScw[nIDj];

            g2o::Sim3 Sji = Sjw * Swi;

            g2o::EdgeSim3* e = new g2o::EdgeSim3();
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            e->setMeasurement(Sji);

            e->information() = matLambda;
            optimizer.addEdge(e);
        }

        // Loop edges
        const set<KeyFrame*> sLoopEdges = pKF->GetLoopEdges();
        for(set<KeyFrame*>::const_iterator sit=sLoopEdges.begin(), send=sLoopEdges.end(); sit!=send; sit++)
        {
            KeyFrame* pLKF = *sit;
            if(pLKF->mnId<pKF->mnId)
            {
                g2o::Sim3 Slw;

                LoopClosing::KeyFrameAndPose::const_iterator itl = NonCorrectedSim3.find(pLKF);

                if(itl!=NonCorrectedSim3.end())
                    Slw = itl->second;
                else
                    Slw = vScw[pLKF->mnId];

                g2o::Sim3 Sli = Slw * Swi;
                g2o::EdgeSim3* el = new g2o::EdgeSim3();
                el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pLKF->mnId)));
                el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
                el->setMeasurement(Sli);
                el->information() = matLambda;
                optimizer.addEdge(el);
            }
        }

        // Covisibility graph edges
        const vector<KeyFrame*> vpConnectedKFs = pKF->GetCovisiblesByWeight(minFeat);
        for(vector<KeyFrame*>::const_iterator vit=vpConnectedKFs.begin(); vit!=vpConnectedKFs.end(); vit++)
        {
            KeyFrame* pKFn = *vit;
            if(pKFn && pKFn!=pParentKF && !pKF->hasChild(pKFn) && !sLoopEdges.count(pKFn))
            {
                if(!pKFn->isBad() && pKFn->mnId<pKF->mnId)
                {
                    if(sInsertedEdges.count(make_pair(min(pKF->mnId,pKFn->mnId),max(pKF->mnId,pKFn->mnId))))
                        continue;

                    g2o::Sim3 Snw;

                    LoopClosing::KeyFrameAndPose::const_iterator itn = NonCorrectedSim3.find(pKFn);

                    if(itn!=NonCorrectedSim3.end())
                        Snw = itn->second;
                    else
                        Snw = vScw[pKFn->mnId];

                    g2o::Sim3 Sni = Snw * Swi;

                    g2o::EdgeSim3* en = new g2o::EdgeSim3();
                    en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFn->mnId)));
                    en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
                    en->setMeasurement(Sni);
                    en->information() = matLambda;
                    optimizer.addEdge(en);
                }
            }
        }
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(20);

    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
    for(size_t i=0;i<vpKFs.size();i++)
    {
        KeyFrame* pKFi = vpKFs[i];

        const int nIDi = pKFi->mnId;

        g2o::VertexSim3Expmap* VSim3 = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(nIDi));
        g2o::Sim3 CorrectedSiw =  VSim3->estimate();
        vCorrectedSwc[nIDi]=CorrectedSiw.inverse();
        Eigen::Matrix3d eigR = CorrectedSiw.rotation().toRotationMatrix();
        Eigen::Vector3d eigt = CorrectedSiw.translation();
        double s = CorrectedSiw.scale();

        eigt *=(1./s); //[R t/s;0 1]

        cv::Mat Tiw = Converter::toCvSE3(eigR,eigt);

        pKFi->SetPose(Tiw);
    }

    // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
    for(size_t i=0, iend=vpMPs.size(); i<iend; i++)
    {
        MapPoint* pMP = vpMPs[i];

        if(pMP->isBad())
            continue;

        int nIDr;
        if(pMP->mnCorrectedByKF==pCurKF->mnId)
        {
            nIDr = pMP->mnCorrectedReference;
        }
        else
        {
            KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();
            nIDr = pRefKF->mnId;
        }


        g2o::Sim3 Srw = vScw[nIDr];
        g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

        cv::Mat P3Dw = pMP->GetWorldPos();
        Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
        Eigen::Matrix<double,3,1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));

        cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
        pMP->SetWorldPos(cvCorrectedP3Dw);

        pMP->UpdateNormalAndDepth();
    }
}

int Optimizer::OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches1, g2o::Sim3 &g2oS12, const float th2, const bool bFixScale)
{
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    // Calibration
    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;

    // Camera poses
    const cv::Mat R1w = pKF1->GetRotation();
    const cv::Mat t1w = pKF1->GetTranslation();
    const cv::Mat R2w = pKF2->GetRotation();
    const cv::Mat t2w = pKF2->GetTranslation();

    // Set Sim3 vertex
    g2o::VertexSim3Expmap * vSim3 = new g2o::VertexSim3Expmap();
    vSim3->_fix_scale=bFixScale;
    vSim3->setEstimate(g2oS12);
    vSim3->setId(0);
    vSim3->setFixed(false);
    vSim3->_principle_point1[0] = K1.at<float>(0,2);
    vSim3->_principle_point1[1] = K1.at<float>(1,2);
    vSim3->_focal_length1[0] = K1.at<float>(0,0);
    vSim3->_focal_length1[1] = K1.at<float>(1,1);
    vSim3->_principle_point2[0] = K2.at<float>(0,2);
    vSim3->_principle_point2[1] = K2.at<float>(1,2);
    vSim3->_focal_length2[0] = K2.at<float>(0,0);
    vSim3->_focal_length2[1] = K2.at<float>(1,1);
    optimizer.addVertex(vSim3);

    // Set MapPoint vertices
    const int N = vpMatches1.size();
    const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
    vector<g2o::EdgeSim3ProjectXYZ*> vpEdges12;
    vector<g2o::EdgeInverseSim3ProjectXYZ*> vpEdges21;
    vector<size_t> vnIndexEdge;

    vnIndexEdge.reserve(2*N);
    vpEdges12.reserve(2*N);
    vpEdges21.reserve(2*N);

    const float deltaHuber = sqrt(th2);

    int nCorrespondences = 0;

    for(int i=0; i<N; i++)
    {
        if(!vpMatches1[i])
            continue;

        MapPoint* pMP1 = vpMapPoints1[i];
        MapPoint* pMP2 = vpMatches1[i];

        const int id1 = 2*i+1;
        const int id2 = 2*(i+1);

        const int i2 = pMP2->GetIndexInKeyFrame(pKF2);

        if(pMP1 && pMP2)
        {
            if(!pMP1->isBad() && !pMP2->isBad() && i2>=0)
            {
                g2o::VertexSBAPointXYZ* vPoint1 = new g2o::VertexSBAPointXYZ();
                cv::Mat P3D1w = pMP1->GetWorldPos();
                cv::Mat P3D1c = R1w*P3D1w + t1w;
                vPoint1->setEstimate(Converter::toVector3d(P3D1c));
                vPoint1->setId(id1);
                vPoint1->setFixed(true);
                optimizer.addVertex(vPoint1);

                g2o::VertexSBAPointXYZ* vPoint2 = new g2o::VertexSBAPointXYZ();
                cv::Mat P3D2w = pMP2->GetWorldPos();
                cv::Mat P3D2c = R2w*P3D2w + t2w;
                vPoint2->setEstimate(Converter::toVector3d(P3D2c));
                vPoint2->setId(id2);
                vPoint2->setFixed(true);
                optimizer.addVertex(vPoint2);
            }
            else
                continue;
        }
        else
            continue;

        nCorrespondences++;

        // Set edge x1 = S12*X2
        Eigen::Matrix<double,2,1> obs1;
        const cv::KeyPoint &kpUn1 = pKF1->mvKeysUn[i];
        obs1 << kpUn1.pt.x, kpUn1.pt.y;

        g2o::EdgeSim3ProjectXYZ* e12 = new g2o::EdgeSim3ProjectXYZ();
        e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id2)));
        e12->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        e12->setMeasurement(obs1);
        const float &invSigmaSquare1 = pKF1->mvInvLevelSigma2[kpUn1.octave];
        e12->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare1);

        g2o::RobustKernelHuber* rk1 = new g2o::RobustKernelHuber;
        e12->setRobustKernel(rk1);
        rk1->setDelta(deltaHuber);
        optimizer.addEdge(e12);

        // Set edge x2 = S21*X1
        Eigen::Matrix<double,2,1> obs2;
        const cv::KeyPoint &kpUn2 = pKF2->mvKeysUn[i2];
        obs2 << kpUn2.pt.x, kpUn2.pt.y;

        g2o::EdgeInverseSim3ProjectXYZ* e21 = new g2o::EdgeInverseSim3ProjectXYZ();

        e21->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id1)));
        e21->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        e21->setMeasurement(obs2);
        float invSigmaSquare2 = pKF2->mvInvLevelSigma2[kpUn2.octave];
        e21->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare2);

        g2o::RobustKernelHuber* rk2 = new g2o::RobustKernelHuber;
        e21->setRobustKernel(rk2);
        rk2->setDelta(deltaHuber);
        optimizer.addEdge(e21);

        vpEdges12.push_back(e12);
        vpEdges21.push_back(e21);
        vnIndexEdge.push_back(i);
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(5);

    // Check inliers
    int nBad=0;
    for(size_t i=0; i<vpEdges12.size();i++)
    {
        g2o::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
        g2o::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
        if(!e12 || !e21)
            continue;

        if(e12->chi2()>th2 || e21->chi2()>th2)
        {
            size_t idx = vnIndexEdge[i];
            vpMatches1[idx]=static_cast<MapPoint*>(NULL);
            optimizer.removeEdge(e12);
            optimizer.removeEdge(e21);
            vpEdges12[i]=static_cast<g2o::EdgeSim3ProjectXYZ*>(NULL);
            vpEdges21[i]=static_cast<g2o::EdgeInverseSim3ProjectXYZ*>(NULL);
            nBad++;
        }
    }

    int nMoreIterations;
    if(nBad>0)
        nMoreIterations=10;
    else
        nMoreIterations=5;

    if(nCorrespondences-nBad<10)
        return 0;

    // Optimize again only with inliers

    optimizer.initializeOptimization();
    optimizer.optimize(nMoreIterations);

    int nIn = 0;
    for(size_t i=0; i<vpEdges12.size();i++)
    {
        g2o::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
        g2o::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
        if(!e12 || !e21)
            continue;

        if(e12->chi2()>th2 || e21->chi2()>th2)
        {
            size_t idx = vnIndexEdge[i];
            vpMatches1[idx]=static_cast<MapPoint*>(NULL);
        }
        else
            nIn++;
    }

    // Recover optimized Sim3
    g2o::VertexSim3Expmap* vSim3_recov = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(0));
    g2oS12= vSim3_recov->estimate();

    return nIn;
}

// local BA with point and cuboid, cuboid 2d error, point-cuboid error
void Optimizer::LocalBACameraPointCuboids2D(KeyFrame *pKF, bool *pbStopFlag, Map *pMap, bool fixCamera, bool fixPoint)
{
    // Local KeyFrames to optimize: First Breath Search from Current Keyframe
    vector<KeyFrame *> lLocalKeyFrames; // local KFs which share map points with current frame.

    lLocalKeyFrames.push_back(pKF); // get current KFs
    pKF->mnBALocalForKF = pKF->mnId;

    const vector<KeyFrame *> vNeighKFs = pKF->GetVectorCovisibleKeyFrames(); // directly get local keyframes.
    for (int i = 0, iend = vNeighKFs.size(); i < iend; i++)
    {
        KeyFrame *pKFi = vNeighKFs[i];
        pKFi->mnBALocalForKF = pKF->mnId;
        if (!pKFi->isBad())
            lLocalKeyFrames.push_back(pKFi); // get neighbor KFs
    }

    // Local MapPoints seen in Local KeyFrames. some points might not be seen by currentKF
    vector<MapPoint *> lLocalMapPoints;
    for (vector<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
    {
        vector<MapPoint *> vpMPs = (*lit)->GetMapPointMatches();
        for (vector<MapPoint *>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++)
        {
            MapPoint *pMP = *vit;
            if (pMP)
                if (!pMP->isBad())
                    if (pMP->mnBALocalForKF != pKF->mnId) // mnBALocalForKF  mnBAFixedForKF are marker
                    {
                        // if (whether_dynamic_object)
                        //     if (pMP->is_dynamic)
                        //         continue;
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF = pKF->mnId;
                    }
        }
    }

    vector<MapCuboid *> lLocalMapCuboids;
    for (vector<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
    {
        vector<MapCuboid *> vpMCs = (*lit)->mvpMapCuboid;
        for (vector<MapCuboid *>::iterator vit = vpMCs.begin(), vend = vpMCs.end(); vit != vend; vit++)
        {
            MapCuboid *pMC = *vit;
            if (pMC)
                if (!pMC->isBad())
                    if (pMC->mnBALocalForKF != pKF->mnId) // mnBALocalForKF  mnBAFixedForKF are marker
                    {
                        lLocalMapCuboids.push_back(pMC);
                        pMC->mnBALocalForKF = pKF->mnId;
                    }
        }
    }

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    vector<KeyFrame *> lFixedCameras;
    for (vector<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
    {
        map<KeyFrame *, size_t> observations = (*lit)->GetObservations();
        for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
        {
            KeyFrame *pKFi = mit->first;

            if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId) // not been added to lLocalKeyFrames, and lFixedCameras!
            {
                pKFi->mnBAFixedForKF = pKF->mnId;
                if (!pKFi->isBad())
                    lFixedCameras.push_back(pKFi);
            }
        }
    }

    // Fixed Keyframes by cuboids. Keyframes that see Local MapCuboids but that are not Local Keyframes
    for (vector<MapCuboid *>::iterator lit = lLocalMapCuboids.begin(), lend = lLocalMapCuboids.end(); lit != lend; lit++)
    {
        unordered_map<KeyFrame *, size_t> observations = (*lit)->GetObservations();
        for (unordered_map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
        {
            KeyFrame *pKFi = mit->first;

            if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId) // not been added to lLocalKeyFrames, and lFixedCameras!
            {
                pKFi->mnBAFixedForKF = pKF->mnId;
                if (!pKFi->isBad())
                    lFixedCameras.push_back(pKFi);
            }
        }
    }

    std::cout << "Optimizer: LocalBAPointObjects num of map points/ cuboids/ keyframes: " 
            << pMap->MapPointsInMap() << " " << pMap->MapCuboidsInMap() << " " << pMap->KeyFramesInMap() << std::endl;
    std::cout << "Optimizer: LocalBAPointObjects num of graph points/ cuboids/ keyframes/ fixkeyframe: " 
            << lLocalMapPoints.size() << " " << lLocalMapCuboids.size() << " " << lLocalKeyFrames.size() << " " << lFixedCameras.size() << std::endl;

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType *linearSolver; // BlockSolverX instead of BlockSolver63
    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if (pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    unsigned long maxKFid = 0;
    // Set Local KeyFrame vertices
    for (vector<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
    {
        KeyFrame *pKFi = *lit;
        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(pKFi->mnId == 0); // if first keyframe frame, set pose fixed.
        if (fixCamera)
            vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if (pKFi->mnId > maxKFid)
            maxKFid = pKFi->mnId;
    }

    // Set Fixed KeyFrame vertices
    for (vector<KeyFrame *>::iterator lit = lFixedCameras.begin(), lend = lFixedCameras.end(); lit != lend; lit++)
    {
        KeyFrame *pKFi = *lit;
        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if (pKFi->mnId > maxKFid)
            maxKFid = pKFi->mnId;
    }

    // #ifdef ObjectFixScale
    //     typedef g2o::VertexCuboidFixScale g2o_object_vertex;
    //     typedef g2o::EdgeSE3CuboidFixScaleProj g2o_camera_obj_2d_edge;
    typedef g2o::VertexCuboid g2o_cuboid_vertex;
    typedef g2o::EdgeSE3CuboidProj g2o_camera_obj_2d_edge;

    // Set MapCuboid vertices
    long int maxCuboidid = 0;
    if (1)
    {
        for (vector<MapCuboid *>::iterator lit = lLocalMapCuboids.begin(), lend = lLocalMapCuboids.end(); lit != lend; lit++)
        {
            MapCuboid *pMCuboid = *lit;
            // cv::Mat cube_pose = pMCuboid->GetWorldPos();  // set as cv mat
            g2o::cuboid cube_pose = pMCuboid->cuboid_global_data;
            std::cout << "cuboid vertex init pose: "<< cube_pose.toMinimalVector().transpose() << std::endl;
            g2o_cuboid_vertex *vCuboid = new g2o_cuboid_vertex();

            vCuboid->setEstimate(cube_pose);
            vCuboid->whether_fixrollpitch = true; // only rotate along object z axis.
            vCuboid->whether_fixheight = true;   //may make camera height estimation bad

            int id = pMCuboid->mnId + maxKFid + 1;
            vCuboid->setId(id);
            vCuboid->setFixed(false);
            optimizer.addVertex(vCuboid);
            if (pMCuboid->mnId > maxCuboidid)
                maxCuboidid = pMCuboid->mnId;
        }
    }

    // Set MapPoint vertices
    const int nExpectedSize = (lLocalKeyFrames.size() + lFixedCameras.size()) * lLocalMapPoints.size();
    vector<g2o::EdgeSE3ProjectXYZ *> vpEdgesMono; // a camera + map point
    vpEdgesMono.reserve(nExpectedSize);
    vector<KeyFrame *> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);
    vector<MapPoint *> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);
    vector<g2o::EdgeStereoSE3ProjectXYZ *> vpEdgesStereo; // left + right camera + map point
    vpEdgesStereo.reserve(nExpectedSize);
    vector<KeyFrame *> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);
    vector<MapPoint *> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);

    // set up map point and camera-point edges
    int obs_greater_one_points = 0;
    int obs_one_points = 0;
    for (vector<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
    {
        MapPoint *pMP = *lit;

        if (pMP->Observations() == 1) // HACK by me, skip observation one point, which is initialized by my depth! won't affect previous mono.
            continue;

        g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));

        int id = pMP->mnId + maxKFid + maxCuboidid + 2;
        vPoint->setId(id);
        if (fixPoint)
            vPoint->setFixed(true);
        else
            vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const map<KeyFrame *, size_t> observations = pMP->GetObservations();
        if (observations.size() == 1)
            obs_one_points++;
        if (observations.size() > 1)
            obs_greater_one_points++;

        //Set edges
        for (map<KeyFrame *, size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
        {
            KeyFrame *pKFi = mit->first;

            if (!pKFi->isBad())
            {
                const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];

                // Monocular observation
                if (pKFi->mvuRight[mit->second] < 0)
                {
                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    g2o::EdgeSE3ProjectXYZ *e = new g2o::EdgeSE3ProjectXYZ(); // camera point edge.  there is no camera-camera odometry edge.

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId))); //get vertex based on Id.
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;

                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);
                }
                else // Stereo observation
                {
                    Eigen::Matrix<double, 3, 1> obs;
                    const float kp_ur = pKFi->mvuRight[mit->second];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
                    e->setInformation(Info);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;
                    e->bf = pKFi->mbf;

                    optimizer.addEdge(e);
                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKFi);
                    vpMapPointEdgeStereo.push_back(pMP);
                }
            }
        }
    }

    // set up point-object 3d association
    std::vector<g2o::EdgePointCuboidOnlyObject *> vpEdgesPointCuboid; // edge between cuboid and points
    vpEdgesPointCuboid.reserve(lLocalMapPoints.size()); // only change capacity(), not change size()
    if (optimize_with_pt_obj_3d)
    {
        // step 1: get object point association.
        // can we do this when validate cuboids?? then don't need to do it every time?
        // get ba points from point cuboid association
        int point_object_threshold = 2; // select points that have observe cuboid more than 2
        vector<vector<Vector3d>> all_object_ba_points_xyz(lLocalMapCuboids.size()); // use for filter
        vector<vector<MapPoint*>> all_object_ba_points_pts(lLocalMapCuboids.size()); // use for filter and show
        for (size_t i = 0; i < lLocalMapCuboids.size(); i++)
        {
            MapCuboid *pMC = lLocalMapCuboids[i];
            // int largest_point_obs_num = pMC->largest_point_observations;
            // point_object_threshold = std::max(int(largest_point_obs_num * 0.4), 2); // whether use adaptive threshold or fixed.

            const std::vector<MapPoint *> &UniquePoints = pMC->GetUniqueMapPoints();
            for (size_t j = 0; j < UniquePoints.size(); j++)
                if (UniquePoints[j])
                    if (!UniquePoints[j]->isBad())
                        if (UniquePoints[j]->MapObjObservations[pMC] > point_object_threshold)
                        {
                            all_object_ba_points_pts[i].push_back(UniquePoints[j]);
                            all_object_ba_points_xyz[i].push_back(Converter::toVector3d(UniquePoints[j]->GetWorldPos()));
                        }
        }

        // step 2: filter bad object points for BA
        double coarse_threshold = 4;
        double fine_threshold = 3;
        for (size_t i = 0; i < lLocalMapCuboids.size(); i++)
        {
            MapCuboid *pMObj = lLocalMapCuboids[i];
            pMObj->used_points_in_BA_filtered.clear();
            // compute the mean, eliminate outlier points.
            Eigen::Vector3d mean_point;
            mean_point.setZero();
            for (size_t j = 0; j < all_object_ba_points_xyz[i].size(); j++)
                mean_point += all_object_ba_points_xyz[i][j];
            mean_point /= (double)(all_object_ba_points_xyz[i].size());

            //NOTE  filtering of points!!!  remove outlier points
            Eigen::Vector3d mean_point_2;
            mean_point_2.setZero();
            int valid_point_num = 0;
            for (size_t j = 0; j < all_object_ba_points_xyz[i].size(); j++)
                if ((mean_point - all_object_ba_points_xyz[i][j]).norm() < coarse_threshold)
                {
                    mean_point_2 += all_object_ba_points_xyz[i][j];
                    valid_point_num++;
                }
            mean_point_2 /= (double)valid_point_num;

            Eigen::Vector3d mean_point_final;
            mean_point_final.setZero();
            std::vector<Eigen::Vector3d> good_points; // for car, if points are 4 meters away from center, usually outlier.
            for (size_t j = 0; j < all_object_ba_points_xyz[i].size(); j++)
            {
                if ((mean_point_2 - all_object_ba_points_xyz[i][j]).norm() < fine_threshold)
                {
                    mean_point_final += all_object_ba_points_xyz[i][j];
                    good_points.push_back(all_object_ba_points_xyz[i][j]);
                    pMObj->used_points_in_BA_filtered.push_back(all_object_ba_points_pts[i][j]);
                }
            }
            mean_point_final /= (double)(good_points.size());
            all_object_ba_points_xyz[i].clear();
            all_object_ba_points_xyz[i] = good_points;

            // if ((all_object_ba_points_xyz[i].size() > 5)) // whether want to initialize object position to be center of points
            // {
            //     g2o_cuboid_vertex *vObject = static_cast<g2o_cuboid_vertex *>(optimizer.vertex(pMObj->mnId + maxKFid + 1));
            //     g2o::cuboid tempcube = vObject->estimate();
            //     tempcube.setTranslation(mean_point_final);
            //     vObject->setEstimate(tempcube);
            //     std::cout << "cuboid vertex new pose: "<< tempcube.toMinimalVector().transpose() << std::endl;
            // }
        }

        // step 3: add point objct 3d measument, set use fixed point or to optimize point
        for (size_t i = 0; i < lLocalMapCuboids.size(); i++) // no need to optimize all objects...., use local KF's map objects?
        {
            // an object connected to many fixed points. optimize only object
            MapCuboid *pMC = lLocalMapCuboids[i];
            g2o::EdgePointCuboidOnlyObject *e = new g2o::EdgePointCuboidOnlyObject();
            for (size_t j = 0; j < all_object_ba_points_xyz[i].size(); j++)
                e->object_points.push_back(all_object_ba_points_xyz[i][j]);

            if (e->object_points.size() > 10)
            {
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pMC->mnId + maxKFid + 1)));
                Eigen::Matrix3d info;
                info.setIdentity();
                e->setInformation(info);
                e->max_outside_margin_ratio = 1;
                // e->prior_object_half_size = Eigen::Vector3d(0.25, 0.08, 0.18);
                std::cout << "Optimizer: get error from point object: " << e->computeError_debug().transpose() << std::endl;
                optimizer.addEdge(e);
                vpEdgesPointCuboid.push_back(e);
            }
        }
    }

    int total_edge_num_cam_obj = 0;

    // set up camera and cuboid edge: 2d measurement
    // float thHuberBbox2d = 80;
    // double ba_weight_bbox = 1.0; // todo: need modify
    std::vector<g2o::EdgeSE3CuboidProj *> vpEdgesCameraCuboidBbox;
    if(optimize_with_cuboid_2d)
    {
        for (vector<MapCuboid *>::iterator lit = lLocalMapCuboids.begin(), lend = lLocalMapCuboids.end(); lit != lend; lit++)
        {
            MapCuboid *pMCuboid = *lit;
            const unordered_map<KeyFrame *, size_t> observations = pMCuboid->GetObservations();
            for (unordered_map<KeyFrame *, size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
            {
                KeyFrame *pKFi = mit->first;
                if (!pKFi->isBad())
                {
                    g2o::EdgeSE3CuboidProj* e = new g2o::EdgeSE3CuboidProj();
                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));                   // camera
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pMCuboid->mnId + maxKFid + 1))); // object
                    const MapCuboid *local_object = pKFi->local_cuboids[mit->second];
                    Eigen::Matrix3d calib;
                    for (size_t ttt = 0; ttt < 3; ttt++)
                        for (size_t zzz = 0; zzz < 3; zzz++)
                            calib(ttt,zzz) = pKFi->mK.at<float>(ttt,zzz);
                    e->Kalib = calib; 
                    e->setMeasurement(local_object->bbox_vec);
                    std::cout << "local_object->bbox_vec " << local_object->bbox_vec.transpose() << std::endl;

                    int object_boundary_margin = 5;
                    cv::Rect bbox_2d = local_object->bbox_2d;
                    // object should be in FOV, otherwise bad for this edge
                    if ((bbox_2d.x > object_boundary_margin) && (bbox_2d.y > object_boundary_margin) && (bbox_2d.x + bbox_2d.width < 640 - object_boundary_margin) &&
                        (bbox_2d.y + bbox_2d.height < 480 - object_boundary_margin))
                    {
                        Eigen::Vector4d inv_sigma;inv_sigma<<1,1,1,1;
                        double meas_quality = local_object->meas_quality; //2.0; //0.75;
                        inv_sigma = inv_sigma*ba_weight_bbox*meas_quality;; // point sigma<1, object error is usually large, no need to set large sigma...
                        Eigen::Matrix4d info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
                        // same info
                        // Eigen::Vector4d inv_sigma;inv_sigma<<1,1,1,1;
                        // inv_sigma = inv_sigma * ba_weight_bbox; // point sigma<1, object error is usually large, no need to set large sigma...
                        // Eigen::Matrix4d camera_object_sigma = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
                        // Eigen::Matrix4d info = camera_object_sigma * local_object->meas_quality * local_object->meas_quality;
                        
                        e->setInformation(info);
                        std::cout << "Optimizer: get error from bbox 2d : \n" << e->get_error_norm() << std::endl;
                        const float thHuberObject = sqrt(thHuberBbox2d); // object reprojection error is usually large
                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberObject);
                        e->setId(total_edge_num_cam_obj);
                        total_edge_num_cam_obj++;
                        optimizer.addEdge(e);
                        vpEdgesCameraCuboidBbox.push_back(e);
                        // std::cout << "add cam cuboid edge " << pKFi->mnId << " " << pMCuboid->object_graph_id + maxKFid + 1 << std::endl;
                    } // loop 2d bbox
                }// loop if !pKFi->isBad()
            }// loop observation
        }// loop cuboid
    }

    // set up camera and cuboid edge: 2d measurement
    // float thHuberConer2d = 10;
    // float ba_weight_corner = 0.1;
    std::vector<g2o::EdgeSE3CuboidCornerProj *> vpEdgesCameraCuboidCorner;
    if(optimize_with_corners_2d)
    {
        for (vector<MapCuboid *>::iterator lit = lLocalMapCuboids.begin(), lend = lLocalMapCuboids.end(); lit != lend; lit++)
        {
            MapCuboid *pMCuboid = *lit;
            const unordered_map<KeyFrame *, size_t> observations = pMCuboid->GetObservations();
            for (unordered_map<KeyFrame *, size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
            {
                KeyFrame *pKFi = mit->first;
                if (!pKFi->isBad())
                {
                    g2o::EdgeSE3CuboidCornerProj* e = new g2o::EdgeSE3CuboidCornerProj();
                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));                   // camera
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pMCuboid->mnId + maxKFid + 1))); // object
                    const MapCuboid *local_object = pKFi->local_cuboids[mit->second];
                    Eigen::Matrix3d calib;
                    for (size_t ttt = 0; ttt < 3; ttt++)
                        for (size_t zzz = 0; zzz < 3; zzz++)
                            calib(ttt,zzz) = pKFi->mK.at<float>(ttt,zzz);
                    e->Kalib = calib; 

                    Eigen::Matrix<double, 16, 1> corners_2d_vec;
                    for (size_t i = 0; i < 8; i++)
                    {
                        corners_2d_vec(i*2) = local_object->box_corners_2d(0,i);
                        corners_2d_vec(i*2+1) = local_object->box_corners_2d(1,i);
                    }
                    e->setMeasurement(corners_2d_vec);
                    std::cout << "local_object->corners_2d_vec " << corners_2d_vec.transpose() << std::endl;

                    int object_boundary_margin = 5;
                    cv::Rect bbox_2d = local_object->bbox_2d;
                    // object should be in FOV, otherwise bad for this edge
                    if ((bbox_2d.x > object_boundary_margin) && (bbox_2d.y > object_boundary_margin) && (bbox_2d.x + bbox_2d.width < 640 - object_boundary_margin) &&
                        (bbox_2d.y + bbox_2d.height < 480 - object_boundary_margin))
                    {
                        Eigen::Matrix<double, 16, 1> inv_sigma;
                        inv_sigma << 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1;
                        double meas_quality = local_object->meas_quality; //2.0; //0.75;
                        inv_sigma = inv_sigma*ba_weight_corner*meas_quality;; // point sigma<1, object error is usually large, no need to set large sigma...
                        Eigen::Matrix<double, 16, 16> info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
  
                        e->setInformation(info);
                        // std::cout << "info" << info << std::endl;
                        std::cout << "Optimizer: get error from corners 2d : \n" << e->get_error_norm() << std::endl;
                        const float thHuberObject = sqrt(thHuberConer2d); // object reprojection error is usually large
                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberObject);
                        e->setId(total_edge_num_cam_obj);
                        total_edge_num_cam_obj++;
                        optimizer.addEdge(e);
                        vpEdgesCameraCuboidCorner.push_back(e);
                        // std::cout << "add cam cuboid edge " << pKFi->mnId << " " << pMCuboid->object_graph_id + maxKFid + 1 << std::endl;
                    } // loop 2d bbox
                }// loop if !pKFi->isBad()
            }// loop observation
        }// loop cuboid
    }

    // set up camera and cuboid edge: 3d measurement
    // float thHuberSE3 = 900;
    // float ba_weight_SE3 = 1.0;
    std::vector<g2o::EdgeSE3Cuboid *> vpEdgesCameraCuboidSE3;
    if(optimize_with_cuboid_3d)
    {
        for (vector<MapCuboid *>::iterator lit = lLocalMapCuboids.begin(), lend = lLocalMapCuboids.end(); lit != lend; lit++)
        {
            MapCuboid *pMCuboid = *lit;
            const unordered_map<KeyFrame *, size_t> observations = pMCuboid->GetObservations();
            for (unordered_map<KeyFrame *, size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
            {
                KeyFrame *pKFi = mit->first;
                if (!pKFi->isBad())
                {
                    const MapCuboid *local_object = pKFi->mvpMapCuboid[mit->second];

                    g2o::EdgeSE3Cuboid* e = new g2o::EdgeSE3Cuboid();
                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));                   // camera
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pMCuboid->object_graph_id + maxKFid + 1))); // object

                    e->setMeasurement(local_object->cuboid_local_meas);
                    Vector9d inv_sigma;inv_sigma<<1,1,1,1,1,1,1,1,1;
                    double meas_quality = 0.75; //2.0; //0.75;
                    inv_sigma = inv_sigma*ba_weight_SE3*meas_quality;// point sigma<1, object error is usually large, no need to set large sigma...
                    Matrix9d info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
                    e->setInformation(info);
                    std::cout << "Optimizer: get error from se3: " << e->get_error_norm() << std::endl;
                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberSE3);
                    e->setId(total_edge_num_cam_obj);
                    total_edge_num_cam_obj++;
                    optimizer.addEdge(e);
                    vpEdgesCameraCuboidSE3.push_back(e);
                    // std::cout << "add cam cuboid edge " << pKFi->mnId << " " << pMCuboid->object_graph_id + maxKFid + 1 << std::endl;
                } // loop keyframe bad
            } // loop observation
        }// loop lLocalMapCuboids
    }


    std::cout << "Optimizer: LocalBAPointObjects graph vertex points/ objects/ frames: " << lLocalMapPoints.size() << " "<< lLocalMapCuboids.size() << "  " << maxKFid << std::endl;
    std::cout << "Optimizer: LocalBAPointObjects graph edge cam-point/ cam-cuboid/ cuboid-point: " << vpMapPointEdgeMono.size()<<" "<< total_edge_num_cam_obj<<" "<< vpEdgesPointCuboid.size()<<std::endl;

    if (pbStopFlag)
        if (*pbStopFlag)
            return;

    optimizer.initializeOptimization();
    optimizer.optimize(5);

    bool bDoMore = true;

    if (pbStopFlag)
        if (*pbStopFlag)
            bDoMore = false;

    if (bDoMore)
    {
        // Check inlier observations
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
        {
            g2o::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
            MapPoint *pMP = vpMapPointEdgeMono[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > 5.991 || !e->isDepthPositive())
            {
                e->setLevel(1); // don't optimize this edge.
            }

            e->setRobustKernel(0);
        }

        for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
        {
            g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
            MapPoint *pMP = vpMapPointEdgeStereo[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > 7.815 || !e->isDepthPositive())
            {
                e->setLevel(1);
            }

            e->setRobustKernel(0);
        }

        for (size_t i = 0, iend = vpEdgesCameraCuboidBbox.size(); i < iend; i++)
        {
            g2o::EdgeSE3CuboidProj *e = vpEdgesCameraCuboidBbox[i];
            if (e->error().norm() > thHuberBbox2d)
            {
                e->setLevel(1);
            }
        }

        for (size_t i = 0, iend = vpEdgesCameraCuboidCorner.size(); i < iend; i++)
        {
            g2o::EdgeSE3CuboidCornerProj *e = vpEdgesCameraCuboidCorner[i];
            if (e->error().norm() > thHuberConer2d)
            {
                e->setLevel(1);
            }
        }

        for (size_t i = 0, iend = vpEdgesCameraCuboidSE3.size(); i < iend; i++)
        {
            g2o::EdgeSE3Cuboid *e = vpEdgesCameraCuboidSE3[i];
            if (e->error().norm() > thHuberSE3)
            {
                e->setLevel(1);
            }
        }

        for (size_t i = 0, iend = vpEdgesPointCuboid.size(); i < iend; i++)
        {
            g2o::EdgePointCuboidOnlyObject *e = vpEdgesPointCuboid[i];
            // to do: not sure how to filter outlier
            // if (e->error().norm() > 80)
            // {
            //     e->setLevel(1);
            // }
        }

        // Optimize again without the outliers
        optimizer.initializeOptimization(0);
        optimizer.optimize(10);
    }

    vector<pair<KeyFrame *, MapPoint *>> vToErase;
    vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

    // Check inlier observations
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
    {
        g2o::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
        MapPoint *pMP = vpMapPointEdgeMono[i];

        if (pMP->isBad())
            continue;

        if (e->chi2() > 5.991 || !e->isDepthPositive())
        {
            KeyFrame *pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi, pMP));
        }
    }

    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
    {
        g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
        MapPoint *pMP = vpMapPointEdgeStereo[i];

        if (pMP->isBad())
            continue;

        if (e->chi2() > 7.815 || !e->isDepthPositive())
        {
            KeyFrame *pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi, pMP));
        }
    }

    // Get Map Mutex
    // if (parallel_mapping)
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    if (!vToErase.empty())
    {
        for (size_t i = 0; i < vToErase.size(); i++)
        {
            KeyFrame *pKFi = vToErase[i].first;
            MapPoint *pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }

    // Recover optimized data
    //Keyframes
    for (vector<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
    {
        KeyFrame *pKFrame = *lit;
        pKFrame->mnBALocalForKF = 0; // added ?
        g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKFrame->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        pKFrame->SetPose(Converter::toCvMat(SE3quat));
    }

    //Points
    for (vector<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
    {
        MapPoint *pMP = *lit;
        pMP->mnBALocalForKF = 0;     // added ?
        // if (pMP->Observations() == 1)    // added ?
        //     continue;
        g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(pMP->mnId + maxKFid + maxCuboidid + 2));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
    }

    // fix camera // doesnot update
    for (vector<KeyFrame *>::iterator lit = lFixedCameras.begin(), lend = lFixedCameras.end(); lit != lend; lit++)
    {
        KeyFrame *pKFrame = *lit;
        pKFrame->mnBAFixedForKF = 0;
        pKFrame->mnBALocalForKF = 0;
    }

    // cuboids
    for (vector<MapCuboid *>::iterator lit = lLocalMapCuboids.begin(), lend = lLocalMapCuboids.end(); lit != lend; lit++)
    {
        MapCuboid *pMCuboid = *lit;
        pMCuboid->mnBALocalForKF = 0;
        pMCuboid->obj_been_optimized = true;
        g2o::VertexCuboid *vCuboid = static_cast<g2o::VertexCuboid *>(optimizer.vertex(pMCuboid->mnId + maxKFid + 1));
        g2o::cuboid pose_after_opti = vCuboid->estimate();
        pMCuboid->SetWorldPos(Converter::toCvMat(pose_after_opti.pose));
        pMCuboid->cuboid_global_opti = pose_after_opti;
        std::cout <<"id: " << pMCuboid->mnId <<" opti: "<<pose_after_opti.toMinimalVector().transpose() << std::endl;
    }
}

// local BA with point and cuboid, cuboid 2d error, point-cuboid error
void Optimizer::LocalBACameraPlaneCuboids(KeyFrame *pKF, bool *pbStopFlag, Map *pMap, bool fixCamera, bool fixPoint)
{
    // Local KeyFrames to optimize: First Breath Search from Current Keyframe
    vector<KeyFrame *> lLocalKeyFrames; // local KFs which share map points with current frame.

    lLocalKeyFrames.push_back(pKF); // get current KFs
    pKF->mnBALocalForKF = pKF->mnId;

    const vector<KeyFrame *> vNeighKFs = pKF->GetVectorCovisibleKeyFrames(); // directly get local keyframes.
    for (int i = 0, iend = vNeighKFs.size(); i < iend; i++)
    {
        KeyFrame *pKFi = vNeighKFs[i];
        pKFi->mnBALocalForKF = pKF->mnId;
        if (!pKFi->isBad())
            lLocalKeyFrames.push_back(pKFi); // get neighbor KFs
    }

    // Local MapPoints seen in Local KeyFrames. some points might not be seen by currentKF
    vector<MapPoint *> lLocalMapPoints;
    for (vector<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
    {
        vector<MapPoint *> vpMPs = (*lit)->GetMapPointMatches();
        for (vector<MapPoint *>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++)
        {
            MapPoint *pMP = *vit;
            if (pMP)
                if (!pMP->isBad())
                    if (pMP->mnBALocalForKF != pKF->mnId) // mnBALocalForKF  mnBAFixedForKF are marker
                    {
                        // if (whether_dynamic_object)
                        //     if (pMP->is_dynamic)
                        //         continue;
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF = pKF->mnId;
                    }
        }
    }

    vector<MapCuboid *> lLocalMapCuboids;
    for (vector<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
    {
        vector<MapCuboid *> vpMCs = (*lit)->mvpMapCuboid;
        for (vector<MapCuboid *>::iterator vit = vpMCs.begin(), vend = vpMCs.end(); vit != vend; vit++)
        {
            MapCuboid *pMC = *vit;
            if (pMC)
                if (!pMC->isBad())
                    if (pMC->mnBALocalForKF != pKF->mnId) // mnBALocalForKF  mnBAFixedForKF are marker
                    {
                        lLocalMapCuboids.push_back(pMC);
                        pMC->mnBALocalForKF = pKF->mnId;
                    }
        }
    }

    vector<MapPlane *> lLocalMapPlanes;
    for (vector<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
    {
        vector<MapPlane *> vpMPs = (*lit)->mvpMapPlanes;
        for (vector<MapPlane *>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++)
        {
            MapPlane *pMP = *vit;
            if (pMP)
                if (!pMP->isBad())
                    if (pMP->mnBALocalForKF != pKF->mnId) // mnBALocalForKF  mnBAFixedForKF are marker
                    {
                        lLocalMapPlanes.push_back(pMP);
                        pMP->mnBALocalForKF = pKF->mnId;
                    }
        }
    }
  
    
    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    vector<KeyFrame *> lFixedCameras;
    for (vector<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
    {
        map<KeyFrame *, size_t> observations = (*lit)->GetObservations();
        for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
        {
            KeyFrame *pKFi = mit->first;

            if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId) // not been added to lLocalKeyFrames, and lFixedCameras!
            {
                pKFi->mnBAFixedForKF = pKF->mnId;
                if (!pKFi->isBad())
                    lFixedCameras.push_back(pKFi);
            }
        }
    }

    // Fixed Keyframes by cuboids. Keyframes that see Local MapCuboids but that are not Local Keyframes
    for (vector<MapCuboid *>::iterator lit = lLocalMapCuboids.begin(), lend = lLocalMapCuboids.end(); lit != lend; lit++)
    {
        unordered_map<KeyFrame *, size_t> observations = (*lit)->GetObservations();
        for (unordered_map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
        {
            KeyFrame *pKFi = mit->first;

            if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId) // not been added to lLocalKeyFrames, and lFixedCameras!
            {
                pKFi->mnBAFixedForKF = pKF->mnId;
                if (!pKFi->isBad())
                    lFixedCameras.push_back(pKFi);
            }
        }
    }

    std::cout << "Optimizer: LocalBAPointObjects num of map points/ planes/ cuboids/ keyframes: " 
            << pMap->MapPointsInMap() << " " << pMap->MapPlanesInMap() << " " << pMap->MapCuboidsInMap() << " " << pMap->KeyFramesInMap() << std::endl;
    std::cout << "Optimizer: LocalBAPointObjects num of graph points/ planes/ cuboids/ keyframes/ fixkeyframe: " 
            << lLocalMapPoints.size() << " " << lLocalMapPlanes.size() << " " << lLocalMapCuboids.size() << " " << lLocalKeyFrames.size() << " " << lFixedCameras.size() << std::endl;

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType *linearSolver; // BlockSolverX instead of BlockSolver63
    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if (pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    unsigned long maxKFid = 0;
    // Set Local KeyFrame vertices
    for (vector<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
    {
        KeyFrame *pKFi = *lit;
        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(pKFi->mnId == 0); // if first keyframe frame, set pose fixed.
        if (fixCamera)
            vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if (pKFi->mnId > maxKFid)
            maxKFid = pKFi->mnId;
    }

    // Set Fixed KeyFrame vertices
    for (vector<KeyFrame *>::iterator lit = lFixedCameras.begin(), lend = lFixedCameras.end(); lit != lend; lit++)
    {
        KeyFrame *pKFi = *lit;
        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if (pKFi->mnId > maxKFid)
            maxKFid = pKFi->mnId;
    }

    // #ifdef ObjectFixScale
    //     typedef g2o::VertexCuboidFixScale g2o_object_vertex;
    //     typedef g2o::EdgeSE3CuboidFixScaleProj g2o_camera_obj_2d_edge;
    typedef g2o::VertexCuboid g2o_cuboid_vertex;
    typedef g2o::EdgeSE3CuboidProj g2o_camera_obj_2d_edge;

    // Set MapCuboid vertices
    std::vector<int> graph_cuboid_id_list;
    long int maxCuboidid = maxKFid;
    if (1)
    {
        for (vector<MapCuboid *>::iterator lit = lLocalMapCuboids.begin(), lend = lLocalMapCuboids.end(); lit != lend; lit++)
        {
            MapCuboid *pMCuboid = *lit;
            // cv::Mat cube_pose = pMCuboid->GetWorldPos();  // set as cv mat
            g2o::cuboid cube_pose = pMCuboid->cuboid_global_data;
            std::cout << "cuboid vertex init pose: "<< cube_pose.toMinimalVector().transpose() << std::endl;
            g2o_cuboid_vertex *vCuboid = new g2o_cuboid_vertex();

            vCuboid->setEstimate(cube_pose);
            vCuboid->whether_fixrollpitch = true; // only rotate along object z axis.
            vCuboid->whether_fixheight = true;   //may make camera height estimation bad

            int id = pMCuboid->mnId + maxKFid + 1;
            vCuboid->setId(id);
            vCuboid->setFixed(false);
            optimizer.addVertex(vCuboid);
            if (id > maxCuboidid)
                maxCuboidid = id;
            graph_cuboid_id_list.push_back(id);
            std::cout << "Optimizer: add cuboid id " << id << " mnid " << pMCuboid->mnId << " init pose: "<< cube_pose.toMinimalVector().transpose() << std::endl;
        }
    }

    // Set MapPlanes vertices
    const int nExpectedPlaneEdgeSize = (lLocalKeyFrames.size()+lFixedCameras.size())*lLocalMapPlanes.size();
    vector<g2o::EdgePlane*> vpEdgesPlane;
    vpEdgesPlane.reserve(nExpectedPlaneEdgeSize);
    vector<g2o::EdgeParallelPlane*> vpEdgesParPlane;
    vpEdgesParPlane.reserve(nExpectedPlaneEdgeSize);
    vector<g2o::EdgeVerticalPlane*> vpEdgesVerPlane;
    vpEdgesVerPlane.reserve(nExpectedPlaneEdgeSize);
    vector<KeyFrame*> vpEdgeKFPlane;
    vpEdgeKFPlane.reserve(nExpectedPlaneEdgeSize);
    vector<MapPlane*> vpMapPlane;
    vpMapPlane.reserve(nExpectedPlaneEdgeSize);

    double angleInfo = plane_angle_info;//1;//0.5;
    angleInfo = 3282.8/(angleInfo*angleInfo);
    double disInfo = plane_dist_info;//100;//50;
    disInfo = disInfo* disInfo;
    double parInfo = 0.5;//0.1;
    parInfo = 3282.8/(parInfo*parInfo);
    double verInfo = 0.5;//0.1;
    verInfo = 3282.8/(verInfo*verInfo);
    double planeChi = plane_chi;//1000;//100;
    const float deltaPlane = sqrt(planeChi);
    double VPplaneChi = 200;//50;
    const float VPdeltaPlane = sqrt(VPplaneChi);
    int total_edge_num_cam_plane = 0;

    for (vector<MapPlane*>::iterator lit=lLocalMapPlanes.begin(), lend=lLocalMapPlanes.end(); lit!=lend; lit++) 
    {
        // add plane vertex
        MapPlane *pMP = *lit;
        g2o::VertexPlane *vPlane = new g2o::VertexPlane();
        vPlane->setEstimate(Converter::toPlane3D(pMP->GetWorldPos()));
        int id = pMP->mnId + maxCuboidid + 1;
        vPlane->setId(id);
        vPlane->setMarginalized(true);
        optimizer.addVertex(vPlane);
        std::cout << "Optimizer: add plane id " << id << " init pose: "<< pMP->GetWorldPos().t() << std::endl;
    }

    // bool optimize_with_plane_3d = false;
    if(optimize_with_plane_3d)
    {
        for (vector<MapPlane*>::iterator lit=lLocalMapPlanes.begin(), lend=lLocalMapPlanes.end(); lit!=lend; lit++) 
        {
            MapPlane *pMP = *lit;
            int id = pMP->mnId + maxCuboidid + 1;
            // add plane edge: g2o::EdgePlane
            const map<KeyFrame *, int> observations = pMP->GetObservations();
            for (map<KeyFrame *, int>::const_iterator mit=observations.begin(), mend=observations.end(); mit!= mend; mit++) 
            {
                KeyFrame *pKFi = mit->first;
                if (!pKFi->isBad()) {
                    g2o::EdgePlane *e = new g2o::EdgePlane();
                    if (optimizer.vertex(id) == NULL || optimizer.vertex(pKFi->mnId) == NULL)
                        continue;
                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(Converter::toPlane3D(pKFi->mvPlaneCoefficients[mit->second]));
                    Eigen::Matrix3d Info;
                    Info << angleInfo, 0, 0,
                            0, angleInfo, 0,
                            0, 0, disInfo;
                    e->setInformation(Info);
                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(deltaPlane);
                    optimizer.addEdge(e);
                    total_edge_num_cam_plane ++;

                    vpEdgesPlane.push_back(e);
                    vpEdgeKFPlane.push_back(pKFi);
                    vpMapPlane.push_back(pMP);
                    // e->computeError();
                    // double chi = e->chi2();
                    // std::cout<<"Optimizer: edge: "<< total_edge_num_cam_plane-1 << " cam: " << pKFi->mnId 
                    //         << " plane: " << id << " error: " << pKFi->mvPlaneCoefficients[mit->second].t() << " chi: " << chi <<std::endl;
                }
            }

            // add vertical plane g2o::EdgeVerticalPlane
            const map<KeyFrame *, int> verObservations = pMP->GetVerObservations();
            for (map<KeyFrame *, int>::const_iterator mit = verObservations.begin(), mend = verObservations.end();
                mit != mend; mit++) {
                KeyFrame *pKFi = mit->first;
                if (!pKFi->isBad()) {
                    g2o::EdgeVerticalPlane *e = new g2o::EdgeVerticalPlane();
                    if (optimizer.vertex(id) == NULL || optimizer.vertex(pKFi->mnId) == NULL)
                        continue;
                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(Converter::toPlane3D(pKFi->mvPlaneCoefficients[mit->second]));
                    Eigen::Matrix2d Info;
                    Info << verInfo, 0,
                            0, verInfo;
                    e->setInformation(Info);
                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(VPdeltaPlane);
                    optimizer.addEdge(e);
                    vpEdgesVerPlane.push_back(e);
                }
            }

            // add parallel plane g2o::EdgeParallelPlane
            const map<KeyFrame *, int> parObservations = pMP->GetParObservations();
            for (map<KeyFrame *, int>::const_iterator mit = parObservations.begin(), mend = parObservations.end();
                mit != mend; mit++) {
                KeyFrame *pKFi = mit->first;
                if (!pKFi->isBad()) {
                    g2o::EdgeParallelPlane *e = new g2o::EdgeParallelPlane();
                    if (optimizer.vertex(id) == NULL || optimizer.vertex(pKFi->mnId) == NULL)
                        continue;
                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(Converter::toPlane3D(pKFi->mvPlaneCoefficients[mit->second]));
                    Eigen::Matrix2d Info;
                    Info << parInfo, 0,
                            0, parInfo;
                    e->setInformation(Info);
                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(VPdeltaPlane);
                    optimizer.addEdge(e);
                    vpEdgesParPlane.push_back(e);
                }
            }
        }
    }

    // Set MapPoint vertices
    const int nExpectedSize = (lLocalKeyFrames.size() + lFixedCameras.size()) * lLocalMapPoints.size();
    vector<g2o::EdgeSE3ProjectXYZ *> vpEdgesMono; // a camera + map point
    vpEdgesMono.reserve(nExpectedSize);
    vector<KeyFrame *> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);
    vector<MapPoint *> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);
    vector<g2o::EdgeStereoSE3ProjectXYZ *> vpEdgesStereo; // left + right camera + map point
    vpEdgesStereo.reserve(nExpectedSize);
    vector<KeyFrame *> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);
    vector<MapPoint *> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);

    // set up map point and camera-point edges
    int obs_greater_one_points = 0;
    int obs_one_points = 0;
    for (vector<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
    {
        MapPoint *pMP = *lit;

        if (pMP->Observations() == 1) // HACK by me, skip observation one point, which is initialized by my depth! won't affect previous mono.
            continue;

        g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));

        int id = pMP->mnId + maxKFid + maxCuboidid + 2;
        vPoint->setId(id);
        if (fixPoint)
            vPoint->setFixed(true);
        else
            vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const map<KeyFrame *, size_t> observations = pMP->GetObservations();
        if (observations.size() == 1)
            obs_one_points++;
        if (observations.size() > 1)
            obs_greater_one_points++;

        //Set edges
        for (map<KeyFrame *, size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
        {
            KeyFrame *pKFi = mit->first;

            if (!pKFi->isBad())
            {
                const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];

                // Monocular observation
                if (pKFi->mvuRight[mit->second] < 0)
                {
                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    g2o::EdgeSE3ProjectXYZ *e = new g2o::EdgeSE3ProjectXYZ(); // camera point edge.  there is no camera-camera odometry edge.

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId))); //get vertex based on Id.
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;

                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);
                }
                else // Stereo observation
                {
                    Eigen::Matrix<double, 3, 1> obs;
                    const float kp_ur = pKFi->mvuRight[mit->second];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
                    e->setInformation(Info);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;
                    e->bf = pKFi->mbf;

                    optimizer.addEdge(e);
                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKFi);
                    vpMapPointEdgeStereo.push_back(pMP);
                }
            }
        }
    }


    int total_edge_num_cam_obj = 0;
    // set up camera and cuboid edge: 2d measurement
    // float thHuberBbox2d = 80;
    // double ba_weight_bbox = 1.0; // todo: need modify
    std::vector<g2o::EdgeSE3CuboidProj *> vpEdgesCameraCuboidBbox;
    if(optimize_with_cuboid_2d)
    {
        for (vector<MapCuboid *>::iterator lit = lLocalMapCuboids.begin(), lend = lLocalMapCuboids.end(); lit != lend; lit++)
        {
            MapCuboid *pMCuboid = *lit;
            const unordered_map<KeyFrame *, size_t> observations = pMCuboid->GetObservations();
            for (unordered_map<KeyFrame *, size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
            {
                KeyFrame *pKFi = mit->first;
                if (!pKFi->isBad())
                {
                    g2o::EdgeSE3CuboidProj* e = new g2o::EdgeSE3CuboidProj();
                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));                   // camera
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pMCuboid->mnId + maxKFid + 1))); // object
                    const MapCuboid *local_object = pKFi->local_cuboids[mit->second];
                    Eigen::Matrix3d calib;
                    for (size_t ttt = 0; ttt < 3; ttt++)
                        for (size_t zzz = 0; zzz < 3; zzz++)
                            calib(ttt,zzz) = pKFi->mK.at<float>(ttt,zzz);
                    e->Kalib = calib; 
                    e->setMeasurement(local_object->bbox_vec);
                    std::cout << "local_object->bbox_vec " << local_object->bbox_vec.transpose() << std::endl;

                    int object_boundary_margin = 5;
                    cv::Rect bbox_2d = local_object->bbox_2d;
                    // object should be in FOV, otherwise bad for this edge
                    if ((bbox_2d.x > object_boundary_margin) && (bbox_2d.y > object_boundary_margin) && (bbox_2d.x + bbox_2d.width < 640 - object_boundary_margin) &&
                        (bbox_2d.y + bbox_2d.height < 480 - object_boundary_margin))
                    {
                        Eigen::Vector4d inv_sigma;inv_sigma<<1,1,1,1;
                        double meas_quality = local_object->meas_quality; //2.0; //0.75;
                        inv_sigma = inv_sigma*ba_weight_bbox*meas_quality;; // point sigma<1, object error is usually large, no need to set large sigma...
                        Eigen::Matrix4d info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
                        // same info
                        // Eigen::Vector4d inv_sigma;inv_sigma<<1,1,1,1;
                        // inv_sigma = inv_sigma * ba_weight_bbox; // point sigma<1, object error is usually large, no need to set large sigma...
                        // Eigen::Matrix4d camera_object_sigma = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
                        // Eigen::Matrix4d info = camera_object_sigma * local_object->meas_quality * local_object->meas_quality;
                        
                        e->setInformation(info);
                        std::cout << "Optimizer: get error from bbox 2d : \n" << e->get_error_norm() << std::endl;
                        const float thHuberObject = sqrt(thHuberBbox2d); // object reprojection error is usually large
                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberObject);
                        e->setId(total_edge_num_cam_obj);
                        total_edge_num_cam_obj++;
                        optimizer.addEdge(e);
                        vpEdgesCameraCuboidBbox.push_back(e);
                        // std::cout << "add cam cuboid edge " << pKFi->mnId << " " << pMCuboid->object_graph_id + maxKFid + 1 << std::endl;
                    } // loop 2d bbox
                }// loop if !pKFi->isBad()
            }// loop observation
        }// loop cuboid
    }

    // set up camera and cuboid edge: 2d measurement
    // float thHuberConer2d = 10;
    // float ba_weight_corner = 0.1;
    std::vector<g2o::EdgeSE3CuboidCornerProj *> vpEdgesCameraCuboidCorner;
    if(optimize_with_corners_2d)
    {
        for (vector<MapCuboid *>::iterator lit = lLocalMapCuboids.begin(), lend = lLocalMapCuboids.end(); lit != lend; lit++)
        {
            MapCuboid *pMCuboid = *lit;
            const unordered_map<KeyFrame *, size_t> observations = pMCuboid->GetObservations();
            for (unordered_map<KeyFrame *, size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
            {
                KeyFrame *pKFi = mit->first;
                if (!pKFi->isBad())
                {
                    g2o::EdgeSE3CuboidCornerProj* e = new g2o::EdgeSE3CuboidCornerProj();
                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));                   // camera
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pMCuboid->mnId + maxKFid + 1))); // object
                    const MapCuboid *local_object = pKFi->local_cuboids[mit->second];
                    Eigen::Matrix3d calib;
                    for (size_t ttt = 0; ttt < 3; ttt++)
                        for (size_t zzz = 0; zzz < 3; zzz++)
                            calib(ttt,zzz) = pKFi->mK.at<float>(ttt,zzz);
                    e->Kalib = calib; 

                    Eigen::Matrix<double, 16, 1> corners_2d_vec;
                    for (size_t i = 0; i < 8; i++)
                    {
                        corners_2d_vec(i*2) = local_object->box_corners_2d(0,i);
                        corners_2d_vec(i*2+1) = local_object->box_corners_2d(1,i);
                    }
                    e->setMeasurement(corners_2d_vec);
                    std::cout << "local_object->corners_2d_vec " << corners_2d_vec.transpose() << std::endl;

                    int object_boundary_margin = 5;
                    cv::Rect bbox_2d = local_object->bbox_2d;
                    // object should be in FOV, otherwise bad for this edge
                    if ((bbox_2d.x > object_boundary_margin) && (bbox_2d.y > object_boundary_margin) && (bbox_2d.x + bbox_2d.width < 640 - object_boundary_margin) &&
                        (bbox_2d.y + bbox_2d.height < 480 - object_boundary_margin))
                    {
                        Eigen::Matrix<double, 16, 1> inv_sigma;
                        inv_sigma << 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1;
                        double meas_quality = local_object->meas_quality; //2.0; //0.75;
                        inv_sigma = inv_sigma*ba_weight_corner*meas_quality;; // point sigma<1, object error is usually large, no need to set large sigma...
                        Eigen::Matrix<double, 16, 16> info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
  
                        e->setInformation(info);
                        // std::cout << "info" << info << std::endl;
                        std::cout << "Optimizer: get error from corners 2d : \n" << e->get_error_norm() << std::endl;
                        const float thHuberObject = sqrt(thHuberConer2d); // object reprojection error is usually large
                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberObject);
                        e->setId(total_edge_num_cam_obj);
                        total_edge_num_cam_obj++;
                        optimizer.addEdge(e);
                        vpEdgesCameraCuboidCorner.push_back(e);
                        // std::cout << "add cam cuboid edge " << pKFi->mnId << " " << pMCuboid->object_graph_id + maxKFid + 1 << std::endl;
                    } // loop 2d bbox
                }// loop if !pKFi->isBad()
            }// loop observation
        }// loop cuboid
    }

    // set up point-object 3d association
    std::vector<g2o::EdgePointCuboidOnlyObject *> vpEdgesPointCuboid; // edge between cuboid and points
    vpEdgesPointCuboid.reserve(lLocalMapPoints.size()); // only change capacity(), not change size()
    if (optimize_with_pt_obj_3d)
    {
        // step 1: get object point association.
        // can we do this when validate cuboids?? then don't need to do it every time?
        // get ba points from point cuboid association
        int point_object_threshold = 2; // select points that have observe cuboid more than 2
        vector<vector<Vector3d>> all_object_ba_points_xyz(lLocalMapCuboids.size()); // use for filter
        vector<vector<MapPoint*>> all_object_ba_points_pts(lLocalMapCuboids.size()); // use for filter and show
        for (size_t i = 0; i < lLocalMapCuboids.size(); i++)
        {
            MapCuboid *pMC = lLocalMapCuboids[i];
            // int largest_point_obs_num = pMC->largest_point_observations;
            // point_object_threshold = std::max(int(largest_point_obs_num * 0.4), 2); // whether use adaptive threshold or fixed.

            const std::vector<MapPoint *> &UniquePoints = pMC->GetUniqueMapPoints();
            for (size_t j = 0; j < UniquePoints.size(); j++)
                if (UniquePoints[j])
                    if (!UniquePoints[j]->isBad())
                        if (UniquePoints[j]->MapObjObservations[pMC] > point_object_threshold)
                        {
                            all_object_ba_points_pts[i].push_back(UniquePoints[j]);
                            all_object_ba_points_xyz[i].push_back(Converter::toVector3d(UniquePoints[j]->GetWorldPos()));
                        }
        }

        // step 2: filter bad object points for BA
        double coarse_threshold = 4;
        double fine_threshold = 3;
        for (size_t i = 0; i < lLocalMapCuboids.size(); i++)
        {
            MapCuboid *pMObj = lLocalMapCuboids[i];
            pMObj->used_points_in_BA_filtered.clear();
            // compute the mean, eliminate outlier points.
            Eigen::Vector3d mean_point;
            mean_point.setZero();
            for (size_t j = 0; j < all_object_ba_points_xyz[i].size(); j++)
                mean_point += all_object_ba_points_xyz[i][j];
            mean_point /= (double)(all_object_ba_points_xyz[i].size());

            //NOTE  filtering of points!!!  remove outlier points
            Eigen::Vector3d mean_point_2;
            mean_point_2.setZero();
            int valid_point_num = 0;
            for (size_t j = 0; j < all_object_ba_points_xyz[i].size(); j++)
                if ((mean_point - all_object_ba_points_xyz[i][j]).norm() < coarse_threshold)
                {
                    mean_point_2 += all_object_ba_points_xyz[i][j];
                    valid_point_num++;
                }
            mean_point_2 /= (double)valid_point_num;

            Eigen::Vector3d mean_point_final;
            mean_point_final.setZero();
            std::vector<Eigen::Vector3d> good_points; // for car, if points are 4 meters away from center, usually outlier.
            for (size_t j = 0; j < all_object_ba_points_xyz[i].size(); j++)
            {
                if ((mean_point_2 - all_object_ba_points_xyz[i][j]).norm() < fine_threshold)
                {
                    mean_point_final += all_object_ba_points_xyz[i][j];
                    good_points.push_back(all_object_ba_points_xyz[i][j]);
                    pMObj->used_points_in_BA_filtered.push_back(all_object_ba_points_pts[i][j]);
                }
            }
            mean_point_final /= (double)(good_points.size());
            all_object_ba_points_xyz[i].clear();
            all_object_ba_points_xyz[i] = good_points;

            // if ((all_object_ba_points_xyz[i].size() > 5)) // whether want to initialize object position to be center of points
            // {
            //     g2o_cuboid_vertex *vObject = static_cast<g2o_cuboid_vertex *>(optimizer.vertex(pMObj->mnId + maxKFid + 1));
            //     g2o::cuboid tempcube = vObject->estimate();
            //     tempcube.setTranslation(mean_point_final);
            //     vObject->setEstimate(tempcube);
            //     std::cout << "cuboid vertex new pose: "<< tempcube.toMinimalVector().transpose() << std::endl;
            // }
        }

        // step 3: add point objct 3d measument, set use fixed point or to optimize point
        for (size_t i = 0; i < lLocalMapCuboids.size(); i++) // no need to optimize all objects...., use local KF's map objects?
        {
            // an object connected to many fixed points. optimize only object
            MapCuboid *pMC = lLocalMapCuboids[i];
            g2o::EdgePointCuboidOnlyObject *e = new g2o::EdgePointCuboidOnlyObject();
            for (size_t j = 0; j < all_object_ba_points_xyz[i].size(); j++)
                e->object_points.push_back(all_object_ba_points_xyz[i][j]);

            if (e->object_points.size() > 10)
            {
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pMC->mnId + maxKFid + 1)));
                Eigen::Matrix3d info;
                info.setIdentity();
                e->setInformation(info);
                e->max_outside_margin_ratio = 1;
                // e->prior_object_half_size = Eigen::Vector3d(0.25, 0.08, 0.18);
                std::cout << "Optimizer: get error from point object: " << e->computeError_debug().transpose() << std::endl;
                optimizer.addEdge(e);
                vpEdgesPointCuboid.push_back(e);
            }
        }
    }

    // set up plane cuboid asscociation
    int total_edge_num_cuboid_plane = 0;
    // bool optimize_with_cuboid_plane = false; // global
    std::vector<g2o::EdgeCuboidPlane *> vpEdgesCuboidPlane; // edge between cuboid and planes
    vpEdgesPointCuboid.reserve(lLocalMapPlanes.size()); // only change capacity(), not change size()
    if(optimize_with_cuboid_plane)
    {
        double angleInfo_cp = cuboid_plane_angle_info;//0.6;//0.5; // not too large when loop closing, see the same plane 
        angleInfo_cp = 3282.8/(angleInfo_cp*angleInfo_cp);
        double disInfo_cp = cuboid_plane_dist_info;//pDist;//100;//100;
        disInfo_cp = disInfo_cp* disInfo_cp;
        double planeChi_cp = cuboid_plane_chi;;//pChi;//1000;//1000;
        const float deltaPlane_cp = sqrt(planeChi_cp);

        for (vector<MapPlane*>::iterator lit=lLocalMapPlanes.begin(), lend=lLocalMapPlanes.end(); lit!=lend; lit++) 
        {
            MapPlane *pMP = *lit;
            if(pMP->asso_cuboid_id == 999)
                continue;

            // add plane edge: g2o::EdgePlane
            g2o::EdgeCuboidPlane* e = new g2o::EdgeCuboidPlane(); 
            // int cuboid_graph_id = pMC->mnId + maxKFid + 1;
            int plane_graph_id = pMP->mnId + maxCuboidid + 1;
            int cuboid_graph_id = pMP->asso_cuboid_id + maxKFid + 1;
            Eigen::Vector3d meas = pMP->asso_cuboid_meas;

            std::cout<<"Optimizer: edge: "<< total_edge_num_cuboid_plane << " cuboid: " << cuboid_graph_id << " "  << pMP->asso_cuboid_id
                    << " plane: " << plane_graph_id  <<std::endl;
            auto it = std::find(graph_cuboid_id_list.begin(), graph_cuboid_id_list.end(), cuboid_graph_id); // check cuboid in graph
            if (it == graph_cuboid_id_list.end()) //id not exist, skip
                continue;
            if(!optimizer.vertex(cuboid_graph_id) || !optimizer.vertex(plane_graph_id))
                continue;
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(cuboid_graph_id))); // cuboid
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(plane_graph_id))); // plane
            e->setMeasurement(meas);
            // e->setId(total_edge_num_cuboid_plane); // need or not
            total_edge_num_cuboid_plane++; // todo: how to define the edge num
            Eigen::Matrix3d Info;
            Info << angleInfo_cp, 0, 0,
                    0, angleInfo_cp, 0,
                    0, 0, disInfo_cp;
            e->setInformation(Info);
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(deltaPlane_cp);
            // std::cout<<"plane_value before aaaa!" <<std::endl;
            optimizer.addEdge(e);
            // std::cout<<"plane_value before bbbbbbbb!" <<std::endl;
            // // debug
            e->computeError();
            double chi = e->chi2();
            std::cout<<"Optimizer: edge: "<< total_edge_num_cuboid_plane-1 << " cuboid: " << cuboid_graph_id 
                    << " plane: " << plane_graph_id << " error: " << meas.transpose() << " chi: " << chi <<std::endl;
        }

    }


    std::cout << "Optimizer: LocalBAPointObjects graph vertex points/ planes/ cuboids/ frames: " 
            << lLocalMapPoints.size() <<" "<< lLocalMapPlanes.size() <<" "<< lLocalMapCuboids.size() << " " << maxKFid << std::endl;
    std::cout << "Optimizer: LocalBAPointObjects graph edge cam-point/ cam-planes/ cam-cuboid/ cuboid-point/ cuboid-plane: "
            << vpMapPointEdgeMono.size()<<" "<< total_edge_num_cam_plane << " " << total_edge_num_cam_obj
            <<" "<< vpEdgesPointCuboid.size() << " " << vpEdgesCuboidPlane.size() <<std::endl;

    if (pbStopFlag)
        if (*pbStopFlag)
            return;

    optimizer.initializeOptimization();
    optimizer.optimize(5);

    bool bDoMore = true;

    if (pbStopFlag)
        if (*pbStopFlag)
            bDoMore = false;

    if (bDoMore)
    {
        // Check inlier observations
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
        {
            g2o::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
            MapPoint *pMP = vpMapPointEdgeMono[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > 5.991 || !e->isDepthPositive())
            {
                e->setLevel(1); // don't optimize this edge.
            }

            e->setRobustKernel(0);
        }

        for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
        {
            g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
            MapPoint *pMP = vpMapPointEdgeStereo[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > 7.815 || !e->isDepthPositive())
            {
                e->setLevel(1);
            }

            e->setRobustKernel(0);
        }

        for (size_t i = 0, iend = vpEdgesCameraCuboidBbox.size(); i < iend; i++)
        {
            g2o::EdgeSE3CuboidProj *e = vpEdgesCameraCuboidBbox[i];
            if (e->error().norm() > thHuberBbox2d)
            {
                e->setLevel(1);
            }
        }

        for (size_t i = 0, iend = vpEdgesCameraCuboidCorner.size(); i < iend; i++)
        {
            g2o::EdgeSE3CuboidCornerProj *e = vpEdgesCameraCuboidCorner[i];
            if (e->error().norm() > thHuberConer2d)
            {
                e->setLevel(1);
            }
        }

        for (size_t i = 0, iend = vpEdgesPlane.size(); i < iend; i++) 
        {
            g2o::EdgePlane *e = vpEdgesPlane[i];
            if (e->chi2() > planeChi) {
                e->setLevel(1);
            }
            e->setRobustKernel(0);
        }

        for (size_t i = 0, iend = vpEdgesParPlane.size(); i < iend; i++) 
        {
            g2o::EdgeParallelPlane *e = vpEdgesParPlane[i];
            if (e->chi2() > VPplaneChi) {
                e->setLevel(1);
            }
            e->setRobustKernel(0);
        }

        for (size_t i = 0, iend = vpEdgesVerPlane.size(); i < iend; i++) 
        {
            g2o::EdgeVerticalPlane *e = vpEdgesVerPlane[i];
            if (e->chi2() > VPplaneChi) {
                e->setLevel(1);
            }
            e->setRobustKernel(0);
        }

        for (size_t i = 0, iend = vpEdgesPointCuboid.size(); i < iend; i++)
        {
            g2o::EdgePointCuboidOnlyObject *e = vpEdgesPointCuboid[i];
            // to do: not sure how to filter outlier
            // if (e->error().norm() > 80)
            // {
            //     e->setLevel(1);
            // }
        }

        for (size_t i = 0, iend = vpEdgesCuboidPlane.size(); i < iend; i++)
        {
            g2o::EdgeCuboidPlane *e = vpEdgesCuboidPlane[i];
            if (e->error().norm() > planeChi)
            {
                e->setLevel(1);
            }
        }

        // Optimize again without the outliers
        optimizer.initializeOptimization(0);
        optimizer.optimize(10);
    }

    vector<pair<KeyFrame *, MapPoint *>> vToErase;
    vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

    // To Erase Plane
    vector<pair<KeyFrame*, MapPlane*> > vToErasePlane;
    vToErasePlane.reserve(vpEdgesPlane.size());

    // Check inlier observations
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
    {
        g2o::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
        MapPoint *pMP = vpMapPointEdgeMono[i];

        if (pMP->isBad())
            continue;

        if (e->chi2() > 5.991 || !e->isDepthPositive())
        {
            KeyFrame *pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi, pMP));
        }
    }

    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
    {
        g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
        MapPoint *pMP = vpMapPointEdgeStereo[i];

        if (pMP->isBad())
            continue;

        if (e->chi2() > 7.815 || !e->isDepthPositive())
        {
            KeyFrame *pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi, pMP));
        }
    }

    for(size_t i=0, iend=vpEdgesPlane.size(); i<iend;i++)
    {
        g2o::EdgePlane* e = vpEdgesPlane[i];
        if(e->chi2()>planeChi)
        {
            MapPlane* pMP = vpMapPlane[i];
            KeyFrame* pKFi = vpEdgeKFPlane[i];
            vToErasePlane.push_back(make_pair(pKFi, pMP));
        }
    }


    // Get Map Mutex
    // if (parallel_mapping)
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    if (!vToErase.empty())
    {
        for (size_t i = 0; i < vToErase.size(); i++)
        {
            KeyFrame *pKFi = vToErase[i].first;
            MapPoint *pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }

    if(!vToErasePlane.empty()){
        for(size_t i=0;i<vToErasePlane.size();i++)
        {
            KeyFrame* pKFi = vToErasePlane[i].first;
            MapPlane* pMPi = vToErasePlane[i].second;
            pKFi->EraseMapPlaneMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }

    // Recover optimized data
    //Keyframes
    for (vector<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
    {
        KeyFrame *pKFrame = *lit;
        pKFrame->mnBALocalForKF = 0; // added ?
        g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKFrame->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        pKFrame->SetPose(Converter::toCvMat(SE3quat));
    }

    //Points
    for (vector<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
    {
        MapPoint *pMP = *lit;
        pMP->mnBALocalForKF = 0;     // added ?
        // if (pMP->Observations() == 1)    // added ?
        //     continue;
        g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(pMP->mnId + maxKFid + maxCuboidid + 2));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
    }

    // fix camera // doesnot update
    for (vector<KeyFrame *>::iterator lit = lFixedCameras.begin(), lend = lFixedCameras.end(); lit != lend; lit++)
    {
        KeyFrame *pKFrame = *lit;
        pKFrame->mnBAFixedForKF = 0;
        pKFrame->mnBALocalForKF = 0;
    }

    // cuboids
    for (vector<MapCuboid *>::iterator lit = lLocalMapCuboids.begin(), lend = lLocalMapCuboids.end(); lit != lend; lit++)
    {
        MapCuboid *pMCuboid = *lit;
        pMCuboid->mnBALocalForKF = 0;
        pMCuboid->obj_been_optimized = true;
        g2o::VertexCuboid *vCuboid = static_cast<g2o::VertexCuboid *>(optimizer.vertex(pMCuboid->mnId + maxKFid + 1));
        g2o::cuboid pose_after_opti = vCuboid->estimate();
        pMCuboid->SetWorldPos(Converter::toCvMat(pose_after_opti.pose));
        pMCuboid->cuboid_global_opti = pose_after_opti;
        std::cout <<"cuboid id: " << pMCuboid->mnId <<" opti: "<<pose_after_opti.toMinimalVector().transpose() << std::endl;
    }

    //Planes
    for(vector<MapPlane*>::iterator lit=lLocalMapPlanes.begin(), lend=lLocalMapPlanes.end(); lit!=lend; lit++)
    {
        MapPlane* pMP = *lit;
        g2o::VertexPlane* vPlane = static_cast<g2o::VertexPlane*>(optimizer.vertex(pMP->mnId+maxCuboidid+1));
        pMP->SetWorldPos(Converter::toCvMat(vPlane->estimate()));
        std::cout <<"plane id: " << pMP->mnId <<" opti: "<<pMP->GetWorldPos().t() << std::endl;
    }
}


} //namespace ORB_SLAM
