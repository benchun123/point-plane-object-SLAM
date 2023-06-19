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

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include <set>

#include <mutex>

#include "MapCuboid.h"
#include "MapPlane.h"

namespace ORB_SLAM2
{

class MapPoint;
class KeyFrame;
class MapCuboid;
class MapPlane;

class Map
{
public:
    Map();

    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    void EraseMapPoint(MapPoint* pMP);
    void EraseKeyFrame(KeyFrame* pKF);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
    void InformNewBigChange();
    int GetLastBigChangeIdx();

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();

    long unsigned int MapPointsInMap();
    long unsigned  KeyFramesInMap();

    long unsigned int GetMaxKFid();

    void clear();

    vector<KeyFrame*> mvpKeyFrameOrigins;

    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;

public:
    std::set<MapCuboid*> mspMapCuboids;
    void AddMapCuboid(MapCuboid* pMC);
    void EraseMapCuboid(MapCuboid* pMC);
    std::vector<MapCuboid *> GetAllMapCuboids();
    long unsigned int MapCuboidsInMap(); // get number of objects.

    // typedef pcl::PointXYZRGB PointT;
    // typedef pcl::PointCloud <PointT> PointCloud;

    std::set<MapPlane*> mspMapPlanes;
    void AddMapPlane(MapPlane* pMP);
    void EraseMapPlane(MapPlane *pMP);
    std::vector<MapPlane*> GetAllMapPlanes();
    long unsigned int MapPlanesInMap();
    // void AssociatePlanesByBoundary(ORB_SLAM2::Frame &pF);
    // double PointDistanceFromPlane(const cv::Mat &plane, PointCloud::Ptr boundry, bool out);


protected:
    std::set<MapPoint*> mspMapPoints;
    std::set<KeyFrame*> mspKeyFrames;

    std::vector<MapPoint*> mvpReferenceMapPoints;

    long unsigned int mnMaxKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;

    std::mutex mMutexMap;
};

} //namespace ORB_SLAM

#endif // MAP_H
