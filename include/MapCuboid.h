#ifndef MAPCUBOID_H
#define MAPCUBOID_H

// #pragma once

#include <vector>

// #include <Eigen/Dense>
// #include <Eigen/Geometry>
#include <mutex>
#include <opencv2/core.hpp>

#include "KeyFrame.h"
#include "Map.h"
// #include "MapPoint.h"
// #include <Converter.h>
#include <unordered_map>
#include "g2o_cuboid.h"

namespace ORB_SLAM2
{
class KeyFrame;
class Map;
class MapPlane;

class MapPoint; // todo: associate cuboid and map points
// struct cmpKeyframe
// { //sort frame based on ID
//     bool operator()(const KeyFrame *a, const KeyFrame *b) const
//     {
//         return a->mnId < b->mnId;
//     }
// };

class MapCuboid
{
public:
    // whether update global unique map object ID. only used in keyframe creation.
    MapCuboid(Map *pMap, bool update_index = false);

    void SetWorldPos(const cv::Mat &Pos);  // set cuboid pose in world coordinate
    cv::Mat GetWorldPos();                 // get cuboid pose in world/init frame.
    // g2o::SE3Quat GetWorldPosInv(); // get cuboid pose in world/init frame.
    // g2o::cuboid GetWorldPosBA();   //get latest pose after BA

    void addObservation(KeyFrame *pKF, size_t idx); // object observed by frames
    void EraseObservation(KeyFrame *pKF);           // called in Keyframe when set bad, and in optimizer when removing outliers
    // std::map<KeyFrame *, size_t> GetObservations();
    std::unordered_map<KeyFrame *, size_t> GetObservations();
    int Observations();

    void SetBadFlag();
    bool isBad(); // whether definitly bad.

    std::vector<KeyFrame *> GetObserveFrames();
    std::vector<KeyFrame *> GetObserveFramesSequential();

    static long int getIncrementedIndex();

    bool IsInKeyFrame(KeyFrame *pKF);      // whether observed by this kF
    int GetIndexInKeyFrame(KeyFrame *pKF); // get the local cuboids ID in pkf
    KeyFrame *GetReferenceKeyFrame();
    void SetReferenceKeyFrame(KeyFrame *refkf);
    // KeyFrame *GetLatestKeyFrame();

    void Replace(MapCuboid* pMC);
    MapCuboid* GetReplaced();
    bool check_enough_map_points(int own_point_thre);

    std::vector<MapPoint *> GetUniqueMapPoints();
    int NumUniqueMapPoints();
    void AddUniqueMapPoint(MapPoint *pMP, int obs_num); // called by pointAddobjectObservations
    void EraseUniqueMapPoint(MapPoint *pMP, int obs_num);
    std::vector<MapPoint *> GetPotentialMapPoints();
    void AddPotentialMapPoint(MapPoint *pMP);


    void AddUniqueMapPlanes(MapPlane *pMP); // called by pointAddobjectObservations
    void EraseUniqueMapPlanes(MapPlane *pMP);
    std::vector<MapPlane *> GetUniqueMapPlanes();


    // // int largest_point_observations; // largest point observations. better use a heap....
    // // int pointOwnedThreshold;        // some threshold to whether use points in optimization.  set in optimizer.
    void MergeIntoLandmark(MapCuboid *otherLocalObject); // merge the points here. this should already be a landmark. observation should be added elsewhere
    void SetAsLandmark();

// public:
    long int mnId;           // unique id for this cuboid
    static long int nNextId; // static member, automatically add 1 when new (true)
    static std::mutex mGlobalMutex;
    bool isGood;  // whether definitely good.


    // //----------for local MapCuboid--------     no mutex needed, for local cuboid storage, not landmark
    int object_id;                      // object id in reference keyframe's local objects.
    int object_graph_id;                      // object id optimizer graph
    std::string object_classname;        // object classname in reference keyframe's local objects.
    g2o::cuboid cuboid_global_data;           //global data from offline
    g2o::cuboid cuboid_local_meas;           //local measurement in camera frame
    double meas_quality;             // [0,1] the higher, the better
    Eigen::Vector4d bbox_vec;        // center, width, height
    cv::Rect bbox_2d;                // (integer) yolo detected 2D bbox_2d x y w h
    Eigen::Matrix2Xd box_corners_2d; // 2*8 on image  usually for local cuboids on reference frame.

    // std::vector<KeyFrame *> observed_frames; // sequential push back  check bad before use

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    bool obj_been_optimized = false;
    g2o::cuboid cuboid_global_opti;           //optimized global data
    // int point_object_BA_counter = -1;

    long unsigned int association_refid_in_tracking;
    // // std::vector<MapPoint *> used_points_in_BA;          // points with enough observation will be used in BA.
    std::vector<MapPoint *> used_points_in_BA_filtered; // actually BA used points, after removing some far away points

    bool become_candidate;
    bool already_associated;
    // bool is_dynamic = false;
    // Eigen::Vector2d velocityPlanar; //actually used, for kitti cars
    // g2o::cuboid pose_Twc_latestKF;
    // // std::map<KeyFrame *, Eigen::Vector2d, cmpKeyframe> velocityhistory; // for offline analysis
    // g2o::cuboid pose_Twc_afterba;                                       // latest pose after BA. might have some delay compared to pose_Twc_latestKF
    // // std::map<KeyFrame *, std::pair<g2o::cuboid, bool>, cmpKeyframe> allDynamicPoses; // poses/velocity in each keyframe due to movement.  poses/whether_BA
    // // std::unordered_map<KeyFrame *, int> bundle_vertex_ids;
    // // int truth_tracklet_id;

    // // Vector6d velocityTwist;                                    //general 6dof twist. for cars can assume no roll pitch   pose_Twc*exp(twist)=newpose
    // // g2o::SE3Quat getMovePose(KeyFrame *kf, double deltaT = 0); // deltaT relative to kf. works for short period where velocity doesn't change much

    // //----------for local MapCuboid--------     no mutex needed, for local cuboid storage, not landmark
    // int object_id_in_localKF;        // object id in reference keyframe's local objects.
    // Eigen::Matrix2Xi box_corners_2d; // 2*8 on image  usually for local cuboids on reference frame.
    // Eigen::MatrixXi edge_markers;    // in order to plot 2d cuboids with 8 corners.
    // cv::Rect bbox_2d;                // (integer) yolo detected 2D bbox_2d x y w h
    // Eigen::Vector4d bbox_vec;        // center, width, height
    // cv::Rect bbox_2d_tight;          // tighted 2d object, used to find points association.
    // double meas_quality;             // [0,1] the higher, the better
    // g2o::cuboid cuboid_local_meas;           //local measurement in camera frame
    // MapCuboid *associated_landmark; // might be itself
    // int left_right_to_car;          // on the left or right of a car. left=1 right=2 undecided=0   inititial=-1

    // g2o::cuboid pose_noopti;

    // int record_txtrow_id = -1;

protected:
    // g2o::cuboid pose_Twc; // cuboid pose to the init/world. initialized as the position from first observe frame
    // g2o::cuboid pose_Tcw; //inverse, not frequently used

    cv::Mat mWorldPos; // Position in absolute coordinates

    // // Keyframes observing the object and associated localcuboid index in keyframe
    // std::map<KeyFrame *, size_t> mObservations;
    std::unordered_map<KeyFrame *, size_t> mObservations;
    KeyFrame *mcRefKF;   // Reference KeyFrame  first frame see this.
    // KeyFrame *mLatestKF; // latest frame see this.
    int nObs;            // num of frame observations

    // Bad flag (we do not currently erase MapPoint from memory)
    bool mbBad;
    Map *mpMap;
    MapCuboid* mcReplaced;

    std::mutex mMutexPos;
    std::mutex mMutexFeatures;
    std::mutex mMutexParam;

    std::set<MapPoint *> mappoints_unique_own;    // uniquedly owned by this object.
    std::set<MapPoint *> mappoints_potential_own; // potentially owned
    std::set<MapPlane *> mapplane_unique_own;    // uniquedly owned by this object.

};

} // namespace ORB_SLAM2
#endif // MAPCUBOID_H