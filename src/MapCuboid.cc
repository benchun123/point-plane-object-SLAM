#include "MapCuboid.h"
#include "KeyFrame.h"
#include "Map.h"
// #include <Converter.h>

#include "mutex"

namespace ORB_SLAM2
{
using namespace std;
long int MapCuboid::nNextId = 0;
mutex MapCuboid::mGlobalMutex;

MapCuboid::MapCuboid(Map *pMap, bool update_index) : mbBad(false), mpMap(pMap)
{
	mnId = nNextId++;

	// if (update_index)
	// 	mnId = nNextId++;
	// else
	// 	mnId = -1;
	nObs = 0;
	mnBALocalForKF = 0;
	// mcRefKF = nullptr;
	// mcReplaced = nullptr;
	// mLatestKF = nullptr;
	// largest_point_observations = 0;
	// pointOwnedThreshold = 0;
	already_associated = false;
	become_candidate = false;
	association_refid_in_tracking = -1;
	// associated_landmark = nullptr;
	// object_id_in_localKF = -1;
	// left_right_to_car = -1;
	// isGood = false;
	// truth_tracklet_id = -1;
	// velocityTwist.setZero();
	// velocityPlanar.setZero();
}

void MapCuboid::SetWorldPos(const cv::Mat &Pos)
{
	unique_lock<mutex> lock2(mGlobalMutex);
	unique_lock<mutex> lock(mMutexPos);
    Pos.copyTo(mWorldPos);
}

cv::Mat MapCuboid::GetWorldPos()
{
	unique_lock<mutex> lock(mMutexPos);
	return mWorldPos.clone();
}

void MapCuboid::addObservation(KeyFrame *pKF, size_t idx)
{
	unique_lock<mutex> lock(mMutexFeatures);
	// if (mLatestKF == nullptr)
	// 	mLatestKF = pKF;
	// if (pKF->mnId >= mLatestKF->mnId)
	// 	mLatestKF = pKF;
	if (mObservations.count(pKF))
		return;
	mObservations[pKF] = idx;
	nObs++;
	// observed_frames.push_back(pKF);
}

void MapCuboid::EraseObservation(KeyFrame *pKF)
{
	bool bBad = false;
	{
		unique_lock<mutex> lock(mMutexFeatures);
		if (mObservations.count(pKF))
		{
			mObservations.erase(pKF);
			nObs--;
			if (nObs <= 0)
				bBad = true;

            if(mcRefKF==pKF)  //update reference keyframe if possible
                mcRefKF=mObservations.begin()->first;
			// if (mcRefKF == pKF) //update reference keyframe if possible
			// {
			// 	long unsigned int min_kf_id = mLatestKF->mnId + 1;
			// 	KeyFrame *smallest_kf = nullptr;
			// 	for (unordered_map<KeyFrame *, size_t>::iterator mit = mObservations.begin(), mend = mObservations.end(); mit != mend; mit++)
			// 		if (mit->first->mnId < min_kf_id)
			// 			smallest_kf = mit->first;
			// 	mcRefKF = smallest_kf;
			// }
			// if (mLatestKF == pKF) //update latest KF if possible
			// {
			// 	observed_frames.pop_back();
			// 	mLatestKF = observed_frames.back();
			// }
		}
	}
	if (bBad)
		SetBadFlag();
}

unordered_map<KeyFrame *, size_t> MapCuboid::GetObservations()
{
	unique_lock<mutex> lock(mMutexFeatures);
	return mObservations;
}

int MapCuboid::Observations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}

void MapCuboid::SetBadFlag()
{
	unordered_map<KeyFrame *, size_t> obs;
	{
		unique_lock<mutex> lock1(mMutexFeatures);
		unique_lock<mutex> lock2(mMutexPos);
		mbBad = true;
		obs = mObservations;   // fully copy the contents, similar to vector
		mObservations.clear(); // remove observations from point side
		// observed_frames.clear();
	}
	for (unordered_map<KeyFrame *, size_t>::iterator mit = obs.begin(), mend = obs.end(); mit != mend; mit++)
	{
		KeyFrame *pKF = mit->first;			   // NOTE when use obj_landmark in association and BA, check if valid.
		pKF->EraseMapCuboidMatch(mit->second); // remove observations from frame side
	}
	mpMap->EraseMapCuboid(this);
}

bool MapCuboid::isBad()
{
	unique_lock<mutex> lock(mMutexFeatures);
	unique_lock<mutex> lock2(mMutexPos);
	return mbBad;
}

bool MapCuboid::IsInKeyFrame(KeyFrame *pKF)
{
	unique_lock<mutex> lock(mMutexFeatures);
	return (mObservations.count(pKF));
}

int MapCuboid::GetIndexInKeyFrame(KeyFrame *pKF)
{
	unique_lock<mutex> lock(mMutexFeatures);
	if (mObservations.count(pKF))
		return mObservations[pKF];
	else
		return -1;
}

KeyFrame *MapCuboid::GetReferenceKeyFrame()
{
	return mcRefKF;
}

void MapCuboid::SetReferenceKeyFrame(KeyFrame *refkf)
{
	mcRefKF = refkf;
}

// KeyFrame *MapCuboid::GetLatestKeyFrame()
// {
// 	unique_lock<mutex> lock(mMutexFeatures);
// 	return mLatestKF;
// }

MapCuboid* MapCuboid::GetReplaced()
{
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mcReplaced;
}

void MapCuboid::Replace(MapCuboid* pMC)
{
	if(pMC->mnId==this->mnId)
        return;
    unordered_map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        obs=mObservations;
        mObservations.clear();
        mbBad=true;
        mcReplaced = pMC;
    }
    for(unordered_map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        KeyFrame* pKF = mit->first;
        if(!pMC->IsInKeyFrame(pKF))
        {
            pKF->ReplaceMapCuboidMatch(mit->second, pMC);
			pMC->addObservation(pKF, mit->second);
        }
        else
        {
            pKF->EraseMapCuboidMatch(mit->second);
        }
    }
    mpMap->EraseMapCuboid(this);
}

bool MapCuboid::check_enough_map_points(int own_point_thre)
{
	// std::cout << "mappoints_potential_own:" << mappoints_potential_own.size()<< std::endl;
	if ((int)mappoints_potential_own.size() > own_point_thre)
		become_candidate = true;
	else
		become_candidate = false;
	return become_candidate;
}

vector<MapPoint *> MapCuboid::GetUniqueMapPoints()
{
	unique_lock<mutex> lock(mMutexFeatures);
	return vector<MapPoint *>(mappoints_unique_own.begin(), mappoints_unique_own.end());
}

int MapCuboid::NumUniqueMapPoints()
{
	unique_lock<mutex> lock(mMutexFeatures);
	return mappoints_unique_own.size();
}

void MapCuboid::AddUniqueMapPoint(MapPoint *pMP, int obs_num)
{
	unique_lock<mutex> lock(mMutexFeatures);
	mappoints_unique_own.insert(pMP);
	// if (obs_num > largest_point_observations) // exact way.
	// 	largest_point_observations = obs_num;
}

void MapCuboid::EraseUniqueMapPoint(MapPoint *pMP, int obs_num)
{
	unique_lock<mutex> lock(mMutexFeatures);
	mappoints_unique_own.erase(pMP);
	// if (obs_num == largest_point_observations)
	// 	largest_point_observations -= 1; // HACK only an approximate way. otherwise need to keep sorted the observations (heap). complicated, not worth it.
}

vector<MapPoint *> MapCuboid::GetPotentialMapPoints()
{
	unique_lock<mutex> lock(mMutexFeatures);
	return vector<MapPoint *>(mappoints_potential_own.begin(), mappoints_potential_own.end());
}


void MapCuboid::AddUniqueMapPlanes(MapPlane *pMP)
{
	unique_lock<mutex> lock(mMutexFeatures);
	mapplane_unique_own.insert(pMP);
}

void MapCuboid::EraseUniqueMapPlanes(MapPlane *pMP)
{
	unique_lock<mutex> lock(mMutexFeatures);
	mapplane_unique_own.erase(pMP);
}

vector<MapPlane *> MapCuboid::GetUniqueMapPlanes()
{
	unique_lock<mutex> lock(mMutexFeatures);
	return vector<MapPlane *>(mapplane_unique_own.begin(), mapplane_unique_own.end());
}

void MapCuboid::AddPotentialMapPoint(MapPoint *pMP)
{
	unique_lock<mutex> lock(mMutexFeatures);
	mappoints_potential_own.insert(pMP);
}

void MapCuboid::SetAsLandmark()
{
	for (set<MapPoint *>::iterator mit = mappoints_potential_own.begin(); mit != mappoints_potential_own.end(); mit++)
	{
		(*mit)->AddObjectObservation(this); // add into actual object landmark observation. make sure this->already_associated=true;
	}
}

void MapCuboid::MergeIntoLandmark(MapCuboid *otherLocalObject)
{
	//NOTE other object is just local. not add to actual point-object observation yet. therefore no need to delete object-point observation.
	for (set<MapPoint *>::iterator mit = otherLocalObject->mappoints_potential_own.begin(); mit != otherLocalObject->mappoints_potential_own.end(); mit++)
	{
		(*mit)->AddObjectObservation(this);
	}
}

long int MapCuboid::getIncrementedIndex()
{
	nNextId++;
	return nNextId;
}






// std::vector<KeyFrame *> MapCuboid::GetObserveFrames()
// {
// 	unique_lock<mutex> lock(mMutexFeatures);
// 	vector<KeyFrame *> res;
// 	for (unordered_map<KeyFrame *, size_t>::iterator mit = mObservations.begin(), mend = mObservations.end(); mit != mend; mit++)
// 	{
// 		res.push_back(mit->first);
// 	}
// 	return res;
// }

// std::vector<KeyFrame *> MapCuboid::GetObserveFramesSequential()
// {
// 	unique_lock<mutex> lock(mMutexFeatures);
// 	return observed_frames;
// }

// g2o::SE3Quat MapCuboid::GetWorldPosInv()
// {
// 	unique_lock<mutex> lock(mMutexPos);
// 	return pose_Tcw.pose;
// }

// g2o::cuboid MapCuboid::GetWorldPosBA()
// {
// 	unique_lock<mutex> lock(mMutexPos);
// 	return pose_Twc_afterba;
// }

} // namespace ORB_SLAM2
