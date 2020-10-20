#ifndef _CPP_NEWCYLINDRICALPROBETRACKER_H_
#define _CPP_NEWCYLINDRICALPROBETRACKER_H_

#include "include_opencv.h"

#define USE_LIBCBDETECTOR


namespace NewCylindricalProbeTrackerNamespace
{
    struct ParameterNativeStruct;
    struct FrameInfoNativeStruct;
    struct TrackedPoseNativeStruct;
    struct PoseAndSpatialCoordinate;
    struct BoundBoxResult;
    struct LocateAtTimestamp;
    struct BoundPoints;
    struct BoundRect;

    class NewCylindricalProbeTracker;

    typedef void (*UpdateResultGrayCB)(cv::Mat* pMat);
    typedef int64_t (*GetFrameInfoStampCB)();
    typedef void (*GetFrameInfoNativeStructCB)(FrameInfoNativeStruct& pFrameInfo);
    typedef void (*TransformPoseToGlobalCoordinateCB)(PoseAndSpatialCoordinate& pose);
    typedef bool (*LocateAtTimestampSpatialCB)(LocateAtTimestamp& lat);
    typedef void (*SetBoundRectCB)(cv::Rect& rect);
};

CVAPI(ExceptionStatus) NewCylindricalProbeTracker_new(
    NewCylindricalProbeTrackerNamespace::ParameterNativeStruct& native
    , NewCylindricalProbeTrackerNamespace::UpdateResultGrayCB updateResultGrayCB
    , NewCylindricalProbeTrackerNamespace::GetFrameInfoStampCB getFrameInfoStampCB
    , NewCylindricalProbeTrackerNamespace::GetFrameInfoNativeStructCB getFrameInfoNativeStructCB
    , NewCylindricalProbeTrackerNamespace::TransformPoseToGlobalCoordinateCB transformPoseToGlobalCoordinateCB
    , NewCylindricalProbeTrackerNamespace::LocateAtTimestampSpatialCB locateAtTimestampSpatialCB
    , NewCylindricalProbeTrackerNamespace::SetBoundRectCB setBoundRectCB
    , NewCylindricalProbeTrackerNamespace::NewCylindricalProbeTracker** returnValue);

CVAPI(ExceptionStatus) NewCylindricalProbeTracker_onNewFrame(NewCylindricalProbeTrackerNamespace::NewCylindricalProbeTracker* self);

CVAPI(ExceptionStatus) NewCylindricalProbeTracker_getTrackedPoseTimestamp(NewCylindricalProbeTrackerNamespace::NewCylindricalProbeTracker* self, int64_t& stamp);

CVAPI(ExceptionStatus) NewCylindricalProbeTracker_getTrackedPose(NewCylindricalProbeTrackerNamespace::NewCylindricalProbeTracker* self
    , NewCylindricalProbeTrackerNamespace::TrackedPoseNativeStruct& pose);

CVAPI(ExceptionStatus) NewCylindricalProbeTracker_resetAdaptiveBlockSize(NewCylindricalProbeTrackerNamespace::NewCylindricalProbeTracker* self);

CVAPI(ExceptionStatus) NewCylindricalProbeTracker_delete(NewCylindricalProbeTrackerNamespace::NewCylindricalProbeTracker* self);

CVAPI(ExceptionStatus) NewCylindricalProbeTracker_setBoundPoints(NewCylindricalProbeTrackerNamespace::NewCylindricalProbeTracker* self,
    NewCylindricalProbeTrackerNamespace::BoundPoints& bound);


CVAPI(ExceptionStatus) NewCylindricalProbeTracker_setEyeGazePoints(NewCylindricalProbeTrackerNamespace::NewCylindricalProbeTracker* self,
    std::vector<float>* pVector);

CVAPI(ExceptionStatus) NewCylindricalProbeTracker_update(NewCylindricalProbeTrackerNamespace::NewCylindricalProbeTracker* self, cv::Mat* mat);

CVAPI(void*) NewCylindricalProbeTracker_InitNearestBoundingBox();

CVAPI(int) NewCylindricalProbeTracker_DeleteNearestBoundingBox(void* pData);

CVAPI(int) NewCylindricalProbeTracker_GetNearestBoudingBox(const unsigned short* pDepth, const unsigned char* pSigma, unsigned int width, unsigned int height
    , unsigned short u16ClampMin, unsigned short u16ClampMax, unsigned short u16Tolerance
    , void* pData, NewCylindricalProbeTrackerNamespace::BoundBoxResult* pBBox1, NewCylindricalProbeTrackerNamespace::BoundBoxResult* pBBox2);




#endif//#ifndef _CPP_NEWCYLINDRICALPROBETRACKER_H_