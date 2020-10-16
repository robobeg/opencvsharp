#ifndef _CPP_SUPERRES_H_
#define _CPP_SUPERRES_H_

#ifndef _WINRT_DLL

// ReSharper disable IdentifierTypo
// ReSharper disable CppInconsistentNaming
// ReSharper disable CppNonInlineFunctionDefinitionInHeaderFile

#include "include_opencv.h"

namespace cv
{
    namespace superres
    {
        class CV_EXPORTS FrameSource;
        class CV_EXPORTS SuperResolution;
        class CV_EXPORTS DenseOpticalFlowExt;
        class CV_EXPORTS FarnebackOpticalFlow;
        class CV_EXPORTS DualTVL1OpticalFlow;
        class CV_EXPORTS BroxOpticalFlow;
        class CV_EXPORTS PyrLKOpticalFlow;
    }
}

#pragma region FrameSource

CVAPI(ExceptionStatus) superres_FrameSource_nextFrame(
    cv::Ptr<cv::superres::FrameSource> *obj, cv::_OutputArray *frame)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SUPERRES
    (*obj)->nextFrame(*frame);
#endif//HAVE_OPENCV_SUPERRES
    END_WRAP
}

CVAPI(ExceptionStatus) superres_FrameSource_reset(
    cv::Ptr<cv::superres::FrameSource> *obj)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SUPERRES
    (*obj)->reset();
#endif//HAVE_OPENCV_SUPERRES
    END_WRAP
}

CVAPI(ExceptionStatus) superres_createFrameSource_Empty(cv::Ptr<cv::superres::FrameSource> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SUPERRES
    *returnValue = clone( cv::superres::createFrameSource_Empty() );
#endif//HAVE_OPENCV_SUPERRES
    END_WRAP
}
CVAPI(ExceptionStatus) superres_createFrameSource_Video(const char *fileName, cv::Ptr<cv::superres::FrameSource> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SUPERRES
    *returnValue = clone( cv::superres::createFrameSource_Video(fileName) );
#endif//HAVE_OPENCV_SUPERRES
    END_WRAP
}
CVAPI(ExceptionStatus) superres_createFrameSource_Video_CUDA(const char *fileName, cv::Ptr<cv::superres::FrameSource> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SUPERRES
    *returnValue = clone( cv::superres::createFrameSource_Video_CUDA(fileName) );
#endif//HAVE_OPENCV_SUPERRES
    END_WRAP
}
CVAPI(ExceptionStatus) superres_createFrameSource_Camera(int deviceId, cv::Ptr<cv::superres::FrameSource> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SUPERRES
    *returnValue = clone( cv::superres::createFrameSource_Camera(deviceId) );
#endif//HAVE_OPENCV_SUPERRES
    END_WRAP
}

CVAPI(ExceptionStatus) superres_Ptr_FrameSource_get(cv::Ptr<cv::superres::FrameSource> *ptr, cv::superres::FrameSource **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SUPERRES
    *returnValue = ptr->get();
#endif//HAVE_OPENCV_SUPERRES
    END_WRAP
}

CVAPI(ExceptionStatus) superres_Ptr_FrameSource_delete(cv::Ptr<cv::superres::FrameSource> *ptr)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SUPERRES
    delete ptr;
#endif//HAVE_OPENCV_SUPERRES
    END_WRAP
}

#pragma endregion

#pragma region SuperResolution

CVAPI(ExceptionStatus) superres_SuperResolution_setInput(
    cv::superres::SuperResolution *obj, cv::Ptr<cv::superres::FrameSource> *frameSource)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SUPERRES
    obj->setInput(*frameSource);
#endif//HAVE_OPENCV_SUPERRES
    END_WRAP
}

CVAPI(ExceptionStatus) superres_SuperResolution_nextFrame(
    cv::superres::SuperResolution *obj, cv::_OutputArray *frame)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SUPERRES
    obj->nextFrame(*frame);
#endif//HAVE_OPENCV_SUPERRES
    END_WRAP
}

CVAPI(ExceptionStatus) superres_SuperResolution_reset(cv::superres::SuperResolution *obj)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SUPERRES
    obj->reset();
#endif//HAVE_OPENCV_SUPERRES
    END_WRAP
}

CVAPI(ExceptionStatus) superres_SuperResolution_collectGarbage(cv::superres::SuperResolution *obj)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SUPERRES
    obj->collectGarbage();
#endif//HAVE_OPENCV_SUPERRES
    END_WRAP
}

CVAPI(ExceptionStatus) superres_createSuperResolution_BTVL1(cv::Ptr<cv::superres::SuperResolution> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SUPERRES
    *returnValue = clone( cv::superres::createSuperResolution_BTVL1() );
#endif//HAVE_OPENCV_SUPERRES
    END_WRAP
}
CVAPI(ExceptionStatus) superres_createSuperResolution_BTVL1_CUDA(cv::Ptr<cv::superres::SuperResolution> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SUPERRES
    *returnValue = clone( cv::superres::createSuperResolution_BTVL1_CUDA() );
#endif//HAVE_OPENCV_SUPERRES
    END_WRAP
}

CVAPI(ExceptionStatus) superres_Ptr_SuperResolution_get(
    cv::Ptr<cv::superres::SuperResolution> *ptr, cv::superres::SuperResolution **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SUPERRES
    *returnValue = ptr->get();
#endif//HAVE_OPENCV_SUPERRES
    END_WRAP
}

CVAPI(ExceptionStatus) superres_Ptr_SuperResolution_delete(cv::Ptr<cv::superres::SuperResolution> *ptr)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SUPERRES
    delete ptr;
#endif//HAVE_OPENCV_SUPERRES
    END_WRAP
}

CVAPI(ExceptionStatus) superres_SuperResolution_getScale(cv::superres::SuperResolution *obj, int *returnValue)              { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
*returnValue = obj->getScale(); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_SuperResolution_setScale(cv::superres::SuperResolution *obj, int val)                       { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
obj->setScale(val); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_SuperResolution_getIterations(cv::superres::SuperResolution *obj, int *returnValue)         { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
*returnValue = obj->getIterations(); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_SuperResolution_setIterations(cv::superres::SuperResolution *obj, int val)                  { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
obj->setIterations(val); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_SuperResolution_getTau(cv::superres::SuperResolution *obj, double *returnValue)             { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
*returnValue = obj->getTau(); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_SuperResolution_setTau(cv::superres::SuperResolution *obj, double val)                      { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
obj->setTau(val); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_SuperResolution_getLambda(cv::superres::SuperResolution *obj, double *returnValue)          { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
*returnValue = obj->getLambda(); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP } 
CVAPI(ExceptionStatus) superres_SuperResolution_setLambda(cv::superres::SuperResolution *obj, double val)                   { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
obj->setLambda(val); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP } 
CVAPI(ExceptionStatus) superres_SuperResolution_getAlpha(cv::superres::SuperResolution *obj, double *returnValue)           { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
*returnValue = obj->getAlpha(); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_SuperResolution_setAlpha(cv::superres::SuperResolution *obj, double val)                    { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
obj->setAlpha(val); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_SuperResolution_getKernelSize(cv::superres::SuperResolution *obj, int *returnValue)         { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
*returnValue = obj->getKernelSize(); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_SuperResolution_setKernelSize(cv::superres::SuperResolution *obj, int val)                  { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
obj->setKernelSize(val); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_SuperResolution_getBlurKernelSize(cv::superres::SuperResolution *obj, int *returnValue)     { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
*returnValue = obj->getBlurKernelSize(); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_SuperResolution_setBlurKernelSize(cv::superres::SuperResolution *obj, int val)              { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
obj->setBlurKernelSize(val); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_SuperResolution_getBlurSigma(cv::superres::SuperResolution *obj, double *returnValue)       { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
*returnValue = obj->getBlurSigma(); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_SuperResolution_setBlurSigma(cv::superres::SuperResolution *obj, double val)                { 

BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
obj->setBlurSigma(val);
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_SuperResolution_getTemporalAreaRadius(cv::superres::SuperResolution *obj, int *returnValue) { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
*returnValue = obj->getTemporalAreaRadius(); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_SuperResolution_setTemporalAreaRadius(cv::superres::SuperResolution *obj, int val)          { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
obj->setTemporalAreaRadius(val); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }

CVAPI(ExceptionStatus) superres_SuperResolution_getOpticalFlow(cv::superres::SuperResolution *obj, cv::Ptr<cv::superres::DenseOpticalFlowExt> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SUPERRES
    *returnValue = new cv::Ptr<cv::superres::DenseOpticalFlowExt>(obj->getOpticalFlow());
#endif//HAVE_OPENCV_SUPERRES
    END_WRAP
}
CVAPI(ExceptionStatus) superres_SuperResolution_setOpticalFlow(cv::superres::SuperResolution *obj, cv::Ptr<cv::superres::DenseOpticalFlowExt> *val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SUPERRES
    obj->setOpticalFlow(*val);
#endif//HAVE_OPENCV_SUPERRES
    END_WRAP
}

#pragma endregion

CVAPI(ExceptionStatus) superres_DenseOpticalFlowExt_calc(cv::superres::DenseOpticalFlowExt *obj,
    cv::_InputArray *frame0, cv::_InputArray *frame1, cv::_OutputArray *flow1, cv::_OutputArray *flow2)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SUPERRES
    obj->calc(*frame0, *frame1, *flow1, entity(flow2));
#endif//HAVE_OPENCV_SUPERRES
    END_WRAP
}

CVAPI(ExceptionStatus) superres_DenseOpticalFlowExt_collectGarbage(cv::superres::DenseOpticalFlowExt *obj)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SUPERRES
    obj->collectGarbage();
#endif//HAVE_OPENCV_SUPERRES
    END_WRAP
}

#pragma region FarnebackOpticalFlow

CVAPI(ExceptionStatus) superres_createOptFlow_Farneback(cv::Ptr<cv::superres::FarnebackOpticalFlow> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SUPERRES
    *returnValue = clone(cv::superres::createOptFlow_Farneback());
#endif//HAVE_OPENCV_SUPERRES
    END_WRAP
}
CVAPI(ExceptionStatus) superres_createOptFlow_Farneback_CUDA(cv::Ptr<cv::superres::FarnebackOpticalFlow> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SUPERRES
    *returnValue = clone(cv::superres::createOptFlow_Farneback_CUDA());
#endif//HAVE_OPENCV_SUPERRES
    END_WRAP
}

CVAPI(ExceptionStatus) superres_Ptr_FarnebackOpticalFlow_get(
    cv::Ptr<cv::superres::FarnebackOpticalFlow> *ptr, cv::superres::FarnebackOpticalFlow **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SUPERRES
    *returnValue = ptr->get();
#endif//HAVE_OPENCV_SUPERRES
    END_WRAP
}

CVAPI(ExceptionStatus) superres_Ptr_FarnebackOpticalFlow_delete(
    cv::Ptr<cv::superres::FarnebackOpticalFlow> *ptr)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SUPERRES
    delete ptr;
#endif//HAVE_OPENCV_SUPERRES
    END_WRAP
}

CVAPI(ExceptionStatus) superres_FarnebackOpticalFlow_getPyrScale(cv::superres::FarnebackOpticalFlow *obj, double *returnValue)  { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
*returnValue = obj->getPyrScale(); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_FarnebackOpticalFlow_setPyrScale(cv::superres::FarnebackOpticalFlow *obj, double val)           { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
obj->setPyrScale(val); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_FarnebackOpticalFlow_getLevelsNumber(cv::superres::FarnebackOpticalFlow *obj, int *returnValue) { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
*returnValue = obj->getLevelsNumber(); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_FarnebackOpticalFlow_setLevelsNumber(cv::superres::FarnebackOpticalFlow *obj, int val)          { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
obj->setLevelsNumber(val); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_FarnebackOpticalFlow_getWindowSize(cv::superres::FarnebackOpticalFlow *obj, int *returnValue)   { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
*returnValue = obj->getWindowSize(); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_FarnebackOpticalFlow_setWindowSize(cv::superres::FarnebackOpticalFlow *obj, int val)            { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
obj->setWindowSize(val); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_FarnebackOpticalFlow_getIterations(cv::superres::FarnebackOpticalFlow *obj, int *returnValue)   { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
*returnValue = obj->getIterations(); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_FarnebackOpticalFlow_setIterations(cv::superres::FarnebackOpticalFlow *obj, int val)            { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
obj->setIterations(val); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_FarnebackOpticalFlow_getPolyN(cv::superres::FarnebackOpticalFlow *obj, int *returnValue)        { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
*returnValue = obj->getPolyN(); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_FarnebackOpticalFlow_setPolyN(cv::superres::FarnebackOpticalFlow *obj, int val)                 { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
obj->setPolyN(val); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_FarnebackOpticalFlow_getPolySigma(cv::superres::FarnebackOpticalFlow *obj, double *returnValue) { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
*returnValue = obj->getPolySigma(); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_FarnebackOpticalFlow_setPolySigma(cv::superres::FarnebackOpticalFlow *obj, double val)          { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
obj->setPolySigma(val); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_FarnebackOpticalFlow_getFlags(cv::superres::FarnebackOpticalFlow *obj, int *returnValue)        { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
*returnValue = obj->getFlags(); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_FarnebackOpticalFlow_setFlags(cv::superres::FarnebackOpticalFlow *obj, int val)                 { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
obj->setFlags(val); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }

#pragma endregion

#pragma region DualTVL1OpticalFlow

CVAPI(ExceptionStatus) superres_createOptFlow_DualTVL1(cv::Ptr<cv::superres::DualTVL1OpticalFlow> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SUPERRES
    *returnValue = clone(cv::superres::createOptFlow_DualTVL1());
#endif//HAVE_OPENCV_SUPERRES
    END_WRAP
}
CVAPI(ExceptionStatus) superres_createOptFlow_DualTVL1_CUDA(cv::Ptr<cv::superres::DualTVL1OpticalFlow> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SUPERRES
    *returnValue = clone(cv::superres::createOptFlow_DualTVL1_CUDA());
#endif//HAVE_OPENCV_SUPERRES
    END_WRAP
}

CVAPI(ExceptionStatus) superres_Ptr_DualTVL1OpticalFlow_get(
    cv::Ptr<cv::superres::DualTVL1OpticalFlow> *ptr, cv::superres::DualTVL1OpticalFlow **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SUPERRES
    *returnValue = ptr->get();
#endif//HAVE_OPENCV_SUPERRES
    END_WRAP
}

CVAPI(ExceptionStatus) superres_Ptr_DualTVL1OpticalFlow_delete(
    cv::Ptr<cv::superres::DualTVL1OpticalFlow> *ptr)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SUPERRES
    delete ptr;
#endif//HAVE_OPENCV_SUPERRES
    END_WRAP
}

CVAPI(ExceptionStatus) superres_DualTVL1OpticalFlow_getTau(cv::superres::DualTVL1OpticalFlow *obj, double *returnValue)         { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
*returnValue = obj->getTau(); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_DualTVL1OpticalFlow_setTau(cv::superres::DualTVL1OpticalFlow *obj, double val)                  { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
obj->setTau(val); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_DualTVL1OpticalFlow_getLambda(cv::superres::DualTVL1OpticalFlow *obj, double *returnValue)      { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
*returnValue = obj->getLambda(); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_DualTVL1OpticalFlow_setLambda(cv::superres::DualTVL1OpticalFlow *obj, double val)               { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
obj->setLambda(val); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_DualTVL1OpticalFlow_getTheta(cv::superres::DualTVL1OpticalFlow *obj, double *returnValue)       { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES
*returnValue = obj->getTheta(); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_DualTVL1OpticalFlow_setTheta(cv::superres::DualTVL1OpticalFlow *obj, double val)                { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES
obj->setTheta(val); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_DualTVL1OpticalFlow_getScalesNumber(cv::superres::DualTVL1OpticalFlow *obj, int *returnValue)   { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
*returnValue = obj->getScalesNumber(); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_DualTVL1OpticalFlow_setScalesNumber(cv::superres::DualTVL1OpticalFlow *obj, int val)            { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
obj->setScalesNumber(val); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_DualTVL1OpticalFlow_getWarpingsNumber(cv::superres::DualTVL1OpticalFlow *obj, int *returnValue) { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
*returnValue = obj->getWarpingsNumber(); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_DualTVL1OpticalFlow_setWarpingsNumber(cv::superres::DualTVL1OpticalFlow *obj, int val)          { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
obj->setWarpingsNumber(val); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_DualTVL1OpticalFlow_getEpsilon(cv::superres::DualTVL1OpticalFlow *obj, double *returnValue)     { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
*returnValue = obj->getEpsilon(); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_DualTVL1OpticalFlow_setEpsilon(cv::superres::DualTVL1OpticalFlow *obj, double val)              { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
obj->setEpsilon(val); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_DualTVL1OpticalFlow_getIterations(cv::superres::DualTVL1OpticalFlow *obj, int *returnValue)     { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
*returnValue = obj->getIterations(); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_DualTVL1OpticalFlow_setIterations(cv::superres::DualTVL1OpticalFlow *obj, int val)              { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
obj->setIterations(val); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_DualTVL1OpticalFlow_getUseInitialFlow(cv::superres::DualTVL1OpticalFlow *obj, int *returnValue) { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
*returnValue = obj->getUseInitialFlow() ? 1 : 0; 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_DualTVL1OpticalFlow_setUseInitialFlow(cv::superres::DualTVL1OpticalFlow *obj, int val)          { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
obj->setUseInitialFlow(val != 0); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }

#pragma endregion

#pragma region BroxOpticalFlow

CVAPI(ExceptionStatus) superres_createOptFlow_Brox_CUDA(cv::Ptr<cv::superres::BroxOpticalFlow> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SUPERRES
    *returnValue = clone(cv::superres::createOptFlow_Brox_CUDA());
#endif//HAVE_OPENCV_SUPERRES
    END_WRAP
}

CVAPI(ExceptionStatus) superres_Ptr_BroxOpticalFlow_get(
    cv::Ptr<cv::superres::BroxOpticalFlow> *ptr, cv::superres::BroxOpticalFlow **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SUPERRES
    *returnValue = ptr->get();
#endif//HAVE_OPENCV_SUPERRES
    END_WRAP
}

CVAPI(ExceptionStatus) superres_Ptr_BroxOpticalFlow_delete(
    cv::Ptr<cv::superres::BroxOpticalFlow> *ptr)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SUPERRES
    delete ptr;
#endif//HAVE_OPENCV_SUPERRES
    END_WRAP
}

CVAPI(ExceptionStatus) superres_BroxOpticalFlow_getAlpha(cv::superres::BroxOpticalFlow *obj, double *returnValue)         { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
*returnValue = obj->getAlpha(); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_BroxOpticalFlow_setAlpha(cv::superres::BroxOpticalFlow *obj, double val)                  { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
obj->setAlpha(val); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_BroxOpticalFlow_getGamma(cv::superres::BroxOpticalFlow *obj, double *returnValue)         { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
*returnValue = obj->getGamma(); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_BroxOpticalFlow_setGamma(cv::superres::BroxOpticalFlow *obj, double val)                  { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
obj->setGamma(val); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_BroxOpticalFlow_getScaleFactor(cv::superres::BroxOpticalFlow *obj, double *returnValue)   { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
*returnValue = obj->getScaleFactor(); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_BroxOpticalFlow_setScaleFactor(cv::superres::BroxOpticalFlow *obj, double val)            { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
obj->setScaleFactor(val); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_BroxOpticalFlow_getInnerIterations(cv::superres::BroxOpticalFlow *obj, int *returnValue)  { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES
*returnValue = obj->getInnerIterations(); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_BroxOpticalFlow_setInnerIterations(cv::superres::BroxOpticalFlow *obj, int val)           { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES
obj->setInnerIterations(val); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_BroxOpticalFlow_getOuterIterations(cv::superres::BroxOpticalFlow *obj, int *returnValue)  { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
*returnValue = obj->getOuterIterations(); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_BroxOpticalFlow_setOuterIterations(cv::superres::BroxOpticalFlow *obj, int val)           { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
obj->setOuterIterations(val); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_BroxOpticalFlow_getSolverIterations(cv::superres::BroxOpticalFlow *obj, int *returnValue) { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
*returnValue = obj->getSolverIterations(); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_BroxOpticalFlow_setSolverIterations(cv::superres::BroxOpticalFlow *obj, int val)          { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
obj->setSolverIterations(val); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }

#pragma endregion

#pragma region PyrLKOpticalFlow

CVAPI(ExceptionStatus) superres_createOptFlow_PyrLK_CUDA(cv::Ptr<cv::superres::PyrLKOpticalFlow> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SUPERRES
    *returnValue = clone(cv::superres::createOptFlow_PyrLK_CUDA());
#endif//HAVE_OPENCV_SUPERRES
    END_WRAP
}

CVAPI(ExceptionStatus) superres_Ptr_PyrLKOpticalFlow_get(
    cv::Ptr<cv::superres::PyrLKOpticalFlow> *ptr, cv::superres::PyrLKOpticalFlow **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SUPERRES
    *returnValue = ptr->get();
#endif//HAVE_OPENCV_SUPERRES
    END_WRAP
}

CVAPI(ExceptionStatus) superres_Ptr_PyrLKOpticalFlow_delete(
    cv::Ptr<cv::superres::PyrLKOpticalFlow> *ptr)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SUPERRES
    delete ptr;
#endif//HAVE_OPENCV_SUPERRES
    END_WRAP
}

CVAPI(ExceptionStatus) superres_PyrLKOpticalFlow_getWindowSize(cv::superres::PyrLKOpticalFlow *obj, int *returnValue) { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES
*returnValue = obj->getWindowSize(); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_PyrLKOpticalFlow_setWindowSize(cv::superres::PyrLKOpticalFlow *obj, int val)          { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES
obj->setWindowSize(val); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_PyrLKOpticalFlow_getMaxLevel(cv::superres::PyrLKOpticalFlow *obj, int *returnValue)   { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
*returnValue = obj->getMaxLevel(); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_PyrLKOpticalFlow_setMaxLevel(cv::superres::PyrLKOpticalFlow *obj, int val)            { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES
obj->setMaxLevel(val); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_PyrLKOpticalFlow_getIterations(cv::superres::PyrLKOpticalFlow *obj, int *returnValue) { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES 
*returnValue = obj->getIterations(); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }
CVAPI(ExceptionStatus) superres_PyrLKOpticalFlow_setIterations(cv::superres::PyrLKOpticalFlow *obj, int val)          { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_SUPERRES
obj->setIterations(val); 
#endif//HAVE_OPENCV_SUPERRES
END_WRAP }

#pragma endregion


#endif // !#ifndef _WINRT_DLL


#endif