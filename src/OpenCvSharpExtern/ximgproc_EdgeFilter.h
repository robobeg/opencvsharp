#ifndef _CPP_XIMGPROC_EDGE_FILTER_H_
#define _CPP_XIMGPROC_EDGE_FILTER_H_

// ReSharper disable IdentifierTypo
// ReSharper disable CppInconsistentNaming
// ReSharper disable CppNonInlineFunctionDefinitionInHeaderFile

#include "include_opencv.h"

namespace cv
{
    namespace ximgproc
    {
        class CV_EXPORTS_W DTFilter;
        class CV_EXPORTS_W GuidedFilter;
        class CV_EXPORTS_W AdaptiveManifoldFilter;
        class CV_EXPORTS_W FastBilateralSolverFilter;
        class CV_EXPORTS_W FastGlobalSmootherFilter;
    }
}

// DTFilter

CVAPI(ExceptionStatus) ximgproc_Ptr_DTFilter_delete(
    cv::Ptr<cv::ximgproc::DTFilter>* obj)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    delete obj;
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}

CVAPI(ExceptionStatus) ximgproc_Ptr_DTFilter_get(
    cv::Ptr<cv::ximgproc::DTFilter>* ptr, cv::ximgproc::DTFilter** returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    *returnValue = ptr->get();
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}

CVAPI(ExceptionStatus) ximgproc_DTFilter_filter(
    cv::ximgproc::DTFilter* obj,
    cv::_InputArray *src, cv::_OutputArray *dst, int dDepth)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    obj->filter(*src, *dst, dDepth);
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}

CVAPI(ExceptionStatus) ximgproc_createDTFilter(
    cv::_InputArray *guide, double sigmaSpatial, double sigmaColor, int mode, int numIters,
    cv::Ptr<cv::ximgproc::DTFilter>** returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    const auto ptr = cv::ximgproc::createDTFilter(*guide, sigmaSpatial, sigmaColor, mode, numIters);
    *returnValue = new cv::Ptr<cv::ximgproc::DTFilter>(ptr);
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}

CVAPI(ExceptionStatus) ximgproc_dtFilter(
    cv::_InputArray *guide, cv::_InputArray *src, cv::_OutputArray *dst, double sigmaSpatial, double sigmaColor, int mode, int numIters)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    cv::ximgproc::dtFilter(*guide, *src, *dst, sigmaSpatial, sigmaColor, mode, numIters);
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}

//////////////////////////////////////////////////////////////////////////
// GuidedFilter

CVAPI(ExceptionStatus) ximgproc_Ptr_GuidedFilter_delete(
    cv::Ptr<cv::ximgproc::GuidedFilter>* obj)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    delete obj;
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}

CVAPI(ExceptionStatus) ximgproc_Ptr_GuidedFilter_get(
    cv::Ptr<cv::ximgproc::GuidedFilter>* ptr, cv::ximgproc::GuidedFilter** returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    *returnValue = ptr->get();
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}

CVAPI(ExceptionStatus) ximgproc_GuidedFilter_filter(
    cv::ximgproc::GuidedFilter* obj,
    cv::_InputArray* src, cv::_OutputArray* dst, int dDepth)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    obj->filter(*src, *dst, dDepth);
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}

CVAPI(ExceptionStatus) ximgproc_createGuidedFilter(
    cv::_InputArray* guide, int radius, double eps, 
    cv::Ptr<cv::ximgproc::GuidedFilter>** returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    const auto ptr = cv::ximgproc::createGuidedFilter(*guide, radius, eps);
    *returnValue = new cv::Ptr<cv::ximgproc::GuidedFilter>(ptr);
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}

CVAPI(ExceptionStatus) ximgproc_guidedFilter(
    cv::_InputArray *guide, cv::_InputArray *src, cv::_OutputArray *dst, int radius, double eps, int dDepth)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    cv::ximgproc::guidedFilter(*guide, *src, *dst, radius, eps, dDepth);
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}

//////////////////////////////////////////////////////////////////////////
// AdaptiveManifoldFilter

CVAPI(ExceptionStatus) ximgproc_Ptr_AdaptiveManifoldFilter_delete(
    cv::Ptr<cv::ximgproc::AdaptiveManifoldFilter>* obj)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    delete obj;
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}

CVAPI(ExceptionStatus) ximgproc_Ptr_AdaptiveManifoldFilter_get(
    cv::Ptr<cv::ximgproc::AdaptiveManifoldFilter>* ptr, cv::ximgproc::AdaptiveManifoldFilter** returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    *returnValue = ptr->get();
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}

CVAPI(ExceptionStatus) ximgproc_AdaptiveManifoldFilter_filter(
    cv::ximgproc::AdaptiveManifoldFilter* obj,
    cv::_InputArray* src, cv::_OutputArray* dst, cv::_InputArray *joint)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    obj->filter(*src, *dst, entity(joint));
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}

CVAPI(ExceptionStatus) ximgproc_AdaptiveManifoldFilter_collectGarbage(
    cv::ximgproc::AdaptiveManifoldFilter* obj)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    obj->collectGarbage();
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}

CVAPI(ExceptionStatus) ximgproc_AdaptiveManifoldFilter_getSigmaS(cv::ximgproc::AdaptiveManifoldFilter* obj, double *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    *returnValue = obj->getSigmaS();
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}
CVAPI(ExceptionStatus) ximgproc_AdaptiveManifoldFilter_setSigmaS(cv::ximgproc::AdaptiveManifoldFilter* obj, double val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    obj->setSigmaS(val);
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP    
}
CVAPI(ExceptionStatus) ximgproc_AdaptiveManifoldFilter_getSigmaR(cv::ximgproc::AdaptiveManifoldFilter* obj, double* returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    *returnValue = obj->getSigmaR();
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}
CVAPI(ExceptionStatus) ximgproc_AdaptiveManifoldFilter_setSigmaR(cv::ximgproc::AdaptiveManifoldFilter* obj, double val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    obj->setSigmaR(val);
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP    
}
CVAPI(ExceptionStatus) ximgproc_AdaptiveManifoldFilter_getTreeHeight(cv::ximgproc::AdaptiveManifoldFilter* obj, int* returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    *returnValue = obj->getTreeHeight();
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}
CVAPI(ExceptionStatus) ximgproc_AdaptiveManifoldFilter_setTreeHeight(cv::ximgproc::AdaptiveManifoldFilter* obj, int val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    obj->setTreeHeight(val);
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}
CVAPI(ExceptionStatus) ximgproc_AdaptiveManifoldFilter_getPCAIterations(cv::ximgproc::AdaptiveManifoldFilter* obj, int* returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    *returnValue = obj->getPCAIterations();
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}
CVAPI(ExceptionStatus) ximgproc_AdaptiveManifoldFilter_setPCAIterations(cv::ximgproc::AdaptiveManifoldFilter* obj, int val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    obj->setPCAIterations(val);
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}
CVAPI(ExceptionStatus) ximgproc_AdaptiveManifoldFilter_getAdjustOutliers(cv::ximgproc::AdaptiveManifoldFilter* obj, int* returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    *returnValue = obj->getAdjustOutliers();
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}
CVAPI(ExceptionStatus) ximgproc_AdaptiveManifoldFilter_setAdjustOutliers(cv::ximgproc::AdaptiveManifoldFilter* obj, int val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    obj->setAdjustOutliers(val);
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}
CVAPI(ExceptionStatus) ximgproc_AdaptiveManifoldFilter_getUseRNG(cv::ximgproc::AdaptiveManifoldFilter* obj, int* returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    *returnValue = obj->getUseRNG() ? 1 : 0;
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}
CVAPI(ExceptionStatus) ximgproc_AdaptiveManifoldFilter_setUseRNG(cv::ximgproc::AdaptiveManifoldFilter* obj, int val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    obj->setUseRNG(val != 0);
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}

CVAPI(ExceptionStatus) ximgproc_createAMFilter(
    double sigma_s, double sigma_r, int adjust_outliers,
    cv::Ptr<cv::ximgproc::AdaptiveManifoldFilter>** returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    const auto ptr = cv::ximgproc::createAMFilter(sigma_s, sigma_r, adjust_outliers != 0);
    *returnValue = new cv::Ptr<cv::ximgproc::AdaptiveManifoldFilter>(ptr);
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}

CVAPI(ExceptionStatus) ximgproc_amFilter(
    cv::_InputArray *joint, cv::_InputArray *src, cv::_OutputArray *dst, double sigma_s, double sigma_r, int adjust_outliers)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    cv::ximgproc::amFilter(*joint, *src, *dst, sigma_s, sigma_r, adjust_outliers != 0);
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}


//////////////////////////////////////////////////////////////////////////

CVAPI(ExceptionStatus) ximgproc_jointBilateralFilter(cv::_InputArray *joint, cv::_InputArray *src, cv::_OutputArray *dst, int d, double sigmaColor, double sigmaSpace, int borderType)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    cv::ximgproc::jointBilateralFilter(*joint, *src, *dst, d, sigmaColor, sigmaSpace, borderType);
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}

CVAPI(ExceptionStatus) ximgproc_bilateralTextureFilter(cv::_InputArray *src, cv::_OutputArray *dst, int fr, int numIter, double sigmaAlpha, double sigmaAvg)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    cv::ximgproc::bilateralTextureFilter(*src, *dst, fr, numIter, sigmaAlpha, sigmaAvg);
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}

CVAPI(ExceptionStatus) ximgproc_rollingGuidanceFilter(cv::_InputArray *src, cv::_OutputArray *dst, int d, double sigmaColor, double sigmaSpace, int numOfIter, int borderType)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    cv::ximgproc::rollingGuidanceFilter(*src, *dst, d, sigmaColor, sigmaSpace, numOfIter, borderType);
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}


//////////////////////////////////////////////////////////////////////////
// FastBilateralSolverFilter 

CVAPI(ExceptionStatus) ximgproc_Ptr_FastBilateralSolverFilter_delete(
    cv::Ptr<cv::ximgproc::FastBilateralSolverFilter>* obj)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    delete obj;
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}

CVAPI(ExceptionStatus) ximgproc_Ptr_FastBilateralSolverFilter_get(
    cv::Ptr<cv::ximgproc::FastBilateralSolverFilter>* ptr, cv::ximgproc::FastBilateralSolverFilter** returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    * returnValue = ptr->get();
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}

CVAPI(ExceptionStatus) ximgproc_FastBilateralSolverFilter_filter(
    cv::ximgproc::FastBilateralSolverFilter* obj,
    cv::_InputArray* src, cv::_InputArray *confidence, cv::_OutputArray* dst)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    obj->filter(*src, *confidence , *dst);
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}

CVAPI(ExceptionStatus) ximgproc_createFastBilateralSolverFilter(
    cv::_InputArray *guide, double sigma_spatial, double sigma_luma, double sigma_chroma, double lambda, int num_iter, double max_tol,
    cv::Ptr<cv::ximgproc::FastBilateralSolverFilter>** returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    const auto ptr = cv::ximgproc::createFastBilateralSolverFilter(*guide, sigma_spatial, sigma_luma, sigma_chroma, lambda, num_iter, max_tol);
    *returnValue = new cv::Ptr<cv::ximgproc::FastBilateralSolverFilter>(ptr);
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}

CVAPI(ExceptionStatus) ximgproc_fastBilateralSolverFilter(
    cv::_InputArray *guide, cv::_InputArray *src, cv::_InputArray *confidence, cv::_OutputArray *dst, 
    double sigma_spatial, double sigma_luma, double sigma_chroma, double lambda, int num_iter, double max_tol)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    cv::ximgproc::fastBilateralSolverFilter(*guide, *src, *confidence, *dst,
        sigma_spatial, sigma_luma, sigma_chroma, lambda, num_iter, max_tol);
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}


//////////////////////////////////////////////////////////////////////////
// FastGlobalSmootherFilter 

CVAPI(ExceptionStatus) ximgproc_Ptr_FastGlobalSmootherFilter_delete(
    cv::Ptr<cv::ximgproc::FastGlobalSmootherFilter>* obj)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    delete obj;
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}

CVAPI(ExceptionStatus) ximgproc_Ptr_FastGlobalSmootherFilter_get(
    cv::Ptr<cv::ximgproc::FastGlobalSmootherFilter>* ptr, cv::ximgproc::FastGlobalSmootherFilter** returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    * returnValue = ptr->get();
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}

CVAPI(ExceptionStatus) ximgproc_FastGlobalSmootherFilter_filter(
    cv::ximgproc::FastGlobalSmootherFilter* obj,
    cv::_InputArray* src, cv::_OutputArray* dst)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    obj->filter(*src, *dst);
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}

CVAPI(ExceptionStatus) ximgproc_createFastGlobalSmootherFilter(
    cv::_InputArray *guide, double lambda, double sigma_color, double lambda_attenuation, int num_iter,
    cv::Ptr<cv::ximgproc::FastGlobalSmootherFilter>** returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    const auto ptr = cv::ximgproc::createFastGlobalSmootherFilter(*guide, lambda, sigma_color, lambda_attenuation, num_iter);
    *returnValue = new cv::Ptr<cv::ximgproc::FastGlobalSmootherFilter>(ptr);
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}

CVAPI(ExceptionStatus) ximgproc_fastGlobalSmootherFilter(
    cv::_InputArray *guide, cv::_InputArray *src, cv::_OutputArray *dst, double lambda, double sigma_color, double lambda_attenuation, int num_iter)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    cv::ximgproc::fastGlobalSmootherFilter(*guide, *src, *dst,
        lambda, sigma_color, lambda_attenuation, num_iter);
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}

CVAPI(ExceptionStatus) ximgproc_l0Smooth(cv::_InputArray *src, cv::_OutputArray *dst, double lambda, double kappa)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    cv::ximgproc::l0Smooth(*src, *dst, lambda, kappa);
    #endif//HAVE_OPENCV_XIMGPROC
    END_WRAP    
}

#endif