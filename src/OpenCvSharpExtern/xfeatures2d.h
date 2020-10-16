#ifndef _CPP_XFEATURES2D_H_
#define _CPP_XFEATURES2D_H_

// ReSharper disable CppInconsistentNaming
// ReSharper disable CppNonInlineFunctionDefinitionInHeaderFile

#include "include_opencv.h"

namespace cv
{
    namespace xfeatures2d
    {
        class CV_EXPORTS_W BriefDescriptorExtractor;
        class CV_EXPORTS_W FREAK;
        class CV_EXPORTS_W StarDetector;
        class CV_EXPORTS_W LUCID;
        class CV_EXPORTS_W LATCH;
        class CV_EXPORTS_W SIFT;
        class CV_EXPORTS_W SURF;
    }
}

#pragma region BriefDescriptorExtractor

CVAPI(ExceptionStatus) xfeatures2d_BriefDescriptorExtractor_create(
    int bytes, cv::Ptr<cv::xfeatures2d::BriefDescriptorExtractor> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XFEATURES2D
    const auto ptr = cv::xfeatures2d::BriefDescriptorExtractor::create(bytes);
    *returnValue = clone(ptr);
#endif//HAVE_OPENCV_XFEATURES2D
    END_WRAP
}

CVAPI(ExceptionStatus) xfeatures2d_Ptr_BriefDescriptorExtractor_delete(
    cv::Ptr<cv::xfeatures2d::BriefDescriptorExtractor> *obj)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XFEATURES2D
    delete obj;
#endif//HAVE_OPENCV_XFEATURES2D
    END_WRAP
}

CVAPI(ExceptionStatus) xfeatures2d_BriefDescriptorExtractor_read(
    cv::Ptr<cv::xfeatures2d::BriefDescriptorExtractor> *obj, cv::FileNode *fn)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XFEATURES2D
    obj->get()->read(*fn);
#endif//HAVE_OPENCV_XFEATURES2D
    END_WRAP
}

CVAPI(ExceptionStatus) xfeatures2d_BriefDescriptorExtractor_write(
    cv::Ptr<cv::xfeatures2d::BriefDescriptorExtractor> *obj, cv::FileStorage *fs)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XFEATURES2D
    obj->get()->write(*fs);
#endif//HAVE_OPENCV_XFEATURES2D
    END_WRAP
}

CVAPI(ExceptionStatus) xfeatures2d_BriefDescriptorExtractor_descriptorSize(
    cv::Ptr<cv::xfeatures2d::BriefDescriptorExtractor> *obj, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XFEATURES2D
    *returnValue = obj->get()->descriptorSize();
#endif//HAVE_OPENCV_XFEATURES2D
    END_WRAP
}

CVAPI(ExceptionStatus) xfeatures2d_BriefDescriptorExtractor_descriptorType(
    cv::Ptr<cv::xfeatures2d::BriefDescriptorExtractor> *obj, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XFEATURES2D
    *returnValue = obj->get()->descriptorType();
#endif//HAVE_OPENCV_XFEATURES2D
    END_WRAP
}

CVAPI(ExceptionStatus) xfeatures2d_Ptr_BriefDescriptorExtractor_get(
    cv::Ptr<cv::xfeatures2d::BriefDescriptorExtractor>* ptr, cv::xfeatures2d::BriefDescriptorExtractor **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XFEATURES2D
    *returnValue = ptr->get();
#endif//HAVE_OPENCV_XFEATURES2D
    END_WRAP
}

#pragma endregion

#pragma region FREAK

CVAPI(ExceptionStatus) xfeatures2d_FREAK_create(
    int orientationNormalized, int scaleNormalized, float patternScale, int nOctaves,
    int *selectedPairs, int selectedPairsLength, 
    cv::Ptr<cv::xfeatures2d::FREAK> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XFEATURES2D
    std::vector<int> selectedPairsVec;
    if (selectedPairs != nullptr)
        selectedPairsVec = std::vector<int>(selectedPairs, selectedPairs + selectedPairsLength);
    const auto ptr = cv::xfeatures2d::FREAK::create(
        orientationNormalized != 0, scaleNormalized != 0, patternScale, nOctaves, selectedPairsVec);
    *returnValue = clone(ptr);
#endif//HAVE_OPENCV_XFEATURES2D
    END_WRAP
}
CVAPI(ExceptionStatus) xfeatures2d_Ptr_FREAK_delete(cv::Ptr<cv::xfeatures2d::FREAK> *ptr)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XFEATURES2D
    delete ptr;
#endif//HAVE_OPENCV_XFEATURES2D
    END_WRAP
}

CVAPI(ExceptionStatus) xfeatures2d_Ptr_FREAK_get(cv::Ptr<cv::xfeatures2d::FREAK> *ptr, cv::xfeatures2d::FREAK **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XFEATURES2D
    *returnValue = ptr->get();
#endif//HAVE_OPENCV_XFEATURES2D
    END_WRAP
}

#pragma endregion

#pragma region StarDetector

CVAPI(ExceptionStatus) xfeatures2d_StarDetector_create(
    int maxSize, int responseThreshold,
    int lineThresholdProjected, int lineThresholdBinarized, int suppressNonmaxSize, 
    cv::Ptr<cv::xfeatures2d::StarDetector> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XFEATURES2D
    const auto ptr = cv::xfeatures2d::StarDetector::create(
        maxSize, responseThreshold, lineThresholdProjected, lineThresholdBinarized, suppressNonmaxSize);
    *returnValue = clone(ptr);
#endif//HAVE_OPENCV_XFEATURES2D
    END_WRAP
}

CVAPI(ExceptionStatus) xfeatures2d_Ptr_StarDetector_delete(cv::Ptr<cv::xfeatures2d::StarDetector> *ptr)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XFEATURES2D
    delete ptr;
#endif//HAVE_OPENCV_XFEATURES2D
    END_WRAP
}

CVAPI(ExceptionStatus) xfeatures2d_Ptr_StarDetector_get(cv::Ptr<cv::xfeatures2d::StarDetector> *ptr, cv::xfeatures2d::StarDetector **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XFEATURES2D
    *returnValue = ptr->get();
#endif//HAVE_OPENCV_XFEATURES2D
    END_WRAP
}

#pragma endregion

#pragma region LUCID

CVAPI(ExceptionStatus) xfeatures2d_LUCID_create(const int lucid_kernel, const int blur_kernel, cv::Ptr<cv::xfeatures2d::LUCID> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XFEATURES2D
    const auto ptr = cv::xfeatures2d::LUCID::create(lucid_kernel, blur_kernel);
    *returnValue = clone(ptr);
#endif//HAVE_OPENCV_XFEATURES2D
    END_WRAP
}

CVAPI(ExceptionStatus) xfeatures2d_Ptr_LUCID_delete(cv::Ptr<cv::xfeatures2d::LUCID> *ptr)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XFEATURES2D
    delete ptr;
#endif//HAVE_OPENCV_XFEATURES2D
    END_WRAP
}

CVAPI(ExceptionStatus) xfeatures2d_Ptr_LUCID_get(cv::Ptr<cv::xfeatures2d::LUCID> *ptr, cv::xfeatures2d::LUCID **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XFEATURES2D
    *returnValue = ptr->get();
#endif//HAVE_OPENCV_XFEATURES2D
    END_WRAP
}

#pragma endregion

#pragma region LATCH

CVAPI(ExceptionStatus) xfeatures2d_LATCH_create(
    int bytes, int rotationInvariance, int half_ssd_size, double sigma, 
    cv::Ptr<cv::xfeatures2d::LATCH> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XFEATURES2D
    const auto ptr = cv::xfeatures2d::LATCH::create(bytes, rotationInvariance != 0, half_ssd_size, sigma);
    *returnValue = clone(ptr);
#endif//HAVE_OPENCV_XFEATURES2D
    END_WRAP
}

CVAPI(ExceptionStatus) xfeatures2d_Ptr_LATCH_delete(cv::Ptr<cv::xfeatures2d::LATCH> *ptr)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XFEATURES2D
    delete ptr;
#endif//HAVE_OPENCV_XFEATURES2D
    END_WRAP
}

CVAPI(ExceptionStatus) xfeatures2d_Ptr_LATCH_get(cv::Ptr<cv::xfeatures2d::LATCH> *ptr, cv::xfeatures2d::LATCH **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XFEATURES2D
    *returnValue = ptr->get();
#endif//HAVE_OPENCV_XFEATURES2D
    END_WRAP
}

#pragma endregion


#pragma region SURF

CVAPI(ExceptionStatus) xfeatures2d_SURF_create(
    double hessianThreshold, int nOctaves,
    int nOctaveLayers, int extended, int upright, 
    cv::Ptr<cv::xfeatures2d::SURF> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XFEATURES2D
    const auto ptr = cv::xfeatures2d::SURF::create(
        hessianThreshold, nOctaves, nOctaveLayers, extended != 0, upright != 0);
    *returnValue = clone(ptr);
#endif//HAVE_OPENCV_XFEATURES2D
    END_WRAP
}

CVAPI(ExceptionStatus) xfeatures2d_Ptr_SURF_delete(cv::Ptr<cv::xfeatures2d::SURF> *ptr)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XFEATURES2D
    delete ptr;
#endif//HAVE_OPENCV_XFEATURES2D
    END_WRAP
}

CVAPI(ExceptionStatus) xfeatures2d_Ptr_SURF_get(cv::Ptr<cv::xfeatures2d::SURF> *ptr, cv::xfeatures2d::SURF **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XFEATURES2D
    *returnValue = ptr->get();
#endif//HAVE_OPENCV_XFEATURES2D
    END_WRAP
}

CVAPI(ExceptionStatus) xfeatures2d_SURF_getHessianThreshold(cv::xfeatures2d::SURF *obj, double *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XFEATURES2D
    *returnValue = obj->getHessianThreshold();
#endif//HAVE_OPENCV_XFEATURES2D
    END_WRAP
}
CVAPI(ExceptionStatus) xfeatures2d_SURF_getNOctaves(cv::xfeatures2d::SURF *obj, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XFEATURES2D
    *returnValue = obj->getNOctaves();
#endif//HAVE_OPENCV_XFEATURES2D
    END_WRAP
}
CVAPI(ExceptionStatus) xfeatures2d_SURF_getNOctaveLayers(cv::xfeatures2d::SURF *obj, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XFEATURES2D
    *returnValue = obj->getNOctaveLayers();
#endif//HAVE_OPENCV_XFEATURES2D
    END_WRAP
}
CVAPI(ExceptionStatus) xfeatures2d_SURF_getExtended(cv::xfeatures2d::SURF *obj, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XFEATURES2D
    *returnValue = obj->getExtended() ? 1 : 0;
#endif//HAVE_OPENCV_XFEATURES2D
    END_WRAP
}
CVAPI(ExceptionStatus) xfeatures2d_SURF_getUpright(cv::xfeatures2d::SURF *obj, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XFEATURES2D
    *returnValue = obj->getUpright() ? 1 : 0;
#endif//HAVE_OPENCV_XFEATURES2D
    END_WRAP
}

CVAPI(ExceptionStatus) xfeatures2d_SURF_setHessianThreshold(cv::xfeatures2d::SURF *obj, double value)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XFEATURES2D
    obj->setHessianThreshold(value);
#endif//HAVE_OPENCV_XFEATURES2D
    END_WRAP
}
CVAPI(ExceptionStatus) xfeatures2d_SURF_setNOctaves(cv::xfeatures2d::SURF *obj, int value)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XFEATURES2D
    obj->setNOctaves(value);
#endif//HAVE_OPENCV_XFEATURES2D
    END_WRAP
}
CVAPI(ExceptionStatus) xfeatures2d_SURF_setNOctaveLayers(cv::xfeatures2d::SURF *obj, int value)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XFEATURES2D
    obj->setNOctaveLayers(value);
#endif//HAVE_OPENCV_XFEATURES2D
    END_WRAP
}
CVAPI(ExceptionStatus) xfeatures2d_SURF_setExtended(cv::xfeatures2d::SURF *obj, int value)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XFEATURES2D
    obj->setExtended(value != 0);
#endif//HAVE_OPENCV_XFEATURES2D
    END_WRAP
}
CVAPI(ExceptionStatus) xfeatures2d_SURF_setUpright(cv::xfeatures2d::SURF *obj, int value)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XFEATURES2D
    obj->setUpright(value != 0);
#endif//HAVE_OPENCV_XFEATURES2D
    END_WRAP
}

#pragma endregion

#endif