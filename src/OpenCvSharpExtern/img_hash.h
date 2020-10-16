#ifndef _CPP_IMG_HASH_H_
#define _CPP_IMG_HASH_H_

// ReSharper disable IdentifierTypo
// ReSharper disable CppInconsistentNaming
// ReSharper disable CppNonInlineFunctionDefinitionInHeaderFile

#include "include_opencv.h"


namespace cv
{
    namespace img_hash
    {
        class CV_EXPORTS_W ImgHashBase;
        class CV_EXPORTS_W AverageHash;
        class CV_EXPORTS_W BlockMeanHash;
        class CV_EXPORTS_W ColorMomentHash;
        class CV_EXPORTS_W MarrHildrethHash;
        class CV_EXPORTS_W PHash;
        class CV_EXPORTS_W RadialVarianceHash;
    }
}



CVAPI(ExceptionStatus) img_hash_ImgHashBase_compute(cv::img_hash::ImgHashBase *obj, cv::_InputArray *inputArr, cv::_OutputArray *outputArr)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_IMG_HASH
    obj->compute(*inputArr, *outputArr);
#endif//HAVE_OPENCV_IMG_HASH
    END_WRAP
}

CVAPI(ExceptionStatus) img_hash_ImgHashBase_compare(cv::img_hash::ImgHashBase *obj, cv::_InputArray *hashOne, cv::_InputArray *hashTwo, double *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_IMG_HASH
    *returnValue = obj->compare(*hashOne, *hashTwo);
#endif//HAVE_OPENCV_IMG_HASH
    END_WRAP
}


// AverageHash

CVAPI(ExceptionStatus) img_hash_AverageHash_create(cv::Ptr<cv::img_hash::AverageHash> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_IMG_HASH
    const auto ptr = cv::img_hash::AverageHash::create();
    *returnValue = clone(ptr);
#endif//HAVE_OPENCV_IMG_HASH
    END_WRAP
}

CVAPI(ExceptionStatus) img_hash_Ptr_AverageHash_delete(cv::Ptr<cv::img_hash::AverageHash> *ptr)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_IMG_HASH
    delete ptr;
#endif//HAVE_OPENCV_IMG_HASH
    END_WRAP
}

CVAPI(ExceptionStatus) img_hash_Ptr_AverageHash_get(cv::Ptr<cv::img_hash::AverageHash> *ptr, cv::img_hash::AverageHash **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_IMG_HASH
    *returnValue = ptr->get();
#endif//HAVE_OPENCV_IMG_HASH
    END_WRAP
}


// BlockMeanHash

CVAPI(ExceptionStatus) img_hash_BlockMeanHash_create(const int mode, cv::Ptr<cv::img_hash::BlockMeanHash> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_IMG_HASH
    const auto ptr = cv::img_hash::BlockMeanHash::create(mode);
    *returnValue = clone(ptr);
#endif//HAVE_OPENCV_IMG_HASH
    END_WRAP
}

CVAPI(ExceptionStatus) img_hash_Ptr_BlockMeanHash_delete(cv::Ptr<cv::img_hash::BlockMeanHash> *ptr)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_IMG_HASH
    delete ptr;
#endif//HAVE_OPENCV_IMG_HASH
    END_WRAP
}

CVAPI(ExceptionStatus) img_hash_Ptr_BlockMeanHash_get(cv::Ptr<cv::img_hash::BlockMeanHash> *ptr, cv::img_hash::BlockMeanHash **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_IMG_HASH
    *returnValue = ptr->get();
#endif//HAVE_OPENCV_IMG_HASH
    END_WRAP
}

CVAPI(ExceptionStatus) img_hash_BlockMeanHash_setMode(cv::img_hash::BlockMeanHash *obj, const int mode)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_IMG_HASH
    obj->setMode(mode);
#endif//HAVE_OPENCV_IMG_HASH
    END_WRAP
}

CVAPI(ExceptionStatus) img_hash_BlockMeanHash_getMean(cv::img_hash::BlockMeanHash *obj, std::vector<double> *outVec)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_IMG_HASH
    const auto mean = obj->getMean();
    outVec->clear();
    outVec->assign(mean.begin(), mean.end());
#endif//HAVE_OPENCV_IMG_HASH
    END_WRAP
}


// ColorMomentHash

CVAPI(ExceptionStatus) img_hash_ColorMomentHash_create(cv::Ptr<cv::img_hash::ColorMomentHash> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_IMG_HASH
    const auto ptr = cv::img_hash::ColorMomentHash::create();
    *returnValue = clone(ptr);
#endif//HAVE_OPENCV_IMG_HASH
    END_WRAP
}

CVAPI(ExceptionStatus) img_hash_Ptr_ColorMomentHash_delete(cv::Ptr<cv::img_hash::ColorMomentHash> *ptr)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_IMG_HASH
    delete ptr;
#endif//HAVE_OPENCV_IMG_HASH
    END_WRAP
}

CVAPI(ExceptionStatus) img_hash_Ptr_ColorMomentHash_get(cv::Ptr<cv::img_hash::ColorMomentHash> *ptr, cv::img_hash::ColorMomentHash **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_IMG_HASH
    *returnValue = ptr->get();
#endif//HAVE_OPENCV_IMG_HASH
    END_WRAP
}


// MarrHildrethHash

CVAPI(ExceptionStatus) img_hash_MarrHildrethHash_create(const float alpha, const float scale, cv::Ptr<cv::img_hash::MarrHildrethHash> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_IMG_HASH
    const auto ptr = cv::img_hash::MarrHildrethHash::create(alpha, scale);
    *returnValue = clone(ptr);
#endif//HAVE_OPENCV_IMG_HASH
    END_WRAP
}

CVAPI(ExceptionStatus) img_hash_Ptr_MarrHildrethHash_delete(cv::Ptr<cv::img_hash::MarrHildrethHash> *ptr)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_IMG_HASH
    delete ptr;
#endif//HAVE_OPENCV_IMG_HASH
    END_WRAP
}

CVAPI(ExceptionStatus) img_hash_Ptr_MarrHildrethHash_get(cv::Ptr<cv::img_hash::MarrHildrethHash> *ptr, cv::img_hash::MarrHildrethHash **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_IMG_HASH
    *returnValue = ptr->get();
#endif//HAVE_OPENCV_IMG_HASH
    END_WRAP
}

CVAPI(ExceptionStatus) img_hash_MarrHildrethHash_setKernelParam(cv::img_hash::MarrHildrethHash *obj, const float alpha, const float scale)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_IMG_HASH
    obj->setKernelParam(alpha, scale);
#endif//HAVE_OPENCV_IMG_HASH
    END_WRAP
}

CVAPI(ExceptionStatus) img_hash_MarrHildrethHash_getAlpha(cv::img_hash::MarrHildrethHash *obj, float *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_IMG_HASH
    *returnValue = obj->getAlpha();
#endif//HAVE_OPENCV_IMG_HASH
    END_WRAP
}

CVAPI(ExceptionStatus) img_hash_MarrHildrethHash_getScale(cv::img_hash::MarrHildrethHash *obj, float *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_IMG_HASH
    *returnValue = obj->getScale();
#endif//HAVE_OPENCV_IMG_HASH
    END_WRAP
}


// PHash

CVAPI(ExceptionStatus) img_hash_PHash_create(cv::Ptr<cv::img_hash::PHash> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_IMG_HASH
    const auto ptr = cv::img_hash::PHash::create();
    *returnValue = clone(ptr);
#endif//HAVE_OPENCV_IMG_HASH
    END_WRAP
}

CVAPI(ExceptionStatus) img_hash_Ptr_PHash_delete(cv::Ptr<cv::img_hash::PHash> *ptr)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_IMG_HASH
    delete ptr;
#endif//HAVE_OPENCV_IMG_HASH
    END_WRAP
}

CVAPI(ExceptionStatus) img_hash_Ptr_PHash_get(cv::Ptr<cv::img_hash::PHash> *ptr, cv::img_hash::PHash **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_IMG_HASH
    *returnValue = ptr->get();
#endif//HAVE_OPENCV_IMG_HASH
    END_WRAP
}


// RadialVarianceHash

CVAPI(ExceptionStatus) img_hash_RadialVarianceHash_create(const double sigma, const int numOfAngleLine, cv::Ptr<cv::img_hash::RadialVarianceHash> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_IMG_HASH
    const auto ptr = cv::img_hash::RadialVarianceHash::create(sigma, numOfAngleLine);
    *returnValue = clone(ptr);
#endif//HAVE_OPENCV_IMG_HASH
    END_WRAP
}

CVAPI(ExceptionStatus) img_hash_Ptr_RadialVarianceHash_delete(cv::Ptr<cv::img_hash::RadialVarianceHash> *ptr)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_IMG_HASH
    delete ptr;
#endif//HAVE_OPENCV_IMG_HASH
    END_WRAP
}

CVAPI(ExceptionStatus) img_hash_Ptr_RadialVarianceHash_get(cv::Ptr<cv::img_hash::RadialVarianceHash> *ptr, cv::img_hash::RadialVarianceHash **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_IMG_HASH
    *returnValue = ptr->get();
#endif//HAVE_OPENCV_IMG_HASH
    END_WRAP
}

CVAPI(ExceptionStatus) img_hash_RadialVarianceHash_setNumOfAngleLine(cv::img_hash::RadialVarianceHash *obj, const int value)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_IMG_HASH
    obj->setNumOfAngleLine(value);
#endif//HAVE_OPENCV_IMG_HASH
    END_WRAP
}

CVAPI(ExceptionStatus) img_hash_RadialVarianceHash_setSigma(cv::img_hash::RadialVarianceHash *obj, const double value)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_IMG_HASH
    obj->setSigma(value);
#endif//HAVE_OPENCV_IMG_HASH
    END_WRAP
}

CVAPI(ExceptionStatus) img_hash_RadialVarianceHash_getNumOfAngleLine(cv::img_hash::RadialVarianceHash *obj, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_IMG_HASH
    *returnValue = obj->getNumOfAngleLine();
#endif//HAVE_OPENCV_IMG_HASH
    END_WRAP
}

CVAPI(ExceptionStatus) img_hash_RadialVarianceHash_getSigma(cv::img_hash::RadialVarianceHash *obj, double *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_IMG_HASH
    *returnValue = obj->getSigma();
#endif//HAVE_OPENCV_IMG_HASH
    END_WRAP
}

#endif