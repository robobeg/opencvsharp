#ifndef _CPP_QUALITY_H_
#define _CPP_QUALITY_H_

#ifndef _WINRT_DLL

// ReSharper disable IdentifierTypo
// ReSharper disable CppInconsistentNaming
// ReSharper disable CppNonInlineFunctionDefinitionInHeaderFile

#include "include_opencv.h"

namespace cv
{
    namespace quality
    {
        class CV_EXPORTS_W QualityBase;
        class CV_EXPORTS_W QualityPSNR;
        class CV_EXPORTS_W QualitySSIM;
        class CV_EXPORTS_W QualityGMSD;
        class CV_EXPORTS_W QualityMSE;
        class CV_EXPORTS_W QualityBRISQUE;
    }
}

#pragma region QualityBase

CVAPI(ExceptionStatus) quality_QualityBase_compute(cv::quality::QualityBase *obj, cv::_InputArray *img, MyCvScalar *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_QUALITY
    const auto ret = obj->compute(*img);
    *returnValue = c(ret);
#endif//HAVE_OPENCV_QUALITY
    END_WRAP
}

CVAPI(ExceptionStatus) quality_QualityBase_getQualityMap(cv::quality::QualityBase *obj, cv::_OutputArray *dst)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_QUALITY
    obj->getQualityMap(*dst);
#endif//HAVE_OPENCV_QUALITY
    END_WRAP
}

CVAPI(ExceptionStatus) quality_QualityBase_clear(cv::quality::QualityBase *obj)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_QUALITY
    obj->clear();
#endif//HAVE_OPENCV_QUALITY
    END_WRAP
}

CVAPI(ExceptionStatus) quality_QualityBase_empty(cv::quality::QualityBase *obj, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_QUALITY
    *returnValue = obj->empty() ? 1 : 0;
#endif//HAVE_OPENCV_QUALITY
    END_WRAP
}

#pragma endregion

#pragma region QualityPSNR

CVAPI(ExceptionStatus) quality_createQualityPSNR(
    cv::_InputArray *ref, double maxPixelValue, cv::Ptr<cv::quality::QualityPSNR> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_QUALITY
    const auto ptr = cv::quality::QualityPSNR::create(*ref, maxPixelValue);
    *returnValue = clone(ptr);
#endif//HAVE_OPENCV_QUALITY
    END_WRAP
}

CVAPI(ExceptionStatus) quality_Ptr_QualityPSNR_delete(cv::Ptr<cv::quality::QualityPSNR> *obj)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_QUALITY
    delete obj;
#endif//HAVE_OPENCV_QUALITY
    END_WRAP
}

CVAPI(ExceptionStatus) quality_Ptr_QualityPSNR_get(
    cv::Ptr<cv::quality::QualityPSNR>* ptr, cv::quality::QualityPSNR **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_QUALITY
    *returnValue = ptr->get();
#endif//HAVE_OPENCV_QUALITY
    END_WRAP
}

CVAPI(ExceptionStatus) quality_QualityPSNR_staticCompute(
    cv::_InputArray *ref, cv::_InputArray *cmp, cv::_OutputArray *qualityMap, double maxPixelValue, MyCvScalar *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_QUALITY
    cv::Scalar ret;
    if (qualityMap == nullptr)
        ret = cv::quality::QualityPSNR::compute(*ref, *cmp, cv::noArray(), maxPixelValue);
    else
        ret = cv::quality::QualityPSNR::compute(*ref, *cmp, *qualityMap, maxPixelValue);
    *returnValue = c(ret);
#endif//HAVE_OPENCV_QUALITY
    END_WRAP
}

CVAPI(ExceptionStatus) quality_QualityPSNR_getMaxPixelValue(cv::quality::QualityPSNR *obj, double *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_QUALITY
    *returnValue = obj->getMaxPixelValue();
#endif//HAVE_OPENCV_QUALITY
    END_WRAP
}

CVAPI(ExceptionStatus) quality_QualityPSNR_setMaxPixelValue(cv::quality::QualityPSNR *obj, double val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_QUALITY
    obj->setMaxPixelValue(val);
#endif//HAVE_OPENCV_QUALITY
    END_WRAP
}

#pragma endregion

#pragma region QualitySSIM

CVAPI(ExceptionStatus) quality_createQualitySSIM(cv::_InputArray* ref, cv::Ptr<cv::quality::QualitySSIM> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_QUALITY
    const auto ptr = cv::quality::QualitySSIM::create(*ref);
    *returnValue = clone(ptr);
#endif//HAVE_OPENCV_QUALITY
    END_WRAP
}

CVAPI(ExceptionStatus) quality_Ptr_QualitySSIM_delete(cv::Ptr<cv::quality::QualitySSIM>* obj)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_QUALITY
    delete obj;
#endif//HAVE_OPENCV_QUALITY
    END_WRAP
}

CVAPI(ExceptionStatus) quality_Ptr_QualitySSIM_get(
    cv::Ptr<cv::quality::QualitySSIM>* ptr, cv::quality::QualitySSIM **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_QUALITY
    *returnValue = ptr->get();
#endif//HAVE_OPENCV_QUALITY
    END_WRAP
}

CVAPI(ExceptionStatus) quality_QualitySSIM_staticCompute(
    cv::_InputArray* ref, cv::_InputArray* cmp, cv::_OutputArray* qualityMap, MyCvScalar *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_QUALITY
    cv::Scalar ret;
    if (qualityMap == nullptr)
        ret = cv::quality::QualitySSIM::compute(*ref, *cmp, cv::noArray());
    else
        ret = cv::quality::QualitySSIM::compute(*ref, *cmp, *qualityMap);
    *returnValue = c(ret);
#endif//HAVE_OPENCV_QUALITY
    END_WRAP
}

#pragma endregion

#pragma region QualityGMSD

CVAPI(ExceptionStatus) quality_createQualityGMSD(cv::_InputArray* ref, cv::Ptr<cv::quality::QualityGMSD> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_QUALITY
    const auto ptr = cv::quality::QualityGMSD::create(*ref);
    *returnValue = clone(ptr);
#endif//HAVE_OPENCV_QUALITY
    END_WRAP
}

CVAPI(ExceptionStatus) quality_Ptr_QualityGMSD_delete(cv::Ptr<cv::quality::QualityGMSD>* obj)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_QUALITY
    delete obj;
#endif//HAVE_OPENCV_QUALITY
    END_WRAP
}

CVAPI(ExceptionStatus) quality_Ptr_QualityGMSD_get(
    cv::Ptr<cv::quality::QualityGMSD>* ptr, cv::quality::QualityGMSD **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_QUALITY
    *returnValue = ptr->get();
#endif//HAVE_OPENCV_QUALITY
    END_WRAP
}

CVAPI(ExceptionStatus) quality_QualityGMSD_staticCompute(
    cv::_InputArray* ref, cv::_InputArray* cmp, cv::_OutputArray* qualityMap, MyCvScalar *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_QUALITY
    cv::Scalar ret;
    if (qualityMap == nullptr)
        ret = cv::quality::QualityGMSD::compute(*ref, *cmp, cv::noArray());
    else
        ret = cv::quality::QualityGMSD::compute(*ref, *cmp, *qualityMap);
    *returnValue = c(ret);
#endif//HAVE_OPENCV_QUALITY
    END_WRAP
}

#pragma endregion

#pragma region QualityMSE

CVAPI(ExceptionStatus) quality_createQualityMSE(cv::_InputArray* ref, cv::Ptr<cv::quality::QualityMSE> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_QUALITY
    const auto ptr = cv::quality::QualityMSE::create(*ref);
    *returnValue = clone(ptr);
#endif//HAVE_OPENCV_QUALITY
    END_WRAP
}

CVAPI(ExceptionStatus) quality_Ptr_QualityMSE_delete(cv::Ptr<cv::quality::QualityMSE>* obj)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_QUALITY
    delete obj;
#endif//HAVE_OPENCV_QUALITY
    END_WRAP
}

CVAPI(ExceptionStatus) quality_Ptr_QualityMSE_get(
    cv::Ptr<cv::quality::QualityMSE>* ptr, cv::quality::QualityMSE **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_QUALITY
    *returnValue = ptr->get();
#endif//HAVE_OPENCV_QUALITY
    END_WRAP
}

CVAPI(ExceptionStatus) quality_QualityMSE_staticCompute(
    cv::_InputArray* ref, cv::_InputArray* cmp, cv::_OutputArray* qualityMap, MyCvScalar *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_QUALITY
    cv::Scalar ret;
    if (qualityMap == nullptr)
        ret = cv::quality::QualityMSE::compute(*ref, *cmp, cv::noArray());
    else
        ret = cv::quality::QualityMSE::compute(*ref, *cmp, *qualityMap);
    *returnValue = c(ret);
#endif//HAVE_OPENCV_QUALITY
    END_WRAP
}

#pragma endregion

#pragma region QualityBRISQUE

CVAPI(ExceptionStatus) quality_createQualityBRISQUE1(
    const char *modelFilePath, const char *rangeFilePath, cv::Ptr<cv::quality::QualityBRISQUE> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_QUALITY
    const auto ptr = cv::quality::QualityBRISQUE::create(modelFilePath, rangeFilePath);
    *returnValue = clone(ptr);
#endif//HAVE_OPENCV_QUALITY
    END_WRAP
}

CVAPI(ExceptionStatus) quality_createQualityBRISQUE2(
    cv::ml::SVM *model, cv::Mat *range, cv::Ptr<cv::quality::QualityBRISQUE> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_QUALITY
    const auto ptr = cv::quality::QualityBRISQUE::create(model, *range);
    *returnValue = clone(ptr);
#endif//HAVE_OPENCV_QUALITY
    END_WRAP
}

CVAPI(ExceptionStatus) quality_Ptr_QualityBRISQUE_delete(cv::Ptr<cv::quality::QualityBRISQUE>* obj)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_QUALITY
    delete obj;
#endif//HAVE_OPENCV_QUALITY
    END_WRAP
}

CVAPI(ExceptionStatus) quality_Ptr_QualityBRISQUE_get(
    cv::Ptr<cv::quality::QualityBRISQUE>* ptr, cv::quality::QualityBRISQUE **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_QUALITY
    *returnValue = ptr->get();
#endif//HAVE_OPENCV_QUALITY
    END_WRAP
}

CVAPI(ExceptionStatus) quality_QualityBRISQUE_staticCompute(
    cv::_InputArray* ref, const char* modelFilePath, const char* rangeFilePath, MyCvScalar *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_QUALITY
    const auto ret = cv::quality::QualityBRISQUE::compute(*ref, modelFilePath, rangeFilePath);
    *returnValue = c(ret);
#endif//HAVE_OPENCV_QUALITY
    END_WRAP
}

CVAPI(ExceptionStatus) quality_QualityBRISQUE_computeFeatures(
    cv::_InputArray* img, cv::_OutputArray *features)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_QUALITY
    cv::quality::QualityBRISQUE::computeFeatures(*img, *features);
#endif//HAVE_OPENCV_QUALITY
    END_WRAP
}

#pragma endregion

#endif // !#ifndef _WINRT_DLL

#endif