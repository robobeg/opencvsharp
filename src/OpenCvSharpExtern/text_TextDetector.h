#ifndef _CPP_TEXT_TEXTDETECTOR_H_
#define _CPP_TEXT_TEXTDETECTOR_H_

#ifndef _WINRT_DLL

// ReSharper disable IdentifierTypo
// ReSharper disable CppInconsistentNaming
// ReSharper disable CppNonInlineFunctionDefinitionInHeaderFile

#include "include_opencv.h"

namespace cv
{
    namespace text
    {
        class CV_EXPORTS_W TextDetector;
        class CV_EXPORTS_W TextDetectorCNN;
}
}

CVAPI(ExceptionStatus) text_TextDetector_detect(cv::text::TextDetector *obj, cv::_InputArray *inputImage, std::vector<cv::Rect> *Bbox, std::vector<float> *confidence)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_TEXT
    obj->detect(*inputImage, *Bbox, *confidence);
#endif//HAVE_OPENCV_TEXT
    END_WRAP
}

CVAPI(ExceptionStatus) text_TextDetectorCNN_detect(cv::text::TextDetectorCNN *obj, cv::_InputArray *inputImage, std::vector<cv::Rect> *Bbox, std::vector<float> *confidence)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_TEXT
    obj->detect(*inputImage, *Bbox, *confidence);
#endif//HAVE_OPENCV_TEXT
    END_WRAP
}

CVAPI(ExceptionStatus) text_TextDetectorCNN_create1(
    const char *modelArchFilename, const char *modelWeightsFilename, MyCvSize *detectionSizes, int detectionSizesLength,
    cv::Ptr<cv::text::TextDetectorCNN> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_TEXT
    std::vector<cv::Size> detectionSizesVec;
    if (detectionSizes != nullptr)
    {
        detectionSizesVec.resize(detectionSizesLength);
        for (int i = 0; i < detectionSizesLength; i++)
            detectionSizesVec[i] = cpp(detectionSizes[i]);
    }

    const auto ptr = cv::text::TextDetectorCNN::create(modelArchFilename, modelWeightsFilename, detectionSizesVec);
    *returnValue = clone(ptr);
#endif//HAVE_OPENCV_TEXT
    END_WRAP
}

CVAPI(ExceptionStatus) text_TextDetectorCNN_create2(
    const char *modelArchFilename, const char *modelWeightsFilename, cv::Ptr<cv::text::TextDetectorCNN> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_TEXT
    const auto ptr = cv::text::TextDetectorCNN::create(modelArchFilename, modelWeightsFilename);
    *returnValue = clone(ptr);
#endif//HAVE_OPENCV_TEXT
    END_WRAP
}

CVAPI(ExceptionStatus) text_Ptr_TextDetectorCNN_delete(cv::Ptr<cv::text::TextDetectorCNN> *obj)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_TEXT
    delete obj;
#endif//HAVE_OPENCV_TEXT
    END_WRAP
}

CVAPI(ExceptionStatus) text_Ptr_TextDetectorCNN_get(cv::Ptr<cv::text::TextDetectorCNN>* obj, cv::text::TextDetectorCNN **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_TEXT
    *returnValue = obj->get();
#endif//HAVE_OPENCV_TEXT
    END_WRAP
}

#endif // !#ifndef _WINRT_DLL

#endif