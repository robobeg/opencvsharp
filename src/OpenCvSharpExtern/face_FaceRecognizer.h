#ifndef _CPP_FACE_FACERECOGNIZER_H_
#define _CPP_FACE_FACERECOGNIZER_H_

// ReSharper disable CppNonInlineFunctionDefinitionInHeaderFile

#include "include_opencv.h"


namespace cv
{
    namespace face
    {
        class CV_EXPORTS_W BasicFaceRecognizer;
        class CV_EXPORTS_W FaceRecognizer;
        class CV_EXPORTS_W EigenFaceRecognizer;
        class CV_EXPORTS_W FisherFaceRecognizer;
        class CV_EXPORTS_W LBPHFaceRecognizer;
    }
}

#pragma region FaceRecognizer

CVAPI(ExceptionStatus) face_FaceRecognizer_train(
    cv::face::FaceRecognizer *obj, cv::Mat **src, int srcLength, int *labels, int labelsLength)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    std::vector<cv::Mat> srcVec(srcLength);
    for (auto i = 0; i < srcLength; i++)
        srcVec[i] = *src[i];
    const std::vector<int> labelsVec(labels, labels + labelsLength);
    obj->train(srcVec, labelsVec);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_FaceRecognizer_update(
    cv::face::FaceRecognizer *obj, cv::Mat **src, int srcLength, int *labels, int labelsLength)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    std::vector<cv::Mat> srcVec(srcLength);
    for (auto i = 0; i < srcLength; i++)
        srcVec[i] = *src[i];
    const std::vector<int> labelsVec(labels, labels + labelsLength);
    obj->update(srcVec, labelsVec);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_FaceRecognizer_predict1(cv::face::FaceRecognizer *obj, cv::_InputArray *src, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    *returnValue = obj->predict(*src);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_FaceRecognizer_predict2(
    cv::face::FaceRecognizer *obj, cv::_InputArray *src, int *label, double *confidence)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->predict(*src, *label, *confidence);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_FaceRecognizer_write1(cv::face::FaceRecognizer *obj, const char *filename)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->write(filename);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_FaceRecognizer_read1(cv::face::FaceRecognizer *obj, const char *filename)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->read(filename);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_FaceRecognizer_write2(cv::face::FaceRecognizer *obj, cv::FileStorage *fs)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->write(*fs);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_FaceRecognizer_read2(cv::face::FaceRecognizer *obj, cv::FileNode *fn)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->read(*fn);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_FaceRecognizer_setLabelInfo(cv::face::FaceRecognizer *obj, int label, const char *strInfo)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->setLabelInfo(label, strInfo);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_FaceRecognizer_getLabelInfo(cv::face::FaceRecognizer *obj, int label, std::string *dst)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    const auto result = obj->getLabelInfo(label);
    dst->assign(result);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_FaceRecognizer_getLabelsByString(cv::face::FaceRecognizer *obj, const char* str, std::vector<int> *dst)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    const auto result = obj->getLabelsByString(str);
    std::copy(result.begin(), result.end(), std::back_inserter(*dst));
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_FaceRecognizer_getThreshold(cv::face::FaceRecognizer *obj, double *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    *returnValue = obj->getThreshold();
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_FaceRecognizer_setThreshold(cv::face::FaceRecognizer *obj, double val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->setThreshold(val);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

#pragma endregion

#pragma region BasicFaceRecognizer

CVAPI(ExceptionStatus) face_BasicFaceRecognizer_getNumComponents(cv::face::BasicFaceRecognizer *obj, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    *returnValue = obj->getNumComponents();
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_BasicFaceRecognizer_setNumComponents(cv::face::BasicFaceRecognizer *obj, int val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->setNumComponents(val);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_BasicFaceRecognizer_getThreshold(cv::face::BasicFaceRecognizer *obj, double *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    *returnValue = obj->getThreshold();
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_BasicFaceRecognizer_setThreshold(cv::face::BasicFaceRecognizer *obj, double val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->setThreshold(val);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_BasicFaceRecognizer_getProjections(cv::face::BasicFaceRecognizer *obj, std::vector<cv::Mat> *dst)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    auto result = obj->getProjections();
    dst->clear();
    dst->reserve(result.size());
    for (size_t i = 0; i < result.size(); i++)
    {
        dst->push_back(result[i]);
    }
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_BasicFaceRecognizer_getLabels(cv::face::BasicFaceRecognizer *obj, cv::Mat *dst)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    const auto result = obj->getLabels();
    result.copyTo(*dst);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_BasicFaceRecognizer_getEigenValues(cv::face::BasicFaceRecognizer *obj, cv::Mat *dst)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    const auto result = obj->getEigenValues();
    result.copyTo(*dst);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_BasicFaceRecognizer_getEigenVectors(cv::face::BasicFaceRecognizer *obj, cv::Mat *dst)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    const auto result = obj->getEigenVectors();
    result.copyTo(*dst);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_BasicFaceRecognizer_getMean(cv::face::BasicFaceRecognizer *obj, cv::Mat *dst)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    const auto result = obj->getMean();
    result.copyTo(*dst);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

#pragma endregion

#pragma region EigenFaceRecognizer

CVAPI(ExceptionStatus) face_EigenFaceRecognizer_create(
    const int numComponents, const double threshold, cv::Ptr<cv::face::EigenFaceRecognizer> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    const auto r = cv::face::EigenFaceRecognizer::create(numComponents, threshold);
    *returnValue = clone(r);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_Ptr_EigenFaceRecognizer_get(cv::Ptr<cv::face::EigenFaceRecognizer> *obj, cv::face::EigenFaceRecognizer **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    *returnValue = obj->get();
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_Ptr_EigenFaceRecognizer_delete(cv::Ptr<cv::face::EigenFaceRecognizer> *obj)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    delete obj;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

#pragma endregion 

#pragma region FisherFaceRecognizer

CVAPI(ExceptionStatus) face_FisherFaceRecognizer_create(
    const int numComponents, const double threshold, cv::Ptr<cv::face::FisherFaceRecognizer> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    const auto r = cv::face::FisherFaceRecognizer::create(numComponents, threshold);
    *returnValue = clone(r);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_Ptr_FisherFaceRecognizer_get(cv::Ptr<cv::face::FisherFaceRecognizer> *obj, cv::face::FisherFaceRecognizer **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    *returnValue = obj->get();
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_Ptr_FisherFaceRecognizer_delete(cv::Ptr<cv::face::FisherFaceRecognizer> *obj)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    delete obj;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

#pragma endregion 

#pragma region LBPHFaceRecognizer

CVAPI(ExceptionStatus) face_LBPHFaceRecognizer_create(
    const int radius, const int neighbors, const int gridX, const int gridY, const double threshold,
    cv::Ptr<cv::face::LBPHFaceRecognizer> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    const auto r = cv::face::LBPHFaceRecognizer::create(radius, neighbors, gridX, gridY, threshold);
    *returnValue = clone(r);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_LBPHFaceRecognizer_getGridX(cv::face::LBPHFaceRecognizer *obj, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    *returnValue = obj->getGridX();
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_LBPHFaceRecognizer_setGridX(cv::face::LBPHFaceRecognizer *obj, int val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->setGridX(val);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_LBPHFaceRecognizer_getGridY(cv::face::LBPHFaceRecognizer *obj, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    *returnValue = obj->getGridY();
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_LBPHFaceRecognizer_setGridY(cv::face::LBPHFaceRecognizer *obj, int val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->setGridY(val);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_LBPHFaceRecognizer_getRadius(cv::face::LBPHFaceRecognizer *obj, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    *returnValue = obj->getRadius();
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_LBPHFaceRecognizer_setRadius(cv::face::LBPHFaceRecognizer *obj, int val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->setRadius(val);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_LBPHFaceRecognizer_getNeighbors(cv::face::LBPHFaceRecognizer *obj, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    *returnValue = obj->getNeighbors();
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_LBPHFaceRecognizer_setNeighbors(cv::face::LBPHFaceRecognizer *obj, int val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->setNeighbors(val);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_LBPHFaceRecognizer_getThreshold(cv::face::LBPHFaceRecognizer *obj, double *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    *returnValue = obj->getThreshold();
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_LBPHFaceRecognizer_setThreshold(cv::face::LBPHFaceRecognizer *obj, double val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->setThreshold(val);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_LBPHFaceRecognizer_getHistograms(cv::face::LBPHFaceRecognizer *obj, std::vector<cv::Mat> *dst)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    auto result = obj->getHistograms();
    dst->clear();
    dst->reserve(result.size());
    for (size_t i = 0; i < result.size(); i++)
    {
        dst->at(i) = result[i];
    }
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_LBPHFaceRecognizer_getLabels(cv::face::LBPHFaceRecognizer *obj, cv::Mat *dst)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    const auto result = obj->getLabels();
    result.copyTo(*dst);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}


CVAPI(ExceptionStatus) face_Ptr_LBPHFaceRecognizer_get(cv::Ptr<cv::face::LBPHFaceRecognizer> *obj, cv::face::LBPHFaceRecognizer **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    *returnValue = obj->get();
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_Ptr_LBPHFaceRecognizer_delete(cv::Ptr<cv::face::LBPHFaceRecognizer> *obj)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    delete obj;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

#pragma endregion 

#endif
