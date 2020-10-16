#ifndef _CPP_XIMGPROC_EDGEBOXES_H_
#define _CPP_XIMGPROC_EDGEBOXES_H_

// ReSharper disable IdentifierTypo
// ReSharper disable CppInconsistentNaming
// ReSharper disable CppNonInlineFunctionDefinitionInHeaderFile

#include "include_opencv.h"

namespace cv
{
    namespace ximgproc
    {
        class CV_EXPORTS_W EdgeBoxes;
    }
}

CVAPI(ExceptionStatus) ximgproc_EdgeBoxes_getBoundingBoxes(
    cv::ximgproc::EdgeBoxes *obj, cv::_InputArray *edge_map, 
    cv::_InputArray *orientation_map, std::vector<cv::Rect> *boxes)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    obj->getBoundingBoxes(*edge_map, *orientation_map, *boxes);
#endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}

CVAPI(ExceptionStatus) ximgproc_EdgeBoxes_getAlpha(cv::ximgproc::EdgeBoxes *obj, float *returnValue)          { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_XIMGPROC 
*returnValue = obj->getAlpha(); 
#endif//HAVE_OPENCV_XIMGPROC
END_WRAP }
CVAPI(ExceptionStatus) ximgproc_EdgeBoxes_setAlpha(cv::ximgproc::EdgeBoxes *obj, float value)                 { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_XIMGPROC 
obj->setAlpha(value); 
#endif//HAVE_OPENCV_XIMGPROC
END_WRAP }
CVAPI(ExceptionStatus) ximgproc_EdgeBoxes_getBeta(cv::ximgproc::EdgeBoxes *obj, float *returnValue)           { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_XIMGPROC 
*returnValue = obj->getBeta(); 
#endif//HAVE_OPENCV_XIMGPROC
END_WRAP }
CVAPI(ExceptionStatus) ximgproc_EdgeBoxes_setBeta(cv::ximgproc::EdgeBoxes *obj, float value)                  { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_XIMGPROC 
obj->setBeta(value); 
#endif//HAVE_OPENCV_XIMGPROC
END_WRAP }
CVAPI(ExceptionStatus) ximgproc_EdgeBoxes_getEta(cv::ximgproc::EdgeBoxes *obj, float *returnValue)            { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_XIMGPROC 
*returnValue = obj->getEta(); 
#endif//HAVE_OPENCV_XIMGPROC
END_WRAP }
CVAPI(ExceptionStatus) ximgproc_EdgeBoxes_setEta(cv::ximgproc::EdgeBoxes *obj, float value)                   { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_XIMGPROC 
obj->setEta(value); 
#endif//HAVE_OPENCV_XIMGPROC
END_WRAP }
CVAPI(ExceptionStatus) ximgproc_EdgeBoxes_getMinScore(cv::ximgproc::EdgeBoxes *obj, float *returnValue)       { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_XIMGPROC 
*returnValue = obj->getMinScore(); 
#endif//HAVE_OPENCV_XIMGPROC
END_WRAP }
CVAPI(ExceptionStatus) ximgproc_EdgeBoxes_setMinScore(cv::ximgproc::EdgeBoxes *obj, float value)              { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_XIMGPROC 
obj->setMinScore(value); 
#endif//HAVE_OPENCV_XIMGPROC
END_WRAP }
CVAPI(ExceptionStatus) ximgproc_EdgeBoxes_getMaxBoxes(cv::ximgproc::EdgeBoxes *obj, int *returnValue)         { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_XIMGPROC 
*returnValue = obj->getMaxBoxes(); 
#endif//HAVE_OPENCV_XIMGPROC
END_WRAP }
CVAPI(ExceptionStatus) ximgproc_EdgeBoxes_setMaxBoxes(cv::ximgproc::EdgeBoxes *obj, int value)                { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_XIMGPROC 
obj->setMaxBoxes(value); 
#endif//HAVE_OPENCV_XIMGPROC
END_WRAP }
CVAPI(ExceptionStatus) ximgproc_EdgeBoxes_getEdgeMinMag(cv::ximgproc::EdgeBoxes *obj, float *returnValue)     { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_XIMGPROC 
*returnValue = obj->getEdgeMinMag(); 
#endif//HAVE_OPENCV_XIMGPROC
END_WRAP }
CVAPI(ExceptionStatus) ximgproc_EdgeBoxes_setEdgeMinMag(cv::ximgproc::EdgeBoxes *obj, float value)            { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_XIMGPROC 
obj->setEdgeMinMag(value); 
#endif//HAVE_OPENCV_XIMGPROC
END_WRAP }
CVAPI(ExceptionStatus) ximgproc_EdgeBoxes_getEdgeMergeThr(cv::ximgproc::EdgeBoxes *obj, float *returnValue)   { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_XIMGPROC 
*returnValue = obj->getEdgeMergeThr(); 
#endif//HAVE_OPENCV_XIMGPROC
END_WRAP }
CVAPI(ExceptionStatus) ximgproc_EdgeBoxes_setEdgeMergeThr(cv::ximgproc::EdgeBoxes *obj, float value)          { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_XIMGPROC 
obj->setEdgeMergeThr(value); 
#endif//HAVE_OPENCV_XIMGPROC
END_WRAP }
CVAPI(ExceptionStatus) ximgproc_EdgeBoxes_getClusterMinMag(cv::ximgproc::EdgeBoxes *obj, float *returnValue)  { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_XIMGPROC 
*returnValue = obj->getClusterMinMag(); 
#endif//HAVE_OPENCV_XIMGPROC
END_WRAP }
CVAPI(ExceptionStatus) ximgproc_EdgeBoxes_setClusterMinMag(cv::ximgproc::EdgeBoxes *obj, float value)         { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_XIMGPROC 
obj->setClusterMinMag(value); 
#endif//HAVE_OPENCV_XIMGPROC
END_WRAP }
CVAPI(ExceptionStatus) ximgproc_EdgeBoxes_getMaxAspectRatio(cv::ximgproc::EdgeBoxes *obj, float *returnValue) { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_XIMGPROC 
*returnValue = obj->getMaxAspectRatio(); 
#endif//HAVE_OPENCV_XIMGPROC
END_WRAP }
CVAPI(ExceptionStatus) ximgproc_EdgeBoxes_setMaxAspectRatio(cv::ximgproc::EdgeBoxes *obj, float value)        { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_XIMGPROC 
obj->setMaxAspectRatio(value); 
#endif//HAVE_OPENCV_XIMGPROC
END_WRAP }
CVAPI(ExceptionStatus) ximgproc_EdgeBoxes_getMinBoxArea(cv::ximgproc::EdgeBoxes *obj, float *returnValue)     { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_XIMGPROC 
*returnValue = obj->getMinBoxArea(); 
#endif//HAVE_OPENCV_XIMGPROC
END_WRAP }
CVAPI(ExceptionStatus) ximgproc_EdgeBoxes_setMinBoxArea(cv::ximgproc::EdgeBoxes *obj, float value)            { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_XIMGPROC 
obj->setMinBoxArea(value); 
#endif//HAVE_OPENCV_XIMGPROC
END_WRAP }
CVAPI(ExceptionStatus) ximgproc_EdgeBoxes_getGamma(cv::ximgproc::EdgeBoxes *obj, float *returnValue)          { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_XIMGPROC 
*returnValue = obj->getGamma(); 
#endif//HAVE_OPENCV_XIMGPROC
END_WRAP }
CVAPI(ExceptionStatus) ximgproc_EdgeBoxes_setGamma(cv::ximgproc::EdgeBoxes *obj, float value)                 { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_XIMGPROC 
obj->setGamma(value); 
#endif//HAVE_OPENCV_XIMGPROC
END_WRAP }
CVAPI(ExceptionStatus) ximgproc_EdgeBoxes_getKappa(cv::ximgproc::EdgeBoxes *obj, float *returnValue)          { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_XIMGPROC 
*returnValue = obj->getKappa(); 
#endif//HAVE_OPENCV_XIMGPROC
END_WRAP }
CVAPI(ExceptionStatus) ximgproc_EdgeBoxes_setKappa(cv::ximgproc::EdgeBoxes *obj, float value)                 { 
BEGIN_WRAP 
#ifdef HAVE_OPENCV_XIMGPROC 
obj->setKappa(value); 
#endif//HAVE_OPENCV_XIMGPROC
END_WRAP }


CVAPI(ExceptionStatus) ximgproc_createEdgeBoxes(
    float alpha,  float beta, float eta, float minScore, int maxBoxes, float edgeMinMag, float edgeMergeThr,
    float clusterMinMag, float maxAspectRatio, float minBoxArea, float gamma, float kappa,
    cv::Ptr<cv::ximgproc::EdgeBoxes> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    *returnValue = clone(cv::ximgproc::createEdgeBoxes(alpha, beta, eta, minScore, maxBoxes, edgeMinMag, edgeMergeThr,
        clusterMinMag, maxAspectRatio, minBoxArea, gamma, kappa));
#endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}

CVAPI(ExceptionStatus) ximgproc_Ptr_EdgeBoxes_delete(cv::Ptr<cv::ximgproc::EdgeBoxes> *obj)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    delete obj;
#endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}

CVAPI(ExceptionStatus) ximgproc_Ptr_EdgeBoxes_get(cv::Ptr<cv::ximgproc::EdgeBoxes> *ptr, cv::ximgproc::EdgeBoxes **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_XIMGPROC
    *returnValue = ptr->get();
#endif//HAVE_OPENCV_XIMGPROC
    END_WRAP
}

#endif