#ifndef _CPP_SHAPE_SHAPEDISTANCEEXTRACTOR_H_
#define _CPP_SHAPE_SHAPEDISTANCEEXTRACTOR_H_

// ReSharper disable IdentifierTypo
// ReSharper disable CppInconsistentNaming
// ReSharper disable CppNonInlineFunctionDefinitionInHeaderFile

#include "include_opencv.h"

namespace cv
{
    class CV_EXPORTS_W ShapeDistanceExtractor;
    class CV_EXPORTS_W ShapeContextDistanceExtractor;
    class CV_EXPORTS_W HausdorffDistanceExtractor;
}

CVAPI(ExceptionStatus) shape_ShapeDistanceExtractor_computeDistance(
    cv::ShapeDistanceExtractor *obj, cv::_InputArray *contour1, cv::_InputArray *contour2, float *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SHAPE
    *returnValue = obj->computeDistance(*contour1, *contour2);
#endif//HAVE_OPENCV_SHAPE
    END_WRAP
}

#pragma region ShapeContextDistanceExtractor

CVAPI(ExceptionStatus) shape_Ptr_ShapeContextDistanceExtractor_delete(
    cv::Ptr<cv::ShapeContextDistanceExtractor> *obj)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SHAPE
    delete obj;
#endif//HAVE_OPENCV_SHAPE
    END_WRAP
}
CVAPI(ExceptionStatus) shape_Ptr_ShapeContextDistanceExtractor_get(
    cv::Ptr<cv::ShapeContextDistanceExtractor> *obj, cv::ShapeContextDistanceExtractor **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SHAPE
    *returnValue = obj->get();
#endif//HAVE_OPENCV_SHAPE
    END_WRAP
}

CVAPI(ExceptionStatus) shape_ShapeContextDistanceExtractor_setAngularBins(
    cv::ShapeContextDistanceExtractor *obj, int val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SHAPE
    obj->setAngularBins(val);
#endif//HAVE_OPENCV_SHAPE
    END_WRAP
}
CVAPI(ExceptionStatus) shape_ShapeContextDistanceExtractor_getAngularBins(
    cv::ShapeContextDistanceExtractor *obj, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SHAPE
    *returnValue = obj->getAngularBins();
#endif//HAVE_OPENCV_SHAPE
    END_WRAP
}

CVAPI(ExceptionStatus) shape_ShapeContextDistanceExtractor_setRadialBins(
    cv::ShapeContextDistanceExtractor *obj, int val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SHAPE
    obj->setRadialBins(val);
#endif//HAVE_OPENCV_SHAPE
    END_WRAP
}
CVAPI(ExceptionStatus) shape_ShapeContextDistanceExtractor_getRadialBins(
    cv::ShapeContextDistanceExtractor *obj, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SHAPE
    *returnValue = obj->getRadialBins();
#endif//HAVE_OPENCV_SHAPE
    END_WRAP
}

CVAPI(ExceptionStatus) shape_ShapeContextDistanceExtractor_setInnerRadius(
    cv::ShapeContextDistanceExtractor *obj, float val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SHAPE
    obj->setInnerRadius(val);
#endif//HAVE_OPENCV_SHAPE
    END_WRAP
}
CVAPI(ExceptionStatus) shape_ShapeContextDistanceExtractor_getInnerRadius(
    cv::ShapeContextDistanceExtractor *obj, float *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SHAPE
    *returnValue = obj->getInnerRadius();
#endif//HAVE_OPENCV_SHAPE
    END_WRAP
}

CVAPI(ExceptionStatus) shape_ShapeContextDistanceExtractor_setOuterRadius(
    cv::ShapeContextDistanceExtractor *obj, float val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SHAPE
    obj->setOuterRadius(val);
#endif//HAVE_OPENCV_SHAPE
    END_WRAP
}
CVAPI(ExceptionStatus) shape_ShapeContextDistanceExtractor_getOuterRadius(
    cv::ShapeContextDistanceExtractor *obj, float *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SHAPE
    *returnValue = obj->getOuterRadius();
#endif//HAVE_OPENCV_SHAPE
    END_WRAP
}

CVAPI(ExceptionStatus) shape_ShapeContextDistanceExtractor_setRotationInvariant(
    cv::ShapeContextDistanceExtractor *obj, int val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SHAPE
    obj->setRotationInvariant(val != 0);
#endif//HAVE_OPENCV_SHAPE
    END_WRAP
}
CVAPI(ExceptionStatus) shape_ShapeContextDistanceExtractor_getRotationInvariant(
    cv::ShapeContextDistanceExtractor *obj, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SHAPE
    *returnValue = obj->getRotationInvariant() ? 1 : 0;
#endif//HAVE_OPENCV_SHAPE
    END_WRAP
}

CVAPI(ExceptionStatus) shape_ShapeContextDistanceExtractor_setShapeContextWeight(
    cv::ShapeContextDistanceExtractor *obj, float val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SHAPE
    obj->setShapeContextWeight(val);
#endif//HAVE_OPENCV_SHAPE
    END_WRAP
}
CVAPI(ExceptionStatus) shape_ShapeContextDistanceExtractor_getShapeContextWeight(
    cv::ShapeContextDistanceExtractor *obj, float *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SHAPE
    *returnValue = obj->getShapeContextWeight();
#endif//HAVE_OPENCV_SHAPE
    END_WRAP
}

CVAPI(ExceptionStatus) shape_ShapeContextDistanceExtractor_setImageAppearanceWeight(
    cv::ShapeContextDistanceExtractor *obj, float val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SHAPE
    obj->setImageAppearanceWeight(val);
#endif//HAVE_OPENCV_SHAPE
    END_WRAP
}
CVAPI(ExceptionStatus) shape_ShapeContextDistanceExtractor_getImageAppearanceWeight(
    cv::ShapeContextDistanceExtractor *obj, float *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SHAPE
    *returnValue = obj->getImageAppearanceWeight();
#endif//HAVE_OPENCV_SHAPE
    END_WRAP
}

CVAPI(ExceptionStatus) shape_ShapeContextDistanceExtractor_setBendingEnergyWeight(
    cv::ShapeContextDistanceExtractor *obj, float val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SHAPE
    obj->setBendingEnergyWeight(val);
#endif//HAVE_OPENCV_SHAPE
    END_WRAP
}
CVAPI(ExceptionStatus) shape_ShapeContextDistanceExtractor_getBendingEnergyWeight(
    cv::ShapeContextDistanceExtractor *obj, float *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SHAPE
    *returnValue = obj->getBendingEnergyWeight();
#endif//HAVE_OPENCV_SHAPE
    END_WRAP
}

CVAPI(ExceptionStatus) shape_ShapeContextDistanceExtractor_setImages(
    cv::ShapeContextDistanceExtractor *obj, cv::_InputArray *image1, cv::_InputArray *image2)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SHAPE
    obj->setImages(*image1, *image2);
#endif//HAVE_OPENCV_SHAPE
    END_WRAP
}
CVAPI(ExceptionStatus) shape_ShapeContextDistanceExtractor_getImages(
    cv::ShapeContextDistanceExtractor *obj, cv::_OutputArray *image1, cv::_OutputArray *image2)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SHAPE
    obj->getImages(*image1, *image2);
#endif//HAVE_OPENCV_SHAPE
    END_WRAP
}

CVAPI(ExceptionStatus) shape_ShapeContextDistanceExtractor_setIterations(
    cv::ShapeContextDistanceExtractor *obj, int val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SHAPE
    obj->setIterations(val);
#endif//HAVE_OPENCV_SHAPE
    END_WRAP
}
CVAPI(ExceptionStatus) shape_ShapeContextDistanceExtractor_getIterations(
    cv::ShapeContextDistanceExtractor *obj, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SHAPE
    *returnValue = obj->getIterations();
#endif//HAVE_OPENCV_SHAPE
    END_WRAP
}

/*CVAPI(void) shape_ShapeContextDistanceExtractor_setCostExtractor(
    cv::ShapeContextDistanceExtractor *obj, Ptr<HistogramCostExtractor> comparer)
{

}*/
/*CVAPI(Ptr<HistogramCostExtractor>) shape_ShapeContextDistanceExtractor_getCostExtractor(
    cv::ShapeContextDistanceExtractor *obj)
{

}*/

CVAPI(ExceptionStatus) shape_ShapeContextDistanceExtractor_setStdDev(
    cv::ShapeContextDistanceExtractor *obj, float val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SHAPE
    obj->setStdDev(val);
#endif//HAVE_OPENCV_SHAPE
    END_WRAP
}
CVAPI(ExceptionStatus) shape_ShapeContextDistanceExtractor_getStdDev(
    cv::ShapeContextDistanceExtractor *obj, float *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SHAPE
    *returnValue = obj->getStdDev();
#endif//HAVE_OPENCV_SHAPE
    END_WRAP
}

/*CVAPI(void) shape_ShapeContextDistanceExtractor_setTransformAlgorithm(
    cv::ShapeContextDistanceExtractor *obj, Ptr<ShapeTransformer> transformer)
{
}*/
/*CVAPI(Ptr<ShapeTransformer>) shape_ShapeContextDistanceExtractor_getTransformAlgorithm(
    cv::ShapeContextDistanceExtractor *obj)
{

}
*/

CVAPI(ExceptionStatus) shape_createShapeContextDistanceExtractor(
    int nAngularBins, int nRadialBins,
    float innerRadius, float outerRadius, int iterations/*,
    const Ptr<HistogramCostExtractor> &comparer = createChiHistogramCostExtractor(),
    const Ptr<ShapeTransformer> &transformer = createThinPlateSplineShapeTransformer()*/,
    cv::Ptr<cv::ShapeContextDistanceExtractor> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SHAPE
    const auto p = cv::createShapeContextDistanceExtractor(
        nAngularBins, nRadialBins, innerRadius, outerRadius, iterations);
    *returnValue = clone(p);
#endif//HAVE_OPENCV_SHAPE
    END_WRAP
}

#pragma endregion

#pragma region HausdorffDistanceExtractor

CVAPI(ExceptionStatus) shape_Ptr_HausdorffDistanceExtractor_delete(
    cv::Ptr<cv::HausdorffDistanceExtractor> *obj)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SHAPE
    delete obj;
#endif//HAVE_OPENCV_SHAPE
    END_WRAP
}
CVAPI(ExceptionStatus) shape_Ptr_HausdorffDistanceExtractor_get(
    cv::Ptr<cv::HausdorffDistanceExtractor> *obj, cv::HausdorffDistanceExtractor **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SHAPE
    *returnValue = obj->get();
#endif//HAVE_OPENCV_SHAPE
    END_WRAP
}


CVAPI(ExceptionStatus) shape_HausdorffDistanceExtractor_setDistanceFlag(
    cv::HausdorffDistanceExtractor *obj, int val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SHAPE
    obj->setDistanceFlag(val);
#endif//HAVE_OPENCV_SHAPE
    END_WRAP
}
CVAPI(ExceptionStatus) shape_HausdorffDistanceExtractor_getDistanceFlag(
    cv::HausdorffDistanceExtractor *obj, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SHAPE
    *returnValue = obj->getDistanceFlag();
#endif//HAVE_OPENCV_SHAPE
    END_WRAP
}

CVAPI(ExceptionStatus) shape_HausdorffDistanceExtractor_setRankProportion(
    cv::HausdorffDistanceExtractor *obj, float val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SHAPE
    obj->setRankProportion(val);
#endif//HAVE_OPENCV_SHAPE
    END_WRAP
}
CVAPI(ExceptionStatus) shape_HausdorffDistanceExtractor_getRankProportion(
    cv::HausdorffDistanceExtractor *obj, float *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SHAPE
    *returnValue = obj->getRankProportion();
#endif//HAVE_OPENCV_SHAPE
    END_WRAP
}


CVAPI(ExceptionStatus) shape_createHausdorffDistanceExtractor(
    int distanceFlag, float rankProp, cv::Ptr<cv::HausdorffDistanceExtractor> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_SHAPE
    const auto p = cv::createHausdorffDistanceExtractor(
        distanceFlag, rankProp);
    *returnValue = clone(p);
#endif//HAVE_OPENCV_SHAPE
    END_WRAP
}

#pragma endregion

#endif