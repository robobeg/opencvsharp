#ifndef _CPP_DNN_NET_H_
#define _CPP_DNN_NET_H_

#ifndef _WINRT_DLL

// ReSharper disable IdentifierTypo
// ReSharper disable CppInconsistentNaming
// ReSharper disable CppNonInlineFunctionDefinitionInHeaderFile

#include "include_opencv.h"

namespace cv
{
    namespace dnn
    {
        class CV_EXPORTS_W_SIMPLE Net;
}
}


CVAPI(ExceptionStatus) dnn_Net_new(cv::dnn::Net **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_DNN
    *returnValue = new cv::dnn::Net;
#endif//HAVE_OPENCV_DNN
    END_WRAP
}

CVAPI(ExceptionStatus) dnn_Net_delete(cv::dnn::Net* net)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_DNN
    delete net;
#endif//HAVE_OPENCV_DNN	
    END_WRAP
}

CVAPI(ExceptionStatus) dnn_Net_readFromModelOptimizer(const char *xml, const char *bin, cv::dnn::Net **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_DNN	
    const auto net = cv::dnn::Net::readFromModelOptimizer(xml, bin);
    *returnValue = new cv::dnn::Net(net);
#endif//HAVE_OPENCV_DNN		
    END_WRAP
}

CVAPI(ExceptionStatus) dnn_Net_empty(cv::dnn::Net* net, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_DNN
    *returnValue = net->empty() ? 1 : 0;
#endif//HAVE_OPENCV_DNN		
    END_WRAP
}

CVAPI(ExceptionStatus) dnn_Net_dump(cv::dnn::Net* net, std::string *outString)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_DNN	
    outString->assign(net->dump());
#endif//HAVE_OPENCV_DNN		
    END_WRAP    
}

CVAPI(ExceptionStatus) dnn_Net_dumpToFile(cv::dnn::Net* net, const char *path)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_DNN	
    net->dumpToFile(path);
#endif//HAVE_OPENCV_DNN		
    END_WRAP    
}

CVAPI(ExceptionStatus) dnn_Net_getLayerId(cv::dnn::Net* net, const char *layer, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_DNN
    *returnValue = net->getLayerId(layer);
#endif//HAVE_OPENCV_DNN
    END_WRAP
}

CVAPI(ExceptionStatus) dnn_Net_getLayerNames(cv::dnn::Net* net, std::vector<cv::String> *outVec)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_DNN
    const auto result = net->getLayerNames();
    outVec->assign(result.begin(), result.end());
#endif//HAVE_OPENCV_DNN
    END_WRAP
}

CVAPI(ExceptionStatus) dnn_Net_connect1(cv::dnn::Net* net, const char *outPin, const char *inpPin)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_DNN
    net->connect(outPin, inpPin);
#endif//HAVE_OPENCV_DNN
    END_WRAP
}

CVAPI(ExceptionStatus) dnn_Net_connect2(cv::dnn::Net* net, int outLayerId, int outNum, int inpLayerId, int inpNum)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_DNN
    net->connect(outLayerId, outNum, inpLayerId, inpNum);
#endif//HAVE_OPENCV_DNN
    END_WRAP
}

CVAPI(ExceptionStatus) dnn_Net_setInputsNames(cv::dnn::Net* net, const char **inputBlobNames, int inputBlobNamesLength)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_DNN
    std::vector<cv::String> inputBlobNamesVec(inputBlobNamesLength);
    for (auto i = 0; i < inputBlobNamesLength; i++)
    {
        inputBlobNamesVec[i] = inputBlobNames[i];
    }
    net->setInputsNames(inputBlobNamesVec);
#endif//HAVE_OPENCV_DNN
    END_WRAP
}

CVAPI(ExceptionStatus) dnn_Net_forward1(cv::dnn::Net* net, const char *outputName, cv::Mat **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_DNN
    const auto outputNameStr = (outputName == nullptr) ? cv::String() : cv::String(outputName);
    const auto ret = net->forward(outputNameStr);
    *returnValue = new cv::Mat(ret);
#endif//HAVE_OPENCV_DNN
    END_WRAP
}

CVAPI(ExceptionStatus) dnn_Net_forward2(
    cv::dnn::Net* net, cv::Mat **outputBlobs, int outputBlobsLength, const char *outputName)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_DNN
    const auto outputNameStr = (outputName == nullptr) ? cv::String() : cv::String(outputName);
    std::vector<cv::Mat> outputBlobsVec;
    toVec(outputBlobs, outputBlobsLength, outputBlobsVec);

    net->forward(outputBlobsVec, outputNameStr);

    for (auto i = 0; i < outputBlobsLength; i++)
    {
        *outputBlobs[i] = outputBlobsVec[i];
    }
#endif//HAVE_OPENCV_DNN
    END_WRAP
}

CVAPI(ExceptionStatus) dnn_Net_forward3(
    cv::dnn::Net* net, cv::Mat **outputBlobs, int outputBlobsLength, const char **outBlobNames, int outBlobNamesLength)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_DNN
    std::vector<cv::Mat> outputBlobsVec;
    toVec(outputBlobs, outputBlobsLength, outputBlobsVec);

    std::vector<cv::String> outBlobNamesVec(outBlobNamesLength);
    for (auto i = 0; i < outBlobNamesLength; i++)
    {
        outBlobNamesVec[i] = outBlobNames[i];
    }

    net->forward(outputBlobsVec, outBlobNamesVec);

    for (auto i = 0; i < outputBlobsLength; i++)
    {
        *outputBlobs[i] = outputBlobsVec[i];
    }
#endif//HAVE_OPENCV_DNN
    END_WRAP
}

CVAPI(ExceptionStatus) dnn_Net_setHalideScheduler(cv::dnn::Net* net, const char *scheduler)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_DNN
    net->setHalideScheduler(scheduler);
#endif//HAVE_OPENCV_DNN
    END_WRAP
}

CVAPI(ExceptionStatus) dnn_Net_setPreferableBackend(cv::dnn::Net* net, int backendId)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_DNN
    net->setPreferableBackend(backendId);
#endif//HAVE_OPENCV_DNN
    END_WRAP
}

CVAPI(ExceptionStatus) dnn_Net_setPreferableTarget(cv::dnn::Net* net, int targetId)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_DNN
    net->setPreferableTarget(targetId);
#endif//HAVE_OPENCV_DNN
    END_WRAP
}

CVAPI(ExceptionStatus) dnn_Net_setInput(cv::dnn::Net* net, const cv::Mat *blob, const char *name)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_DNN
    const auto nameStr = (name == nullptr) ? "" : cv::String(name);
    net->setInput(*blob, nameStr);
#endif//HAVE_OPENCV_DNN
    END_WRAP
}

CVAPI(ExceptionStatus) dnn_Net_getUnconnectedOutLayers(cv::dnn::Net* net, std::vector<int> *result)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_DNN
    const auto v = net->getUnconnectedOutLayers();
    result->clear();
    result->resize(v.size());
    for (size_t i = 0; i < v.size(); i++)
    {
        result->at(i) = v[i];
    }
#endif//HAVE_OPENCV_DNN
    END_WRAP
}

CVAPI(ExceptionStatus) dnn_Net_getUnconnectedOutLayersNames(cv::dnn::Net* net, std::vector<std::string> *result)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_DNN
    const auto v = net->getUnconnectedOutLayersNames();
    result->clear();
    result->resize(v.size());
    for (size_t i = 0; i < v.size(); i++)
    {
        result->at(i) = v[i];
    }
#endif//HAVE_OPENCV_DNN
    END_WRAP
}

CVAPI(ExceptionStatus) dnn_Net_enableFusion(cv::dnn::Net* net, int fusion)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_DNN
    net->enableFusion(fusion != 0);
#endif//HAVE_OPENCV_DNN
    END_WRAP
}

CVAPI(ExceptionStatus) dnn_Net_getPerfProfile(cv::dnn::Net* net, std::vector<double> *timings, int64 *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_DNN
    *returnValue = net->getPerfProfile(*timings);
#endif//HAVE_OPENCV_DNN
    END_WRAP
}

#endif // !#ifndef _WINRT_DLL

#endif