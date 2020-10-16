#ifndef _INCLUDE_OPENCV_H_
#define _INCLUDE_OPENCV_H_

//#define ENABLED_CONTRIB
//#undef ENABLED_CONTRIB

#ifndef CV_EXPORTS
# if (defined _WIN32 || defined WINCE || defined __CYGWIN__)
#   define CV_EXPORTS __declspec(dllexport)
# elif defined __GNUC__ && __GNUC__ >= 4 && defined(__APPLE__)
#   define CV_EXPORTS __attribute__ ((visibility ("default")))
# endif
#endif

#ifndef CV_EXPORTS
# define CV_EXPORTS
#endif

#ifdef _MSC_VER
#define NOMINMAX
#define _CRT_SECURE_NO_WARNINGS
#pragma warning(push)
#pragma warning(disable: 4251)
#pragma warning(disable: 4996)
#endif

#define OPENCV_TRAITS_ENABLE_DEPRECATED

#include <opencv2/opencv.hpp>

// MP! Added: To correctly support imShow under WinRT.
#ifdef _WINRT_DLL
#ifdef HAVE_OPENCV_HIGHGUI
#include <opencv2/highgui/highgui_winrt.hpp>
#endif//HAVE_OPENCV_HIGHGUI
#endif
#include <opencv2/imgproc/imgproc_c.h>
#ifdef HAVE_OPENCV_SHAPE
#include <opencv2/shape.hpp>
#endif//HAVE_OPENCV_SHAPE
#include <opencv2/stitching.hpp>
#include <opencv2/video.hpp>
#ifndef _WINRT_DLL
#ifdef HAVE_OPENCV_SUPERRES
#include <opencv2/superres.hpp>
#include <opencv2/superres/optical_flow.hpp>
#endif//HAVE_OPENCV_SUPERRES
#endif

// opencv_contrib
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#ifdef HAVE_OPENCV_BGSEGM
#include <opencv2/bgsegm.hpp>
#endif//HAVE_OPENCV_BGSEGM
#ifdef HAVE_OPENCV_FACE
#include <opencv2/face.hpp>
#endif//HAVE_OPENCV_FACE
#ifdef HAVE_OPENCV_IMG_HASH
#include <opencv2/img_hash.hpp>
#endif//HAVE_OPENCV_IMG_HASH
#ifdef HAVE_OPENCV_OPTFLOW
#include <opencv2/optflow.hpp>
#endif//HAVE_OPENCV_OPTFLOW
#ifdef HAVE_OPENCV_TRACKING
#include <opencv2/tracking.hpp>
#endif//HAVE_OPENCV_TRACKING
#ifdef HAVE_OPENCV_XFEATURES2D
#include <opencv2/xfeatures2d.hpp>
#endif//HAVE_OPENCV_XFEATURES2D
#ifdef HAVE_OPENCV_XIMGPROC
#include <opencv2/ximgproc.hpp>
#endif//HAVE_OPENCV_XIMGPROC
#ifdef HAVE_OPENCV_XPHOTO
#include <opencv2/xphoto.hpp>
#endif//HAVE_OPENCV_XPHOTO
#ifdef HAVE_OPENCV_QUALITY
#include <opencv2/quality.hpp>
#endif//HAVE_OPENCV_QUALITY
#ifndef _WINRT_DLL
#ifdef HAVE_OPENCV_DNN
#include <opencv2/dnn.hpp>
#endif//HAVE_OPENCV_DNN
#ifdef HAVE_OPENCV_TEXT
#include <opencv2/text.hpp>
#endif//HAVE_OPENCV_TEXT
#endif

#include <string>
#include <vector>
#include <algorithm>
#include <iterator>
#include <sstream>
#include <iterator>
#include <fstream>
#include <iostream>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#ifdef _MSC_VER
#pragma warning(pop)
#endif

// Additional types
#include "my_types.h"

// Additional functions
#include "my_functions.h"

#endif
