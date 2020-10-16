#ifndef _CPP_HIGHGUI_H_
#define _CPP_HIGHGUI_H_

// ReSharper disable IdentifierTypo
// ReSharper disable CppInconsistentNaming
// ReSharper disable CppNonInlineFunctionDefinitionInHeaderFile

#include "include_opencv.h"


namespace cv
{
    /** @brief Callback function for mouse events. see cv::setMouseCallback
    @param event one of the cv::MouseEventTypes constants.
    @param x The x-coordinate of the mouse event.
    @param y The y-coordinate of the mouse event.
    @param flags one of the cv::MouseEventFlags constants.
    @param userdata The optional parameter.
     */
    typedef void (*MouseCallback)(int event, int x, int y, int flags, void* userdata);

    /** @brief Callback function for Trackbar see cv::createTrackbar
    @param pos current position of the specified trackbar.
    @param userdata The optional parameter.
     */
    typedef void (*TrackbarCallback)(int pos, void* userdata);

    /** @brief Callback function defined to be called every frame. See cv::setOpenGlDrawCallback
    @param userdata The optional parameter.
     */
    typedef void (*OpenGlDrawCallback)(void* userdata);

    /** @brief Callback function for a button created by cv::createButton
    @param state current state of the button. It could be -1 for a push button, 0 or 1 for a check/radio box button.
    @param userdata The optional parameter.
     */
    typedef void (*ButtonCallback)(int state, void* userdata);
}

CVAPI(ExceptionStatus) highgui_namedWindow(const char *winname, int flags)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_HIGHGUI
    cv::namedWindow(winname, flags);
#endif//HAVE_OPENCV_HIGHGUI
    END_WRAP
}

CVAPI(ExceptionStatus) highgui_destroyWindow(const char *winName)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_HIGHGUI
    cv::destroyWindow(winName);
#endif//HAVE_OPENCV_HIGHGUI
    END_WRAP
}

CVAPI(ExceptionStatus) highgui_destroyAllWindows()
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_HIGHGUI
    cv::destroyAllWindows();
#endif//HAVE_OPENCV_HIGHGUI
    END_WRAP
}

CVAPI(ExceptionStatus) highgui_startWindowThread(int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_HIGHGUI
    *returnValue = cv::startWindowThread();
#endif//HAVE_OPENCV_HIGHGUI
    END_WRAP
}

CVAPI(ExceptionStatus) highgui_waitKeyEx(int delay, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_HIGHGUI
    *returnValue = cv::waitKeyEx(delay);
#endif//HAVE_OPENCV_HIGHGUI
    END_WRAP
}

CVAPI(ExceptionStatus) highgui_waitKey(int delay, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_HIGHGUI
    *returnValue = cv::waitKey(delay);
#endif//HAVE_OPENCV_HIGHGUI
    END_WRAP
}
  

CVAPI(ExceptionStatus) highgui_imshow(const char *winname, cv::Mat *mat)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_HIGHGUI
    cv::imshow(winname, *mat);
#endif//HAVE_OPENCV_HIGHGUI
    END_WRAP
}

CVAPI(ExceptionStatus) highgui_resizeWindow(const char *winName, int width, int height)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_HIGHGUI
    cv::resizeWindow(winName, width, height);
#endif//HAVE_OPENCV_HIGHGUI
    END_WRAP
}

CVAPI(ExceptionStatus) highgui_moveWindow(const char *winName, int x, int y)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_HIGHGUI
    cv::moveWindow(winName, x, y);
#endif//HAVE_OPENCV_HIGHGUI
    END_WRAP
}

CVAPI(ExceptionStatus) highgui_setWindowProperty(const char *winName, int propId, double propValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_HIGHGUI
    cv::setWindowProperty(winName, propId, propValue);
#endif//HAVE_OPENCV_HIGHGUI
    END_WRAP
}

CVAPI(ExceptionStatus) highgui_setWindowTitle(const char *winname, const char *title)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_HIGHGUI
    // TODO Resolve:
#ifndef _WINRT_DLL
    cv::setWindowTitle(winname, title);
#endif
#endif//HAVE_OPENCV_HIGHGUI
    END_WRAP
}

CVAPI(ExceptionStatus) highgui_getWindowProperty(const char *winName, int propId, double *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_HIGHGUI
    *returnValue = cv::getWindowProperty(winName, propId);
#endif//HAVE_OPENCV_HIGHGUI
    END_WRAP
}

CVAPI(ExceptionStatus) highgui_getWindowImageRect(const char *winName, MyCvRect *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_HIGHGUI
    *returnValue = c(cv::getWindowImageRect(winName));
#endif//HAVE_OPENCV_HIGHGUI
    END_WRAP
}

CVAPI(ExceptionStatus) highgui_setMouseCallback(const char *winName, cv::MouseCallback onMouse, void* userData)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_HIGHGUI
    cv::setMouseCallback(winName, onMouse, userData);
#endif//HAVE_OPENCV_HIGHGUI
    END_WRAP
}

CVAPI(ExceptionStatus) highgui_getMouseWheelDelta(int flags, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_HIGHGUI
    *returnValue = cv::getMouseWheelDelta(flags);
#endif//HAVE_OPENCV_HIGHGUI
    END_WRAP
}

CVAPI(ExceptionStatus) highgui_selectROI1(const char *windowName, cv::_InputArray *img, int showCrosshair, int fromCenter, MyCvRect *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_HIGHGUI
    *returnValue = c(cv::selectROI(windowName, *img, showCrosshair != 0, fromCenter != 0));
#endif//HAVE_OPENCV_HIGHGUI
    END_WRAP
}

CVAPI(ExceptionStatus) highgui_selectROI2(cv::_InputArray *img, int showCrosshair, int fromCenter, MyCvRect *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_HIGHGUI
    *returnValue = c(cv::selectROI(*img, showCrosshair != 0, fromCenter != 0));
#endif//HAVE_OPENCV_HIGHGUI
    END_WRAP
}

CVAPI(ExceptionStatus) highgui_selectROIs(const char * windowName, cv::_InputArray *img,
                             std::vector<cv::Rect> *boundingBoxes, int showCrosshair, int fromCenter)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_HIGHGUI
    cv::selectROIs(windowName, *img, *boundingBoxes, showCrosshair != 0, fromCenter != 0);
#endif//HAVE_OPENCV_HIGHGUI
    END_WRAP
}

CVAPI(ExceptionStatus) highgui_createTrackbar(const char *trackbarName, const char *winName,
    int* value, int count, cv::TrackbarCallback onChange, void* userData, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_HIGHGUI
    *returnValue = cv::createTrackbar(trackbarName, winName, value, count, onChange, userData);
#endif//HAVE_OPENCV_HIGHGUI
    END_WRAP
}
CVAPI(ExceptionStatus) highgui_getTrackbarPos(const char *trackbarName, const char *winName, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_HIGHGUI
    *returnValue = cv::getTrackbarPos(trackbarName, winName);
#endif//HAVE_OPENCV_HIGHGUI
    END_WRAP  
}
CVAPI(ExceptionStatus) highgui_setTrackbarPos(const char *trackbarName, const char *winName, int pos)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_HIGHGUI
    cv::setTrackbarPos(trackbarName, winName, pos);
#endif//HAVE_OPENCV_HIGHGUI
    END_WRAP
}

CVAPI(ExceptionStatus) highgui_setTrackbarMax(const char *trackbarName, const char *winName, int maxVal)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_HIGHGUI
    cv::setTrackbarMax(trackbarName, winName, maxVal);
#endif//HAVE_OPENCV_HIGHGUI
    END_WRAP
}
CVAPI(ExceptionStatus) highgui_setTrackbarMin(const char *trackbarName, const char *winName, int minVal)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_HIGHGUI
    cv::setTrackbarMin(trackbarName, winName, minVal);
#endif//HAVE_OPENCV_HIGHGUI
    END_WRAP
}

/*CVAPI(ExceptionStatus) highgui_createButton(const char *bar_name, cv::ButtonCallback on_change,
    void* user_data, int type, int initial_button_state, int *returnValue)
{
    BEGIN_WRAP
    *returnValue = cv::createButton(bar_name, on_change, user_data, type, initial_button_state != 0);
    END_WRAP
}*/

#ifdef _WINRT_DLL
CVAPI(ExceptionStatus) highgui_initContainer(::Windows::UI::Xaml::Controls::Panel^ panel)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_HIGHGUI
    cv::winrt_initContainer(panel);
#endif//HAVE_OPENCV_HIGHGUI
    END_WRAP
}
#endif

#endif