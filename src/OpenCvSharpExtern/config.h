/**
* Copyright 2018, ftdlyc <yclu.cn@gmail.com>
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 3 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, see <http://www.gnu.org/licenses/>.
*/

/*
% Copyright 2012. All rights reserved.
% Author: Andreas Geiger
%         Institute of Measurement and Control Systems (MRT)
%         Karlsruhe Institute of Technology (KIT), Germany

% This is free software; you can redistribute it and/or modify it under the
% terms of the GNU General Public License as published by the Free Software
% Foundation; either version 3 of the License, or any later version.

% This software is distributed in the hope that it will be useful, but WITHOUT ANY
% WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
% PARTICULAR PURPOSE. See the GNU General Public License for more details.

% You should have received a copy of the GNU General Public License along with
% this software; if not, write to the Free Software Foundation, Inc., 51 Franklin
% Street, Fifth Floor, Boston, MA 02110-1301, USA
*/

#pragma once
#ifndef LIBCBDETECT_CONFIG_H
#define LIBCBDETECT_CONFIG_H

#include <vector>
#include <cmath>

#include <opencv2/opencv.hpp>

#if CV_VERSION_MAJOR == 3 && CV_VERSION_MINOR <= 2
#include <functional>
namespace cv {
class ParallelLoopBodyLambdaWrapper : public ParallelLoopBody {
private:
  std::function<void(const Range&)> m_functor;

public:
  ParallelLoopBodyLambdaWrapper(std::function<void(const Range&)> functor)
      : m_functor(functor) {}

  virtual void operator()(const cv::Range& range) const {
    m_functor(range);
  }
};

inline void parallel_for_(const Range& range, std::function<void(const Range&)> functor, float nstripes = -1.) {
  parallel_for_(range, ParallelLoopBodyLambdaWrapper(functor), nstripes);
}
} // namespace cv
#endif

#ifdef _MSC_VER
#define M_PI 3.14159265358979323846f   /* pi */
#define M_PI_2 1.57079632679489661923f /* pi/2 */
#define M_PI_4 0.78539816339744830962f /* pi/4 */
#define M_1_PI 0.31830988618379067154f /* 1/pi */
#define M_2_PI 0.63661977236758134308f /* 2/pi */
#endif

#ifndef LIBCBDETECT_DLL_DECL
#if IS_A_DLL && defined(_MSC_VER)
#define LIBCBDETECT_DLL_DECL __declspec(dllexport)
#else
#define LIBCBDETECT_DLL_DECL
#endif
#endif

namespace cbdetect {

enum DetectMethod {
  // origin method fast mode
  TemplateMatchFast = 0,
  // origin method slow mode
  TemplateMatchSlow,

  // compute hessian of image, detect by a threshold
  // form https://github.com/facebookincubator/deltille
  HessianResponse,

  // paper: Accurate Detection and Localization of Checkerboard Corners for Calibration
  LocalizedRadonTransform
};

//enum CornerType {
//  SaddlePoint = 0,
//  MonkeySaddlePoint
//};

typedef struct Params {
  //bool show_processing;
  //bool show_debug_image;
  //bool show_grow_processing;
  bool refine_location;
  bool norm;
  bool polynomial_fit;
  int norm_half_kernel_size;
  int polynomial_fit_half_kernel_size;
  float init_loc_thr;
  float score_thr;
  bool strict_grow;
  bool overlay;
  bool occlusion;
  DetectMethod detect_method;
  //CornerType corner_type;
  std::vector<int> radius;

  Params()
      //: show_processing(true)
      //, show_debug_image(false)
      //, show_grow_processing(false)
      : refine_location(true)
      , norm(false)
      , polynomial_fit(true)
      , norm_half_kernel_size(31)
      , polynomial_fit_half_kernel_size(4)
      , init_loc_thr(0.01f)
      , score_thr(0.01f)
      , strict_grow(true)
      , overlay(false)
      , occlusion(true)
      , detect_method(HessianResponse)
      //, corner_type(SaddlePoint)
      , radius({5, 7}) {}
} Params;

typedef struct Corner {
  std::vector<cv::Point2f> p;
  std::vector<int> r;
  std::vector<cv::Point2f> v1;
  std::vector<cv::Point2f> v2;
  //std::vector<cv::Point2f> v3;
  std::vector<float> score;

  cv::Mat img_norm;
  cv::Mat img_du;
  cv::Mat img_dv;
  cv::Mat img_angle;
  cv::Mat img_weight;

  void reset()
  {
      p.clear();
      r.clear();
      v1.clear();
      v2.clear();
      score.clear();
  }

} Corner;

enum CornerType {
  SaddlePoint = 0,
  MonkeySaddlePoint
};

enum CheckerType {
  CheckerInvalid = 0,
  CheckerWhite = 1,
  CheckerBlack = 2,
  CheckerBlackWhiteBit = 3,
  CheckerUndetermined = 4,
};

typedef struct BoardElm
{
    int idx;
    CheckerType type;
    cv::Vec3f energy;

    BoardElm()
        : idx(-1)
        , type(CheckerInvalid)
        , energy(FLT_MAX, FLT_MAX, FLT_MAX)
    {
    }

    void reset()
    {
        idx = -1;
        type = CheckerInvalid;
        energy = cv::Vec3f(FLT_MAX, FLT_MAX, FLT_MAX);
    }
} BoardElm;

enum BoardDirection {
    BoardDirectionTop = 0,
    BoardDirectionLeft = 1,
    BoardDirectionBottom = 2,
    BoardDirectionRight = 3,

};

enum BoardType {
    BoardTypeInvalid = 0,
    BoardTypeBlack = 1,
    BoardTypeWhite = 2,
};

typedef struct Board {

private:
  int m_rows;
  int m_cols;
  BoardType m_eBoardType;
  std::vector< BoardElm> m_data;
  int m_num;

private:
    int calc_index(int r, int c) const { return m_cols * r + c; }
    int calc_index(const cv::Point2i& i) const { return m_cols * i.y + i.x; }
    BoardElm& access_data(int r, int c) {
        return m_data[calc_index(r, c)];
    }
    BoardElm& access_data(const cv::Point2i& i) {
        return m_data[calc_index(i)];
    }
    const BoardElm& get_data(int r, int c) const {
        return m_data[calc_index(r, c)];
    }
    const BoardElm& get_data(const cv::Point2i& i) const {
        return m_data[calc_index(i)];
    }

public:

    int rows() const { return m_rows; }
    int cols() const { return m_cols; }

    int& idx(int r, int c) {
        return access_data(r, c).idx;
    }
    int idx(int r, int c) const {
        return get_data(r, c).idx;
    }
    int& idx(const cv::Point2i& i) {
        return access_data(i).idx;
    }
    int idx(const cv::Point2i& i) const {
        return get_data(i).idx;
    }
    CheckerType& type(int r, int c) {
        return access_data(r, c).type;
    }
    CheckerType type(int r, int c) const {
        return get_data(r, c).type;
    }
    CheckerType& type(const cv::Point2i& i) {
        return access_data(i).type;
    }
    CheckerType type(const cv::Point2i& i) const {
        return get_data(i).type;
    }
    cv::Vec3f& energy(int r, int c) {
        return access_data(r, c).energy;
    }
    const cv::Vec3f& energy(int r, int c) const {
        return get_data(r, c).energy;
    }
    cv::Vec3f& energy(const cv::Point2i& i) {
        return access_data(i).energy;
    }
    const cv::Vec3f& energy(const cv::Point2i& i) const {
        return get_data(i).energy;
    }

    int num() const { return m_num; }
    void set_num(int newnum) { m_num = newnum; }
    BoardType getBoardType() const { return m_eBoardType; }
    void setBoardType(BoardType eBoardType) { m_eBoardType = eBoardType; }


    Board()
        : m_num(0)
        , m_eBoardType(BoardTypeInvalid)
        , m_rows(0)
        , m_cols(0)
        , m_data()
    {
    }
    
    Board(const Board& rhs)
        : m_rows(rhs.m_rows)
        , m_cols(rhs.m_cols)
        , m_data(rhs.m_data)
        , m_num(rhs.m_num)
        , m_eBoardType(rhs.m_eBoardType)
    {
    }

    Board(Board&& rhs)
        : m_rows(rhs.m_rows)
        , m_cols(rhs.m_cols)
        , m_data(std::move(rhs.m_data))
        , m_num(rhs.m_num)
        , m_eBoardType(rhs.m_eBoardType)
    {
    }

    Board& operator = (const Board& rhs)
    {
        if (this != &rhs)
        {
            m_rows = rhs.m_rows;
            m_cols = rhs.m_cols;
            m_data = rhs.m_data;
            m_num = rhs.m_num;
            m_eBoardType = rhs.m_eBoardType;
        }
        return *this;
    }

    Board& operator = (Board&& rhs)
    {
        if (this != &rhs)
        {
            m_rows = rhs.m_rows;
            m_cols = rhs.m_cols;
            m_data.swap(rhs.m_data);
            m_num = rhs.m_num;
            m_eBoardType = rhs.m_eBoardType;
        }
        return *this;
    }

    void reset()
    {
        m_rows = 0;
        m_cols = 0;
        m_data.clear();
        m_num = 0;
        m_eBoardType = BoardTypeInvalid;
    }

    void set_size(int rows, int cols);

    void add_board_boundary(BoardDirection eDir);

} Board;

inline float mynorm(const cv::Point2f& a)
{
    return std::sqrtf(a.x * a.x + a.y * a.y);
}

} // namespace cbdetect



#endif //LIBCBDETECT_CONFIG_H
