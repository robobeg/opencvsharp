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

#include <math.h>

#include <opencv2/core/hal/hal.hpp>
#include <opencv2/opencv.hpp>

#include "config.h"
#include "image_normalization_and_gradients.h"

namespace cbdetect {

void box_filter(const cv::Mat& img, cv::Mat& blur_img, int kernel_size_x, int kernel_size_y) {
  if(kernel_size_y < 0) {
    kernel_size_y = kernel_size_x;
  }
  blur_img.create(img.size(), CV_32F);
  std::vector<float> buf(img.cols, 0);
  std::vector<int> count_buf(img.cols, 0);
  int count = 0;
  for(int j = 0; j < std::min(kernel_size_y, img.rows - 1); ++j) {
    for(int i = 0; i < img.cols; ++i) {
      buf[i] += img.at<float>(j, i);
      ++count_buf[i];
    }
  }
  for(int j = 0; j < img.rows; ++j) {
    if(j > kernel_size_y) {
      for(int i = 0; i < img.cols; ++i) {
        buf[i] -= img.at<float>(j - kernel_size_y - 1, i);
        --count_buf[i];
      }
    }
    if(j + kernel_size_y < img.rows) {
      for(int i = 0; i < img.cols; ++i) {
        buf[i] += img.at<float>(j + kernel_size_y, i);
        ++count_buf[i];
      }
    }
    blur_img.at<float>(j, 0) = 0;
    count                     = 0;
    for(int i = 0; i <= std::min(kernel_size_x, img.cols - 1); ++i) {
      blur_img.at<float>(j, 0) += buf[i];
      count += count_buf[i];
    }
    for(int i = 1; i < img.cols; ++i) {
      blur_img.at<float>(j, i) = blur_img.at<float>(j, i - 1);
      blur_img.at<float>(j, i - 1) /= count;
      if(i > kernel_size_x) {
        blur_img.at<float>(j, i) -= buf[i - kernel_size_x - 1];
        count -= count_buf[i - kernel_size_x - 1];
      }
      if(i + kernel_size_x < img.cols) {
        blur_img.at<float>(j, i) += buf[i + kernel_size_x];
        count += count_buf[i + kernel_size_x];
      }
    }
    blur_img.at<float>(j, img.cols - 1) /= count;
  }
}

void image_normalization_and_gradients(cv::Mat& img, cv::Mat& img_du, cv::Mat& img_dv,
                                       cv::Mat& img_angle, cv::Mat& img_weight, const Params& params) {
  //normalize image
  if(params.norm) {
    cv::Mat blur_img;
    box_filter(img, blur_img, params.norm_half_kernel_size);
    img = img - blur_img;
    img = 2.5 * (cv::max(cv::min(img + 0.2, 0.4), 0));
  }

  // sobel masks
#if CV_VERSION_MAJOR == 4
  cv::Mat_<float> du({3, 3}, {1, 0, -1, 2, 0, -2, 1, 0, -1});
  cv::Mat_<float> dv({3, 3}, {1, 2, 1, 0, 0, 0, -1, -2, -1});
#else
  float du_array[9] = {1, 0, -1, 2, 0, -2, 1, 0, -1};
  float dv_array[9] = {1, 2, 1, 0, 0, 0, -1, -2, -1};
  cv::Mat du(3, 3, CV_32F, du_array);
  cv::Mat dv(3, 3, CV_32F, dv_array);
#endif

  // compute image derivatives (for principal axes estimation)
  cv::filter2D(img, img_du, -1, du, cv::Point(-1, -1), 0, cv::BORDER_REFLECT);
  cv::filter2D(img, img_dv, -1, dv, cv::Point(-1, -1), 0, cv::BORDER_REFLECT);
  img_angle.create(img.size(), img.type());
  img_weight.create(img.size(), img.type());
  if(!img_du.isContinuous()) {
    cv::Mat tmp = img_du.clone();
    std::swap(tmp, img_du);
  }
  if(!img_dv.isContinuous()) {
    cv::Mat tmp = img_dv.clone();
    std::swap(tmp, img_dv);
  }
  if(!img_angle.isContinuous()) {
    cv::Mat tmp = img_angle.clone();
    std::swap(tmp, img_angle);
  }
  if(!img_weight.isContinuous()) {
    cv::Mat tmp = img_weight.clone();
    std::swap(tmp, img_weight);
  }
  cv::hal::fastAtan32f((const float*)img_dv.data, (const float*)img_du.data,
                       (float*)img_angle.data, img.rows * img.cols, false);
  img_angle.forEach<float>([](float& pixel, const int* pos) -> void {
    pixel = pixel >= M_PI ? pixel - M_PI : pixel;
  });

  cv::hal::magnitude32f((const float*)img_du.data, (const float*)img_dv.data
      , (float*) img_weight.data, img.rows * img.cols);
  //img_weight.forEach<float>([&img_du, &img_dv](float& pixel, const int* pos) -> void {
  //  int u = pos[1];
  //  int v = pos[0];
  //  pixel = std::sqrt(
  //      img_du.at<float>(v, u) * img_du.at<float>(v, u) + img_dv.at<float>(v, u) * img_dv.at<float>(v, u));
  //});

  // scale input image
  double dimg_min = 0, dimg_max = 1;
  cv::minMaxLoc(img, &dimg_min, &dimg_max);//  float img_min = (float)dimg_min, img_max = (float)dimg_max;
  //img = (img - img_min) / (img_max - img_min);
  img.convertTo(img, -1, 1.0 / ( dimg_max - dimg_min), -dimg_min / ( dimg_max - dimg_min));
}

} // namespace cbdetect
