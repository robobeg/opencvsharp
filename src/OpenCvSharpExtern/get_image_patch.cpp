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

#include <opencv2/opencv.hpp>

#include "get_image_patch.h"

namespace cbdetect {

void get_image_patch(const cv::Mat& img, int iu, int iv, float du, float dv, int r, cv::Mat& img_sub) {
  //int iu     = u;
  //int iv     = v;
  //float du  = u - iu;
  //float dv  = v - iv;
  float a00 = 1 - du - dv + du * dv;
  float a01 = du - du * dv;
  float a10 = dv - du * dv;
  float a11 = du * dv;

  img_sub.create(2 * r + 1, 2 * r + 1, CV_32F);
  for(int j = -r; j <= r; ++j) {
    for(int i = -r; i <= r; ++i) {
      img_sub.at<float>(j + r, i + r) =
          a00 * img.at<float>(iv + j, iu + i) + a01 * img.at<float>(iv + j, iu + i + 1) +
          a10 * img.at<float>(iv + j + 1, iu + i) + a11 * img.at<float>(iv + j + 1, iu + i + 1);
    }
  }
}

void get_image_patch_with_mask(const cv::Mat& img, const cv::Mat& mask, int iu, int iv, float du, float dv, int r, cv::Mat& img_sub) {
  //int iu     = u;
  //int iv     = v;
  //float du  = u - iu;
  //float dv  = v - iv;
  float a00 = 1 - du - dv + du * dv;
  float a01 = du - du * dv;
  float a10 = dv - du * dv;
  float a11 = du * dv;

  img_sub.create((2 * r + 1) * (2 * r + 1), 1, CV_32F);
  int num = 0;
  for(int j = -r; j <= r; ++j) {
    for(int i = -r; i <= r; ++i) {
      if(mask.at<float>(j + r, i + r) >= 1e-6) {
        img_sub.at<float>(num, 0) =
            a00 * img.at<float>(iv + j, iu + i) + a01 * img.at<float>(iv + j, iu + i + 1) +
            a10 * img.at<float>(iv + j + 1, iu + i) + a11 * img.at<float>(iv + j + 1, iu + i + 1);
        ++num;
      }
    }
  }
  img_sub.resize(num);
}

} // namespace cbdetect
