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

#include <vector>

#include <opencv2/opencv.hpp>

#include "score_corners.h"
#include "config.h"
#include "create_correlation_patch.h"
#include "find_corners.h"
#include "get_image_patch.h"
//#include "weight_mask.h"

namespace cbdetect {

float corner_correlation_score(const cv::Mat& img, const cv::Mat& img_weight,
                                const cv::Point2f& v1, const cv::Point2f& v2) {
  // compute gradient filter kernel (bandwith = 3 px)
  float center      = (img.cols - 1) / 2;
  cv::Mat img_filter = cv::Mat::ones(img.size(), CV_32F) * -1;
  for(int u = 0; u < img.cols; ++u) {
    for(int v = 0; v < img.rows; ++v) {
      cv::Point2f p1{u - center, v - center};
      cv::Point2f p2{(p1.x * v1.x + p1.y * v1.y) * v1.x, (p1.x * v1.x + p1.y * v1.y) * v1.y};
      cv::Point2f p3{(p1.x * v2.x + p1.y * v2.y) * v2.x, (p1.x * v2.x + p1.y * v2.y) * v2.y};
      if(mynorm(p1 - p2) <= 1.5 || mynorm(p1 - p3) <= 1.5) {
        img_filter.at<float>(v, u) = 1;
      }
    }
  }

  // normalize
  cv::Scalar mean, std;
  cv::meanStdDev(img_filter, mean, std);
  img_filter = (img_filter - mean[0]) / std[0];
  cv::meanStdDev(img_weight, mean, std);
  cv::Mat img_weight_norm = (img_weight - mean[0]) / std[0];

  // compute gradient score
  float score_gradient = (float) cv::sum(img_weight_norm.mul(img_filter))[0];
  score_gradient        = std::max(score_gradient / (img.cols * img.rows - 1), 0.f);

  // create intensity filter kernel
  std::vector<cv::Mat> template_kernel(4); // a1, a2, b1, b2
  create_correlation_patch(template_kernel, std::atan2(v1.y, v1.x), std::atan2(v2.y, v2.x), (img.cols - 1) / 2);

  // checkerboard responses
  float a1 = (float) cv::sum(img.mul(template_kernel[0]))[0];
  float a2 = (float)cv::sum(img.mul(template_kernel[1]))[0];
  float b1 = (float)cv::sum(img.mul(template_kernel[2]))[0];
  float b2 = (float)cv::sum(img.mul(template_kernel[3]))[0];

  // mean
  float mu = (a1 + a2 + b1 + b2) / 4;

  // case 1: a=white, b=black
  float s1 = std::min(std::min(a1, a2) - mu, mu - std::min(b1, b2));

  // case 2: b=white, a=black
  float s2 = std::min(mu - std::min(a1, a2), std::min(b1, b2) - mu);

  // intensity score: max. of the 2 cases
  float score_intensity = std::max(std::max(s1, s2), 0.f);

  // final score: product of gradient and intensity score
  return score_gradient * score_intensity;
}

float corner_correlation_score(const cv::Mat& img, const cv::Mat& img_weight,
                                const cv::Point2f& v1, const cv::Point2f& v2, const cv::Point2f& v3) {
  // compute gradient filter kernel (bandwith = 3 px)
  float center      = (img.cols - 1) / 2;
  cv::Mat img_filter = cv::Mat::ones(img.size(), CV_32F) * -1;
  for(int u = 0; u < img.cols; ++u) {
    for(int v = 0; v < img.rows; ++v) {
      cv::Point2f p1{u - center, v - center};
      cv::Point2f p2{(p1.x * v1.x + p1.y * v1.y) * v1.x, (p1.x * v1.x + p1.y * v1.y) * v1.y};
      cv::Point2f p3{(p1.x * v2.x + p1.y * v2.y) * v2.x, (p1.x * v2.x + p1.y * v2.y) * v2.y};
      cv::Point2f p4{(p1.x * v3.x + p1.y * v3.y) * v3.x, (p1.x * v3.x + p1.y * v3.y) * v3.y};
      if(mynorm(p1 - p2) <= 1.5 || mynorm(p1 - p3) <= 1.5 || mynorm(p1 - p4) <= 1.5) {
        img_filter.at<float>(v, u) = 1;
      }
    }
  }

  // normalize
  cv::Scalar mean, std;
  cv::meanStdDev(img_filter, mean, std);
  img_filter = (img_filter - mean[0]) / std[0];
  cv::meanStdDev(img_weight, mean, std);
  cv::Mat img_weight_norm = (img_weight - mean[0]) / std[0];

  // compute gradient score
  float score_gradient = (float)cv::sum(img_weight_norm.mul(img_filter))[0];
  score_gradient        = std::max(score_gradient / (img.cols * img.rows - 1), 0.f);

  // create intensity filter kernel
  std::vector<cv::Mat> template_kernel(6); // a1, a2, a3, b1, b2, b3
  create_correlation_patch(template_kernel,
                           std::atan2(v1.y, v1.x), std::atan2(v2.y, v2.x), std::atan2(v3.y, v3.x), (img.cols - 1) / 2);

  // checkerboard responses
  float a1 = (float)cv::sum(img.mul(template_kernel[0]))[0];
  float a2 = (float)cv::sum(img.mul(template_kernel[1]))[0];
  float a3 = (float)cv::sum(img.mul(template_kernel[2]))[0];
  float b1 = (float)cv::sum(img.mul(template_kernel[3]))[0];
  float b2 = (float)cv::sum(img.mul(template_kernel[4]))[0];
  float b3 = (float)cv::sum(img.mul(template_kernel[5]))[0];

  // mean
  float mu    = (a1 + a2 + a3 + b1 + b2 + b3) / 6;
  float min_a = std::min(std::min(a1, a2), a3);
  float min_b = std::min(std::min(b1, b2), b3);

  // case 1: a=white, b=black
  float s1 = std::min(min_a - mu, mu - min_b);

  // case 2: b=white, a=black
  float s2 = std::min(mu - min_a, min_b - mu);

  // intensity score: max. of the 2 cases
  float score_intensity = std::max(std::max(s1, s2), 0.f);

  // final score: product of gradient and intensity score
  return score_gradient * score_intensity;
}
void sorce_corners(const cv::Mat& img, const cv::Mat& img_weight, Corner& corners, const Params& params) {
  corners.score.resize(corners.p.size());
  int width = img.cols, height = img.rows;
  //auto mask = weight_mask(params.radius);

  // for all corners do
  cv::parallel_for_(cv::Range(0, corners.p.size()), [&](const cv::Range& range) -> void {
    for(int i = range.start; i < range.end; ++i) {
      // corner location
      float u = corners.p[i].x;
      float v = corners.p[i].y;
      int rindex = corners.rindex[i];
      int r    = params.radius[rindex];
      int iu = (int)std::floor(u);
      int iv = (int)std::floor(v);

      if(iu - r < 0 || iu + r >= width - 1 || iv - r < 0 || iv + r >= height - 1) {
        corners.score[i] = 0.;
        continue;
      }
      cv::Mat img_sub, img_weight_sub;
      get_image_patch(img, iu, iv, u-iu, v-iv, r, img_sub);
      get_image_patch(img_weight, iu, iv, u-iu, v-iv, r, img_weight_sub);
      img_weight_sub = img_weight_sub.mul(params.weight_mask[rindex]);
      //if(params.corner_type == SaddlePoint) {
        corners.score[i] = corner_correlation_score(img_sub, img_weight_sub, corners.v1[i], corners.v2[i]);
      //} else if(params.corner_type == MonkeySaddlePoint) {
      //  corners.score[i] =
      //      corner_correlation_score(img_sub, img_weight_sub, corners.v1[i], corners.v2[i], corners.v3[i]);
      //}
    }
  });
}

void remove_low_scoring_corners(float tau, Corner& corners, const Params& params) {
    std::vector<cv::Point2f> corners_out_p, corners_out_v1, corners_out_v2;// , corners_out_v3;
  std::vector<float> corners_out_score;
  std::vector<int> corners_out_rindex;
  //bool is_monkey_saddle = params.corner_type == MonkeySaddlePoint;
  for(int i = 0; i < corners.p.size(); ++i) {
    if(corners.score[i] > tau) {
      corners_out_p.emplace_back(corners.p[i]);
      corners_out_rindex.emplace_back(corners.rindex[i]);
      corners_out_v1.emplace_back(corners.v1[i]);
      corners_out_v2.emplace_back(corners.v2[i]);
      //if(is_monkey_saddle) {
      //  corners_out_v3.emplace_back(corners.v3[i]);
      //}
      corners_out_score.emplace_back(corners.score[i]);
    }
  }
  corners.p  = std::move(corners_out_p);
  corners.rindex  = std::move(corners_out_rindex);
  corners.v1 = std::move(corners_out_v1);
  corners.v2 = std::move(corners_out_v2);
  //if(is_monkey_saddle) {
  //  corners.v3 = std::move(corners_out_v3);
  //}
  corners.score = std::move(corners_out_score);
}

} // namespace cbdetect
