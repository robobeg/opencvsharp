// c++ version by ftdlyc

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
#include <stdio.h>

#include <vector>

#include <opencv2/opencv.hpp>

#include "config.h"
#include "filter_corners.h"
#include "find_corners.h"
#include "get_init_location.h"
#include "image_normalization_and_gradients.h"
#include "non_maximum_suppression.h"
//#include "plot_corners.h"
#include "polynomial_fit.h"
#include "refine_corners.h"
#include "score_corners.h"

namespace cbdetect {

void find_corners_resized(cv::Mat& img_gray, Corner& corners, cv::Mat& img_gray_resized, Corner& corners_resized, const Params& params) {

    corners_resized.reset();


  // resize image
  float scale = 0;
  if(img_gray.rows < 640 || img_gray.cols < 480) 
  {
    scale = 2.0f;
  } 
  else 
  if(img_gray.rows >= 640 || img_gray.cols >= 480) 
  {
    scale = 0.5f;
  } 
  else
  {
    return;
  }

  cv::Size sizeResized = cv::Size(img_gray.cols * scale, img_gray.rows * scale);
  cv::resize(img_gray, img_gray_resized, sizeResized, 0, 0, cv::INTER_LINEAR);

  cv::Mat& img_norm = corners_resized.img_norm;
  img_gray_resized.convertTo(img_norm, CV_32F, 1 / 255.0, 0);

  // normalize image and calculate gradients
  cv::Mat& img_du = corners_resized.img_du;
  cv::Mat& img_dv = corners_resized.img_dv;
  cv::Mat& img_angle = corners_resized.img_angle;
  cv::Mat& img_weight = corners_resized.img_weight;
  image_normalization_and_gradients(img_norm, img_du, img_dv, img_angle, img_weight, params);


  // get corner's initial locaiton
  get_init_location(img_norm, img_du, img_dv, corners_resized, params);
  if(corners_resized.p.empty()) {
    return;
  }
  //if(params.show_processing) {
  //  printf("Initializing conres (%d x %d) ... %lu\n", img_norm.cols, img_norm.rows, corners_resized.p.size());
  //}
  //if(params.show_debug_image) {
  //  plot_corners(img_gray_resized, corners_resized.p, "init location resized");
  //}

  // pre-filter corners according to zero crossings
  filter_corners(img_norm, img_angle, img_weight, corners_resized, params);
  //if(params.show_processing) {
  //  printf("Filtering corners (%d x %d) ... %lu\n", img_norm.cols, img_norm.rows, corners_resized.p.size());
  //}
  //if(params.show_debug_image) {
  //  plot_corners(img_gray_resized, corners_resized.p, "filter corners resized");
  //}

  // refinement
  refine_corners(img_du, img_dv, img_angle, img_weight, corners_resized, params);
  //if(params.show_processing) {
  //  printf("Refining corners (%d x %d) ... %lu\n", img_norm.cols, img_norm.rows, corners_resized.p.size());
  //}
  //if(params.show_debug_image) {
  //  plot_corners(img_gray_resized, corners_resized.p, "refine corners resized");
  //}

  // merge corners
  cv::Size2f invScale((float) img_gray.cols /(float) sizeResized.width, (float) img_gray.rows / (float)sizeResized.height);
  std::for_each(corners_resized.p.begin(), corners_resized.p.end(), [&invScale](auto& p) { p.x *= invScale.width; p.y *= invScale.height; });
  // std::for_each(corners_resized.r.begin(), corners_resized.r.end(), [&scale](auto &r) { r = (float) r / scale; });
  float min_dist_thr = scale > 1 ? 3 : 5;
  for(int i = 0; i < corners_resized.p.size(); ++i) {
    float min_dist = FLT_MAX;
    cv::Point2f& p2 = corners_resized.p[i];
    for(int j = 0; j < corners.p.size(); ++j) {
      cv::Point2f& p1 = corners.p[j];
      float dist     = mynorm(p2 - p1);
      min_dist        = dist < min_dist ? dist : min_dist;
    }
    if(min_dist > min_dist_thr) {
      corners.p.emplace_back(corners_resized.p[i]);
      corners.r.emplace_back(corners_resized.r[i]);
      corners.v1.emplace_back(corners_resized.v1[i]);
      corners.v2.emplace_back(corners_resized.v2[i]);
      //if(params.corner_type == MonkeySaddlePoint) {
      //  corners.v3.emplace_back(corners_resized.v3[i]);
      //}
    }
  }
}

void find_corners(cv::Mat& img_gray, Corner& corners, cv::Mat& img_gray_resized, Corner& corners_resized, const Params& params) {
  // clear old data

    corners.reset();

  cv::Mat& img_norm = corners.img_norm;

    img_gray.copyTo(img_norm);
    img_norm.convertTo(img_norm, CV_32F, 1. / 255., 0);


  // normalize image and calculate gradients
    cv::Mat& img_du = corners.img_du;
    cv::Mat& img_dv = corners.img_dv;
    cv::Mat& img_angle = corners.img_angle;
    cv::Mat& img_weight = corners.img_weight;

  image_normalization_and_gradients(img_norm, img_du, img_dv, img_angle, img_weight, params);


  // get corner's initial locaiton
  get_init_location(img_norm, img_du, img_dv, corners, params);
  if(corners.p.empty()) {
    return;
  }
  //if(params.show_processing) {
  //  printf("Initializing conres (%d x %d) ... %lu\n", img_norm.cols, img_norm.rows, corners.p.size());
  //}
  //if(params.show_debug_image) {
  //  plot_corners(img_gray, corners.p, "init location");
  //}

  // pre-filter corners according to zero crossings
  filter_corners(img_norm, img_angle, img_weight, corners, params);
  //if(params.show_processing) {
  //  printf("Filtering corners (%d x %d) ... %lu\n", img_norm.cols, img_norm.rows, corners.p.size());
  //}
  //if(params.show_debug_image) {
  //  plot_corners(img_gray, corners.p, "filter corners");
  //}

  // refinement
  refine_corners(img_du, img_dv, img_angle, img_weight, corners, params);
  //if(params.show_processing) {
  //  printf("Refining corners (%d x %d) ... %lu\n", img_norm.cols, img_norm.rows, corners.p.size());
  //}
  //if(params.show_debug_image) {
  //  plot_corners(img_gray, corners.p, "refine corners");
  //}

  // resize image to detect more corners
  find_corners_resized(img_gray, corners, img_gray_resized, corners_resized, params);
  //if(params.show_processing) {
  //  printf("Merging corners (%d x %d) ... %lu\n", img_norm.cols, img_norm.rows, corners.p.size());
  //}
  //if(params.show_debug_image) {
  //  plot_corners(img_gray, corners.p, "merge corners");
  //}

  // polynomial fit
  if(params.polynomial_fit) {
    polynomial_fit(img_norm, corners, params);
    //if(params.show_processing) {
    //  printf("Polyfitting corners (%d x %d) ... %lu\n", img_norm.cols, img_norm.rows, corners.p.size());
    //}
    //if(params.show_debug_image) {
    //  plot_corners(img_gray, corners.p, "polynomial fit corners");
    //}
  }

  // score corners
  sorce_corners(img_norm, img_weight, corners, params);

  // remove low scoring corners
  remove_low_scoring_corners(params.score_thr, corners, params);

  // non maximum suppression
  non_maximum_suppression_sparse(corners, 3, img_gray.size(), params);
  //if(params.show_processing) {
  //  printf("Scoring corners (%d x %d) ... %lu\n", img_norm.cols, img_norm.rows, corners.p.size());
  //}
  //if(params.show_debug_image) {
  //  plot_corners(img_gray, corners.p, "scoring corners");
  //}
}

} // namespace cbdetect
