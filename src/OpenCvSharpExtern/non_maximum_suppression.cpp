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

#include <stdio.h>

#include <opencv2/opencv.hpp>

#include "config.h"
#include "non_maximum_suppression.h"

namespace cbdetect {

void non_maximum_suppression(const cv::Mat& img, float tau, Corner& corners, const Params& params) 
{
    int margin, n;
    for (int rindex = 0; rindex < (int)params.radius.size(); rindex++)
    {
        margin = n = params.radius[rindex];

        cv::Mat choose_img = cv::Mat::zeros(img.size(), CV_8U);
        cv::parallel_for_(cv::Range(1, (int)std::floorf((img.rows - 2 * margin) / (n + 1)) + 1), [&](const cv::Range& range) -> void {
            for (int j = range.start * (n + 1) + margin - 1; j < range.end * (n + 1) + margin - 1; j += n + 1) {
                for (int i = n + margin; i < img.cols - n - margin; i += n + 1) {
                    int maxi = i, maxj = j;
                    float maxval = img.at<float>(j, i);

                    for (int j2 = j; j2 <= j + n; ++j2) {
                        for (int i2 = i; i2 <= i + n; ++i2) {
                            if (img.at<float>(j2, i2) > maxval) {
                                maxi = i2;
                                maxj = j2;
                                maxval = img.at<float>(j2, i2);
                            }
                        }
                    }

                    // maximum
                    for (int j2 = maxj - n; j2 <= std::min(maxj + n, img.rows - 1 - margin); ++j2) {
                        for (int i2 = maxi - n; i2 <= std::min(maxi + n, img.cols - 1 - margin); ++i2) {
                            if (img.at<float>(j2, i2) > maxval) {
                                goto GOTO_FAILED;
                            }
                        }
                    }

                    if (maxval > tau) {
                        choose_img.at<uint8_t>(maxj, maxi) = 1;
                    }
                GOTO_FAILED:;
                }
            }
        });

        for (int j = margin; j < img.rows - margin; ++j) {
            for (int i = margin; i < img.cols - margin; ++i) {
                if (choose_img.at<uint8_t>(j, i) == 1) {
                    corners.p.emplace_back(cv::Point2f(i, j));
                    corners.rindex.emplace_back(rindex);
                }
            }
        }
    }
}

void non_maximum_suppression_sparse(Corner& corners, int n, cv::Size img_size, const Params& params) {
  cv::Mat img_score = cv::Mat::zeros(img_size, CV_32F);
  cv::Mat used      = cv::Mat::ones(img_size, CV_32S) * -1;
  for(int i = 0; i < corners.p.size(); ++i) {
    int u = std::round(corners.p[i].x);
    int v = std::round(corners.p[i].y);
    if(img_score.at<float>(v, u) < corners.score[i]) {
      img_score.at<float>(v, u) = corners.score[i];
      used.at<int>(v, u)         = i;
    }
  }
  std::vector<cv::Point2f> corners_out_p, corners_out_v1, corners_out_v2;// , corners_out_v3;
  std::vector<float> corners_out_score;
  std::vector<int> corners_out_rindex;
  //bool is_monkey_saddle = params.corner_type == MonkeySaddlePoint;
  for(int i = 0; i < corners.p.size(); ++i) {
    int u        = std::round(corners.p[i].x);
    int v        = std::round(corners.p[i].y);
    float score = corners.score[i];
    if(used.at<int>(v, u) != i) {
      continue;
    }
    for(int j2 = v - n; j2 <= v + n; ++j2) {
      for(int i2 = u - n; i2 <= u + n; ++i2) {
        if(j2 < 0 || j2 >= img_size.height || i2 < 0 || i2 >= img_size.height) {
          continue;
        }
        if(img_score.at<float>(j2, i2) > score && (i2 != u || j2 != v)) {
          goto GOTO_FAILED;
        }
      }
    }
    corners_out_p.emplace_back(corners.p[i]);
    corners_out_rindex.emplace_back(corners.rindex[i]);
    corners_out_v1.emplace_back(corners.v1[i]);
    corners_out_v2.emplace_back(corners.v2[i]);
    //if(is_monkey_saddle) {
    //  corners_out_v3.emplace_back(corners.v3[i]);
    //}
    corners_out_score.emplace_back(corners.score[i]);
  GOTO_FAILED:;
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
