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

#include "config.h"
#include "grow_board.h"

namespace cbdetect {

// linear prediction (old)
// function pred = predict_corners(p1,p2,p3)
// pred = 2*p3-p2;
//
// replica prediction (new)
std::vector<cv::Point2f> predict_corners(const Corner& corners,
                                         const std::vector<int>& p1,
                                         const std::vector<int>& p2,
                                         const std::vector<int>& p3) {
  std::vector<cv::Point2f> pred(p3.size());
  if(p1.empty()) {
    for(int i = 0; i < pred.size(); ++i) {
      pred[i] = 2 * corners.p[p3[i]] - corners.p[p2[i]];
    }
  } else {
    for(int i = 0; i < pred.size(); ++i) {
      // compute vectors
      cv::Point2f v1 = corners.p[p2[i]] - corners.p[p1[i]];
      cv::Point2f v2 = corners.p[p3[i]] - corners.p[p2[i]];

      // predict angles
      float a1 = std::atan2(v1.y, v1.x);
      float a2 = std::atan2(v2.y, v2.x);
      float a3 = 2 * a2 - a1;

      //  predict scales
      float s1 = mynorm(v1);
      float s2 = mynorm(v2);
      float s3 = 2 * s2 - s1;

      // predict p4 (the factor 0.75 ensures that under extreme
      // distortions (omnicam) the closer prediction is selected)
      pred[i].x = corners.p[p3[i]].x + 0.75f * s3 * std::cosf(a3);
      pred[i].y = corners.p[p3[i]].y + 0.75f * s3 * std::sinf(a3);
    }
  }
  return pred;
}

std::vector<int> predict_board_corners(const Corner& corners,
                                       std::vector<int>& used,
                                       std::vector<int>& p1,
                                       std::vector<int>& p2,
                                       std::vector<int>& p3) {
  std::vector<cv::Point2f> pred = predict_corners(corners, p1, p2, p3);
  std::vector<int> pred_idx(pred.size(), -2);

  // build distance matrix
  std::vector<std::vector<float>> D(pred.size(), std::vector<float>(corners.p.size(), FLT_MAX));
  for(int i = 0; i < pred.size(); ++i) {
    cv::Point2f w = pred[i] - corners.p[p3[i]];
    //    float angle_w = std::atan2(w.y, w.x);
    for(int j = 0; j < corners.p.size(); ++j) {
      if(used[j] == 1) {
        continue;
      }
      cv::Point2f v_tmp = corners.p[j] - corners.p[p3[i]];
      cv::Point2f v(v_tmp.dot(w), v_tmp.dot(cv::Point2f(w.y, -w.x)));
      v         = v / (mynorm(w) * mynorm(w));
      float d1 = std::atan2(v.y, v.x);
      float d2 = (1 - mynorm(v));
      D[i][j]   = std::sqrt(std::abs(d1) + d2 * d2 * d2 * d2);
      //      cv::Point2f v = corners.p[j] - corners.p[p3[i]];
      //      float d_angle = angle_w - std::atan2(v.y, v.x);
      //      float d1 = 1 - mynorm(v) / mynorm(w);
      //      float d2 = std::abs(1 - 1 / (std::cos(d_angle) + 0.001));
      //      D[i][j] = d1 * d1 + 10 * d2;
    }
  }

  // search for closest corners
  for(int i = 0; i < pred.size(); ++i) {
    float min_D = FLT_MAX;
    int min_row  = 0;
    int min_col  = 0;
    for(int j = 0; j < pred.size(); ++j) {
      int min_row_2 = std::min_element(D[j].begin(), D[j].end()) - D[j].begin();
      if(D[j][min_row_2] < min_D) {
        min_D   = D[j][min_row_2];
        min_row = min_row_2;
        min_col = j;
      }
    }

    // all detect corners have been used
    if(FLT_MAX - min_D < 1e-6) {
      break;
    }

    for(auto& j : D[min_col]) {
      j = FLT_MAX;
    }
    for(int j = 0; j < pred.size(); ++j) {
      D[j][min_row] = FLT_MAX;
    }
    pred_idx[min_col] = min_row;
    used[min_row]     = 1;
  }

  return pred_idx;
}

//bool add_board_boundary(Board& board, int direction) {
//  int rows = board.idx.size(), cols = board.idx[0].size();
//  bool add_board = false;
//
//  // top/left/bottom//right
//  switch(direction) {
//  case 0: {
//    for(int i = 0; i < cols; ++i) {
//      if(board.idx[0][i] != -2 && board.idx[0][i] != -1) {
//        add_board = true;
//        break;
//      }
//    }
//    if(add_board) {
//      std::vector<int> idx(cols, -1);
//      std::vector<std::vector<float>> energy(cols, std::vector<float>(3, FLT_MAX));
//      board.idx.insert(board.idx.begin(), idx);
//      board.energy.insert(board.energy.begin(), energy);
//    }
//    break;
//  }
//  case 1: {
//    for(int i = 0; i < rows; ++i) {
//      if(board.idx[i][0] != -2 && board.idx[i][0] != -1) {
//        add_board = true;
//        break;
//      }
//    }
//    if(add_board) {
//      for(int i = 0; i < rows; ++i) {
//        board.idx[i].insert(board.idx[i].begin(), -1);
//        board.energy[i].insert(board.energy[i].begin(), std::vector<float>(3, FLT_MAX));
//      }
//    }
//    break;
//  }
//  case 2: {
//    for(int i = 0; i < cols; ++i) {
//      if(board.idx[rows - 1][i] != -2 && board.idx[rows - 1][i] != -1) {
//        add_board = true;
//        break;
//      }
//    }
//    if(add_board) {
//      std::vector<int> idx(cols, -1);
//      std::vector<std::vector<float>> energy(cols, std::vector<float>(3, FLT_MAX));
//      board.idx.emplace_back(idx);
//      board.energy.emplace_back(energy);
//    }
//    break;
//  }
//  case 3: {
//    for(int i = 0; i < rows; ++i) {
//      if(board.idx[i][cols - 1] != -2 && board.idx[i][cols - 1] != -1) {
//        add_board = true;
//        break;
//      }
//    }
//    if(add_board) {
//      for(int i = 0; i < rows; ++i) {
//        board.idx[i].emplace_back(-1);
//        board.energy[i].emplace_back(std::vector<float>(3, FLT_MAX));
//      }
//    }
//    break;
//  }
//  default:
//    break;
//  }
//
//  return add_board;
//}

bool add_board_boundary(Board& board, int direction) {
    int rows = board.rows(), cols = board.cols();
    bool add_board = false;

    // top/left/bottom//right
    switch (direction) {
    case 0: {
        for (int i = 0; i < cols; ++i) {
            if (board.idx(0,i) != -2 && board.idx(0,i) != -1) {
                add_board = true;
                break;
            }
        }
        if (add_board) {
            board.add_board_boundary(BoardDirectionTop);
        }
        break;
    }
    case 1: {
        for (int i = 0; i < rows; ++i) {
            if (board.idx(i,0) != -2 && board.idx(i,0) != -1) {
                add_board = true;
                break;
            }
        }
        if (add_board) {
            board.add_board_boundary(BoardDirectionLeft);
        }
        break;
    }
    case 2: {
        for (int i = 0; i < cols; ++i) {
            if (board.idx(rows - 1,i) != -2 && board.idx(rows - 1,i) != -1) {
                add_board = true;
                break;
            }
        }
        if (add_board) {
            board.add_board_boundary(BoardDirectionBottom);
        }
        break;
    }
    case 3: {
        for (int i = 0; i < rows; ++i) {
            if (board.idx(i,cols - 1) != -2 && board.idx(i,cols - 1) != -1) {
                add_board = true;
                break;
            }
        }
        if (add_board) {
            board.add_board_boundary(BoardDirectionRight);
        }
        break;
    }
    default:
        break;
    }

    return add_board;
}


GrowType grow_board(const Corner& corners, std::vector<int>& used, Board& board,
                    std::vector<cv::Point2i>& proposal, int direction, const Params& params) {
  // return immediately, if there do not exist any chessboards
  if(board.rows() == 0 || board.cols() == 0 ) {
    return GrowType_Failure;
  }
  int cols = board.cols();
  int rows = board.rows();
  std::vector<int> idx, p1, p2, p3, pred;

  // fill inside corners top/left/bottom/right/top-left/bottom-right
  if(/*params.corner_type == MonkeySaddlePoint ||*/params.occlusion) {
    switch(direction) {
    case 0: {
      for(int i = rows - 4; i >= 0; --i) {
        for(int j = 0; j < cols; ++j) {
          if(board.idx(i,j) != -1) {
            continue;
          }
          int idx1 = board.idx(i + 3,j);
          int idx2 = board.idx(i + 2,j);
          int idx3 = board.idx(i + 1,j);
          if(idx1 < 0 || idx2 < 0 || idx3 < 0) {
            continue;
          }
          p1.emplace_back(idx1);
          p2.emplace_back(idx2);
          p3.emplace_back(idx3);
          proposal.emplace_back(cv::Point2i(j, i));
        }
      }
      if(!proposal.empty() || params.strict_grow) {
        break;
      }
      for(int i = rows - 3; i >= 0; --i) {
        for(int j = 0; j < cols; ++j) {
          if(board.idx(i,j) != -1) {
            continue;
          }
          int idx2 = board.idx(i + 2,j);
          int idx3 = board.idx(i + 1,j);
          if(idx2 < 0 || idx3 < 0) {
            continue;
          }
          p2.emplace_back(idx2);
          p3.emplace_back(idx3);
          proposal.emplace_back(cv::Point2i(j, i));
        }
      }
      break;
    }
    case 1: {
      for(int i = 0; i < rows; ++i) {
        for(int j = cols - 4; j >= 0; --j) {
          if(board.idx(i,j) != -1) {
            continue;
          }
          int idx1 = board.idx(i,j + 3);
          int idx2 = board.idx(i,j + 2);
          int idx3 = board.idx(i,j + 1);
          if(idx1 < 0 || idx2 < 0 || idx3 < 0) {
            continue;
          }
          p1.emplace_back(idx1);
          p2.emplace_back(idx2);
          p3.emplace_back(idx3);
          proposal.emplace_back(cv::Point2i(j, i));
        }
      }
      if(!proposal.empty() || params.strict_grow) {
        break;
      }
      for(int i = 0; i < rows; ++i) {
        for(int j = cols - 3; j >= 0; --j) {
          if(board.idx(i,j) != -1) {
            continue;
          }
          int idx2 = board.idx(i,j + 2);
          int idx3 = board.idx(i,j + 1);
          if(idx2 < 0 || idx3 < 0) {
            continue;
          }
          p2.emplace_back(idx2);
          p3.emplace_back(idx3);
          proposal.emplace_back(cv::Point2i(j, i));
        }
      }
      break;
    }
    case 2: {
      for(int i = 3; i < rows; ++i) {
        for(int j = 0; j < cols; ++j) {
          if(board.idx(i,j) != -1) {
            continue;
          }
          int idx1 = board.idx(i - 3,j);
          int idx2 = board.idx(i - 2,j);
          int idx3 = board.idx(i - 1,j);
          if(idx1 < 0 || idx2 < 0 || idx3 < 0) {
            continue;
          }
          p1.emplace_back(idx1);
          p2.emplace_back(idx2);
          p3.emplace_back(idx3);
          proposal.emplace_back(cv::Point2i(j, i));
        }
      }
      if(!proposal.empty() || params.strict_grow) {
        break;
      }
      for(int i = 2; i < rows; ++i) {
        for(int j = 0; j < cols; ++j) {
          if(board.idx(i,j) != -1) {
            continue;
          }
          int idx2 = board.idx(i - 2,j);
          int idx3 = board.idx(i - 1,j);
          if(idx2 < 0 || idx3 < 0) {
            continue;
          }
          p2.emplace_back(idx2);
          p3.emplace_back(idx3);
          proposal.emplace_back(cv::Point2i(j, i));
        }
      }
      break;
    }
    case 3: {
      for(int i = 0; i < rows; ++i) {
        for(int j = 3; j < cols; ++j) {
          if(board.idx(i,j) != -1) {
            continue;
          }
          int idx1 = board.idx(i,j - 3);
          int idx2 = board.idx(i,j - 2);
          int idx3 = board.idx(i,j - 1);
          if(idx1 < 0 || idx2 < 0 || idx3 < 0) {
            continue;
          }
          p1.emplace_back(idx1);
          p2.emplace_back(idx2);
          p3.emplace_back(idx3);
          proposal.emplace_back(cv::Point2i(j, i));
        }
      }
      if(!proposal.empty() || params.strict_grow) {
        break;
      }
      for(int i = 0; i < rows; ++i) {
        for(int j = 2; j < cols; ++j) {
          if(board.idx(i,j) != -1) {
            continue;
          }
          int idx2 = board.idx(i,j - 2);
          int idx3 = board.idx(i,j - 1);
          if(idx2 < 0 || idx3 < 0) {
            continue;
          }
          p2.emplace_back(idx2);
          p3.emplace_back(idx3);
          proposal.emplace_back(cv::Point2i(j, i));
        }
      }
      break;
    }
    case 4: {
      for(int i = 3; i < rows; ++i) {
        for(int j = 3; j < cols; ++j) {
          if(board.idx(i,j) != -1) {
            continue;
          }
          int idx1 = board.idx(i - 3,j - 3);
          int idx2 = board.idx(i - 2,j - 2);
          int idx3 = board.idx(i - 1,j - 1);
          if(idx1 < 0 || idx2 < 0 || idx3 < 0) {
            continue;
          }
          p1.emplace_back(idx1);
          p2.emplace_back(idx2);
          p3.emplace_back(idx3);
          proposal.emplace_back(cv::Point2i(j, i));
        }
      }
      if(!proposal.empty() || params.strict_grow) {
        break;
      }
      for(int i = 2; i < rows; ++i) {
        for(int j = 2; j < cols; ++j) {
          if(board.idx(i,j) != -1) {
            continue;
          }
          int idx2 = board.idx(i - 2,j - 2);
          int idx3 = board.idx(i - 1,j - 1);
          if(idx2 < 0 || idx3 < 0) {
            continue;
          }
          p2.emplace_back(idx2);
          p3.emplace_back(idx3);
          proposal.emplace_back(cv::Point2i(j, i));
        }
      }
      break;
    }
    case 5: {
      for(int i = 0; i < rows - 3; ++i) {
        for(int j = 0; j < cols - 3; ++j) {
          if(board.idx(i,j) != -1) {
            continue;
          }
          int idx1 = board.idx(i + 3,j + 3);
          int idx2 = board.idx(i + 2,j + 2);
          int idx3 = board.idx(i + 1,j + 1);
          if(idx1 < 0 || idx2 < 0 || idx3 < 0) {
            continue;
          }
          p1.emplace_back(idx1);
          p2.emplace_back(idx2);
          p3.emplace_back(idx3);
          proposal.emplace_back(cv::Point2i(j, i));
        }
      }
      if(!proposal.empty() || params.strict_grow) {
        break;
      }
      for(int i = 0; i < rows - 2; ++i) {
        for(int j = 0; j < cols - 2; ++j) {
          if(board.idx(i,j) != -1) {
            continue;
          }
          int idx2 = board.idx(i + 2,j + 2);
          int idx3 = board.idx(i + 1,j + 1);
          if(idx2 < 0 || idx3 < 0) {
            continue;
          }
          p2.emplace_back(idx2);
          p3.emplace_back(idx3);
          proposal.emplace_back(cv::Point2i(j, i));
        }
      }
      break;
    }
    default:
      break;
    }

    // predict inside corners
    pred = predict_board_corners(corners, used, p1, p2, p3);
    board.set_num( board.num() + proposal.size());
    for(int i = 0; i < proposal.size(); ++i) {
      if(pred[i] == -2) {
          board.set_num(board.num() - 1);
      }
      board.idx(proposal[i]) = pred[i];
    }
  }

  // grow inside corners
  if(!proposal.empty()) {
    return GrowType_Inside;
  }

  // add proposal top/left/bottom/right
  if(!add_board_boundary(board, direction)) {
    return GrowType_Failure;
  }
  p1.clear();
  p2.clear();
  p3.clear();
  cols = board.cols();
  rows = board.rows();

  // grow board corners top/left/bottom/right
  switch(direction) {
  case 0: {
    for(int i = 0; i < cols; ++i) {
      int idx1 = board.idx(3,i);
      int idx2 = board.idx(2,i);
      int idx3 = board.idx(1,i);
      if(idx1 < 0 || idx2 < 0 || idx3 < 0) {
        continue;
      }
      p1.emplace_back(idx1);
      p2.emplace_back(idx2);
      p3.emplace_back(idx3);
      proposal.emplace_back(cv::Point2i(i, 0));
    }
    break;
  }
  case 1: {
    for(int i = 0; i < rows; ++i) {
      int idx1 = board.idx(i,3);
      int idx2 = board.idx(i,2);
      int idx3 = board.idx(i,1);
      if(idx1 < 0 || idx2 < 0 || idx3 < 0) {
        continue;
      }
      p1.emplace_back(idx1);
      p2.emplace_back(idx2);
      p3.emplace_back(idx3);
      proposal.emplace_back(cv::Point2i(0, i));
    }
    break;
  }
  case 2: {
    for(int i = 0; i < cols; ++i) {
      int idx1 = board.idx(rows - 4,i);
      int idx2 = board.idx(rows - 3,i);
      int idx3 = board.idx(rows - 2,i);
      if(idx1 < 0 || idx2 < 0 || idx3 < 0) {
        continue;
      }
      p1.emplace_back(idx1);
      p2.emplace_back(idx2);
      p3.emplace_back(idx3);
      proposal.emplace_back(cv::Point2i(i, rows - 1));
    }
    break;
  }
  case 3: {
    for(int i = 0; i < rows; ++i) {
      int idx1 = board.idx(i,cols - 4);
      int idx2 = board.idx(i,cols - 3);
      int idx3 = board.idx(i,cols - 2);
      if(idx1 < 0 || idx2 < 0 || idx3 < 0) {
        continue;
      }
      p1.emplace_back(idx1);
      p2.emplace_back(idx2);
      p3.emplace_back(idx3);
      proposal.emplace_back(cv::Point2i(cols - 1, i));
    }
    break;
  }
  default:
    break;
  }

  // predict board corners
  pred = predict_board_corners(corners, used, p1, p2, p3);
  if(/*params.corner_type == SaddlePoint &&*/ !params.occlusion) {
    for(int i = 0; i < proposal.size(); ++i) {
      if(pred[i] == -2) {
        board.set_num(board.num() - proposal.size());
        proposal.clear();
        return GrowType_Failure;
      }
    }
  }
  board.set_num( board.num() + proposal.size());
  for(int i = 0; i < proposal.size(); ++i) {
    if(pred[i] == -2) {
        board.set_num(board.num() - 1);
    }
    board.idx(proposal[i]) = pred[i];
  }

  if(proposal.empty()) {
    return GrowType_Failure;
  }
  return GrowType_Boundary;
}

} // namespace cbdetect
