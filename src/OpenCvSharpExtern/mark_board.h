
#pragma once
#ifndef LIBCBDETECT_MARK_BOARD_H
#define LIBCBDETECT_MARK_BOARD_H

#include <vector>

#include "config.h"

namespace cbdetect {

	LIBCBDETECT_DLL_DECL bool mark_board(const cv::Mat& mat_norm, const Corner& corners, std::vector<cbdetect::Board>& boards);

}

#endif //LIBCBDETECT_MARK_BOARD_H
