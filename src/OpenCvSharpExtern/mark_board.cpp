#include <opencv2/opencv.hpp>
#include "mark_board.h"

namespace cbdetect {
	

	static bool calc2DSegmentsIntersection(const cv::Point2f& p1, const cv::Point2f& p2, const cv::Point2f& p3, const cv::Point2f& p4, /*out*/cv::Point2f& p)
	{
		p.x = p.y = 0.0f;

		float denominator = (p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x);
		if (denominator == 0.0f)
			return false;
		float invDenominator = 1.0f / denominator;

		float t = ((p1.x - p3.x) * (p3.y - p4.y) - (p1.y - p3.y) * (p3.x - p4.x)) * invDenominator;
		if (t >= 0.0f && t <= 1.0f)
		{
			p = p1 * (1.0f - t) + p2 * t;
			return true;
		}
		float u = ((p1.x - p2.x) * (p1.y - p3.y) - (p1.y - p2.y) * (p1.x - p3.x)) * invDenominator;
		if (u >= 0.0f && u <= 1.0f)
		{
			p = p3 * (1.0 - u) + p4 * u;
			return true;
		}
		return false;
	}

	enum eCheckUpDown
	{
		eCheckUpDown_INVALID = 0,
		eCheckUpDown_BLACK_WHITE = 1,
		eCheckUpDown_WHITE_BLACK = 2,
		eCheckUpDown_WHITE_BLACK_WHITE = 3,
		eCheckUpDown_NUMS
	};

	eCheckUpDown checkUpDown(const cv::Mat& mat_norm, const cv::Point2f& a, const cv::Point2f& b, std::vector<unsigned char>& vecLine )
	{
		int width = mat_norm.cols;
		int height = mat_norm.rows;
		//cv::Point2i imgSize(width, height);
		cv::Point2i pt1((int)(a.x + 0.5f), (int)(a.y + 0.5f));
		cv::Point2i pt2((int)(b.x + 0.5f), (int)(b.y + 0.5f));

		if (pt1.x < 0 || pt1.x >= width || pt2.x < 0 || pt2.x >= width)
			return eCheckUpDown_INVALID;
		if (pt1.y < 0 || pt1.y >= height || pt2.y < 0 || pt2.y >= height)
			return eCheckUpDown_INVALID;

		cv::LineIterator it(mat_norm, pt1, pt2, 4);
		int iCount = it.count;
		if (iCount <= 1)
			return eCheckUpDown_INVALID;
		vecLine.resize(0);
		vecLine.reserve(iCount * 2);
		for (int i = 0; i < iCount; i++, it++)
			vecLine.push_back(*((unsigned char*)*it));

		cv::Mat matLine(1, iCount, CV_8U, vecLine.data());
		cv::Mat matResult(1, iCount, CV_8U, vecLine.data() + iCount);
		cv::threshold(matLine, matResult, 0.0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
		cv::medianBlur(matResult, matLine, 5);
		const unsigned char* pBin = matLine.ptr<unsigned char>();
		bool bBit = (pBin[0] != 0);

		if (bBit == false)
		{
			int i = 0;
			for (i = 1; i < iCount; i++)
			{
				if (pBin[i] != 0 )
				{
					bBit = true;
					break;
				}
			}
			if (bBit == false)
				return eCheckUpDown_INVALID;
			for (; i < iCount; i++)
			{
				if (pBin[i] == 0)
				{
					//bBit = false;
					//break;
					return eCheckUpDown_INVALID;
				}
			}
			//if ( bBit == true )
				return eCheckUpDown_BLACK_WHITE;
			//for (; i < iCount; i++)
			//{
			//	if (pBin[i] != 0)
			//	{
			//		bBit = true;
			//		break;
			//	}
			//}
			//if (bBit == false)
			//	return eCheckUpDown_BLACK_WHITE_BLACK;
			//for (; i < iCount; i++)
			//{
			//	if (pBin[i] == 0)
			//	{
			//		return eCheckUpDown_INVALID;
			//	}
			//}
			//return eCheckUpDown_BLACK_WHITE_BLACK_WHITE;
		}
		else
		{
			int i = 0; 
			for (i = 1; i < iCount; i++)
			{
				if (pBin[i] == 0 )
				{
					bBit = false;
					break;
				}
			}
			if ( bBit == true )
				return eCheckUpDown_INVALID;
			for (; i < iCount; i++)
			{
				if (pBin[i] != 0 )
				{
					bBit = true;
					break;
				}
			}
			if (bBit == false)
				return eCheckUpDown_WHITE_BLACK;
			for (; i < iCount; i++)
			{
				if (pBin[i] == 0)
				{
					//bBit = false;
					//break;
					return eCheckUpDown_INVALID;
				}
			}
			//if (bBit == true)
				return eCheckUpDown_WHITE_BLACK_WHITE;
			//for (; i < iCount; i++)
			//{
			//	if (pBin[i] != 0)
			//	{
			//		return eCheckUpDown_INVALID;
			//	}
			//}
			//return eCheckUpDown_WHITE_BLACK_WHITE_BLACK;
		}
	}


	bool mark_board(const cv::Mat& mat_norm, const Corner& corners, std::vector<cbdetect::Board>& boards)
	{
		std::vector<unsigned char> vecLine;
		bool bIntersect = false;
		cv::Point2f pp, p;

		std::vector< std::vector<eCheckUpDown> > vecRight;
		std::vector< std::vector<eCheckUpDown> > vecDown;


		for (int n = 0; n < boards.size(); ++n)
		{
			Board& board = boards[n];

			int nrows = (int)board.rows();
			if (nrows == 0)
				continue;
			int ncols = (int)board.cols();

			vecRight.resize(0);
			vecRight.resize(nrows);
			vecDown.resize(0);
			vecDown.resize(nrows);

			for (int i = 0; i < nrows; i++)
			{
				vecRight[i].resize(ncols, eCheckUpDown_INVALID);
				vecDown[i].resize(ncols, eCheckUpDown_INVALID);
				if (i == 0 || i + 3 > nrows)
					continue;
				
				for( int j = 1; j + 3 <= ncols; j++ )
				{ 
					if (board.idx(i,j) >= 0 && board.idx(i,j + 1) >= 0 && board.idx(i + 1,j) >= 0 && board.idx(i + 1,j + 1) >= 0)
					{
						board.type(i,j) = CheckerUndetermined;
					}
				}
			}

			if (nrows < 4 || ncols < 4 )
				continue;

			if (nrows == 4 && ncols == 4)
				continue;

			int jmin = 1;
			int jmax = ncols - 3;
			int imin = 1;
			int imax = nrows - 3;

			int iBlack = 0;
			int iWhite = 0;


			if (nrows == 4)
			{
				bool bpvalid = false;
				bool bvalid = false;
				bool bOpposite = true;
				for (int j = 1; j <= jmax; j++, pp = p, bpvalid = bvalid, bOpposite = !bOpposite )
				{
					bvalid = board.type(1,j) == CheckerUndetermined;
					if (bvalid == true)
					{
						bIntersect = calc2DSegmentsIntersection(corners.p[board.idx(1,j)], corners.p[board.idx(2,j+1)]
							, corners.p[board.idx(1,j+1)], corners.p[board.idx(2,j)], p);
						if (bIntersect == false)
							bvalid = false;
					}
					if ( bvalid == true && bpvalid == true)
					{
						eCheckUpDown eCheck = checkUpDown(mat_norm, pp, p, vecLine);
						vecRight[1][j - 1] = eCheck;

						if (eCheck == eCheckUpDown_BLACK_WHITE)
						{
							if (!bOpposite)
								iBlack++;
							else
								iWhite++;
						}
						else if (eCheck == eCheckUpDown_WHITE_BLACK)
						{
							if (!bOpposite)
								iWhite++;
							else
								iBlack++;
						}
					}


				}
			}
			else if (ncols == 4)
			{
				bool bpvalid = false;
				bool bvalid = false;
				bool bOpposite = true;
				for (int i = 1; i <= imax; i++, pp = p, bpvalid = bvalid, bOpposite = !bOpposite)
				{
					bvalid = board.type(i,1) == CheckerUndetermined;
					if (bvalid == true)
					{
						bIntersect = calc2DSegmentsIntersection(corners.p[board.idx(i,1)], corners.p[board.idx(i+1,2)]
							, corners.p[board.idx(i,2)], corners.p[board.idx(i+1,1)], p);
						if (bIntersect == false)
							bvalid = false;
					}
					if (bvalid == true && bpvalid == true)
					{
						eCheckUpDown eCheck = checkUpDown(mat_norm, pp, p, vecLine);
						vecRight[i - 1][1] = eCheck;
						if (eCheck == eCheckUpDown_BLACK_WHITE)
						{
							if (!bOpposite)
								iBlack++;
							else
								iWhite++;
						}
						else if (eCheck == eCheckUpDown_WHITE_BLACK)
						{
							if (!bOpposite)
								iWhite++;
							else
								iBlack++;
						}
					}
				}
			}
			else
			{
				std::vector<bool> vecpValid;
				vecpValid.resize(ncols, false);
				std::vector<cv::Point2f> vecpp;
				vecpp.resize(ncols);
				bool bOpposite = true;

				for (int i = 1; i <= imax; i++, bOpposite = !bOpposite )
				{
					bool bpvalid = false;
					bool bvalid = false;
					bool bJOpposite = bOpposite;
					for (int j = 1; j <= jmax; j++, pp = p, bpvalid = bvalid, bJOpposite = !bJOpposite )
					{
						bvalid = board.type(i,j) == CheckerUndetermined;
						if (bvalid == true)
						{
							bIntersect = calc2DSegmentsIntersection(corners.p[board.idx(i,j)], corners.p[board.idx(i+1,j + 1)]
								, corners.p[board.idx(i,j + 1)], corners.p[board.idx(i+1,j)], p);
							if (bIntersect == false)
								bvalid = false;

						}
						if (bvalid == true && bpvalid == true)
						{
							eCheckUpDown eCheck = checkUpDown(mat_norm, pp, p, vecLine);
							vecRight[i][j - 1] = eCheck;
							if (eCheck == eCheckUpDown_BLACK_WHITE)
							{
								if (!bJOpposite)
									iBlack++;
								else
									iWhite++;
							}
							else if (eCheck == eCheckUpDown_WHITE_BLACK)
							{
								if (!bJOpposite)
									iWhite++;
								else
									iBlack++;
							}
						}
						if (bvalid == true && i > 1 && vecpValid[j] == true )
						{
							eCheckUpDown eCheck = checkUpDown(mat_norm, vecpp[j], p, vecLine);
							vecDown[i - 1][j] = eCheck;
							if (eCheck == eCheckUpDown_BLACK_WHITE)
							{
								if (!bJOpposite)
									iBlack++;
								else
									iWhite++;
							}
							else if (eCheck == eCheckUpDown_WHITE_BLACK)
							{
								if (!bJOpposite)
									iWhite++;
								else
									iBlack++;
							}
						}
						vecpValid[j] = bvalid;
						vecpp[j] = p;
					}
				}
			}
			if (iBlack == iWhite)
				continue;


			bool bWhite = (iBlack > iWhite) ? false : true;
			board.setBoardType((bWhite) ? BoardTypeWhite : BoardTypeBlack);

			if (nrows == 4)
			{
				for (int j = 1; j <= jmax; j++, bWhite = !bWhite)
				{
					if (j == 1)
					{
						if (bWhite == true)
						{
							board.type(1,j) = CheckerWhite;
						}
						else
						{
							switch (vecRight[1][j])
							{
							case eCheckUpDown_BLACK_WHITE:
								board.type(1,j) = CheckerBlack;
								break;
							case eCheckUpDown_WHITE_BLACK:
							case eCheckUpDown_WHITE_BLACK_WHITE:
								board.type(1,j) = CheckerBlackWhiteBit;
								break;
							}
						}
					}
					else if (j == jmax)
					{
						if (bWhite == true)
						{
							board.type(1,j) = CheckerWhite;
						}
						else
						{
							switch (vecRight[1][j-1])
							{
							case eCheckUpDown_WHITE_BLACK:
								board.type(1,j) = CheckerBlack;
								break;
							case eCheckUpDown_BLACK_WHITE:
							case eCheckUpDown_WHITE_BLACK_WHITE:
								board.type(1,j) = CheckerBlackWhiteBit;
								break;
							}
						}
					}
					else// if (j < jmax)
					{
						if (bWhite == true)
						{
							board.type(1,j) = CheckerWhite;
						}
						else
						{
							if (vecRight[1][j - 1] == eCheckUpDown_INVALID && vecRight[1][j] == eCheckUpDown_INVALID)
								board.type(1,j) = CheckerUndetermined;
							else if (vecRight[1][j - 1] == eCheckUpDown_WHITE_BLACK && vecRight[1][j] == eCheckUpDown_BLACK_WHITE)
								board.type(1,j) = CheckerBlack;
							else if ((vecRight[1][j - 1] == eCheckUpDown_WHITE_BLACK_WHITE || vecRight[1][j - 1] == eCheckUpDown_BLACK_WHITE || vecRight[1][j - 1] == eCheckUpDown_INVALID )
								&& ( vecRight[1][j] == eCheckUpDown_WHITE_BLACK_WHITE || vecRight[1][j] == eCheckUpDown_WHITE_BLACK || vecRight[1][j] == eCheckUpDown_INVALID ) )
								board.type(1,j) = CheckerBlackWhiteBit;
						}
					}
				}
			}
			else if (ncols == 4)
			{
				for (int i = 1; i <= imax; i++, bWhite = !bWhite)
				{
					if (i == 1)
					{
						if (bWhite == true)
						{
							board.type(i,1) = CheckerWhite;
						}
						else
						{
							switch (vecDown[i][1])
							{
							case eCheckUpDown_BLACK_WHITE:
								board.type(i,1) = CheckerBlack;
								break;
							case eCheckUpDown_WHITE_BLACK:
							case eCheckUpDown_WHITE_BLACK_WHITE:
								board.type(i,1) = CheckerBlackWhiteBit;
								break;
							}
						}
					}
					else if (i == imax)
					{
						if (bWhite == true)
						{
							board.type(i,1) = CheckerWhite;
						}
						else
						{
							switch (vecDown[i-1][1])
							{
							case eCheckUpDown_WHITE_BLACK:
								board.type(i,1) = CheckerBlack;
								break;
							case eCheckUpDown_BLACK_WHITE:
							case eCheckUpDown_WHITE_BLACK_WHITE:
								board.type(i,1) = CheckerBlackWhiteBit;
								break;
							}
						}
					}
					else// if (i < imax)
					{
						if (bWhite == true)
						{
							board.type(i,1) = CheckerWhite;
						}
						else
						{
							if (vecDown[i - 1][1] == eCheckUpDown_INVALID && vecDown[i][1] == eCheckUpDown_INVALID)
								board.type(i,1) = CheckerUndetermined;
							else if (vecDown[i-1][1] == eCheckUpDown_WHITE_BLACK && vecDown[i][1] == eCheckUpDown_BLACK_WHITE)
								board.type(i,1) = CheckerBlack;
							else if ((vecDown[i-1][1] == eCheckUpDown_WHITE_BLACK_WHITE || vecDown[i-1][1] == eCheckUpDown_BLACK_WHITE || vecDown[i - 1][1] == eCheckUpDown_INVALID)
								&& (vecDown[i][1] == eCheckUpDown_WHITE_BLACK_WHITE || vecDown[i][1] == eCheckUpDown_WHITE_BLACK || vecDown[i][1] == eCheckUpDown_INVALID) )
								board.type(i,1) = CheckerBlackWhiteBit;
						}
					}
				}
			}
			else
			{
				for (int i = 1; i <= imax; i++, bWhite = !bWhite)
				{
					bool bJWhite = bWhite;
					for (int j = 1; j <= jmax; j++, bJWhite = !bJWhite)
					{
						if (bJWhite == true)
						{
							board.type(i,j) = CheckerWhite;
						}
						else
						{
							CheckerType vert = CheckerUndetermined;
							CheckerType horiz = CheckerUndetermined;
							if ((i == 1 || i == imax) && (j == 1 || j == jmax))
							{
								if (i == 1)
								{
									switch (vecDown[i][j])
									{
									case eCheckUpDown_BLACK_WHITE:
										vert = CheckerBlack;
										break;
									case eCheckUpDown_WHITE_BLACK:
									case eCheckUpDown_WHITE_BLACK_WHITE:
										vert = CheckerBlackWhiteBit;
										break;
									}
								}
								else
								{
									switch (vecDown[i - 1][j])
									{
									case eCheckUpDown_WHITE_BLACK:
										vert = CheckerBlack;
										break;
									case eCheckUpDown_BLACK_WHITE:
									case eCheckUpDown_WHITE_BLACK_WHITE:
										vert = CheckerBlackWhiteBit;
										break;
									}
								}

								if ( j == 1 )
								{

									switch (vecRight[i][j])
									{
									case eCheckUpDown_BLACK_WHITE:
										horiz = CheckerBlack;
										break;
									case eCheckUpDown_WHITE_BLACK:
									case eCheckUpDown_WHITE_BLACK_WHITE:
										horiz = CheckerBlackWhiteBit;
										break;
									}
								}
								else
								{
									switch (vecRight[i][j - 1])
									{
									case eCheckUpDown_WHITE_BLACK:
										horiz = CheckerBlack;
										break;
									case eCheckUpDown_BLACK_WHITE:
									case eCheckUpDown_WHITE_BLACK_WHITE:
										horiz = CheckerBlackWhiteBit;
										break;
									}
								}
							}
							else if (i == 1 || i == imax || j == 1 || j == jmax)
							{
								if (i == 1 || i == imax)
								{
									if ( i == 1 )
									{
										switch (vecDown[i][j])
										{
										case eCheckUpDown_BLACK_WHITE:
											vert = CheckerBlack;
											break;
										case eCheckUpDown_WHITE_BLACK:
										case eCheckUpDown_WHITE_BLACK_WHITE:
											vert = CheckerBlackWhiteBit;
											break;
										}
									}
									else
									{
										switch (vecDown[i - 1][j])
										{
										case eCheckUpDown_WHITE_BLACK:
											vert = CheckerBlack;
											break;
										case eCheckUpDown_BLACK_WHITE:
										case eCheckUpDown_WHITE_BLACK_WHITE:
											vert = CheckerBlackWhiteBit;
											break;
										}
									}
									if (vecRight[i][j - 1] == eCheckUpDown_INVALID && vecRight[i][j] == eCheckUpDown_INVALID)
										horiz = CheckerUndetermined;
									else if (vecRight[i][j - 1] == eCheckUpDown_WHITE_BLACK && vecRight[i][j] == eCheckUpDown_BLACK_WHITE)
										horiz = CheckerBlack;
									else if ((vecRight[i][j - 1] == eCheckUpDown_WHITE_BLACK_WHITE || vecRight[i][j - 1] == eCheckUpDown_BLACK_WHITE || vecRight[i][j - 1] == eCheckUpDown_INVALID)
										&& (vecRight[i][j] == eCheckUpDown_WHITE_BLACK_WHITE || vecRight[i][j] == eCheckUpDown_WHITE_BLACK || vecRight[i][j] == eCheckUpDown_INVALID))
										horiz = CheckerBlackWhiteBit;
								}
								else
								{
									if (j == 1)
									{
										switch (vecRight[i][j])
										{
										case eCheckUpDown_BLACK_WHITE:
											horiz = CheckerBlack;
											break;
										case eCheckUpDown_WHITE_BLACK:
										case eCheckUpDown_WHITE_BLACK_WHITE:
											horiz = CheckerBlackWhiteBit;
											break;
										}
									}
									else// if (j == jmax)
									{
										switch (vecRight[i][j - 1])
										{
										case eCheckUpDown_WHITE_BLACK:
											horiz = CheckerBlack;
											break;
										case eCheckUpDown_BLACK_WHITE:
										case eCheckUpDown_WHITE_BLACK_WHITE:
											horiz = CheckerBlackWhiteBit;
											break;
										}
									}
									if (vecDown[i - 1][j] == eCheckUpDown_INVALID && vecDown[i][j] == eCheckUpDown_INVALID)
										vert = CheckerUndetermined;
									else if (vecDown[i - 1][j] == eCheckUpDown_WHITE_BLACK && vecDown[i][j] == eCheckUpDown_BLACK_WHITE)
										vert = CheckerBlack;
									else if ((vecDown[i - 1][j] == eCheckUpDown_WHITE_BLACK_WHITE || vecDown[i - 1][j] == eCheckUpDown_BLACK_WHITE || vecDown[i - 1][j] == eCheckUpDown_INVALID)
										&& (vecDown[i][j] == eCheckUpDown_WHITE_BLACK_WHITE || vecDown[i][j] == eCheckUpDown_WHITE_BLACK || vecDown[i][j] == eCheckUpDown_INVALID))
										vert = CheckerBlackWhiteBit;
								}
							}
							else
							{
								if (vecRight[i][j - 1] == eCheckUpDown_INVALID && vecRight[i][j] == eCheckUpDown_INVALID)
									horiz = CheckerUndetermined;
								else if (vecRight[i][j - 1] == eCheckUpDown_WHITE_BLACK && vecRight[i][j] == eCheckUpDown_BLACK_WHITE)
									horiz = CheckerBlack;
								else if ((vecRight[i][j - 1] == eCheckUpDown_WHITE_BLACK_WHITE || vecRight[i][j - 1] == eCheckUpDown_BLACK_WHITE || vecRight[i][j - 1] == eCheckUpDown_INVALID)
									&& (vecRight[i][j] == eCheckUpDown_WHITE_BLACK_WHITE || vecRight[i][j] == eCheckUpDown_WHITE_BLACK || vecRight[i][j] == eCheckUpDown_INVALID))
									horiz = CheckerBlackWhiteBit;

								if (vecDown[i - 1][j] == eCheckUpDown_INVALID && vecDown[i][j] == eCheckUpDown_INVALID)
									vert = CheckerUndetermined;
								else if (vecDown[i - 1][j] == eCheckUpDown_WHITE_BLACK && vecDown[i][j] == eCheckUpDown_BLACK_WHITE)
									vert = CheckerBlack;
								else if ((vecDown[i - 1][j] == eCheckUpDown_WHITE_BLACK_WHITE || vecDown[i - 1][j] == eCheckUpDown_BLACK_WHITE || vecDown[i - 1][j] == eCheckUpDown_INVALID)
									&& (vecDown[i][j] == eCheckUpDown_WHITE_BLACK_WHITE || vecDown[i][j] == eCheckUpDown_WHITE_BLACK || vecDown[i][j] == eCheckUpDown_INVALID))
									vert = CheckerBlackWhiteBit;
							}
							if (vert == CheckerUndetermined)
								board.type(i,j) = horiz;
							else if (horiz == CheckerUndetermined)
								board.type(i,j) = vert;
							else
								board.type(i,j) = (vert == horiz) ? vert : CheckerUndetermined;
						}
					}
				}
			}
		}

		return true;
	}



}