#include "config.h"
//#include "weight_mask.h"

namespace cbdetect {


	void  Params::Prepare()
	{
		weight_mask.clear();
		weight_mask.resize(radius.size());
		for (int i = 0; i < (int)radius.size(); i++)
		{
			int r = radius[i];
			weight_mask[i] = cv::Mat::zeros(r * 2 + 1, r * 2 + 1, CV_32F);
			cv::Mat& mat = weight_mask[i];
			for (int v = 0; v < r * 2 + 1; ++v) 
			{
				for (int u = 0; u < r * 2 + 1; ++u)
				{
					float dist = std::sqrtf((u - r) * (u - r) + (v - r) * (v - r)) / r;
					dist = std::min(std::max(dist, 0.7f), 1.3f);
					mat.at<float>(v, u) = (1.3f - dist) / 0.6f;
				}
			}
		}
	}


	void Board::set_size(int rows, int cols)
	{
		m_rows = rows;
		m_cols = cols;
		m_data.resize(0);
		m_data.resize(rows * cols);
	}

	void Board::add_board_boundary(BoardDirection eDir)
	{
		int r, c, index, index_old;
		switch (eDir)
		{
		case BoardDirectionTop:
			m_rows++;
			m_data.resize(m_rows * m_cols);
			for (r = m_rows - 2; r >= 0; r--)
			{
				for (c = 0; c < m_cols; c++)
				{
					index = m_cols * r + c;
					m_data[index + m_cols] = m_data[index];
				}
			}
			for (c = 0; c < m_cols; c++)
				m_data[c].reset();
			break;
		case BoardDirectionLeft:
			m_cols++;
			m_data.resize(m_rows * m_cols);
			for (r = m_rows - 1; r >= 0; r--)
			{
				index = m_cols * r;
				index_old = (m_cols - 1) * r;
				for (c = m_cols - 2; c >= 0; c--)
				{
					m_data[index + c + 1] = m_data[index_old + c];
				}
			}
			for (r = 0; r < m_rows; r++)
				m_data[m_cols * r].reset();
			break;
		case BoardDirectionBottom:
			m_rows++;
			m_data.resize(m_rows * m_cols);
			index = m_cols * (m_rows - 1);
			for (c = 0; c < m_cols; c++)
				m_data[index + c].reset();
			break;
		case BoardDirectionRight:
			m_cols++;
			m_data.resize(m_rows * m_cols);
			for (r = m_rows - 1; r >= 1; r--)
			{
				index = m_cols * r;
				index_old = (m_cols - 1) * r;
				for (c = m_cols - 2; c >= 0; c--)
				{
					m_data[index + c] = m_data[index_old + c];
				}
			}
			for (r = 0; r < m_rows; r++)
				m_data[m_cols * r + m_cols - 1].reset();
			break;

		}
	}

}