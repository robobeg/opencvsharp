#include "NewCylindricalProbeTracker.h"

#ifdef USE_LIBCBDETECTOR
#include "config.h"
#include "find_corners.h"
#include "boards_from_corners.h"
#include "mark_board.h"
#endif USE_LIBCBDETECTOR

#include <vector>
#include <list>
#include <limits>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <cmath>
#include <algorithm>
#include <atomic>
#include <unknwn.h>


namespace NewCylindricalProbeTrackerNamespace
{
	typedef cv::Point2i Point2i;
	typedef cv::Point2f Point2;
	typedef cv::Point3f Point3;
	typedef cv::Size2i  Size;
	typedef cv::Rect	OpenCVRect;
	typedef unsigned int uint;
	typedef cv::Matx44f Matrix4x4;
	typedef cv::Vec3f	Vector3;
	typedef cv::Vec4f   Vector4;
	typedef cv::Vec4f   Quaternion;
	typedef unsigned char Byte;

	struct BoundBoxResult
	{
		UINT    m_xMin;
		UINT    m_yMin;
		UINT    m_zMin;
		UINT    m_xMax;
		UINT    m_yMax;
		UINT    m_zMax;
	};

	struct LocateAtTimestamp
	{
		int64_t m_i64Timestamp;
		IUnknown* m_pCoordinateSystem;
		Point3    m_position;
		Quaternion m_orientation;
	};

	struct  BoundPoints
	{
		Point3    m_aPos[8];
		int64_t	  m_i64Timestamp;

		BoundPoints()
			: m_i64Timestamp(0)
		{
		}
	};

	struct CheckerboardCoordinate
	{
		float outerX;
		float outerY;
		float middleY;
		float outerZ;
	};

	struct AuxiliaryMarkerNativeStruct
	{
		int type;

		float pivotX;
		float pivotY;
		float pivotZ;
		float angVecX;
		float angVecY;
		float angVecZ;

		int codeMajor;
		uint code;

		int checkerboardXLength;
		float checkerboardMarkerSize;
		int checkerboardCenterCoordinate;
		int checkerboardCoordinatesSize;
		const CheckerboardCoordinate* checkerboardCoordinates;
	};

	struct ParameterNativeStruct
	{
		float radius;
		float blobRadius;
		float markerSize;
		int markerCodeLength;
		float markerCodeStartAngle;
		int markerCodeStartMajor;
		int markerCodePatternsSize;
		float screenZ;
		int auxiliaryMarkerSettingsSize;
		float auxMarkerTrackCosAngle;
		float validPositionChange;
		float validAngleChange;
		float adaptiveThresholdBlockRatio1;
		float adaptiveThresholdBlockRatio2;
		float adaptiveThresholdC;
		int minDilations;
		int maxDilations;
		float expandAreaRatio;
		int expandAreaMargin;
		float blobDetectionTolerance;
		int cornerPixelTolerance;
		int trackPixelTolerance;
		int trackNumToleranceFrames;
		int useDetectionThread;
		int useTrackingThread;
		cv::aruco::DetectorParameters detectorParams;
		const uint* markerCodePatterns;
		const AuxiliaryMarkerNativeStruct* auxiliaryMarkerSettings;
	};

	struct FrameInfoNativeStruct
	{
		cv::Mat* m_pMatCamera;
		cv::Mat* m_pMatImageGray;
		cv::Mat* m_pMatImageOriginal;
		std::vector<float>* m_pVecEyeGaze;
		IUnknown* m_pSpatialCoordinateSystem;
		int64_t m_i64FrameTimestamp;

		FrameInfoNativeStruct()
			: m_i64FrameTimestamp(0)
			, m_pMatCamera(nullptr)
			, m_pMatImageGray(nullptr)
			, m_pMatImageOriginal(nullptr)
			, m_pVecEyeGaze(nullptr)
			, m_pSpatialCoordinateSystem(nullptr)
		{
		}
	};

	struct PoseAndSpatialCoordinate
	{
		float m_fPositionX;
		float m_fPositionY;
		float m_fPositionZ;
		float m_fOrientationX;
		float m_fOrientationY;
		float m_fOrientationZ;
		float m_fOrientationW;
		IUnknown* m_pSpatialCoordinateSystem;

		PoseAndSpatialCoordinate()
			: m_fPositionX(0.f)
			, m_fPositionY(0.f)
			, m_fPositionZ(0.f)
			, m_fOrientationX(0.f)
			, m_fOrientationY(0.f)
			, m_fOrientationZ(0.f)
			, m_fOrientationW(1.f)
			, m_pSpatialCoordinateSystem(nullptr)
		{
		}
	};

	enum EState
	{
		TrackedPose_INVALID = 0,
		TrackedPose_UNSTABLE = 1,
		TrackedPose_STABLE = 2,
	};

	struct TrackedPoseNativeStruct
	{
		int64_t		m_i64Timestamp;
		int			m_eState;
		float		m_fPositionX;
		float		m_fPositionY;
		float		m_fPositionZ;
		float		m_fOrientationX;
		float		m_fOrientationY;
		float		m_fOrientationZ;
		float		m_fOrientationW;

		TrackedPoseNativeStruct()
			: m_i64Timestamp(0)
			, m_eState(TrackedPose_INVALID)
			, m_fPositionX(0)
			, m_fPositionY(0)
			, m_fPositionZ(0)
			, m_fOrientationX(0)
			, m_fOrientationY(0)
			, m_fOrientationZ(0)
			, m_fOrientationW(1)
		{
		}
	};

	struct BoundRect
	{
		int m_iX;
		int m_iY;
		int m_iWidth;
		int m_iHeight;
	};


	struct AuxiliaryMarker
	{
		enum Type
		{
			CHECKER,
			NOPATTERN,
		};

		Type type;

		float pivotX;
		float pivotY;
		float pivotZ;
		float angVecX;
		float angVecY;
		float angVecZ;

		bool codeMajor;
		uint code;

		int checkerboardXLength;
		float checkerboardMarkerSize;
		int checkerboardCenterCoordinate;
		std::vector<CheckerboardCoordinate> checkerboardCoordinates;

		AuxiliaryMarker()
			: type(CHECKER)
			, pivotX(0)
			, pivotY(0)
			, pivotZ(0)
			, angVecX(0)
			, angVecY(0)
			, angVecZ(0)
			, codeMajor(false)
			, code(0)
			, checkerboardXLength(0)
			, checkerboardMarkerSize(0.0f)
			, checkerboardCenterCoordinate(0)
			, checkerboardCoordinates()
		{
		}
	};

	struct Parameter
	{
		float radius;
		float blobRadius;
		float markerSize;
		int markerCodeLength;
		float markerCodeStartAngle;
		bool markerCodeStartMajor;
		std::vector<uint> markerCodePatterns;
		float screenZ;
		std::vector<AuxiliaryMarker> auxiliaryMarkerSettings;

		float auxMarkerTrackCosAngle;
		cv::aruco::DetectorParameters detectorParams;

		float validPositionChange;
		float validAngleChange;

		float adaptiveThresholdBlockRatio1;
		float adaptiveThresholdBlockRatio2;
		float adaptiveThresholdC;

		int   minDilations;
		int	maxDilations;
		float expandAreaRatio;
		int expandAreaMargin;

		float blobDetectionTolerance;
		int cornerPixelTolerance;
		int trackPixelTolerance;
		int trackNumToleranceFrames;
		bool useDetectionThread;
		bool useTrackingThread;

		Parameter()
			: radius(0)
			, blobRadius(0)
			, markerSize(0)
			, markerCodeLength(0)
			, markerCodeStartAngle(0)
			, markerCodeStartMajor(false)
			, markerCodePatterns()
			, screenZ(0)
			, auxiliaryMarkerSettings(0)
			, auxMarkerTrackCosAngle(0)
			, detectorParams()
			, validPositionChange(0)
			, validAngleChange(0)
			, adaptiveThresholdBlockRatio1(0)
			, adaptiveThresholdBlockRatio2(0)
			, adaptiveThresholdC(0)
			, minDilations(0)
			, maxDilations(0)
			, expandAreaRatio(0)
			, expandAreaMargin(0)
			, blobDetectionTolerance(0)
			, cornerPixelTolerance(0)
			, trackPixelTolerance(0)
			, trackNumToleranceFrames(0)
			, useDetectionThread(false)
			, useTrackingThread(false)
		{
		}

		uint getRevertedPattern(uint pattern)
		{
			uint ret = 0;
			for (int i = 0; i < markerCodeLength; i++)
			{
				ret <<= 1;
				ret |= ((pattern & 1U) != 0U) ? 1U : 0U;
				pattern >>= 1;
			}
			return ret;
		}
	};



#define	 MY_PI	(3.14159265358979323846f)
#define  MY_DEG2RAD	(MY_PI/180.f)
#define  MY_RAD2DEG (180.f/MY_PI)

	class LineIter
	{
	private:
		static bool clipLine(const Size& img_size, Point2i& pt1, Point2i& pt2)
		{
			int c1, c2;
			int right = img_size.width - 1, bottom = img_size.height - 1;

			if (img_size.width <= 0 || img_size.height <= 0)
				return false;

			c1 = ((pt1.x < 0) ? 1 : 0) + ((pt1.x > right) ? 2 : 0) + ((pt1.y < 0) ? 4 : 0) + ((pt1.y > bottom) ? 8 : 0);
			c2 = ((pt2.x < 0) ? 1 : 0) + ((pt2.x > right) ? 2 : 0) + ((pt2.y < 0) ? 4 : 0) + ((pt2.y > bottom) ? 8 : 0);

			if ((c1 & c2) == 0 && (c1 | c2) != 0)
			{
				int a;
				if ((c1 & 12) != 0)
				{
					a = c1 < 8 ? 0 : bottom;
					pt1.x += (int)((double)(a - pt1.y) * (pt2.x - pt1.x) / (pt2.y - pt1.y));
					pt1.y = a;
					c1 = ((pt1.x < 0) ? 1 : 0) + ((pt1.x > right) ? 2 : 0);
				}
				if ((c2 & 12) != 0)
				{
					a = c2 < 8 ? 0 : bottom;
					pt2.x += (int)((double)(a - pt2.y) * (pt2.x - pt1.x) / (pt2.y - pt1.y));
					pt2.y = a;
					c2 = ((pt2.x < 0) ? 1 : 0) + ((pt2.x > right) ? 2 : 0);
				}
				if ((c1 & c2) == 0 && (c1 | c2) != 0)
				{
					if (c1 != 0)
					{
						a = c1 == 1 ? 0 : right;
						pt1.y += (int)((double)(a - pt1.x) * (pt2.y - pt1.y) / (pt2.x - pt1.x));
						pt1.x = a;
						c1 = 0;
					}
					if (c2 != 0)
					{
						a = c2 == 1 ? 0 : right;
						pt2.y += (int)((double)(a - pt2.x) * (pt2.y - pt1.y) / (pt2.x - pt1.x));
						pt2.x = a;
						c2 = 0;
					}
				}

				//Assert.IsTrue((c1 & c2) != 0 || (pt1.x | pt1.y | pt2.x | pt2.y) >= 0);
			}

			return (c1 | c2) == 0;
		}

		Point2i pt1;
		Point2i pt2;
		int count;
		int ptr;
		int step;
		int err;
		int minusDelta, plusDelta;
		int minusStep, plusStep;
		int mask;

	public:

		bool Next(Point2i& p)
		{
			if (count <= 0)
				return false;

			count--;
			p.y = ptr / step;
			p.x = ptr - p.y * step;
			mask = err < 0 ? -1 : 0;
			err += minusDelta + (plusDelta & mask);
			ptr += minusStep + (plusStep & mask);
			return true;
		}

		int  GetCount() const { return count >= 0 ? count : 0; }

		LineIter(const Size& imgSize, const Point2i& pt1_, const Point2i& pt2_, bool bConnectivity8, bool left_to_right = true)
			: pt1(pt1_)
			, pt2(pt2_)
		{
			count = 0;
			//Assert.IsTrue(connectivity == 8 || connectivity == 4);

			if (pt1.x < 0 || pt1.x >= imgSize.width ||
				pt2.x < 0 || pt2.x >= imgSize.width ||
				pt1.y < 0 || pt1.y >= imgSize.height ||
				pt2.y < 0 || pt2.y >= imgSize.height)
			{
				if (!clipLine(imgSize, pt1, pt2))
				{
					return;
				}
			}

			int bt_pix = 1;
			int istep = imgSize.width;

			int dx = pt2.x - pt1.x;
			int dy = pt2.y - pt1.y;
			int s = dx < 0 ? -1 : 0;

			if (left_to_right == true)
			{
				dx = (dx ^ s) - s;
				dy = (dy ^ s) - s;
				pt1.x ^= (pt1.x ^ pt2.x) & s;
				pt1.y ^= (pt1.y ^ pt2.y) & s;
			}
			else
			{
				dx = (dx ^ s) - s;
				bt_pix = (bt_pix ^ s) - s;
			}

			ptr = pt1.y * istep + pt1.x;

			s = dy < 0 ? -1 : 0;
			dy = (dy ^ s) - s;
			istep = (istep ^ s) - s;

			s = dy > dx ? -1 : 0;

			/* conditional swaps */
			dx ^= dy & s;
			dy ^= dx & s;
			dx ^= dy & s;

			bt_pix ^= istep & s;
			istep ^= bt_pix & s;
			bt_pix ^= istep & s;

			if (bConnectivity8)
			{
				//Assert.IsTrue(dx >= 0 && dy >= 0);

				err = dx - (dy + dy);
				plusDelta = dx + dx;
				minusDelta = -(dy + dy);
				plusStep = istep;
				minusStep = bt_pix;
				count = dx + 1;
			}
			else /* connectivity == 4 */
			{
				//Assert.IsTrue(dx >= 0 && dy >= 0);

				err = 0;
				plusDelta = (dx + dx) + (dy + dy);
				minusDelta = -(dy + dy);
				plusStep = (istep - bt_pix);
				minusStep = bt_pix;
				count = dx + dy + 1;
			}

			step = imgSize.width;
			mask = 0;

		}
	};



	class NewCylindricalProbeTracker
	{
	private:



		enum MarkerType
		{
			MarkerType_INVALID = -1,
			MarkerType_CYLINDER = 0,
			MarkerType_AUX_BASE = 1,
		};

		enum MarkerTypeFlag
		{
			MarkerTypeFlag_INVALID = 0,
			MarkerTypeFlag_CYLINDER = 1 << (int)MarkerType_CYLINDER,
			//TOP = 2,
			//BOTTOM = 4,
			//ALL = CYLINDER | TOP | BOTTOM,
		};

		struct CodeClass
		{
			int m_type;
			int m_index;

			CodeClass()
				: m_type(0)
				, m_index(0)
			{
			}
		};

		struct MarkerInfo
		{
			bool m_bMajor;

			std::vector<Point3> m_corner3DPoints;
			//List<Point3> m_lcenter3DPoints;
			std::vector<Point3> m_corner3DNormals;

			MarkerInfo()
				: m_bMajor(false)
			{
			}

			MarkerInfo(int iMarkerCodeLength)
				: m_bMajor(false)
			{
				m_corner3DPoints.resize(iMarkerCodeLength + 1);
				m_corner3DNormals.resize(iMarkerCodeLength + 1);
			}

			MarkerInfo(const MarkerInfo& rhs_)
				: m_bMajor(rhs_.m_bMajor)
				, m_corner3DPoints(rhs_.m_corner3DPoints)
				, m_corner3DNormals(rhs_.m_corner3DNormals)
			{
			}

			MarkerInfo(MarkerInfo&& rhs_)
				: m_bMajor(rhs_.m_bMajor)
				, m_corner3DPoints(std::move(rhs_.m_corner3DPoints))
				, m_corner3DNormals(std::move(rhs_.m_corner3DNormals))
			{
			}


			MarkerInfo& operator = (const MarkerInfo& rhs_)
			{
				if (this != &rhs_)
				{
					m_bMajor = rhs_.m_bMajor;
					m_corner3DPoints = rhs_.m_corner3DPoints;
					m_corner3DNormals = rhs_.m_corner3DNormals;
				}
				return *this;
			}

			MarkerInfo& operator = (MarkerInfo&& rhs_)
			{
				if (this != &rhs_)
				{
					m_bMajor = rhs_.m_bMajor;
					m_corner3DPoints.swap(rhs_.m_corner3DPoints);
					m_corner3DNormals.swap(rhs_.m_corner3DNormals);
				}
				return *this;
			}

		};

		struct MarkerRuntime
		{
			std::vector<Point2> m_prevImgPoints;
			std::vector<Point2> m_prevLcenterImgPoints;
			std::vector<Point2> m_tmpImgPoints;
			std::vector<Point2> m_tmpLcenterImgPoints;
			std::vector<int> m_tmpImgPointsErr;

			MarkerRuntime()
			{
			}

			MarkerRuntime(const MarkerRuntime& rhs_)
				: m_prevImgPoints(rhs_.m_prevImgPoints)
				, m_prevLcenterImgPoints(rhs_.m_prevLcenterImgPoints)
				, m_tmpImgPoints(rhs_.m_tmpImgPoints)
				, m_tmpLcenterImgPoints(rhs_.m_tmpLcenterImgPoints)
				, m_tmpImgPointsErr(rhs_.m_tmpImgPointsErr)
			{
			}

			MarkerRuntime(MarkerRuntime&& rhs_)
				: m_prevImgPoints(std::move(rhs_.m_prevImgPoints))
				, m_prevLcenterImgPoints(std::move(rhs_.m_prevLcenterImgPoints))
				, m_tmpImgPoints(std::move(rhs_.m_tmpImgPoints))
				, m_tmpLcenterImgPoints(std::move(rhs_.m_tmpLcenterImgPoints))
				, m_tmpImgPointsErr(std::move(rhs_.m_tmpImgPointsErr))
			{
			}

			MarkerRuntime(int iMarkerCodeLength)
			{
				m_prevImgPoints.resize(iMarkerCodeLength + 1, Point2(0, 0));
				m_tmpImgPoints.resize(iMarkerCodeLength + 1, Point2(0, 0));
				m_tmpImgPointsErr.resize(iMarkerCodeLength + 1, 0);

				m_prevLcenterImgPoints.resize(iMarkerCodeLength, Point2(0, 0));
				m_tmpLcenterImgPoints.resize(iMarkerCodeLength, Point2(0, 0));
			}

			MarkerRuntime& operator = (const MarkerRuntime& rhs_)
			{
				if (this != &rhs_)
				{
					m_prevImgPoints = rhs_.m_prevImgPoints;
					m_prevLcenterImgPoints = rhs_.m_prevLcenterImgPoints;
					m_tmpImgPoints = rhs_.m_tmpImgPoints;
					m_tmpLcenterImgPoints = rhs_.m_tmpLcenterImgPoints;
					m_tmpImgPointsErr = rhs_.m_tmpImgPointsErr;
				}
				return *this;
			}

			MarkerRuntime& operator = (MarkerRuntime&& rhs_)
			{
				if (this != &rhs_)
				{
					m_prevImgPoints.swap(rhs_.m_prevImgPoints);
					m_prevLcenterImgPoints.swap(rhs_.m_prevLcenterImgPoints);
					m_tmpImgPoints.swap(rhs_.m_tmpImgPoints);
					m_tmpLcenterImgPoints.swap(rhs_.m_tmpLcenterImgPoints);
					m_tmpImgPointsErr.swap(rhs_.m_tmpImgPointsErr);
				}
				return *this;
			}

			int getCodeLength()
			{
				return (int)m_prevImgPoints.size() - 1;
			}
		};

		struct QuadStack
		{
			bool hole;
			bool quadGenerated;
			int this_index;
			int child_index;

			QuadStack()
				: hole(false)
				, quadGenerated(false)
				, this_index(0)
				, child_index(0)
			{
			}
		};

		struct CvCBCorner
		{
			Point2 pt;
#ifndef USE_LIBCBDETECTOR
			int row;
			int column;
#endif  USE_LIBCBDETECTOR
			//bool needsNeighbor;
			int mark;
			int imageError;

			CvCBCorner()
				: pt(0, 0)
#ifndef USE_LIBCBDETECTOR
				, row(0)
				, column(0)
#endif USE_LIBCBDETECTOR
				, mark(0)
				, imageError(0)
			{
			}

			void Reset()
			{
				pt.x = 0;
				pt.y = 0;
#ifndef USE_LIBCBDETECTOR
				row = 0;
				column = 0;
#endif  USE_LIBCBDETECTOR
				mark = 0;
				imageError = 0;
			}
		};

		struct CvCBQuad;

		enum CheckerType {
			CheckerInvalid = 0,
			CheckerWhite = 1,
			CheckerBlack = 2,
			CheckerBlackWhiteBit = 3,
			CheckerUndetermined = 4,
		};


		struct CvCBQuad
		{
			int count;
			int group_idx;
			float edge_max_sqr;
			float edge_len_sqr;
			float min_width_sqr;
			float max_width_sqr;
			float min_height_sqr;
			float max_height_sqr;
			OpenCVRect bound;
			CvCBCorner* corners[4];

			//float area;
#ifdef USE_LIBCBDETECTOR
			cbdetect::CheckerType checker_type;
#else USE_LIBCBDETECTOR
			CvCBQuad* neighbors[4];
			bool labeled;
			bool bit;
#endif USE_LIBCBDETECTOR
			//MatOfPoint bitContour;

			CvCBQuad()
				: bound(0, 0, 0, 0)
				, corners()
			{
				count = 0;
				group_idx = -1;
				edge_len_sqr = std::numeric_limits<float>::max();
				edge_max_sqr = 0;
				min_width_sqr = 0;
				max_width_sqr = 0;
				min_height_sqr = 0;
				max_height_sqr = 0;
#ifdef USE_LIBCBDETECTOR
				checker_type = cbdetect::CheckerInvalid;
#else  USE_LIBCBDETECTOR
				//area = 0.0f;
				labeled = false;
				bit = false;
#endif USE_LIBCBDETECTOR
				//bitContour = null;
				for (int i = 0; i < 4; i++)
				{
					corners[i] = nullptr;
#ifndef USE_LIBCBDETECTOR
					neighbors[i] = nullptr;
#endif  USE_LIBCBDETECTOR
				}
			}

			void Reset()
			{
				group_idx = -1;
				edge_len_sqr = std::numeric_limits<float>::max();
				edge_max_sqr = 0;
				max_width_sqr = 0;
				max_height_sqr = 0;
				bound = OpenCVRect(0, 0, 0, 0);
				for (int i = 0; i < 4; i++)
				{
					corners[i] = nullptr;
#ifndef USE_LIBCBDETECTOR
					neighbors[i] = nullptr;
#endif  USE_LIBCBDETECTOR
				}
				//area = 0.0f;
#ifdef USE_LIBCBDETECTOR
				checker_type = cbdetect::CheckerInvalid;
#else  USE_LIBCBDETECTOR
				labeled = false;
				bit = false;
#endif USE_LIBCBDETECTOR
				//bitContour = null;
			}
		};

		enum Direction
		{
			Direction_ROW = 0,
			Direction_REVERSE_ROW = 2,
			Direction_COLUMN = 1,
			Direction_REVERSE_COLUMN = 3,
		};

		struct SqrMinMaxWH
		{
			float min_width_sqr;
			float max_width_sqr;
			float min_height_sqr;
			float max_height_sqr;

			SqrMinMaxWH()
				: min_width_sqr(0.f)
				, max_width_sqr(0.f)
				, min_height_sqr(0.f)
				, max_height_sqr(0.f)
			{
			}

			void Reset()
			{
				min_width_sqr = 0.f;
				max_width_sqr = 0.f;
				min_height_sqr = 0.f;
				max_height_sqr = 0.f;
			}

			void Merge(const SqrMinMaxWH& rhs_)
			{
				min_width_sqr = std::min(min_width_sqr, rhs_.min_width_sqr);
				max_width_sqr = std::max(max_width_sqr, rhs_.max_width_sqr);
				min_height_sqr = std::min(min_height_sqr, rhs_.min_height_sqr);
				max_height_sqr = std::max(max_height_sqr, rhs_.max_height_sqr);
			}

			float MinSqr()
			{
				return std::min(min_width_sqr, min_height_sqr);
			}

			float MaxSqr()
			{
				return std::max(max_width_sqr, max_height_sqr);
			}
		};

		struct SinglePattern
		{
			int pattern_type;
#ifdef USE_LIBCBDETECTOR
			std::vector<CvCBQuad*> block_quads;
#else  USE_LIBCBDETECTOR
			std::vector<CvCBQuad*> pattern_quads;
#endif USE_LIBCBDETECTOR
			int pattern_index;
			int pattern_count;
			Direction direction;
			std::vector<SqrMinMaxWH>  pattern_widths;
			SqrMinMaxWH min_max_wh_sqr;

			SinglePattern()
				: pattern_type((int)MarkerType_INVALID)
#ifdef USE_LIBCBDETECTOR
				, block_quads()
#else  USE_LIBCBDETECTOR
				, pattern_quads()
#endif USE_LIBCBDETECTOR
				, pattern_index(-1)
				, pattern_count(0)
				, direction(Direction_ROW)
				, pattern_widths()
				, min_max_wh_sqr()
			{
			}

			SinglePattern(SinglePattern&& rhs_)
				: pattern_type(rhs_.pattern_type)
#ifdef USE_LIBCBDETECTOR
				, block_quads(std::move(rhs_.block_quads))
#else  USE_LIBCBDETECTOR
				, pattern_quads(std::move(rhs_.pattern_quads))
#endif USE_LIBCBDETECTOR
				, pattern_index(rhs_.pattern_index)
				, pattern_count(rhs_.pattern_count)
				, direction(rhs_.direction)
				, pattern_widths(std::move(rhs_.pattern_widths))
				, min_max_wh_sqr(rhs_.min_max_wh_sqr)
			{
			}

			SinglePattern& operator = (SinglePattern&& rhs_)
			{
				if (this != &rhs_)
				{
					pattern_type = rhs_.pattern_type;
#ifdef USE_LIBCBDETECTOR
					block_quads.swap(rhs_.block_quads);
#else  USE_LIBCBDETECTOR
					pattern_quads.swap(rhs_.pattern_quads);
#endif USE_LIBCBDETECTOR
					pattern_index = rhs_.pattern_index;
					pattern_count = rhs_.pattern_count;
					direction = rhs_.direction;
					pattern_widths.swap(rhs_.pattern_widths);
					min_max_wh_sqr = rhs_.min_max_wh_sqr;
				}
				return *this;
			}

			void Reset()
			{
				pattern_type = (int)MarkerType_INVALID;
#ifdef USE_LIBCBDETECTOR
				block_quads.clear();
#else  USE_LIBCBDETECTOR
				pattern_quads.clear();
#endif USE_LIBCBDETECTOR
				pattern_index = -1;
				pattern_count = 0;
				direction = Direction_ROW;
				pattern_widths.clear();
				min_max_wh_sqr.Reset();
			}
		};

		struct BitInfo
		{
#ifndef USE_LIBCBDETECTOR
			std::vector<CvCBQuad*> quads;
			int bitCount;
#endif USE_LIBCBDETECTOR

			int rowArray;
			int colArray;
			int rowNopArray;
			int colNopArray;

			BitInfo()
				: 
#ifndef USE_LIBCBDETECTOR				
				quads()
				, bitCount(0),
#endif USE_LIBCBDETECTOR				
				rowArray(0)
				, colArray(0)
				, rowNopArray(0)
				, colNopArray(0)
			{
			}

			void Reset()
			{
#ifndef USE_LIBCBDETECTOR
				quads.clear();
				bitCount = 0;
#endif USE_LIBCBDETECTOR
				rowArray = 0;
				colArray = 0;
				rowNopArray = 0;
				colNopArray = 0;
			}
		};


		struct VerifyProjectionRet
		{
			float m_dMaxEdge;
			OpenCVRect m_rect;

			VerifyProjectionRet()
				: m_dMaxEdge(0)
				, m_rect(0, 0, 0, 0)
			{
			}

			void Reset()
			{
				m_dMaxEdge = 0;
				m_rect = OpenCVRect(0, 0, 0, 0);
			}
		};

		enum HierarchyIndex
		{
			HIERARCHY_NEXT = 0,
			HIEARCHY_PREV = 1,
			HIERARCHY_CHILD = 2,
			HIERARCHY_PARENT = 3,
		};

		enum CVRUpdate
		{
			CVRUPDATE_NONE,
			CVRUPDATE_UPDATE,
			CVRUPDATE_IF_VALID,
		};

		struct WorkContext
		{
			//Matrix4x4   m_curCameraTransform;
			//Quaternion  m_curCameraRotation;

			cv::Mat		m_matCamera;

			uint		m_uTrackType;
			uint        m_uPrevValidTrackType;
			uint		m_uPossibleTrackType;
			int			m_iPrevMarkerIndex;
			int			m_iPrevMarkerCount;

			IUnknown* m_pPrevSpatialCoordinateSystem;
			int64_t	    m_i64PrevTimestamp;
			cv::Mat		m_prevGray;
			cv::Mat		m_prevOriginal;
			std::vector<cv::Mat>	m_vecPrevGrayPyr;
			uint		m_uExternalTrackType;

			std::vector<MarkerRuntime>
							m_cylinderMarkerRuntimes;
			std::vector<std::vector<MarkerRuntime>>
							m_auxliaryMarkerRuntimes;

			//List<MarkerRuntime>
			//					m_topProbeMarkerRuntimes;
			//List<MarkerRuntime>
			//					m_bottomProbeMarkerRuntimes;
			int			m_iPrevDetectFailCount;

			bool			m_bLastVisionPose;
			Vector3			m_v3LastVisionPosition;
			Quaternion		m_qLastVisionOrientation;
			cv::Mat			m_rvec;
			cv::Mat			m_tvec;
			bool			m_validRTVec;
			int			m_lastAdaptiveBlockSize;
			int			m_adaptiveBlockSizeCounter;
			OpenCVRect	m_prevImgBound;
			int64_t	    m_i64PrevImgBoundTimestamp;
			float		m_prevEdgeMax;
			OpenCVRect	m_prevCylinderMarkerImgBound;
			float		m_prevCylinderMarkerEdgeMax;
			std::vector<OpenCVRect>
				m_prevAuxiliaryMarkersImgBound;
			std::vector<float> m_prevAuxiliaryMarkersEdgeMax;
			std::vector<float> m_vecEyeGaze;
			WorkContext()
				: /*m_curCameraTransform(1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1)
				, m_curCameraRotation(0,0,0,1),*/
				m_matCamera(3, 3, CV_64FC1)

				, m_uTrackType(0)
				, m_uPrevValidTrackType(0)
				, m_uPossibleTrackType(0)
				, m_iPrevMarkerIndex(-1)
				, m_iPrevMarkerCount(0)
				, m_cylinderMarkerRuntimes()
				, m_auxliaryMarkerRuntimes()
				, m_pPrevSpatialCoordinateSystem()
				, m_i64PrevTimestamp(0)
				, m_prevGray()
				, m_prevOriginal()
				, m_vecPrevGrayPyr()
				, m_uExternalTrackType(0)

				, m_iPrevDetectFailCount(0)

				, m_bLastVisionPose(false)
				, m_v3LastVisionPosition(0, 0, 0)
				, m_qLastVisionOrientation(0, 0, 0, 1)
				, m_rvec(3, 1, CV_64FC1)
				, m_tvec(3, 1, CV_64FC1)
				, m_validRTVec(false)
				, m_lastAdaptiveBlockSize(0)
				, m_adaptiveBlockSizeCounter(0)
				, m_prevImgBound(0, 0, 0, 0)
				, m_i64PrevImgBoundTimestamp(0)
				, m_prevEdgeMax(0)
				, m_prevCylinderMarkerImgBound(0, 0, 0, 0)
				, m_prevCylinderMarkerEdgeMax(0)
				, m_prevAuxiliaryMarkersImgBound()
				, m_prevAuxiliaryMarkersEdgeMax()
				, m_vecEyeGaze()
			{
			}

			WorkContext& operator = (const WorkContext& rhs_)
			{
				if (this != &rhs_)
				{
					//this->m_curCameraTransform = rhs_.m_curCameraTransform;
					//this->m_curCameraRotation = rhs_.m_curCameraRotation;
					rhs_.m_matCamera.copyTo(this->m_matCamera);

					this->m_uTrackType = rhs_.m_uTrackType;
					this->m_uPrevValidTrackType = rhs_.m_uPrevValidTrackType;
					this->m_uPossibleTrackType = rhs_.m_uPossibleTrackType;
					this->m_iPrevMarkerIndex = rhs_.m_iPrevMarkerIndex;
					this->m_iPrevMarkerCount = rhs_.m_iPrevMarkerCount;

					this->m_cylinderMarkerRuntimes = rhs_.m_cylinderMarkerRuntimes;
					this->m_auxliaryMarkerRuntimes = rhs_.m_auxliaryMarkerRuntimes;

					{
						IUnknown* pPrev = m_pPrevSpatialCoordinateSystem;
						m_pPrevSpatialCoordinateSystem = rhs_.m_pPrevSpatialCoordinateSystem;
						if (m_pPrevSpatialCoordinateSystem != nullptr)
							m_pPrevSpatialCoordinateSystem->AddRef();
						if (pPrev != nullptr)
							pPrev->Release();
					}
					m_i64PrevTimestamp = rhs_.m_i64PrevTimestamp;

					rhs_.m_prevGray.copyTo(this->m_prevGray);
					rhs_.m_prevOriginal.copyTo(this->m_prevOriginal);
					this->m_vecPrevGrayPyr.resize(rhs_.m_vecPrevGrayPyr.size());
					for (size_t i = 0; i < rhs_.m_vecPrevGrayPyr.size(); i++)
					{
						rhs_.m_vecPrevGrayPyr[i].copyTo(this->m_vecPrevGrayPyr[i]);
					}

					this->m_uExternalTrackType = rhs_.m_uExternalTrackType;

					//deepCopy(m_topProbeMarkerRuntimes, dst.m_topProbeMarkerRuntimes);
					//deepCopy(m_bottomProbeMarkerRuntimes, dst.m_bottomProbeMarkerRuntimes);
					this->m_iPrevDetectFailCount = rhs_.m_iPrevDetectFailCount;
					if (rhs_.m_bLastVisionPose == true)
					{
						this->m_bLastVisionPose = rhs_.m_bLastVisionPose;
						this->m_v3LastVisionPosition = rhs_.m_v3LastVisionPosition;
						this->m_qLastVisionOrientation = rhs_.m_qLastVisionOrientation;
					}
					if (rhs_.m_validRTVec == true)
					{
						rhs_.m_rvec.copyTo(this->m_rvec);
						rhs_.m_tvec.copyTo(this->m_tvec);
						this->m_validRTVec = rhs_.m_validRTVec;
					}
					this->m_lastAdaptiveBlockSize = rhs_.m_lastAdaptiveBlockSize;
					this->m_adaptiveBlockSizeCounter = rhs_.m_adaptiveBlockSizeCounter;
					this->m_prevImgBound = rhs_.m_prevImgBound;
					this->m_i64PrevImgBoundTimestamp = rhs_.m_i64PrevImgBoundTimestamp;
					this->m_prevEdgeMax = rhs_.m_prevEdgeMax;
					this->m_prevCylinderMarkerImgBound = rhs_.m_prevCylinderMarkerImgBound;
					this->m_prevCylinderMarkerEdgeMax = rhs_.m_prevCylinderMarkerEdgeMax;

					this->m_prevAuxiliaryMarkersImgBound = rhs_.m_prevAuxiliaryMarkersImgBound;
					this->m_prevAuxiliaryMarkersEdgeMax = rhs_.m_prevAuxiliaryMarkersEdgeMax;
					this->m_vecEyeGaze = rhs_.m_vecEyeGaze;
				}
				return *this;
			}
		};

		struct LabelQuadGroupData
		{
			int		aBitIndices;
			int		aIndices;
			CvCBQuad* aQuadIndices;

			LabelQuadGroupData()
				: aBitIndices(0)
				, aIndices(0)
				, aQuadIndices(nullptr)
			{
			}
		};


		Parameter		m_parameters__fixed;
		uint			m_uMarkerMask__fixed;
		uint			m_uAuxMarkerMask__fixed;
		uint			m_uAuxPatternMarkerMask__fixed;
		uint			m_uAuxNoPatternMarkerMask__fixed;
		int             m_iCheckerBoardMaxDimension__fixed;
		int				m_iCheckerBoardMinDimension__fixed;
		int				m_iCheckerBoardMinBlocks__fixed;

		std::vector<CodeClass>
			m_dictionaryMajor__fixed;
		std::vector<CodeClass>
			m_dictionaryMinor__fixed;

		cv::Mat			m_matCrossKernel__fixed;
		cv::Mat			m_matRectKernel__fixed;


		std::vector<MarkerInfo>
			m_cylinderarkerInfos__fixed;


		std::vector<std::vector<MarkerInfo>>
			m_auxliaryMarkerInfos__fixed;

		//List<MarkerInfo>
		//						m_topProbeMarkerInfos__fixed;
		//Point3			m_topProbeMarkerNormal__fixed;
		//List<MarkerInfo>
		//						m_bottomProbeMarkerInfos__fixed;
		//Point3			m_bottomProbeMarkerNormal__fixed;




		std::mutex				m_mutexTrackedPost__mutex;
		TrackedPoseNativeStruct	m_trackedPose__mutex;

		std::atomic<int>		m_iTerminated__atomic;
		std::atomic<int>		m_iResetLastAdaptiveBlockSize__atomic;


		std::mutex				m_mutexNewBoundPoints__mutex;
		BoundPoints				m_newBoundPoints__mutex;
		int                     m_iBoundPointsStamp__mutex;

		int						m_iBoundPointStamp__detect;

		uint			m_uContextStamp__track;
		WorkContext		m_context__track;

		int				m_iTrackSkipFrameCounter__track;

		std::vector<Point2>		m_prePointsRuntime__track;
		std::vector<Byte>	m_opticalFlowStatusRuntime__track;
		std::vector<Byte>	m_opticalFlowStatus2Runtime__track;
		std::vector<float>	m_opticalFlowErrRuntime__track;
		std::vector<Point2>		m_imgPointsRuntime__track;
		std::vector<Point3>		m_modelPointsRuntime__track;
		std::vector<Point2>		m_vecp2Temp__track;
		std::vector<Point3>		m_vecp3Temp__track;
		std::vector<VerifyProjectionRet> m_vecVerifyProjectionRet__track;
		std::vector<int>		m_veciTemp__track;
		std::vector<int>		m_veci2Temp__track;
		std::vector<bool>		m_vecbTemp__track;
		std::vector<Byte>		m_vecbyTemp__track;
		int64_t					m_lastTrackedPose_m_i64Timestamp__track;
		int						m_lastTrackedPose_m_eState__track;
		Vector3					m_lastTrackedPose_m_v3Position__track;
		Quaternion				m_lastTrackedPose_m_qOrientation__track;
		FrameInfoNativeStruct	m_info__track;

		std::thread     m_thread__track;
		std::mutex		m_mutex__track;
		std::condition_variable	m_condvar__track;
		//bool						m_bSaveImage;



		WorkContext		m_context__detect;

#ifndef USE_LIBCBDETECTOR
		cv::Mat m_aCurThresholded__detect[2];
#endif USE_LIBCBDETECTOR

		std::vector<cv::Vec4i>	m_hierarchy__detect;
		std::vector<QuadStack>	m_quadStack__detect;
		std::vector<Point2i>	m_approxPoly2Temp__detect;
		std::vector<Point2i>	m_approxPoly2Temp2__detect;

		std::vector<CvCBQuad>	m_quadsBase__detect;
		std::vector<CvCBCorner>	m_cornersBase__detect;
		std::vector<CvCBQuad*>	m_quadGroup__detect;

		std::vector<CvCBQuad*>	m_findConnectedQuad__detect;

		std::list<SinglePattern>
			m_singlePatternBase__detect;
		std::vector<SinglePattern*>
			m_foundPattern__detect;
		std::vector<SinglePattern*>
			m_markerPatternsRuntime__detect;
		std::vector<SinglePattern*> m_rowPatternsRuntime__detect;
		std::vector<SinglePattern*>m_reverseRowPatternsRuntime__detect;
		std::vector<SinglePattern*>m_columnPatternsRuntime__detect;
		std::vector<SinglePattern*>m_reverseColumnPatternsRuntime__detect;

		std::vector< BitInfo>	m_vecBitInfoBase__detect;

#ifndef USE_LIBCBDETECTOR
		std::vector<LabelQuadGroupData>	m_vecLabelQuadGroupData__detect;
#endif USE_LIBCBDETECTOR
		std::vector<VerifyProjectionRet> m_vecVerifyProjectionRet__detect;

		std::vector<Point2>		m_imgPointsRuntime__detect;
		std::vector<Point3>		m_modelPointsRuntime__detect;

		std::vector< OpenCVRect>	m_vecRectRuntime__detect;
		std::vector<float>		m_vecFloatRuntime__detect;

		std::vector<int>		m_veciTemp__detect;
		std::vector<int>		m_veci2Temp__detect;
		std::vector<bool>		m_vecbTemp__detect;
		std::vector<Byte>		m_vecbyTemp__detect;


		std::vector<Point2>		m_vecp2Temp__detect;

#ifdef USE_LIBCBDETECTOR
		cbdetect::Params                m_cbdetectParams__fixed;
		cbdetect::Corner				m_corner__detect;
		cbdetect::Corner				m_cornerResized__detect;
		std::vector<cbdetect::Board>	m_boards__detect;
#endif USE_LIBCBDETECTOR


		std::thread		m_thread__detect;
		std::mutex		m_mutex__detect;
		std::condition_variable	m_condvar__detect;
		uint			m_uContextStamp__monitor;

		//MainScript		 m_mainScript = null;
		//UnityAction		m_mainScriptFrameAction = null;

		//static NewCylindricalProbeTracker()
		//{
		//	ms_cvMatEmpty__fixed = new Mat();
		//}

		UpdateResultGrayCB	m_updateResultGrayCB;
		GetFrameInfoStampCB	m_getFrameInfoStampCB;
		GetFrameInfoNativeStructCB m_getFrameInfoNativeStructCB;
		TransformPoseToGlobalCoordinateCB m_transformPoseToGlobalCoordinateCB;
		LocateAtTimestampSpatialCB m_locateAtTimestampSpatialCB;
		SetBoundRectCB m_setBoundRectCB;
	public:

		NewCylindricalProbeTracker(const Parameter& _params
			, UpdateResultGrayCB updateResultGrayCB
			, GetFrameInfoStampCB getFrameInfoStampCB
			, GetFrameInfoNativeStructCB getFrameInfoNativeStructCB
			, TransformPoseToGlobalCoordinateCB transformPoseToGlobalCoordinateCB
			, LocateAtTimestampSpatialCB locateAtTimestampSpatialCB
			, SetBoundRectCB setBoundRectCB)
			: m_parameters__fixed(_params)
			, m_updateResultGrayCB(updateResultGrayCB)
			, m_getFrameInfoStampCB(getFrameInfoStampCB)
			, m_getFrameInfoNativeStructCB(getFrameInfoNativeStructCB)
			, m_transformPoseToGlobalCoordinateCB(transformPoseToGlobalCoordinateCB)
			, m_locateAtTimestampSpatialCB(locateAtTimestampSpatialCB)
			, m_setBoundRectCB(setBoundRectCB)
			, m_uMarkerMask__fixed(0)
			, m_uAuxMarkerMask__fixed(0)
			, m_uAuxPatternMarkerMask__fixed(0)
			, m_uAuxNoPatternMarkerMask__fixed(0)
			, m_iCheckerBoardMaxDimension__fixed(0)
			, m_iCheckerBoardMinDimension__fixed(0)
			, m_iCheckerBoardMinBlocks__fixed(0)
			, m_dictionaryMajor__fixed()
			, m_dictionaryMinor__fixed()
			, m_matCrossKernel__fixed()
			, m_matRectKernel__fixed()
			, m_cylinderarkerInfos__fixed()
			, m_auxliaryMarkerInfos__fixed()
			, m_mutexTrackedPost__mutex()
			, m_trackedPose__mutex()

			, m_iTerminated__atomic(0)
			, m_iResetLastAdaptiveBlockSize__atomic(0)

			, m_mutexNewBoundPoints__mutex()
			, m_newBoundPoints__mutex()
			, m_iBoundPointsStamp__mutex(0)

			, m_uContextStamp__track(0)
			, m_context__track()
			, m_iTrackSkipFrameCounter__track(0)
			, m_prePointsRuntime__track()
			, m_opticalFlowStatusRuntime__track()
			, m_opticalFlowStatus2Runtime__track()
			, m_opticalFlowErrRuntime__track()
			, m_imgPointsRuntime__track()
			, m_modelPointsRuntime__track()
			, m_vecp2Temp__track()
			, m_vecp3Temp__track()
			, m_vecVerifyProjectionRet__track()
			, m_veciTemp__track()
			, m_veci2Temp__track()
			, m_vecbTemp__track()
			, m_vecbyTemp__track()
			, m_lastTrackedPose_m_i64Timestamp__track(0)
			, m_lastTrackedPose_m_eState__track(TrackedPose_INVALID)
			, m_lastTrackedPose_m_v3Position__track(0, 0, 0)
			, m_lastTrackedPose_m_qOrientation__track(0, 0, 0, 1)
			, m_info__track()
			, m_thread__track()
			, m_mutex__track()
			, m_condvar__track()
			//, m_bSaveImage(false)

			, m_context__detect()
#ifndef USE_LIBCBDETECTOR
			, m_aCurThresholded__detect()
#endif USE_LIBCBDETECTOR
			, m_hierarchy__detect()
			, m_quadStack__detect()
			, m_approxPoly2Temp__detect()
			, m_approxPoly2Temp2__detect()
			, m_quadsBase__detect()
			, m_cornersBase__detect()
			, m_quadGroup__detect()
			, m_findConnectedQuad__detect()
			, m_singlePatternBase__detect()
			, m_foundPattern__detect()
			, m_markerPatternsRuntime__detect()
			, m_rowPatternsRuntime__detect()
			, m_reverseRowPatternsRuntime__detect()
			, m_columnPatternsRuntime__detect()
			, m_reverseColumnPatternsRuntime__detect()

			, m_vecBitInfoBase__detect()
			, m_iBoundPointStamp__detect(0)
#ifndef USE_LIBCBDETECTOR
			, m_vecLabelQuadGroupData__detect()
#endif USE_LIBCBDETECTOR
			, m_vecVerifyProjectionRet__detect()
			, m_vecRectRuntime__detect()
			, m_vecFloatRuntime__detect()
			, m_veciTemp__detect()
			, m_veci2Temp__detect()
			, m_vecbTemp__detect()
			, m_vecbyTemp__detect()
			, m_vecp2Temp__detect()
			, m_thread__detect()
			, m_mutex__detect()
			, m_condvar__detect()
			, m_uContextStamp__monitor(0)
		{
			int iMarkerCodeLength = m_parameters__fixed.markerCodeLength;
			int iMajorCodeLength = (iMarkerCodeLength + 1) >> 1;
			int iMinorCodeLength = iMarkerCodeLength - iMajorCodeLength;
			m_iCheckerBoardMinBlocks__fixed = iMarkerCodeLength;
			m_iCheckerBoardMinDimension__fixed = 2;
			m_iCheckerBoardMaxDimension__fixed = iMarkerCodeLength;

			{
				int iCodeClassNum = 1 << iMarkerCodeLength;
				uint patternMask = (uint)(iCodeClassNum - 1);
				m_dictionaryMajor__fixed.resize(iCodeClassNum);
				m_dictionaryMinor__fixed.resize(iCodeClassNum);
				for (int i = 0; i < iCodeClassNum; i++)
				{
					m_dictionaryMajor__fixed[i].m_type = (int)MarkerType_INVALID;
					m_dictionaryMajor__fixed[i].m_index = -1;
					m_dictionaryMinor__fixed[i].m_type = (int)MarkerType_INVALID;
					m_dictionaryMinor__fixed[i].m_index = -1;
				}
				bool bMajor = m_parameters__fixed.markerCodeStartMajor;
				for (int i = 0; i < (int)m_parameters__fixed.markerCodePatterns.size(); i++, bMajor = !bMajor)
				{
					uint pattern = m_parameters__fixed.markerCodePatterns[i];
					pattern &= patternMask;
					if (bMajor == true)
					{
						//Assert.IsTrue(m_dictionaryMajor__fixed[pattern].m_type == (int)MarkerType_INVALID);
						m_dictionaryMajor__fixed[pattern].m_type = (int)MarkerType_CYLINDER;
						m_dictionaryMajor__fixed[pattern].m_index = i;
					}
					else
					{
						//Assert.IsTrue(m_dictionaryMinor__fixed[pattern].m_type == (int)MarkerType_INVALID);
						m_dictionaryMinor__fixed[pattern].m_type = (int)MarkerType_CYLINDER;
						m_dictionaryMinor__fixed[pattern].m_index = i;
					}
				}
				for (int aux = 0; aux < (int)m_parameters__fixed.auxiliaryMarkerSettings.size(); aux++)
				{
					switch (m_parameters__fixed.auxiliaryMarkerSettings[aux].type)
					{
					case AuxiliaryMarker::CHECKER:
					{
						uint pattern = m_parameters__fixed.auxiliaryMarkerSettings[aux].code & patternMask;
						if (m_parameters__fixed.auxiliaryMarkerSettings[aux].codeMajor)
						{
							//Assert.IsTrue(m_dictionaryMajor__fixed[pattern].m_type == (int)MarkerType_INVALID);
							m_dictionaryMajor__fixed[pattern].m_type = (int)MarkerType_AUX_BASE + aux;
							m_dictionaryMajor__fixed[pattern].m_index = 1;
						}
						else
						{
							//Assert.IsTrue(m_dictionaryMinor__fixed[pattern].m_type == (int)MarkerType_INVALID);
							m_dictionaryMinor__fixed[pattern].m_type = (int)MarkerType_AUX_BASE + aux;
							m_dictionaryMinor__fixed[pattern].m_index = 1;
						}
					}
					break;
					case AuxiliaryMarker::NOPATTERN:
					{
					}
					break;
					}
				}
			}

			float dAngleCir = (float)(m_parameters__fixed.markerSize / m_parameters__fixed.radius);
			float dBlobCir = (float)((m_parameters__fixed.blobRadius * 2) / m_parameters__fixed.radius);

			m_matCrossKernel__fixed = cv::getStructuringElement(cv::MORPH_CROSS, Size(3, 3), Point2i(1, 1));
			m_matRectKernel__fixed = cv::getStructuringElement(cv::MORPH_RECT, Size(3, 3), Point2i(1, 1));

			m_cylinderarkerInfos__fixed.reserve(m_parameters__fixed.markerCodePatterns.size());

			m_auxliaryMarkerInfos__fixed.reserve(m_parameters__fixed.auxiliaryMarkerSettings.size());

			//m_topProbeMarkerInfos__fixed = null;
			//m_bottomProbeMarkerInfos__fixed = null;
			//m_topProbeMarkerNormal__fixed = new Point3(0, 0, -1);
			//m_bottomProbeMarkerNormal__fixed = new Point3(0, 0, 1);

			m_uContextStamp__track = 0;
			//m_context__track = new WorkContext();

			//m_iResetLastAdaptiveBlockSize__atomic.exchage(0);
			m_iTrackSkipFrameCounter__track = 0;

			{
				float dAngle = (float)m_parameters__fixed.markerCodeStartAngle * MY_DEG2RAD;
				uint uCodeMask = (1U << iMarkerCodeLength) - 1U;

				bool bMajor = m_parameters__fixed.markerCodeStartMajor;
				for (int i = 0; i < (int)m_parameters__fixed.markerCodePatterns.size(); i++, dAngle -= dAngleCir, bMajor = !bMajor)
				{
					MarkerInfo markerInfo(iMarkerCodeLength);
					MarkerRuntime markerRuntime(iMarkerCodeLength);
					markerInfo.m_bMajor = bMajor;

					Point3 normal(std::cos(dAngle), std::sin(dAngle), 0.0f);
					float rx = (float)m_parameters__fixed.radius * std::cos(dAngle + dAngleCir * 0.5f);
					float ry = (float)m_parameters__fixed.radius * std::sin(dAngle + dAngleCir * 0.5f);


					for (int c = 0; c <= iMarkerCodeLength; c++)
					{
						float x = normal.x * (float)m_parameters__fixed.radius;
						float y = normal.y * (float)m_parameters__fixed.radius;
						float z = c * (float)m_parameters__fixed.markerSize;
						markerInfo.m_corner3DPoints[c] = Point3(x, y, z);
						markerInfo.m_corner3DNormals[c] = normal;
						//if (c != iMarkerCodeLength)
						//{
						//	double rz = (c + 0.5) * m_parameters__fixed.markerSize;
						//	markerInfo.m_lcenter3DPoints.Add(new Point3(rx, ry, rz));
						//}
					}

					m_cylinderarkerInfos__fixed.push_back(std::move(markerInfo));
					m_context__track.m_cylinderMarkerRuntimes.push_back(std::move(markerRuntime));
				}
			}

			m_uAuxMarkerMask__fixed = 0;
			m_uAuxPatternMarkerMask__fixed = 0;
			m_uAuxNoPatternMarkerMask__fixed = 0;

			if (m_parameters__fixed.auxiliaryMarkerSettings.empty() == false)
			{
				cv::Mat cvMatTmp3x1_64F(3, 1, CV_64FC1);

				for (int aux = 0; aux < (int)m_parameters__fixed.auxiliaryMarkerSettings.size(); aux++)
				{
					uint flag = (uint)(1 << ((int)MarkerType_AUX_BASE + aux));
					m_uAuxMarkerMask__fixed |= flag;
					int iAuxMarkerCodeLength = m_parameters__fixed.auxiliaryMarkerSettings[aux].checkerboardXLength;
					float fAuxMarkerSize = (m_parameters__fixed.auxiliaryMarkerSettings[aux].checkerboardMarkerSize > 0.0f)
						? m_parameters__fixed.auxiliaryMarkerSettings[aux].checkerboardMarkerSize : m_parameters__fixed.markerSize;
					int iYHeight = (int)m_parameters__fixed.auxiliaryMarkerSettings[aux].checkerboardCoordinates.size() - 1;

					int iDimensionMin = std::min(iYHeight, iAuxMarkerCodeLength);
					int iDimensionMax = std::max(iYHeight, iAuxMarkerCodeLength);

					int iNumBlocks = EstimateCheckerBoardNumBlocks(iAuxMarkerCodeLength, iYHeight, m_parameters__fixed.auxiliaryMarkerSettings[aux].codeMajor);
					m_iCheckerBoardMinBlocks__fixed = std::min(m_iCheckerBoardMinBlocks__fixed, iNumBlocks);
					m_iCheckerBoardMinDimension__fixed = std::min(iDimensionMin, m_iCheckerBoardMinDimension__fixed);
					m_iCheckerBoardMaxDimension__fixed = std::min(iDimensionMax, m_iCheckerBoardMaxDimension__fixed);

					switch (m_parameters__fixed.auxiliaryMarkerSettings[aux].type)
					{
					case AuxiliaryMarker::CHECKER:
					case AuxiliaryMarker::NOPATTERN:
					{
						if (m_parameters__fixed.auxiliaryMarkerSettings[aux].type == AuxiliaryMarker::CHECKER)
							m_uAuxPatternMarkerMask__fixed |= flag;
						else
							m_uAuxNoPatternMarkerMask__fixed |= flag;

						std::vector<float> aInvRadius; aInvRadius.resize(iYHeight + 1);
						std::vector<float> aZ; aZ.resize(iYHeight + 1);
						std::vector<float> aYmiddle; aYmiddle.resize(iYHeight + 1);
						std::vector<float> aYouter; aYouter.resize(iYHeight + 1);
						std::vector<bool> aBoolArc; aBoolArc.resize(iYHeight + 1);
						std::vector<float> aXouter; aXouter.resize(iYHeight + 1);
						//float[] aCenterZmiddle = new float[iYHeight];
						//float[] aCenterYmiddle = new float[iYHeight];
						//float[] aCenterZouter = new float[iYHeight];
						//float[] aCenterYouter = new float[iYHeight];
						//float[] aCenterInvRadius = new float[iYHeight];

						for (int y = 0; y <= iYHeight; y++)
						{
							aZ[y] = m_parameters__fixed.auxiliaryMarkerSettings[aux].checkerboardCoordinates[y].outerZ;
							aYmiddle[y] = m_parameters__fixed.auxiliaryMarkerSettings[aux].checkerboardCoordinates[y].middleY;
							aYouter[y] = m_parameters__fixed.auxiliaryMarkerSettings[aux].checkerboardCoordinates[y].outerY;
							aBoolArc[y] = m_parameters__fixed.auxiliaryMarkerSettings[aux].checkerboardCoordinates[y].outerX == 0;
							aXouter[y] = (aBoolArc[y] == true)
								? fAuxMarkerSize * iAuxMarkerCodeLength * 0.5f
								: m_parameters__fixed.auxiliaryMarkerSettings[aux].checkerboardCoordinates[y].outerX;
						}

						std::vector<double> vecRotXYZ;
						vecRotXYZ.push_back(m_parameters__fixed.auxiliaryMarkerSettings[aux].angVecX);
						vecRotXYZ.push_back(m_parameters__fixed.auxiliaryMarkerSettings[aux].angVecY);
						vecRotXYZ.push_back(m_parameters__fixed.auxiliaryMarkerSettings[aux].angVecZ);
						cv::Mat matRot(3, 3, CV_64FC1);
						cv::Rodrigues(vecRotXYZ, matRot);
						//cv::Mat matRot = computeZXYToRotation(-m_parameters__fixed.auxiliaryMarkerSettings[aux].eulerZ,
						//	-m_parameters__fixed.auxiliaryMarkerSettings[aux].eulerX,
						//	m_parameters__fixed.auxiliaryMarkerSettings[aux].eulerY);

						//{

						//	if (iYHeight  != 2)
						//	{
						//		Debug.LogAssertion("NewCylindricalProbeTracker.Start : IYLength != 2 ");
						//	}
						//	else
						//	{
						//		double cx, cy;
						//		double r = estimateFittingCircle(aZ[0], aYmiddle[0], aZ[1], aYmiddle[1], aZ[2], aYmiddle[2]
						//		, /*out*/cx, /*out*/cy);
						//		for (int i = 0; i < iYHeight; i++)
						//		{
						//			Point p = new Point(0.5 * (aZ[i] + aZ[i + 1]) - cx, 0.5 * (aYmiddle[i] + aYmiddle[i + 1]) - cy);
						//			normalize(p);
						//			aCenterZmiddle[i] = p.x * r + cx;
						//			aCenterYmiddle[i] = p.y * r + cy;
						//		}
						//	}
						//}
						//{
						//	if (iYHeight != 2)
						//	{
						//		Debug.LogAssertion("NewCylindricalProbeTracker.Start : IYLength != 2 ");
						//	}
						//	else
						//	{
						//		double cx, cy;
						//		double r = estimateFittingCircle(aZ[0], aYouter[0], aZ[1], aYouter[1], aZ[2], aYouter[2]
						//			, /*out*/cx, /*out*/cy);
						//		for (int i = 0; i < iYHeight; i++)
						//		{
						//			Point p = new Point(0.5 * (aZ[i] + aZ[i + 1]) - cx, 0.5 * (aYouter[i] + aYouter[i + 1]) - cy);
						//			normalize(p);
						//			aCenterZouter[i] = p.x * r + cx;
						//			aCenterYouter[i] = p.y * r + cy;
						//		}
						//	}
						//}

						for (int i = 0; i <= iYHeight; i++)
						{
							aInvRadius[i] = (aBoolArc[i] == true) ? estimateInvRadius(aYmiddle[i], aYouter[i], aXouter[i])
								: estimateInvRadiusFromChord(aYmiddle[i], aYouter[i], /*ref*/ aXouter[i]);
						}
						//for (int i = 0; i < iYHeight; i++)
						//{
						//	aCenterInvRadius[i] = estimateInvRadius(aCenterYmiddle[i], aCenterYouter[i], ( aXouter[i] + aXouter[i+1]) * 0.5);
						//}

						std::vector<MarkerInfo> auxMarkerInfo;
						bool bMajor = !m_parameters__fixed.auxiliaryMarkerSettings[aux].codeMajor;
						std::vector<MarkerRuntime> auxMarkerRuntime;


						for (int i = 0; i <= iYHeight; i++, bMajor = !bMajor)
						{
							MarkerInfo markerInfo(iAuxMarkerCodeLength);
							MarkerRuntime markerRuntime(iAuxMarkerCodeLength);
							markerInfo.m_bMajor = bMajor;

							Point3 p3Tmp(0.0f, (float)-aYmiddle[i], (float)aZ[i]);
							p3Tmp = computeRotation(matRot, p3Tmp);
							//p3Tmp.x += m_parameters__fixed.auxiliaryMarkerSettings[aux].pivotX;
							//p3Tmp.y -= m_parameters__fixed.auxiliaryMarkerSettings[aux].pivotY;
							//p3Tmp.Z += m_parameters__fixed.auxiliaryMarkerSettings[aux].pivotZ;
							//p3Tmp.CopyTo(markerInfo.m_marker);
							for (int c = 0; c <= iAuxMarkerCodeLength; c++)
							{
								float arc = aXouter[i] * 2.0f * (float)(0.5f * iAuxMarkerCodeLength - c) / (float)iAuxMarkerCodeLength;
								float x = (aInvRadius[i] == 0.0f) ? arc : ((1.0f / aInvRadius[i]) * std::sin(arc * aInvRadius[i]));
								float y = (aInvRadius[i] == 0.0f) ? (-aYmiddle[i]) : (-aYmiddle[i] + (1.0f / aInvRadius[i]) * (1 - std::cos(arc * aInvRadius[i])));
								{
									Point3 p3(x, y, aZ[i]);
									p3 = computeRotation(matRot, p3);
									p3.x += (float)m_parameters__fixed.auxiliaryMarkerSettings[aux].pivotX;
									p3.y -= (float)m_parameters__fixed.auxiliaryMarkerSettings[aux].pivotY;
									p3.z += (float)m_parameters__fixed.auxiliaryMarkerSettings[aux].pivotZ;
									markerInfo.m_corner3DPoints[c] = p3;
								}
								//if (i != 0 && c != iAuxMarkerCodeLength)
								//{
								//	//float dInvRadius = 0.5 * aCenterInvRadius[i - 1];
								//	float dInvRadius = aCenterInvRadius[i - 1];
								//	float rarc = (aXouter[i-1] + aXouter[i]) * (float)(0.5 * iAuxMarkerCodeLength - c - 0.5) / (float)iAuxMarkerCodeLength;
								//	float rx = (dInvRadius == 0.0) ? rarc : ((1.0 / dInvRadius) * std::sin(rarc * dInvRadius));
								//	float ry = (dInvRadius == 0.0) ? (-aCenterYmiddle[i - 1]) : (-aCenterYmiddle[i - 1] + (1.0 / dInvRadius) * (1 - std::cos(rarc * dInvRadius)));

								//	{
								//		Point3 p3 = new Point3();
								//		p3.x = rx;
								//		p3.y = ry;
								//		p3.Z = 0.5 * (aZ[i - 1] + aZ[i]);
								//		computeRotation(matRot, p3, p3, cvMatTmp3x1_64F);
								//		p3.x += m_parameters__fixed.auxiliaryMarkerSettings[aux].pivotX;
								//		p3.y -= m_parameters__fixed.auxiliaryMarkerSettings[aux].pivotY;
								//		p3.Z += m_parameters__fixed.auxiliaryMarkerSettings[aux].pivotZ;
								//		markerInfo.m_lcenter3DPoints.Add(p3);
								//	}
								//}
							}
							auxMarkerInfo.push_back(std::move(markerInfo));
							auxMarkerRuntime.push_back(std::move(markerRuntime));
						}


						for (int i = 0; i <= iYHeight; i++)
						{
							for (int c = 0; c < (int)auxMarkerInfo[i].m_corner3DPoints.size(); c++)
							{
								Point3 vx;
								Point3 vz;
								if (c == 0)
								{
									vx = auxMarkerInfo[i].m_corner3DPoints[c] - auxMarkerInfo[i].m_corner3DPoints[c + 1];
								}
								else if (c == (int)auxMarkerInfo[i].m_corner3DPoints.size() - 1)
								{
									vx = auxMarkerInfo[i].m_corner3DPoints[c - 1] - auxMarkerInfo[i].m_corner3DPoints[c];
								}
								else
								{
									vx = auxMarkerInfo[i].m_corner3DPoints[c - 1] - auxMarkerInfo[i].m_corner3DPoints[c + 1];
								}

								if (i == 0)
								{
									vz = auxMarkerInfo[i].m_corner3DPoints[c] - auxMarkerInfo[i + 1].m_corner3DPoints[c];
								}
								else if (i == iYHeight)
								{
									vz = auxMarkerInfo[i - 1].m_corner3DPoints[c] - auxMarkerInfo[i].m_corner3DPoints[c];
								}
								else
								{
									vz = auxMarkerInfo[i - 1].m_corner3DPoints[c] - auxMarkerInfo[i + 1].m_corner3DPoints[c];
								}
								Point3 vnormal = vx.cross(vz);
								Normalize(/*ref*/ vnormal);
								auxMarkerInfo[i].m_corner3DNormals[c] = vnormal;
							}
						}

						m_auxliaryMarkerInfos__fixed.push_back(std::move(auxMarkerInfo));
						m_context__track.m_auxliaryMarkerRuntimes.push_back(std::move(auxMarkerRuntime));
						m_context__track.m_prevAuxiliaryMarkersImgBound.push_back(OpenCVRect(0, 0, 0, 0));
						m_context__track.m_prevAuxiliaryMarkersEdgeMax.push_back(0.0f);
					}
					break;
					//case AuxiliaryMarker::NOPATTERN:
					//	{
					//		m_uAuxNoPatternMarkerMask__fixed |= flag;

					//		double[] aInvRadius = new double[iYHeight+1];
					//		double[] aZ = new double[iYHeight+1];
					//		double[] aYmiddle = new double[iYHeight+1];
					//		double[] aYouter = new double[iYHeight+1];
					//		bool[] aBoolArc = new bool[iYHeight+1];
					//		double[] aXouter = new double[iYHeight+1];

					//		for (int y = 0; y <= iYHeight; y++)
					//		{
					//			aZ[y] = m_parameters__fixed.auxiliaryMarkerSettings[aux].checkerboardCoordinates[y].outerZ;
					//			aYmiddle[y] = m_parameters__fixed.auxiliaryMarkerSettings[aux].checkerboardCoordinates[y].middleY;
					//			aYouter[y] = m_parameters__fixed.auxiliaryMarkerSettings[aux].checkerboardCoordinates[y].outerY;
					//			aBoolArc[y] = m_parameters__fixed.auxiliaryMarkerSettings[aux].checkerboardCoordinates[y].outerX == 0;
					//			aXouter[y] = (aBoolArc[y] == true)
					//					? m_parameters__fixed.markerSize * iAuxMarkerCodeLength * 0.5
					//					: m_parameters__fixed.auxiliaryMarkerSettings[aux].checkerboardCoordinates[y].outerX;
					//		}

					//		Mat matRot = computeZXYToRotation(-m_parameters__fixed.auxiliaryMarkerSettings[aux].eulerZ,
					//			-m_parameters__fixed.auxiliaryMarkerSettings[aux].eulerX,
					//			m_parameters__fixed.auxiliaryMarkerSettings[aux].eulerY);


					//		for (int i = 0; i <= iYHeight; i++)
					//		{
					//			aInvRadius[i] = (aBoolArc[i] == true) ? estimateInvRadius(aYmiddle[i], aYouter[i], aXouter[i])
					//					: estimateInvRadiusFromChord(aYmiddle[i], aYouter[i], ref aXouter[i]);
					//		}

					//		List<MarkerInfo> auxMarkerInfo = new List<MarkerInfo>(3);
					//		bool bMajor = !m_parameters__fixed.auxiliaryMarkerSettings[aux].codeMajor;
					//		List<MarkerRuntime> auxMarkerRuntime = new List<MarkerRuntime>(3);


					//		for (int i = 0; i <= iYHeight; i++, bMajor = !bMajor)
					//		{
					//			MarkerInfo markerInfo = new MarkerInfo(iAuxMarkerCodeLength);
					//			MarkerRuntime markerRuntime = new MarkerRuntime(iAuxMarkerCodeLength);
					//			markerInfo.m_bMajor = bMajor;

					//			Point3 p3Tmp = new Point3(0.0, -aYmiddle[i], aZ[i]);
					//			computeRotation(matRot, p3Tmp, p3Tmp, cvMatTmp3x1_64F);
					//			p3Tmp.x += m_parameters__fixed.auxiliaryMarkerSettings[aux].pivotX;
					//			p3Tmp.y -= m_parameters__fixed.auxiliaryMarkerSettings[aux].pivotY;
					//			p3Tmp.Z += m_parameters__fixed.auxiliaryMarkerSettings[aux].pivotZ;
					//			p3Tmp.CopyTo(markerInfo.m_marker);
					//			for (int c = 0; c <= iAuxMarkerCodeLength; c++)
					//			{
					//				double arc = aXouter[i] * 2.0 * (double)(0.5 * iAuxMarkerCodeLength - c) / (double)iAuxMarkerCodeLength;
					//				double x = (aInvRadius[i] == 0.0) ? arc : ((1.0 / aInvRadius[i]) * std::sin(arc * aInvRadius[i]));
					//				double y = (aInvRadius[i] == 0.0) ? (-aYmiddle[i]) : (-aYmiddle[i] + (1.0 / aInvRadius[i]) * (1 - std::cos(arc * aInvRadius[i])));
					//				{
					//					Point3 p3 = new Point3();
					//					p3.x = x;
					//					p3.y = y;
					//					p3.Z = aZ[i];
					//					computeRotation(matRot, p3, p3, cvMatTmp3x1_64F);
					//					p3.x += m_parameters__fixed.auxiliaryMarkerSettings[aux].pivotX;
					//					p3.y -= m_parameters__fixed.auxiliaryMarkerSettings[aux].pivotY;
					//					p3.Z += m_parameters__fixed.auxiliaryMarkerSettings[aux].pivotZ;
					//					markerInfo.m_corner3DPoints.Add(p3);
					//				}
					//			}
					//			auxMarkerInfo.Add(markerInfo);
					//			auxMarkerRuntime.Add(markerRuntime);
					//		}
					//		m_auxliaryMarkerInfos__fixed.Add(auxMarkerInfo);
					//		m_context__track.m_auxliaryMarkerRuntimes.Add(auxMarkerRuntime);
					//		m_context__track.m_prevAuxiliaryMarkersImgBound.Add(new OpenCVRect());
					//		m_context__track.m_prevAuxiliaryMarkersEdgeMax.Add(0.0);

					//		for (int i = 0; i <= iYHeight; i++)
					//		{
					//			for (int c = 0; c < auxMarkerInfo[i].m_corner3DPoints.size(); c++)
					//			{
					//				Point3 vx = null;
					//				Point3 vz = null;
					//				if (c == 0)
					//				{
					//					vx = auxMarkerInfo[i].m_corner3DPoints[c] - auxMarkerInfo[i].m_corner3DPoints[c + 1];
					//				}
					//				else if (c == auxMarkerInfo[i].m_corner3DPoints.size() - 1)
					//				{
					//					vx = auxMarkerInfo[i].m_corner3DPoints[c - 1] - auxMarkerInfo[i].m_corner3DPoints[c];
					//				}
					//				else
					//				{
					//					vx = auxMarkerInfo[i].m_corner3DPoints[c - 1] - auxMarkerInfo[i].m_corner3DPoints[c + 1];
					//				}

					//				if (i == 0)
					//				{
					//					vz = auxMarkerInfo[i].m_corner3DPoints[c] - auxMarkerInfo[i + 1].m_corner3DPoints[c];
					//				}
					//				else if (i == 2)
					//				{
					//					vz = auxMarkerInfo[i - 1].m_corner3DPoints[c] - auxMarkerInfo[i].m_corner3DPoints[c];
					//				}
					//				else
					//				{
					//					vz = auxMarkerInfo[i - 1].m_corner3DPoints[c] - auxMarkerInfo[i + 1].m_corner3DPoints[c];
					//				}
					//				Point3 vnormal = vx.cross(vz);
					//				normalize(vnormal);
					//				auxMarkerInfo[i].m_corner3DNormals.Add(vnormal);
					//			}
					//		}
					//	}
					//	break;
					}
				}
			}

			m_uMarkerMask__fixed = m_uAuxMarkerMask__fixed | ((uint)MarkerTypeFlag_CYLINDER);

			//m_context__detect = new WorkContext();
			m_context__detect = m_context__track;


			//m_quadsTmpRuntime1__detect = new List<CvCBQuad>();
			//m_quadsTmpRuntime2__detect = new List<CvCBQuad>();
			//m_quadsTmpRuntime3__detect = new List<CvCBQuad>();
			//m_quadsTmpRuntime4__detect = new List<CvCBQuad>(1024);

			//m_rowPatternsRuntime__detect = new List<SinglePattern>();
			//m_reverseRowPatternsRuntime__detect = new List<SinglePattern>();
			//m_columnPatternsRuntime__detect = new List<SinglePattern>();
			//m_reverseColumnPatternsRuntime__detect = new List<SinglePattern>();
			//m_foundPatternsRuntime__detect = new List<SinglePattern>();
			//m_markerPatternsRuntime__detect = new List<SinglePattern>();

			m_uContextStamp__monitor = 0;
			//m_iTerminated__atomic.exchange(0);

			//m_mutexTrackedPost__mutex = new object();

	  //      {
			//	std::lock_guard<std::mutex> lock(m_mutexTrackedPost__mutex);

			//	m_trackedPose__mutex.m_eState = TrackedPose_INVALID;
			//	m_trackedPose__mutex.m_uStamp = 0;
			//	m_trackedPose__mutex.m_v3Position = Vector3(0, 0, 0);
			//	m_trackedPose__mutex.m_qOrientation = Quaternion(0, 0, 0, 1);
			//	m_trackedPose__mutex.m_pSpatialCoordinateSystem = nullptr;
			//}

			//m_matCamera__fixed = matCamera_;
			//m_matDistortionCoeff__fixed = distortionCoeff_;




			//m_mainScriptFrameAction = new UnityAction(OnNewFrame);
			//m_mainScript.RegisterFrameEvent(m_mainScriptFrameAction);

			//m_cbdetectParams__fixed.show_debug_image = false;
			//m_cbdetectParams__fixed.show_grow_processing = false;
			m_cbdetectParams__fixed.polynomial_fit = false;
			m_cbdetectParams__fixed.refine_location = true;
			m_cbdetectParams__fixed.detect_method = cbdetect::HessianResponse;


		}

		virtual ~NewCylindricalProbeTracker()
		{
			m_iTerminated__atomic.exchange(1);

			if (m_thread__detect.get_id() != std::thread::id())
			{
				std::unique_lock<std::mutex> lock__track(m_mutex__detect);

				m_condvar__detect.notify_all();
			}
			if (m_thread__track.get_id() != std::thread::id())
			{
				std::unique_lock<std::mutex> lock__track(m_mutex__track);

				m_condvar__track.notify_all();
			}
			if (m_thread__detect.get_id() != std::thread::id())
			{
				m_thread__detect.join();
			}
			if (m_thread__track.get_id() != std::thread::id())
			{
				m_thread__track.join();
			}

			Finalize__track();

			//if (m_mainScript != null)
			//{
			//	if (m_mainScriptFrameAction != null )
	  //          {
			//		m_mainScript.UnregisterFrameEvent(m_mainScriptFrameAction);
			//		m_mainScriptFrameAction = null;
			//	}
			//	m_mainScript = null;
			//}
		}


		void	Start()
		{
			//m_thread__track = std::thread(&NewCylindricalProbeTracker::Run__track, this);
			if (m_parameters__fixed.useDetectionThread)
			{
				m_thread__detect = std::thread(&NewCylindricalProbeTracker::Run__detect, this);
			}
			//m_mainScript = MainScript.Instance;

			if (m_parameters__fixed.useTrackingThread)
			{
				m_thread__track = std::thread(&NewCylindricalProbeTracker::Run__track, this);
			}
		}

		void    GetTrackedPose(TrackedPoseNativeStruct& ret)
		{
			std::lock_guard<std::mutex> lock(m_mutexTrackedPost__mutex);

			ret = m_trackedPose__mutex;
		}

		int64_t GetTrackedPoseTimestamp()
		{
			return m_trackedPose__mutex.m_i64Timestamp;
		}

		void  OnNewFrame()
		{
			if (m_thread__track.get_id() != std::thread::id())
			{
				//if (bSave)
				//	m_bSaveImage = true;
				std::unique_lock<std::mutex> lock__track(m_mutex__track, std::defer_lock);
				if (lock__track.try_lock())
				{
					m_condvar__track.notify_all();
					lock__track.unlock();
				}

			}
		}

		void Routine__track(cv::Mat& matColor)
		{
			if (m_getFrameInfoNativeStructCB != nullptr)
			{
				m_getFrameInfoNativeStructCB(m_info__track);
			}

			if (m_info__track.m_i64FrameTimestamp != 0 && m_info__track.m_i64FrameTimestamp != m_lastTrackedPose_m_i64Timestamp__track
				&& m_info__track.m_pMatCamera != nullptr && m_info__track.m_pMatImageGray != nullptr)
			{
				m_lastTrackedPose_m_i64Timestamp__track = m_info__track.m_i64FrameTimestamp;

				int numaux = (int)m_parameters__fixed.auxiliaryMarkerSettings.size();

				//if (m_updateResultGrayCB != nullptr)
				//{
				//	m_updateResultGrayCB(m_info__track.m_pMatImageGray);
				//}

				//if (bUseThread == false ||  Monitor.TryEnter(m_thread__detect, 0))
				if (m_thread__detect.get_id() != std::thread::id())
				{

					bool bResetLastActiveBlockSize = m_iResetLastAdaptiveBlockSize__atomic.exchange(0) != 0;
					// detection thread 는 wait 중. 안전하게 tracking 가능

					std::unique_lock<std::mutex> lock__detect(m_mutex__detect, std::defer_lock);
					IUnknown* pPrevDetectSpacialCoordinateSystem = nullptr;

					uint uValidMainMarker = m_context__track.m_uTrackType;
					uint uPrevMainMarker = m_context__track.m_uPrevValidTrackType;
					int64_t i64PrevMainMarkerStamp = m_context__track.m_i64PrevTimestamp;
					uint uPossiblePrevMainMaker = m_context__track.m_uPossibleTrackType;
					if (((uValidMainMarker | uPrevMainMarker) & (uint)MarkerTypeFlag_CYLINDER) == 0
						|| !(m_context__track.m_iPrevMarkerIndex >= 0 && m_context__track.m_iPrevMarkerCount > 0))
					{
						uValidMainMarker &= ~((uint)MarkerTypeFlag_CYLINDER);
						uPrevMainMarker &= ~((uint)MarkerTypeFlag_CYLINDER);
						m_context__track.m_iPrevMarkerIndex = -1;
						m_context__track.m_iPrevMarkerCount = 0;
					}

					if (uValidMainMarker == 0)
					{
						if (uPrevMainMarker != 0)
						{
							m_iTrackSkipFrameCounter__track++;
							if (m_iTrackSkipFrameCounter__track > m_parameters__fixed.trackNumToleranceFrames)
							{
								m_iTrackSkipFrameCounter__track = 0;
								m_context__track.m_uPrevValidTrackType = uPrevMainMarker = 0;

								m_context__track.m_uPossibleTrackType = uPossiblePrevMainMaker = 0;
								m_context__track.m_i64PrevTimestamp = i64PrevMainMarkerStamp = 0;
								m_context__track.m_vecPrevGrayPyr.clear();
							}
						}
						else
						{
							m_iTrackSkipFrameCounter__track = 0;
							m_context__track.m_uPossibleTrackType = uPossiblePrevMainMaker = 0;
							m_context__track.m_uExternalTrackType = 0;
						}
					}
					else
					{
						m_iTrackSkipFrameCounter__track = 0;
					}

					if (lock__detect.try_lock())
					{
						if (bResetLastActiveBlockSize)
						{
							m_context__track.m_lastAdaptiveBlockSize = 0;
							m_context__track.m_prevImgBound.width = 0;
							m_context__track.m_prevImgBound.height = 0;
							m_context__track.m_i64PrevImgBoundTimestamp = 0;
							m_context__track.m_iPrevDetectFailCount = 0;
							m_context__detect.m_lastAdaptiveBlockSize = 0;
							m_context__detect.m_prevImgBound.width = 0;
							m_context__detect.m_prevImgBound.height = 0;
							m_context__detect.m_i64PrevImgBoundTimestamp = 0;
							m_context__detect.m_iPrevDetectFailCount = 0;
						}
						//m_context__detect.m_curCameraTransform = m_info__track.m_mCameraToWorld;
						//m_context__detect.m_curCameraRotation = m_info__track.m_qCameraToWorld;


						// detect thread -> main thread 로부터 받아오기
						if (m_uContextStamp__track != m_uContextStamp__monitor)
						{
							m_uContextStamp__track = m_uContextStamp__monitor;

							if (m_context__detect.m_bLastVisionPose && !m_context__track.m_bLastVisionPose)
							{
								m_context__track.m_bLastVisionPose = true;
								m_context__track.m_v3LastVisionPosition = m_context__detect.m_v3LastVisionPosition;
								m_context__track.m_qLastVisionOrientation = m_context__detect.m_qLastVisionOrientation;
							}
							if (m_context__detect.m_validRTVec && !m_context__track.m_validRTVec)
							{
								m_context__track.m_validRTVec = true;
								m_context__detect.m_rvec.copyTo(m_context__track.m_rvec);
								m_context__detect.m_tvec.copyTo(m_context__track.m_tvec);
							}

							if ((m_context__detect.m_uTrackType & (uint)MarkerTypeFlag_CYLINDER) == 0
								|| !(m_context__detect.m_iPrevMarkerIndex >= 0 && m_context__detect.m_iPrevMarkerCount > 0))
							{
								m_context__detect.m_uTrackType &= ~((uint)MarkerTypeFlag_CYLINDER);
							}

							uint uValidDetectMarker = m_context__detect.m_uTrackType;
							int64_t i64ValidDetecterStamp = m_context__detect.m_i64PrevTimestamp;

							m_context__track.m_uExternalTrackType = 0;

							bool bUseDetectResult = false;
							if (((uPrevMainMarker & ((uint)MarkerTypeFlag_CYLINDER)) == 0
								|| i64PrevMainMarkerStamp < i64ValidDetecterStamp)
								&& (uValidDetectMarker & ((uint)MarkerTypeFlag_CYLINDER)) != 0)
							{
								uPrevMainMarker &= ~((uint)MarkerTypeFlag_CYLINDER);

								m_context__track.m_uExternalTrackType |= (uint)MarkerTypeFlag_CYLINDER;
								m_context__track.m_cylinderMarkerRuntimes = m_context__detect.m_cylinderMarkerRuntimes;
								m_context__track.m_iPrevMarkerIndex = m_context__detect.m_iPrevMarkerIndex;
								m_context__track.m_iPrevMarkerCount = m_context__detect.m_iPrevMarkerCount;
								m_context__track.m_uPossibleTrackType = uPossiblePrevMainMaker = m_context__detect.m_uPossibleTrackType;
								m_context__track.m_prevCylinderMarkerImgBound = m_context__detect.m_prevCylinderMarkerImgBound;
								m_context__track.m_prevCylinderMarkerEdgeMax = m_context__detect.m_prevCylinderMarkerEdgeMax;
								bUseDetectResult = true;
							}
							for (int aux = 0; aux < numaux; aux++)
							{
								uint flag = (uint)(1 << ((int)MarkerType_AUX_BASE + aux));

								if (((uPrevMainMarker & flag) == 0 || i64PrevMainMarkerStamp < i64ValidDetecterStamp)
									&& ( // (uPrevMainMarker & m_uAuxUniqueMarkerMask__fixed) == 0 ||
										((uPossiblePrevMainMaker & uValidDetectMarker) & flag) != 0))
								{
									m_context__track.m_uExternalTrackType |= flag;
									uPrevMainMarker &= ~flag;
									m_context__track.m_auxliaryMarkerRuntimes[aux] = m_context__detect.m_auxliaryMarkerRuntimes[aux];
									m_context__track.m_prevAuxiliaryMarkersImgBound[aux] = m_context__detect.m_prevAuxiliaryMarkersImgBound[aux];
									m_context__track.m_prevAuxiliaryMarkersEdgeMax[aux] = m_context__detect.m_prevAuxiliaryMarkersEdgeMax[aux];
									bUseDetectResult = true;
								}
							}
							if (bUseDetectResult == true)
							{
								m_context__detect.m_prevGray.copyTo(m_context__track.m_prevGray);
								m_context__detect.m_prevOriginal.copyTo(m_context__track.m_prevOriginal);
								m_iTrackSkipFrameCounter__track = 0;
							}
							m_context__track.m_uPrevValidTrackType = uPrevMainMarker;
						}
						// activate 할 요소가 있으면 main thread -> detect thread 로 전달한 후 thread activate

						{
							uint uCurrentValidMarker = uPrevMainMarker | m_context__track.m_uExternalTrackType;
							uint uNeedDetectMarker = 0;
							if ((uCurrentValidMarker & ((uint)MarkerTypeFlag_CYLINDER)) == 0)
								uNeedDetectMarker |= ((uint)MarkerTypeFlag_CYLINDER);

							for (int aux = 0; aux < numaux; aux++)
							{
								uint flag = (uint)(1 << ((int)MarkerType_AUX_BASE + aux));
								if ((uCurrentValidMarker & flag) == 0
									&& ( //(uCurrentValidMarker & m_uAuxUniqueMarkerMask__fixed) == 0 ||
										(uPossiblePrevMainMaker & flag) != 0))
								{
									uNeedDetectMarker |= flag;
									uNeedDetectMarker |= ((uint)MarkerTypeFlag_CYLINDER);
								}
							}

							if (uNeedDetectMarker != 0)
							{
								OpenCVRect rect(0, 0, 0, 0);
								float dMaxEdge = 0.0f;

								if ((uNeedDetectMarker & ((uint)MarkerTypeFlag_CYLINDER)) != 0)
								{
									rect = m_context__track.m_prevCylinderMarkerImgBound;
									dMaxEdge = m_context__track.m_prevCylinderMarkerEdgeMax;
								}
								if (rect.empty() == false)
								{
									for (int aux = 0; aux < numaux; aux++)
									{
										uint flag = (uint)(1 << ((int)MarkerType_AUX_BASE + aux));
										if ((uNeedDetectMarker & flag) != 0)
										{
											dMaxEdge = std::max(dMaxEdge, m_context__track.m_prevAuxiliaryMarkersEdgeMax[aux]);
											if (m_context__track.m_prevAuxiliaryMarkersImgBound[aux].empty() == false)
											{
												rect = rect | m_context__track.m_prevAuxiliaryMarkersImgBound[aux];
											}
										}
									}
								}

								m_info__track.m_pMatCamera->copyTo(m_context__detect.m_matCamera);
								m_info__track.m_pMatImageGray->copyTo(m_context__detect.m_prevGray);
								m_info__track.m_pMatImageOriginal->copyTo(m_context__detect.m_prevOriginal);
								
								pPrevDetectSpacialCoordinateSystem = m_context__detect.m_pPrevSpatialCoordinateSystem;
								m_context__detect.m_pPrevSpatialCoordinateSystem = m_info__track.m_pSpatialCoordinateSystem;
								if (m_context__detect.m_pPrevSpatialCoordinateSystem != nullptr)
									m_context__detect.m_pPrevSpatialCoordinateSystem->AddRef();
								m_context__detect.m_i64PrevTimestamp = m_info__track.m_i64FrameTimestamp;

								if (m_context__track.m_bLastVisionPose &&
									(!m_context__detect.m_bLastVisionPose || (m_context__track.m_uTrackType & m_uMarkerMask__fixed) != 0))
								{
									m_context__detect.m_bLastVisionPose = true;
									m_context__detect.m_v3LastVisionPosition = m_context__track.m_v3LastVisionPosition;
									m_context__detect.m_qLastVisionOrientation = m_context__track.m_qLastVisionOrientation;
								}
								if (m_context__track.m_validRTVec &&
									(!m_context__detect.m_validRTVec || (m_context__track.m_uTrackType & m_uMarkerMask__fixed) != 0))
								{
									m_context__detect.m_validRTVec = true;
									m_context__track.m_rvec.copyTo(m_context__detect.m_rvec);
									m_context__track.m_tvec.copyTo(m_context__detect.m_tvec);
								}

								m_context__detect.m_uExternalTrackType = uNeedDetectMarker;
								if (rect.empty() == false)
								{
									rect.x -= m_parameters__fixed.expandAreaMargin;
									rect.y -= m_parameters__fixed.expandAreaMargin;
									rect.width += m_parameters__fixed.expandAreaMargin * 2;
									rect.height += m_parameters__fixed.expandAreaMargin * 2;
									m_context__detect.m_i64PrevImgBoundTimestamp = m_context__track.m_i64PrevImgBoundTimestamp;
								}
								else
								{
									m_context__detect.m_i64PrevImgBoundTimestamp = 0;
								}
								m_context__detect.m_prevImgBound = rect;

								m_context__detect.m_lastAdaptiveBlockSize = (int)std::round(dMaxEdge * 2);
								if ((uCurrentValidMarker & ((uint)MarkerTypeFlag_CYLINDER)) != 0)
								{
									m_context__detect.m_uTrackType = (uint)MarkerTypeFlag_CYLINDER;
									m_context__detect.m_cylinderMarkerRuntimes = m_context__track.m_cylinderMarkerRuntimes;
									m_context__detect.m_iPrevMarkerIndex = m_context__track.m_iPrevMarkerIndex;
									m_context__detect.m_iPrevMarkerCount = m_context__track.m_iPrevMarkerCount;
									m_context__detect.m_adaptiveBlockSizeCounter = 0;
								}
								else
								{
									m_context__detect.m_uTrackType = 0;
									m_context__detect.m_iPrevMarkerIndex = -1;
									m_context__detect.m_iPrevMarkerCount = 0;
								}
								std::vector<float> vecFrameEyeGaze = *m_info__track.m_pVecEyeGaze;
								m_context__detect.m_vecEyeGaze = vecFrameEyeGaze;

								m_uContextStamp__track++;
								if (m_uContextStamp__track == 0)
									m_uContextStamp__track = 1;
								m_uContextStamp__monitor = m_uContextStamp__track;
								m_condvar__detect.notify_all();
							}
						}
						lock__detect.unlock();
					}

					if (bResetLastActiveBlockSize)
					{
						m_context__track.m_lastAdaptiveBlockSize = 0;
						m_context__track.m_prevImgBound.width = 0;
						m_context__track.m_prevImgBound.height = 0;
						m_context__track.m_i64PrevImgBoundTimestamp = 0;
						m_context__track.m_iPrevDetectFailCount = 0;
					}

					if (pPrevDetectSpacialCoordinateSystem != nullptr)
					{
						pPrevDetectSpacialCoordinateSystem->Release();
						pPrevDetectSpacialCoordinateSystem = nullptr;
					}

					if (((m_context__track.m_uPrevValidTrackType | m_context__track.m_uExternalTrackType) & m_uMarkerMask__fixed) != 0)
					{
						m_info__track.m_pMatCamera->copyTo(m_context__track.m_matCamera);

						if (trackMarkerNew__track(m_context__track, *m_info__track.m_pMatImageGray, *m_info__track.m_pMatImageOriginal, matColor, m_info__track.m_pSpatialCoordinateSystem, m_info__track.m_i64FrameTimestamp, /*out*/ m_lastTrackedPose_m_v3Position__track, /*out*/ m_lastTrackedPose_m_qOrientation__track))
						{
							m_context__track.m_uExternalTrackType = 0;
						}
						else
						{
							m_context__track.m_uTrackType = 0;
						}
					}
				}
				else //if (bUseThread == false )//&& m_iPrevMarkerIndex < 0 )
				{
					// detection thread는 working 중

					m_context__track.m_uTrackType = 0;
					m_context__track.m_uPrevValidTrackType = 0;
					m_context__track.m_iPrevMarkerIndex = -1;
					m_context__track.m_iPrevMarkerCount = 0;
					m_context__track.m_uExternalTrackType = m_uMarkerMask__fixed;
					//m_context__track.m_curCameraTransform = m_info__track.m_mCameraToWorld;
					//m_context__track.m_curCameraRotation = m_info__track.m_qCameraToWorld;

					m_info__track.m_pMatCamera->copyTo(m_context__track.m_matCamera);

					if (detectMarkerNew__detect(m_context__track, *m_info__track.m_pMatImageGray, *m_info__track.m_pMatImageOriginal, matColor, m_info__track.m_pSpatialCoordinateSystem, m_info__track.m_i64FrameTimestamp, /*out*/ m_lastTrackedPose_m_v3Position__track, /*out*/ m_lastTrackedPose_m_qOrientation__track))
					{
					}
					else
					{
						m_context__track.m_uTrackType = 0;
						m_context__track.m_uPrevValidTrackType = 0;
						m_context__track.m_iPrevMarkerIndex = -1;
						m_context__track.m_iPrevMarkerCount = 0;
					}
				}

				//m_curColor__track = cur_color_;
				bool bSkip = false;
				if ((m_context__track.m_uTrackType & m_uMarkerMask__fixed) != 0)
				{
					if ((m_context__track.m_uTrackType & (uint)MarkerTypeFlag_CYLINDER) != 0
						&& (m_context__track.m_uTrackType & m_uAuxMarkerMask__fixed) != 0)
					{
						m_lastTrackedPose_m_eState__track = TrackedPose_STABLE;
					}
					else
					{
						m_lastTrackedPose_m_eState__track = TrackedPose_UNSTABLE;
					}
				}
				else
				{
					m_lastTrackedPose_m_eState__track = TrackedPose_INVALID;
					if (m_info__track.m_pSpatialCoordinateSystem != nullptr)
					{
						m_info__track.m_pSpatialCoordinateSystem->Release();
						m_info__track.m_pSpatialCoordinateSystem = nullptr;
					}
					if (m_lastTrackedPose_m_eState__track == m_trackedPose__mutex.m_eState)
					{
						bSkip = true;
					}
				}

				if (bSkip == false)
				{
					std::lock_guard<std::mutex> lock(m_mutexTrackedPost__mutex);

					m_trackedPose__mutex.m_fPositionX = m_lastTrackedPose_m_v3Position__track[0];
					m_trackedPose__mutex.m_fPositionY = m_lastTrackedPose_m_v3Position__track[1];
					m_trackedPose__mutex.m_fPositionZ = m_lastTrackedPose_m_v3Position__track[2];
					m_trackedPose__mutex.m_fOrientationX = m_lastTrackedPose_m_qOrientation__track[0];
					m_trackedPose__mutex.m_fOrientationY = m_lastTrackedPose_m_qOrientation__track[1];
					m_trackedPose__mutex.m_fOrientationZ = m_lastTrackedPose_m_qOrientation__track[2];
					m_trackedPose__mutex.m_fOrientationW = m_lastTrackedPose_m_qOrientation__track[3];
					m_trackedPose__mutex.m_eState = m_lastTrackedPose_m_eState__track;
					m_trackedPose__mutex.m_i64Timestamp = m_lastTrackedPose_m_i64Timestamp__track;
				}
			}
			else
			{
				m_lastTrackedPose_m_i64Timestamp__track = m_info__track.m_i64FrameTimestamp;
			}
		}

		void Finalize__track()
		{
			if (m_info__track.m_pMatCamera)
			{
				delete m_info__track.m_pMatCamera;
				m_info__track.m_pMatCamera = nullptr;
			}
			if (m_info__track.m_pMatImageGray)
			{
				delete m_info__track.m_pMatImageGray;
				m_info__track.m_pMatImageGray = nullptr;
			}
			if ( m_info__track.m_pMatImageOriginal)
			{
				delete m_info__track.m_pMatImageOriginal;
				m_info__track.m_pMatImageOriginal = nullptr;
			}
			if (m_info__track.m_pSpatialCoordinateSystem != nullptr)
			{
				m_info__track.m_pSpatialCoordinateSystem->Release();
				m_info__track.m_pSpatialCoordinateSystem = nullptr;
			}
		}

		void Update(cv::Mat& matColor)
		{
			if (m_thread__track.get_id() == std::thread::id())
			{
				Routine__track(matColor);
			}
		}

		void Run__track()
		{
			cv::Mat matColor;

			std::unique_lock<std::mutex> lock__track(m_mutex__track);
			{
				while (m_iTerminated__atomic == 0)
				{
					int64_t i64CurFrameTimestamp = 0;
					if (m_getFrameInfoStampCB != nullptr)
					{
						i64CurFrameTimestamp = m_getFrameInfoStampCB();
					}
					if (i64CurFrameTimestamp == 0 || i64CurFrameTimestamp == m_lastTrackedPose_m_i64Timestamp__track)
					{
						m_condvar__track.wait(lock__track);
					}

					Routine__track(matColor);
				}
			}
			Finalize__track();
		}


		void Run__detect()
		{
			uint uLastContextStamp = 0;

			std::unique_lock<std::mutex> lock__detect(m_mutex__detect);

			while (m_iTerminated__atomic == 0)
			{
				m_condvar__detect.wait(lock__detect);

				// 여기서부터는 안전하게 detection 작업 가능
				//try
				{
					if (uLastContextStamp != m_uContextStamp__monitor)
					{
						uLastContextStamp = m_uContextStamp__monitor;
						Vector3 pos(0, 0, 0);
						Quaternion orient(0, 0, 0, 1);
						//Debug.Log("Enter : detectMarkerNew__detect");
						bool bRet = detectMarkerNew__detect(m_context__detect, m_context__detect.m_prevGray, m_context__detect.m_prevOriginal, cv::Mat()
							, m_context__detect.m_pPrevSpatialCoordinateSystem, m_context__detect.m_i64PrevTimestamp
							, /*out*/ pos, /*out*/ orient);
						//Debug.Log("Exit : detectMarkerNew__detect " + ((bRet) ? "success" : "fail"));
						if (bRet == true)
						{
							m_uContextStamp__monitor++;
							if (m_uContextStamp__monitor == 0)
								m_uContextStamp__monitor = 1;
							uLastContextStamp = m_uContextStamp__monitor;
						}
					}
				}
				//catch (...)
				//{
				//	//Debug.Log(e);
				//}
			}
		}


		void resetAdaptiveBlockSize()
		{
			m_iResetLastAdaptiveBlockSize__atomic.exchange(1);
			//m_lastAdaptiveBlockSize = 0;
		}

		void setBoundPoints(const BoundPoints& boundPoints)
		{
			std::lock_guard<std::mutex> lock(m_mutexNewBoundPoints__mutex);
			m_newBoundPoints__mutex = boundPoints;
			int iStamp = m_iBoundPointsStamp__mutex + 1;
			if (iStamp == 0)
				iStamp = 1;
			m_iBoundPointsStamp__mutex = iStamp;
		}


		static float estimateFittingCircle(float x1, float y1, float x2, float y2, float x3, float y3
			, float& cx, float& cy)
		{
			float x12 = x1 - x2;
			float x13 = x1 - x3;

			float y12 = y1 - y2;
			float y13 = y1 - y3;

			float y31 = y3 - y1;
			float y21 = y2 - y1;

			float x31 = x3 - x1;
			float x21 = x2 - x1;

			// x1^2 - x3^2 
			float sx13 = x1 * x1 - x3 * x3;

			// y1^2 - y3^2 
			float sy13 = y1 * y1 - y3 * y3;

			float sx21 = x2 * x2 - x1 * x1;
			float sy21 = y2 * y2 - y1 * y1;

			float f = ((sx13) * (x12)
				+(sy13) * (x12)
				+(sx21) * (x13)
				+(sy21) * (x13))
				/ (2 * ((y31) * (x12)-(y21) * (x13)));
			float g = ((sx13) * (y12)
				+(sy13) * (y12)
				+(sx21) * (y13)
				+(sy21) * (y13))
				/ (2 * ((x31) * (y12)-(x21) * (y13)));

			float c = -x1 * x1 - y1 * y1 - 2 * g * x1 - 2 * f * y1;

			// eqn of circle be x^2 + y^2 + 2*g*x + 2*f*y + c = 0 
			// where centre is (h = -g, k = -f) and radius r 
			// as r^2 = h^2 + k^2 - c 
			float h = -g;
			float k = -f;
			float sqr_of_r = h * h + k * k - c;

			// r is the radius 
			cx = h;
			cy = k;
			return std::sqrt(sqr_of_r);
		}

		static float estimateInvRadiusFromChord(float centerY, float outerY, float& outerX)
		{
			float d = centerY - outerY;
			if (d == 0)
			{
				return 0;
			}

			if (d > 0)
			{
				float theta = std::atan(outerX / d);
				float invRadius = std::sin(2 * theta) / outerX;
				outerX = (MY_PI - 2 * theta) / invRadius;
				return invRadius;
			}
			else
			{
				float theta = std::atan(outerX / (-d));
				float invRadius = std::sin(2 * theta) / outerX;
				outerX = (MY_PI - 2 * theta) / invRadius;
				return -invRadius;
			}
		}

		static float estimateInvRadius(float centerY, float outerY, float arc)
		{
			float a = arc / 12.0f;
			float b = 0;
			float c = -arc / 2.0f;
			float d = centerY - outerY;

			if (d == 0)
			{
				return 0;
			}

			b /= a;
			c /= a;
			d /= a;

			float disc, q, r, dum1, s, t, term1, r13;
			q = (3.0f * c - (b * b)) / 9.0f;
			r = -(27.0f * d) + b * (9.0f * c - 2.0f * (b * b));
			r /= 54.0f;
			disc = q * q * q + r * r;
			term1 = (b / 3.0f);
			if (disc > 0)
			{
				// one root real, two are complex
				s = r + std::sqrt(disc);
				s = ((s < 0) ? -std::pow(-s, (1.0f / 3.0f)) : std::pow(s, (1.0f / 3.0f)));
				t = r - std::sqrt(disc);
				t = ((t < 0) ? -std::pow(-t, (1.0f / 3.0f)) : std::pow(t, (1.0f / 3.0f)));
				float theta = -term1 + s + t;
				return theta / arc;
			}
			// End if (disc > 0)

			// The remaining options are all real
			if (disc == 0)
			{ // All roots real, at least two are equal.
				r13 = ((r < 0) ? -std::pow(-r, (1.0f / 3.0f)) : std::pow(r, (1.0f / 3.0f)));
				float theta1 = -term1 + 2.0f * r13;
				float theta2 = -(r13 + term1);

				if (theta1 > 0.0f && theta2 > 0.0f)
				{
					return std::min(theta1, theta2) / arc;
				}
				else if (theta1 > 0.0f)
				{
					return theta1 / arc;
				}
				else if (theta2 > 0.0f)
				{
					return theta2 / arc;
				}
				else
				{
					return 0;
				}

			} // End if (disc == 0)
			  // Only option left is that all roots are real and unequal (to get here, q < 0)
			else
			{
				q = -q;
				dum1 = q * q * q;
				dum1 = std::acos(r / std::sqrt(dum1));
				r13 = 2.0f * std::sqrt(q);

				float theta1 = -term1 + r13 * std::cos(dum1 / 3.0f);
				float theta2 = -term1 + r13 * std::cos((dum1 + 2.0f * MY_PI) / 3.0f);
				float theta3 = -term1 + r13 * std::cos((dum1 + 4.0f * MY_PI) / 3.0f);

				if (theta1 > 0.0f)
				{
					if (theta2 > 0.0f)
					{
						if (theta3 > 0.0f)
						{
							return std::min(std::min(theta1, theta2), theta3) / arc;
						}
						else
						{
							return std::min(theta1, theta2) / arc;
						}
					}
					else
					{
						if (theta3 > 0.0f)
						{
							return std::min(theta1, theta3) / arc;
						}
						else
						{
							return theta1 / arc;
						}
					}
				}
				else if (theta2 > 0.0f)
				{
					if (theta3 > 0.0f)
					{
						return std::min(theta2, theta3) / arc;
					}
					else
					{
						return theta2 / arc;
					}
				}
				else if (theta3 > 0.0f)
				{
					return theta3 / arc;
				}
				else
				{
					return 0.0f;
				}
			}
		}

		static int EstimateCheckerBoardNumBlocks(int width, int height, bool bMajor)
		{
			if (width <= 0 || height <= 0)
				return 0;

			int iWidthMajor = (width + 1) >> 1;
			int iWidthMinor = width - iWidthMajor;
			int iHeightMajor = (height + 1) >> 1;
			int iHeightMinor = height - iHeightMajor;

			if (bMajor == true)
			{
				return iWidthMajor * iHeightMajor + iWidthMinor * iHeightMinor;
			}
			else
			{
				return iWidthMinor * iHeightMajor + iWidthMajor * iHeightMinor;
			}
		}




		//static bool solveZ( Mat perspective, Point ip, ref Point3 wp )
		//{
		//	double[] arrProj = new double[12];
		//	perspective.get(0, 0, arrProj);

		//	Point3 a = new Point3(arrProj[4 * 0 + 0] * wp.x + arrProj[4 * 0 + 1] * wp.y + arrProj[4 * 0 + 3]
		//		, arrProj[4 * 1 + 0] * wp.x + arrProj[4 * 1 + 1] * wp.y + arrProj[4 * 1 + 3]
		//		, arrProj[4 * 2 + 0] * wp.x + arrProj[4 * 2 + 1] * wp.y + arrProj[4 * 2 + 3]);
		//	Point3 b = new Point3(arrProj[4 * 0 + 2], arrProj[4 * 1 + 2], arrProj[4 * 2 + 2]);
		//	Point3 n = a.cross(b);

		//	double anx = std::abs(n.x);
		//	double any = std::abs(n.y);
		//	Point s = new Point(0.0, 0.0);
		//	if ( anx >= any )
		//	{
		//		if (anx == 0.0)
		//			return false;
		//		s.x = -n.Z / n.x;
		//	}
		//	else
		//	{
		//		s.y = -n.Z / n.y;
		//	}
		//	Point dir = new Point(n.y, -n.x);
		//	normalize(dir);
		//	Point diff = ip - s ;
		//	Point closest = s + dir * (dir.dot(diff));
		//	Point3 closest3 = new Point3(closest.x, closest.y, 1.0);

		//	if (normalize(n) == 0.0)
		//		return false;

		//	Point3 a_ = new Point3(a.x, a.y, a.Z);
		//	double alen = normalize(a_);

		//	Point3 b_ = n.cross(a_);
		//	double bcoeff = closest3.dot(b_) / b.dot(b_);
		//	double acoeff = (closest3 - b * bcoeff).dot(a_) / alen;
		//	if (acoeff == 0.0)
		//		return false;

		//	wp.Z = bcoeff / acoeff;

		//	return true;
		//}





		//static bool findStraightLine2(List<Point> approxPoly2f, ref int start, ref int end, double tolerance)
		//{
		//	int polyVertices = approxPoly2f.size();
		//	if (polyVertices <= 4)
		//		return false;

		//	int iL0 = -1;
		//	int iL1 = -1;
		//	double tolerance2 = tolerance * tolerance;
		//	//double minCos = 1.0;
		//	double dL2 = 0.0;

		//	if (start == 0 && end == polyVertices)
		//	{
		//		for (int a = 0; a < polyVertices; a++)
		//		{
		//			//Point p_1 = approxPoly2f[(a + polyVertices - 1) % polyVertices];
		//			Point p0 = approxPoly2f[a];
		//			Point p1 = approxPoly2f[(a + 1) % polyVertices];

		//			//Point diff0 = p0 - p_1;
		//			//double len0 = normalize(diff0);
		//			//Point diff1 = p1 - p0;
		//			//double len1 = normalize(diff1);
		//			//if ( len0 > 0.0 && len1 > 0.0 )
		//			//{
		//			//	double cos = diff0.dot(diff1);
		//			//	if ( cos < minCos )
		//			//	{
		//			//		minCos = cos;
		//			//		iL0 = a;
		//			//		iL1 = a + 1;
		//			//	}
		//			//}

		//			double d2 = (p1.x - p0.x) * (p1.x - p0.x) + (p1.y - p0.y) * (p1.y - p0.y);
		//			if (d2 > dL2)
		//			{
		//				dL2 = d2;
		//				iL0 = a;
		//				iL1 = a + 1;
		//			}
		//		}
		//		if (iL0 < 0)
		//			return false;
		//		Point pL0 = approxPoly2f[iL0];
		//		Point pL0_1 = approxPoly2f[iL1 % polyVertices];

		//		int istart = iL0 + 1;
		//		int iend = iL0 + polyVertices;

		//		for (int a = istart; a < iend; a++)
		//		{
		//			Point p0 = approxPoly2f[a % polyVertices];
		//			Point p1 = approxPoly2f[(a + 1) % polyVertices];
		//			double dot1 = (p1.x - p0.x) * (p0.x - pL0.x) + (p1.y - p0.y) * (p0.y - pL0.y);
		//			if (dot1 <= 0.0)
		//				break;
		//			//double dot2 = (p1.x - p0.x) * (pL0.y - p0.y) + (p1.y - p0.y) * (p0.x - pL0.x);
		//			//if (std::abs(dot2) >= dot1)
		//			//	break;
		//			double dL20_1_tol = tolerance * calcDistanceSqr(pL0, p1);
		//			bool bError = false;
		//			for (int b = istart; b <= a; b++)
		//			{
		//				double d2 = calcDistanceNumerator(pL0, p1, approxPoly2f[b % polyVertices]);
		//				if (d2 >= dL20_1_tol)
		//				{
		//					bError = true;
		//					break;
		//				}
		//			}
		//			if ( !bError )
		//			{
		//				iL1 = a + 1;
		//			}
		//			else
		//			{
		//				break;
		//			}

		//		}
		//		if ((iL1 % polyVertices) == iL0)
		//			return false;
		//		pL0_1 = approxPoly2f[iL1 % polyVertices];

		//		istart = iL0 - 1;
		//		iend = iL1 - 1 - polyVertices;

		//		for (int a = istart; a > iend; a--)
		//		{

		//			Point p0 = approxPoly2f[(a + polyVertices) % polyVertices];
		//			Point p1 = approxPoly2f[(a + polyVertices + 1) % polyVertices];
		//			double dot1 = (p1.x - p0.x) * (pL0_1.x - p0.x) + (p1.y - p0.y) * (pL0_1.y - p0.y);
		//			if (dot1 <= 0.0)
		//				break;
		//			//double dot2 = (p1.x - p0.x) * (p0.y - pL0_1.y) + (p1.y - p0.y) * (pL0_1.x - p0.x);
		//			//if (std::abs(dot2) >= dot1)
		//			//	break;
		//			double dL20_1_tol = tolerance * calcDistanceSqr(p0, pL0_1);
		//			bool bError = false;
		//			for ( int b = iL1 - 1;  b >= a + 1; b--)
		//			{
		//				double d2 = calcDistanceNumerator(p0, pL0_1, approxPoly2f[(b + polyVertices ) % polyVertices]);
		//				if (d2 >= dL20_1_tol)
		//				{
		//					bError = true;
		//					break;
		//				}
		//			}
		//			if ( !bError )
		//			{
		//				iL0 = a;
		//			}
		//			else
		//			{
		//				break;
		//			}
		//		}
		//		if ((iL0 + polyVertices) % polyVertices == (iL1 % polyVertices))
		//			return false;

		//		start = (iL0 + polyVertices) % polyVertices;
		//		end = iL1 % polyVertices;
		//		return true;

		//	}
		//	else
		//	{
		//		int istart = start % polyVertices;
		//		int iend = end % polyVertices;

		//		if (istart == iend)
		//			return false;
		//		else if (istart > iend)
		//			iend += polyVertices;

		//		for (int a = istart; a < iend; a++)
		//		{
		//			//Point p_1 = approxPoly2f[(a + polyVertices - 1) % polyVertices];
		//			Point p0 = approxPoly2f[a % polyVertices];
		//			Point p1 = approxPoly2f[(a + 1) % polyVertices];
		//			double d2 = (p1.x - p0.x) * (p1.x - p0.x) + (p1.y - p0.y) * (p1.y - p0.y);
		//			if (d2 > dL2)
		//			{
		//				dL2 = d2;
		//				iL0 = a;
		//				iL1 = a + 1;
		//			}
		//			//	Point diff0 = p0 - p_1;
		//			//double len0 = normalize(diff0);
		//			//Point diff1 = p1 - p0;
		//			//double len1 = normalize(diff1);
		//			//if (len0 > 0.0 && len1 > 0.0)
		//			//{
		//				//double cos = diff0.dot(diff1);
		//				//if (cos < minCos)
		//				//{
		//					//minCos = cos;
		//					//iL0 = a;
		//					//iL1 = a + 1;
		//				//}
		//			//}
		//		}
		//		if (iL0 < 0)
		//			return false;
		//		Point pL0 = approxPoly2f[iL0 % polyVertices];
		//		Point pL0_1 = approxPoly2f[iL1 % polyVertices];

		//		for (int a = iL1; a < iend; a++)
		//		{
		//			Point p0 = approxPoly2f[a % polyVertices];
		//			Point p1 = approxPoly2f[(a + 1) % polyVertices];
		//			double dot1 = (p1.x - p0.x) * (p0.x - pL0.x) + (p1.y - p0.y) * (p0.y - pL0.y);
		//			if (dot1 <= 0.0)
		//				break;
		//			//double dot2 = (p1.x - p0.x) * (pL0.y - p0.y) + (p1.y - p0.y) * (p0.x - pL0.x);
		//			//if (std::abs(dot2) >= dot1)
		//			//	break;
		//			double dL20_1_tol = tolerance * calcDistanceSqr(pL0, p1);
		//			bool bError = false;
		//			for (int b = iL0 + 1; b <= a; b++)
		//			{
		//				double d2 = calcDistanceNumerator(pL0, p1, approxPoly2f[b % polyVertices]);
		//				if (d2 >= dL20_1_tol)
		//				{
		//					bError = true;
		//					break;
		//				}
		//			}
		//			if (!bError)
		//			{
		//				iL1 = a + 1;
		//			}
		//			else
		//			{
		//				break;
		//			}
		//		}
		//		if ((iL1 % polyVertices) == (iL0 % polyVertices))
		//			return false;

		//		pL0_1 = approxPoly2f[iL1 % polyVertices];

		//		for (int a = iL0 - 1; a >= istart; a--)
		//		{
		//			Point p0 = approxPoly2f[(a + polyVertices) % polyVertices];
		//			Point p1 = approxPoly2f[(a + polyVertices + 1) % polyVertices];
		//			double dot1 = (p1.x - p0.x) * (pL0_1.x - p0.x) + (p1.y - p0.y) * (pL0_1.y - p0.y);
		//			if (dot1 <= 0.0)
		//				break;
		//			//double dot2 = (p1.x - p0.x) * (p0.y - pL0_1.y) + (p1.y - p0.y) * (pL0_1.x - p0.x);
		//			//if (std::abs(dot2) >= dot1)
		//			//	break;
		//			double dL20_1_tol = tolerance * calcDistanceSqr(p0, pL0_1);
		//			bool bError = false;
		//			for (int b = iL1 - 1; b >= a + 1; b--)
		//			{
		//				double d2 = calcDistanceNumerator(p0, pL0_1, approxPoly2f[(b + polyVertices) % polyVertices]);
		//				if (d2 >= dL20_1_tol)
		//				{
		//					bError = true;
		//					break;
		//				}
		//			}
		//			if (!bError)
		//			{
		//				iL0 = a;
		//			}
		//			else
		//			{
		//				break;
		//			}
		//		}
		//		if ((iL0 % polyVertices) == (iL1 % polyVertices))
		//			return false;

		//		start = iL0 % polyVertices;
		//		end = iL1 % polyVertices;
		//		return true;
		//	}

		//}

		//List<Point> calculateRectangleBound2(List<Point> approxPoly2f, double minCornerDistancePixels, double minDistance, double tolerancePixels
		//	, bool bDraw )
		//{
		//	if (approxPoly2f == null)
		//		return null;
		//	int polyVertices = approxPoly2f.size();
		//	if (polyVertices < 4)
		//		return null;

		//	int iL1 = -1;
		//	int iL11 = -1;
		//	int iL2 = -1;
		//	int iL21 = -1;
		//	//bool bCheckLongest = false;


		//	if (polyVertices == 4)
		//	{
		//		iL1 = 0;
		//		iL11 = 1;
		//		iL2 = 2;
		//		iL21 = 3;
		//		//bCheckLongest = true;
		//	}
		//	else
		//	{
		//		iL1 = 0;
		//		iL11 = polyVertices;

		//		if (false == findStraightLine2(approxPoly2f, ref iL1, ref iL11, tolerancePixels) || iL1 == iL11)
		//			return null;

		//		iL2 = iL11;
		//		iL21 = iL1;

		//		if (false == findStraightLine2(approxPoly2f, ref iL2, ref iL21, tolerancePixels) || iL2 == iL21)
		//			return null;

		//		if (iL11 == iL2 || iL21 == iL1)
		//			return null;

		//		int iCount = (iL1 < iL11) ? (iL11 - iL1 + 1) : ((polyVertices - iL11) + iL1 + 1);
		//		List<Point> convexTest1 = new List<Point>(iCount);
		//		if (iL1 < iL11)
		//		{
		//			convexTest1.AddRange(approxPoly2f.GetRange(iL1, iL11 - iL1 + 1));
		//		}
		//		else
		//		{
		//			convexTest1.AddRange(approxPoly2f.GetRange(iL11, polyVertices - iL11));
		//			convexTest1.AddRange(approxPoly2f.GetRange(0, iL1 + 1));
		//		}
		//		MatOfPoint matConvexTest1 = new MatOfPoint();
		//		matConvexTest1.fromList(convexTest1);

		//		iCount = (iL2 < iL21) ? (iL21 - iL2 + 1) : ((polyVertices - iL21) + iL2 + 1);
		//		List<Point> convexTest2 = new List<Point>(iCount);
		//		if (iL2 < iL21)
		//		{
		//			convexTest2.AddRange(approxPoly2f.GetRange(iL2, iL21 - iL2 + 1));
		//		}
		//		else
		//		{
		//			convexTest2.AddRange(approxPoly2f.GetRange(iL21, polyVertices - iL21));
		//			convexTest2.AddRange(approxPoly2f.GetRange(0, iL2 + 1));
		//		}
		//		MatOfPoint matConvexTest2 = new MatOfPoint();
		//		matConvexTest2.fromList(convexTest2);


		//		bool convex1 = Imgproc.isContourConvex(matConvexTest1);
		//		bool convex2 = Imgproc.isContourConvex(matConvexTest2);

		//		if ( convex1 == false && bDraw )
		//		{
		//			List<MatOfPoint> tmp2 = new List<MatOfPoint>(1);
		//			tmp2.Add(matConvexTest1);
		//			Imgproc.drawContours(m_curColor__track, tmp2, -1, new Scalar(255, 0, 0, 255));
		//		}
		//		if (convex2 == false && bDraw )
		//		{
		//			List<MatOfPoint> tmp2 = new List<MatOfPoint>(1);
		//			tmp2.Add(matConvexTest2);
		//			Imgproc.drawContours(m_curColor__track, tmp2, -1, new Scalar(255, 0, 0, 255));
		//		}

		//		if (convex1 == false || convex2 == false )
		//		{
		//			return null;
		//		}

		//	}

		//	Point pL10 = approxPoly2f[iL1];
		//	Point pL11 = approxPoly2f[iL11];
		//	Point pL20 = approxPoly2f[iL2];
		//	Point pL21 = approxPoly2f[iL21];

		//	{
		//		double dL1 = (pL11.x - pL10.x) * (pL11.x - pL10.x) + (pL11.y - pL10.y) * (pL11.y - pL10.y);
		//		double dL2 = (pL21.x - pL20.x) * (pL21.x - pL20.x) + (pL21.y - pL20.y) * (pL21.y - pL20.y);
		//		double dL12 = (pL20.x - pL11.x) * (pL20.x - pL11.x) + (pL20.y - pL11.y) * (pL20.y - pL11.y);
		//		double dL21 = (pL10.x - pL21.x) * (pL10.x - pL21.x) + (pL10.y - pL21.y) * (pL10.y - pL21.y);

		//		double minDistSq = std::min(dL1, dL2);
		//		minDistSq = std::min(minDistSq, dL12);
		//		minDistSq = std::min(minDistSq, dL21);

		//		if (minCornerDistancePixels > 0.0 && minDistSq < minCornerDistancePixels * minCornerDistancePixels)
		//			return null;
		//		double maxDistSq = std::max(dL1, dL2);
		//		maxDistSq = std::min(maxDistSq, dL12);
		//		maxDistSq = std::min(maxDistSq, dL21);
		//		if (minDistance > 0.0 && maxDistSq < minDistance * minDistance)
		//			return null;

		//		//if (bCheckLongest == true)
		//		//{
		//		//	double dLong = dL1 + dL2;
		//		//	double dLat = dL12 + dL21;

		//		//	if (dLat > dLong)
		//		//	{
		//		//		Point tmp = pL10;
		//		//		pL10 = pL11;
		//		//		pL11 = pL20;
		//		//		pL20 = pL21;
		//		//		pL21 = tmp;
		//		//		int itmp = iL1;
		//		//		iL1 = iL11;
		//		//		iL11 = iL2;
		//		//		iL2 = iL21;
		//		//		iL21 = itmp;
		//		//	}
		//		//}
		//	}

		//	double dx1 = pL11.x - pL10.x;
		//	double dy1 = pL11.y - pL10.y;
		//	double dx2 = pL20.x - pL10.x;
		//	double dy2 = pL20.y - pL10.y;
		//	double crossProduct = (dx1 * dy2) - (dy1 * dx2);

		//	List<Point> newAppproxPoly2f = new List<Point>(4);
		//	if (crossProduct >= 0)
		//	{
		//		newAppproxPoly2f.Add(pL10);
		//		newAppproxPoly2f.Add(pL11);
		//		newAppproxPoly2f.Add(pL20);
		//		newAppproxPoly2f.Add(pL21);
		//	}
		//	else
		//	{
		//		newAppproxPoly2f.Add(pL11);
		//		newAppproxPoly2f.Add(pL10);
		//		newAppproxPoly2f.Add(pL21);
		//		newAppproxPoly2f.Add(pL20);
		//	}

		//	return newAppproxPoly2f;

		//}

		//static bool findStraightLineForward(List<Point> approxPoly2f, int start, ref int end, double tolerance)
		//{
		//	int polyVertices = approxPoly2f.size();
		//	if (polyVertices <= 4)
		//		return false;

		//	int iL0 = -1;
		//	int iL1 = -1;
		//	double tolerance2 = tolerance * tolerance;

		//	int istart = start % polyVertices;
		//	int iend = end % polyVertices;

		//	if (istart == iend)
		//		return false;
		//	else if (istart > iend)
		//		iend += polyVertices;


		//	iL0 = start;
		//	iL1 = start + 1;

		//	Point pL0 = approxPoly2f[iL0 % polyVertices];
		//	Point pL0_1 = approxPoly2f[iL1 % polyVertices];

		//	for (int a = iL1; a < iend; a++)
		//	{
		//		Point p0 = approxPoly2f[a % polyVertices];
		//		Point p1 = approxPoly2f[(a + 1) % polyVertices];
		//		double dot1 = (p1.x - p0.x) * (p0.x - pL0.x) + (p1.y - p0.y) * (p0.y - pL0.y);
		//		if (dot1 <= 0.0)
		//			break;
		//		//double dot2 = (p1.x - p0.x) * (pL0.y - p0.y) + (p1.y - p0.y) * (p0.x - pL0.x);
		//		//if (std::abs(dot2) >= dot1)
		//		//	break;
		//		double dL20_1_tol = tolerance * calcDistanceSqr(pL0, p1);
		//		bool bError = false;
		//		for (int b = iL0 + 1; b <= a; b++)
		//		{
		//			double d2 = calcDistanceNumerator(pL0, p1, approxPoly2f[b % polyVertices]);
		//			if (d2 >= dL20_1_tol)
		//			{
		//				bError = true;
		//				break;
		//			}
		//		}
		//		if (!bError)
		//		{
		//			iL1 = a + 1;
		//		}
		//		else
		//		{
		//			break;
		//		}
		//	}
		//	if ((iL1 % polyVertices) == (iL0 % polyVertices))
		//		return false;

		//	end = iL1 % polyVertices;
		//	return true;

		//}

		//static bool findStraightLineBackward(List<Point> approxPoly2f, ref int start,  int end, double tolerance)
		//{
		//	int polyVertices = approxPoly2f.size();
		//	if (polyVertices <= 4)
		//		return false;

		//	int iL0 = -1;
		//	int iL1 = -1;
		//	double tolerance2 = tolerance * tolerance;

		//	int istart = start % polyVertices;
		//	int iend = end % polyVertices;

		//	if (istart == iend)
		//		return false;
		//	else if (istart > iend)
		//		iend += polyVertices;


		//	iL0 = iend - 1;
		//	iL1 = iend;

		//	Point pL0 = approxPoly2f[iL0 % polyVertices];
		//	Point pL0_1 = approxPoly2f[iL1 % polyVertices];

		//	pL0_1 = approxPoly2f[iL1 % polyVertices];

		//	for (int a = iL0 - 1; a >= istart; a--)
		//	{
		//		Point p0 = approxPoly2f[(a + polyVertices) % polyVertices];
		//		Point p1 = approxPoly2f[(a + polyVertices + 1) % polyVertices];
		//		double dot1 = (p1.x - p0.x) * (pL0_1.x - p0.x) + (p1.y - p0.y) * (pL0_1.y - p0.y);
		//		if (dot1 <= 0.0)
		//			break;
		//		//double dot2 = (p1.x - p0.x) * (p0.y - pL0_1.y) + (p1.y - p0.y) * (pL0_1.x - p0.x);
		//		//if (std::abs(dot2) >= dot1)
		//		//	break;
		//		double dL20_1_tol = tolerance * calcDistanceSqr(p0, pL0_1);
		//		bool bError = false;
		//		for (int b = iL1 - 1; b >= a + 1; b--)
		//		{
		//			double d2 = calcDistanceNumerator(p0, pL0_1, approxPoly2f[(b + polyVertices) % polyVertices]);
		//			if (d2 >= dL20_1_tol)
		//			{
		//				bError = true;
		//				break;
		//			}
		//		}
		//		if (!bError)
		//		{
		//			iL0 = a;
		//		}
		//		else
		//		{
		//			break;
		//		}
		//	}
		//	if ((iL0 % polyVertices) == (iL1 % polyVertices))
		//		return false;


		//	start = iL0 % polyVertices;
		//	return true;


		//}


		// bool checkNonOverlap( int a0, int a1, int b0, int b1, int numVertices )
		//{
		//	a0 = a0 % numVertices;
		//	a1 = a1 % numVertices;
		//	b0 = b0 % numVertices;
		//	b1 = b1 % numVertices;

		//	if (a0 == a1)
		//		return false;
		//	if (b0 == b1)
		//		return false;

		//	if ( a0 < a1 )
		//	{
		//		if ( b0 < b1 )
		//		{
		//			return a1 <= b0 || b1 <= a0;
		//		}
		//		else
		//		{
		//			return b1 <= a0 && a1 <= b0;
		//		}
		//	}
		//	else
		//	{
		//		if ( b0 < b1 )
		//		{
		//			return a1 <= b0 && b1 <= a0;
		//		}
		//		else
		//		{
		//			return false;
		//		}
		//	}

		//}

		//List<Point> simplify(List<Point> contour, double eps )
		//{
		//	MatOfPoint matPoint = new MatOfPoint();
		//	matPoint.fromList(contour);
		//	MatOfInt convexhull = new MatOfInt();
		//	Imgproc.convexHull(matPoint, convexhull, true);

		//	if (convexhull.rows() < 4)
		//	{
		//		return null;
		//	}

		//	int numVertices = convexhull.rows();
		//	List<Point> listConvex = new List<Point>(numVertices);
		//	for (int i = 0; i < numVertices; i++)
		//	{
		//		int index = (int)convexhull.get(i, 0)[0];
		//		listConvex.Add(contour[index]);
		//	}

		//	int i0 = 0;
		//	int i1 = numVertices;

		//	if (false == findStraightLine2(listConvex, ref i0, ref i1, eps) || i0 == i1)
		//	{
		//		return null;
		//	}

		//	int i2 = i0;
		//	if (false == findStraightLineForward(listConvex, i1, ref i2, eps) || i2 == i0)
		//	{
		//		return null;
		//	}

		//	int i_1 = i1;
		//	if (false == findStraightLineBackward(listConvex, ref i_1, i0, eps) || i_1 == i1 || i2 == i_1
		//		|| checkNonOverlap(i1, i2, i_1, i0, numVertices) == false)
		//	{
		//		return null;
		//	}

		//	int i3 = i0;
		//	if (false == findStraightLineForward(listConvex, i2, ref i3, eps) || i3 == i0)
		//	{
		//		return null;
		//	}

		//	List<Point> listRet = new List<Point>();

		//	if (i3 == i_1)
		//	{

		//		// i0, i1, i2, i_1
		//		listRet.Add(listConvex[i0]);
		//		listRet.Add(listConvex[i1]);
		//		listRet.Add(listConvex[i2]);
		//		listRet.Add(listConvex[i_1]);
		//		return listRet;
		//	}
		//	else if (checkNonOverlap(i2, i3, i_1, i0, numVertices) == false)
		//	{
		//		// i2, i3 / i_1, i0  중에서 극점을 찾아야 함
		//		// i0, i1, i2, i_1
		//		listRet.Add(listConvex[i0]);
		//		listRet.Add(listConvex[i1]);
		//		listRet.Add(listConvex[i2]);
		//		listRet.Add(listConvex[i_1]);
		//		return listRet;
		//	}

		//	int i_2 = i1;
		//	if (false == findStraightLineBackward(listConvex, ref i_2, i_1, eps) || i_2 == i1)
		//	{
		//		return null;
		//	}

		//	if (i_2 == i2)
		//	{
		//		// i0, i1, i2, i_1
		//		listRet.Add(listConvex[i0]);
		//		listRet.Add(listConvex[i1]);
		//		listRet.Add(listConvex[i2]);
		//		listRet.Add(listConvex[i_1]);
		//		return listRet;
		//	}
		//	else if (checkNonOverlap(i_2, i_1, i1, i2, numVertices) == false)
		//	{
		//		//i_2, i_1 / i1, i2 중에서 극점을 찾아야 함
		//		// i0, i1, i2, i_1
		//		listRet.Add(listConvex[i0]);
		//		listRet.Add(listConvex[i1]);
		//		listRet.Add(listConvex[i2]);
		//		listRet.Add(listConvex[i_1]);
		//		return listRet;
		//	}

		//	if (checkNonOverlap(i2, i3, i_2, i_1, numVertices) == false)
		//	{
		//		// i0, i1, i2, i_1
		//		listRet.Add(listConvex[i0]);
		//		listRet.Add(listConvex[i1]);
		//		listRet.Add(listConvex[i2]);
		//		listRet.Add(listConvex[i_1]);
		//		return listRet;
		//	}

		//	// 영역 비교

		//	//{
		//	//	// i_2, i_1, i0, i1
		//	//	// i_1, i0, i1, i2
		//	//	// i0, i1, i2, i3
		//	//	List<Point> list1 = new List<Point>();
		//	//	list1.Add(listConvex[i_2]);
		//	//	list1.Add(listConvex[i_1]);
		//	//	list1.Add(listConvex[i0]);
		//	//	list1.Add(listConvex[i1]);
		//	//	MatOfPoint mat = new MatOfPoint();
		//	//	mat.fromList(list1);
		//	//	double area1 = Imgproc.contourArea(mat, false);

		//	//	List<Point> list2 = new List<Point>();
		//	//	list2.Add(listConvex[i_1]);
		//	//	list2.Add(listConvex[i0]);
		//	//	list2.Add(listConvex[i1]);
		//	//	list1.Add(listConvex[i2]);
		//	//	mat.fromList(list2);
		//	//	double area2 = Imgproc.contourArea(mat, false);

		//	//	List<Point> list3 = new List<Point>();
		//	//	list3.Add(listConvex[i0]);
		//	//	list3.Add(listConvex[i1]);
		//	//	list3.Add(listConvex[i2]);
		//	//	list3.Add(listConvex[i3]);
		//	//	mat.fromList(list3);
		//	//	double area3 = Imgproc.contourArea(mat, false);

		//	//	mat.fromList(listConvex);
		//	//	double area = Imgproc.contourArea(mat, false);

		//	//	if ( area2 >= area1 )
		//	//	{
		//	//		if ( area2 >= area3 )
		//	//		{
		//	//			if (area - area2 < area2)
		//	//				return list2;
		//	//		}
		//	//		else
		//	//		{
		//	//			if (area - area3 < area3)
		//	//				return list3;
		//	//		}
		//	//	}
		//	//	else
		//	//	{
		//	//		if ( area1 >= area3 )
		//	//		{
		//	//			if (area - area1 < area1)
		//	//				return list1;
		//	//		}
		//	//		else
		//	//		{
		//	//			if (area - area3 < area3)
		//	//				return list3;
		//	//		}
		//	//	}
		//	//}
		//	return null;
		//}

		//static float calcDistanceSqr(const Point2i& a, Point2i b)
		//{
		//	float diffx = b.x - a.x;
		//	float diffy = b.y - a.y;
		//	return diffx * diffx + diffy * diffy;
		//}

		//static float calcDistanceNumerator(Point2i a, Point2i b, Point2i p)
		//{
		//	float d = (b.y - a.y) * p.x - (b.x - a.x) * p.y + b.x * a.y - b.y * a.x;
		//	return d * d;
		//}

		//Mat solvePerspective(Mat a /*CV_CV_64FC2*/, Mat obj/*CV_CV_64FC3*/, Mat camMatrix, Mat distortionCoeff)
		//{
		//	using (Mat rvec = new Mat())
		//	{
		//		using (Mat tvec = new Mat())
		//		{

		//			try
		//			{
		//				Cv2.SolvePnP(obj, a, camMatrix, distortionCoeff, rvec, tvec, false, SolvePnPFlags.P3P);
		//			}
		//			catch (Exception)
		//			{
		//				return new Mat();
		//			}

		//			using (Mat rod = new Mat(3, 3, CV_64FC1))
		//			{
		//				Cv2.Rodrigues(rvec, rod);

		//				List<Mat> src = new List<Mat>();
		//				src.Add(rod);
		//				src.Add(tvec);
		//				using (Mat concat = new Mat())
		//				{
		//					Cv2.HConcat(src, concat);
		//					return camMatrix * concat;
		//				}
		//			}
		//		}
		//	}
		//}

		//static Point computeCameraSpaceProjection(Mat matCamera, Point3 p3)
		//{
		//	Mat matP3 = new Mat(3, 1, CV_64FC1);
		//	matP3.put(0, 0, p3.x, p3.y, p3.Z);
		//	Mat matP2 = matCamera * matP3;
		//	double[] arrP2 = new double[3];
		//	matP2.get(0, 0, arrP2);
		//	return new Point(arrP2[0] / arrP2[2], arrP2[1] / arrP2[2]);
		//}


		//static double normalize(Point a)
		//{
		//	double len = a.dot(a);
		//	if (len == 0.0)
		//	{
		//		a.x = 1;
		//		a.y = 0;
		//		return 0.0;
		//	}
		//	len = std::sqrt(len);
		//	double invlen = 1.0 / len;
		//	a.x *= invlen;
		//	a.y *= invlen;
		//	return len;
		//}

		static float Normalize(Point2& a)
		{
			float len = a.dot(a);
			if (len == 0.0f)
			{
				a.x = 1;
				a.y = 0;
				return 0.0f;
			}
			len = std::sqrt(len);
			float invlen = 1.0f / len;
			a.x *= invlen;
			a.y *= invlen;
			return len;
		}

		static float Normalize(Point3& a)
		{
			float len = a.dot(a);
			if (len == 0.0f)
			{
				a.x = 1;
				a.y = 0;
				a.z = 0;
				return 0.0f;
			}
			len = std::sqrt(len);
			float invlen = 1.0f / len;
			a.x *= invlen;
			a.y *= invlen;
			a.z *= invlen;
			return len;
		}

		static float Normalize(Vector3& v)
		{
			float len = v.dot(v);
			if (len == 0.f)
			{
				v[0] = 1.f;
				v[1] = v[2] = 0.f;
				return 0.f;
			}
			len = std::sqrt(len);
			float invlen = 1.f / len;
			v *= invlen;
			return len;
		}


		static bool Contain(const OpenCVRect& rectme, const OpenCVRect& rect)
		{
			return rectme.x <= rect.x &&
				(rect.x + rect.width) <= (rectme.x + rectme.width) &&
				rectme.y <= rect.y &&
				(rect.y + rect.height) <= (rectme.y + rectme.height);
		}


		bool outer(const Point2i& p0, const Point2i& p1, const Point2i& p)
		{
			int x = p1.x - p0.x;
			int y = p1.y - p0.y;
			return (y * (p.x - p0.x) + (-x) * (p.y - p0.y)) > 0;
		}

		//bool indexOrderCheck( int[] indices)
		//{
		//	int iStart = indices[0];
		//	int iPrev = iStart;
		//	int iRollback = 0;
		//	for (int i = 1; i < 4; i++)
		//	{
		//		if (indices[i] == iPrev)
		//			return false;
		//		if (iRollback == 0)
		//		{
		//			if (indices[i] < iPrev)
		//			{
		//				if (indices[i] >= iStart)
		//					return false;
		//				iRollback = 1;
		//			}
		//		}
		//		else if (iRollback == 1)
		//		{
		//			if (indices[i] <= iPrev || indices[i] >= iStart)
		//				return false;
		//		}
		//		iPrev = indices[i];
		//	}
		//	return true;
		//}

		//void enlargeQuad( List<Point2f> contour, List<Point2f> quadApprox )
		//{
		//	int iNumVertices = contour.size();
		//	if (iNumVertices <= 4 || quadApprox.size() != 4 )
		//		return;

		//	int[] indices = new int[4];
		//	for( int i = 0; i < 4; i++ )
		//	{
		//		Point p = quadApprox[i];
		//		indices[i] = -1;
		//		for ( int j = 0; j < iNumVertices; j++ )
		//		{
		//			Point c = contour[j];
		//			if ( p.x == c.x && p.y == c.y )
		//			{
		//				indices[i] = j;
		//				break;
		//			}
		//		}
		//		if (indices[i] < 0)
		//			return;
		//	}
		//	if (!indexOrderCheck(indices))
		//		return;

		//	bool bChangedAtAll = false;
		//	bool bChanged = false;
		//	int[] indicesTmp = new int[4];
		//	while(true)
		//	{
		//		bChanged = false;

		//		indicesTmp[0] = indices[0];
		//		indicesTmp[1] = indices[1];
		//		indicesTmp[2] = indices[2];
		//		indicesTmp[3] = indices[3];

		//		for ( int i = 0; i < 4; i++ )
		//		{
		//			int i_index = indices[i];
		//			int j_index = indices[(i + 1) % 4];
		//			int k_index = indices[(i + 2) % 4];

		//			Point p0 = contour[i_index];
		//			Point p1 = contour[j_index];
		//			Point p2 = contour[k_index];
		//			Point p01 = p1 - p0;

		//			if (j_index == i_index)
		//				continue;
		//			if (k_index < i_index)
		//				k_index = k_index + iNumVertices;
		//			i_index++;
		//			k_index--;
		//			if (i_index > k_index)
		//				continue;
		//			bool bSet = false;
		//			double mindot = 0.0;
		//			for( int l = i_index; l <= k_index; l++ )
		//			{
		//				int l_index = l % iNumVertices;
		//				Point p = contour[l_index];
		//				if (outer(p0, p1, p) && outer(p1, p2, p))
		//				{
		//					Point diff = p - p0;
		//					normalize(diff);
		//					double dot = p01.dot(diff);
		//					if ( bSet == false || dot < mindot )
		//					{
		//						bSet = true;
		//						mindot = dot;
		//						bChanged = true;
		//						indicesTmp[(i + 1) % 4] = l_index;
		//					}
		//				}
		//			}
		//		}
		//		if (bChanged == false)
		//			break;
		//		if (indexOrderCheck(indicesTmp) == false)
		//			break;
		//		indices[0] = indicesTmp[0];
		//		indices[1] = indicesTmp[1];
		//		indices[2] = indicesTmp[2];
		//		indices[3] = indicesTmp[3];
		//		bChangedAtAll = true;
		//	}
		//	if (bChangedAtAll == true )
		//	{
		//		for( int i = 0; i < 4; i++ )
		//		{
		//			quadApprox[i].x = contour[indices[i]].x;
		//			quadApprox[i].y = contour[indices[i]].y;
		//		}
		//	}
		//}




		int determineBit(const std::vector<std::vector<Point2i>>& contours, const std::vector<cv::Vec4i>& hierarchy, int c, double dArea, double dMinRatio)
		{
			assert(c >= 0 && c < (int)hierarchy.size());


			c = hierarchy[c][HIERARCHY_CHILD];
			if (c == -1)
				return -1;

			assert(c >= 0 && c < (int)hierarchy.size());

			double maxblobarea = 0.0;
			int iMaxArea = -1;
			for (; c >= 0; c = hierarchy[c][HIERARCHY_NEXT])
			{
				assert(c < (int)contours.size());

				const std::vector<Point2i>& contour = contours[c];
				double blobarea = cv::contourArea(contour, true);
				if (blobarea > maxblobarea)
				{
					maxblobarea = blobarea;
					iMaxArea = c;
				}
			}
			if (iMaxArea >= 0 && maxblobarea >= dMinRatio * dArea)
			{
				return iMaxArea;
			}
			return -1;
		}

		//CvCBQuad allocateCvQuad()
		//{
		//	m_cvTmpCornersRuntime
		//}

		//void     deallocateCvQuad( CvCBQuad q )
		//{
		//}

		int DistanceSq(const Point2i& a, const Point2i& b)
		{
			return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
		}

		float DistanceSq(const Point2& a, const Point2& b)
		{
			return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
		}

		float DistanceSq(const Point3& a, const Point3& b)
		{
			return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z);
		}

		float DistanceSq(const Vector3& a, const Vector3& b)
		{
			return (a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]);
		}

		Vector3 MultiplyPoint3x4(const Matrix4x4& mat, const Vector3& p)
		{
			Vector4 result = mat * Vector4(p[0], p[1], p[2], 1.f);
			return Vector3(result[0], result[1], result[2]);
		}

		Quaternion QuaternionMultiply(const Quaternion& q1, const Quaternion& q2)
		{
			Quaternion q;

			q[3] = q1[3] * q2[3] - q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2];
			q[0] = q1[3] * q2[0] + q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1];
			q[1] = q1[3] * q2[1] + q1[1] * q2[3] + q1[2] * q2[0] - q1[0] * q2[2];
			q[2] = q1[3] * q2[2] + q1[2] * q2[3] + q1[0] * q2[1] - q1[1] * q2[0];

			return q;
		}

		Quaternion RotationVectorToQuaternion(const Vector3& v)
		{
			Vector3 n(v);
			float len = Normalize(n);
			float cos = std::cos(len * 0.5f);
			float sin = std::sin(len * 0.5f);
			n *= sin;
			return Quaternion(n[0], n[1], n[2], cos);
		}

#ifdef USE_LIBCBDETECTOR

		void convertCorners(const cbdetect::Corner& icorners, std::vector<CvCBCorner>& ocorners, OpenCVRect rect )
		{
			int iNumCorners = icorners.p.size();
			ocorners.resize(iNumCorners+1);
			for (int i = 0; i < iNumCorners; i++)
			{
				CvCBCorner& ocorner = ocorners[i];
				ocorner.pt = cv::Point2d(icorners.p[i].x + rect.x, icorners.p[i].y + rect.y);
				ocorner.mark = 0;
				ocorner.imageError = icorners.r[i];
			}
			CvCBCorner& cdummy = ocorners.back();
			cdummy.pt = Point2(FLT_MAX, FLT_MAX);
			cdummy.mark = 0;
			cdummy.imageError = 0;
		}

#else  USE_LIBCBDETECTOR

		int generateQuads__detect(
			const std::vector<std::vector<Point2i>>& scaleContours
			, const std::vector<cv::Vec4i>& hierarchy
			, std::vector<CvCBQuad>& quads, std::vector<CvCBCorner>& corners, const OpenCVRect& rect
			, int width, int height, int dilation, cv::Mat& imageColor, bool firstRun)
		{
			if (scaleContours.empty() || hierarchy.empty() || hierarchy.size() != scaleContours.size())
				return 0;

			//quads.Clear();
			int minPerimeterPixels =
				(int)(m_parameters__fixed.detectorParams.minMarkerPerimeterRate * std::max(width, height));
			int maxPerimeterPixels =
				(int)(m_parameters__fixed.detectorParams.maxMarkerPerimeterRate * std::max(width, height));
			float approxAccuracyRate = (float)m_parameters__fixed.detectorParams.polygonalApproxAccuracyRate;
			float minCornerDistanceRate = (float)m_parameters__fixed.detectorParams.minCornerDistanceRate;

			//Profiler.EndSample();


			//MatOfPoint2f approxPoly2f_tmp = new MatOfPoint2f();
			//MatOfPoint approxPoly2_tmp = new MatOfPoint();
			//MatOfPoint2f contour2f = new MatOfPoint2f();
			//MatOfPoint approxPoly = new MatOfPoint();
			//float[] afApproxPoly2 = new float[8];

			float boxarea = m_parameters__fixed.markerSize * m_parameters__fixed.markerSize;
			float blobarea = MY_PI * m_parameters__fixed.blobRadius * m_parameters__fixed.blobRadius;
			float ratio = blobarea / boxarea * m_parameters__fixed.blobDetectionTolerance;


			//for (int c = 0; c < scaleContours.size(); c++ )
			double contourArea = 0.0;

			std::vector<QuadStack>& stack = m_quadStack__detect;
			stack.clear();
			stack.resize(scaleContours.size() + 1);

			stack[0].hole = false;
			stack[0].quadGenerated = false;
			stack[0].this_index = 0;
			stack[0].child_index = hierarchy[0][HIERARCHY_CHILD];
			int iTopStack = 0;

			while (true)
			{
				assert(iTopStack >= 0 && iTopStack < (int)stack.size());
				int child_index = stack[iTopStack].child_index;
				if (child_index >= 0)
				{
					assert(child_index < (int)hierarchy.size());

					int iPushStack = iTopStack + 1;
					assert(iPushStack < (int)stack.size());
					stack[iPushStack].hole = !stack[iTopStack].hole;
					stack[iPushStack].this_index = child_index;
					stack[iPushStack].child_index = hierarchy[child_index][HIERARCHY_CHILD];
					stack[iTopStack].child_index = hierarchy[child_index][HIERARCHY_NEXT];
					iTopStack = iPushStack;
					continue;
				}

				if (stack[iTopStack].quadGenerated == false && stack[iTopStack].hole == false)
				{
					int c = stack[iTopStack].this_index;
					assert(c >= 0 && c < (int)scaleContours.size());

					//if ((int)hierarcyvalues[2] != -1 || (int)hierarcyvalues[3] == -1)
					//	continue;
					const std::vector<Point2i>& contour = scaleContours[c];

					//Imgproc.drawContours(m_curColor__track, scaleContours, c, new Scalar(255, 255, 0, 255));

					if (contour.size() < 4)
					{
						goto outer;
					}

					contourArea = -cv::contourArea(contour, true);
					if (contourArea <= 0)
					{
						//Imgproc.drawContours(m_curColor__track, scaleContours, c, new Scalar(255, 0, 0, 255));
						goto outer;
					}

					double arcLength = cv::arcLength(contour, true);
					if (arcLength < minPerimeterPixels || arcLength > maxPerimeterPixels)
					{
						goto outer;
					}


					//MatOfInt convexhull = new MatOfInt();
					//Imgproc.convexHull(contour, convexhull, true);
					//if (convexhull.rows() < 4)
					//	continue;

					//int numVertices = convexhull.rows();
					//List<Point> listConvex = new List<Point>(numVertices);
					//for (int i = 0; i < numVertices; i++)
					//{
					//	int index = (int)convexhull.get(i, 0)[0];
					//	listConvex.Add(listContour[index]);
					//}
					//contour2f.fromList(listConvex);


					float minCornerDistancePixels = (float)(minCornerDistanceRate * arcLength);

					int min_approx_level = 2;
					int max_approx_level = 0;
					//if (firstRun == true)
					//	max_approx_level = 3;
					//else
					max_approx_level = 7;// std::max(7, (int) (arcLength * approxAccuracyRate));
					int approx_level = 0;

					//List<Point> approxPolyList = null;

					//Profiler.BeginSample("approxPolyDP");
					std::vector<Point2i>& approxPoly2_tmp2 = m_approxPoly2Temp2__detect;
					std::vector<Point2i>& approxPoly2_tmp = m_approxPoly2Temp__detect;
					approxPoly2_tmp2.clear();
					approxPoly2_tmp.clear();

					for (approx_level = min_approx_level; approx_level <= max_approx_level; approx_level++)
					{
						cv::approxPolyDP(contour, /*out*/ approxPoly2_tmp2, (double)approx_level, true);
						if (approxPoly2_tmp2.empty() == false)
							cv::approxPolyDP(approxPoly2_tmp2, approxPoly2_tmp, (double)approx_level, true);

						if (approxPoly2_tmp.empty() == false && approxPoly2_tmp.size() == 4)
						{
							if (cv::isContourConvex(approxPoly2_tmp))
							{
								//Imgproc.drawContours(image, scaleContours, c, new Scalar(0), -1);
								break;
							}
						}

						//if (approxPoly2f_tmp.rows() == 4 )
						//{
						//}
						////List<Point> listApproxPoly2f_tmp = approxPoly2f_tmp.toList();

						//	//approxPolyList = calculateRectangleBound2(listApproxPoly2f_tmp, minCornerDistancePixels, 0.0, (double)approx_level, /*out*/quadType
						//	//	, approx_level == max_approx_level);
						//	//if (approxPolyList != null)
						//	//{

						//	//	break;
						//	//}
						//if (listContour.size() == 4)
						//{
						//	approxPolyList = listContour;
						//	break;
						//}
						//approxPolyList = simplify(listContour, (double)approx_level);
						//if (approxPolyList != null)
						//	break;

					}

					//Profiler.EndSample();

					if (approx_level <= max_approx_level)
					{
						//approxPoly2f_tmp.get(0, 0, afApproxPoly2);

						//enlargeQuad(listContour, approxPolyList);
						int i = 0;
						for (i = 0; i < 4; i++)
						{
							float dist2 = (float)DistanceSq(approxPoly2_tmp[i], approxPoly2_tmp[((i + 1) & 3)]);
							if (dist2 < minCornerDistancePixels * minCornerDistancePixels)
								break;
						}
						if (i < 4)
							goto outer;

						//if (imageColor != null)
						//{
						//	for (i = 0; i < 4; i++)
						//	{
						//		Point a = new Point(afApproxPoly2[2 * i], afApproxPoly2[2 * i + 1]);
						//		Point b = new Point(afApproxPoly2[2 * ((i+1)%4)], afApproxPoly2[2 * ((i + 1) % 4) + 1]);
						//		Imgproc.line(imageColor, a, b, new Scalar(255, 255, 255, 255));
						//	}
						//}

						quads.resize(quads.size() + 1);
						CvCBQuad& q = quads.back();
						q.group_idx = -1;
						//q.area = (float)contourArea;
						int iBlobIndex = determineBit(scaleContours, hierarchy, c, contourArea, ratio);
						q.bit = (iBlobIndex >= 0) ? true : false;
						//if (iBlobIndex >= 0)
						//{
						//	q.bitContour = scaleContours[iBlobIndex];
						//}
						size_t corner_offset = corners.size();
						corners.resize(corner_offset + 4);
						CvCBCorner* q_corners[4];

						for (i = 0; i < 4; i++)
						{
							q_corners[i] = &corners[corner_offset + i];
							q.corners[i] = (CvCBCorner*)((char*)q_corners[i] - (char*)&corners[0]);
							q_corners[i]->pt.x = (float)(approxPoly2_tmp[3 - i].x + rect.x);
							q_corners[i]->pt.y = (float)(approxPoly2_tmp[3 - i].y + rect.y);
							q_corners[i]->imageError = dilation;
						}
						float bound_min_x = 0;
						float bound_min_y = 0;
						float bound_max_x = 0;
						float bound_max_y = 0;
						for (i = 0; i < 4; i++)
						{
							float d = DistanceSq(q_corners[i]->pt, q_corners[(i + 1) & 3]->pt);
							if (i == 0 || q.edge_len_sqr > d)
								q.edge_len_sqr = d;
							if (i == 0 || q.edge_max_sqr < d)
								q.edge_max_sqr = d;
							if (i == 0)
							{
								bound_min_x = bound_max_x = q_corners[i]->pt.x;
								bound_min_y = bound_max_y = q_corners[i]->pt.y;
							}
							else
							{
								bound_min_x = std::min(bound_min_x, q_corners[i]->pt.x);
								bound_min_y = std::min(bound_min_y, q_corners[i]->pt.y);
								bound_max_x = std::max(bound_max_x, q_corners[i]->pt.x);
								bound_max_y = std::max(bound_max_y, q_corners[i]->pt.y);
							}
							switch (i)
							{
							case 0:
								q.max_width_sqr = d;
								q.min_width_sqr = d;
								break;
							case 1:
								q.max_height_sqr = d;
								q.min_height_sqr = d;
								break;
							case 2:
								q.max_width_sqr = std::max(q.max_width_sqr, d);
								q.min_width_sqr = std::min(q.min_width_sqr, d);
								break;
							case 3:
								q.max_height_sqr = std::max(q.max_height_sqr, d);
								q.min_height_Sqr = std::min(q.min_height_sqr, d);
								break;
							}
						}
						{
							q.bound.x = (int)bound_min_x;
							q.bound.y = (int)bound_min_y;
							q.bound.width = ((int)bound_max_x) - q.bound.x + 1;
							q.bound.height = ((int)bound_max_y) - q.bound.y + 1;
						}

						stack[iTopStack].quadGenerated = true;

					}
				}
			outer:
				if (iTopStack > 0)
				{
					int iPopStack = iTopStack - 1;
					stack[iPopStack].quadGenerated = stack[iPopStack].quadGenerated || stack[iTopStack].quadGenerated;
					iTopStack = iPopStack;
				}
				else
				{
					assert(stack[0].this_index >= 0 && stack[0].this_index < (int)hierarchy.size());
					int sibling = hierarchy[stack[0].this_index][HIERARCHY_NEXT];
					if (sibling < 0)
						break;

					assert(sibling < (int)hierarchy.size());
					stack[0].quadGenerated = false;
					stack[0].this_index = sibling;
					stack[0].child_index = hierarchy[sibling][HIERARCHY_CHILD];
				}
			}
			return (int)quads.size();
		}



		void findQuadNeighbors__detect(std::vector<CvCBQuad>& quads)
		{

			// Thresh dilation is used to counter the effect of dilation on the
			// distance between 2 neighboring corners. Since the distance below is 
			// computed as its square, we do here the same. Additionally, we take the
			// conservative assumption that dilation was performed using the 3x3 CROSS
			// kernel, which coresponds to the 4-neighborhood.
			//double thresh_dilation = (double)(2 * dilation + 3) * (2 * dilation + 3) * 2;   // the "*2" is for the x and y component
			//double thresh_dilation = (double)(2 * dilation + 7) * (2 * dilation + 7) * 2;   // the "*2" is for the x and y component
			float dist;

			// Find quad neighbors
			for (int idx = 0; idx < (int)quads.size(); idx++)
			{
				CvCBQuad& cur_quad = quads[idx];

				// Go through all quadrangles and label them in groups
				// For each corner of this quadrangle
				for (int i = 0; i < 4; i++)
				{

					float min_dist = std::numeric_limits<float>::max();
					int closest_corner_idx = -1;
					CvCBQuad* closest_quad = nullptr;
					CvCBCorner* closest_corner = nullptr;

					if (cur_quad.neighbors[i] != nullptr)
						continue;

					Point2 pt = cur_quad.corners[i]->pt;
					int ptImageError = cur_quad.corners[i]->imageError;

					// Find the closest corner in all other quadrangles
					for (int k = 0; k < (int)quads.size(); k++)
						//for (int k = idx + 1; k < quads.size(); k++)
					{
						if (k == idx)
							continue;

						CvCBQuad& quad_k = quads[k];

						for (int j = 0; j < 4; j++)
						{
							// If it already has a neighbor
							if (quad_k.neighbors[j] != nullptr)
								continue;

							dist = DistanceSq(pt, quad_k.corners[j]->pt);

							// The following "if" checks, whether "dist" is the
							// shortest so far and smaller than the smallest
							// edge length of the current and target quads

							float maxbound = cur_quad.corners[i]->imageError + ptImageError + (float)m_parameters__fixed.cornerPixelTolerance;
							maxbound = maxbound * maxbound * 2;

							if (dist < min_dist &&
								//dist <= (cur_quad.edge_len_sqr + thresh_dilation) &&
								//dist <= (quad_k.edge_len_sqr + thresh_dilation))
								dist <= maxbound)
							{
								// First Check everything from the viewpoint of the current quad
								// compute midpoints of "parallel" quad sides 1
								float x1 = (cur_quad.corners[i]->pt.x + cur_quad.corners[(i + 1) % 4]->pt.x) / 2;
								float y1 = (cur_quad.corners[i]->pt.y + cur_quad.corners[(i + 1) % 4]->pt.y) / 2;
								float x2 = (cur_quad.corners[(i + 2) % 4]->pt.x + cur_quad.corners[(i + 3) % 4]->pt.x) / 2;
								float y2 = (cur_quad.corners[(i + 2) % 4]->pt.y + cur_quad.corners[(i + 3) % 4]->pt.y) / 2;
								// compute midpoints of "parallel" quad sides 2
								float x3 = (cur_quad.corners[i]->pt.x + cur_quad.corners[(i + 3) % 4]->pt.x) / 2;
								float y3 = (cur_quad.corners[i]->pt.y + cur_quad.corners[(i + 3) % 4]->pt.y) / 2;
								float x4 = (cur_quad.corners[(i + 1) % 4]->pt.x + cur_quad.corners[(i + 2) % 4]->pt.x) / 2;
								float y4 = (cur_quad.corners[(i + 1) % 4]->pt.y + cur_quad.corners[(i + 2) % 4]->pt.y) / 2;

								// MARTIN: Heuristic
								// For the corner "j" of quad "k" to be considered, 
								// it needs to be on the same side of the two lines as 
								// corner "i". This is given, if the cross product has 
								// the same sign for both computations below:
								float a1 = x1 - x2;
								float b1 = y1 - y2;
								// the current corner
								float c11 = cur_quad.corners[i]->pt.x - x2;
								float d11 = cur_quad.corners[i]->pt.y - y2;
								// the candidate corner
								float c12 = quad_k.corners[j]->pt.x - x2;
								float d12 = quad_k.corners[j]->pt.y - y2;
								float sign11 = a1 * d11 - c11 * b1;
								float sign12 = a1 * d12 - c12 * b1;

								float a2 = x3 - x4;
								float b2 = y3 - y4;
								// the current corner
								float c21 = cur_quad.corners[i]->pt.x - x4;
								float d21 = cur_quad.corners[i]->pt.y - y4;
								// the candidate corner
								float c22 = quad_k.corners[j]->pt.x - x4;
								float d22 = quad_k.corners[j]->pt.y - y4;
								float sign21 = a2 * d21 - c21 * b2;
								float sign22 = a2 * d22 - c22 * b2;


								// Then make shure that two border quads of the same row or
								// column don't link. Check from the current corner's view,
								// whether the corner diagonal from the candidate corner
								// is also on the same side of the two lines as the current
								// corner and the candidate corner.
								float c13 = quad_k.corners[(j + 2) % 4]->pt.x - x2;
								float d13 = quad_k.corners[(j + 2) % 4]->pt.y - y2;
								float c23 = quad_k.corners[(j + 2) % 4]->pt.x - x4;
								float d23 = quad_k.corners[(j + 2) % 4]->pt.y - y4;
								float sign13 = a1 * d13 - c13 * b1;
								float sign23 = a2 * d23 - c23 * b2;


								// Then check everything from the viewpoint of the candidate quad
								// compute midpoints of "parallel" quad sides 1
								float u1 = (quad_k.corners[j]->pt.x + quad_k.corners[(j + 1) % 4]->pt.x) / 2;
								float v1 = (quad_k.corners[j]->pt.y + quad_k.corners[(j + 1) % 4]->pt.y) / 2;
								float u2 = (quad_k.corners[(j + 2) % 4]->pt.x + quad_k.corners[(j + 3) % 4]->pt.x) / 2;
								float v2 = (quad_k.corners[(j + 2) % 4]->pt.y + quad_k.corners[(j + 3) % 4]->pt.y) / 2;
								// compute midpoints of "parallel" quad sides 2
								float u3 = (quad_k.corners[j]->pt.x + quad_k.corners[(j + 3) % 4]->pt.x) / 2;
								float v3 = (quad_k.corners[j]->pt.y + quad_k.corners[(j + 3) % 4]->pt.y) / 2;
								float u4 = (quad_k.corners[(j + 1) % 4]->pt.x + quad_k.corners[(j + 2) % 4]->pt.x) / 2;
								float v4 = (quad_k.corners[(j + 1) % 4]->pt.y + quad_k.corners[(j + 2) % 4]->pt.y) / 2;

								// MARTIN: Heuristic
								// for the corner "j" of quad "k" to be considered, it 
								// needs to be on the same side of the two lines as 
								// corner "i". This is again given, if the cross
								//product has the same sign for both computations below:
								float a3 = u1 - u2;
								float b3 = v1 - v2;
								// the current corner
								float c31 = cur_quad.corners[i]->pt.x - u2;
								float d31 = cur_quad.corners[i]->pt.y - v2;
								// the candidate corner
								float c32 = quad_k.corners[j]->pt.x - u2;
								float d32 = quad_k.corners[j]->pt.y - v2;
								float sign31 = a3 * d31 - c31 * b3;
								float sign32 = a3 * d32 - c32 * b3;

								float a4 = u3 - u4;
								float b4 = v3 - v4;
								// the current corner
								float c41 = cur_quad.corners[i]->pt.x - u4;
								float d41 = cur_quad.corners[i]->pt.y - v4;
								// the candidate corner
								float c42 = quad_k.corners[j]->pt.x - u4;
								float d42 = quad_k.corners[j]->pt.y - v4;
								float sign41 = a4 * d41 - c41 * b4;
								float sign42 = a4 * d42 - c42 * b4;


								// Then make shure that two border quads of the same row or
								// column don't link. Check from the candidate corner's view,
								// whether the corner diagonal from the current corner
								// is also on the same side of the two lines as the current
								// corner and the candidate corner.
								float c33 = cur_quad.corners[(i + 2) % 4]->pt.x - u2;
								float d33 = cur_quad.corners[(i + 2) % 4]->pt.y - v2;
								float c43 = cur_quad.corners[(i + 2) % 4]->pt.x - u4;
								float d43 = cur_quad.corners[(i + 2) % 4]->pt.y - v4;
								float sign33 = a3 * d33 - c33 * b3;
								float sign43 = a4 * d43 - c43 * b4;


								// Check whether conditions are fulfilled
								if (((sign11 < 0 && sign12 < 0) || (sign11 > 0 && sign12 > 0)) &&
									((sign21 < 0 && sign22 < 0) || (sign21 > 0 && sign22 > 0)) &&
									((sign31 < 0 && sign32 < 0) || (sign31 > 0 && sign32 > 0)) &&
									((sign41 < 0 && sign42 < 0) || (sign41 > 0 && sign42 > 0)) &&
									((sign11 < 0 && sign13 < 0) || (sign11 > 0 && sign13 > 0)) &&
									((sign21 < 0 && sign23 < 0) || (sign21 > 0 && sign23 > 0)) &&
									((sign31 < 0 && sign33 < 0) || (sign31 > 0 && sign33 > 0)) &&
									((sign41 < 0 && sign43 < 0) || (sign41 > 0 && sign43 > 0)))

								{
									closest_corner_idx = j;
									closest_quad = &quad_k;
									min_dist = dist;
								}
							}
						}
					}

					// Have we found a matching corner point?
					if (closest_corner_idx >= 0 && min_dist < std::numeric_limits<float>::max())
					{
						closest_corner = closest_quad->corners[closest_corner_idx];

						// Make shure that the closest quad does not have the current
						// quad as neighbor already
						int j;
						for (j = 0; j < 4; j++)
						{
							if (closest_quad->neighbors[j] == &cur_quad)
								break;
						}
						if (j < 4)
							continue;

						// We've found one more corner - remember it
						closest_corner->pt = (pt + closest_corner->pt) * 0.5f;
						closest_corner->imageError = (int)std::round((closest_corner->imageError + ptImageError) * 0.5f);
						//closest_corner.imageError = (int)std::round(
						//	std::max((closest_corner.imageError + ptImageError) * 0.5
						//	, std::sqrt(min_dist)));
						cur_quad.count++;
						cur_quad.neighbors[i] = closest_quad;
						cur_quad.corners[i] = closest_corner;

						closest_quad->count++;
						closest_quad->neighbors[closest_corner_idx] = &cur_quad;
						closest_quad->corners[closest_corner_idx] = closest_corner;
					}
				}
			}
		}

		int findConnectedQuads__detect(std::vector<CvCBQuad>& quads, int iFindStartIndex, std::vector<CvCBQuad*>& out_group, /*List<CvCBQuad> out_single_group,*/
			int group_idx, cv::Mat matColor_)
		{
			out_group.clear();

			if (iFindStartIndex >= (int)quads.size())
				return iFindStartIndex;

			std::vector<CvCBQuad*>& stack = m_findConnectedQuad__detect;
			stack.clear();

			// Scan the array for a first unlabeled quad
			for (; iFindStartIndex < (int)quads.size(); iFindStartIndex++)
			{
				//if ( (out_single_group != null || quad[iFindStartIndex].count > 0) && quad[iFindStartIndex].group_idx == -1 )
				//	break;
				if (quads[iFindStartIndex].count > 0 && quads[iFindStartIndex].group_idx == -1)
					break;
			}

			// Recursively find a group of connected quads starting from the seed
			// quad[i]
			if (iFindStartIndex >= (int)quads.size())
				return iFindStartIndex;

			CvCBQuad* q = &quads[iFindStartIndex];

			//if (q.neighbors[0] == null && q.neighbors[1] == null && q.neighbors[2] == null && q.neighbors[3] == null)
			//{
			//	if (out_single_group != null)
			//		out_single_group.Add(q);

			//	//if (matColor_ != null)
			//	//{
			//	//	for (int c = 0; c < 4; c++)
			//	//	{
			//	//		int c_ = (c + 1) % 4;
			//	//		Imgproc.line(matColor_, q.corners[c].pt, q.corners[c_].pt, new Scalar(255, 255, 255, 255));
			//	//	}
			//	//}
			//}
			//else
			//{
			out_group.push_back(q);
			q->group_idx = group_idx;

			stack.push_back(q);

			while (stack.empty() == false)
			{
				q = stack[stack.size() - 1];
				stack.pop_back();
				for (int j = 0; j < 4; j++)
				{
					CvCBQuad* neighbor = q->neighbors[j];

					// If he neighbor exists and the neighbor has more than 0 
					// neighbors and the neighbor has not been classified yet.
					if (neighbor != nullptr && neighbor->count > 0 && neighbor->group_idx == -1)
					{
						stack.push_back(neighbor);
						out_group.push_back(neighbor);
						neighbor->group_idx = group_idx;
					}
				}
			}
			//}

			return iFindStartIndex + 1;
		}

#endif USE_LIBCBDETECTOR

		//void cleanFoundConnectedQuads(List<CvCBQuad> quad_group)
		//{
		//	//CvPoint2D32f* centers = 0;

		//	Point center = new Point(0, 0);
		//	int i, j, k;

		//	int quad_count = quad_group.size();


		//	// Number of quads this pattern should contain
		//	//int count = ((pattern_size.width + 1) * (pattern_size.height + 1) + 1) / 2;
		//	int count = m_parameters__fixed.markerCodeLength * 2 + (m_parameters__fixed.markerCodeLength + 1) / 2;

		//	// If we have more quadrangles than we should, try to eliminate duplicates
		//	// or ones which don't belong to the pattern rectangle. Else go to the end
		//	// of the function
		//	if (quad_count <= count)
		//		return;

		//	// Create an array of quadrangle centers
		//	List<Point> centers = new List<Point>(quad_count);
		//	for (i = 0; i < quad_count; i++)
		//	{
		//		Point ci = new Point(0, 0);
		//		CvCBQuad q = quad_group[i];

		//		for (j = 0; j < 4; j++)
		//		{
		//			Point pt = q.corners[j].pt;
		//			ci += pt;
		//		}
		//		ci *= 0.25;

		//		// Centers(i), is the geometric center of quad(i)
		//		// Center, is the center of all found quads
		//		centers.Add(ci);
		//		center.x += ci.x;
		//		center.y += ci.y;
		//	}
		//	center.x /= quad_count;
		//	center.y /= quad_count;

		//	// If we have more quadrangles than we should, we try to eliminate bad
		//	// ones based on minimizing the bounding box. We iteratively remove the
		//	// point which reduces the size of the bounding box of the blobs the most
		//	// (since we want the rectangle to be as small as possible) remove the
		//	// quadrange that causes the biggest reduction in pattern size until we
		//	// have the correct number

		//	for (; quad_count > count; )
		//	{
		//		double min_box_area = double.MaxValue;
		//		int skip, min_box_area_index = -1;
		//		CvCBQuad q0 = null;
		//		CvCBQuad q = null;

		//		// For each point, calculate box area without that point
		//		for (skip = 0; skip < quad_count; skip++)
		//		{
		//			// get bounding rectangle
		//			Point temp = centers[skip];
		//			centers[skip] = center;
		//			MatOfPoint pointMat = new MatOfPoint();
		//			pointMat.fromList(centers);
		//			MatOfInt hullIndices = new MatOfInt();
		//			Imgproc.convexHull(pointMat, hullIndices, true);
		//			centers[skip] = temp;
		//			MatOfPoint hull = new MatOfPoint();
		//			for (i = 0; i < hullIndices.rows(); i++)
		//			{
		//				int index = (int)hullIndices.get(i, 0)[0];
		//				hull.push_back(pointMat.row(index));
		//			}
		//			double hull_area = Imgproc.contourArea(hull, false );
		//			// remember smallest box area
		//			if (hull_area < min_box_area)
		//			{
		//				min_box_area = hull_area;
		//				min_box_area_index = skip;
		//			}
		//		}

		//		q0 = quad_group[min_box_area_index];

		//		// remove any references to this quad as a neighbor
		//		for (i = 0; i < quad_count; i++)
		//		{
		//			q = quad_group[i];
		//			for (j = 0; j < 4; j++)
		//			{
		//				if (q.neighbors[j] == q0)
		//				{
		//					q.neighbors[j] = null;
		//					q.count--;
		//					for (k = 0; k < 4; k++)
		//					{
		//						if (q0.neighbors[k] == q)
		//						{
		//							q0.neighbors[k] = null;
		//							q0.count--;
		//							break;
		//						}
		//					}
		//					break;
		//				}
		//			}
		//		}

		//		// remove the quad by copying th last quad in the list into its place
		//		quad_count--;
		//		quad_group[min_box_area_index] = quad_group[quad_count];
		//		centers[min_box_area_index] = centers[quad_count];
		//		quad_group.RemoveAt(quad_count);
		//		centers.RemoveAt(quad_count);
		//	}

		//	return;
		//}


		SinglePattern* allocSinglePattern__detect(int pattern_type)
		{
			if (m_singlePatternBase__detect.empty() == true
				|| m_singlePatternBase__detect.front().pattern_type != (int)MarkerType_INVALID)
			{
				m_singlePatternBase__detect.push_back(std::move(SinglePattern()));
			}
			else
			{
				m_singlePatternBase__detect.splice(m_singlePatternBase__detect.end(), m_singlePatternBase__detect, m_singlePatternBase__detect.begin());
			}
			m_singlePatternBase__detect.back().pattern_type = pattern_type;
			return &m_singlePatternBase__detect.back();
		}

#ifdef USE_LIBCBDETECTOR

		void addPattern2__detect(std::vector<SinglePattern*>& foundPattern, SinglePattern* pattern)
		{
			if (pattern->pattern_type == (int)MarkerType_CYLINDER)
			{
				int iNumPatterns = (int)m_cylinderarkerInfos__fixed.size();
				int iMarkerCodeLength = m_parameters__fixed.markerCodeLength;

				int iPattern = 0;

				while (iPattern < foundPattern.size())
				{
					SinglePattern* existing = foundPattern[iPattern];

					if (existing->pattern_type == (int)MarkerType_CYLINDER)
					{
						if ((existing->pattern_index + existing->pattern_count) % iNumPatterns == pattern->pattern_index)
						{
							bool bDiff = false;
							int offset = existing->pattern_count * iMarkerCodeLength;
							for (int c = 0; c < iMarkerCodeLength; c++)
							{
								if (existing->block_quads[offset + c] != pattern->block_quads[c])
								{
									bDiff = true;
									break;
								}
							}
							if (!bDiff)
							{
								existing->pattern_count += pattern->pattern_count;
								existing->block_quads.insert(existing->block_quads.end()
									, pattern->block_quads.begin() + iMarkerCodeLength, pattern->block_quads.end());
								existing->pattern_widths.insert(existing->pattern_widths.end(), pattern->pattern_widths.begin(), pattern->pattern_widths.end());
								existing->min_max_wh_sqr.Merge(pattern->min_max_wh_sqr);

								if (iPattern < (int)foundPattern.size() - 1)
								{
									foundPattern[iPattern] = foundPattern.back();
									foundPattern.pop_back();
								}
								else
								{
									foundPattern.erase(foundPattern.begin() + iPattern);
								}
								pattern = existing;
								continue;
							}
						}
						else if ((pattern->pattern_index + pattern->pattern_count) % iNumPatterns == existing->pattern_index)
						{
							bool bDiff = false;
							int offset = pattern->pattern_count * iMarkerCodeLength;
							for (int c = 0; c < iMarkerCodeLength; c++)
							{
								if (pattern->block_quads[offset + c] != existing->block_quads[c])
								{
									bDiff = true;
									break;
								}
							}
							if (!bDiff)
							{
								pattern->pattern_count += existing->pattern_count;
								pattern->block_quads.insert(pattern->block_quads.end()
									, existing->block_quads.begin() + iMarkerCodeLength
									, existing->block_quads.end());

								pattern->pattern_widths.insert(pattern->pattern_widths.end(),
									existing->pattern_widths.begin(), existing->pattern_widths.end());
								existing->min_max_wh_sqr.Merge(pattern->min_max_wh_sqr);
								if (iPattern < (int)foundPattern.size() - 1)
								{
									foundPattern[iPattern] = foundPattern.back();
									foundPattern.pop_back();
								}
								else
								{
									foundPattern.erase(foundPattern.begin() + iPattern);
								}
							}
						}
					}
					iPattern++;
				}
			}

			foundPattern.push_back(pattern);
		}

#else  USE_LIBCBDETECTOR

		void addPattern__detect(std::vector<SinglePattern*>& foundPattern, SinglePattern* pattern)
		{
			if (pattern->pattern_type == (int)MarkerType_CYLINDER)
			{
				int iNumPatterns = (int)m_cylinderarkerInfos__fixed.size();
				int iMarkerCodeLength = m_parameters__fixed.markerCodeLength;
				int iMajorCodeLength = (iMarkerCodeLength + 1) >> 1;
				int iMinorCodeLength = iMarkerCodeLength - iMajorCodeLength;

				int iPattern = 0;

				while (iPattern < foundPattern.size())
				{
					SinglePattern* existing = foundPattern[iPattern];

					if (existing->pattern_type == (int)MarkerType_CYLINDER)
					{
						if ((existing->pattern_index + existing->pattern_count) % iNumPatterns == pattern->pattern_index)
						{
							int iBaseCodeLength = (m_cylinderarkerInfos__fixed[pattern->pattern_index].m_bMajor) ? iMajorCodeLength : iMinorCodeLength;
							bool bDiff = false;
							int iExistingQuadLength = (int)existing->pattern_quads.size();
							for (int c = 0; c < iBaseCodeLength; c++)
							{
								if (existing->pattern_quads[iExistingQuadLength - iBaseCodeLength + c] != pattern->pattern_quads[c])
								{
									bDiff = true;
									break;
								}
							}
							if (!bDiff)
							{
								existing->pattern_count += pattern->pattern_count;
								existing->pattern_quads.insert(existing->pattern_quads.end()
									, pattern->pattern_quads.begin() + iBaseCodeLength, pattern->pattern_quads.end());
								existing->pattern_widths.insert(existing->pattern_widths.end(), pattern->pattern_widths.begin(), pattern->pattern_widths.end());
								existing->min_max_wh_sqr.Merge(pattern->min_max_wh_sqr);

								if (iPattern < (int)foundPattern.size() - 1)
								{
									foundPattern[iPattern] = foundPattern.back();
									foundPattern.pop_back();
								}
								else
								{
									foundPattern.erase(foundPattern.begin() + iPattern);
								}
								pattern = existing;
								continue;
							}
						}
						else if ((pattern->pattern_index + pattern->pattern_count) % iNumPatterns == existing->pattern_index)
						{
							int iBaseCodeLength = (m_cylinderarkerInfos__fixed[existing->pattern_index].m_bMajor) ? iMajorCodeLength : iMinorCodeLength;
							bool bDiff = false;
							int iPatternQuadLength = (int)pattern->pattern_quads.size();
							for (int c = 0; c < iBaseCodeLength; c++)
							{
								if (pattern->pattern_quads[iPatternQuadLength - iBaseCodeLength + c] != existing->pattern_quads[c])
								{
									bDiff = true;
									break;
								}
							}
							if (!bDiff)
							{
								pattern->pattern_count += existing->pattern_count;
								pattern->pattern_quads.insert(pattern->pattern_quads.end()
									, existing->pattern_quads.begin() + iBaseCodeLength
									, existing->pattern_quads.end());

								pattern->pattern_widths.insert(pattern->pattern_widths.end(),
									existing->pattern_widths.begin(), existing->pattern_widths.end());
								existing->min_max_wh_sqr.Merge(pattern->min_max_wh_sqr);
								if (iPattern < (int)foundPattern.size() - 1)
								{
									foundPattern[iPattern] = foundPattern.back();
									foundPattern.pop_back();
								}
								else
								{
									foundPattern.erase(foundPattern.begin() + iPattern);
								}
							}
						}
					}
					iPattern++;
				}
			}

			foundPattern.push_back(pattern);
		}

		bool checkLineInner(const Point2& p0, const Point2& p1, const Point2& p)
		{
			float x = p1.x - p0.x;
			float y = p1.y - p0.y;
			return ((-y) * (p.x - p0.x) + x * (p.y - p0.y)) > 0.0f;
		}

		bool checkPatternStraight__detect(SinglePattern* pattern, const std::vector<MarkerInfo>& markerInfos, int iMarkerCodeLength, bool bCyclic)
		{
			if (markerInfos.empty() == true)
				return false;

			if (pattern->pattern_index < 0 || pattern->pattern_index >= (int)markerInfos.size())
				return false;

			int iNumBlocks = EstimateCheckerBoardNumBlocks(iMarkerCodeLength, pattern->pattern_count + 1, markerInfos[pattern->pattern_index].m_bMajor);
			if ((int)pattern->pattern_quads.size() != iNumBlocks)
				return false;


			if (bCyclic == false && pattern->pattern_index + pattern->pattern_count - 1 >= (int)markerInfos.size())
				return false;

			int iMajorCodeLength = (iMarkerCodeLength + 1) >> 1;
			int iMinorCodeLength = iMarkerCodeLength - iMajorCodeLength;

			Point2 p0;
			Point2 p1;
			Point2 p2;
			Point2 p3;
			Point2 p;

			int iOffset = (int)pattern->direction;
			int iOffset_1 = (iOffset + 1) % 4;
			int iOffset_2 = (iOffset + 2) % 4;
			int iOffset_3 = (iOffset + 3) % 4;

			if (iMajorCodeLength == iMinorCodeLength)
			{
				int stride = 0;
				int quad_offset = 0;
				for (int i = 0; i < pattern->pattern_count; i++, quad_offset += stride)
				{
					int pattern_index = (pattern->pattern_index + i) % markerInfos.size();

					int iBaseCodeLength = iMajorCodeLength;
					CvCBQuad* iLineStart = nullptr;
					CvCBQuad* iLineEnd = nullptr;

					if (markerInfos[pattern_index].m_bMajor)
					{
						iLineStart = pattern->pattern_quads[quad_offset];
						iLineEnd = pattern->pattern_quads[quad_offset + iMarkerCodeLength - 1];
						stride = iMajorCodeLength;
					}
					else
					{
						iLineStart = pattern->pattern_quads[quad_offset + iMinorCodeLength];
						iLineEnd = pattern->pattern_quads[quad_offset + iMinorCodeLength - 1];
						stride = iMinorCodeLength;
					}

					p0 = iLineStart->corners[iOffset]->pt;
					p1 = iLineStart->corners[iOffset_1]->pt;
					p2 = iLineEnd->corners[iOffset_2]->pt;
					p3 = iLineEnd->corners[iOffset_3]->pt;

					bool bFailed = false;
					for (int c = 0; c < iMarkerCodeLength; c++)
					{
						CvCBQuad* iQuadIndex = pattern->pattern_quads[quad_offset + c];
						if (c < iBaseCodeLength)
						{
							if (iQuadIndex != iLineStart)
							{
								p = iQuadIndex->corners[iOffset_1]->pt;
								if (!checkLineInner(p0, p1, p) || !checkLineInner(p2, p3, p))
								{
									bFailed = true;
									break;
								}
							}
							if (iQuadIndex != iLineEnd)
							{
								p = iQuadIndex->corners[iOffset_2]->pt;
								if (!checkLineInner(p0, p1, p) || !checkLineInner(p2, p3, p))
								{
									bFailed = true;
									break;
								}
							}
						}
						else
						{
							if (iQuadIndex != iLineEnd)
							{
								p = iQuadIndex->corners[iOffset_3]->pt;
								if (!checkLineInner(p0, p1, p) || !checkLineInner(p2, p3, p))
								{
									bFailed = true;
									break;
								}
							}
							if (iQuadIndex != iLineStart)
							{
								p = iQuadIndex->corners[iOffset]->pt;
								if (!checkLineInner(p0, p1, p) || !checkLineInner(p2, p3, p))
								{
									bFailed = true;
									break;
								}
							}
						}
					}
					if (bFailed)
						return false;
				}
			}
			else
			{
				int stride = 0;
				int quad_offset = 0;

				for (int i = 0; i < pattern->pattern_count; i++, quad_offset += stride)
				{
					int pattern_index = (pattern->pattern_index + i) % markerInfos.size();

					int iBaseCodeLength;
					CvCBQuad* iLineStart = nullptr;
					CvCBQuad* iLinePreEnd = nullptr;
					CvCBQuad* iLineEnd = nullptr;
					if (markerInfos[pattern_index].m_bMajor)
					{
						iBaseCodeLength = iMajorCodeLength;
						iLineStart = pattern->pattern_quads[quad_offset];
						iLinePreEnd = pattern->pattern_quads[quad_offset + iMarkerCodeLength - 1];
						iLineEnd = pattern->pattern_quads[quad_offset + iBaseCodeLength - 1];
						stride = iMajorCodeLength;

					}
					else
					{
						iBaseCodeLength = iMinorCodeLength;
						iLineStart = pattern->pattern_quads[quad_offset + iBaseCodeLength];
						iLinePreEnd = pattern->pattern_quads[quad_offset + iMarkerCodeLength - 1];
						iLineEnd = pattern->pattern_quads[quad_offset + iBaseCodeLength - 1];
						stride = iMinorCodeLength;

					}

					p0 = iLineStart->corners[iOffset]->pt;
					p1 = iLineStart->corners[iOffset_1]->pt;
					p2 = iLinePreEnd->corners[iOffset_2]->pt;
					p3 = iLineEnd->corners[iOffset_3]->pt;

					bool bFailed = false;
					for (int c = 0; c < iMarkerCodeLength; c++)
					{
						CvCBQuad* iQuadIndex = pattern->pattern_quads[quad_offset + c];
						if (c < iBaseCodeLength)
						{
							if (iQuadIndex != iLineStart)
							{
								p = iQuadIndex->corners[iOffset_1]->pt;
								if (!checkLineInner(p0, p1, p) || !checkLineInner(p2, p3, p))
								{
									bFailed = true;
									break;
								}
							}
							if (iQuadIndex != iLinePreEnd)
							{
								p = iQuadIndex->corners[iOffset_2]->pt;
								if (!checkLineInner(p0, p1, p) || !checkLineInner(p2, p3, p))
								{
									bFailed = true;
									break;
								}
							}
						}
						else
						{
							if (iQuadIndex != iLineEnd)
							{
								p = iQuadIndex->corners[iOffset_3]->pt;
								if (!checkLineInner(p0, p1, p) || !checkLineInner(p2, p3, p))
								{
									bFailed = true;
									break;
								}
							}
							if (iQuadIndex != iLineStart)
							{
								p = iQuadIndex->corners[iOffset]->pt;
								if (!checkLineInner(p0, p1, p) || !checkLineInner(p2, p3, p))
								{
									bFailed = true;
									break;
								}
							}
						}
					}
					if (bFailed)
						return false;
				}
			}

			return true;
		}

#endif USE_LIBCBDETECTOR


#ifdef USE_LIBCBDETECTOR


		__forceinline void fillCvCBQuad2(std::vector<CvCBQuad*>& vecDest, const std::vector<CvCBQuad*>& vecSrc, bool bReverse)
		{

			if (bReverse == false)
			{
				vecDest.assign(vecSrc.begin(), vecSrc.end());
			}
			else
			{
				vecDest.assign(vecSrc.rbegin(), vecSrc.rend());
			}
		}

		bool verifyPattern2__detect(WorkContext context_, CvCBQuad* quad_group, std::vector<BitInfo>& bitInfo, int iNumRows, int iNumColumns
			, bool bTopMajor
			, int row, int column, bool bColumnDir
			, std::vector<CvCBQuad*>& vecLabelQuadGroupData
			, std::vector<SinglePattern*>& patterns, std::vector<SinglePattern*>& reversePatterns
			, uint uMarkerFlag)
		{
			bool bMajor = (((row + column) & 1) == 0) ? bTopMajor : !bTopMajor;
			int iMarkerCodeLength = m_parameters__fixed.markerCodeLength;
			int bitIndexBase = row * iNumColumns + column;
			bool bReverseMajor = ((iMarkerCodeLength & 1) == 0) ? bMajor : !bMajor;

			bool bAdded = false;

			bool bIMajor = bMajor;
			uint code = 0;
			uint reverse_code = 0;
			
			int i = 0;
			int iLabelIndex = 0;
			float min_width_sqr = 0.f;
			float max_width_sqr = 0.f;
			float min_height_sqr = 0.f;
			float max_height_sqr = 0.f;


			vecLabelQuadGroupData.resize(iMarkerCodeLength * 2);

			for (int k = 0; k < 2; k++, bIMajor = !bIMajor)
			{
				int r, c;
				bool bJMajor = bIMajor;
				for (int j = 0; j < iMarkerCodeLength; j++, bJMajor = !bJMajor, iLabelIndex++)
				{
					if (bColumnDir == false)
					{
						c = k;
						r = j;
					}
					else
					{
						r = k;
						c = -j;
					}
					int bitIndex = bitIndexBase + r * iNumColumns + c;

					CvCBQuad* quad = &quad_group[bitIndex];
					if (bJMajor == true)
					{
						if (false == (quad->checker_type == cbdetect::CheckerBlack || quad->checker_type == cbdetect::CheckerBlackWhiteBit))
							return false;

						code |= (quad->checker_type == cbdetect::CheckerBlackWhiteBit) ? (1U << i) : 0;
						reverse_code |= (quad->checker_type == cbdetect::CheckerBlackWhiteBit) ? (1U << (iMarkerCodeLength - 1 - i)) : 0;

						i++;
					}
					else
					{
						if (quad_group[bitIndex].checker_type != cbdetect::CheckerWhite)
							return false;
					}
					vecLabelQuadGroupData[iLabelIndex] = quad;
					if (j == 0)
					{
						min_width_sqr = quad->min_width_sqr;
						max_width_sqr = quad->max_width_sqr;
						min_height_sqr = quad->min_height_sqr;
						max_height_sqr = quad->max_height_sqr;
					}
					else
					{
						min_width_sqr = std::min(min_width_sqr, quad->min_width_sqr);
						max_width_sqr = std::max(max_width_sqr, quad->max_width_sqr);
						min_height_sqr = std::min(min_height_sqr, quad->min_height_sqr);
						max_height_sqr = std::max(max_height_sqr, quad->max_height_sqr);
					}
				}
			}
			{
				{
					CodeClass codeClass = (bMajor) ? m_dictionaryMajor__fixed[code] : m_dictionaryMinor__fixed[code];
					if (codeClass.m_type != (int)MarkerType_INVALID && ((uint)(1 << codeClass.m_type) & uMarkerFlag) != 0 && codeClass.m_index >= 0)
					{
						SinglePattern* sPattern = allocSinglePattern__detect(codeClass.m_type);
						fillCvCBQuad2(sPattern->block_quads, vecLabelQuadGroupData, false);
						sPattern->pattern_type = codeClass.m_type;
						sPattern->pattern_index = codeClass.m_index;
						sPattern->pattern_count = 1;
						sPattern->direction = (bColumnDir) ? Direction_COLUMN : Direction_ROW;
						if (bColumnDir == false)
						{
							sPattern->min_max_wh_sqr.min_width_sqr = min_width_sqr;
							sPattern->min_max_wh_sqr.max_width_sqr = max_width_sqr;
							sPattern->min_max_wh_sqr.min_height_sqr = min_height_sqr;
							sPattern->min_max_wh_sqr.max_height_sqr = max_height_sqr;
						}
						else
						{
							sPattern->min_max_wh_sqr.min_width_sqr = min_height_sqr;
							sPattern->min_max_wh_sqr.max_width_sqr = max_height_sqr;
							sPattern->min_max_wh_sqr.min_height_sqr = min_width_sqr;
							sPattern->min_max_wh_sqr.max_height_sqr = max_width_sqr;
						}
						sPattern->pattern_widths.clear();
						if (codeClass.m_type == (int)MarkerType_CYLINDER)
						{
							sPattern->pattern_widths.push_back(sPattern->min_max_wh_sqr);
						}

						//if (checkPatternStraight__detect(sPattern, (codeClass.m_type == (int)MarkerType_CYLINDER)
						//	? m_cylinderarkerInfos__fixed
						//	: m_auxliaryMarkerInfos__fixed[codeClass.m_type - 1]
						//	, iMarkerCodeLength, codeClass.m_type == (int)MarkerType_CYLINDER))

						//{
						addPattern2__detect(patterns, sPattern);
						bAdded = true;
						//}
					}
				}
				{
					CodeClass codeClass = (bReverseMajor) ? m_dictionaryMajor__fixed[reverse_code] : m_dictionaryMinor__fixed[reverse_code];
					if (codeClass.m_type != (int)MarkerType_INVALID && ((uint)(1 << codeClass.m_type) & uMarkerFlag) != 0 && codeClass.m_index >= 0)
					{
						SinglePattern* sPattern = allocSinglePattern__detect(codeClass.m_type);
						fillCvCBQuad2(sPattern->block_quads, vecLabelQuadGroupData, true);
						sPattern->pattern_type = codeClass.m_type;
						sPattern->pattern_index = codeClass.m_index;
						sPattern->pattern_count = 1;
						sPattern->direction = (bColumnDir) ? Direction_REVERSE_COLUMN : Direction_REVERSE_ROW;
						if (bColumnDir == false)
						{
							sPattern->min_max_wh_sqr.min_width_sqr = min_width_sqr;
							sPattern->min_max_wh_sqr.max_width_sqr = max_width_sqr;
							sPattern->min_max_wh_sqr.min_height_sqr = min_height_sqr;
							sPattern->min_max_wh_sqr.max_height_sqr = max_height_sqr;
						}
						else
						{
							sPattern->min_max_wh_sqr.min_width_sqr = min_height_sqr;
							sPattern->min_max_wh_sqr.max_width_sqr = max_height_sqr;
							sPattern->min_max_wh_sqr.min_height_sqr = min_width_sqr;
							sPattern->min_max_wh_sqr.max_height_sqr = max_width_sqr;
						}
						sPattern->pattern_widths.clear();
						if (codeClass.m_type == (int)MarkerType_CYLINDER)
						{
							sPattern->pattern_widths.push_back(sPattern->min_max_wh_sqr);
						}

						//if (checkPatternStraight__detect(sPattern, (codeClass.m_type == (int)MarkerType_CYLINDER)
						//	? m_cylinderarkerInfos__fixed
						//	: m_auxliaryMarkerInfos__fixed[codeClass.m_type - 1]
						//	, iMarkerCodeLength, codeClass.m_type == (int)MarkerType_CYLINDER))
						//{
						addPattern2__detect(reversePatterns, sPattern);
						bAdded = true;
						//}
					}
				}
			}
			return bAdded;
		}

		bool verifyNoPatternMarker2__detect(int aux, int iXWidth, int iYHeight, bool bCodeMajor, const std::vector<MarkerInfo>& markerInfo, const std::vector<MarkerRuntime>& markerRuntime
			, CvCBQuad* quad_group, std::vector<BitInfo>& bitInfo
			, int iNumRows, int iNumColumns
			, bool bTopMajor
			, int row, int column, bool bColumnDir
			, std::vector<CvCBQuad*>& vecLabelQuadGroupData
			, std::vector<SinglePattern*>& patterns
			, uint uMarkerFlag)
		{
			bool bMajor = (((row + column) & 1) == 0) ? bTopMajor : !bTopMajor;
			bool bReverseMajor = (((iXWidth - 1 + iYHeight - 1) & 1) == 0) ? bMajor : !bMajor;
			if (bCodeMajor != bMajor && bCodeMajor != bReverseMajor)
				return false;

			vecLabelQuadGroupData.resize(iXWidth * iYHeight);
			int i = 0;
			int iLabelIndex = 0;
			bool bAdded = false;
			if (bColumnDir == false)
			{
				SqrMinMaxWH sqr_wh;
				for (int c = column; c < column + iYHeight; c++)
				{
					for (int r = row; r < row + iXWidth; r++, iLabelIndex++)
					{
						bool bCurMajor = (((r + c) & 1) == 0) ? bTopMajor : !bTopMajor;
						int index = r * iNumColumns + c;
						CvCBQuad* quad = &quad_group[index];
						if (bCurMajor == true)
						{
							if (false == (quad->checker_type == cbdetect::CheckerBlack
								|| quad->checker_type == cbdetect::CheckerBlackWhiteBit))
								return false;
							i++;
						}
						else
						{
							if (quad->checker_type != cbdetect::CheckerWhite)
								return false;
						}
						vecLabelQuadGroupData[iLabelIndex] = quad;

						if (iLabelIndex == 0)
						{
							sqr_wh.min_width_sqr = quad->min_width_sqr;
							sqr_wh.max_width_sqr = quad->max_width_sqr;
							sqr_wh.min_height_sqr = quad->min_height_sqr;
							sqr_wh.max_height_sqr = quad->max_height_sqr;
						}
						else
						{
							sqr_wh.min_width_sqr = std::min(sqr_wh.min_width_sqr, quad->min_width_sqr);
							sqr_wh.max_width_sqr = std::max(sqr_wh.max_width_sqr, quad->max_width_sqr);
							sqr_wh.min_height_sqr = std::min(sqr_wh.min_height_sqr, quad->min_height_sqr);
							sqr_wh.max_height_sqr = std::max(sqr_wh.max_height_sqr, quad->max_height_sqr);
						}

					}
				}
				if (bCodeMajor == bMajor)
				{
					SinglePattern* sPattern = allocSinglePattern__detect((int)MarkerType_AUX_BASE + aux);
					fillCvCBQuad2(sPattern->block_quads, vecLabelQuadGroupData, false);
					sPattern->pattern_type = (int)MarkerType_AUX_BASE + aux;
					sPattern->pattern_index = 1;
					sPattern->pattern_count = iYHeight - 1;
					sPattern->direction = Direction_ROW;
					sPattern->pattern_widths.clear();
					sPattern->min_max_wh_sqr = sqr_wh;

					//if (checkPatternStraight__detect(sPattern, markerInfo, iXWidth, false))
					//{
					addPattern2__detect(patterns, sPattern);
					bAdded = true;
					//}
				}
				if (bCodeMajor == bReverseMajor)
				{
					SinglePattern* sPattern = allocSinglePattern__detect((int)MarkerType_AUX_BASE + aux);
					fillCvCBQuad2(sPattern->block_quads, vecLabelQuadGroupData, true);
					sPattern->pattern_type = (int)MarkerType_AUX_BASE + aux;
					sPattern->pattern_index = 1;
					sPattern->pattern_count = iYHeight - 1;
					sPattern->direction = Direction_REVERSE_ROW;
					sPattern->pattern_widths.clear();
					sPattern->min_max_wh_sqr = sqr_wh;

					//if (checkPatternStraight__detect(sPattern, markerInfo, iXWidth, false))
					//{
					addPattern2__detect(patterns, sPattern);
					bAdded = true;
					//}
				}
			}
			else
			{
				SqrMinMaxWH sqr_wh;

				for (int r = row; r < row + iYHeight; r++)
				{
					for (int c = column; c > column - iXWidth; c--, iLabelIndex++)
					{
						bool bCurMajor = (((r + c) & 1) == 0) ? bTopMajor : !bTopMajor;
						int index = r * iNumColumns + c;
						CvCBQuad* quad = &quad_group[index];

						if (bCurMajor == true)
						{
							if (false == (quad->checker_type == cbdetect::CheckerBlack
								|| quad->checker_type == cbdetect::CheckerBlackWhiteBit))
								return false;
							i++;
						}
						else
						{
							if (quad->checker_type != cbdetect::CheckerWhite)
								return false;
						}
						vecLabelQuadGroupData[iLabelIndex] = quad;

						if (iLabelIndex == 0)
						{
							sqr_wh.min_width_sqr = quad->min_height_sqr;
							sqr_wh.max_width_sqr = quad->max_height_sqr;
							sqr_wh.min_height_sqr = quad->min_width_sqr;
							sqr_wh.max_height_sqr = quad->max_width_sqr;
						}
						else
						{
							sqr_wh.min_width_sqr = std::min(sqr_wh.min_width_sqr, quad->min_height_sqr);
							sqr_wh.max_width_sqr = std::max(sqr_wh.max_width_sqr, quad->max_height_sqr);
							sqr_wh.min_height_sqr = std::min(sqr_wh.min_height_sqr, quad->min_width_sqr);
							sqr_wh.max_height_sqr = std::max(sqr_wh.max_height_sqr, quad->max_width_sqr);
						}
					}
				}
				if (bCodeMajor == bMajor)
				{
					SinglePattern* sPattern = allocSinglePattern__detect((int)MarkerType_AUX_BASE + aux);
					fillCvCBQuad2(sPattern->block_quads, vecLabelQuadGroupData, false);
					sPattern->pattern_type = (int)MarkerType_AUX_BASE + aux;
					sPattern->pattern_index = 1;
					sPattern->pattern_count = iYHeight - 1;
					sPattern->direction = Direction_COLUMN;
					sPattern->pattern_widths.clear();
					sPattern->min_max_wh_sqr = sqr_wh;

					//if (checkPatternStraight__detect(sPattern, markerInfo, iXWidth, false))
					//{
					addPattern2__detect(patterns, sPattern);
					bAdded = true;
					//}
				}
				if (bCodeMajor == bReverseMajor)
				{
					SinglePattern* sPattern = allocSinglePattern__detect((int)MarkerType_AUX_BASE + aux);
					fillCvCBQuad2(sPattern->block_quads, vecLabelQuadGroupData, true);
					sPattern->pattern_type = (int)MarkerType_AUX_BASE + aux;
					sPattern->pattern_index = 1;
					sPattern->pattern_count = iYHeight - 1;
					sPattern->direction = Direction_REVERSE_COLUMN;
					sPattern->pattern_widths.clear();
					sPattern->min_max_wh_sqr = sqr_wh;

					//if (checkPatternStraight__detect(sPattern, markerInfo, iXWidth, false))
					//{
					addPattern2__detect(patterns, sPattern);
					bAdded = true;
					//}
				}
			}
			return bAdded;
		}

#else  USE_LIBCBDETECTOR


		__forceinline void fillCvCBQuad(std::vector<CvCBQuad*>& vec, std::vector< LabelQuadGroupData>& vecLabelQuadGroupData, bool bReverse)
		{
			vec.resize(vecLabelQuadGroupData.size());
			if (bReverse == false)
			{
				vec =
					for (int i = 0; i < (int)vec.size(); i++)
						vec[i] = vecLabelQuadGroupData[i].aQuadIndices;
			}
			else
			{
				for (int i = 0; i < (int)vec.size(); i++)
					vec[i] = vecLabelQuadGroupData[vecLabelQuadGroupData.size() - i - 1].aQuadIndices;
			}
		}

		bool verifyPattern__detect(WorkContext context_, std::vector<BitInfo>& bitInfo, int iNumRows, int iNumColumns, int min_row, int min_column
			, int row, int column, bool bColumnDir
			, std::vector<LabelQuadGroupData>& vecLabelQuadGroupData
			, std::vector<SinglePattern*>& patterns, std::vector<SinglePattern*>& reversePatterns
			, uint uMarkerFlag)
		{
			bool bMajor = (((row + column) & 1) == 0) ? true : false;

			int iMarkerCodeLength = m_parameters__fixed.markerCodeLength;

			int iMajorCodeLength = (iMarkerCodeLength + 1) >> 1;
			int iMinorCodeLength = iMarkerCodeLength - iMajorCodeLength;
			int bitIndexBase = (row - min_row) * iNumColumns + (column - min_column);
			int iBaseCodeLength = bMajor ? iMajorCodeLength : iMinorCodeLength;
			bool bReverseMajor = ((iMarkerCodeLength & 1) == 0) ? bMajor : !bMajor;


			int i;
			bool bAdded = false;

			for (i = 0; i < iMarkerCodeLength; i++)
			{
				int r, c;

				if (i < iBaseCodeLength)
				{
					if (bColumnDir == false)
					{
						c = 0;
						r = i * 2 + (bMajor ? 0 : 1);
					}
					else
					{
						r = 0;
						c = -(i * 2 + (bMajor ? 0 : 1));
					}
				}
				else
				{
					if (bColumnDir == false)
					{
						c = 1;
						r = (i - iBaseCodeLength) * 2 + (bMajor ? 1 : 0);
					}
					else
					{
						r = 1;
						c = -((i - iBaseCodeLength) * 2 + (bMajor ? 1 : 0));
					}
				}
				int bitIndex = bitIndexBase + r * iNumColumns + c;
				if (bitInfo[bitIndex].quads.empty() == true)
					return false;
				vecLabelQuadGroupData[i].aBitIndices = bitIndex;
				vecLabelQuadGroupData[i].aIndices = 0;
			}
			do
			{
				uint code = 0;
				uint reverse_code = 0;
				float min_width_sqr = 0.f;
				float max_width_sqr = 0.0f;
				float min_height_sqr = 0.f;
				float max_height_sqr = 0.0f;
				for (i = 0; i < iMarkerCodeLength; i++)
				{
					CvCBQuad* quad = (vecLabelQuadGroupData[i].aIndices < (int)bitInfo[vecLabelQuadGroupData[i].aBitIndices].quads.size())
						? bitInfo[vecLabelQuadGroupData[i].aBitIndices].quads[vecLabelQuadGroupData[i].aIndices]
						: nullptr;
					if (quad == nullptr)
						return false;
					//Assert.IsNotNull(quad);
					vecLabelQuadGroupData[i].aQuadIndices = quad;
					code |= (quad->bit) ? (1U << i) : 0;
					reverse_code |= (quad->bit) ? (1U << (iMarkerCodeLength - 1 - i)) : 0;
					if (i == 0)
					{
						min_width_sqr = quad->min_width_sqr;
						max_width_sqr = quad->max_width_sqr;
						min_height_sqr = quad->min_height_sqr;
						max_height_sqr = quad->max_height_sqr;
					}
					else
					{
						min_width_sqr = std::min(min_width_sqr,quad->min_width_sqr);
						max_width_sqr = std::max(max_width_sqr,quad->max_width_sqr);
						min_height_sqr = std::min(min_height_sqr,quad->min_height_sqr);
						max_height_sqr = std::max(max_height_sqr,quad->max_height_sqr);


					}
				}
				{
					CodeClass codeClass = (bMajor) ? m_dictionaryMajor__fixed[code] : m_dictionaryMinor__fixed[code];
					if (codeClass.m_type != (int)MarkerType_INVALID && ((uint)(1 << codeClass.m_type) & uMarkerFlag) != 0 && codeClass.m_index >= 0)
					{
						SinglePattern* sPattern = allocSinglePattern__detect(codeClass.m_type);
						fillCvCBQuad(sPattern->pattern_quads, vecLabelQuadGroupData, false);
						sPattern->pattern_type = codeClass.m_type;
						sPattern->pattern_index = codeClass.m_index;
						sPattern->pattern_count = 1;
						sPattern->direction = (bColumnDir) ? Direction_COLUMN : Direction_ROW;
						if (bColumnDir == false)
						{
							sPattern->min_max_wh_sqr.min_width_sqr = min_width_sqr;
							sPattern->min_max_wh_sqr.max_width_sqr = max_width_sqr;
							sPattern->min_max_wh_sqr.min_height_sqr = min_height_sqr;
							sPattern->min_max_wh_sqr.max_height_sqr = max_height_sqr;
						}
						else
						{
							sPattern->min_max_wh_sqr.min_width_sqr = min_height_sqr;
							sPattern->min_max_wh_sqr.max_width_sqr = max_height_sqr;
							sPattern->min_max_wh_sqr.min_height_sqr = min_width_sqr;
							sPattern->min_max_wh_sqr.max_height_sqr = max_width_sqr;
						}
						sPattern->pattern_widths.clear();
						if (codeClass.m_type == (int)MarkerType_CYLINDER)
						{
							sPattern->pattern_widths.push_back(sPattern->min_max_wh_sqr);
						}

						if (checkPatternStraight__detect(sPattern, (codeClass.m_type == (int)MarkerType_CYLINDER)
							? m_cylinderarkerInfos__fixed
							: m_auxliaryMarkerInfos__fixed[codeClass.m_type - 1]
							, iMarkerCodeLength, codeClass.m_type == (int)MarkerType_CYLINDER))

						{
							addPattern__detect(patterns, sPattern);
							bAdded = true;
						}
					}
				}
				{
					CodeClass codeClass = (bReverseMajor) ? m_dictionaryMajor__fixed[reverse_code] : m_dictionaryMinor__fixed[reverse_code];
					if (codeClass.m_type != (int)MarkerType_INVALID && ((uint)(1 << codeClass.m_type) & uMarkerFlag) != 0 && codeClass.m_index >= 0)
					{
						SinglePattern* sPattern = allocSinglePattern__detect(codeClass.m_type);
						fillCvCBQuad(sPattern->pattern_quads, vecLabelQuadGroupData, true);
						sPattern->pattern_type = codeClass.m_type;
						sPattern->pattern_index = codeClass.m_index;
						sPattern->pattern_count = 1;
						sPattern->direction = (bColumnDir) ? Direction_REVERSE_COLUMN : Direction_REVERSE_ROW;
						if (bColumnDir == false)
						{
							sPattern->min_max_wh_sqr.min_width_sqr = min_width_sqr;
							sPattern->min_max_wh_sqr.max_width_sqr = max_width_sqr;
							sPattern->min_max_wh_sqr.min_height_sqr = min_height_sqr;
							sPattern->min_max_wh_sqr.max_height_sqr = max_height_sqr;
						}
						else
						{
							sPattern->min_max_wh_sqr.min_width_sqr = min_height_sqr;
							sPattern->min_max_wh_sqr.max_width_sqr = max_height_sqr;
							sPattern->min_max_wh_sqr.min_height_sqr = min_width_sqr;
							sPattern->min_max_wh_sqr.max_height_sqr = max_width_sqr;
						}
						sPattern->pattern_widths.clear();
						if (codeClass.m_type == (int)MarkerType_CYLINDER)
						{
							sPattern->pattern_widths.push_back(sPattern->min_max_wh_sqr);
						}

						if (checkPatternStraight__detect(sPattern, (codeClass.m_type == (int)MarkerType_CYLINDER)
							? m_cylinderarkerInfos__fixed
							: m_auxliaryMarkerInfos__fixed[codeClass.m_type - 1]
							, iMarkerCodeLength, codeClass.m_type == (int)MarkerType_CYLINDER))
						{
							addPattern__detect(reversePatterns, sPattern);
							bAdded = true;
						}
					}
				}

				for (i = 0; i < iMarkerCodeLength; i++)
				{
					vecLabelQuadGroupData[i].aIndices++;
					if (vecLabelQuadGroupData[i].aIndices >= (int)bitInfo[vecLabelQuadGroupData[i].aBitIndices].quads.size())
					{
						vecLabelQuadGroupData[i].aIndices = 0;
					}
					else
					{
						break;
					}
				}
			} while (i < iMarkerCodeLength);
			return bAdded;
		}

		bool verifyNoPatternMarker__detect(int aux, int iXWidth, int iYHeight, bool bCodeMajor, const std::vector<MarkerInfo>& markerInfo, const std::vector<MarkerRuntime>& markerRuntime
			, std::vector<BitInfo>& bitInfo, std::vector< LabelQuadGroupData>& vecLabelQuadGroupData, int iNumRows, int iNumColumns, int min_row, int min_column
			, int row, int column, bool bColumnDir
			, std::vector<SinglePattern*>& patterns
			, uint uMarkerFlag)
		{
			bool bMajor = (((row + column) & 1) == 0) ? true : false;
			bool bReverseMajor = (((iXWidth - 1 + iYHeight - 1) & 1) == 0) ? bMajor : !bMajor;
			if (bCodeMajor != bMajor && bCodeMajor != bReverseMajor)
				return false;


			if (bColumnDir == false)
			{
				for (int c = column; c < column + iYHeight; c++)
				{
					for (int r = row; r < row + iXWidth; r++)
					{
						bool bCurMajor = (((r + c) & 1) == 0) ? true : false;
						if (bCurMajor == false)
							continue;
						int index = (r - min_row) * iNumColumns + (c - min_column);
						if (bitInfo[index].quads.empty() == true)
							return false;
					}
				}
			}
			else
			{
				for (int r = row; r < row + iYHeight; r++)
				{
					for (int c = column; c > column - iXWidth; c--)
					{
						bool bCurMajor = (((r + c) & 1) == 0) ? true : false;
						if (bCurMajor == false)
							continue;
						int index = (r - min_row) * iNumColumns + (c - min_column);
						if (bitInfo[index].quads.empty() == true)
							return false;
					}
				}
			}

			bool bAdded = false;
			int iNumBlacks = (int)vecLabelQuadGroupData.size();
			for (int i = 0; i < iNumBlacks; i++)
			{
				vecLabelQuadGroupData[i].aIndices = 0;
				vecLabelQuadGroupData[i].aQuadIndices = nullptr;
			}

			if (bColumnDir == false)
			{
				bool bContinue = true;
				do
				{
					SqrMinMaxWH sqr_wh;
					int i = 0;
					for (int c = column; c < column + iYHeight; c++)
					{
						for (int r = row; r < row + iXWidth; r++)
						{
							bool bCurMajor = (((r + c) & 1) == 0) ? true : false;
							if (bCurMajor == false)
								continue;
							int index = (r - min_row) * iNumColumns + (c - min_column);
							CvCBQuad* quad = (vecLabelQuadGroupData[i].aIndices < (int)bitInfo[index].quads.size())
								? bitInfo[index].quads[vecLabelQuadGroupData[i].aIndices] : nullptr;
							if (quad == nullptr)
								return false;
							vecLabelQuadGroupData[i].aQuadIndices = quad;

							if (i == 0)
							{
								sqr_wh.min_width_sqr = quad->min_width_sqr;
								sqr_wh.max_width_sqr = quad->max_width_sqr;
								sqr_wh.min_height_sqr = quad->min_height_sqr;
								sqr_wh.max_height_sqr = quad->max_height_sqr;
							}
							else
							{
								sqr_wh.min_width_sqr = std::min(sqr_wh.min_width_sqr, quad->min_width_sqr);
								sqr_wh.max_width_sqr = std::max(sqr_wh.max_width_sqr, quad->max_width_sqr);
								sqr_wh.min_height_sqr = std::min(sqr_wh.min_height_sqr, quad->min_height_sqr);
								sqr_wh.max_height_sqr = std::max(sqr_wh.max_height_sqr, quad->max_height_sqr);
							}
							i++;
						}
					}
					if (bCodeMajor == bMajor)
					{
						SinglePattern* sPattern = allocSinglePattern__detect((int)MarkerType_AUX_BASE + aux);
						fillCvCBQuad(sPattern->pattern_quads, vecLabelQuadGroupData, false);
						sPattern->pattern_type = (int)MarkerType_AUX_BASE + aux;
						sPattern->pattern_index = 1;
						sPattern->pattern_count = iYHeight - 1;
						sPattern->direction = Direction_ROW;
						sPattern->pattern_widths.clear();
						sPattern->min_max_wh_sqr = sqr_wh;

						if (checkPatternStraight__detect(sPattern, markerInfo, iXWidth, false))
						{
							addPattern__detect(patterns, sPattern);
							bAdded = true;
						}
					}
					if (bCodeMajor == bReverseMajor)
					{
						SinglePattern* sPattern = allocSinglePattern__detect((int)MarkerType_AUX_BASE + aux);
						fillCvCBQuad(sPattern->pattern_quads, vecLabelQuadGroupData, true);
						sPattern->pattern_type = (int)MarkerType_AUX_BASE + aux;
						sPattern->pattern_index = 1;
						sPattern->pattern_count = iYHeight - 1;
						sPattern->direction = Direction_REVERSE_ROW;
						sPattern->pattern_widths.clear();
						sPattern->min_max_wh_sqr = sqr_wh;

						if (checkPatternStraight__detect(sPattern, markerInfo, iXWidth, false))
						{
							addPattern__detect(patterns, sPattern);
							bAdded = true;
						}
					}

					i = 0;
					bContinue = false;
					for (int c = column; c < column + iYHeight; c++)
					{
						for (int r = row; r < row + iXWidth; r++)
						{
							bool bCurMajor = (((r + c) & 1) == 0) ? true : false;
							int index = (r - min_row) * iNumColumns + (c - min_column);
							if (bCurMajor == false)
								continue;
							vecLabelQuadGroupData[i].aIndices++;
							if (vecLabelQuadGroupData[i].aIndices >= (int)bitInfo[index].quads.size())
							{
								vecLabelQuadGroupData[i].aIndices = 0;
							}
							else
							{
								bContinue = true;
								goto out1;
							}
							i++;
						}
					}
				out1:;
				} while (bContinue);
			}
			else
			{
				bool bContinue = true;
				do
				{
					SqrMinMaxWH sqr_wh;
					int i = 0;
					for (int r = row; r < row + iYHeight; r++)
					{
						for (int c = column; c > column - iXWidth; c--)
						{
							bool bCurMajor = (((r + c) & 1) == 0) ? true : false;
							if (bCurMajor == false)
								continue;
							int index = (r - min_row) * iNumColumns + (c - min_column);
							CvCBQuad* quad = (vecLabelQuadGroupData[i].aIndices < (int)bitInfo[index].quads.size())
								? bitInfo[index].quads[vecLabelQuadGroupData[i].aIndices] : nullptr;
							if (quad == nullptr)
								return false;
							vecLabelQuadGroupData[i].aQuadIndices = quad;
							if (i == 0)
							{
								sqr_wh.min_width_sqr = quad->min_height_sqr;
								sqr_wh.max_width_sqr = quad->max_height_sqr;
								sqr_wh.min_height_sqr = quad->min_width_sqr;
								sqr_wh.max_height_sqr = quad->max_width_sqr;
							}
							else
							{
								sqr_wh.min_width_sqr = std::min(sqr_wh.min_width_sqr, quad->min_height_sqr);
								sqr_wh.max_width_sqr = std::max(sqr_wh.max_width_sqr, quad->max_height_sqr);
								sqr_wh.min_height_sqr = std::min(sqr_wh.min_height_sqr, quad->min_width_sqr);
								sqr_wh.max_height_sqr = std::max(sqr_wh.max_height_sqr, quad->max_width_sqr);
							}
							i++;
						}
					}
					if (bCodeMajor == bMajor)
					{
						SinglePattern* sPattern = allocSinglePattern__detect((int)MarkerType_AUX_BASE + aux);
						fillCvCBQuad(sPattern->pattern_quads, vecLabelQuadGroupData, false);
						sPattern->pattern_type = (int)MarkerType_AUX_BASE + aux;
						sPattern->pattern_index = 1;
						sPattern->pattern_count = iYHeight - 1;
						sPattern->direction = Direction_COLUMN;
						sPattern->pattern_widths.clear();
						sPattern->min_max_wh_sqr = sqr_wh;

						if (checkPatternStraight__detect(sPattern, markerInfo, iXWidth, false))
						{
							addPattern__detect(patterns, sPattern);
							bAdded = true;
						}
					}
					if (bCodeMajor == bReverseMajor)
					{
						SinglePattern* sPattern = allocSinglePattern__detect((int)MarkerType_AUX_BASE + aux);
						fillCvCBQuad(sPattern->pattern_quads, vecLabelQuadGroupData, true);
						sPattern->pattern_type = (int)MarkerType_AUX_BASE + aux;
						sPattern->pattern_index = 1;
						sPattern->pattern_count = iYHeight - 1;
						sPattern->direction = Direction_REVERSE_COLUMN;
						sPattern->pattern_widths.clear();
						sPattern->min_max_wh_sqr = sqr_wh;

						if (checkPatternStraight__detect(sPattern, markerInfo, iXWidth, false))
						{
							addPattern__detect(patterns, sPattern);
							bAdded = true;
						}
					}

					i = 0;
					bContinue = false;
					for (int r = row; r < row + iYHeight; r++)
					{
						for (int c = column; c > column - iXWidth; c--)
						{
							bool bCurMajor = (((r + c) & 1) == 0) ? true : false;
							int index = (r - min_row) * iNumColumns + (c - min_column);
							if (bCurMajor == false)
								continue;
							vecLabelQuadGroupData[i].aIndices++;
							if (vecLabelQuadGroupData[i].aIndices >= (int)bitInfo[index].quads.size())
							{
								vecLabelQuadGroupData[i].aIndices = 0;
							}
							else
							{
								bContinue = true;
								goto out2;
							}
							i++;
						}
					}
				out2:;
				} while (bContinue);
			}
			return bAdded;
		}

#endif USE_LIBCBDETECTOR

#ifdef USE_LIBCBDETECTOR

		void labelQuadGroup2__detect(WorkContext& context_, CvCBQuad* quad_group, int rows, int cols, cbdetect::BoardType eBoardType, std::vector<SinglePattern*>& foundPattern, uint uMarkerFlag)
		{
			if (rows <= 0 || cols <= 0 || eBoardType == cbdetect::BoardTypeInvalid || quad_group == nullptr )
				return;

			// CYLINDER, CHECKER 인식
			uint uCylinderCheckerFlag = (uMarkerFlag & ((uint)MarkerTypeFlag_CYLINDER | m_uAuxPatternMarkerMask__fixed));
			// NOPATTERN 인식
			uint uNonUniqueMarkerFlag = (uMarkerFlag & m_uAuxNoPatternMarkerMask__fixed);

			if (uCylinderCheckerFlag == 0 && uNonUniqueMarkerFlag == 0)
				return;

			int iNumBlocks = rows * cols;

			bool bCheck = false;

			bCheck = rows >= m_iCheckerBoardMaxDimension__fixed && cols >= m_iCheckerBoardMinDimension__fixed
				|| cols >= m_iCheckerBoardMaxDimension__fixed && rows >= m_iCheckerBoardMinDimension__fixed;

			if (bCheck == false)
			{
				return;
			}

			std::vector<BitInfo>& bitArray = m_vecBitInfoBase__detect;
			bitArray.resize(0);
			bitArray.resize(iNumBlocks);

			bool bTopMajor = eBoardType == cbdetect::BoardTypeBlack;

			for (int col = 0; col < cols; col++)
			{
				for (int row = 0; row < rows; row++)
				{
					int index = row * cols + col;
					int index_pc = (col > 0) ? row * cols + (col - 1) : -1;
					int index_pr = (row > 0) ? (row - 1) * cols + col : -1;
					cbdetect::CheckerType type = quad_group[index].checker_type;
					bool bMajor = (((row + col) & 1) == 0) ? bTopMajor : !bTopMajor;
					bool bValid = false;
					bool bNopValid = false;

					if (bMajor == true)
					{
						bValid = type == cbdetect::CheckerBlack || type == cbdetect::CheckerBlackWhiteBit;
						bNopValid = type == cbdetect::CheckerBlack;
					}
					else
					{
						bValid = type == cbdetect::CheckerWhite;
						bNopValid = type == cbdetect::CheckerWhite;
					}
					if (uCylinderCheckerFlag != 0)
					{
						if (bValid == true)
						{
							if ((index_pc < 0 || bitArray[index_pc].colArray == 0)
								&& (index_pr < 0 || bitArray[index_pr].rowArray == 0))
							{
								bitArray[index].rowArray = 1;
								bitArray[index].colArray = 1;
							}
							else if (index_pc < 0 || bitArray[index_pc].colArray == 0 && index_pr >= 0)
							{
								bitArray[index].rowArray = bitArray[index_pr].rowArray + 1;
								bitArray[index].colArray = 1;
							}
							else if (index_pr < 0 || bitArray[index_pr].rowArray == 0 && index_pc >= 0)
							{
								bitArray[index].rowArray = 1;
								bitArray[index].colArray = bitArray[index_pc].colArray + 1;
							}
							else
							{
								bitArray[index].rowArray = std::max(bitArray[index_pr].rowArray + 1, bitArray[index_pc].rowArray);
								bitArray[index].colArray = std::max(bitArray[index_pr].colArray, bitArray[index_pc].colArray + 1);
							}
						}
						else
						{
							bitArray[index].rowArray = 0;
							bitArray[index].colArray = 0;
						}
					}
					if (uNonUniqueMarkerFlag != 0)
					{
						if (bNopValid == true)
						{
							if ((index_pc < 0 || bitArray[index_pc].colNopArray == 0)
								&& (index_pr < 0 || bitArray[index_pr].rowNopArray == 0))
							{
								bitArray[index].rowNopArray = 1;
								bitArray[index].colNopArray = 1;
							}
							else if (index_pc < 0 || bitArray[index_pc].colNopArray == 0 && index_pr >= 0)
							{
								bitArray[index].rowNopArray = bitArray[index_pr].rowNopArray + 1;
								bitArray[index].colNopArray = 1;
							}
							else if (index_pr < 0 || bitArray[index_pr].rowNopArray == 0 && index_pc >= 0)
							{
								bitArray[index].rowNopArray = 1;
								bitArray[index].colNopArray = bitArray[index_pc].colNopArray + 1;
							}
							else
							{
								bitArray[index].rowNopArray = std::max(bitArray[index_pr].rowNopArray + 1, bitArray[index_pc].rowNopArray);
								bitArray[index].colNopArray = std::max(bitArray[index_pr].colNopArray, bitArray[index_pc].colNopArray + 1);
							}
						}
						else
						{
							bitArray[index].rowNopArray = 0;
							bitArray[index].colNopArray = 0;
						}
					}
				}
			}

			// CYLINDER, CHECKER 인식
			if (uCylinderCheckerFlag != 0)
			{
				int iMarkerCodeLength = m_parameters__fixed.markerCodeLength;
				
				std::vector<CvCBQuad*>& vecQuadTemp = m_findConnectedQuad__detect;
				std::vector<SinglePattern*>& rowPatterns = m_rowPatternsRuntime__detect;
				std::vector<SinglePattern*>& reverseRowPatterns = m_reverseRowPatternsRuntime__detect;
				std::vector<SinglePattern*>& columnPatterns = m_columnPatternsRuntime__detect;
				std::vector<SinglePattern*>& reverseColumnPatterns = m_reverseColumnPatternsRuntime__detect;
				vecQuadTemp.clear();
				rowPatterns.clear();
				reverseRowPatterns.clear();
				columnPatterns.clear();
				reverseColumnPatterns.clear();

				for (int col = 0; col < cols; col++)
				{
					for (int row = 0; row < rows; row++)
					{
						int index = row * cols + col;

						if (bitArray[index].rowArray >= iMarkerCodeLength && bitArray[index].colArray >= 2)
						{
							verifyPattern2__detect(context_, quad_group, bitArray, rows, cols, bTopMajor
								, row - iMarkerCodeLength + 1, col - 1, false, vecQuadTemp, rowPatterns, reverseRowPatterns, uCylinderCheckerFlag);
						}

						if (bitArray[index].rowArray >= 2 && bitArray[index].colArray >= iMarkerCodeLength)
						{
							verifyPattern2__detect(context_, quad_group, bitArray, rows, cols, bTopMajor
								, row - 1, col, true, vecQuadTemp, columnPatterns, reverseColumnPatterns, uCylinderCheckerFlag);
						}
					}
				}

				foundPattern.insert(foundPattern.end(), rowPatterns.begin(), rowPatterns.end());
				foundPattern.insert(foundPattern.end(), reverseRowPatterns.begin(), reverseRowPatterns.end());
				foundPattern.insert(foundPattern.end(), columnPatterns.begin(), columnPatterns.end());
				foundPattern.insert(foundPattern.end(), reverseColumnPatterns.begin(), reverseColumnPatterns.end());

				rowPatterns.clear();
				reverseRowPatterns.clear();
				columnPatterns.clear();
				reverseColumnPatterns.clear();
			}


			// NOPATTERN 인식
			if (uNonUniqueMarkerFlag != 0)
			{
				std::vector<CvCBQuad*>& vecQuadTemp = m_findConnectedQuad__detect;
				std::vector<SinglePattern*>& rowPatterns = m_rowPatternsRuntime__detect;
				vecQuadTemp.clear();
				rowPatterns.clear();

				int auxnum = (int)m_parameters__fixed.auxiliaryMarkerSettings.size();

				for (int aux = 0; aux < auxnum; aux++)
				{
					uint flag = (uint)(1 << ((int)MarkerType_AUX_BASE + aux));
					if ((flag & uNonUniqueMarkerFlag) == 0)
						continue;

					int XWidth = m_parameters__fixed.auxiliaryMarkerSettings[aux].checkerboardXLength;
					int YHeight = (int)m_parameters__fixed.auxiliaryMarkerSettings[aux].checkerboardCoordinates.size() - 1;
					bool bMajor = m_parameters__fixed.auxiliaryMarkerSettings[aux].codeMajor;


					for (int col = 0; col < cols; col++)
					{
						for (int row = 0; row < rows; row++)
						{
							int index = row * cols + col;

							if (bitArray[index].rowNopArray >= XWidth && bitArray[index].colNopArray >= YHeight)
							{
								verifyNoPatternMarker2__detect(aux, XWidth, YHeight, bMajor, m_auxliaryMarkerInfos__fixed[aux], context_.m_auxliaryMarkerRuntimes[aux]
									, quad_group, bitArray, rows, cols, bTopMajor
									, row - XWidth + 1, col - YHeight + 1, false, vecQuadTemp, rowPatterns, flag);
							}
							if (bitArray[index].colNopArray >= XWidth && bitArray[index].rowNopArray >= YHeight)
							{
								verifyNoPatternMarker2__detect(aux, XWidth, YHeight, bMajor, m_auxliaryMarkerInfos__fixed[aux], context_.m_auxliaryMarkerRuntimes[aux]
									, quad_group, bitArray, rows, cols, bTopMajor
									, row - YHeight + 1, col, true, vecQuadTemp, rowPatterns, flag);
							}
						}
					}
				}
				foundPattern.insert(foundPattern.end(), rowPatterns.begin(), rowPatterns.end());
				rowPatterns.clear();
			}
		}

#else  USE_LIBCBDETECTOR

		void labelQuadGroup__detect(WorkContext& context_, std::vector<CvCBQuad*>& quad_group, std::vector<SinglePattern*>& foundPattern, uint uMarkerFlag)
		{
			int count = (int)quad_group.size();
			// If this is the first function call, a seed quad needs to be selected
			if (count <= 0)
				return;

			// CYLINDER, CHECKER 인식
			uint uCylinderCheckerFlag = (uMarkerFlag & ((uint)MarkerTypeFlag_CYLINDER | m_uAuxPatternMarkerMask__fixed));
			// NOPATTERN 인식
			uint uNonUniqueMarkerFlag = (uMarkerFlag & m_uAuxNoPatternMarkerMask__fixed);

			if (uCylinderCheckerFlag == 0 && uNonUniqueMarkerFlag == 0)
				return;

			// Search for the (first) quad with the maximum number of neighbors
			// (usually 4). This will be our starting point.
			int max_id = -1;
			int max_number = -1;
			int max_neighbor_count = -1;
			for (int i = 0; i < count; i++)
			{
				CvCBQuad* q = quad_group[i];
				int neighbor_count = 0;
				if (q->count > max_number || q->count == max_number)
				{
					for (int j = 0; j < 4; j++)
					{
						if (q->neighbors[j] != nullptr)
							neighbor_count += q->neighbors[j]->count;
					}
				}
				if (q->count > max_number || (q->count == max_number && neighbor_count > max_neighbor_count))
				{
					max_number = q->count;
					max_id = i;
					max_neighbor_count = neighbor_count;
				}
			}

			// Mark the starting quad's (per definition) upper left corner with
			//(0,0) and then proceed clockwise
			// The following labeling sequence enshures a "right coordinate system"
			(quad_group[max_id])->labeled = true;

			(quad_group[max_id])->corners[0]->row = 0;
			(quad_group[max_id])->corners[0]->column = 0;
			(quad_group[max_id])->corners[1]->row = 0;
			(quad_group[max_id])->corners[1]->column = 1;
			(quad_group[max_id])->corners[2]->row = 1;
			(quad_group[max_id])->corners[2]->column = 1;
			(quad_group[max_id])->corners[3]->row = 1;
			(quad_group[max_id])->corners[3]->column = 0;


			// Mark all other corners with their respective row and column
			bool flag_changed = true;
			while (flag_changed == true)
			{
				// First reset the flag to "false"
				flag_changed = false;

				// Go through all quads top down is faster, since unlabeled quads will
				// be inserted at the end of the list
				for (int i = (count - 1); i >= 0; i--)
				{
					CvCBQuad* q_i = quad_group[i];

					// Check whether quad "i" has been labeled already
					if (q_i->labeled == false)
					{
						// Check its neighbors, whether some of them have been labeled
						// already
						for (int j = 0; j < 4; j++)
						{
							// Check whether the neighbor exists (i.e. is not the NULL
							// pointer)
							if (q_i->neighbors[j] != nullptr)
							{
								CvCBQuad* quadNeighborJ = q_i->neighbors[j];


								// Only proceed, if neighbor "j" was labeled
								if (quadNeighborJ->labeled == true)
								{
									// For every quad it could happen to pass here 
									// multiple times. We therefore "break" later.
									// Check whitch of the neighbors corners is 
									// connected to the current quad
									int connectedNeighborCornerId = -1;
									int k = 0;
									for (k = 0; k < 4; k++)
									{
										if (quadNeighborJ->neighbors[k] == q_i)
										{
											connectedNeighborCornerId = k;


											// there is only one, therefore
											break;
										}
									}
									if (k == 4)
									{
										//Debug.Log("labelQuadGroup : invalid");
										continue;
									}

									// For the following calculations we need the row 
									// and column of the connected neighbor corner and 
									// all other corners of the connected quad "j", 
									// clockwise (CW)
									CvCBCorner* conCorner = quadNeighborJ->corners[connectedNeighborCornerId];
									CvCBCorner* conCornerCW1 = quadNeighborJ->corners[(connectedNeighborCornerId + 1) % 4];
									CvCBCorner* conCornerCW2 = quadNeighborJ->corners[(connectedNeighborCornerId + 2) % 4];
									CvCBCorner* conCornerCW3 = quadNeighborJ->corners[(connectedNeighborCornerId + 3) % 4];

									CvCBCorner* q_iCorner = q_i->corners[j];
									CvCBCorner* q_iCornerCW1 = q_i->corners[(j + 1) % 4];
									CvCBCorner* q_iCornerCW2 = q_i->corners[(j + 2) % 4];
									CvCBCorner* q_iCornerCW3 = q_i->corners[(j + 3) % 4];

									q_iCorner->row = conCorner->row;
									q_iCorner->column = conCorner->column;
									q_iCornerCW1->row = conCorner->row - conCornerCW2->row + conCornerCW3->row;
									q_iCornerCW1->column = conCorner->column - conCornerCW2->column + conCornerCW3->column;
									q_iCornerCW2->row = conCorner->row + conCorner->row - conCornerCW2->row;
									q_iCornerCW2->column = conCorner->column + conCorner->column - conCornerCW2->column;
									q_iCornerCW3->row = conCorner->row - conCornerCW2->row + conCornerCW1->row;
									q_iCornerCW3->column = conCorner->column - conCornerCW2->column + conCornerCW1->column;

									// Mark this quad as labeled
									q_i->labeled = true;


									// Changes have taken place, set the flag
									flag_changed = true;


									// once is enough!
									break;
								}
							}
						}
					}
				}
			}

			// 0번 코너의 row, column 이 가장 작도록 회전시킴
			{
				CvCBCorner* tmpCorners[4];
				CvCBQuad* tmpQuads[4];

				for (int i = 0; i < count; i++)
				{
					CvCBQuad* q = quad_group[i];
					int k = 0;
					int mink = 0;
					int minrow = q->corners[0]->row;
					int mincolumn = q->corners[1]->column;
					for (k = 1; k < 4; k++)
					{
						if ((q->corners[k]->row < minrow && q->corners[k]->column <= mincolumn)
							|| (q->corners[k]->row <= minrow && q->corners[k]->column < mincolumn))
						{
							mink = k;
						}
					}
					if (mink != 0)
					{
						for (k = 0; k < 4; k++)
						{
							tmpCorners[k] = q->corners[(mink + k) % 4];
							tmpQuads[k] = q->neighbors[(mink + k) % 4];
						}
						for (k = 0; k < 4; k++)
						{
							q->corners[k] = tmpCorners[k]; tmpCorners[k] = nullptr;
							q->neighbors[k] = tmpQuads[k]; tmpQuads[k] = nullptr;
						}
						if (mink == 1 || mink == 3)
						{
							float tmp = q->max_width_sqr;
							q->max_width_sqr = q->max_height_sqr;
							q->max_height_sqr = tmp;
						}
					}
				}
			}

			// All corners are marked with row and column
			// Record the minimal and maximal row and column indices
			// It is unlikely that more than 8bit checkers are used per dimension, if there are
			// an error would have been thrown at the beginning of "cvFindChessboardCorners2"
			int min_row = quad_group[0]->corners[0]->row;
			int max_row = min_row;
			int min_column = quad_group[0]->corners[0]->column;
			int max_column = min_column;

			for (int i = 1; i < count; i++)
			{
				CvCBQuad* q = quad_group[i];
				CvCBCorner* corner = q->corners[0];

				min_row = std::min(min_row, corner->row);
				max_row = std::max(max_row, corner->row);
				min_column = std::min(min_column, corner->column);
				max_column = std::max(max_column, corner->column);
			}

			int iNumRows = max_row - min_row + 1;
			int iNumColumns = max_column - min_column + 1;
			int iNumBlocks = iNumRows * iNumColumns;

			bool bCheck = false;

			bCheck = iNumRows >= m_iCheckerBoardMaxDimension__fixed && iNumColumns >= m_iCheckerBoardMinDimension__fixed
				|| iNumColumns >= m_iCheckerBoardMaxDimension__fixed && iNumRows >= m_iCheckerBoardMinDimension__fixed;

			if (bCheck == false)
			{
				return;
			}


			std::vector<BitInfo>& bitArray = m_vecBitInfoBase__detect;
			{
				int iCurSize = (int)bitArray.size();
				if (iNumBlocks > iCurSize)
				{
					bitArray.resize(iNumBlocks);
					for (int i = 0; i < iCurSize; i++)
						bitArray[i].Reset();
				}
				else
				{
					for (int i = 0; i < iNumBlocks; i++)
						bitArray[i].Reset();
				}
			}


			for (int i = 0; i < count; i++)
			{
				CvCBQuad* q = quad_group[i];
				int row = q->corners[0]->row - min_row;
				int col = q->corners[0]->column - min_column;
				int index = row * iNumColumns + col;

				switch ((int)bitArray[index].quads.size())
				{
				case 0:
				{
					bitArray[index].quads.push_back(q);
					if (q->bit)
						bitArray[index].bitCount = 1;
					else
						bitArray[index].bitCount = 0;
				}
				break;

				case 1:
				{
					if (Contain(q->bound, bitArray[index].quads.front()->bound))
					{
						bitArray[index].quads.front() = q;
						if (q->bit)
							bitArray[index].bitCount = 1;
						else
							bitArray[index].bitCount = 0;
					}
					else if (Contain(bitArray[index].quads.front()->bound, q->bound))
					{
					}
					else
					{
						if (q->bit)
							bitArray[index].bitCount++;
						bitArray[index].quads.push_back(q);
					}
				}
				break;

				default:
				{
					int j = 0;
					while (j < (int)bitArray[index].quads.size())
					{
						CvCBQuad* q_existing = bitArray[index].quads[j];
						if (Contain(q->bound, q_existing->bound))
						{
							if (q_existing->bit)
								bitArray[index].bitCount--;
							if (j < (int)bitArray[index].quads.size() - 1)
							{
								bitArray[index].quads[j] = bitArray[index].quads.back();
							}
							bitArray[index].quads.pop_back();
							continue;
						}
						else if (Contain(q_existing->bound, q->bound))
						{
							break;
						}
						else
						{
							j++;
						}
					}
					if (j >= (int)bitArray[index].quads.size())
					{
						if (q->bit)
							bitArray[index].bitCount++;
						bitArray[index].quads.push_back(q);
					}
					if (bitArray[index].quads.empty())
					{
						bitArray[index].bitCount = 0;
					}
				}
				break;
				}
			}

			for (int col = min_column; col <= max_column; col++)
			{
				for (int row = min_row; row <= max_row; row++)
				{
					int index = (row - min_row) * iNumColumns + (col - min_column);
					int index_pc = (col > min_column) ? (row - min_row) * iNumColumns + (col - 1 - min_column) : -1;
					int index_pr = (row > min_row) ? (row - 1 - min_row) * iNumColumns + (col - min_column) : -1;
					bool bMajor = (((row + col) & 1) == 0) ? true : false;
					bool bValid = false;
					bool bNopValid = false;
					if (bMajor == true)
					{
						bValid = bitArray[index].quads.empty() == false;
						bNopValid = (int)bitArray[index].quads.size() > bitArray[index].bitCount;
					}
					else
					{
						bValid = true;
						bNopValid = true;
					}
					if (uCylinderCheckerFlag != 0)
					{
						if (bValid == true)
						{
							if ((index_pc < 0 || bitArray[index_pc].colArray == 0)
								&& (index_pr < 0 || bitArray[index_pr].rowArray == 0))
							{
								bitArray[index].rowArray = 1;
								bitArray[index].colArray = 1;
							}
							else if (index_pc < 0 || bitArray[index_pc].colArray == 0 && index_pr >= 0)
							{
								bitArray[index].rowArray = bitArray[index_pr].rowArray + 1;
								bitArray[index].colArray = 1;
							}
							else if (index_pr < 0 || bitArray[index_pr].rowArray == 0 && index_pc >= 0)
							{
								bitArray[index].rowArray = 1;
								bitArray[index].colArray = bitArray[index_pc].colArray + 1;
							}
							else
							{
								bitArray[index].rowArray = std::max(bitArray[index_pr].rowArray + 1, bitArray[index_pc].rowArray);
								bitArray[index].colArray = std::max(bitArray[index_pr].colArray, bitArray[index_pc].colArray + 1);
							}
						}
						else
						{
							bitArray[index].rowArray = 0;
							bitArray[index].colArray = 0;
						}
					}
					if (uNonUniqueMarkerFlag != 0)
					{
						if (bNopValid == true)
						{
							if ((index_pc < 0 || bitArray[index_pc].colNopArray == 0)
								&& (index_pr < 0 || bitArray[index_pr].rowNopArray == 0))
							{
								bitArray[index].rowNopArray = 1;
								bitArray[index].colNopArray = 1;
							}
							else if (index_pc < 0 || bitArray[index_pc].colNopArray == 0 && index_pr >= 0)
							{
								bitArray[index].rowNopArray = bitArray[index_pr].rowNopArray + 1;
								bitArray[index].colNopArray = 1;
							}
							else if (index_pr < 0 || bitArray[index_pr].rowNopArray == 0 && index_pc >= 0)
							{
								bitArray[index].rowNopArray = 1;
								bitArray[index].colNopArray = bitArray[index_pc].colNopArray + 1;
							}
							else
							{
								bitArray[index].rowNopArray = std::max(bitArray[index_pr].rowNopArray + 1, bitArray[index_pc].rowNopArray);
								bitArray[index].colNopArray = std::max(bitArray[index_pr].colNopArray, bitArray[index_pc].colNopArray + 1);
							}
						}
						else
						{
							bitArray[index].rowNopArray = 0;
							bitArray[index].colNopArray = 0;
						}
					}
				}
			}

			// CYLINDER, CHECKER 인식
			if (uCylinderCheckerFlag != 0)
			{
				int iMarkerCodeLength = m_parameters__fixed.markerCodeLength;

				std::vector<SinglePattern*>& rowPatterns = m_rowPatternsRuntime__detect;
				std::vector<SinglePattern*>& reverseRowPatterns = m_reverseRowPatternsRuntime__detect;
				std::vector<SinglePattern*>& columnPatterns = m_columnPatternsRuntime__detect;
				std::vector<SinglePattern*>& reverseColumnPatterns = m_reverseColumnPatternsRuntime__detect;
				rowPatterns.clear();
				reverseRowPatterns.clear();
				columnPatterns.clear();
				reverseColumnPatterns.clear();


				m_vecLabelQuadGroupData__detect.clear();
				m_vecLabelQuadGroupData__detect.resize(iMarkerCodeLength);

				for (int col = min_column; col <= max_column; col++)
				{
					for (int row = min_row; row <= max_row; row++)
					{
						int index = (row - min_row) * iNumColumns + (col - min_column);

						if (bitArray[index].rowArray >= iMarkerCodeLength && bitArray[index].colArray >= 2)
						{
							verifyPattern__detect(context_, bitArray, iNumRows, iNumColumns, min_row, min_column
								, row - iMarkerCodeLength + 1, col - 1, false, m_vecLabelQuadGroupData__detect, rowPatterns, reverseRowPatterns, uCylinderCheckerFlag);
						}

						if (bitArray[index].rowArray >= 2 && bitArray[index].colArray >= iMarkerCodeLength)
						{
							verifyPattern__detect(context_, bitArray, iNumRows, iNumColumns, min_row, min_column
								, row - 1, col, true, m_vecLabelQuadGroupData__detect, columnPatterns, reverseColumnPatterns, uCylinderCheckerFlag);
						}
					}
				}

				foundPattern.insert(foundPattern.end(), rowPatterns.begin(), rowPatterns.end());
				foundPattern.insert(foundPattern.end(), reverseRowPatterns.begin(), reverseRowPatterns.end());
				foundPattern.insert(foundPattern.end(), columnPatterns.begin(), columnPatterns.end());
				foundPattern.insert(foundPattern.end(), reverseColumnPatterns.begin(), reverseColumnPatterns.end());

				rowPatterns.clear();
				reverseRowPatterns.clear();
				columnPatterns.clear();
				reverseColumnPatterns.clear();
			}


			// NOPATTERN 인식
			if (uNonUniqueMarkerFlag != 0)
			{
				std::vector<SinglePattern*>& rowPatterns = m_rowPatternsRuntime__detect;
				rowPatterns.clear();

				int auxnum = (int)m_parameters__fixed.auxiliaryMarkerSettings.size();

				for (int aux = 0; aux < auxnum; aux++)
				{
					uint flag = (uint)(1 << ((int)MarkerType_AUX_BASE + aux));
					if ((flag & uNonUniqueMarkerFlag) == 0)
						continue;

					int XWidth = m_parameters__fixed.auxiliaryMarkerSettings[aux].checkerboardXLength;
					int YHeight = (int)m_parameters__fixed.auxiliaryMarkerSettings[aux].checkerboardCoordinates.size() - 1;
					bool bMajor = m_parameters__fixed.auxiliaryMarkerSettings[aux].codeMajor;

					int iNumBlacks = EstimateCheckerBoardNumBlocks(XWidth, YHeight, bMajor);

					m_vecLabelQuadGroupData__detect.clear();
					m_vecLabelQuadGroupData__detect.resize(iNumBlacks);

					for (int col = min_column; col <= max_column; col++)
					{
						for (int row = min_row; row <= max_row; row++)
						{
							int index = (row - min_row) * iNumColumns + (col - min_column);

							if (bitArray[index].rowNopArray >= XWidth && bitArray[index].colNopArray >= YHeight)
							{
								verifyNoPatternMarker__detect(aux, XWidth, YHeight, bMajor, m_auxliaryMarkerInfos__fixed[aux], context_.m_auxliaryMarkerRuntimes[aux]
									, bitArray, m_vecLabelQuadGroupData__detect, iNumRows, iNumColumns, min_row, min_column
									, row - XWidth + 1, col - YHeight + 1, false, rowPatterns, flag);
							}
							if (bitArray[index].colNopArray >= XWidth && bitArray[index].rowNopArray >= YHeight)
							{
								verifyNoPatternMarker__detect(aux, XWidth, YHeight, bMajor, m_auxliaryMarkerInfos__fixed[aux], context_.m_auxliaryMarkerRuntimes[aux]
									, bitArray, m_vecLabelQuadGroupData__detect, iNumRows, iNumColumns, min_row, min_column
									, row - YHeight + 1, col, true, rowPatterns, flag);
							}
						}
					}
				}
				foundPattern.insert(foundPattern.end(), rowPatterns.begin(), rowPatterns.end());
				rowPatterns.clear();
			}
		}

#endif USE_LIBCBDETECTOR

		void simplifyCylinderPattern(SinglePattern* pattern)
		{
			int iPatternCount = pattern->pattern_count;
			if (iPatternCount <= 1)
				return;
			//Assert.AreEqual(pattern->pattern_count, pattern->pattern_widths.size());

			int iChosenIndex = 0;
			if ((iPatternCount & 1) != 0)
			{
				iChosenIndex = iPatternCount / 2;
			}
			else
			{
				int iIndex1 = iPatternCount / 2;
				int iIndex0 = iPatternCount / 2 - 1;

				if (pattern->pattern_widths[iIndex0].min_width_sqr >= pattern->pattern_widths[iIndex1].min_width_sqr)
					iChosenIndex = iIndex0;
				else
					iChosenIndex = iIndex1;
			}

			int iPatternOldIndex = pattern->pattern_index;
			pattern->min_max_wh_sqr = pattern->pattern_widths[iChosenIndex];
			pattern->pattern_index = (iPatternOldIndex + iChosenIndex) % m_cylinderarkerInfos__fixed.size();
			pattern->pattern_count = 1;
			pattern->pattern_widths.erase(pattern->pattern_widths.begin() + 1, pattern->pattern_widths.end());
			pattern->pattern_widths[0] = pattern->min_max_wh_sqr;


#ifdef USE_LIBCBDETECTOR

			int iMarkerCodeLength = m_parameters__fixed.markerCodeLength;
			pattern->block_quads.resize((iChosenIndex + 2) * iMarkerCodeLength);
			if (iChosenIndex > 0)
			{
				pattern->block_quads.erase(pattern->block_quads.begin(), pattern->block_quads.begin() + iChosenIndex * iMarkerCodeLength);
			}
#else  USE_LIBCBDETECTOR


			int iMajorCodeLength = (iMarkerCodeLength + 1) >> 1;
			int iMinorCodeLength = iMarkerCodeLength - iMajorCodeLength;

			bool bOldMajor = m_cylinderarkerInfos__fixed[iPatternOldIndex].m_bMajor;
			int evens = (iChosenIndex + 1) / 2;
			int odds = iChosenIndex - evens;
			int iCvQuadIndex = 0;
			if (bOldMajor == true)
				iCvQuadIndex = iMajorCodeLength * evens + iMinorCodeLength * odds;
			else
				iCvQuadIndex = iMinorCodeLength * evens + iMajorCodeLength * odds;
			pattern->pattern_quads.erase(pattern->pattern_quads.begin() + (iCvQuadIndex + iMarkerCodeLength), pattern->pattern_quads.end());
			pattern->pattern_quads.erase(pattern->pattern_quads.begin(), pattern->pattern_quads.begin() + iCvQuadIndex);

#endif USE_LIBCBDETECTOR
		}


		bool estimateCylinderDirection(WorkContext context_, Point2& dirCode, Point2& dirHeight, Point3& normal)
		{
			dirCode = Point2(0, 0);
			dirHeight = Point2(0, 0);
			normal = Point3(0, 0, 0);

			if (false == (context_.m_iPrevMarkerCount == 1 && context_.m_iPrevMarkerIndex >= 0 && context_.m_iPrevMarkerIndex < m_cylinderarkerInfos__fixed.size()))
				return false;


			int iiMarkerCodeLength = m_parameters__fixed.markerCodeLength;
			int iPatternNum = (int)m_parameters__fixed.markerCodePatterns.size();
			Point2 p = context_.m_cylinderMarkerRuntimes[context_.m_iPrevMarkerIndex].m_prevImgPoints[iiMarkerCodeLength]
				- context_.m_cylinderMarkerRuntimes[context_.m_iPrevMarkerIndex].m_prevImgPoints[0];
			dirCode += p;

			int iLeftIndex = (context_.m_iPrevMarkerIndex + iPatternNum - 1) % iPatternNum;
			int iRightIndex = (context_.m_iPrevMarkerIndex + 1) % iPatternNum;

			for (int i = 0; i <= iiMarkerCodeLength; i++)
			{
				p = context_.m_cylinderMarkerRuntimes[iRightIndex].m_prevImgPoints[i] -
					context_.m_cylinderMarkerRuntimes[iLeftIndex].m_prevImgPoints[i];
				dirHeight += p;
				normal += m_cylinderarkerInfos__fixed[context_.m_iPrevMarkerIndex].m_corner3DNormals[i];

			}
			Normalize(dirCode);
			Normalize(dirHeight);
			Normalize(normal);

			return true;
		}


#ifdef USE_LIBCBDETECTOR

		bool estimatePatternDirection(SinglePattern* pattern, const std::vector<MarkerInfo>& markerInfos, int iMarkerCodeLength, int iCenterCoordinate, bool bCyclic,
			Point2& dirCode, Point2& dirHeight, Point3& normal)
		{
			dirCode = Point2(0, 0);
			dirHeight = Point2(0, 0);
			normal = Point3(0, 0, 0);

			if (markerInfos.empty() == true)
				return false;

			if (pattern->pattern_index < 0 || pattern->pattern_index >= (int)markerInfos.size()
				|| iCenterCoordinate < 0 || iCenterCoordinate >= (int)markerInfos.size())
				return false;

			int iNumBlocks = iMarkerCodeLength * (pattern->pattern_count + 1);
			if ((int)pattern->block_quads.size() != iNumBlocks)
				return false;

			if (bCyclic == false && pattern->pattern_index + pattern->pattern_count - 1 >= (int)markerInfos.size())
				return false;

			int iOffset = (int)pattern->direction;
			int iOffset_1 = (iOffset + 1) % 4;
			int iOffset_2 = (iOffset + 2) % 4;
			int iOffset_3 = (iOffset + 3) % 4;

			Point2 p;

			for (int i = 0; i <= iMarkerCodeLength; i++)
			{
				normal += markerInfos[iCenterCoordinate].m_corner3DNormals[i];
			}


			int quad_offset = 0;
			for (int i = 0; i < pattern->pattern_count; i++, quad_offset += iMarkerCodeLength)
			{
				int pattern_index = (pattern->pattern_index + i) % markerInfos.size();
				p = pattern->block_quads[quad_offset + iMarkerCodeLength - 1]->corners[iOffset_2]->pt
					- pattern->block_quads[quad_offset]->corners[iOffset_1]->pt;
				dirCode += p;

				for (int c = 0; c < iMarkerCodeLength; c++)
				{
					if (c == 0)
					{
						p = pattern->block_quads[quad_offset + c + iMarkerCodeLength]->corners[iOffset_1]->pt
							- pattern->block_quads[quad_offset + c]->corners[iOffset]->pt;
						dirHeight += p;
					}
					p = pattern->block_quads[quad_offset + c + iMarkerCodeLength]->corners[iOffset_2]->pt
						- pattern->block_quads[quad_offset + c]->corners[iOffset_3]->pt;
					dirHeight += p;
				}
			}


			Normalize(dirCode);
			Normalize(dirHeight);
			Normalize(normal);

			return true;
		}


#else USE_LIBCBDETECTOR

		bool estimatePatternDirection(SinglePattern* pattern, const std::vector<MarkerInfo>& markerInfos, int iMarkerCodeLength, int iCenterCoordinate, bool bCyclic,
			Point2& dirCode, Point2& dirHeight, Point3& normal)
		{
			dirCode = Point2(0, 0);
			dirHeight = Point2(0, 0);
			normal = Point3(0, 0, 0);

			if (markerInfos.empty() == true)
				return false;

			if (pattern->pattern_index < 0 || pattern->pattern_index >= (int)markerInfos.size()
				|| iCenterCoordinate < 0 || iCenterCoordinate >= (int)markerInfos.size())
				return false;

			int iNumBlocks = EstimateCheckerBoardNumBlocks(iMarkerCodeLength, pattern->pattern_count + 1, markerInfos[pattern->pattern_index].m_bMajor);
			if ((int)pattern->pattern_quads.size() != iNumBlocks)
				return false;

			if (bCyclic == false && pattern->pattern_index + pattern->pattern_count - 1 >= (int)markerInfos.size())
				return false;

			int iMajorCodeLength = (iMarkerCodeLength + 1) >> 1;
			int iMinorCodeLength = iMarkerCodeLength - iMajorCodeLength;

			int iOffset = (int)pattern->direction;
			int iOffset_1 = (iOffset + 1) % 4;
			int iOffset_2 = (iOffset + 2) % 4;
			int iOffset_3 = (iOffset + 3) % 4;

			Point2 p;

			for (int i = 0; i <= iMarkerCodeLength; i++)
			{
				normal += markerInfos[iCenterCoordinate].m_corner3DNormals[i];
			}

			if (iMajorCodeLength == iMinorCodeLength)
			{
				int stride = 0;
				int quad_offset = 0;
				for (int i = 0; i < pattern->pattern_count; i++, quad_offset += stride)
				{
					int pattern_index = (pattern->pattern_index + i) % markerInfos.size();
					stride = iMajorCodeLength;
					if (markerInfos[pattern_index].m_bMajor)
					{
						p = pattern->pattern_quads[quad_offset + iMarkerCodeLength - 1]->corners[iOffset_3]->pt
							- pattern->pattern_quads[quad_offset]->corners[iOffset_1]->pt;
						dirCode += p;

						for (int c = 0; c < stride; c++)
						{
							if (c != 0)
							{
								p = pattern->pattern_quads[quad_offset + c - 1 + stride]->corners[iOffset_2]->pt
									- pattern->pattern_quads[quad_offset + c]->corners[iOffset]->pt;
								dirHeight += p;
							}
							p = pattern->pattern_quads[quad_offset + c + stride]->corners[iOffset_1]->pt
								- pattern->pattern_quads[quad_offset + c]->corners[iOffset_3]->pt;
							dirHeight += p;
						}
					}
					else
					{
						p = pattern->pattern_quads[quad_offset + stride - 1]->corners[iOffset_2]->pt
							- pattern->pattern_quads[quad_offset + stride]->corners[iOffset_1]->pt;
						dirCode += p;

						for (int c = 0; c < stride; c++)
						{
							p = pattern->pattern_quads[quad_offset + c + stride]->corners[iOffset_2]->pt
								- pattern->pattern_quads[quad_offset + c]->corners[iOffset]->pt;
							dirHeight += p;
							if (c != stride - 1)
							{
								p = pattern->pattern_quads[quad_offset + c + 1 + stride]->corners[iOffset_1]->pt
									- pattern->pattern_quads[quad_offset + c]->corners[iOffset_3]->pt;
								dirHeight += p;
							}
						}
					}
				}
			}
			else
			{
				int stride = 0;
				int quad_offset = 0;

				for (int i = 0; i < pattern->pattern_count; i++, quad_offset += stride)
				{
					int pattern_index = (pattern->pattern_index + i) % markerInfos.size();

					if (markerInfos[pattern_index].m_bMajor)
					{
						stride = iMajorCodeLength;
						p = pattern->pattern_quads[quad_offset + stride - 1]->corners[iOffset_2]->pt
							- pattern->pattern_quads[quad_offset]->corners[iOffset_1]->pt;
						dirCode += p;
						for (int c = 0; c < stride; c++)
						{
							if (c != 0)
							{
								p = pattern->pattern_quads[quad_offset + c - 1 + stride]->corners[iOffset_2]->pt
									- pattern->pattern_quads[quad_offset + c]->corners[iOffset]->pt;
								dirHeight += p;
							}
							if (c != stride - 1)
							{
								p = pattern->pattern_quads[quad_offset + c + stride]->corners[iOffset_1]->pt
									- pattern->pattern_quads[quad_offset + c]->corners[iOffset_3]->pt;
								dirHeight += p;
							}
						}
					}
					else
					{
						stride = iMinorCodeLength;
						p = pattern->pattern_quads[quad_offset + iMarkerCodeLength - 1]->corners[iOffset_3]->pt
							- pattern->pattern_quads[quad_offset + stride]->corners[iOffset]->pt;
						dirCode += p;

						for (int c = 0; c < stride; c++)
						{
							p = pattern->pattern_quads[quad_offset + c + stride]->corners[iOffset_2]->pt
								- pattern->pattern_quads[quad_offset + c]->corners[iOffset]->pt
								+ pattern->pattern_quads[quad_offset + c + 1 + stride]->corners[iOffset_1]->pt
								- pattern->pattern_quads[quad_offset + c]->corners[iOffset_3]->pt;
							dirHeight += p;
						}
					}
				}
			}
			Normalize(dirCode);
			Normalize(dirHeight);
			Normalize(normal);

			return true;
		}

#endif USE_LIBCBDETECTOR


		void copyPoint(const CvCBCorner* a, Point2& p, int& err)
		{
			p = a->pt;
			err = a->imageError;

		}

		void calcAverage(const CvCBCorner* a, const CvCBCorner* b, Point2& p, int& err)
		{
			if (a == b)
			{
				err = a->imageError;
				p = a->pt;
			}
			else
			{
				float diffx = a->pt.x - b->pt.x;
				float diffy = a->pt.y - b->pt.y;
				err = (int)std::round(0.5f * (a->imageError + b->imageError) + std::max(std::abs(diffx), std::abs(diffy)));
				p = (a->pt + b->pt) * 0.5f;
			}
		}

#ifdef USE_LIBCBDETECTOR

		void update2Dcollect3DPoints(int iMarkerCodeLength, SinglePattern* pattern, std::vector<Point3>& points3d, const std::vector<MarkerInfo>& markerInfos, std::vector<MarkerRuntime>& markerRuntimes)
		{
			//if (pattern->pattern_count <= 0)
			//	return false;	
			int iPatternNums = (int)markerInfos.size();

			int iOffset_0 = (int)pattern->direction;
			int iOffset_1 = (iOffset_0 + 1) % 4;
			int iOffset_2 = (iOffset_0 + 2) % 4;
			int iOffset_3 = (iOffset_0 + 3) % 4;

		
			int iBlockOffset = 0;
			int err = 0;
			Point2 cp;
			int pattern_index = (pattern->pattern_index + iPatternNums - 1) % iPatternNums;

			points3d.insert(points3d.end(), markerInfos[pattern_index].m_corner3DPoints.begin(), markerInfos[pattern_index].m_corner3DPoints.end());
			copyPoint(pattern->block_quads[0]->corners[iOffset_0], cp, err);
			markerRuntimes[pattern_index].m_tmpImgPoints[0] = cp;
			markerRuntimes[pattern_index].m_tmpImgPointsErr[0] = err;
			for (int c = 0; c < iMarkerCodeLength; c++)
			{
				copyPoint(pattern->block_quads[c]->corners[iOffset_3], cp, err);
				markerRuntimes[pattern_index].m_tmpImgPoints[c+1] = cp;
				markerRuntimes[pattern_index].m_tmpImgPointsErr[c+1] = err;
			}

			for (int p = 0; p <= pattern->pattern_count; p++, iBlockOffset += iMarkerCodeLength )
			{
				pattern_index = (pattern_index + 1) % iPatternNums;
				points3d.insert(points3d.end(), markerInfos[pattern_index].m_corner3DPoints.begin(), markerInfos[pattern_index].m_corner3DPoints.end());
				copyPoint(pattern->block_quads[iBlockOffset]->corners[iOffset_1], cp, err);
				markerRuntimes[pattern_index].m_tmpImgPoints[0] = cp;
				markerRuntimes[pattern_index].m_tmpImgPointsErr[0] = err;
				for (int c = 0; c < iMarkerCodeLength; c++)
				{
					copyPoint(pattern->block_quads[iBlockOffset+c]->corners[iOffset_2], cp, err);
					markerRuntimes[pattern_index].m_tmpImgPoints[c+1] = cp;
					markerRuntimes[pattern_index].m_tmpImgPointsErr[c+1] = err;
				}
			}

			//Assert.IsTrue(points3d.size() == iNumPoints + iCurPoints3dNum , "NewCylindricalProbeTracker.collect2D3DPoints : mismatch");

		}

		void collect2DPoints(int iMarkerCodeLength, SinglePattern* pattern, std::vector<Point2>& pointsImg, const std::vector<MarkerInfo>& markerInfos, std::vector<MarkerRuntime>& markerRuntimes)
		{

			int iPatternNums = (int)markerInfos.size();

			int pattern_index = (pattern->pattern_index + iPatternNums - 1) % iPatternNums;
			int iBlockOffset = 0;

			for( int p = -1; p <= pattern->pattern_count; p++ )
			{
				pointsImg.insert(pointsImg.end(), markerRuntimes[pattern_index].m_tmpImgPoints.begin(), markerRuntimes[pattern_index].m_tmpImgPoints.end());
				pattern_index = (pattern_index + 1 ) % iPatternNums;
			}
		}


#else  USE_LIBCBDETECTOR

		void update2Dcollect3DPoints(int iMarkerCodeLength, SinglePattern* pattern, std::vector<Point3>& points3d, const std::vector<MarkerInfo>& markerInfos, std::vector<MarkerRuntime>& markerRuntimes)
		{
			//if (pattern->pattern_count <= 0)
			//	return false;	
			int iPatternNums = (int)markerInfos.size();
			//if (pattern->pattern_index < 0 || pattern->pattern_index >= iPatternNums)
			//	return false;

			int iCurPoints3dNum = (int)points3d.size();

			int end_pattern_index = (pattern->pattern_index + pattern->pattern_count - 1) % iPatternNums;

			int iMajorCodeLength = (iMarkerCodeLength + 1) >> 1;
			int iMinorCodeLength = iMarkerCodeLength - iMajorCodeLength;

			int prev_pattern_index = (pattern->pattern_index + iPatternNums - 1) % iPatternNums;
			int cur_pattern_index = pattern->pattern_index;
			bool bCurMajor = markerInfos[cur_pattern_index].m_bMajor;
			bool bEndMajor = markerInfos[end_pattern_index].m_bMajor;

			//int iNumPoints = pattern->pattern_count * (iMarkerCodeLength + 1)
			//	+ (bCurMajor ? iMajorCodeLength : iMinorCodeLength) * 2
			//	+ (bEndMajor ? iMajorCodeLength : iMinorCodeLength) * 2;

			int iOffset_0 = (int)pattern->direction;
			int iOffset_1 = (iOffset_0 + 1) % 4;
			int iOffset_2 = (iOffset_0 + 2) % 4;
			int iOffset_3 = (iOffset_0 + 3) % 4;

			int iCodeIndex = 0;
			int iPointIndex = 0;
			int err = 0;
			Point2 cp;

			if (iMajorCodeLength == iMinorCodeLength)
			{
				if (bCurMajor)
					points3d.insert(points3d.end(), markerInfos[prev_pattern_index].m_corner3DPoints.begin()
						, markerInfos[prev_pattern_index].m_corner3DPoints.begin() + iMarkerCodeLength);
				else
					points3d.insert(points3d.end(), markerInfos[prev_pattern_index].m_corner3DPoints.begin() + 1
						, markerInfos[prev_pattern_index].m_corner3DPoints.begin() + (1 + iMarkerCodeLength));
			}
			else
			{
				if (bCurMajor)
					points3d.insert(points3d.end(), markerInfos[prev_pattern_index].m_corner3DPoints.begin()
						, markerInfos[prev_pattern_index].m_corner3DPoints.end());
				else
					points3d.insert(points3d.end(), markerInfos[prev_pattern_index].m_corner3DPoints.begin() + 1
						, markerInfos[prev_pattern_index].m_corner3DPoints.begin() + (1 + iMinorCodeLength * 2));
			}

			int iLeftHalfCodeLength = (bCurMajor) ? iMajorCodeLength : iMinorCodeLength;

			// major, minor 모두 동일
			if (bCurMajor == false)
			{
				markerRuntimes[prev_pattern_index].m_tmpImgPointsErr[iPointIndex] = 0;
				iPointIndex++;
			}
			for (int c = 0; c < iLeftHalfCodeLength; c++)
			{
				copyPoint(pattern->pattern_quads[iCodeIndex + c]->corners[iOffset_0], cp, err);
				markerRuntimes[prev_pattern_index].m_tmpImgPoints[iPointIndex] = cp;
				markerRuntimes[prev_pattern_index].m_tmpImgPointsErr[iPointIndex] = err;
				iPointIndex++;


				copyPoint(pattern->pattern_quads[iCodeIndex + c]->corners[iOffset_3], cp, err);
				markerRuntimes[prev_pattern_index].m_tmpImgPoints[iPointIndex] = cp;
				markerRuntimes[prev_pattern_index].m_tmpImgPointsErr[iPointIndex] = err;
				iPointIndex++;
			}
			if (iPointIndex < iCodeIndex)
			{
				markerRuntimes[prev_pattern_index].m_tmpImgPointsErr[iPointIndex] = 0;
				iPointIndex++;
			}

			for (int p = 0; p < pattern->pattern_count; p++)
			{
				cur_pattern_index = (pattern->pattern_index + p) % iPatternNums;
				bool bMajor = markerInfos[cur_pattern_index].m_bMajor;
				points3d.insert(points3d.end(), markerInfos[cur_pattern_index].m_corner3DPoints.begin()
					, markerInfos[cur_pattern_index].m_corner3DPoints.end());

				iPointIndex = 0;

				if (iMajorCodeLength == iMinorCodeLength)
				{
					if (bMajor == true)
					{
						for (int c = 0; c < iMajorCodeLength; c++)
						{

							if (c == 0)
							{
								copyPoint(pattern->pattern_quads[iCodeIndex + c]->corners[iOffset_1], /*out*/cp, /*out*/err);
							}
							else
							{
								calcAverage(pattern->pattern_quads[iCodeIndex + iMajorCodeLength + c - 1]->corners[iOffset_3]
									, pattern->pattern_quads[iCodeIndex + c]->corners[iOffset_1], /*out*/cp, /*out*/err);
							}
							markerRuntimes[cur_pattern_index].m_tmpImgPoints[iPointIndex] = cp;
							markerRuntimes[cur_pattern_index].m_tmpImgPointsErr[iPointIndex] = err;
							iPointIndex++;

							calcAverage(pattern->pattern_quads[iCodeIndex + c]->corners[iOffset_2]
								, pattern->pattern_quads[iCodeIndex + iMajorCodeLength + c]->corners[iOffset_0], /*out*/cp, /*out*/err);
							markerRuntimes[cur_pattern_index].m_tmpImgPoints[iPointIndex] = cp;
							markerRuntimes[cur_pattern_index].m_tmpImgPointsErr[iPointIndex] = err;
							iPointIndex++;
						}

						copyPoint(pattern->pattern_quads[iCodeIndex + iMarkerCodeLength - 1]->corners[iOffset_3], /*out*/cp, /*out*/err);
						markerRuntimes[cur_pattern_index].m_tmpImgPoints[iPointIndex] = cp;
						markerRuntimes[cur_pattern_index].m_tmpImgPointsErr[iPointIndex] = err;
						iPointIndex++;
					}
					else
					{

						copyPoint(pattern->pattern_quads[iCodeIndex + iMinorCodeLength]->corners[iOffset_0], /*out*/cp, /*out*/err);
						markerRuntimes[cur_pattern_index].m_tmpImgPoints[iPointIndex] = cp;
						markerRuntimes[cur_pattern_index].m_tmpImgPointsErr[iPointIndex] = err;
						iPointIndex++;
						for (int c = 0; c < iMinorCodeLength; c++)
						{
							calcAverage(pattern->pattern_quads[iCodeIndex + iMinorCodeLength + c]->corners[iOffset_3]
								, pattern->pattern_quads[iCodeIndex + c]->corners[iOffset_1], /*out*/cp, /*out*/err);
							markerRuntimes[cur_pattern_index].m_tmpImgPoints[iPointIndex] = cp;
							markerRuntimes[cur_pattern_index].m_tmpImgPointsErr[iPointIndex] = err;
							iPointIndex++;

							if (c + 1 != iMinorCodeLength)
							{
								calcAverage(pattern->pattern_quads[iCodeIndex + c]->corners[iOffset_2]
									, pattern->pattern_quads[iCodeIndex + iMinorCodeLength + c + 1]->corners[iOffset_0], /*out*/cp, /*out*/err);
							}
							else
							{
								copyPoint(pattern->pattern_quads[iCodeIndex + c]->corners[iOffset_2], /*out*/cp, /*out*/err);
							}
							markerRuntimes[cur_pattern_index].m_tmpImgPoints[iPointIndex] = cp;
							markerRuntimes[cur_pattern_index].m_tmpImgPointsErr[iPointIndex] = err;
							iPointIndex++;
						}
					}
				}
				else
				{
					if (bMajor == true)
					{
						for (int c = 0; c < iMajorCodeLength; c++)
						{

							if (c == 0)
							{
								copyPoint(pattern->pattern_quads[iCodeIndex + c]->corners[iOffset_1], /*out*/cp, /*out*/err);
							}
							else
							{
								calcAverage(pattern->pattern_quads[iCodeIndex + iMajorCodeLength + c - 1]->corners[iOffset_3]
									, pattern->pattern_quads[iCodeIndex + c]->corners[iOffset_1], /*out*/cp, /*out*/err);
							}
							markerRuntimes[cur_pattern_index].m_tmpImgPoints[iPointIndex] = cp;
							markerRuntimes[cur_pattern_index].m_tmpImgPointsErr[iPointIndex] = err;
							iPointIndex++;


							if (c + 1 != iMajorCodeLength)
							{
								calcAverage(pattern->pattern_quads[iCodeIndex + c]->corners[iOffset_2]
									, pattern->pattern_quads[iCodeIndex + iMajorCodeLength + c]->corners[iOffset_0], /*out*/cp, /*out*/err);
							}
							else
							{
								copyPoint(pattern->pattern_quads[iCodeIndex + c]->corners[iOffset_2], /*out*/cp, /*out*/err);
							}
							markerRuntimes[cur_pattern_index].m_tmpImgPoints[iPointIndex] = cp;
							markerRuntimes[cur_pattern_index].m_tmpImgPointsErr[iPointIndex] = err;
							iPointIndex++;
						}
					}
					else
					{
						copyPoint(pattern->pattern_quads[iCodeIndex + iMinorCodeLength]->corners[iOffset_0], /*out*/cp, /*out*/err);
						markerRuntimes[cur_pattern_index].m_tmpImgPoints[iPointIndex] = cp;
						markerRuntimes[cur_pattern_index].m_tmpImgPointsErr[iPointIndex] = err;
						iPointIndex++;

						for (int c = 0; c < iMinorCodeLength; c++)
						{
							calcAverage(pattern->pattern_quads[iCodeIndex + iMinorCodeLength + c]->corners[iOffset_3]
								, pattern->pattern_quads[iCodeIndex + c]->corners[iOffset_1], /*out*/cp, /*out*/err);
							markerRuntimes[cur_pattern_index].m_tmpImgPoints[iPointIndex] = cp;
							markerRuntimes[cur_pattern_index].m_tmpImgPointsErr[iPointIndex] = err;
							iPointIndex++;

							calcAverage(pattern->pattern_quads[iCodeIndex + c]->corners[iOffset_2]
								, pattern->pattern_quads[iCodeIndex + iMinorCodeLength + c + 1]->corners[iOffset_0], /*out*/cp, /*out*/err);
							markerRuntimes[cur_pattern_index].m_tmpImgPoints[iPointIndex] = cp;
							markerRuntimes[cur_pattern_index].m_tmpImgPointsErr[iPointIndex] = err;
							iPointIndex++;
						}
						copyPoint(pattern->pattern_quads[iCodeIndex + iMarkerCodeLength - 1]->corners[iOffset_3], /*out*/cp, /*out*/err);
						markerRuntimes[cur_pattern_index].m_tmpImgPoints[iPointIndex] = cp;
						markerRuntimes[cur_pattern_index].m_tmpImgPointsErr[iPointIndex] = err;
						iPointIndex++;
					}
				}
				iCodeIndex += (bMajor) ? iMajorCodeLength : iMinorCodeLength;
			}

			int next_pattern_index = (pattern->pattern_index + pattern->pattern_count) % iPatternNums;

			if (iMajorCodeLength == iMinorCodeLength)
			{
				if (bEndMajor)
					points3d.insert(points3d.end(), markerInfos[next_pattern_index].m_corner3DPoints.begin() + 1
						, markerInfos[next_pattern_index].m_corner3DPoints.begin() + (1 + iMarkerCodeLength));
				else
					points3d.insert(points3d.end(), markerInfos[next_pattern_index].m_corner3DPoints.begin()
						, markerInfos[next_pattern_index].m_corner3DPoints.begin() + iMarkerCodeLength);
			}
			else
			{
				if (bEndMajor)
					points3d.insert(points3d.end(), markerInfos[next_pattern_index].m_corner3DPoints.begin() + 1
						, markerInfos[next_pattern_index].m_corner3DPoints.begin() + (1 + iMinorCodeLength * 2));
				else
					points3d.insert(points3d.end(), markerInfos[next_pattern_index].m_corner3DPoints.begin()
						, markerInfos[next_pattern_index].m_corner3DPoints.end());
			}

			iLeftHalfCodeLength = (bEndMajor) ? iMajorCodeLength : iMinorCodeLength;
			int iRightHalfCodeLength = iMarkerCodeLength - iLeftHalfCodeLength;

			iPointIndex = 0;
			if (bCurMajor == true)
			{
				markerRuntimes[next_pattern_index].m_tmpImgPointsErr[iPointIndex] = 0;
				iPointIndex++;
			}

			// major, minor 모두 동일
			for (int c = 0; c < iRightHalfCodeLength; c++)
			{
				copyPoint(pattern->pattern_quads[iCodeIndex + c]->corners[iOffset_1], /*out*/cp, /*out*/err);
				markerRuntimes[next_pattern_index].m_tmpImgPoints[iPointIndex] = cp;
				markerRuntimes[next_pattern_index].m_tmpImgPointsErr[iPointIndex] = err;
				iPointIndex++;

				copyPoint(pattern->pattern_quads[iCodeIndex + c]->corners[iOffset_2], /*out*/cp, /*out*/err);
				markerRuntimes[next_pattern_index].m_tmpImgPoints[iPointIndex] = cp;
				markerRuntimes[next_pattern_index].m_tmpImgPointsErr[iPointIndex] = err;
				iPointIndex++;
			}
			if (iPointIndex < iCodeIndex)
			{
				markerRuntimes[next_pattern_index].m_tmpImgPointsErr[iPointIndex] = 0;
				iPointIndex++;
			}

			//Assert.IsTrue(points3d.size() == iNumPoints + iCurPoints3dNum , "NewCylindricalProbeTracker.collect2D3DPoints : mismatch");

		}

		void collect2DPoints(int iMarkerCodeLength, SinglePattern* pattern, std::vector<Point2>& pointsImg, const std::vector<MarkerInfo>& markerInfos, std::vector<MarkerRuntime>& markerRuntimes)
		{
			//if (pattern->pattern_count <= 0)
			//	return false;	
			int iPatternNums = (int)markerInfos.size();
			//if (pattern->pattern_index < 0 || pattern->pattern_index >= iPatternNums)
			//	return false;

			int iCurPointsImgNum = (int)pointsImg.size();

			int end_pattern_index = (pattern->pattern_index + pattern->pattern_count - 1) % iPatternNums;

			int iMajorCodeLength = (iMarkerCodeLength + 1) >> 1;
			int iMinorCodeLength = iMarkerCodeLength - iMajorCodeLength;

			int prev_pattern_index = (pattern->pattern_index + iPatternNums - 1) % iPatternNums;
			int cur_pattern_index = pattern->pattern_index;
			bool bCurMajor = markerInfos[cur_pattern_index].m_bMajor;
			bool bEndMajor = markerInfos[end_pattern_index].m_bMajor;

			int iNumPoints = pattern->pattern_count * (iMarkerCodeLength + 1)
				+ (bCurMajor ? iMajorCodeLength : iMinorCodeLength) * 2
				+ (bEndMajor ? iMajorCodeLength : iMinorCodeLength) * 2;

			int iOffset_0 = (int)pattern->direction;
			int iOffset_1 = (iOffset_0 + 1) % 4;
			int iOffset_2 = (iOffset_0 + 2) % 4;
			int iOffset_3 = (iOffset_0 + 3) % 4;

			int iCodeIndex = 0;
			int iPointIndex = 0;

			int iLeftHalfCodeLength = (bCurMajor) ? iMajorCodeLength : iMinorCodeLength;

			// major, minor 모두 동일
			if (bCurMajor == false)
			{
				iPointIndex++;
			}
			pointsImg.insert(pointsImg.end(), markerRuntimes[prev_pattern_index].m_tmpImgPoints.begin() + iPointIndex
				, markerRuntimes[prev_pattern_index].m_tmpImgPoints.begin() + (iPointIndex + iLeftHalfCodeLength * 2));
			iPointIndex += 2 * iLeftHalfCodeLength;
			if (iPointIndex < iCodeIndex)
			{
				iPointIndex++;
			}

			for (int p = 0; p < pattern->pattern_count; p++)
			{
				cur_pattern_index = (pattern->pattern_index + p) % iPatternNums;
				bool bMajor = markerInfos[cur_pattern_index].m_bMajor;

				iPointIndex = 0;

				if (iMajorCodeLength == iMinorCodeLength)
				{
					pointsImg.insert(pointsImg.end(), markerRuntimes[cur_pattern_index].m_tmpImgPoints.begin() + iPointIndex
						, markerRuntimes[cur_pattern_index].m_tmpImgPoints.begin() + (iPointIndex + iMajorCodeLength * 2 + 1));
					iPointIndex += iMajorCodeLength * 2 + 1;
				}
				else
				{
					if (bMajor == true)
					{
						pointsImg.insert(pointsImg.end(), markerRuntimes[cur_pattern_index].m_tmpImgPoints.begin() + iPointIndex
							, markerRuntimes[cur_pattern_index].m_tmpImgPoints.begin() + (iPointIndex + iMajorCodeLength * 2));
						iPointIndex += iMajorCodeLength * 2;
					}
					else
					{
						pointsImg.insert(pointsImg.end(), markerRuntimes[cur_pattern_index].m_tmpImgPoints.begin() + iPointIndex
							, markerRuntimes[cur_pattern_index].m_tmpImgPoints.begin() + (iPointIndex + iMinorCodeLength * 2 + 2));
						iPointIndex += iMinorCodeLength * 2 + 2;
					}
				}
				iCodeIndex += (bMajor) ? iMajorCodeLength : iMinorCodeLength;
			}

			int next_pattern_index = (pattern->pattern_index + pattern->pattern_count) % iPatternNums;


			iLeftHalfCodeLength = (bEndMajor) ? iMajorCodeLength : iMinorCodeLength;
			int iRightHalfCodeLength = iMarkerCodeLength - iLeftHalfCodeLength;

			iPointIndex = 0;
			if (bCurMajor == true)
			{
				iPointIndex++;
			}

			// major, minor 모두 동일
			pointsImg.insert(pointsImg.end(), markerRuntimes[next_pattern_index].m_tmpImgPoints.begin() + iPointIndex
				, markerRuntimes[next_pattern_index].m_tmpImgPoints.begin() + (iPointIndex + iRightHalfCodeLength * 2));
			iPointIndex += iRightHalfCodeLength * 2;

			if (iPointIndex < iCodeIndex)
			{
				iPointIndex++;
			}

			//Assert.IsTrue(pointsImg.size() == iNumPoints + iCurPointsImgNum
			//	, "NewCylindricalProbeTracker.collect2D3DPoints : mismatch");

		}

#endif USE_LIBCBDETECTOR





		void refineCyclicCorners(cv::Mat& matGray, int iPatternIndex, int iPatternCount, const std::vector<MarkerInfo>& markerInfo, std::vector<MarkerRuntime>& markerRuntime
			, std::vector<int>& listCornerCounter, std::vector<Point2>& listCorners, double xScale = 1.0, double yScale = 1.0)
		{
			int iPatternNums = (int)markerInfo.size();
			if (iPatternCount <= 0 || iPatternIndex < 0 || iPatternIndex >= iPatternNums)
				return;

			int cur_pattern_index = iPatternIndex;
			int prev_pattern_index = (iPatternIndex + iPatternNums - 1) % iPatternNums;
			int end_pattern_index = (iPatternIndex + iPatternCount - 1) % iPatternNums;
			int iMarkerCodeLength = m_parameters__fixed.markerCodeLength;

			bool bCurMajor = markerInfo[cur_pattern_index].m_bMajor;
			bool bEndMajor = markerInfo[end_pattern_index].m_bMajor;

			int iNumPoints = iPatternCount * (iMarkerCodeLength + 3);

			listCornerCounter.clear();
			listCorners.clear(); listCorners.reserve(iNumPoints);
			int iCornerCount = 0;

			for (int i = -1; i <= iPatternCount; i++)
			{
				int index = (iPatternIndex + iPatternNums + i) % iPatternNums;
				for (int c = 0; c <= iMarkerCodeLength; c++)
				{
					int ierro = markerRuntime[index].m_tmpImgPointsErr[c];
					if (ierro <= 0)
						continue;
					if (ierro >= (int)listCornerCounter.size())
					{
						listCornerCounter.resize(ierro + 1, 0);
					}
					listCornerCounter[ierro]++;
					iCornerCount++;
				}
			}

			for (int err = 0; err < (int)listCornerCounter.size(); err++)
			{
				if (listCornerCounter[err] == 0)
					continue;

				listCorners.clear();

				for (int i = -1; i <= iPatternCount; i++)
				{
					int index = (iPatternIndex + iPatternNums + i) % iPatternNums;
					for (int c = 0; c <= iMarkerCodeLength; c++)
					{
						if (markerRuntime[index].m_tmpImgPointsErr[c] != err)
							continue;
						listCorners.push_back(markerRuntime[index].m_tmpImgPoints[c]);
					}
				}
				
				for (int iCornerIndex = 0; iCornerIndex < listCorners.size(); ++iCornerIndex)
				{
					listCorners[iCornerIndex].x *= xScale;
					listCorners[iCornerIndex].y *= yScale;
				}

				//Imgproc.cornerSubPix(m_curGray, matCorners, new Size(dilation+3, dilation+3), new Size(-1, -1),
				cv::cornerSubPix(matGray, listCorners
					, Size(err + m_parameters__fixed.cornerPixelTolerance, err + m_parameters__fixed.cornerPixelTolerance)
					, Size(-1, -1)
					, cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS,
						m_parameters__fixed.detectorParams.cornerRefinementMaxIterations,
						m_parameters__fixed.detectorParams.cornerRefinementMinAccuracy));

				int iCount = 0;
				for (int i = -1; i <= iPatternCount; i++)
				{
					int index = (iPatternIndex + iPatternNums + i) % iPatternNums;
					for (int c = 0; c <= iMarkerCodeLength; c++)
					{
						if (markerRuntime[index].m_tmpImgPointsErr[c] != err)
							continue;
						markerRuntime[index].m_tmpImgPoints[c] = listCorners[iCount++];
					}
				}
			}
		}

#ifdef USE_LIBCBDETECTOR

		void refinePlaneCorners(int iXWidth, cv::Mat& matGray, const std::vector<MarkerInfo>& markerInfo, std::vector<MarkerRuntime>& markerRuntime
			, std::vector<int>& listCornerCounter, std::vector<Point2>& listCorners, double xScale = 1.0, double yScale = 1.0 )
		{
			if (markerRuntime.size() != markerInfo.size())
				return;
			int iYCount = (int)markerInfo.size();
			int iNumPoints = iYCount * (iXWidth + 1);

			listCornerCounter.clear();
			listCorners.clear(); listCorners.reserve(iNumPoints);
			int iCornerCount = 0;

			for (int i = 0; i < iYCount; i++)
			{
				for (int c = 0; c <= iXWidth; c++)
				{
					int ierro = markerRuntime[i].m_tmpImgPointsErr[c];
					if (ierro <= 0)
						continue;
					if (ierro >= listCornerCounter.size())
					{
						listCornerCounter.resize(ierro + 1, 0);
					}
					listCornerCounter[ierro]++;
					iCornerCount++;
				}
			}

			for (int err = 0; err < (int)listCornerCounter.size(); err++)
			{
				if (listCornerCounter[err] == 0)
					continue;

				listCorners.clear();
				for (int i = 0; i < iYCount; i++)
				{
					for (int c = 0; c <= iXWidth; c++)
					{
						if (markerRuntime[i].m_tmpImgPointsErr[c] == err)
						{
							listCorners.push_back(markerRuntime[i].m_tmpImgPoints[c]);
						}
					}
				}
				
				for (int iCornerIndex = 0; iCornerIndex < listCorners.size(); ++iCornerIndex)
				{
					listCorners[iCornerIndex].x *= xScale;
					listCorners[iCornerIndex].y *= yScale;
				}
				cv::cornerSubPix(matGray, listCorners
					, Size(err + m_parameters__fixed.cornerPixelTolerance, err + m_parameters__fixed.cornerPixelTolerance)
					, Size(-1, -1)
					, cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS
						, m_parameters__fixed.detectorParams.cornerRefinementMaxIterations
						, m_parameters__fixed.detectorParams.cornerRefinementMinAccuracy));

				int iCount = 0;
				for (int i = 0; i < iYCount; i++)
				{
					for (int c = 0; c <= iXWidth; c++)
					{
						if (markerRuntime[i].m_tmpImgPointsErr[c] == err)
						{
							markerRuntime[i].m_tmpImgPoints[c] = listCorners[iCount++];
						}
					}
				}
			}
		}

#else  USE_LIBCBDETECTOR

		void refinePlaneCorners(int iXWidth, cv::Mat& matGray, const std::vector<MarkerInfo>& markerInfo, std::vector<MarkerRuntime>& markerRuntime
			, std::vector<int>& listCornerCounter, std::vector<Point2>& listCorners)
		{
			if (markerRuntime.size() != markerInfo.size())
				return;
			int iYCount = (int)markerInfo.size();

			bool bCurMajor = markerInfo[1].m_bMajor;

			int iMajorCodeLength = (iXWidth + 1) >> 1;
			int iMinorCodeLength = iXWidth - iMajorCodeLength;
			int iNumPoints = iYCount * (iXWidth + 1);

			listCornerCounter.clear();
			listCorners.clear(); listCorners.reserve(iNumPoints);
			int iCornerCount = 0;

			bool bMajor = bCurMajor;
			int iStartX, iEndX;
			for (int i = 0; i < iYCount; i++)
			{
				if (i > 0 && i < iYCount - 1)
				{
					iStartX = 0;
					iEndX = iXWidth;
				}
				else
				{
					if (iMajorCodeLength == iMinorCodeLength)
					{
						if (bMajor == true)
						{
							iStartX = 0;
							iEndX = iXWidth - 1;
						}
						else
						{
							iStartX = 1;
							iEndX = iXWidth;
						}
					}
					else
					{
						if (bMajor == true)
						{
							iStartX = 0;
							iEndX = iXWidth;
						}
						else
						{
							iStartX = 1;
							iEndX = iXWidth - 1;
						}
					}
					if (i != 0)
					{
						bMajor = !bMajor;
					}
				}

				for (int c = iStartX; c <= iEndX; c++)
				{
					int ierro = markerRuntime[i].m_tmpImgPointsErr[c];
					if (ierro <= 0)
						continue;
					if (ierro >= listCornerCounter.size())
					{
						listCornerCounter.resize(ierro + 1, 0);
					}
					listCornerCounter[ierro]++;
					iCornerCount++;
				}
			}

			for (int err = 0; err < (int)listCornerCounter.size(); err++)
			{
				if (listCornerCounter[err] == 0)
					continue;

				listCorners.clear();
				bMajor = bCurMajor;
				for (int i = 0; i < iYCount; i++)
				{
					if (i > 0 && i < iYCount - 1)
					{
						iStartX = 0;
						iEndX = iXWidth;
					}
					else
					{
						if (iMajorCodeLength == iMinorCodeLength)
						{
							if (bMajor == true)
							{
								iStartX = 0;
								iEndX = iXWidth - 1;
							}
							else
							{
								iStartX = 1;
								iEndX = iXWidth;
							}
						}
						else
						{
							if (bMajor == true)
							{
								iStartX = 0;
								iEndX = iXWidth;
							}
							else
							{
								iStartX = 1;
								iEndX = iXWidth - 1;
							}
						}
						if (i != 0)
						{
							bMajor = !bMajor;
						}
					}

					for (int c = iStartX; c <= iEndX; c++)
					{
						if (markerRuntime[i].m_tmpImgPointsErr[c] != err)
							continue;
						listCorners.push_back(markerRuntime[i].m_tmpImgPoints[c]);
					}
				}

				cv::cornerSubPix(matGray, listCorners
					, Size(err + m_parameters__fixed.cornerPixelTolerance, err + m_parameters__fixed.cornerPixelTolerance)
					, Size(-1, -1)
					, cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS
						, m_parameters__fixed.detectorParams.cornerRefinementMaxIterations
						, m_parameters__fixed.detectorParams.cornerRefinementMinAccuracy));


				int iCount = 0;
				bMajor = bCurMajor;
				for (int i = 0; i < iYCount; i++)
				{
					if (i > 0 && i < iYCount - 1)
					{
						iStartX = 0;
						iEndX = iXWidth;
					}
					else
					{
						if (iMajorCodeLength == iMinorCodeLength)
						{
							if (bMajor == true)
							{
								iStartX = 0;
								iEndX = iXWidth - 1;
							}
							else
							{
								iStartX = 1;
								iEndX = iXWidth;
							}
						}
						else
						{
							if (bMajor == true)
							{
								iStartX = 0;
								iEndX = iXWidth;
							}
							else
							{
								iStartX = 1;
								iEndX = iXWidth - 1;
							}
						}
						if (i != 0)
						{
							bMajor = !bMajor;
						}
					}

					for (int c = iStartX; c <= iEndX; c++)
					{
						if (markerRuntime[i].m_tmpImgPointsErr[c] != err)
							continue;
						markerRuntime[i].m_tmpImgPoints[c] = listCorners[iCount++];
					}
				}
			}
		}

#endif USE_LIBCBDETECTOR


		//void refineBlockCorners(Mat matGray, List<MarkerInfo> markerInfo, List<MarkerRuntime> markerRuntime)
		//{
		//	if (markerInfo.size() != 2 || markerRuntime.size() != 2)
		//		return;

		//	List<int> listCornerCounter = new List<int>();
		//	List<Point> listCorners = new List<Point>(4);
		//	int iCornerCount = 0;

		//	for (int i = 0; i <= 1; i++)
		//	{
		//		for (int c = 0; c <=1; c++)
		//		{
		//			int ierro = markerRuntime[i].m_tmpImgPointsErr[c];
		//			if (ierro <= 0)
		//				continue;
		//			if (ierro >= listCornerCounter.size())
		//			{
		//				for (int k = listCornerCounter.size(); k <= ierro; k++)
		//					listCornerCounter.Add(0);
		//			}
		//			listCornerCounter[ierro]++;
		//			iCornerCount++;
		//		}
		//	}

		//	for (int err = 0; err < listCornerCounter.size(); err++)
		//	{
		//		if (listCornerCounter[err] == 0)
		//			continue;

		//		listCorners.Clear();
		//		for (int i = 0; i <= 1; i++)
		//		{
		//			for (int c = 0; c <=1; c++)
		//			{
		//				if (markerRuntime[i].m_tmpImgPointsErr[c] != err)
		//					continue;
		//				listCorners.Add(markerRuntime[i].m_tmpImgPoints[c]);
		//			}
		//		}

		//		MatOfPoint2f matCorners = new MatOfPoint2f();
		//		matCorners.fromList(listCorners);
		//		//Imgproc.cornerSubPix(m_curGray, matCorners, new Size(dilation+3, dilation+3), new Size(-1, -1),
		//		Imgproc.cornerSubPix(matGray, matCorners, new Size(err + m_parameters__fixed.cornerPixelTolerance, err + m_parameters__fixed.cornerPixelTolerance), new Size(-1, -1),
		//										new TermCriteria(TermCriteria.MAX_ITER | TermCriteria.EPS,
		//												m_parameters__fixed.detectorParams.get_cornerRefinementMaxIterations(),
		//												m_parameters__fixed.detectorParams.get_cornerRefinementMinAccuracy()));

		//		float[] afCorners = new float[2 * listCorners.size()];
		//		matCorners.get(0, 0, afCorners);

		//		for (int i = 0; i < listCorners.size(); i++)
		//		{
		//			listCorners[i].x = afCorners[i * 2];
		//			listCorners[i].y = afCorners[i * 2 + 1];
		//		}
		//	}
		//}


		//static void computeZXYToRotation(double z, double x, double y, cv::Mat& mat )
		//{
		//	mat.create(3, 3, CV_64FC1);
		//	bool bSet = false;
		//	double init[3];
		//	cv::Mat rvec(3, 1, CV_64FC1, init);
		//	double tmp[9];
		//	cv::Mat matTmp(3, 3, CV_64FC1, tmp);

		//	if (z != 0.0)
		//	{
		//		init[0] = 0; init[1] = 0; init[2] = z;
		//		cv::Rodrigues(rvec, mat);
		//		bSet = true;
		//	}
		//	if (x != 0.0)
		//	{
		//		init[0] = x; init[1] = 0; init[2] = 0;
		//		if (bSet == false)
		//		{
		//			cv::Rodrigues(rvec, mat);
		//			bSet = true;
		//		}
		//		else
		//		{
		//			cv::Rodrigues(rvec, matTmp);
		//			cv::gemm(matTmp, mat, 1.0, 0, 0, mat);
		//		}
		//	}
		//	if ( y != 0.0 )
		//	{
		//		init[0] = 0; init[1] = y; init[2] = 0;
		//		if (bSet == false)
		//		{
		//			cv::Rodrigues(rvec, mat);
		//			bSet = true;
		//		}
		//		else
		//		{
		//			cv::Rodrigues(rvec, matTmp);
		//			cv::gemm(matTmp, mat, 1.0, 0, 0, mat);
		//		}
		//	}
		//	if (bSet == false)
		//	{
		//		double* ptr = mat.ptr<double>();
		//		ptr[0] = 1; ptr[1] = 0; ptr[2] = 0;
		//		ptr[3] = 0; ptr[4] = 1; ptr[5] = 0;
		//		ptr[6] = 0; ptr[7] = 0; ptr[8] = 1;
		//	}
		//}

		static Point2 computeProjection(const cv::Mat& matProj, const Point3& p3)
		{
			double init[4];
			double arrP2[3];
			init[0] = p3.x; init[1] = p3.y; init[2] = p3.z; init[3] = 1.0;
			cv::Mat matP3(4, 1, CV_64FC1, init);
			cv::Mat matResult(3, 1, CV_64FC1, arrP2);
			cv::gemm(matProj, matP3, 1.0, 0, 0, matResult);
			double* result = matResult.ptr<double>();
			return Point2((float)(result[0] / result[2]), (float)(result[1] / result[2]));
		}

		static Point2 computeCameraProjection(const cv::Mat& matCamera, const Point3& p3)
		{
			double init[3];
			double arrP2[3];
			init[0] = p3.x; init[1] = p3.y; init[2] = p3.z;
			cv::Mat matP3(3, 1, CV_64FC1, init);
			cv::Mat matResult(3, 1, CV_64FC1, arrP2);
			cv::gemm(matCamera, matP3, 1.0, 0, 0, matResult);
			double* result = matResult.ptr<double>();
			return Point2((float)(result[0] / result[2]), (float)(result[1] / result[2]));
		}

		static void   setTransformMat(cv::Mat& cvMatTransform, const Point3& pos, const Quaternion& q)
		{
			double* pData = cvMatTransform.ptr<double>();

			pData[0] = 1.f - 2 * (q[1] * q[1] + q[2] * q[2]);
			pData[1] = 2 * (q[0] * q[1] - q[2] * q[3]);
			pData[2] = 2 * (q[0] * q[2] + q[1] * q[3]);
			pData[3] = pos.x;
			pData[4] = 2 * (q[0] * q[1] + q[2] * q[3]);
			pData[5] = 1.f - 2 * (q[0] * q[0] + q[2] * q[2]);
			pData[6] = 2 * (q[1] * q[2] - q[0] * q[3]);
			pData[7] = pos.y;
			pData[8] = 2 * (q[0] * q[2] - q[1] * q[3]);
			pData[9] = 2 * (q[1] * q[2] + q[0] * q[3]);
			pData[10] = 1.f - 2 * (q[0] * q[0] + q[1] * q[1]);
			pData[11] = pos.z;
			pData[12] = 0.f;
			pData[13] = 0.f;
			pData[14] = 0.f;
			pData[15] = 1.f;
		}


		static Point3 computeTransform(const cv::Mat& matTrans, const Point3& p3)
		{
			double init[4];
			double arrDest[4];
			init[0] = p3.x; init[1] = p3.y; init[2] = p3.z; init[3] = 1.f;
			cv::Mat matP3(4, 1, CV_64FC1, init);
			cv::Mat matDest(4, 1, CV_64FC1, arrDest);
			cv::gemm(matTrans, matP3, 1.0, 0, 0, matDest);
			double* result = matDest.ptr<double>();
			return Point3((float)result[0], (float)result[1], (float)result[2]);
		}


		static Point3 computeRotation(const cv::Mat& matRots, const Point3& p3)
		{
			double init[3];
			double arrDest[3];
			init[0] = p3.x; init[1] = p3.y; init[2] = p3.z;
			cv::Mat matP3(3, 1, CV_64FC1, init);
			cv::Mat matDest(3, 1, CV_64FC1, arrDest);
			cv::gemm(matRots, matP3, 1.0, 0, 0, matDest);
			double* result = matDest.ptr<double>();
			return Point3((float)result[0], (float)result[1], (float)result[2]);
		}

		static void decomposeAxes(const cv::Mat& cvMatTransform, /*out*/Point3& x, /*out*/Point3& y, /*out*/Point3& z)
		{
			const double* arrRot = cvMatTransform.ptr<double>();
			x.x = (float)arrRot[0];
			x.y = (float)arrRot[4];
			x.z = (float)arrRot[8];
			y.x = (float)arrRot[1];
			y.y = (float)arrRot[5];
			y.z = (float)arrRot[9];
			z.x = (float)arrRot[2];
			z.y = (float)arrRot[6];
			z.z = (float)arrRot[10];
		}

		int checkUpDown(const cv::Mat& cvMatGray, const Point2& a, const Point2& b, std::vector<int>& vecCode, std::vector<Byte>& listByte
			, std::vector<int>& veciCodeBit, std::vector<bool>& vecbCodeBitOK)
		{
			if (vecCode.size() == 0)
				return -1;

			int width = cvMatGray.cols;
			int height = cvMatGray.rows;
			Point2i imgSize(width, height);
			Point2i pt1((int)(a.x + 0.5f), (int)(a.y + 0.5f));
			Point2i pt2((int)(b.x + 0.5f), (int)(b.y + 0.5f));

			if (pt1.x < 0 || pt1.x >= width || pt2.x < 0 || pt2.x >= width)
				return 0;
			if (pt1.y < 0 || pt1.y >= height || pt2.y < 0 || pt2.y >= height)
				return 0;

			{
				LineIter line(imgSize, pt1, pt2, false, false);
				listByte.clear();
				listByte.reserve(line.GetCount());

				Point2i p;
				while (line.Next(p))
				{
					listByte.push_back(cvMatGray.at<Byte>(p.y, p.x));
				}
			}
			int ilistByteSize = (int)listByte.size();
			listByte.resize(ilistByteSize * 2);

			cv::Mat matLine(1, ilistByteSize, CV_8UC1, listByte.data());
			cv::Mat matResult(1, ilistByteSize, CV_8UC1, listByte.data() + ilistByteSize);
			cv::threshold(matLine, matResult, 0.0, 255.0, cv::THRESH_BINARY | cv::THRESH_OTSU);
			cv::medianBlur(matResult, matLine, 5);
			const Byte* listByte2 = matLine.ptr<Byte>();

			int iCodeSize = (int)vecCode.size();
			veciCodeBit.clear(); veciCodeBit.reserve(iCodeSize);
			vecbCodeBitOK.clear(); vecbCodeBitOK.resize(iCodeSize, true);
			for (int& code : vecCode)
			{
				veciCodeBit.push_back(code & 1);
				code >>= 1;
			}
			for (int i = 0; i < ilistByteSize; i++)
			{
				Byte value = listByte2[i];
				int iValueBit = value < 128 ? 0 : 1;

				int iFailCount = 0;
				for (int i = 0; i < iCodeSize; i++)
				{
					if (vecbCodeBitOK[i])
					{
						if (iValueBit != veciCodeBit[i])
						{
							int& code = vecCode[i];
							veciCodeBit[i] = code & 1;
							code >>= 1;
							if (iValueBit != veciCodeBit[i])
							{
								vecbCodeBitOK[i] = false;
								iFailCount++;
							}
						}
					}
					else
					{
						iFailCount++;
					}

				}
				if (iFailCount >= iCodeSize)
				{
					return -1;
				}
			}
			for (int i = 0; i < iCodeSize; i++)
			{
				if (vecbCodeBitOK[i])
				{
					int iCodeBit = veciCodeBit[i];
					int code = vecCode[i];
					if ((iCodeBit == 0 && code == 0) || (iCodeBit == 1 && code == -1))
					{
						return i;
					}
				}
			}
			return -1;
		}


		static bool calc2DSegmentsIntersection(const Point2& p1, const Point2& p2, const Point2& p3, const Point2& p4, /*out*/Point2& p)
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


		bool computeAndVerifyReprojection(WorkContext& context_, const cv::Mat& cvMatGray
			, const cv::Mat& cvMatRotation, const cv::Mat& cvMatTransform, const cv::Mat& cvMatProjection
			, bool bTrack, CVRUpdate eUpdate
			, int& iPatternIndex, int& iPatternCount, uint& uTrackType, /*out*/uint& uPossibleTrackType
			, /*out*/float& dMaxEdge, /*out*/OpenCVRect& rectAll
			, std::vector<VerifyProjectionRet>& listOutput
			, std::vector<int>& vecUpDownCode
			, std::vector<Byte>& listByte
			, std::vector<int>& veciCodeBit
			, std::vector<bool>& vecbCodeBitOK)
		{
			int auxnum = (int)m_parameters__fixed.auxiliaryMarkerSettings.size();
			listOutput.clear();
			listOutput.resize(1 + auxnum);
			dMaxEdge = 0.0f;
			rectAll = OpenCVRect(0, 0, 0, 0);

			int iPatternNums = (int)m_parameters__fixed.markerCodePatterns.size();
			if (iPatternCount != 1 || iPatternIndex < 0 || iPatternIndex >= iPatternNums)
			{
				iPatternIndex = -1;
				iPatternCount = 0;
			}

			int iMarkerCodeLength = m_parameters__fixed.markerCodeLength;

			Point3 p3Origin = computeTransform(cvMatTransform, Point3(0, 0, 0));
			Point3 p3AxisX;
			Point3 p3AxisY;
			Point3 p3AxisZ;
			decomposeAxes(cvMatTransform, /*out*/p3AxisX, /*out*/p3AxisY, /*out*/p3AxisZ);
			Normalize(p3AxisX);
			Normalize(p3AxisY);
			Normalize(p3AxisZ);
			//Point3 p3Cam = (p3Origin - p3Origin.dot(p3AxisZ) * p3AxisZ);
			Point3 p3DirCam = -(p3Origin - p3AxisZ * p3Origin.dot(p3AxisZ));
			float dDistCam = Normalize(p3DirCam);
			if (dDistCam <= m_parameters__fixed.radius)
				dDistCam = 0.0f;

			uint uCheck = m_uMarkerMask__fixed;
			uPossibleTrackType = 0;

			for (int aux = 0; aux < auxnum; aux++)
			{
				if (m_auxliaryMarkerInfos__fixed[aux].empty() || context_.m_auxliaryMarkerRuntimes[aux].empty())
				{
					uCheck &= (uint)~(1 << ((int)MarkerType_AUX_BASE + aux));
					uTrackType &= (uint)~(1 << ((int)MarkerType_AUX_BASE + aux));
				}
			}

			{
				Point3 normal(0, 0, 1);
				for (int aux = 0; aux < auxnum; aux++)
				{
					uint uflag = (uint)(1 << ((int)MarkerType_AUX_BASE + aux));

					uPossibleTrackType |= uflag;
					int iCount = (int)m_auxliaryMarkerInfos__fixed[aux].size();
					for (int i = 0; i < iCount; i++)
					{
						for (int c = 0; c < m_auxliaryMarkerInfos__fixed[aux][i].m_corner3DNormals.size(); c++)
						{
							normal = computeRotation(cvMatRotation, m_auxliaryMarkerInfos__fixed[aux][i].m_corner3DNormals[c]);
							if (-normal.z <= m_parameters__fixed.auxMarkerTrackCosAngle)
							{
								uTrackType &= ~uflag;
								uCheck &= ~uflag;
								uPossibleTrackType &= ~uflag;
							}
						}
					}
				}
			}
			//if ((uTrackType & m_uMarkerMask__fixed) == 0)
			//	return false;

			float dCosLimit = 0.0f;
			if (dDistCam != 0.0f)
				dCosLimit = m_parameters__fixed.radius / dDistCam;

			int iStart = -2;
			int iEnd = iPatternCount + 1;
			//int iVerifyStart = 0;
			//int iVerifyEnd = 0;

			//bool bZCompare = p3DirCam.Z < 0.0 && d3LeftBound.Z < 0.0 && d3RightBound.Z < 0.0;
			//bool bZCompareSucceeded = false;

			if (dDistCam == 0.0)
			{
				iPatternCount = 0;
				iPatternIndex = -1;
				uTrackType &= ~(uint)MarkerTypeFlag_CYLINDER;
			}
			else
			{
				if (iPatternIndex >= 0)
				{
					Point3 markerDir;
					float dCosAngles[5];

					for (int i = -2; i <= iPatternCount + 1; i++)
					{
						int index = (iPatternIndex + iPatternNums + i) % iPatternNums;
						markerDir = p3AxisX * m_cylinderarkerInfos__fixed[index].m_corner3DNormals[0].x
							+ p3AxisY * m_cylinderarkerInfos__fixed[index].m_corner3DNormals[0].y;
						dCosAngles[i + 2] = markerDir.dot(p3DirCam);
					}

					for (iStart = -2; iStart <= iPatternCount + 1; iStart++)
					{
						if (dCosAngles[iStart + 2] > dCosLimit)
							break;
					}
					for (iEnd = iPatternCount + 1; iEnd >= -2; iEnd--)
					{
						if (dCosAngles[iEnd + 2] > dCosLimit)
							break;
					}
					if (iStart > iPatternCount + 1 || iEnd < -2 || iEnd - iStart < 2)
					{
						iPatternCount = 0;
						iPatternIndex = -1;
						uTrackType &= ~(uint)MarkerTypeFlag_CYLINDER;
					}
					else
					{
						while (iEnd - iStart > 2)
						{
							if (dCosAngles[iStart + 2] >= dCosAngles[iEnd + 2])
								iEnd--;
							else
								iStart++;
						}
					}
				}
				if (iPatternIndex < 0)
				{
					Point3 markerDir;
					float dMaxCosAngle = 0.0f;
					int iMaxIndex = -1;
					for (int i = 0; i < iPatternNums; i++)
					{
						markerDir = p3AxisX * m_cylinderarkerInfos__fixed[i].m_corner3DNormals[0].x + p3AxisY * m_cylinderarkerInfos__fixed[i].m_corner3DNormals[0].y;
						float dCosAngle = markerDir.dot(p3DirCam);
						if (iMaxIndex < 0 || dMaxCosAngle < dCosAngle)
						{
							iMaxIndex = i;
							dMaxCosAngle = dCosAngle;
						}
					}
					if (iMaxIndex == iPatternNums || dMaxCosAngle <= dCosLimit)
					{
						iPatternCount = 0;
						iPatternIndex = -1;
						uTrackType &= ~(uint)MarkerTypeFlag_CYLINDER;
					}
					else
					{
						iPatternIndex = iMaxIndex;
						iPatternCount = 1;
						iStart = -2;
						iEnd = iPatternCount + 1;
					}
				}
			}

			//if ((uTrackType & m_uMarkerMask__fixed) == 0)
			//	return false;


			Point2 projected(0, 0);
			Point2 projectedPrevCenter(0, 0);
			Point2 projectedCenter(0, 0);

			vecUpDownCode.clear();

			{
				float dMaxEdgeSqr = 0.0f;
				bool bBound = false;
				Point2 p2Min(0, 0);
				Point2 p2Max(0, 0);
				if (iPatternIndex >= 0)
				{
					int iValidCount = 0;
					int iValidLineCount = 0;
					int prev_index = -1;

					for (int i = iStart; i <= iEnd; i++)
					{
						int index = (iPatternIndex + iPatternNums + i) % iPatternNums;
						uint code = m_parameters__fixed.markerCodePatterns[index];

						bool bMajor = m_cylinderarkerInfos__fixed[index].m_bMajor;
						bool bBit = false;
						bool bLineCounted = false;
						int iPrevCodeType = -1;
						for (int c = 0; c <= iMarkerCodeLength; c++)
						{
							projected = computeProjection(cvMatProjection, m_cylinderarkerInfos__fixed[index].m_corner3DPoints[c]);
							//if (c < iMarkerCodeLength && i != iStart)
							//{
							//	computeProjection(cvMatProjection, m_cylinderarkerInfos__fixed[index].m_lcenter3DPoints[c], projectedCenter, cvMatTmp3x1_64F, cvMatTmp4x1_64F);
							//	context_.m_cylinderMarkerRuntimes[index].m_tmpLcenterImgPoints[c].x = projectedCenter.x;
							//	context_.m_cylinderMarkerRuntimes[index].m_tmpLcenterImgPoints[c].y = projectedCenter.y;
							//}
							Point2 diff = context_.m_cylinderMarkerRuntimes[index].m_tmpImgPoints[c] - projected;
							context_.m_cylinderMarkerRuntimes[index].m_tmpImgPoints[c] = projected;
							if ((uTrackType & (uint)MarkerTypeFlag_CYLINDER) != 0)
							{
								int err = context_.m_cylinderMarkerRuntimes[index].m_tmpImgPointsErr[c];
								if (err > 0)
								{
									if (std::max(std::abs(diff.x), std::abs(diff.y))
										< err + m_parameters__fixed.cornerPixelTolerance)
									{
										iValidCount++;
										if (bLineCounted == false)
										{
											bLineCounted = true;
											iValidLineCount++;
										}
									}
								}
								if (c > 0 && i != iStart)
								{
									if (bMajor)
									{
										bBit = ((code & 1) != 0) ? true : false;
										code >>= 1;
									}
									if (!calc2DSegmentsIntersection(context_.m_cylinderMarkerRuntimes[prev_index].m_tmpImgPoints[c - 1],
										context_.m_cylinderMarkerRuntimes[index].m_tmpImgPoints[c],
										context_.m_cylinderMarkerRuntimes[prev_index].m_tmpImgPoints[c],
										context_.m_cylinderMarkerRuntimes[index].m_tmpImgPoints[c - 1],
										/*out*/ projectedCenter))
									{
										uTrackType &= ~(uint)MarkerTypeFlag_CYLINDER;
									}
									else if (c > 1)
									{
										vecUpDownCode.clear();
										if (bMajor == true)
										{
											iPrevCodeType = -1;
											if (bBit)
											{
												vecUpDownCode.push_back((int)0xfffffffd);
												vecUpDownCode.push_back(0x1);
												vecUpDownCode.push_back(0x5);
											}
											else
											{
												vecUpDownCode.push_back(0x1);
											}
										}
										else
										{
											if (bBit)
											{
												if (iPrevCodeType == 0)
												{
													vecUpDownCode.push_back(((int)0xfffffffd));
												}
												else if (iPrevCodeType == 1)
												{
													vecUpDownCode.push_back(((int)0xfffffffa));
												}
												else if (iPrevCodeType == 2)
												{
													vecUpDownCode.push_back(((int)0xfffffffe));
												}
												else
												{
													vecUpDownCode.push_back(((int)0xfffffffd));
													vecUpDownCode.push_back(((int)0xfffffffa));
													vecUpDownCode.push_back(((int)0xfffffffe));
												}
											}
											else
											{
												vecUpDownCode.push_back(((int)0xfffffffe));
											}
										}

										iPrevCodeType = checkUpDown(cvMatGray, projectedPrevCenter, projectedCenter, vecUpDownCode, listByte, veciCodeBit, vecbCodeBitOK);
										if (iPrevCodeType < 0)
										{
											uTrackType &= ~(uint)MarkerTypeFlag_CYLINDER;
										}
									}

									projectedPrevCenter = projectedCenter;
									bMajor = !bMajor;
								}
							}

							context_.m_cylinderMarkerRuntimes[index].m_tmpImgPointsErr[c] = 0;
							if (bBound == false)
							{
								p2Max = p2Min = projected;
								bBound = true;
							}
							else
							{
								p2Min.x = std::min(p2Min.x, projected.x);
								p2Max.x = std::max(p2Max.x, projected.x);
								p2Min.y = std::min(p2Min.y, projected.y);
								p2Max.y = std::max(p2Max.y, projected.y);
							}
							if (c > 0)
							{
								float dsqr = DistanceSq(projected, context_.m_cylinderMarkerRuntimes[index].m_tmpImgPoints[c - 1]);
								dMaxEdgeSqr = std::max(dMaxEdgeSqr, dsqr);
							}
							if (prev_index >= 0)
							{
								float dsqr = DistanceSq(projected, context_.m_cylinderMarkerRuntimes[prev_index].m_tmpImgPoints[c]);
								dMaxEdgeSqr = std::max(dMaxEdgeSqr, dsqr);
							}
						}
						prev_index = index;
					}

					if (iValidCount < 4 || iValidLineCount <= 1)
					{
						uTrackType &= ~(uint)MarkerTypeFlag_CYLINDER;
					}
				}
				if (dCosLimit != 0.0f)
				{
					float dRadiusAngle = std::acos(dCosLimit);
					float dDirCamSin = p3DirCam.dot(p3AxisY);
					float dDirCamCos = p3DirCam.dot(p3AxisX);
					float dDirCamAngle = std::atan2(dDirCamSin, dDirCamCos);
					//Point3 d3DirOrtho = p3AxisX * (dDirCamSin) + p3AxisY * (-dDirCamCos);

					Point3 d3LT(m_parameters__fixed.radius * std::cos(dDirCamAngle + dRadiusAngle), m_parameters__fixed.radius * std::sin(dDirCamAngle + dRadiusAngle), 0);
					Point3 d3RT(m_parameters__fixed.radius * std::cos(dDirCamAngle - dRadiusAngle), m_parameters__fixed.radius * std::sin(dDirCamAngle - dRadiusAngle), 0);
					Point3 d3LB(m_parameters__fixed.radius * std::cos(dDirCamAngle + dRadiusAngle), m_parameters__fixed.radius * std::sin(dDirCamAngle + dRadiusAngle)
						, m_parameters__fixed.markerSize * m_parameters__fixed.markerCodeLength);
					Point3 d3RB(m_parameters__fixed.radius * std::cos(dDirCamAngle - dRadiusAngle), m_parameters__fixed.radius * std::sin(dDirCamAngle - dRadiusAngle)
						, m_parameters__fixed.markerSize * m_parameters__fixed.markerCodeLength);

					projected = computeProjection(cvMatProjection, d3LT);
					if (bBound == false)
					{
						bBound = true;
						p2Max.x = p2Min.x = projected.x;
						p2Min.y = p2Max.y = projected.y;
					}
					else
					{
						p2Min.x = std::min(p2Min.x, projected.x);
						p2Max.x = std::max(p2Max.x, projected.x);
						p2Min.y = std::min(p2Min.y, projected.y);
						p2Max.y = std::max(p2Max.y, projected.y);
					}

					projected = computeProjection(cvMatProjection, d3RT);
					p2Min.x = std::min(p2Min.x, projected.x);
					p2Max.x = std::max(p2Max.x, projected.x);
					p2Min.y = std::min(p2Min.y, projected.y);
					p2Max.y = std::max(p2Max.y, projected.y);

					projected = computeProjection(cvMatProjection, d3LB);
					p2Min.x = std::min(p2Min.x, projected.x);
					p2Max.x = std::max(p2Max.x, projected.x);
					p2Min.y = std::min(p2Min.y, projected.y);
					p2Max.y = std::max(p2Max.y, projected.y);

					projected = computeProjection(cvMatProjection, d3RB);
					p2Min.x = std::min(p2Min.x, projected.x);
					p2Max.x = std::max(p2Max.x, projected.x);
					p2Min.y = std::min(p2Min.y, projected.y);
					p2Max.y = std::max(p2Max.y, projected.y);
				}

				if (bBound == true)
				{
					listOutput[0].m_rect.x = (int)p2Min.x;
					listOutput[0].m_rect.y = (int)p2Min.y;
					listOutput[0].m_rect.width = (int)(p2Max.x + 0.5f) - listOutput[0].m_rect.x;
					listOutput[0].m_rect.height = (int)(p2Max.y + 0.5f) - listOutput[0].m_rect.y;
				}
				if (dMaxEdgeSqr > 0.0f)
				{
					listOutput[0].m_dMaxEdge = std::sqrt(dMaxEdgeSqr);
				}
			}

			for (int aux = 0; aux < auxnum; aux++)
			{
				uint uflag = (uint)(1 << ((int)MarkerType_AUX_BASE + aux));
				if ((uCheck & uflag) != 0)
				{
					int iValidCount = 0;
					int iValidLineCount = 0;
					int prev_index = -1;
					bool bBound = false;
					Point2 p2Min(0, 0);
					Point2 p2Max(0, 0);
					float dMaxEdgeSqr = 0.0f;

					int iAuxMarkerCodeLength = m_parameters__fixed.auxiliaryMarkerSettings[aux].checkerboardXLength;
					int iAuxLineNum = (int)m_parameters__fixed.auxiliaryMarkerSettings[aux].checkerboardCoordinates.size();

					switch (m_parameters__fixed.auxiliaryMarkerSettings[aux].type)
					{
					case AuxiliaryMarker::CHECKER:
					case AuxiliaryMarker::NOPATTERN:
						uint code = (m_parameters__fixed.auxiliaryMarkerSettings[aux].type == AuxiliaryMarker::CHECKER)
							? m_parameters__fixed.auxiliaryMarkerSettings[aux].code : 0;
						for (int index = 0; index < iAuxLineNum; index++)
						{
							bool bMajor = m_auxliaryMarkerInfos__fixed[aux][index].m_bMajor;
							bool bBit = false;
							int iLineCount = 0;
							int iPrevCodeType = -1;
							for (int c = 0; c <= iAuxMarkerCodeLength; c++)
							{
								projected = computeProjection(cvMatProjection, m_auxliaryMarkerInfos__fixed[aux][index].m_corner3DPoints[c]);

								//if (c < iAuxMarkerCodeLength && index != 0)
								//{
								//	computeProjection(cvMatProjection, m_auxliaryMarkerInfos__fixed[aux][index].m_lcenter3DPoints[c], projectedCenter, cvMatTmp3x1_64F, cvMatTmp4x1_64F);
								//	projectedCenter.CopyTo(context_.m_auxliaryMarkerRuntimes[aux][index].m_tmpLcenterImgPoints[c]);
								//}
								Point2 diff = context_.m_auxliaryMarkerRuntimes[aux][index].m_tmpImgPoints[c] - projected;
								context_.m_auxliaryMarkerRuntimes[aux][index].m_tmpImgPoints[c] = projected;
								if ((uTrackType & uflag) != 0)
								{
									int err = context_.m_auxliaryMarkerRuntimes[aux][index].m_tmpImgPointsErr[c];
									if (err > 0)
									{
										if (std::max(std::abs(diff.x), std::abs(diff.y))
											< err + m_parameters__fixed.cornerPixelTolerance)
										{
											iValidCount++;
											iLineCount++;
											if (iLineCount == 2)
											{
												iValidLineCount++;
											}
										}
									}
									if (c > 0 && index != 0)
									{
										if (bMajor)
										{
											bBit = ((code & 1) != 0) ? true : false;
											code >>= 1;
										}
										if (!calc2DSegmentsIntersection(context_.m_auxliaryMarkerRuntimes[aux][index - 1].m_tmpImgPoints[c - 1],
											context_.m_auxliaryMarkerRuntimes[aux][index].m_tmpImgPoints[c],
											context_.m_auxliaryMarkerRuntimes[aux][index - 1].m_tmpImgPoints[c],
											context_.m_auxliaryMarkerRuntimes[aux][index].m_tmpImgPoints[c - 1],
											/*out*/ projectedCenter))
										{
											uTrackType &= ~uflag;
										}
										else if (c > 1)
										{
											vecUpDownCode.clear();
											if (bMajor == true)
											{
												iPrevCodeType = -1;
												if (bBit)
												{
													vecUpDownCode.push_back(((int)0xfffffffd));
													vecUpDownCode.push_back(0x1);
													vecUpDownCode.push_back(0x5);
												}
												else
												{
													vecUpDownCode.push_back(0x1);
												}
											}
											else
											{
												if (bBit)
												{
													if (iPrevCodeType == 0)
													{
														vecUpDownCode.push_back(((int)0xfffffffd));
													}
													else if (iPrevCodeType == 1)
													{
														vecUpDownCode.push_back(((int)0xfffffffa));
													}
													else if (iPrevCodeType == 2)
													{
														vecUpDownCode.push_back(((int)0xfffffffe));
													}
													else
													{
														vecUpDownCode.push_back(((int)0xfffffffd));
														vecUpDownCode.push_back(((int)0xfffffffa));
														vecUpDownCode.push_back(((int)0xfffffffe));
													}
												}
												else
												{
													vecUpDownCode.push_back(((int)0xfffffffe));
												}
											}

											iPrevCodeType = checkUpDown(cvMatGray, projectedPrevCenter, projectedCenter, vecUpDownCode, listByte, veciCodeBit, vecbCodeBitOK);
											if (iPrevCodeType < 0)
											{
												uTrackType &= ~uflag;
											}
										}
										projectedPrevCenter.x = projectedCenter.x;
										projectedPrevCenter.y = projectedCenter.y;
										bMajor = !bMajor;
									}
								}
								context_.m_auxliaryMarkerRuntimes[aux][index].m_tmpImgPointsErr[c] = 0;
								if (bBound == false)
								{
									bBound = true;
									p2Max.x = p2Min.x = projected.x;
									p2Min.y = p2Max.y = projected.y;
								}
								else
								{
									p2Min.x = std::min(p2Min.x, projected.x);
									p2Max.x = std::max(p2Max.x, projected.x);
									p2Min.y = std::min(p2Min.y, projected.y);
									p2Max.y = std::max(p2Max.y, projected.y);
								}
								if (c > 0)
								{
									float dsqr = DistanceSq(projected, context_.m_auxliaryMarkerRuntimes[aux][index].m_tmpImgPoints[c - 1]);
									dMaxEdgeSqr = std::max(dMaxEdgeSqr, dsqr);
								}
								if (prev_index >= 0)
								{
									float dsqr = DistanceSq(projected, context_.m_auxliaryMarkerRuntimes[aux][prev_index].m_tmpImgPoints[c]);
									dMaxEdgeSqr = std::max(dMaxEdgeSqr, dsqr);
								}
							}
							prev_index = index;
						}
						break;
					}
					if (iValidCount < 4 || iValidLineCount <= 1)
					{
						uTrackType &= ~uflag;
					}
					if (bBound == true)
					{
						listOutput[1 + aux].m_rect.x = (int)p2Min.x;
						listOutput[1 + aux].m_rect.y = (int)p2Min.y;
						listOutput[1 + aux].m_rect.width = (int)(p2Max.x + 0.5f) - listOutput[1 + aux].m_rect.x;
						listOutput[1 + aux].m_rect.height = (int)(p2Max.y + 0.5f) - listOutput[1 + aux].m_rect.y;
					}
					if (dMaxEdgeSqr > 0.0f)
					{
						listOutput[1 + aux].m_dMaxEdge = std::sqrt(dMaxEdgeSqr);
					}
				}
			}


			bool bDetectValid = (uTrackType & m_uMarkerMask__fixed) != 0;
			bool bUpdate = eUpdate == CVRUPDATE_UPDATE || (eUpdate == CVRUPDATE_IF_VALID && bDetectValid == true);


			if (bUpdate)
			{
				if (iPatternIndex >= 0)
				{
					for (int i = iStart; i <= iEnd; i++)
					{
						int index = (iPatternIndex + iPatternNums + i) % iPatternNums;
						for (int c = 0; c <= iMarkerCodeLength; c++)
						{
							context_.m_cylinderMarkerRuntimes[index].m_prevImgPoints[c] = context_.m_cylinderMarkerRuntimes[index].m_tmpImgPoints[c];
							if (i != iStart && c != iMarkerCodeLength)
							{
								context_.m_cylinderMarkerRuntimes[index].m_prevLcenterImgPoints[c] = context_.m_cylinderMarkerRuntimes[index].m_tmpLcenterImgPoints[c];
							}
						}
					}

					iPatternIndex = (iPatternIndex + iPatternNums + iStart + 1) % iPatternNums;
				}

				for (int aux = 0; aux < auxnum; aux++)
				{
					int iIndexCount = (int)context_.m_auxliaryMarkerRuntimes[aux].size();
					int iAuxMarkerCodeLength = m_parameters__fixed.auxiliaryMarkerSettings[aux].checkerboardXLength;
					switch (m_parameters__fixed.auxiliaryMarkerSettings[aux].type)
					{
					case AuxiliaryMarker::CHECKER:
						for (int index = 0; index < iIndexCount; index++)
						{
							for (int c = 0; c <= iAuxMarkerCodeLength; c++)
							{
								context_.m_auxliaryMarkerRuntimes[aux][index].m_prevImgPoints[c] = context_.m_auxliaryMarkerRuntimes[aux][index].m_tmpImgPoints[c];

								if (index != 0 && c != iAuxMarkerCodeLength)
								{
									context_.m_auxliaryMarkerRuntimes[aux][index].m_prevLcenterImgPoints[c] = context_.m_auxliaryMarkerRuntimes[aux][index].m_tmpLcenterImgPoints[c];
								}
							}
						}
						break;
					case AuxiliaryMarker::NOPATTERN:
						for (int index = 0; index < iIndexCount; index++)
						{
							for (int c = 0; c <= iAuxMarkerCodeLength; c++)
							{
								context_.m_auxliaryMarkerRuntimes[aux][index].m_prevImgPoints[c] = context_.m_auxliaryMarkerRuntimes[aux][index].m_tmpImgPoints[c];
							}
						}
						break;
					}
				}

				dMaxEdge = 0.0f;
				bool bRectAll = false;
				for (const VerifyProjectionRet& output : listOutput)
				{
					dMaxEdge = std::max(dMaxEdge, output.m_dMaxEdge);
					if (output.m_rect.empty() == false)
					{
						if (bRectAll == false)
						{
							rectAll = output.m_rect;
							bRectAll = true;
						}
						else
						{
							rectAll = rectAll | output.m_rect;
						}
					}
				}
			}

			return bDetectValid;
		}



		static void composeTransformProjectionMatrix(cv::Mat& mrvec, cv::Mat& mtvec, cv::Mat& mcam, cv::Mat& rotation, cv::Mat& transform, cv::Mat& projection)
		{
			cv::Rodrigues(mrvec, rotation);
			cv::Mat src[2] = { rotation, mtvec };
			cv::hconcat(src, 2, transform);
			cv::gemm(mcam, transform, 1.0, 0, 0, projection);
		}

		//bool checkIntersection( Point a, Point b, Point c, Point d)
		//{
		//	double signdaba = (d.x - a.x) * (b.y - a.y) - (d.y - a.y) * (b.x - a.x);
		//	double signcaba = (c.x - a.x) * (b.y - a.y) - (c.y - a.y) * (b.x - a.x);
		//	if (false == (signdaba > 0.0 && signcaba > 0.0 || signdaba < 0.0 && signcaba < 0.0))
		//	{
		//		double signacdc = (a.x - c.x) * (d.y - c.y) - (a.y - c.y) * (d.x - c.x);
		//		double signbcdc = (b.x - c.x) * (d.y - c.y) - (b.y - c.y) * (d.x - c.x);
		//		if (false == (signacdc > 0.0 && signbcdc > 0.0 || signacdc < 0.0 && signbcdc < 0.0))
		//			return true;
		//	}
		//	return true;
		//}

		//bool checkIntersection( CvCBQuad a, Point[] b )
		//{
		//	for( int ai = 0; ai < 4; ai++ )
		//	{
		//		for( int bi = 0; bi < 4; bi++ )
		//		{
		//			if (checkIntersection(a.corners[ai].pt, a.corners[(ai + 1) % 4].pt, b[bi], b[(bi + 1) % 4]) )
		//				return true;
		//		}
		//	}
		//	bool bInner = true;
		//	for( int ai = 0; ai < 4; ai++ )
		//	{
		//		if (!checkLineInner(a.corners[ai].pt, a.corners[(ai + 1) % 4].pt, b[0]))
		//		{
		//			bInner = false;
		//			break;
		//		}
		//	}
		//	if ( bInner == false )
		//	{
		//		for( int bi = 0; bi < 4; bi++ )
		//		{
		//			if (!checkLineInner(b[bi], b[(bi + 1) % 4], a.corners[0].pt))
		//				return false;
		//		}
		//	}
		//	return true;
		//}

		//int estimateDirection(CvCBQuad a, Point[] b, double dRatio, /*out*/double dbestdotsum)
		//{
		//	int ibestoffset = -1;
		//	dbestdotsum = 0.0;
		//	double dInvRatio = 1.0 / dRatio;
		//	double dRatioMin = std::min(dRatio, dInvRatio);
		//	double dRatioMax = std::max(dRatio, dInvRatio);
		//	for( int iOffset = 0; iOffset < 4; iOffset++ )
		//	{
		//		double ddotsum = 0.0;
		//		for ( int j = 0; j < 4; j++ )
		//		{
		//			int i = (iOffset + j) % 4;
		//			int i_ = (iOffset + j + 1) % 4;
		//			int j_ = (j + 1) % 4;
		//			Point adir = a.corners[i_].pt - a.corners[i].pt;
		//			Point bdir = b[j_] - b[j];
		//			double alen = normalize(adir);
		//			double blen = normalize(bdir);
		//			if (alen <= 0.0 || blen <= 0.0)
		//			{
		//				ddotsum = 0.0;
		//				break;
		//			}
		//			double r = (alen < blen) ? alen / blen : blen / alen;
		//			if (r < dRatioMin || r > dRatioMax)
		//			{
		//				ddotsum = 0.0;
		//				break;
		//			}
		//			double dot = adir.dot(bdir);
		//			if (dot <= 0.0)
		//			{
		//				ddotsum = 0.0;
		//				break;
		//			}
		//			ddotsum += dot * r;
		//		}
		//		if ( ddotsum > dbestdotsum )
		//		{
		//			ibestoffset = iOffset;
		//			dbestdotsum = ddotsum;
		//		}
		//	}

		//	return ibestoffset;
		//}

		bool detectMarkerNew__detect(WorkContext& context_, cv::Mat& curGray_, cv::Mat& originalGray_, cv::Mat& curColor_, IUnknown* pSpatialCoordinate, int64_t i64Timestamp
			, Vector3& position_, Quaternion& orientation_)
		{
			position_ = Vector3(0, 0, 0);
			orientation_ = Quaternion(0, 0, 0, 1);
			//rotVec_ = Vector3(0, 0, 0);

			if (curGray_.empty() || context_.m_uExternalTrackType == 0)
				return false;

			double xScale = (double)originalGray_.cols / (double)curGray_.cols;
			double yScale = (double)originalGray_.rows / (double)curGray_.rows;

			bool bLastDetectPoseValid = false;
			bool bTryValid = false;
			//uint uAuxBlockMarker = context_.m_uExternalTrackType & m_uAuxNoPatternMarkerMask__fixed;

			Vector3 lastDetectPosition = Vector3(0, 0, 0);
			Quaternion lastDetectOrientation = Quaternion(0, 0, 0, 1);
			//Vector3 lastDetectRotVec = Vector3(0, 0, 0);

			std::vector<Point2>& imgPointsRuntime = m_imgPointsRuntime__detect;
			std::vector<Point3>& modelPointsRuntime = m_modelPointsRuntime__detect;
			imgPointsRuntime.clear();
			modelPointsRuntime.clear();

			//int max_dilation_run_ID = -1;
			int width = curGray_.cols;
			int height = curGray_.rows;
			//int max_count = 0;

			int whmax = std::max((int)(std::max(width, height) * m_parameters__fixed.expandAreaRatio), 1);

			OpenCVRect rect = OpenCVRect(0, 0, width, height);

			if (pSpatialCoordinate != nullptr && m_locateAtTimestampSpatialCB != nullptr)
			{
				BoundPoints boundPoints;

				int iNewBoundStamp = m_iBoundPointsStamp__mutex;
				if ( iNewBoundStamp != m_iBoundPointStamp__detect
					&& m_mutexNewBoundPoints__mutex.try_lock() == true )
				{
					boundPoints = m_newBoundPoints__mutex;
					m_iBoundPointStamp__detect = m_iBoundPointsStamp__mutex;
					m_mutexNewBoundPoints__mutex.unlock();

					if (m_iBoundPointStamp__detect > context_.m_i64PrevImgBoundTimestamp)
					{
						LocateAtTimestamp lat;
						lat.m_i64Timestamp = i64Timestamp;
						lat.m_pCoordinateSystem = pSpatialCoordinate;
						if ((*m_locateAtTimestampSpatialCB)(lat))
						{
							double init[16];
							cv::Mat matTransform(4, 4, CV_64FC1, init);
							setTransformMat(matTransform, lat.m_position, lat.m_orientation);

							Point2 p2Min;
							Point2 p2Max;
							Point2 p;
							Point3 p3;


							for (int i = 0; i < 8; i++)
							{
								p3 = boundPoints.m_aPos[i];
								p3.y = -p3.y;
								p3.z = -p3.z;
								p3 = computeTransform(matTransform, boundPoints.m_aPos[i]);
								p3.x = -p3.x;
								p3.z = -p3.z;
								p = computeCameraProjection(context_.m_matCamera, p3);
								if (i == 0)
								{
									p2Min = p2Max = p;
								}
								else
								{
									p2Min.x = std::min(p2Min.x, p.x);
									p2Min.y = std::min(p2Min.y, p.y);
									p2Max.x = std::max(p2Max.x, p.x);
									p2Max.y = std::max(p2Max.y, p.y);
								}
							}

							context_.m_prevImgBound.x = (int)p2Min.x - m_parameters__fixed.expandAreaMargin;
							context_.m_prevImgBound.y = (int)p2Min.y - m_parameters__fixed.expandAreaMargin;
							context_.m_prevImgBound.width = (int)(p2Max.x + m_parameters__fixed.expandAreaMargin  - context_.m_prevImgBound.x + 1);
							context_.m_prevImgBound.height = (int)(p2Max.y + m_parameters__fixed.expandAreaMargin  - context_.m_prevImgBound.y + 1);

							if (m_setBoundRectCB != nullptr)
							{
								(*m_setBoundRectCB)(context_.m_prevImgBound);
							}

							context_.m_iPrevDetectFailCount = 0;
						}
					}
				}
			}

			if (context_.m_prevImgBound.empty() == false)
			{
				int x0 = context_.m_prevImgBound.x;
				int y0 = context_.m_prevImgBound.y;
				int x1 = context_.m_prevImgBound.x + context_.m_prevImgBound.width;
				int y1 = context_.m_prevImgBound.y + context_.m_prevImgBound.height;

				int x;
				int y;
				int x_1;
				int y_1;

				x0 -= whmax * context_.m_iPrevDetectFailCount;
				y0 -= whmax * context_.m_iPrevDetectFailCount;
				x1 += whmax * context_.m_iPrevDetectFailCount;
				y1 += whmax * context_.m_iPrevDetectFailCount;

				x0 = std::max(0, x0);
				y0 = std::max(0, y0);
				x1 = std::min(width, x1);
				y1 = std::min(height, y1);
				context_.m_prevImgBound.x = x0;
				context_.m_prevImgBound.y = y0;
				context_.m_prevImgBound.width = x1 - x0;
				context_.m_prevImgBound.height = y1 - y0;

				x = x0;
				y = y0;
				x_1 = x1;
				y_1 = y1;
				if (context_.m_vecEyeGaze.empty() == false)
				{
					int centerX = width * context_.m_vecEyeGaze[0];
					int centerY = height * context_.m_vecEyeGaze[1];

					int iHalfWidth = (x_1 - x) / 2;
					int iHalfHeight = (y_1 - y) / 2;

					x = centerX - iHalfWidth;
					y = centerY - iHalfHeight;
					x_1 = centerX + iHalfWidth;
					y_1 = centerY + iHalfHeight;
				}

				context_.m_prevImgBound.x = x0;
				context_.m_prevImgBound.y = y0;
				context_.m_prevImgBound.width = x1 - x0;
				context_.m_prevImgBound.height = y1 - y0;

				x = std::max(0, x);
				y = std::max(0, y);
				x_1 = std::min(width, x_1);
				y_1 = std::min(height, y_1);

				if (context_.m_prevImgBound.x == 0 && context_.m_prevImgBound.y == 0 && context_.m_prevImgBound.width == width && context_.m_prevImgBound.height == height
					|| x == 0 && y == 0 && x_1 == width && y_1 == height)
				{
					context_.m_prevImgBound.width = 0;
					context_.m_prevImgBound.height = 0;
					context_.m_i64PrevImgBoundTimestamp = 0;
				}
				else if (x_1 - x > 0 && y_1 - y > 0)
				{
					rect.x = x;
					rect.y = y;
					rect.width = x_1 - x;
					rect.height = y_1 - y;
				}
			}


			//Mat thresh_img = new Mat(height, width, CV_8UC1);
#ifndef USE_LIBCBDETECTOR
			if ( m_aCurThresholded__detect[0].cols != width || m_aCurThresholded__detect[0].rows != height
				|| m_aCurThresholded__detect[0].type() != CV_8UC1)
			{
				m_aCurThresholded__detect[0].create(height, width, CV_8UC1);
			}
			if (m_aCurThresholded__detect[1].cols != width || m_aCurThresholded__detect[1].rows != height
				|| m_aCurThresholded__detect[1].type() != CV_8UC1)
			{
				m_aCurThresholded__detect[1].create(height, width, CV_8UC1);
			}
#endif USE_LIBCBDETECTOR

			cv::Mat rectGray;
			cv::Mat rectColor;
#ifndef USE_LIBCBDETECTOR
			cv::Mat aRectThresholded[2];
#endif USE_LIBCBDETECTOR
			if (rect.x == 0 && rect.y == 0 && rect.width == width && rect.height == height)
			{
				rectGray = curGray_;
				rectColor = curColor_;
#ifndef USE_LIBCBDETECTOR
				aRectThresholded[0] = m_aCurThresholded__detect[0];
				aRectThresholded[1] = m_aCurThresholded__detect[1];
#endif USE_LIBCBDETECTOR
			}
			else
			{
				rectGray = curGray_(rect);
				if (curColor_.empty() == false)
				{
					rectColor = curColor_(rect);
				}
#ifndef USE_LIBCBDETECTOR
				aRectThresholded[0] = m_aCurThresholded__detect[0](rect);
				aRectThresholded[1] = m_aCurThresholded__detect[1](rect);
#endif USE_LIBCBDETECTOR
			}

			std::vector<CvCBQuad>& quads = m_quadsBase__detect;
			std::vector<CvCBCorner>& corners = m_cornersBase__detect;
			std::vector<CvCBQuad*>& quad_group = m_quadGroup__detect;
			quads.clear();
			corners.clear();
			quad_group.clear();


			//List<CvCBQuad> output_quad_group = new List<CvCBQuad>();
			double adTransform[12];
			double adProjection[12];
			double adRotation[9];
			double adrvec[3];
			double adtvec[3];
			cv::Mat matTransform(3, 4, CV_64FC1, adTransform);
			cv::Mat matProjection(3, 4, CV_64FC1, adProjection);
			cv::Mat matRotation(3, 3, CV_64FC1, adRotation);
			cv::Mat mrvec(3, 1, CV_64FC1, adrvec);
			cv::Mat mtvec(3, 1, CV_64FC1, adtvec);
			{

				int iPatternNums = (int)m_cylinderarkerInfos__fixed.size();


#ifdef USE_LIBCBDETECTOR

				cv::Mat rectGrayResized;
				cbdetect::find_corners(rectGray, m_corner__detect, rectGrayResized, m_cornerResized__detect, m_cbdetectParams__fixed);
				cbdetect::boards_from_corners(m_corner__detect, m_boards__detect, m_cbdetectParams__fixed);
				cbdetect::mark_board(rectGray, m_corner__detect, m_boards__detect);


#else  USE_LIBCBDETECTOR

				int iAdaptiveBlockSize = 0;
				if (context_.m_lastAdaptiveBlockSize == 0)
				{
					if ((context_.m_adaptiveBlockSizeCounter & 1) == 0)
						iAdaptiveBlockSize = (int)std::round(std::min(width, height) * m_parameters__fixed.adaptiveThresholdBlockRatio2);
					else
						iAdaptiveBlockSize = (int)std::round(std::min(width, height) * m_parameters__fixed.adaptiveThresholdBlockRatio1);
				}
				else
				{
					if ((context_.m_adaptiveBlockSizeCounter & 2) == 0)
					{
						iAdaptiveBlockSize = std::min(context_.m_lastAdaptiveBlockSize, std::min(width, height));
					}
					else
					{
						if ((context_.m_adaptiveBlockSizeCounter & 1) == 0)
							iAdaptiveBlockSize = (int)std::round(std::min(width, height) * m_parameters__fixed.adaptiveThresholdBlockRatio2);
						else
							iAdaptiveBlockSize = (int)std::round(std::min(width, height) * m_parameters__fixed.adaptiveThresholdBlockRatio1);
					}
				}
				context_.m_adaptiveBlockSizeCounter++;

				//Profiler.BeginSample("AdaptiveThreshold");

				int iThresholdIndex = 0;
				cv::adaptiveThreshold(rectGray, aRectThresholded[iThresholdIndex], 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, iAdaptiveBlockSize | 1
					, m_parameters__fixed.adaptiveThresholdC);

				/*f (m_bSaveImage == true )
				{
					m_bSaveImage = false;

					if (m_strSavePath__fixed.empty() == false)
					{
						static int s_iCount = 0;
						s_iCount++;
						char szPath[MAX_PATH];
						int iLength = (int)m_strSavePath__fixed.length();
						strncpy(szPath, m_strSavePath__fixed.c_str(), iLength);
						if (szPath[iLength - 1] != '/' && szPath[iLength] != '\\')
						{
							szPath[iLength] = '/';
							iLength++;
						};
						snprintf(&szPath[iLength], MAX_PATH - iLength, "img%d.bmp", s_iCount);
						cv::imwrite(szPath, m_aCurThresholded__detect[iThresholdIndex]);
					}
				}*/





				//Profiler.EndSample();



				//Profiler.BeginSample("Dilation");

				int dilations = 0;
				for (; dilations < m_parameters__fixed.minDilations; dilations++)
				{
					if (dilations > 0)
					{
						if ((dilations & 1) == 1)
							cv::erode(aRectThresholded[iThresholdIndex], aRectThresholded[1 - iThresholdIndex], m_matCrossKernel__fixed);
						else
							cv::erode(aRectThresholded[iThresholdIndex], aRectThresholded[1 - iThresholdIndex], m_matRectKernel__fixed);
						iThresholdIndex = 1 - iThresholdIndex;
					}
				}

				//Profiler.EndSample();

				{
					std::vector<std::vector<Point2i>> scaleContours;
					std::vector<cv::Vec4i>& hierarchy = m_hierarchy__detect;


					for (; dilations <= m_parameters__fixed.maxDilations; dilations++)
					{
						//Profiler.BeginSample("Dilation");

						if (dilations > 0)
						{
							if ((dilations & 1) == 1)
								cv::erode(aRectThresholded[iThresholdIndex], aRectThresholded[1 - iThresholdIndex], m_matCrossKernel__fixed);
							else
								cv::erode(aRectThresholded[iThresholdIndex], aRectThresholded[1 - iThresholdIndex], m_matRectKernel__fixed);
							iThresholdIndex = 1 - iThresholdIndex;

						}
						//m_aCurThresholded__detect.CopyTo(thresh_img);
						//Imgproc.rectangle(thresh_img, rect, new Scalar(0), 3, 8);

						//Profiler.EndSample();

						if (dilations == m_parameters__fixed.maxDilations && m_updateResultGrayCB != nullptr)
						{
							m_updateResultGrayCB(&m_aCurThresholded__detect[iThresholdIndex]);
						}


						scaleContours.clear();
						hierarchy.clear();
						cv::Mat& matImage = aRectThresholded[iThresholdIndex];
						cv::findContours(matImage, /*out*/scaleContours, /*out*/ hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
						if (scaleContours.empty() || hierarchy.empty() || hierarchy.size() != scaleContours.size())
							continue;
						int w = matImage.cols;
						int h = matImage.rows;
						generateQuads__detect(scaleContours, hierarchy, quads, corners, rect, w, h, dilations, curColor_, true);

					}
				}
				for (int i = 0; i < (int)quads.size(); i++)
				{
					for (int c = 0; c < 4; c++)
						quads[i].corners[c] = (CvCBCorner*)((char*)(&corners.front()) + (std::ptrdiff_t)(quads[i].corners[c]));
				}




				//if (rectColor != null && aRectThresholded[iThresholdIndex] != null)
				//{
				//	Imgproc.cvtColor(aRectThresholded[iThresholdIndex], rectColor, Imgproc.COLOR_GRAY2RGBA);
				//}



				//int quad_count = quads.size();
				//if (quad_count < m_parameters__fixed.markerCodeLength)
				//{
				//	quads.Clear();
				//	dialation_quads.Clear();
				//	quad_group.Clear();
				//	if (quad_singles != null)
				//		quad_singles.Clear();
				//	context_.m_iPrevDetectFailCount++;
				//	return false;
				//}


				//Profiler.BeginSample("findQuadNeighbors__detect");

				findQuadNeighbors__detect(quads);

				//(Profiler.EndSample();


				// The connected quads will be organized in groups. The following loop
				// increases a "group_idx" identifier.
				// The function "icvFindConnectedQuads assigns all connected quads
				// a unique group ID.
				// If more quadrangles were assigned to a given group (i.e. connected)
				// than are expected by the input variable "pattern_size", the 
				// function "icvCleanFoundConnectedQuads" erases the surplus
				// quadrangles by minimizing the convex hull of the remaining pattern->

#endif USE_LIBCBDETECTOR

				int iNextQuadIndex = 0;

				//Profiler.BeginSample("findConnectedQuads__detect");

				//List<SinglePattern> foundPattern = m_foundPatternsRuntime__detect;
				for (SinglePattern& pattern : m_singlePatternBase__detect)
				{
					pattern.Reset();
				}
				std::vector<SinglePattern*>& foundPattern = m_foundPattern__detect;
				foundPattern.clear();

#ifdef USE_LIBCBDETECTOR

				convertCorners(m_corner__detect, corners, rect);
				CvCBCorner& cdummy = corners.back();
				int iNumGroups = (int) m_boards__detect.size();
				{
					int iQuadOffset = 0;

					for (int i = 0; i < iNumGroups; i++)
					{
						const cbdetect::Board& board = m_boards__detect[i];
						int rows = board.rows() - 3;
						int cols = board.cols() - 3;
						if (rows > 0 && cols > 0 && board.getBoardType() != cbdetect::BoardTypeInvalid )
						{
							iQuadOffset += rows * cols;
						}
					}
					quads.resize(0);
					quads.resize(iQuadOffset);
					iQuadOffset = 0;

					for (int i = 0; i < iNumGroups; i++)
					{
						const cbdetect::Board& board = m_boards__detect[i];
						int rows = board.rows() - 3;
						int cols = board.cols() - 3;
						if ( false == (rows > 0 && cols > 0 && board.getBoardType() != cbdetect::BoardTypeInvalid) )
							continue;

						for (int r = 0; r < rows; r++)
						{
							for (int c = 0; c < cols; c++)
							{
								CvCBQuad& quad = quads[iQuadOffset + cols * r + c];
								quad.count = 0;
								quad.group_idx = i;
								quad.checker_type = board.type(r + 1, c + 1);
								int i00 = board.idx(r + 1, c + 1);
								int i01 = board.idx(r + 1, c + 2);
								int i11 = board.idx(r + 2, c + 2);
								int i10 = board.idx(r + 2, c + 1);
								int ccount = 0;
								if (i00 >= 0)
								{
									CvCBCorner* pCorner = &corners[i00];
									quad.corners[0] = pCorner;
									ccount++;
								}
								else
								{
									quad.corners[0] = &cdummy;
								}
								if (i01 >= 0)
								{
									CvCBCorner* pCorner = &corners[i01];
									quad.corners[1] = pCorner;
									ccount++;
								}
								else
								{
									quad.corners[1] = &cdummy;
								}
								if (i11 >= 0)
								{
									CvCBCorner* pCorner = &corners[i11];
									quad.corners[2] = pCorner;
									ccount++;
								}
								else
								{
									quad.corners[2] = &cdummy;
								}
								if (i10 >= 0)
								{
									CvCBCorner* pCorner = &corners[i10];
									quad.corners[3] = pCorner;
									ccount++;
								}
								else
								{
									quad.corners[3] = &cdummy;
								}
								if (ccount == 4)
								{
									float bound_min_x = 0;
									float bound_min_y = 0;
									float bound_max_x = 0;
									float bound_max_y = 0;

									for (int c = 0; c < 4; c++)
									{
										float d = DistanceSq(quad.corners[c]->pt, quad.corners[(c + 1) & 3]->pt);
										if (c == 0 || quad.edge_len_sqr > d)
											quad.edge_len_sqr = d;
										if (c == 0 || quad.edge_max_sqr < d)
											quad.edge_max_sqr = d;
										if (c == 0)
										{
											bound_min_x = bound_max_x = quad.corners[c]->pt.x;
											bound_min_y = bound_max_y = quad.corners[c]->pt.y;
										}
										else
										{
											bound_min_x = std::min(bound_min_x, quad.corners[c]->pt.x);
											bound_min_y = std::min(bound_min_y, quad.corners[c]->pt.y);
											bound_max_x = std::max(bound_max_x, quad.corners[c]->pt.x);
											bound_max_y = std::max(bound_max_y, quad.corners[c]->pt.y);
										}
										switch (c)
										{
										case 0:
											quad.max_width_sqr = d;
											break;
										case 1:
											quad.max_height_sqr = d;
											break;
										case 2:
											quad.max_width_sqr = std::max(quad.max_width_sqr, d);
											break;
										case 3:
											quad.max_height_sqr = std::max(quad.max_height_sqr, d);
											break;
										}
									}
									quad.bound.x = (int)bound_min_x;
									quad.bound.y = (int)bound_min_y;
									quad.bound.width = ((int)bound_max_x) - quad.bound.x + 1;
									quad.bound.height = ((int)bound_max_y) - quad.bound.y + 1;
								}
								else
								{
									quad.checker_type = cbdetect::CheckerInvalid;
								}
							}
						}

						iQuadOffset += rows * cols;
					}


					iQuadOffset = 0;
					for (int i = 0; i < iNumGroups; i++)
					{
						const cbdetect::Board& board = m_boards__detect[i];
						int rows = board.rows() - 3;
						int cols = board.cols() - 3;
						cbdetect::BoardType eBoardType = board.getBoardType();
						if (false == (rows > 0 && cols > 0 && eBoardType != cbdetect::BoardTypeInvalid))
							continue;

						labelQuadGroup2__detect(context_, quads.data() + iQuadOffset, rows, cols, eBoardType, foundPattern, context_.m_uExternalTrackType);

						iQuadOffset += rows * cols;
					}
				}


#else  USE_LIBCBDETECTOR

				for (int group_idx = 0; ; group_idx++)
				{

					iNextQuadIndex = findConnectedQuads__detect(quads, iNextQuadIndex, quad_group, group_idx, curColor_);
					int count = (int)quad_group.size();
					if (count == 0)
						break;

					// 고쳐야 함
					//if (count < m_parameters__fixed.markerCodeLength )
					if (count < m_iCheckerBoardMinBlocks__fixed)
						continue;

					//cleanFoundConnectedQuads(quad_group);

						// MARTIN's Code
						// To save computational time, only proceed, if the number of
						// found quads during this dilation run is larger than the 
						// largest previous found number
						//if (count >= max_count)

					labelQuadGroup__detect(context_, quad_group, foundPattern, context_.m_uExternalTrackType);



					//// The following function labels all corners of every quad 
					//// with a row and column entry.
					//// "count" specifies the number of found quads in "quad_group"
					//// with group identifier "group_idx"
					//// The last parameter is set to "true", because this is the
					//// first function call and some initializations need to be
					//// made.
					//mrLabelQuadGroup(quad_group, max_count, pattern_size, true);

					////END------------------------------------------------------------------------


					//// Allocate memory
					//CV_CALL(output_quad_group = (CvCBQuad**)cvAlloc(sizeof(output_quad_group[0]) * ((pattern_size.height + 2) * (pattern_size.width + 2))));


					//// The following function copies every member of "quad_group"
					//// to "output_quad_group", because "quad_group" will be 
					//// overwritten during the next loop pass.
					//// "output_quad_group" is a true copy of "quad_group" and 
					//// later used for output
					//mrCopyQuadGroup(quad_group, output_quad_group, max_count);

				}



				//if (output_quad_group.size() > 0)
				//{

				//	for (int i = 0; i < output_quad_group.size(); i++)
				//	{
				//		CvCBQuad q = output_quad_group[i];
				//		Point p = new Point(0, 0);
				//		for (int j = 0; j < 4; j++)
				//		{
				//			if ( j == 0 )
				//			{
				//				p.x = q.corners[j].pt.x;
				//				p.y = q.corners[j].pt.y;
				//			}
				//			else
				//			{
				//				p.x = std::min(q.corners[j].pt.x, p.x);
				//				p.y = std::min(q.corners[j].pt.y, p.y);
				//			}
				//		}
				//		p.x = (int)p.x;
				//		p.y = (int)p.y;
				//		string coord = string.Format("{0}", q.bit ? 1 : 0);
				//		int[] baseLine = new int[1];
				//		Size size = Imgproc.getTextSize(coord, Imgproc.FONT_HERSHEY_SCRIPT_SIMPLEX, 0.8, 1, baseLine);

				//		p.x += (int)size.width;
				//		p.y += (int) size.height;
				//		Imgproc.putText(m_curColor__track, coord, p, Imgproc.FONT_HERSHEY_SCRIPT_SIMPLEX, 0.8, new Scalar(255, 255, 0, 255));
				//	}
				//}

#endif USE_LIBCBDETECTOR

				std::vector<SinglePattern*>& markerPatterns = m_markerPatternsRuntime__detect;
				int numaux = (int)m_parameters__fixed.auxiliaryMarkerSettings.size();
				markerPatterns.clear();
				markerPatterns.resize(1 + numaux, nullptr);

				bool bFindMarker = false;

				if (foundPattern.size() > 0)
				{
					bool bFindAuxMarker = false;
					for (int pindex = 0; pindex < foundPattern.size(); pindex++)
					{
						SinglePattern* pattern = foundPattern[pindex];
						uint flag = (uint)(1 << pattern->pattern_type);
						if (((context_.m_uExternalTrackType & (uint)MarkerTypeFlag_CYLINDER) & flag) == 0)
						{
							if (((context_.m_uExternalTrackType & (m_uAuxPatternMarkerMask__fixed | m_uAuxNoPatternMarkerMask__fixed)) & flag) != 0)
							{
								bFindAuxMarker = true;
							}
							continue;
						}

						if (pattern->pattern_type == (int)MarkerType_CYLINDER)
						{
							if (markerPatterns[pattern->pattern_type] == nullptr
								|| pattern->pattern_count > markerPatterns[pattern->pattern_type]->pattern_count
								|| pattern->pattern_count == markerPatterns[pattern->pattern_type]->pattern_count && pattern->min_max_wh_sqr.min_width_sqr > markerPatterns[pattern->pattern_type]->min_max_wh_sqr.min_width_sqr)
							{
								markerPatterns[pattern->pattern_type] = pattern;
								bFindMarker = true;
							}
						}
						else
						{
							if (markerPatterns[pattern->pattern_type] == nullptr
								|| pattern->min_max_wh_sqr.MinSqr() > markerPatterns[pattern->pattern_type]->min_max_wh_sqr.MinSqr() )
							{
								markerPatterns[pattern->pattern_type] = pattern;
								bFindMarker = true;
							}
						}
					}

					if (markerPatterns[(int)MarkerType_CYLINDER] != nullptr)
					{
						simplifyCylinderPattern(markerPatterns[(int)MarkerType_CYLINDER]);
					}

					if (bFindAuxMarker == true)
					{
						if (markerPatterns[(int)MarkerType_CYLINDER] != nullptr)
						{
							Point2 dirCylCode(0, 0);
							Point2 dirCylHeight(0, 0);
							Point3 dirCylNormal(0, 0, 0);
							if (estimatePatternDirection(markerPatterns[(int)MarkerType_CYLINDER], m_cylinderarkerInfos__fixed,
								m_parameters__fixed.markerCodeLength, markerPatterns[(int)MarkerType_CYLINDER]->pattern_index,
								true, /*out*/dirCylCode, /*out*/dirCylHeight, /*out*/dirCylNormal))
							{
								for (int pindex = 0; pindex < foundPattern.size(); pindex++)
								{
									SinglePattern* pattern = foundPattern[pindex];
									uint flag = (uint)(1 << pattern->pattern_type);
									if (((context_.m_uExternalTrackType & (m_uAuxPatternMarkerMask__fixed| m_uAuxNoPatternMarkerMask__fixed)) & flag) == 0)
										continue;

									Point2 dirCode(0, 0);
									Point2 dirHeight(0, 0);
									Point3 dirNormal(0, 0, 0);

									if (estimatePatternDirection(pattern, m_auxliaryMarkerInfos__fixed[pattern->pattern_type - (int)MarkerType_AUX_BASE],
										m_parameters__fixed.auxiliaryMarkerSettings[pattern->pattern_type - (int)MarkerType_AUX_BASE].checkerboardXLength,
										m_parameters__fixed.auxiliaryMarkerSettings[pattern->pattern_type - (int)MarkerType_AUX_BASE].checkerboardCenterCoordinate,
										false, /*out*/dirCode, /*out*/dirHeight, /*out*/dirNormal))
									{
										if (dirCode.dot(dirCylHeight) > 0.0f && dirHeight.dot(dirCylCode) < 0.0f && dirCylNormal.dot(dirNormal) > 0.0f)
										{
											if (markerPatterns[pattern->pattern_type] == nullptr
												|| pattern->min_max_wh_sqr.MinSqr() > markerPatterns[pattern->pattern_type]->min_max_wh_sqr.MinSqr())
											{
												markerPatterns[pattern->pattern_type] = pattern;
												bFindMarker = true;
											}
										}
									}
								}
							}
						}
						else if ((context_.m_uTrackType & (uint)MarkerTypeFlag_CYLINDER) != 0
							&& context_.m_iPrevMarkerCount == 1 && context_.m_iPrevMarkerIndex >= 0 && context_.m_iPrevMarkerIndex < m_cylinderarkerInfos__fixed.size())
						{
							Point2 dirCylCode(0, 0);
							Point2 dirCylHeight(0, 0);
							Point3 dirCylNormal(0, 0, 0);
							if (estimateCylinderDirection(context_, /*out*/dirCylCode, /*out*/dirCylHeight, /*out*/dirCylNormal))
							{
								for (int pindex = 0; pindex < foundPattern.size(); pindex++)
								{
									SinglePattern* pattern = foundPattern[pindex];
									uint flag = (uint)(1 << pattern->pattern_type);
									if (((context_.m_uExternalTrackType & (m_uAuxPatternMarkerMask__fixed | m_uAuxNoPatternMarkerMask__fixed)) & flag) == 0)
										continue;

									Point2 dirCode(0, 0);
									Point2 dirHeight(0, 0);
									Point3 dirNormal(0, 0, 0);

									if (estimatePatternDirection(pattern, m_auxliaryMarkerInfos__fixed[pattern->pattern_type - (int)MarkerType_AUX_BASE],
										m_parameters__fixed.auxiliaryMarkerSettings[pattern->pattern_type - (int)MarkerType_AUX_BASE].checkerboardXLength,
										m_parameters__fixed.auxiliaryMarkerSettings[pattern->pattern_type - (int)MarkerType_AUX_BASE].checkerboardCenterCoordinate,
										false, /*out*/dirCode, /*out*/dirHeight, /*out*/dirNormal))
									{
										if (dirCode.dot(dirCylHeight) > 0.0f && dirHeight.dot(dirCylCode) < 0.0f && dirCylNormal.dot(dirNormal) > 0.0f)
										{
											if (markerPatterns[pattern->pattern_type] == nullptr
												|| pattern->min_max_wh_sqr.MinSqr() > markerPatterns[pattern->pattern_type]->min_max_wh_sqr.MinSqr())
											{
												markerPatterns[pattern->pattern_type] = pattern;
												bFindMarker = true;
											}
										}
									}
								}
							}
						}
					}
				}

				if (bFindMarker == true)
				{
					//List<Point> imgPoints = new List<Point>();
					//List<Point3> modelPoints = new List<Point3>();
					//List<int> pointsErr = new List<int>();
					//List<bool> pointsCross = new List<bool>();

					bool bSecondTry = false;

					if (markerPatterns[(int)MarkerType_CYLINDER] != nullptr)
					{
						update2Dcollect3DPoints(m_parameters__fixed.markerCodeLength, markerPatterns[(int)MarkerType_CYLINDER], modelPointsRuntime, m_cylinderarkerInfos__fixed, context_.m_cylinderMarkerRuntimes);
						refineCyclicCorners(originalGray_, markerPatterns[(int)MarkerType_CYLINDER]->pattern_index, markerPatterns[(int)MarkerType_CYLINDER]->pattern_count, m_cylinderarkerInfos__fixed
							, context_.m_cylinderMarkerRuntimes, m_veciTemp__detect, m_vecp2Temp__detect, xScale, yScale);
						collect2DPoints(m_parameters__fixed.markerCodeLength, markerPatterns[(int)MarkerType_CYLINDER], imgPointsRuntime, m_cylinderarkerInfos__fixed, context_.m_cylinderMarkerRuntimes);
					}

					for (int aux = 0; aux < numaux; aux++)
					{
						if (markerPatterns[(int)MarkerType_AUX_BASE + aux] != nullptr)
						{
							update2Dcollect3DPoints(m_parameters__fixed.auxiliaryMarkerSettings[aux].checkerboardXLength,
								markerPatterns[(int)MarkerType_AUX_BASE + aux], modelPointsRuntime, m_auxliaryMarkerInfos__fixed[aux], context_.m_auxliaryMarkerRuntimes[aux]);
							refinePlaneCorners(m_parameters__fixed.auxiliaryMarkerSettings[aux].checkerboardXLength,
								originalGray_, m_auxliaryMarkerInfos__fixed[aux], context_.m_auxliaryMarkerRuntimes[aux], m_veciTemp__detect, m_vecp2Temp__detect, xScale, yScale);
							collect2DPoints(m_parameters__fixed.auxiliaryMarkerSettings[aux].checkerboardXLength,
								markerPatterns[(int)MarkerType_AUX_BASE + aux], imgPointsRuntime, m_auxliaryMarkerInfos__fixed[aux], context_.m_auxliaryMarkerRuntimes[aux]);
							if (markerPatterns[(int)MarkerType_CYLINDER] != nullptr)
								bSecondTry = true;
						}
					}

					if (context_.m_validRTVec == true)
					{
						context_.m_rvec.copyTo(mrvec);
						context_.m_tvec.copyTo(mtvec);
					}
					//MatOfPoint3f matModelPoints = new MatOfPoint3f();
					//matModelPoints.fromList(modelPointsRuntime);

					//List<Point> imgPointsList = imgPoints.toList();
					//MatOfPoint2f matImgPoints = new MatOfPoint2f();
					//matImgPoints.fromList(imgPointsRuntime);


					Vector3 v3TryLastDetectPosition;
					Quaternion qTryLastDetectOrientation;
					Vector3 v3TryLastDetectRotVec;

					int iTryPrevMarkerIndex = -1;
					int iTryPrevMarkerCount = 0;
					uint uTryTrackType = 0;
					uint uTryPossibleTrackType = 0;
					OpenCVRect rectTryAll;
					float dTryPrevEdgeMax = 0.0f;
					OpenCVRect rectTryPrevCylinderMarkerImgBound;
					float uTryPrevCylinderMarkerEdgeMax = 0.0f;

					std::vector<OpenCVRect>& listTryPrevRectAuxiliaryMarkersEdgeMax = m_vecRectRuntime__detect;
					std::vector<float>& listTryPrevAuxiliaryMarkerEdgeMax = m_vecFloatRuntime__detect;
					listTryPrevRectAuxiliaryMarkersEdgeMax.clear();
					listTryPrevRectAuxiliaryMarkersEdgeMax.resize(numaux);
					listTryPrevAuxiliaryMarkerEdgeMax.clear();
					listTryPrevAuxiliaryMarkerEdgeMax.resize(numaux, 0.f);

					int iTryLastAdaptiveBlockSize = 0;

					for (int trackTry = 0; trackTry <= ((bSecondTry) ? 1 : 0); trackTry++)
					{
						uint uTrackTryMark = 0;
						if (trackTry == 1)
						{
							modelPointsRuntime.clear();
							imgPointsRuntime.clear();

							//if (markerPatterns[(int)MarkerType_CYLINDER] != null)
							//{
							//simplifyCylinderPattern(markerPatterns[(int)MarkerType_CYLINDER]);
							update2Dcollect3DPoints(m_parameters__fixed.markerCodeLength, markerPatterns[(int)MarkerType_CYLINDER], modelPointsRuntime, m_cylinderarkerInfos__fixed, context_.m_cylinderMarkerRuntimes);
							refineCyclicCorners(originalGray_, markerPatterns[(int)MarkerType_CYLINDER]->pattern_index, markerPatterns[(int)MarkerType_CYLINDER]->pattern_count
								, m_cylinderarkerInfos__fixed, context_.m_cylinderMarkerRuntimes
								, m_veciTemp__detect, m_vecp2Temp__detect, xScale, yScale);
							collect2DPoints(m_parameters__fixed.markerCodeLength, markerPatterns[(int)MarkerType_CYLINDER], imgPointsRuntime, m_cylinderarkerInfos__fixed, context_.m_cylinderMarkerRuntimes);
							//}

							//for (int aux = 0; aux < numaux; aux++)
							//{
							//	if (markerPatterns[(int)MarkerType_AUX_BASE + aux] != null)
							//	{
							//		collect2D3DPoints(m_parameters__fixed.auxiliaryMarkerSettings[aux].checkerboardXLength, 
							//			markerPatterns[(int)MarkerType_AUX_BASE + aux], modelPointsRuntime, imgPointsRuntime, m_auxliaryMarkerInfos__fixed[aux], context_.m_auxliaryMarkerRuntimes[aux]);
							//		refinePlaneCorners(m_parameters__fixed.auxiliaryMarkerSettings[aux].checkerboardXLength
							//			, curGray_, m_auxliaryMarkerInfos__fixed[aux], context_.m_auxliaryMarkerRuntimes[aux]);
							//	}
							//}

							//matModelPoints.fromList(modelPointsRuntime);
							//matImgPoints.fromList(imgPointsRuntime);

						}
						if (cv::solvePnPRansac(modelPointsRuntime, imgPointsRuntime, context_.m_matCamera, cv::Mat(), mrvec, mtvec, context_.m_validRTVec))
						{

							double* tvec = mtvec.ptr<double>();

							if (tvec[2] == 0.0)
							{
								continue;
							}
							else if (tvec[2] < 0.0)
							{
								tvec[0] = -tvec[0];
								tvec[1] = -tvec[1];
								tvec[2] = -tvec[2];

								cv::Rodrigues(mrvec, matRotation);
								matRotation = -matRotation;

								cv::Rodrigues(matRotation, mrvec);
							}
							double* rvec = mrvec.ptr<double>();


							Vector3 p((float)tvec[0], (float)-tvec[1], (float)tvec[2]);
							Vector3 rv = Vector3((float)-rvec[0], (float)rvec[1], (float)-rvec[2]);
							Quaternion r = RotationVectorToQuaternion(rv);
							if (true == (std::isnan(p[0]) || std::isnan(p[1]) || std::isnan(p[2])
								|| std::isnan(r[0]) || std::isnan(r[1]) || std::isnan(r[2]) || std::isnan(r[3])))
							{
								continue;
							}

							mrvec.copyTo(context_.m_rvec);
							mtvec.copyTo(context_.m_tvec);
							context_.m_validRTVec = true;

							composeTransformProjectionMatrix(mrvec, mtvec, context_.m_matCamera, matRotation, matTransform, matProjection);

							//lastDetectPosition = MultiplyPoint3x4(context_.m_curCameraTransform, p);
							//lastDetectOrientation = QuaternionMultiply(context_.m_curCameraRotation, r);

							if (pSpatialCoordinate != nullptr && m_transformPoseToGlobalCoordinateCB != nullptr)
							{
								PoseAndSpatialCoordinate pose;
								pose.m_fPositionX = p[0];
								pose.m_fPositionY = p[1];
								pose.m_fPositionZ = p[2];
								pose.m_fOrientationX = r[0];
								pose.m_fOrientationY = r[1];
								pose.m_fOrientationZ = r[2];
								pose.m_fOrientationW = r[3];
								pose.m_pSpatialCoordinateSystem = pSpatialCoordinate;
								(*m_transformPoseToGlobalCoordinateCB)(pose);
								p[0] = pose.m_fPositionX;
								p[1] = pose.m_fPositionY;
								p[2] = pose.m_fPositionZ;
								r[0] = pose.m_fOrientationX;
								r[1] = pose.m_fOrientationY;
								r[2] = pose.m_fOrientationZ;
								r[3] = pose.m_fOrientationW;
							}

							lastDetectPosition = p;
							lastDetectOrientation = r;
							//lastDetectRotVec = rv;

							bLastDetectPoseValid = true;

							bool bDetectValid = true;
							{
								if (context_.m_bLastVisionPose == true)
								{
									float fDist2 = DistanceSq(lastDetectPosition, context_.m_v3LastVisionPosition);
									float fQDot = lastDetectOrientation.dot(context_.m_qLastVisionOrientation);
									if (fQDot < 0.0f)
										fQDot = -fQDot;
									float fQAngle = std::acos(fQDot) * 2.0f;
									if (fDist2 > m_parameters__fixed.validPositionChange * m_parameters__fixed.validPositionChange
										|| fQAngle > m_parameters__fixed.validAngleChange)
									{
										bDetectValid = false;
									}
								}
								//if (bDetectValid == true)
								//{
								//    this.invalidatePrevData();
								//    m_iPrevMarkerIndex = iMarkerIndex;
								//    if (this.updateTrackedMarker(rvec, tvec) == false)
								//    {
								//        bDetectValid = false;
								//        bTrackValid = false;
								//    }
								//}
							}

							if (bDetectValid == true)
							{
								int pattern_index = -1;
								int pattern_count = 0;

								uint uTrackType = 0;
								uint uPossibleTrackType = 0;
								for (int marker = 0; marker <= numaux; marker++)
								{
									uint flag = (uint)(1 << marker);
									if (markerPatterns[marker] != nullptr)
									{
										if (flag == (uint)MarkerTypeFlag_CYLINDER)
										{
											uTrackType |= (uint)MarkerTypeFlag_CYLINDER;
											pattern_index = markerPatterns[0]->pattern_index;
											pattern_count = markerPatterns[0]->pattern_count;
										}
										else
										{
											uTrackType |= flag;
										}
									}
								}
								uTrackType |= uTrackTryMark;

								float dMaxEdge = 0.0f;
								std::vector<VerifyProjectionRet>& listResult = m_vecVerifyProjectionRet__detect;
								listResult.clear();
								OpenCVRect rectAll;
								if (computeAndVerifyReprojection(context_, originalGray_, matRotation, matTransform, matProjection, false
									, (trackTry == 0) ? CVRUPDATE_UPDATE : CVRUPDATE_IF_VALID, /*ref*/ pattern_index, /*ref*/ pattern_count
									, /*ref*/ uTrackType, /*out*/uPossibleTrackType, /*out*/dMaxEdge, /*out*/rectAll, listResult
									, m_veciTemp__detect, m_vecbyTemp__detect, m_veci2Temp__detect, m_vecbTemp__detect) == false)
								{
									bDetectValid = false;
									continue;
								}

								bTryValid = true;
								v3TryLastDetectPosition = lastDetectPosition;
								qTryLastDetectOrientation = lastDetectOrientation;
								//v3TryLastDetectRotVec = lastDetectRotVec;

								iTryPrevMarkerIndex = pattern_index;
								iTryPrevMarkerCount = pattern_count;
								uTryTrackType = uTrackType;
								uTryPossibleTrackType = uPossibleTrackType;
								rectTryAll = rectAll;
								dTryPrevEdgeMax = dMaxEdge;
								rectTryPrevCylinderMarkerImgBound = listResult[0].m_rect;
								uTryPrevCylinderMarkerEdgeMax = listResult[0].m_dMaxEdge;
								for (int aux = 0; aux < numaux; aux++)
								{
									listTryPrevRectAuxiliaryMarkersEdgeMax[aux] = listResult[1 + aux].m_rect;
									listTryPrevAuxiliaryMarkerEdgeMax[aux] = listResult[1 + aux].m_dMaxEdge;
								}
								iTryLastAdaptiveBlockSize = (int)std::round(dMaxEdge * 2);
								break;
								//if (curColor_.empty() == false)
								//{
								//	//if (markerPatterns[0] != null)
								//	//{
								//	//	for (int c = 0; c < markerPatterns[0].pattern_quads.size(); c++)
								//	//	{
								//	//		CvCBQuad q = markerPatterns[0].pattern_quads[c];
								//	//		for (int j = 0; j < 4; j++)
								//	//		{
								//	//			Imgproc.line(curColor_, q.corners[j].pt, q.corners[(j + 1) & 3].pt, new Scalar(0, 0, 255, 255));
								//	//		}

								//	//		if (q.bitContour != null)
								//	//		{
								//	//			List<MatOfPoint> mat = new List<MatOfPoint>();
								//	//			mat.Add(q.bitContour);
								//	//			Imgproc.drawContours(curColor_, mat, 0, new Scalar(255, 255, 0, 255)
								//	//				, 1, 8, new Mat(), int.MaxValue, new Point(rect.x, rect.y));

								//	//		}
								//	//	}
								//	//}

								//	//for (int marker = 1; marker <= numaux; marker++)
								//	//{
								//	//	uint flag = (uint)(1 << marker);
								//	//	if (markerPatterns[marker] != null)
								//	//	{
								//	//		for (int c = 0; c < markerPatterns[marker].pattern_quads.size(); c++)
								//	//		{
								//	//			CvCBQuad q = markerPatterns[marker].pattern_quads[c];
								//	//			for (int j = 0; j < 4; j++)
								//	//			{
								//	//				Imgproc.line(curColor_, q.corners[j].pt, q.corners[(j + 1) & 3].pt, new Scalar(0, 0, 255, 255));
								//	//			}

								//	//			if (q.bitContour != null)
								//	//			{
								//	//				List<MatOfPoint> mat = new List<MatOfPoint>();
								//	//				mat.Add(q.bitContour);
								//	//				Imgproc.drawContours(curColor_, mat, 0, new Scalar(255, 255, 0, 255)
								//	//					, 1, 8, new Mat(), int.MaxValue, new Point(rect.x, rect.y));

								//	//			}
								//	//		}
								//	//	}
								//	//}s




								//}
							}
						}

						//if (max_count < count)
						//{
						//	//detectPattern(quad_group, num_rows, num_columns, dilations);

						//	// set max_count to its new value
						//	max_count = count;
						//	//max_dilation_run_ID = dilations;
						//	output_quad_group.Clear();
						//	output_quad_group.AddRange(quad_group);
						//}
					}

					if (bLastDetectPoseValid == true)
					{
						context_.m_v3LastVisionPosition = lastDetectPosition;
						context_.m_qLastVisionOrientation = lastDetectOrientation;
						context_.m_bLastVisionPose = true;
					}

					if (bTryValid == true)
					{
						position_ = v3TryLastDetectPosition;
						orientation_ = qTryLastDetectOrientation;
						//rotVec_ = v3TryLastDetectRotVec;

						context_.m_iPrevDetectFailCount = 0;
						context_.m_uExternalTrackType = 0;
						if (curGray_.ptr<byte>() != context_.m_prevGray.ptr<byte>())
							curGray_.copyTo(context_.m_prevGray);

						if (originalGray_.ptr<byte>() != context_.m_prevOriginal.ptr<byte>())
							originalGray_.copyTo(context_.m_prevOriginal);

						context_.m_iPrevMarkerIndex = iTryPrevMarkerIndex;
						context_.m_iPrevMarkerCount = iTryPrevMarkerCount;
						context_.m_uTrackType = uTryTrackType;
						context_.m_uPossibleTrackType = uTryPossibleTrackType;
						if (rectTryAll.empty() == false)
						{
							rectTryAll.x -= m_parameters__fixed.expandAreaMargin;
							rectTryAll.y -= m_parameters__fixed.expandAreaMargin;
							rectTryAll.width += m_parameters__fixed.expandAreaMargin * 2;
							rectTryAll.height += m_parameters__fixed.expandAreaMargin * 2;
							context_.m_i64PrevImgBoundTimestamp = i64Timestamp;
						}
						else
						{
							context_.m_i64PrevImgBoundTimestamp = 0;
						}
						context_.m_prevImgBound = rectTryAll;

						context_.m_prevEdgeMax = context_.m_prevCylinderMarkerEdgeMax;
						context_.m_prevCylinderMarkerImgBound = rectTryPrevCylinderMarkerImgBound;
						context_.m_prevCylinderMarkerEdgeMax = uTryPrevCylinderMarkerEdgeMax;
						for (int aux = 0; aux < numaux; aux++)
						{
							context_.m_prevAuxiliaryMarkersImgBound[aux] = listTryPrevRectAuxiliaryMarkersEdgeMax[aux];
							context_.m_prevAuxiliaryMarkersEdgeMax[aux] = listTryPrevAuxiliaryMarkerEdgeMax[aux];
						}
						context_.m_lastAdaptiveBlockSize = iTryLastAdaptiveBlockSize;

						if (curColor_.empty() == false)
						{

							if (iTryPrevMarkerIndex >= 0 && iTryPrevMarkerCount == 1)
							{
								if ((uTryTrackType & (uint)MarkerTypeFlag_CYLINDER) != 0)
								{
									for (int i = -1; i <= iTryPrevMarkerCount; i++)
									{
										int index = (iTryPrevMarkerIndex + iPatternNums + i) % iPatternNums;
										for (const Point2& point : context_.m_cylinderMarkerRuntimes[index].m_prevImgPoints)
										{
											cv::line(curColor_, Point2i((int)point.x - 2, (int)point.y - 2), Point2i((int)point.x + 2, (int)point.y + 2), cv::Scalar(255, 255, 0, 255));
											cv::line(curColor_, Point2i((int)point.x + 2, (int)point.y - 2), Point2i((int)point.x - 2, (int)point.y + 2), cv::Scalar(255, 255, 0, 255));
										}
									}
								}

								for (int i = 0; i <= iTryPrevMarkerCount; i++)
								{
									int index = (iTryPrevMarkerIndex + iPatternNums + i) % iPatternNums;
									for (const Point2& point : context_.m_cylinderMarkerRuntimes[index].m_prevLcenterImgPoints)
									{
										cv::line(curColor_, Point2i((int)point.x - 2, (int)point.y - 2), Point2i((int)point.x + 2, (int)point.y + 2), cv::Scalar(0, 0, 255, 255));
										cv::line(curColor_, Point2i((int)point.x + 2, (int)point.y - 2), Point2i((int)point.x - 2, (int)point.y + 2), cv::Scalar(0, 0, 255, 255));
									}
								}
							}

							for (int aux = 0; aux < numaux; aux++)
							{
								uint flag = (uint)(1 << ((int)MarkerType_AUX_BASE + aux));
								if ((uTryTrackType & flag) != 0)
								{
									int iCount = (int)context_.m_auxliaryMarkerRuntimes[aux].size();
									for (int index = 0; index < iCount; index++)
									{
										for (const Point2& point : context_.m_auxliaryMarkerRuntimes[aux][index].m_prevImgPoints)
										{
											cv::line(curColor_, Point2i((int)point.x - 2, (int)point.y - 2), Point2i((int)point.x + 2, (int)point.y + 2), cv::Scalar(255, 255, 0, 255));
											cv::line(curColor_, Point2i((int)point.x + 2, (int)point.y - 2), Point2i((int)point.x - 2, (int)point.y + 2), cv::Scalar(255, 255, 0, 255));
										}
									}


									if ((uTryPossibleTrackType & flag) != 0)
									{
										switch (m_parameters__fixed.auxiliaryMarkerSettings[aux].type)
										{
										case AuxiliaryMarker::CHECKER:
											for (int index = 1; index < iCount; index++)
											{
												for (const Point2& point : context_.m_auxliaryMarkerRuntimes[aux][index].m_prevLcenterImgPoints)
												{
													cv::line(curColor_, Point2i((int)point.x - 2, (int)point.y - 2), Point2i((int)point.x + 2, (int)point.y + 2), cv::Scalar(0, 0, 255, 255));
													cv::line(curColor_, Point2i((int)point.x + 2, (int)point.y - 2), Point2i((int)point.x - 2, (int)point.y + 2), cv::Scalar(0, 0, 255, 255));
												}
											}
											break;
										}
									}

								}

							}
						}

					}

				}

				quads.clear();
				//dialation_quads.Clear();
				quad_group.clear();
				foundPattern.clear();

				//			Profiler.EndSample();
			}
			if (bTryValid == true)
				return true;

			context_.m_iPrevDetectFailCount++;

			return false;

		}

		bool trackMarkerNew__track(WorkContext& context_, cv::Mat& matCurGray_, cv::Mat& matCurOriginal_, cv::Mat& matCurColor_, IUnknown* pSpatialCoordinate, int64_t i64Timestamp, /*out*/Vector3& position_, /*out*/Quaternion& orientation_)
		{
			position_ = Vector3(0, 0, 0);
			orientation_ = Quaternion(0, 0, 0, 1);

			if (matCurGray_.empty() == true || matCurOriginal_.empty() == true)
			{
				return false;
			}

			int iPatternNums = (int)m_cylinderarkerInfos__fixed.size();


			int width = matCurGray_.cols;
			int height = matCurGray_.rows;

			Vector3 lastDetectPosition;
			Quaternion lastDetectOrientation;
			//Vector3 lastDetectRotVec;
			bool bLastDetectPoseValid = false;
			bool bDetectValid = false;

			int iMarkerCodeLength = m_parameters__fixed.markerCodeLength;
			int iMajorCodeLength = (iMarkerCodeLength + 1) >> 1;
			int iMinorCodeLength = iMarkerCodeLength - iMajorCodeLength;

			if (context_.m_iPrevMarkerIndex < 0 || context_.m_iPrevMarkerIndex >= iPatternNums || context_.m_iPrevMarkerCount < 1)
			{
				context_.m_uPrevValidTrackType &= ~((uint)MarkerTypeFlag_CYLINDER);
				context_.m_uExternalTrackType &= ~((uint)MarkerTypeFlag_CYLINDER);
			}

			int numaux = (int)m_parameters__fixed.auxiliaryMarkerSettings.size();
			for (int aux = 0; aux < numaux; aux++)
			{
				uint flag = (uint)(1 << ((int)MarkerType_AUX_BASE + aux));
				if (m_auxliaryMarkerInfos__fixed[aux].empty() || context_.m_auxliaryMarkerRuntimes[aux].empty())
				{
					context_.m_uPrevValidTrackType &= ~flag;
					context_.m_uExternalTrackType &= ~flag;
				}
			}

			if (context_.m_vecPrevGrayPyr.empty() == true || context_.m_vecPrevGrayPyr.front().cols != matCurGray_.cols || context_.m_vecPrevGrayPyr.front().rows != matCurGray_.rows)
			{
				context_.m_uPrevValidTrackType = 0;
				context_.m_vecPrevGrayPyr.clear();
			}
			if (context_.m_prevGray.cols != matCurGray_.cols || context_.m_prevGray.rows != matCurGray_.rows)
			{
				context_.m_uExternalTrackType = 0;
			}
			uint uPrevValidType = context_.m_uPrevValidTrackType;

			context_.m_uExternalTrackType &= ~uPrevValidType;

			if (((uPrevValidType | context_.m_uExternalTrackType) & m_uMarkerMask__fixed) == 0)
			{
				return false;
			}

			double xScale = (double)matCurOriginal_.cols / (double)matCurGray_.cols;
			double yScale = (double)matCurOriginal_.rows / (double)matCurGray_.rows;
			double invXScale = 1.0 / xScale;
			double invYScale = 1.0 / yScale;

			Size winSize(32, 32);
			std::vector<cv::Mat> vecCurGrayPyr;
			cv::buildOpticalFlowPyramid(matCurGray_, vecCurGrayPyr, winSize, 3, true, cv::BORDER_DEFAULT, cv::BORDER_CONSTANT, false);

			std::vector<Point2>& curPointsList = m_imgPointsRuntime__track;
			curPointsList.clear();
			std::vector<Byte>& _status = m_opticalFlowStatusRuntime__track;
			_status.clear();

			if ((uPrevValidType & m_uMarkerMask__fixed) != 0)
			{
				std::vector<Point2>& prevPointsList = m_prePointsRuntime__track;
				prevPointsList.clear();

				if ((uPrevValidType & (uint)MarkerTypeFlag_CYLINDER) != 0)
				{
					for (int i = -1; i <= context_.m_iPrevMarkerCount; i++)
					{
						int iPatternIndex = (context_.m_iPrevMarkerIndex + iPatternNums + i) % iPatternNums;
						prevPointsList.insert(prevPointsList.end()
							, context_.m_cylinderMarkerRuntimes[iPatternIndex].m_prevImgPoints.begin()
							, context_.m_cylinderMarkerRuntimes[iPatternIndex].m_prevImgPoints.end());
					}
				}
				for (int aux = 0; aux < numaux; aux++)
				{
					uint flag = (uint)(1 << ((int)MarkerType_AUX_BASE + aux));
					if ((uPrevValidType & flag) != 0)
					{
						int iAuxMarkerCodeLength = m_parameters__fixed.auxiliaryMarkerSettings[aux].checkerboardXLength;
						int iAuxLineNum = (int)m_parameters__fixed.auxiliaryMarkerSettings[aux].checkerboardCoordinates.size();
#ifndef USE_LIBCBDETECTOR
						int iAuxMajorCodeLength = (iAuxMarkerCodeLength + 1) >> 1;
						int iAuxMinorCodeLength = iAuxMarkerCodeLength - iAuxMajorCodeLength;
						bool bMajor = m_parameters__fixed.auxiliaryMarkerSettings[aux].codeMajor;
						bool bLastMajor = (((iAuxLineNum - 2) & 1) == 0) ? bMajor : !bMajor;
#endif USE_LIBCBDETECTOR

						switch (m_parameters__fixed.auxiliaryMarkerSettings[aux].type)
						{
						case AuxiliaryMarker::CHECKER:
						case AuxiliaryMarker::NOPATTERN:
#ifdef USE_LIBCBDETECTOR
							for (int i = 0; i < iAuxLineNum; i++)
							{
								prevPointsList.insert(prevPointsList.end()
									, context_.m_auxliaryMarkerRuntimes[aux][i].m_prevImgPoints.begin()
									, context_.m_auxliaryMarkerRuntimes[aux][i].m_prevImgPoints.end());
							}
#else  USE_LIBCBDETECTOR
							if (iAuxMajorCodeLength == iAuxMinorCodeLength)
							{
								prevPointsList.insert(prevPointsList.end()
									, context_.m_auxliaryMarkerRuntimes[aux][0].m_prevImgPoints.begin() + ((bMajor) ? 0 : 1)
									, context_.m_auxliaryMarkerRuntimes[aux][0].m_prevImgPoints.begin() + (((bMajor) ? 0 : 1) + iAuxMarkerCodeLength));
								for (int i = 1; i < iAuxLineNum - 1; i++)
								{
									prevPointsList.insert(prevPointsList.end()
										, context_.m_auxliaryMarkerRuntimes[aux][i].m_prevImgPoints.begin()
										, context_.m_auxliaryMarkerRuntimes[aux][i].m_prevImgPoints.end());
								}
								prevPointsList.insert(prevPointsList.end()
									, context_.m_auxliaryMarkerRuntimes[aux][iAuxLineNum - 1].m_prevImgPoints.begin() + ((bLastMajor) ? 1 : 0)
									, context_.m_auxliaryMarkerRuntimes[aux][iAuxLineNum - 1].m_prevImgPoints.begin() + (((bLastMajor) ? 1 : 0) + iAuxMarkerCodeLength));
							}
							else
							{
								if (bMajor)
									prevPointsList.insert(prevPointsList.end()
										, context_.m_auxliaryMarkerRuntimes[aux][0].m_prevImgPoints.begin()
										, context_.m_auxliaryMarkerRuntimes[aux][0].m_prevImgPoints.end());
								else
									prevPointsList.insert(prevPointsList.end()
										, context_.m_auxliaryMarkerRuntimes[aux][0].m_prevImgPoints.begin() + 1
										, context_.m_auxliaryMarkerRuntimes[aux][0].m_prevImgPoints.begin() + (1 + iAuxMinorCodeLength * 2));
								for (int i = 1; i < iAuxLineNum - 1; i++)
								{
									prevPointsList.insert(prevPointsList.end()
										, context_.m_auxliaryMarkerRuntimes[aux][i].m_prevImgPoints.begin()
										, context_.m_auxliaryMarkerRuntimes[aux][i].m_prevImgPoints.end());
								}
								if (bLastMajor)
									prevPointsList.insert(prevPointsList.end()
										, context_.m_auxliaryMarkerRuntimes[aux][iAuxLineNum - 1].m_prevImgPoints.begin()
										, context_.m_auxliaryMarkerRuntimes[aux][iAuxLineNum - 1].m_prevImgPoints.end());
								else
									prevPointsList.insert(prevPointsList.end()
										, context_.m_auxliaryMarkerRuntimes[aux][iAuxLineNum - 1].m_prevImgPoints.begin() + 1
										, context_.m_auxliaryMarkerRuntimes[aux][iAuxLineNum - 1].m_prevImgPoints.begin() + (1 + iAuxMinorCodeLength * 2));
							}
#endif USE_LIBCBDETECTOR
							break;
						}
					}
				}

				std::vector<float>& errArray = m_opticalFlowErrRuntime__track;
				errArray.clear();

				for (int i = 0; i < prevPointsList.size(); ++i)
				{					
					prevPointsList[i].x *= invXScale;
					prevPointsList[i].y *= invYScale;
				}

				cv::calcOpticalFlowPyrLK(context_.m_vecPrevGrayPyr, vecCurGrayPyr, prevPointsList, curPointsList, /*out*/_status, /*out*/errArray, winSize, 3);
				if (_status.size() != prevPointsList.size() || curPointsList.size() != prevPointsList.size())
					return false;

			}
			if ((context_.m_uExternalTrackType & m_uMarkerMask__fixed) != 0)
			{
				std::vector<Point2>& prevPointsList = m_prePointsRuntime__track;
				prevPointsList.clear();

				if ((context_.m_uExternalTrackType & (uint)MarkerTypeFlag_CYLINDER) != 0)
				{
					for (int i = -1; i <= context_.m_iPrevMarkerCount; i++)
					{
						int iPatternIndex = (context_.m_iPrevMarkerIndex + iPatternNums + i) % iPatternNums;
						prevPointsList.insert(prevPointsList.end()
							, context_.m_cylinderMarkerRuntimes[iPatternIndex].m_prevImgPoints.begin()
							, context_.m_cylinderMarkerRuntimes[iPatternIndex].m_prevImgPoints.end());
					}
				}
				for (int aux = 0; aux < numaux; aux++)
				{
					uint flag = (uint)(1 << ((int)MarkerType_AUX_BASE + aux));
					if ((context_.m_uExternalTrackType & flag) != 0)
					{
						int iAuxMarkerCodeLength = m_parameters__fixed.auxiliaryMarkerSettings[aux].checkerboardXLength;
						int iAuxLineNum = (int)m_parameters__fixed.auxiliaryMarkerSettings[aux].checkerboardCoordinates.size();
#ifndef USE_LIBCBDETECTOR
						int iAuxMajorCodeLength = (iAuxMarkerCodeLength + 1) >> 1;
						int iAuxMinorCodeLength = iAuxMarkerCodeLength - iAuxMajorCodeLength;
						bool bMajor = m_parameters__fixed.auxiliaryMarkerSettings[aux].codeMajor;
						bool bLastMajor = (((iAuxLineNum - 2) & 1) == 0) ? bMajor : !bMajor;
#endif USE_LIBCBDETECTOR
						switch (m_parameters__fixed.auxiliaryMarkerSettings[aux].type)
						{
						case AuxiliaryMarker::CHECKER:
						case AuxiliaryMarker::NOPATTERN:
#ifdef USE_LIBCBDETECTOR
							for (int i = 0; i < iAuxLineNum; i++)
							{
								prevPointsList.insert(prevPointsList.end(), context_.m_auxliaryMarkerRuntimes[aux][i].m_prevImgPoints.begin()
									, context_.m_auxliaryMarkerRuntimes[aux][i].m_prevImgPoints.end());
							}
#else  USE_LIBCBDETECTOR
							if (iAuxMajorCodeLength == iAuxMinorCodeLength)
							{
								prevPointsList.insert(prevPointsList.end()
									, context_.m_auxliaryMarkerRuntimes[aux][0].m_prevImgPoints.begin() + ((bMajor) ? 0 : 1)
									, context_.m_auxliaryMarkerRuntimes[aux][0].m_prevImgPoints.begin() + (((bMajor) ? 0 : 1) + iAuxMarkerCodeLength));
								for (int i = 1; i < iAuxLineNum - 1; i++)
								{
									prevPointsList.insert(prevPointsList.end()
										, context_.m_auxliaryMarkerRuntimes[aux][i].m_prevImgPoints.begin()
										, context_.m_auxliaryMarkerRuntimes[aux][i].m_prevImgPoints.end());
								}
								prevPointsList.insert(prevPointsList.end()
									, context_.m_auxliaryMarkerRuntimes[aux][iAuxLineNum - 1].m_prevImgPoints.begin() + ((bLastMajor) ? 1 : 0)
									, context_.m_auxliaryMarkerRuntimes[aux][iAuxLineNum - 1].m_prevImgPoints.begin() + (((bLastMajor) ? 1 : 0) + iAuxMarkerCodeLength));
							}
							else
							{
								if (bMajor)
									prevPointsList.insert(prevPointsList.end()
										, context_.m_auxliaryMarkerRuntimes[aux][0].m_prevImgPoints.begin()
										, context_.m_auxliaryMarkerRuntimes[aux][0].m_prevImgPoints.end());
								else
									prevPointsList.insert(prevPointsList.end()
										, context_.m_auxliaryMarkerRuntimes[aux][0].m_prevImgPoints.begin() + 1
										, context_.m_auxliaryMarkerRuntimes[aux][0].m_prevImgPoints.begin() + (1 + iAuxMinorCodeLength * 2));
								for (int i = 1; i < iAuxLineNum - 1; i++)
								{
									prevPointsList.insert(prevPointsList.end()
										, context_.m_auxliaryMarkerRuntimes[aux][i].m_prevImgPoints.begin()
										, context_.m_auxliaryMarkerRuntimes[aux][i].m_prevImgPoints.end());
								}
								if (bLastMajor)
									prevPointsList.insert(prevPointsList.end()
										, context_.m_auxliaryMarkerRuntimes[aux][iAuxLineNum - 1].m_prevImgPoints.begin()
										, context_.m_auxliaryMarkerRuntimes[aux][iAuxLineNum - 1].m_prevImgPoints.end());
								else
									prevPointsList.insert(prevPointsList.end()
										, context_.m_auxliaryMarkerRuntimes[aux][iAuxLineNum - 1].m_prevImgPoints.begin() + 1
										, context_.m_auxliaryMarkerRuntimes[aux][iAuxLineNum - 1].m_prevImgPoints.end() + (1 + iAuxMinorCodeLength * 2));
							}
#endif USE_LIBCBDETECTOR
							break;
						}
					}
					// FRONT, BACK 이 둘 다 들어있으면 안 됨
				}

				for (int i = 0; i < prevPointsList.size(); ++i)
				{
					prevPointsList[i].x *= invXScale;
					prevPointsList[i].y *= invYScale;
				}

				std::vector<Point2>& _curPoints = m_vecp2Temp__track;
				_curPoints.clear();
				std::vector<Byte>& statusArray = m_opticalFlowStatus2Runtime__track;
				statusArray.clear();
				std::vector<float>& errArray = m_opticalFlowErrRuntime__track;
				errArray.clear();

				cv::calcOpticalFlowPyrLK(context_.m_prevGray, vecCurGrayPyr, prevPointsList, _curPoints, /*out*/statusArray, /*out*/errArray, winSize, 3);
				if (statusArray.size() != prevPointsList.size() || _curPoints.size() != prevPointsList.size())
					return false;

				curPointsList.insert(curPointsList.end(), _curPoints.begin(), _curPoints.end());
				_status.insert(_status.end(), statusArray.begin(), statusArray.end());
			}

			int iPointIndex = 0;
			int iValidCount = 0;
			int iValidLineCount = 0;

			std::vector<Point3>& modelPointsRuntime = m_modelPointsRuntime__track;
			modelPointsRuntime.clear();

			int iCylinderModelPointsRuntimeCount = 0;

			int iCylinderValidCount = 0;
			int iCylinderValidLineCount = 0;

			if ((uPrevValidType & m_uMarkerMask__fixed) != 0)
			{
				if ((uPrevValidType & (uint)MarkerTypeFlag_CYLINDER) != 0)
				{
					for (int i = -1; i <= context_.m_iPrevMarkerCount; i++)
					{
						int iPatternIndex = (context_.m_iPrevMarkerIndex + iPatternNums + i) % iPatternNums;
						int iForLineCount = 0;
						for (int c = 0; c <= iMarkerCodeLength; c++, iPointIndex++)
						{
							if (_status[iPointIndex] == 0)
							{
								context_.m_cylinderMarkerRuntimes[iPatternIndex].m_tmpImgPointsErr[c] = 0;
							}
							else
							{
								context_.m_cylinderMarkerRuntimes[iPatternIndex].m_tmpImgPointsErr[c] = m_parameters__fixed.trackPixelTolerance;
								context_.m_cylinderMarkerRuntimes[iPatternIndex].m_tmpImgPoints[c] = curPointsList[iPointIndex];
								iValidCount++;
								iForLineCount++;
								if (iForLineCount == 2)
								{
									iValidLineCount++;
								}
								modelPointsRuntime.push_back(m_cylinderarkerInfos__fixed[iPatternIndex].m_corner3DPoints[c]);
							}
						}
					}

					iCylinderModelPointsRuntimeCount = (int)modelPointsRuntime.size();
					iCylinderValidCount = iValidCount;
					iCylinderValidLineCount = iValidLineCount;

				}
				for (int aux = 0; aux < numaux; aux++)
				{
					uint flag = (uint)(1 << ((int)MarkerType_AUX_BASE + aux));
					if ((uPrevValidType & flag) != 0)
					{
						int iAuxMarkerCodeLength = m_parameters__fixed.auxiliaryMarkerSettings[aux].checkerboardXLength;
						int iAuxLineNum = (int)m_parameters__fixed.auxiliaryMarkerSettings[aux].checkerboardCoordinates.size();
#ifndef USE_LIBCBDETECTOR
						int iAuxMajorCodeLength = (iAuxMarkerCodeLength + 1) >> 1;
						int iAuxMinorCodeLength = iAuxMarkerCodeLength - iAuxMajorCodeLength;
						bool bMajor = m_parameters__fixed.auxiliaryMarkerSettings[aux].codeMajor;
						bool bLastMajor = (((iAuxLineNum - 2) & 1) == 0) ? bMajor : !bMajor;
#endif USE_LIBCBDETECTOR
						switch (m_parameters__fixed.auxiliaryMarkerSettings[aux].type)
						{
						case AuxiliaryMarker::CHECKER:
						case AuxiliaryMarker::NOPATTERN:
							for (int i = 0; i < iAuxLineNum; i++)
							{
								int iForLineCount = 0;
#ifdef USE_LIBCBDETECTOR
								int iStart = 0;
								int iEnd = iAuxMarkerCodeLength;
#else  USE_LIBCBDETECTOR
								int iStart = 0;
								int iEnd = 0;
								if (iAuxMajorCodeLength == iAuxMinorCodeLength)
								{
									if (i == 0)
									{
										iStart = (bMajor) ? 0 : 1;
										iEnd = iStart + iAuxMarkerCodeLength - 1;
									}
									else if (i == iAuxLineNum - 1)
									{
										iStart = (bLastMajor) ? 0 : 1;
										iEnd = iStart + iAuxMarkerCodeLength - 1;
									}
									else
									{
										iStart = 0;
										iEnd = iAuxMarkerCodeLength;
									}
								}
								else
								{
									if (i == 0)
									{
										if (bMajor)
										{
											iStart = 0;
											iEnd = iAuxMarkerCodeLength;
										}
										else
										{
											iStart = 1;
											iEnd = iAuxMinorCodeLength * 2;
										}
									}
									else if (i == iAuxLineNum - 1)
									{
										if (bLastMajor)
										{
											iStart = 0;
											iEnd = iAuxMarkerCodeLength;
										}
										else
										{
											iStart = 1;
											iEnd = iAuxMinorCodeLength * 2;
										}
									}
									else
									{
										iStart = 0;
										iEnd = iAuxMarkerCodeLength;
									}
								}
#endif USE_LIBCBDETECTOR
								for (int c = iStart; c <= iEnd; c++, iPointIndex++)
								{
									if (_status[iPointIndex] == 0)
									{
										context_.m_auxliaryMarkerRuntimes[aux][i].m_tmpImgPointsErr[c] = 0;
									}
									else
									{
										context_.m_auxliaryMarkerRuntimes[aux][i].m_tmpImgPointsErr[c] = m_parameters__fixed.trackPixelTolerance;
										context_.m_auxliaryMarkerRuntimes[aux][i].m_tmpImgPoints[c] = curPointsList[iPointIndex];
										iValidCount++;
										iForLineCount++;
										if (iForLineCount == 2)
										{
											iValidLineCount++;
										}
										modelPointsRuntime.push_back(m_auxliaryMarkerInfos__fixed[aux][i].m_corner3DPoints[c]);
									}
								}
							}
							break;
						}
					}
					// FRONT, BACK 이 둘 다 들어있으면 안 됨
				}
			}

			int iValidNoExternalCount = iValidCount;
			int iValidNoExternalLineCount = iValidLineCount;

			int iNoExternalModelPointsRuntimeCount = (int)modelPointsRuntime.size();

			if ((context_.m_uExternalTrackType & m_uMarkerMask__fixed) != 0)
			{
				if ((context_.m_uExternalTrackType & (uint)MarkerTypeFlag_CYLINDER) != 0)
				{
					for (int i = -1; i <= context_.m_iPrevMarkerCount; i++)
					{
						int iPatternIndex = (context_.m_iPrevMarkerIndex + iPatternNums + i) % iPatternNums;
						int iForLineCount = 0;
						for (int c = 0; c <= iMarkerCodeLength; c++, iPointIndex++)
						{
							if (_status[iPointIndex] == 0)
							{
								context_.m_cylinderMarkerRuntimes[iPatternIndex].m_tmpImgPointsErr[c] = 0;
							}
							else
							{
								context_.m_cylinderMarkerRuntimes[iPatternIndex].m_tmpImgPointsErr[c] = m_parameters__fixed.trackPixelTolerance;
								context_.m_cylinderMarkerRuntimes[iPatternIndex].m_tmpImgPoints[c] = curPointsList[iPointIndex];
								iValidCount++;
								iForLineCount++;
								if (iForLineCount == 2)
								{
									iValidLineCount++;
								}
								modelPointsRuntime.push_back(m_cylinderarkerInfos__fixed[iPatternIndex].m_corner3DPoints[c]);
							}
						}
					}
				}
				for (int aux = 0; aux < numaux; aux++)
				{
					uint flag = (uint)(1 << ((int)MarkerType_AUX_BASE + aux));
					if ((context_.m_uExternalTrackType & flag) != 0)
					{
						int iAuxMarkerCodeLength = m_parameters__fixed.auxiliaryMarkerSettings[aux].checkerboardXLength;
						int iAuxLineNum = (int)m_parameters__fixed.auxiliaryMarkerSettings[aux].checkerboardCoordinates.size();
#ifndef USE_LIBCBDETECTOR
						int iAuxMajorCodeLength = (iAuxMarkerCodeLength + 1) >> 1;
						int iAuxMinorCodeLength = iAuxMarkerCodeLength - iAuxMajorCodeLength;
						bool bMajor = m_parameters__fixed.auxiliaryMarkerSettings[aux].codeMajor;
						bool bLastMajor = (((iAuxLineNum - 2) & 1) == 0) ? bMajor : !bMajor;
#endif USE_LIBCBDETECTOR
						switch (m_parameters__fixed.auxiliaryMarkerSettings[aux].type)
						{
						case AuxiliaryMarker::CHECKER:
						case AuxiliaryMarker::NOPATTERN:
							for (int i = 0; i < iAuxLineNum; i++)
							{
								int iForLineCount = 0;
#ifdef USE_LIBCBDETECTOR
								int iStart = 0;
								int iEnd = iAuxMarkerCodeLength;
#else  USE_LIBCBDETECTOR
								int iStart = 0;
								int iEnd = 0;
								if (iAuxMajorCodeLength == iAuxMinorCodeLength)
								{
									if (i == 0)
									{
										iStart = (bMajor) ? 0 : 1;
										iEnd = iStart + iAuxMarkerCodeLength - 1;
									}
									else if (i == iAuxLineNum - 1)
									{
										iStart = (bLastMajor) ? 0 : 1;
										iEnd = iStart + iAuxMarkerCodeLength - 1;
									}
									else
									{
										iStart = 0;
										iEnd = iAuxMarkerCodeLength;
									}
								}
								else
								{
									if (i == 0)
									{
										if (bMajor)
										{
											iStart = 0;
											iEnd = iAuxMarkerCodeLength;
										}
										else
										{
											iStart = 1;
											iEnd = iAuxMinorCodeLength * 2;
										}
									}
									else if (i == iAuxLineNum - 1)
									{
										if (bLastMajor)
										{
											iStart = 0;
											iEnd = iAuxMarkerCodeLength;
										}
										else
										{
											iStart = 1;
											iEnd = iAuxMinorCodeLength * 2;
										}
									}
									else
									{
										iStart = 0;
										iEnd = iAuxMarkerCodeLength;
									}
								}

#endif USE_LIBCBDETECTOR

								for (int c = iStart; c <= iEnd; c++, iPointIndex++)
								{
									if (_status[iPointIndex] == 0)
									{
										context_.m_auxliaryMarkerRuntimes[aux][i].m_tmpImgPointsErr[c] = 0;
									}
									else
									{
										context_.m_auxliaryMarkerRuntimes[aux][i].m_tmpImgPointsErr[c] = m_parameters__fixed.trackPixelTolerance;
										context_.m_auxliaryMarkerRuntimes[aux][i].m_tmpImgPoints[c] = curPointsList[iPointIndex];
										iValidCount++;
										iForLineCount++;
										if (iForLineCount == 2)
										{
											iValidLineCount++;
										}
										modelPointsRuntime.push_back(m_auxliaryMarkerInfos__fixed[aux][i].m_corner3DPoints[c]);
									}
								}
							}
							break;
						}
					}
				}
			}

			if (iValidCount < 4 || iValidLineCount <= 1)
			{
				return false;
			}

			if (((uPrevValidType | context_.m_uExternalTrackType) & (uint)MarkerTypeFlag_CYLINDER) != 0)
			{
				refineCyclicCorners(matCurOriginal_, context_.m_iPrevMarkerIndex, context_.m_iPrevMarkerCount, m_cylinderarkerInfos__fixed, context_.m_cylinderMarkerRuntimes
					, m_veciTemp__track, m_vecp2Temp__track, xScale, yScale);
			}
			for (int aux = 0; aux < numaux; aux++)
			{
				uint flag = (uint)(1 << ((int)MarkerType_AUX_BASE + aux));
				if (((uPrevValidType | context_.m_uExternalTrackType) & flag) != 0)
				{
					//if ( (m_uAuxUniqueMarkerMask__fixed & flag ) != 0 )
					refinePlaneCorners(m_parameters__fixed.auxiliaryMarkerSettings[aux].checkerboardXLength,
						matCurOriginal_, m_auxliaryMarkerInfos__fixed[aux], context_.m_auxliaryMarkerRuntimes[aux]
						, m_veciTemp__track, m_vecp2Temp__track, xScale, yScale);
					//else
					//	refineBlockCorners(matCurGray_, m_auxliaryMarkerInfos__fixed[aux], context_.m_auxliaryMarkerRuntimes[aux]);
				}

				// FRONT, BACK 이 둘 다 들어있으면 안 됨
			}

			iPointIndex = 0;
			std::vector<Point2>& imgPointsRuntime = m_imgPointsRuntime__track;
			imgPointsRuntime.clear();
			int iCylinderImgPointsRuntimeCount = 0;

			if ((uPrevValidType & m_uMarkerMask__fixed) != 0)
			{
				if ((uPrevValidType & (uint)MarkerTypeFlag_CYLINDER) != 0)
				{
					for (int i = -1; i <= context_.m_iPrevMarkerCount; i++)
					{
						int iPatternIndex = (context_.m_iPrevMarkerIndex + iPatternNums + i) % iPatternNums;
						for (int c = 0; c <= iMarkerCodeLength; c++, iPointIndex++)
						{
							if (_status[iPointIndex] != 0)
							{
								imgPointsRuntime.push_back(context_.m_cylinderMarkerRuntimes[iPatternIndex].m_tmpImgPoints[c]);
							}
						}
					}
					iCylinderImgPointsRuntimeCount = (int)imgPointsRuntime.size();

				}
				for (int aux = 0; aux < numaux; aux++)
				{
					uint flag = (uint)(1 << ((int)MarkerType_AUX_BASE + aux));
					if ((uPrevValidType & flag) != 0)
					{
						int iAuxMarkerCodeLength = m_parameters__fixed.auxiliaryMarkerSettings[aux].checkerboardXLength;
						int iAuxLineNum = (int)m_parameters__fixed.auxiliaryMarkerSettings[aux].checkerboardCoordinates.size();
#ifndef USE_LIBCBDETECTOR
						int iAuxMajorCodeLength = (iAuxMarkerCodeLength + 1) >> 1;
						int iAuxMinorCodeLength = iAuxMarkerCodeLength - iAuxMajorCodeLength;
						bool bMajor = m_parameters__fixed.auxiliaryMarkerSettings[aux].codeMajor;
						bool bLastMajor = (((iAuxLineNum - 2) & 1) == 0) ? bMajor : !bMajor;
#endif USE_LIBCBDETECTOR
						switch (m_parameters__fixed.auxiliaryMarkerSettings[aux].type)
						{
						case AuxiliaryMarker::CHECKER:
						case AuxiliaryMarker::NOPATTERN:
							for (int i = 0; i < iAuxLineNum; i++)
							{
#ifdef USE_LIBCBDETECTOR
								int iStart = 0;
								int iEnd = iAuxMarkerCodeLength;
#else  USE_LIBCBDETECTOR
								int iStart = 0;
								int iEnd = 0;
								if (iAuxMajorCodeLength == iAuxMinorCodeLength)
								{
									if (i == 0)
									{
										iStart = (bMajor) ? 0 : 1;
										iEnd = iStart + iAuxMarkerCodeLength - 1;
									}
									else if (i == iAuxLineNum - 1)
									{
										iStart = (bLastMajor) ? 0 : 1;
										iEnd = iStart + iAuxMarkerCodeLength - 1;
									}
									else
									{
										iStart = 0;
										iEnd = iAuxMarkerCodeLength;
									}
								}
								else
								{
									if (i == 0)
									{
										if (bMajor)
										{
											iStart = 0;
											iEnd = iAuxMarkerCodeLength;
										}
										else
										{
											iStart = 1;
											iEnd = iAuxMinorCodeLength * 2;
										}
									}
									else if (i == iAuxLineNum - 1)
									{
										if (bLastMajor)
										{
											iStart = 0;
											iEnd = iAuxMarkerCodeLength;
										}
										else
										{
											iStart = 1;
											iEnd = iAuxMinorCodeLength * 2;
										}
									}
									else
									{
										iStart = 0;
										iEnd = iAuxMarkerCodeLength;
									}
								}


#endif USE_LIBCBDETECTOR
								for (int c = iStart; c <= iEnd; c++, iPointIndex++)
								{
									if (_status[iPointIndex] != 0)
									{
										imgPointsRuntime.push_back(context_.m_auxliaryMarkerRuntimes[aux][i].m_tmpImgPoints[c]);
									}
								}
							}
							break;
						}
					}
					// FRONT, BACK 이 둘 다 들어있으면 안 됨
				}
			}
			int iNoExternalImgPointsRuntimeCount = (int)imgPointsRuntime.size();

			if ((context_.m_uExternalTrackType & m_uMarkerMask__fixed) != 0)
			{
				if ((context_.m_uExternalTrackType & (uint)MarkerTypeFlag_CYLINDER) != 0)
				{
					for (int i = -1; i <= context_.m_iPrevMarkerCount; i++)
					{
						int iPatternIndex = (context_.m_iPrevMarkerIndex + iPatternNums + i) % iPatternNums;
						for (int c = 0; c <= iMarkerCodeLength; c++, iPointIndex++)
						{
							if (_status[iPointIndex] != 0)
							{
								imgPointsRuntime.push_back(context_.m_cylinderMarkerRuntimes[iPatternIndex].m_tmpImgPoints[c]);
							}
						}
					}
				}
				for (int aux = 0; aux < numaux; aux++)
				{
					uint flag = (uint)(1 << ((int)MarkerType_AUX_BASE + aux));
					if ((context_.m_uExternalTrackType & flag) != 0)
					{
						int iAuxMarkerCodeLength = m_parameters__fixed.auxiliaryMarkerSettings[aux].checkerboardXLength;
						int iAuxLineNum = (int)m_parameters__fixed.auxiliaryMarkerSettings[aux].checkerboardCoordinates.size();
#ifndef USE_LIBCBDETECTOR
						int iAuxMajorCodeLength = (iAuxMarkerCodeLength + 1) >> 1;
						int iAuxMinorCodeLength = iAuxMarkerCodeLength - iAuxMajorCodeLength;
						bool bMajor = m_parameters__fixed.auxiliaryMarkerSettings[aux].codeMajor;
						bool bLastMajor = (((iAuxLineNum - 2) & 1) == 0) ? bMajor : !bMajor;
#endif USE_LIBCBDETECTOR
						switch (m_parameters__fixed.auxiliaryMarkerSettings[aux].type)
						{
						case AuxiliaryMarker::CHECKER:
						case AuxiliaryMarker::NOPATTERN:
							for (int i = 0; i < iAuxLineNum; i++)
							{
#ifdef USE_LIBCBDETECTOR
								int iStart = 0;
								int iEnd = iAuxMarkerCodeLength;
#else  USE_LIBCBDETECTOR
								int iStart = 0;
								int iEnd = 0;
								if (iAuxMajorCodeLength == iAuxMinorCodeLength)
								{
									if (i == 0)
									{
										iStart = (bMajor) ? 0 : 1;
										iEnd = iStart + iAuxMarkerCodeLength - 1;
									}
									else if (i == iAuxLineNum - 1)
									{
										iStart = (bLastMajor) ? 0 : 1;
										iEnd = iStart + iAuxMarkerCodeLength - 1;
									}
									else
									{
										iStart = 0;
										iEnd = iAuxMarkerCodeLength;
									}
								}
								else
								{
									if (i == 0)
									{
										if (bMajor)
										{
											iStart = 0;
											iEnd = iAuxMarkerCodeLength;
										}
										else
										{
											iStart = 1;
											iEnd = iAuxMinorCodeLength * 2;
										}
									}
									else if (i == iAuxLineNum - 1)
									{
										if (bLastMajor)
										{
											iStart = 0;
											iEnd = iAuxMarkerCodeLength;
										}
										else
										{
											iStart = 1;
											iEnd = iAuxMinorCodeLength * 2;
										}
									}
									else
									{
										iStart = 0;
										iEnd = iAuxMarkerCodeLength;
									}
								}


#endif USE_LIBCBDETECTOR
								for (int c = iStart; c <= iEnd; c++, iPointIndex++)
								{
									if (_status[iPointIndex] != 0)
									{
										imgPointsRuntime.push_back(context_.m_auxliaryMarkerRuntimes[aux][i].m_tmpImgPoints[c]);
									}
								}
							}
							break;
						}
					}
				}
			}


			double adTransform[12];
			double adProjection[12];
			double adRotation[9];
			double adrvec[3];
			double adtvec[3];
			cv::Mat matTransform(3, 4, CV_64FC1, adTransform);
			cv::Mat matProjection(3, 4, CV_64FC1, adProjection);
			cv::Mat matRotation(3, 3, CV_64FC1, adRotation);
			cv::Mat mrvec(3, 1, CV_64FC1, adrvec);
			cv::Mat mtvec(3, 1, CV_64FC1, adtvec);
			{
				if (context_.m_validRTVec == true)
				{
					context_.m_rvec.copyTo(mrvec);
					context_.m_tvec.copyTo(mtvec);
				}

				uint uTryMark = 7U;
				if ((uPrevValidType & m_uMarkerMask__fixed) == 0 || iValidNoExternalCount < 4 || iValidNoExternalLineCount <= 1
					|| (context_.m_uExternalTrackType & m_uMarkerMask__fixed) == 0)
				{
					uTryMark &= ~6U;
				}
				else if ((uPrevValidType & (uint)MarkerTypeFlag_CYLINDER) == 0 || iCylinderValidCount < 4 || iCylinderValidLineCount <= 1
					|| iCylinderModelPointsRuntimeCount == iNoExternalModelPointsRuntimeCount)
				{
					uTryMark &= ~4U;
				}

				for (int trackTry = 0; trackTry < 3; trackTry++)
				{
					std::vector<Point3>& matModelPoints = m_vecp3Temp__track;
					matModelPoints.clear();
					std::vector<Point2>& matImgPoints = m_vecp2Temp__track;
					uint uTrackType = 0;
					uint uPossibleTrackType = 0;
					bool bUpdatePrevimgPoints = false;

					switch (trackTry)
					{
					case 0:
						if ((uTryMark & 1) == 0)
							continue;
						matModelPoints.assign(modelPointsRuntime.begin(), modelPointsRuntime.end());
						matImgPoints.assign(imgPointsRuntime.begin(), imgPointsRuntime.end());
						uTrackType = uPrevValidType | context_.m_uExternalTrackType;
						bUpdatePrevimgPoints = ((uTryMark & (~1U)) == 0) ? true : false;
						break;
					case 1:
						if ((uTryMark & 2) == 0)
							continue;
						uTrackType = uPrevValidType;
						matModelPoints.assign(modelPointsRuntime.begin(), modelPointsRuntime.begin() + iNoExternalModelPointsRuntimeCount);
						matImgPoints.assign(imgPointsRuntime.begin(), imgPointsRuntime.begin() + iNoExternalImgPointsRuntimeCount);
						bUpdatePrevimgPoints = ((uTryMark & (~3U)) == 0) ? true : false;
						break;
					case 2:
						if ((uTryMark & 4) == 0)
							continue;
						uTrackType = (uint)MarkerTypeFlag_CYLINDER;
						matModelPoints.assign(modelPointsRuntime.begin(), modelPointsRuntime.begin() + iCylinderModelPointsRuntimeCount);
						matImgPoints.assign(imgPointsRuntime.begin(), imgPointsRuntime.begin() + iCylinderImgPointsRuntimeCount);
						bUpdatePrevimgPoints = true;
						break;
					}

					if (cv::solvePnPRansac(matModelPoints, matImgPoints, context_.m_matCamera, cv::Mat(), mrvec, mtvec, context_.m_validRTVec) == true)
					{
						double* tvec = mtvec.ptr<double>();

						if (tvec[2] == 0.0)
						{
							continue;
						}
						else if (tvec[2] < 0.0)
						{
							tvec[0] = -tvec[0];
							tvec[1] = -tvec[1];
							tvec[2] = -tvec[2];

							cv::Rodrigues(mrvec, matRotation);
							matRotation = -matRotation;
							cv::Rodrigues(matRotation, mrvec);
						}

						const double* rvec = mrvec.ptr<double>();

						Vector3 p((float)tvec[0], (float)-tvec[1], (float)tvec[2]);
						Vector3 rv = Vector3((float)-rvec[0], (float)rvec[1], (float)-rvec[2]);
						Quaternion r = RotationVectorToQuaternion(rv);

						if (true == (std::isnan(p[0]) || std::isnan(p[1]) || std::isnan(p[2])
							|| std::isnan(r[0]) || std::isnan(r[1]) || std::isnan(r[2]) || std::isnan(r[3])))
						{
							continue;
						}

						mrvec.copyTo(context_.m_rvec);
						mtvec.copyTo(context_.m_tvec);
						context_.m_validRTVec = true;

						composeTransformProjectionMatrix(mrvec, mtvec, context_.m_matCamera, matRotation, matTransform, matProjection);

						if (pSpatialCoordinate != nullptr && m_transformPoseToGlobalCoordinateCB != nullptr)
						{
							PoseAndSpatialCoordinate pose;
							pose.m_fPositionX = p[0];
							pose.m_fPositionY = p[1];
							pose.m_fPositionZ = p[2];
							pose.m_fOrientationX = r[0];
							pose.m_fOrientationY = r[1];
							pose.m_fOrientationZ = r[2];
							pose.m_fOrientationW = r[3];
							pose.m_pSpatialCoordinateSystem = pSpatialCoordinate;
							(*m_transformPoseToGlobalCoordinateCB)(pose);
							p[0] = pose.m_fPositionX;
							p[1] = pose.m_fPositionY;
							p[2] = pose.m_fPositionZ;
							r[0] = pose.m_fOrientationX;
							r[1] = pose.m_fOrientationY;
							r[2] = pose.m_fOrientationZ;
							r[3] = pose.m_fOrientationW;
						}

						//lastDetectPosition = MultiplyPoint3x4( context_.m_curCameraTransform, p );
						//lastDetectOrientation = QuaternionMultiply(context_.m_curCameraRotation, r);
						lastDetectPosition = p;
						lastDetectOrientation = r;
						//lastDetectRotVec = rv;

						bLastDetectPoseValid = true;

						bDetectValid = true;
						if (context_.m_bLastVisionPose == true)
						{
							float fDist2 = DistanceSq(lastDetectPosition, context_.m_v3LastVisionPosition);
							float fQDot = lastDetectOrientation.dot(context_.m_qLastVisionOrientation);
							if (fQDot < 0.0f)
								fQDot = -fQDot;
							float fQAngle = std::acos(fQDot) * 2.0f;
							if (fDist2 > m_parameters__fixed.validPositionChange * m_parameters__fixed.validPositionChange
								|| fQAngle > m_parameters__fixed.validAngleChange)
							{
								bDetectValid = false;
							}
						}

						if (bDetectValid == true)
						{
							int pattern_index = context_.m_iPrevMarkerIndex;
							int pattern_count = context_.m_iPrevMarkerCount;

							if ((uTrackType & (uint)MarkerTypeFlag_CYLINDER) == 0)
							{
								pattern_index = -1;
								pattern_count = 0;
							}

							float dMaxEdge = 0.0f;
							std::vector<VerifyProjectionRet>& listResult = m_vecVerifyProjectionRet__track;
							listResult.clear();
							OpenCVRect rectAll;
							if (computeAndVerifyReprojection(context_, matCurOriginal_, matRotation, matTransform, matProjection, true, CVRUPDATE_IF_VALID
								, /*ref*/ pattern_index, /*ref*/ pattern_count, /*ref*/ uTrackType, /*out*/uPossibleTrackType
								, /*out*/dMaxEdge, /*out*/rectAll, listResult, m_veciTemp__track, m_vecbyTemp__track
								, m_veci2Temp__track, m_vecbTemp__track) == false)
							{
								bDetectValid = false;
								continue;
							}

							if (matCurColor_.empty() == false)
							{
								if (pattern_index >= 0 && pattern_count == 1)
								{
									if ((uTrackType & (uint)MarkerTypeFlag_CYLINDER) != 0)
									{
										for (int i = -1; i <= pattern_count; i++)
										{
											int index = (pattern_index + iPatternNums + i) % iPatternNums;
											for (const Point2& point : context_.m_cylinderMarkerRuntimes[index].m_prevImgPoints)
											{
												cv::line(matCurColor_, Point2i((int)point.x - 2, (int)point.y - 2), Point2i((int)point.x + 2, (int)point.y + 2), cv::Scalar(255, 255, 0, 255));
												cv::line(matCurColor_, Point2i((int)point.x + 2, (int)point.y - 2), Point2i((int)point.x - 2, (int)point.y + 2), cv::Scalar(255, 255, 0, 255));
											}
										}
									}

									for (int i = 0; i <= pattern_count; i++)
									{
										int index = (pattern_index + iPatternNums + i) % iPatternNums;
										for (const Point2& point : context_.m_cylinderMarkerRuntimes[index].m_prevLcenterImgPoints)
										{
											cv::line(matCurColor_, Point2i((int)point.x - 2, (int)point.y - 2), Point2i((int)point.x + 2, (int)point.y + 2), cv::Scalar(0, 0, 255, 255));
											cv::line(matCurColor_, Point2i((int)point.x + 2, (int)point.y - 2), Point2i((int)point.x - 2, (int)point.y + 2), cv::Scalar(0, 0, 255, 255));
										}
									}
								}

								for (int aux = 0; aux < numaux; aux++)
								{
									uint flag = (uint)(1 << ((int)MarkerType_AUX_BASE + aux));
									if ((uTrackType & flag) != 0)
									{
										int iCount = (int)context_.m_auxliaryMarkerRuntimes[aux].size();
										int iAuxLineNum = (int)m_parameters__fixed.auxiliaryMarkerSettings[aux].checkerboardCoordinates.size();
										for (int index = 0; index < iCount; index++)
										{
											for (const Point2& point : context_.m_auxliaryMarkerRuntimes[aux][index].m_prevImgPoints)
											{
												cv::line(matCurColor_, Point2i((int)point.x - 2, (int)point.y - 2), Point2i((int)point.x + 2, (int)point.y + 2), cv::Scalar(255, 255, 0, 255));
												cv::line(matCurColor_, Point2i((int)point.x + 2, (int)point.y - 2), Point2i((int)point.x - 2, (int)point.y + 2), cv::Scalar(255, 255, 0, 255));
											}
										}

										if ((uPossibleTrackType & flag) != 0)
										{
											switch (m_parameters__fixed.auxiliaryMarkerSettings[aux].type)
											{
											case AuxiliaryMarker::CHECKER:
											case AuxiliaryMarker::NOPATTERN:
												for (int index = 1; index < iAuxLineNum; index++)
												{
													for (const Point2& point : context_.m_auxliaryMarkerRuntimes[aux][index].m_prevLcenterImgPoints)
													{
														cv::line(matCurColor_, Point2i((int)point.x - 2, (int)point.y - 2), Point2i((int)point.x + 2, (int)point.y + 2), cv::Scalar(0, 0, 255, 255));
														cv::line(matCurColor_, Point2i((int)point.x + 2, (int)point.y - 2), Point2i((int)point.x - 2, (int)point.y + 2), cv::Scalar(0, 0, 255, 255));
													}
												}
												break;
											}
										}

									}

								}
							}

							context_.m_iPrevMarkerIndex = pattern_index;
							context_.m_iPrevMarkerCount = pattern_count;
							context_.m_uTrackType = uTrackType;
							context_.m_uPrevValidTrackType = uTrackType;
							context_.m_uPossibleTrackType = uPossibleTrackType;
							context_.m_uExternalTrackType = 0;
							context_.m_vecPrevGrayPyr.swap(vecCurGrayPyr);
							context_.m_i64PrevTimestamp = i64Timestamp;

							context_.m_lastAdaptiveBlockSize = (int)std::round(dMaxEdge * 2);
							if (rectAll.empty() == false)
							{
								rectAll.x -= m_parameters__fixed.expandAreaMargin;
								rectAll.y -= m_parameters__fixed.expandAreaMargin;
								rectAll.width += m_parameters__fixed.expandAreaMargin * 2;
								rectAll.height += m_parameters__fixed.expandAreaMargin * 2;
								context_.m_i64PrevImgBoundTimestamp = i64Timestamp;
							}
							else
							{
								context_.m_i64PrevImgBoundTimestamp = 0;
							}

							context_.m_prevImgBound = rectAll;
							context_.m_prevEdgeMax = dMaxEdge;
							context_.m_prevCylinderMarkerImgBound = listResult[0].m_rect;
							context_.m_prevCylinderMarkerEdgeMax = listResult[0].m_dMaxEdge;
							for (int aux = 0; aux < numaux; aux++)
							{
								context_.m_prevAuxiliaryMarkersImgBound[aux] = listResult[1 + aux].m_rect;
								context_.m_prevAuxiliaryMarkersEdgeMax[aux] = listResult[1 + aux].m_dMaxEdge;
							}

							//double edgeMax2 = 0;

							//for (int c = 0; c < pattern->pattern_quads.size(); c++)
							//{
							//	CvCBQuad q = pattern->pattern_quads[c];
							//	edgeMax2 = std::max(edgeMax2, q.edge_max_sqr);
							//	for (int j = 0; j < 4; j++)
							//	{
							//		Imgproc.line(m_curColor__track, q.corners[j].pt, q.corners[(j + 1) & 3].pt, new Scalar(0, 0, 255, 255));
							//	}

							//	if (q.bitContour != null)
							//	{
							//		List<MatOfPoint> mat = new List<MatOfPoint>();
							//		mat.Add(q.bitContour);
							//		Imgproc.drawContours(m_curColor__track, mat, 0, (pattern->pattern_count > 1)
							//			? new Scalar(255, 0, 0, 255)
							//			: new Scalar(255, 255, 0, 255)
							//			);
							//	}
							//}
							//if (edgeMax2 > 0)
							//{
							//	int size = (int)std::round(std::sqrt(edgeMax2) * 2);
							//	int defaultSize = (int)std::round(std::min(width, height) * m_parameters__fixed.adaptiveThresholdBlockRatio);

							//	m_lastAdaptiveBlockSize = std::min(size, defaultSize) | 1;
							//}
							//goto outer;
							break;

						}
					}

				}

				//if (max_count < count)
				//{
				//	//detectPattern(quad_group, num_rows, num_columns, dilations);

				//	// set max_count to its new value
				//	max_count = count;
				//	//max_dilation_run_ID = dilations;
				//	output_quad_group.Clear();
				//	output_quad_group.AddRange(quad_group);
				//}

				if (bLastDetectPoseValid == true)
				{
					context_.m_v3LastVisionPosition = lastDetectPosition;
					context_.m_qLastVisionOrientation = lastDetectOrientation;
					context_.m_bLastVisionPose = true;
				}

				if (bDetectValid == true)
				{
					position_ = lastDetectPosition;
					orientation_ = lastDetectOrientation;
					//rotVec_ = lastDetectRotVec;
					return true;
				}

			}

			return false;
		}

	};



}


CVAPI(ExceptionStatus) NewCylindricalProbeTracker_new(
	NewCylindricalProbeTrackerNamespace::ParameterNativeStruct& native
	, NewCylindricalProbeTrackerNamespace::UpdateResultGrayCB updateResultGrayCB
	, NewCylindricalProbeTrackerNamespace::GetFrameInfoStampCB getFrameInfoStampCB
	, NewCylindricalProbeTrackerNamespace::GetFrameInfoNativeStructCB getFrameInfoNativeStructCB
	, NewCylindricalProbeTrackerNamespace::TransformPoseToGlobalCoordinateCB transformPoseToGlobalCoordinateCB
	, NewCylindricalProbeTrackerNamespace::LocateAtTimestampSpatialCB locateAtTimestampSpatialCB
	, NewCylindricalProbeTrackerNamespace::SetBoundRectCB setBoundRectCB
	, NewCylindricalProbeTrackerNamespace::NewCylindricalProbeTracker** returnValue)
{
	BEGIN_WRAP

		NewCylindricalProbeTrackerNamespace::Parameter param;
	param.radius = native.radius;
	param.blobRadius = native.blobRadius;
	param.markerSize = native.markerSize;
	param.markerCodeLength = native.markerCodeLength;
	param.markerCodeStartAngle = native.markerCodeStartAngle;
	param.markerCodeStartMajor = (native.markerCodeStartMajor) ? true : false;
	param.screenZ = native.screenZ;
	param.auxMarkerTrackCosAngle = native.auxMarkerTrackCosAngle;
	param.validPositionChange = native.validPositionChange;
	param.validAngleChange = native.validAngleChange;
	param.adaptiveThresholdBlockRatio1 = native.adaptiveThresholdBlockRatio1;
	param.adaptiveThresholdBlockRatio2 = native.adaptiveThresholdBlockRatio2;
	param.adaptiveThresholdC = native.adaptiveThresholdC;
	param.minDilations = native.minDilations;
	param.maxDilations = native.maxDilations;
	param.expandAreaRatio = native.expandAreaRatio;
	param.expandAreaMargin = native.expandAreaMargin;
	param.blobDetectionTolerance = native.blobDetectionTolerance;
	param.cornerPixelTolerance = native.cornerPixelTolerance;
	param.trackPixelTolerance = native.trackPixelTolerance;
	param.trackNumToleranceFrames = native.trackNumToleranceFrames;
	param.useDetectionThread = (native.useDetectionThread) ? true : false;
	param.useTrackingThread = (native.useTrackingThread) ? true : false;
	param.detectorParams = native.detectorParams;

	param.markerCodePatterns.assign(native.markerCodePatterns, native.markerCodePatterns + native.markerCodePatternsSize);
	param.auxiliaryMarkerSettings.resize(native.auxiliaryMarkerSettingsSize);
	for (int i = 0; i < native.auxiliaryMarkerSettingsSize; i++)
	{
		const NewCylindricalProbeTrackerNamespace::AuxiliaryMarkerNativeStruct& auxNative = native.auxiliaryMarkerSettings[i];
		NewCylindricalProbeTrackerNamespace::AuxiliaryMarker& aux = param.auxiliaryMarkerSettings[i];
		aux.type = (auxNative.type == 0) ? NewCylindricalProbeTrackerNamespace::AuxiliaryMarker::CHECKER
			: NewCylindricalProbeTrackerNamespace::AuxiliaryMarker::NOPATTERN;
		aux.pivotX = auxNative.pivotX;
		aux.pivotY = auxNative.pivotY;
		aux.pivotZ = auxNative.pivotZ;
		aux.angVecX = auxNative.angVecX;
		aux.angVecY = auxNative.angVecY;
		aux.angVecZ = auxNative.angVecZ;
		aux.codeMajor = (auxNative.codeMajor != 0) ? true : false;
		aux.code = auxNative.code;
		aux.checkerboardXLength = auxNative.checkerboardXLength;
		aux.checkerboardMarkerSize = auxNative.checkerboardMarkerSize;
		aux.checkerboardCenterCoordinate = auxNative.checkerboardCenterCoordinate;
		aux.checkerboardCoordinates.assign(auxNative.checkerboardCoordinates, auxNative.checkerboardCoordinates + auxNative.checkerboardCoordinatesSize);
	}

	*returnValue = new NewCylindricalProbeTrackerNamespace::NewCylindricalProbeTracker(param
		, updateResultGrayCB
		, getFrameInfoStampCB
		, getFrameInfoNativeStructCB
		, transformPoseToGlobalCoordinateCB
		, locateAtTimestampSpatialCB
		, setBoundRectCB
	);

	END_WRAP

}


CVAPI(ExceptionStatus) NewCylindricalProbeTracker_start(NewCylindricalProbeTrackerNamespace::NewCylindricalProbeTracker* self)
{
	BEGIN_WRAP

		if (self != nullptr)
		{
			self->Start();
		}

	END_WRAP
}


CVAPI(ExceptionStatus) NewCylindricalProbeTracker_onNewFrame(NewCylindricalProbeTrackerNamespace::NewCylindricalProbeTracker* self)
{
	BEGIN_WRAP

		if (self != nullptr)
		{
			self->OnNewFrame();
		}

	END_WRAP
}

CVAPI(ExceptionStatus) NewCylindricalProbeTracker_getTrackedPoseTimestamp(NewCylindricalProbeTrackerNamespace::NewCylindricalProbeTracker* self, int64_t& stamp)
{
	stamp = 0;

	BEGIN_WRAP

		if (self != nullptr)
		{
			stamp = self->GetTrackedPoseTimestamp();
		}

	END_WRAP

}

CVAPI(ExceptionStatus) NewCylindricalProbeTracker_getTrackedPose(NewCylindricalProbeTrackerNamespace::NewCylindricalProbeTracker* self
	, NewCylindricalProbeTrackerNamespace::TrackedPoseNativeStruct& pose)
{
	BEGIN_WRAP

		if (self != nullptr)
		{
			self->GetTrackedPose(pose);
		}

	END_WRAP
}

CVAPI(ExceptionStatus) NewCylindricalProbeTracker_resetAdaptiveBlockSize(NewCylindricalProbeTrackerNamespace::NewCylindricalProbeTracker* self)
{
	BEGIN_WRAP

		if (self != nullptr)
		{
			self->resetAdaptiveBlockSize();
		}

	END_WRAP
}


CVAPI(ExceptionStatus) NewCylindricalProbeTracker_setBoundPoints(NewCylindricalProbeTrackerNamespace::NewCylindricalProbeTracker* self
	, NewCylindricalProbeTrackerNamespace::BoundPoints& bound)
{
	BEGIN_WRAP

		if (self != nullptr)
		{
			self->setBoundPoints(bound);
		}

	END_WRAP
}



CVAPI(ExceptionStatus) NewCylindricalProbeTracker_delete(NewCylindricalProbeTrackerNamespace::NewCylindricalProbeTracker* self)
{
	BEGIN_WRAP

		if (self != nullptr)
		{
			delete self;
		}

	END_WRAP
}


CVAPI(ExceptionStatus) NewCylindricalProbeTracker_update(NewCylindricalProbeTrackerNamespace::NewCylindricalProbeTracker* self, cv::Mat* mat)
{
	BEGIN_WRAP
		if (self != nullptr)
		{
			if (mat != nullptr)
			{
				self->Update(*mat);
			}
			else
			{
				cv::Mat matEmpty;
				self->Update(matEmpty);
			}
		}
	END_WRAP
}


struct CoordValue
{
	unsigned int    m_x;
	unsigned int    m_y;

	CoordValue()
		: m_x(0xffffffff), m_y(0xffffffff)
	{
	}

	CoordValue(unsigned int x, unsigned int y)
		: m_x(x), m_y(y)
	{
	}
};



struct NearestBoundingBox
{
	std::vector<unsigned short> m_vecDepth;
	std::vector<int>   m_vecFill;
	std::vector<CoordValue> m_vecStack;
};

CVAPI(void*) NewCylindricalProbeTracker_InitNearestBoundingBox()
{
	return new NearestBoundingBox;
}

CVAPI(int) NewCylindricalProbeTracker_DeleteNearestBoundingBox(void* pData)
{
	if (pData == nullptr)
		return E_INVALIDARG;
	NearestBoundingBox* pNBB = static_cast<NearestBoundingBox*>(pData);
	delete pNBB;
	return S_OK;
}

static void EstimateBoundBox(NearestBoundingBox* pData, NewCylindricalProbeTrackerNamespace::BoundBoxResult* pBBox1, unsigned int width, unsigned int height, unsigned int px, unsigned int py, unsigned short u16Tolerance
	, int iFill )
{
	pData->m_vecStack.resize(0);
	int index = width * py + px;
	unsigned short pz = pData->m_vecDepth[index];
	pData->m_vecStack.push_back(CoordValue(px, py));

	while (pData->m_vecStack.empty() == false)
	{
		CoordValue pt = pData->m_vecStack.back(); pData->m_vecStack.pop_back(); // pop a line segment from the stack
		// we'll keep track of the transitions between set and clear cells both above and below the line segment that
		// we're filling. on a transition from a filled cell to a clear cell, we'll push that point as a new segment
		bool setAbove = true, setBelow = true; // initially consider them set so that a clear cell is immediately pushed
		bool bFill;
		unsigned int x;
		index = width * pt.m_y + pt.m_x;
		unsigned short pv = pData->m_vecDepth[index];
		unsigned short pv0 = pv;
		unsigned short v, tv;

		for (x = pt.m_x; x < width; x++) // scan to the right
		{
			index = width * pt.m_y + x;
			if (pData->m_vecFill[index] != 0)
				break;
			v = pData->m_vecDepth[index];
			if (v == 0xffff || !(v <= pv + u16Tolerance && pv <= v + u16Tolerance))
				break;
			pv = v;
			pData->m_vecFill[index] = iFill;
			if (pt.m_y > 0) // if there's a transition in the cell above...
			{
				index = width * (pt.m_y - 1) + x;
				tv = pData->m_vecDepth[index];
				bFill = tv == 0xffff || !(v <= tv + u16Tolerance && tv <= v + u16Tolerance)
					|| pData->m_vecFill[index] != 0;
				if (bFill != setAbove)
				{
					setAbove = !setAbove;
					if (!setAbove)
						pData->m_vecStack.push_back(CoordValue(x, pt.m_y - 1)); // push the new point if it transitioned to clear
				}
			}
			if (pt.m_y + 1 < height ) // if there's a transition in the cell below...
			{
				index = width * (pt.m_y + 1) + x;
				tv = pData->m_vecDepth[index];
				bFill = tv == 0xffff && !(v <= tv + u16Tolerance && tv <= v + u16Tolerance)
					|| pData->m_vecFill[index] != 0;
				if (bFill != setBelow)
				{
					setBelow = !setBelow;
					if (!setBelow)
						pData->m_vecStack.push_back(CoordValue(x, pt.m_y + 1)); // push the new point if it transitioned to clear
				}
			}
		}

		if (pt.m_x > 0) // now we'll scan to the left, if there's anything to the left
		{
			pv = pv0;
			// this time, we want to initialize the flags based on the actual cell values so that we don't add the line
			// segments twice. (e.g. if it's clear above, it needs to transition to set and then back to clear.)
			if (pt.m_y > 0)
			{
				index = width * (pt.m_y - 1) + pt.m_x;
				tv = pData->m_vecDepth[index];
				setAbove = tv == 0xffff || !(pv <= tv + u16Tolerance && tv <= pv + u16Tolerance)
					|| pData->m_vecFill[index] != 0;
			}
			else
			{
				setAbove = false;
			}
			if (pt.m_y < height - 1)
			{
				index = width * (pt.m_y + 1) + pt.m_x;
				tv = pData->m_vecDepth[index];
				setBelow = tv == 0xffff || !(pv <= tv + u16Tolerance && tv <= pv + u16Tolerance)
					|| pData->m_vecFill[index] != 0;
			}
			else
			{
				setBelow = false;
			}

			for (x = pt.m_x; x >= 1;) // scan to the left
			{
				x--;
				index = width * pt.m_y + x;
				if (pData->m_vecFill[index] != 0)
					break;
				v = pData->m_vecDepth[index];
				if (v == 0xffff || !(pv <= v + u16Tolerance && v <= pv + u16Tolerance))
					break;
				pv = v;
				pData->m_vecFill[index] = iFill;
				if (pt.m_y > 0) // if there's a transition in the cell above...
				{
					index = width * (pt.m_y - 1) + x;
					tv = pData->m_vecDepth[index];
					bFill = tv == 0xffff || !(v <= tv + u16Tolerance && tv <= v + u16Tolerance)
						|| pData->m_vecFill[index] != 0;
					if (bFill != setAbove)
					{
						setAbove = !setAbove;
						if (!setAbove)
							pData->m_vecStack.push_back(CoordValue(x, pt.m_y - 1)); // push the new point if it transitioned to clear
					}
				}
				if (pt.m_y + 1 < height) // if there's a transition in the cell below...
				{
					index = width * (pt.m_y + 1) + x;
					tv = pData->m_vecDepth[index];
					bFill = tv == 0xffff || !(v <= tv + u16Tolerance && tv <= v + u16Tolerance)
						|| pData->m_vecFill[index] != 0;
					if (bFill != setBelow)
					{
						setBelow = !setBelow;
						if (!setBelow)
							pData->m_vecStack.push_back(CoordValue(x, pt.m_y + 1)); // push the new point if it transitioned to clear
					}
				}
			}
		}
	}

	pBBox1->m_xMin = pBBox1->m_xMax = px;
	pBBox1->m_yMin = pBBox1->m_yMax = py;
	pBBox1->m_zMin = pBBox1->m_zMax = pz;
	for (unsigned int y = 0; y < height; y++)
	{
		for (unsigned int x = 0; x < width; x++)
		{
			index = width * y + x;
			if (pData->m_vecFill[index] == iFill)
			{
				if (x < pBBox1->m_xMin)
					pBBox1->m_xMin = x;
				if (x > pBBox1->m_xMax)
					pBBox1->m_xMax = x;
				if (y < pBBox1->m_yMin)
					pBBox1->m_yMin = y;
				if (y > pBBox1->m_yMax)
					pBBox1->m_yMax = y;
				pz = pData->m_vecDepth[index];
				if (pz < pBBox1->m_zMin)
					pBBox1->m_zMin = pz;
				if (pz > pBBox1->m_zMax)
					pBBox1->m_zMax = pz;
			}
		}
	}
}

__forceinline unsigned short CheckDepthPixel(unsigned short v, BYTE bSigma, unsigned short mask, unsigned short minshort, unsigned short maxshort)
{
	if ((mask != 0) && (bSigma & mask) > 0)
	{
		return 0xffff;
	}
	if ((maxshort != 0) && (v > maxshort))
	{
		return 0xffff;
	}
	//if (v < minshort)
	//{
	//	return 0xffff;
	//}
	return v;
}

CVAPI(int) NewCylindricalProbeTracker_GetNearestBoudingBox(const unsigned short* pDepth, const unsigned char* pSigma, unsigned int width, unsigned int height
	, unsigned short u16ClampMin, unsigned short u16ClampMax, unsigned short u16Tolerance
	, void* pData_, NewCylindricalProbeTrackerNamespace::BoundBoxResult* pBBox1, NewCylindricalProbeTrackerNamespace::BoundBoxResult* pBBox2)
{
	if (pDepth == nullptr || pData_ == nullptr || pBBox1 == nullptr)
		return E_INVALIDARG;


	unsigned short maxClampDepth = 0;
	unsigned short mask = 0;
	size_t outBufferCount = 0;

	if (pSigma != nullptr)
	{
		mask = 0x80;
		maxClampDepth = 4000;
		if (u16ClampMax < maxClampDepth)
			maxClampDepth = u16ClampMax;
	}
	else
	{
		mask = 0x0;
		maxClampDepth = 1000;
		if (u16ClampMax < maxClampDepth)
			maxClampDepth = u16ClampMax;
		//maxshort = 4090;
	}

	unsigned int minx[2] = { 0xffffffff, 0xffffffff };
	unsigned int miny[2] = { 0xffffffff, 0xffffffff };
	unsigned short minv[2] = { 0xffff, 0xffff };
	unsigned short v;
	unsigned int index;

	NearestBoundingBox* pData = static_cast<NearestBoundingBox*>(pData_);

	pData->m_vecDepth.resize(0);
	pData->m_vecDepth.resize(width * height, 0xffff);
	pData->m_vecFill.resize(0);
	pData->m_vecFill.resize(width * height, 0);

	for (unsigned int y = 0; y < height; y++)
	{
		for (unsigned int x = 0; x < width; x++)
		{
			index = width * y + x;
			v = CheckDepthPixel(pDepth[index], pSigma ? pSigma[index] : 0, mask, u16ClampMin, maxClampDepth);
			pData->m_vecDepth[index] = v;
			if (v >= u16ClampMin && v < minv[0])
			{
				minx[0] = x;
				miny[0] = y;
				minv[0] = v;
			}
		}
	}

	if (minx[0] == 0xffffffff || miny[0] == 0xffffffff || minv[0] == 0xffff)
		return E_FAIL;

	EstimateBoundBox(pData, pBBox1, width, height, minx[0], miny[0], u16Tolerance, 1);
	if (pBBox2 != nullptr)
	{
		pBBox2->m_xMin = 1; pBBox2->m_xMax = 0;
		pBBox2->m_yMin = 1; pBBox2->m_yMax = 0;
		pBBox2->m_zMin = 1; pBBox2->m_yMax = 0;

		for (unsigned int y = 0; y < height; y++)
		{
			for (unsigned int x = 0; x < width; x++)
			{
				if (x >= pBBox1->m_xMin && x <= pBBox1->m_xMax
					&& y >= pBBox1->m_yMin && y <= pBBox1->m_yMax)
					continue;
				index = width * y + x;
				if (pData->m_vecFill[index] == 0)
				{
					v = pData->m_vecDepth[index];
					if (v >= u16ClampMin && v < minv[1])
					{
						minx[1] = x;
						miny[1] = y;
						minv[1] = v;
					}
				}
			}
		}
		if (minx[1] != 0xffffffff && miny[1] != 0xffffffff && minv[1] < 0xffff)
		{
			EstimateBoundBox(pData, pBBox2, width, height, minx[1], miny[1], u16Tolerance, 2);
		}
	}

	return S_OK;
}

