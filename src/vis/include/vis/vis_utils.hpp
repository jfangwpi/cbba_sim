#ifndef VIS_UTILS_HPP
#define VIS_UTILS_HPP

#include <cstdint>

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"

#include "map/common_types.hpp"
namespace librav{

class VisUtils {
private:
	static cv::Scalar pt_color_;		// default point color
	static cv::Scalar ln_color_;		// default line color
	static cv::Scalar area_color_;		// default area color

public:
	static void DrawPoint(cv::Mat img, cv::Point pos, const cv::Scalar& color = VisUtils::pt_color_);
	static void DrawLine(cv::Mat img, cv::Point pt1, cv::Point pt2, const cv::Scalar& color = VisUtils::ln_color_);
	static void FillRectangularArea(cv::Mat img, BoundingBox<int32_t> bbox, const cv::Scalar& color = VisUtils::area_color_);
	
};

}

#endif /* VIS_UTILS_HPP */
