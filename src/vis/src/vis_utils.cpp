#include "vis/vis_utils.hpp"


using namespace librav;
using namespace cv;

cv::Scalar VisUtils::pt_color_ = Scalar( 0, 0, 255 );
cv::Scalar VisUtils::ln_color_ = Scalar(Scalar(0,0,0));
cv::Scalar VisUtils::area_color_ = Scalar(0,255,255);

void VisUtils::DrawPoint(cv::Mat img, cv::Point pos, const cv::Scalar& color)
{
	int thickness = -1;
	int lineType = 8;
	Point center(pos.x,pos.y);
	circle( img,
			center,
			0.5,
			Scalar( 0, 0, 255 ),
			thickness,
			lineType);
}

void VisUtils::DrawLine(cv::Mat img, cv::Point pt1, cv::Point pt2, const cv::Scalar& color)
{
	int thickness = 1;
	int lineType = 8;

	line( img,
		  pt1,
	      pt2,
		  Scalar( 237, 149, 100 ),
		  thickness,
		  lineType);
}

void VisUtils::FillRectangularArea(cv::Mat img, BoundingBox<int32_t> bbox, const cv::Scalar& color)
{
	Range rngx(bbox.x.min, bbox.x.max);
	Range rngy(bbox.y.min, bbox.y.max);
	img(rngy,rngx) = color;
}




