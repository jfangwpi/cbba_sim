#include <iostream>
#include <string>

#include "vis/graph_vis.hpp"
#include "graph/graph.hpp"
#include "map/square_grid.hpp"
#include "vis/vis_utils.hpp"

using namespace librav;
using namespace cv;

cv::Scalar GraphVis::bk_color_ = Scalar(255,255,255);
cv::Scalar GraphVis::ln_color_ = Scalar(Scalar(0,0,0));
//cv::Scalar GraphVis::obs_color_ = Scalar(Scalar(0,102,204));
cv::Scalar GraphVis::obs_color_ = Scalar(128,128,128);
cv::Scalar GraphVis::unkn_color_ = Scalar(Scalar(160,160,160));
cv::Scalar GraphVis::aoi_color_ = Scalar(Scalar(0,162,255));
cv::Scalar GraphVis::start_color_ = Scalar(0,0,255);
cv::Scalar GraphVis::finish_color_ = Scalar(153,76,0);

void GraphVis::VisSquareGrid(const SquareGrid& grid, cv::OutputArray _dst)
{
	int32_t vis_side_size = grid.cell_size_ * grid.pixel_per_meter_;
	_dst.create(Size(grid.num_col_*vis_side_size, grid.num_row_*vis_side_size), CV_8UC3);
	Mat dst = _dst.getMat();
	dst = bk_color_;

	// fill cell color
	for(auto &gird_col: grid.grid_cells_)
	{
		for (auto &itc: gird_col){
			if((*itc).occu_ == OccupancyType::OCCUPIED)
				//FillSquareCellColor((*itc).second->bbox_, obs_color_, dst);
				VisUtils::FillRectangularArea(dst, (*itc).bbox_, obs_color_);
			else if((*itc).occu_ == OccupancyType::INTERESTED)
				//FillSquareCellColor((*itc).second->bbox_, aoi_color_, dst);
				VisUtils::FillRectangularArea(dst, (*itc).bbox_, aoi_color_);

			auto cell = (*itc);
			uint64_t x,y;
			x = cell.bbox_.x.min + (cell.bbox_.x.max - cell.bbox_.x.min)/2;
			x = x + (cell.bbox_.x.max - cell.bbox_.x.min)/6;
			y = cell.bbox_.y.min + (cell.bbox_.y.max - cell.bbox_.y.min)/2;
			y = y + (cell.bbox_.y.max - cell.bbox_.y.min)*3/7;
		}

		//std::string id = std::to_string(cell.second->data_id_);
		//putText(dst, id ,Point(x,y), CV_FONT_NORMAL, 0.5, Scalar(0,0,0),1,1);
	}

	// draw grid lines
	line(dst, Point(0,0),Point(0,grid.num_row_*vis_side_size-1),ln_color_, 1);
	for(int i = 1; i <= grid.num_col_; i++){
		line(dst, Point(i*vis_side_size-1,0),Point(i*vis_side_size-1,grid.num_row_*vis_side_size-1),ln_color_, 1);
	}

	line(dst, Point(0,0),Point(grid.num_col_*vis_side_size-1,0),ln_color_, 1);
	for(int i = 1; i <= grid.num_row_; i++){
		line(dst, Point(0,i*vis_side_size-1),Point(grid.num_col_*vis_side_size-1,i*vis_side_size-1),ln_color_, 1);
	}
}


void GraphVis::VisSquareGridGraph(Graph_t<SquareCell*>& graph, cv::InputArray _src, cv::OutputArray _dst, bool show_id)
{
	Mat src, dst;
	int src_type = _src.getMat().type();
	if(src_type == CV_8UC1)
	{
		cvtColor(_src, src, CV_GRAY2BGR);
		_dst.create(src.size(), src.type());
		dst = _dst.getMat();
	}
	else
	{
		src = _src.getMat();
		_dst.create(_src.size(), _src.type());
		dst = _dst.getMat();
		src.copyTo(dst);
	}

	// draw all vertices
	//std::vector<Vertex_t<SquareCell> *> vertices;
	//auto vertices = graph.GetGraphVertices();
	
	
	for(auto itv = graph.vertex_begin(); itv != graph.vertex_end(); ++itv)
	{
		cv::Point center((*itv).state_->position_.x, (*itv).state_->position_.y);
		//DrawNodeCenter(center,dst);
		VisUtils::DrawPoint(dst, center);

		// current vertex center coordinate
		int64_t x1,y1,x2,y2;
		x1 = (*itv).state_->position_.x;
		y1 = (*itv).state_->position_.y;

		if(show_id) {
			//if((*itv)->bundled_data_->data_id_ % 2 == 0)
			//{
				//uint64_t id_old = (*itv).state_->id_;
				//uint64_t id_show = (14-(id_old)/15)*15 + ((id_old)%15) + 1;
				std::string id = std::to_string((*itv).state_->id_);
				putText(dst, id ,Point(x1,y1), CV_FONT_NORMAL, 1, Scalar(0,0,0),1,1);
			//}
		}
	}

	//draw all edges
	auto edges = graph.GetAllEdges();
	for(auto &ite: edges)
	{
		uint64_t x1,y1,x2,y2;
		x1 = (*ite).src_->state_->position_.x;
		y1 = (*ite).src_->state_->position_.y;
		x2 = (*ite).dst_->state_->position_.x;
		y2 = (*ite).dst_->state_->position_.y;

		//DrawEdge(Point(x1,y1), Point(x2,y2), dst);
		VisUtils::DrawLine(dst, Point(x1,y1), Point(x2,y2));
	}
}

void GraphVis::VisSquareGridPath(Path_t<SquareCell *>& path, cv::InputArray _src, cv::OutputArray _dst)
{
	Mat src, dst;
	int src_type = _src.getMat().type();
	if(src_type == CV_8UC1)
	{
		cvtColor(_src, src, CV_GRAY2BGR);
		_dst.create(src.size(), src.type());
		dst = _dst.getMat();
	}
	else
	{
		src = _src.getMat();
		_dst.create(_src.size(), _src.type());
		dst = _dst.getMat();
		src.copyTo(dst);
	}

	// draw starting and finishing cell
	auto cell_s = path[0];
	uint64_t x,y;
	x = cell_s->position_.x;
	x = x - (cell_s->bbox_.x.max - cell_s->bbox_.x.min)/8;
	y = cell_s->position_.y;
	y = y + (cell_s->bbox_.y.max - cell_s->bbox_.y.min)/8;
	//FillSquareCellColor(cell_s->bbox_, start_color_, dst);
	VisUtils::FillRectangularArea(dst, cell_s->bbox_, start_color_);
	putText(dst, "S" ,Point(x,y), CV_FONT_NORMAL, 1, Scalar(0,0,0),1,1);

	auto cell_f = (*(path.end()-1));
	x = cell_f->position_.x;
	x = x - (cell_f->bbox_.x.max - cell_f->bbox_.x.min)/8;
	y = cell_f->position_.y;
	y = y + (cell_f->bbox_.y.max - cell_f->bbox_.y.min)/8;
	//FillSquareCellColor(cell_f->bbox_, finish_color_, dst);
	VisUtils::FillRectangularArea(dst, cell_f->bbox_, finish_color_);
	putText(dst, "F" ,Point(x,y), CV_FONT_NORMAL, 1, Scalar(0,0,0),1,1);

	// draw path
	uint64_t x1,y1,x2,y2;
	int thickness = 3;
	int lineType = 8;
	int pathline_thickness = 3;

	for(auto it = path.begin(); it != path.end()-1; it++)
	{
		// consecutive cells
		auto cell1 = (*it);
		auto cell2 = (*(it+1));

		// center coordinates
		x1 = cell1->position_.x;
		y1 = cell1->position_.y;

		x2 = cell2->position_.x;
		y2 = cell2->position_.y;

		line( dst,
				Point(x1,y1),
				Point(x2,y2),
				//Scalar( 237, 149, 100 ),
				Scalar( 255, 153, 51 ),
				pathline_thickness,
				lineType);
	}
}


void GraphVis::VisColorMap(Graph_t<SquareCell*>& graph){
	int n = 25;
	srand(time(NULL));
	std::vector<uchar> r(n,0);
	for (int i =0; i < n; i++){
		r[i] = rand() % 10 +1;
	}

	for (int j= 0; j < n; j++){
		std::cout << r[j] << std::endl;
	}
	Mat mat1d(r);
	Mat mat2d = mat1d.reshape(1, 5);
	mat2d *= (255/10);

	std::cout << mat2d << std::endl;

	Mat image;
	applyColorMap(mat2d, image, COLORMAP_HOT);

	namedWindow("colormap", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
	imshow("colormap", image);
	waitKey(0);
}
