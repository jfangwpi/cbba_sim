#ifndef GRAPH_VIS_HPP
#define GRAPH_VIS_HPP

#include <vector>
#include <cmath>
#include <stdlib.h>

#include "opencv2/opencv.hpp"
#include "map/square_grid.hpp"
#include "graph/graph.hpp"
#include "graph/vertex.hpp"

namespace librav {

enum class TreeVisType
{
	FREE_SPACE,
	OCCU_SPACE,
	ALL_SPACE
};

class GraphVis
{
private:
	static cv::Scalar bk_color_;		// background color
	static cv::Scalar ln_color_;		// line color
	static cv::Scalar obs_color_;		// obstacle color
    static cv::Scalar aoi_color_;		// area of interest color
    static cv::Scalar unkn_color_;      // unknown cell color
	static cv::Scalar start_color_; 	// starting cell color
	static cv::Scalar finish_color_;	// finishing cell color
	static cv::Scalar v0_color_;  		// vehicle 0 cell color
	static cv::Scalar v1_color_; 		// vehicle 1 cell color
	static cv::Scalar v2_color_; 		// vehilce 2 cell color
	static cv::Scalar v3_color_; 		// vehicle 3 cell color
	static cv::Scalar v4_color_; 		// vehilce 2 cell color
	static cv::Scalar v5_color_; 		// vehicle 3 cell color
	static cv::Scalar v6_color_; 		// vehilce 2 cell color
	static cv::Scalar v7_color_; 
	static cv::Scalar v8_color_; 		// vehilce 2 cell color
	static cv::Scalar v9_color_; 

public:
	// square grid visualization
	static void VisSquareGrid(const SquareGrid& grid, cv::OutputArray _dst);
	// graph visualizationstatic
	static void VisSquareGridGraph(Graph_t<SquareCell*>& graph, cv::InputArray _src, cv::OutputArray _dst, bool show_id);
	static void VisSquareGridPath(Path_t<SquareCell *>& path, cv::InputArray _src, cv::OutputArray _dst);
	static void VisSquareGridPath(Path_t<SquareCell *>& path, cv::InputArray _src, cv::OutputArray _dst, int64_t idx);
	// For information gain
	static void VisColorMap(Graph_t<SquareCell*>& graph);


	
};

}

#endif /* GRAPH_VIS_HPP */
