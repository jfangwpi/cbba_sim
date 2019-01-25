// standard libaray
#include <stdio.h>
#include <vector>
#include <ctime>
#include <cmath>
#include <tuple>
#include <string>

// opencv
#include "opencv2/opencv.hpp"

// user
#include "vis/graph_vis.hpp"
#include "map/square_grid.hpp"

using namespace cv;
using namespace librav;


double CalcHeuristic(SquareCell *node1, SquareCell *node2)
{
    int32_t dist_row = node1->coordinate_.x - node2->coordinate_.x;
    int32_t dist_col = node1->coordinate_.y - node2->coordinate_.y;

    return std::sqrt(dist_row * dist_row + dist_col * dist_col);
}

int main(int argc, char** argv )
{

	/*** 1. Read from config file: map.ini ***/
	std::shared_ptr<SquareGrid> grid = GraphFromGrid::CreateSquareGrid();

    // grid->SetCellOccupancy(5, OccupancyType::OCCUPIED);
    grid->SetCellOccupancy(20, OccupancyType::UNKNOWN);

	/*** 2. Construct a graph from the square grid ***/
	std::shared_ptr<Graph_t<SquareCell*>> graph = GraphFromGrid::BuildGraphFromSquareGrid(grid, false, true);
    
    // GraphVis::VisSquareGridGraph(*graph);
    GraphVis::VisColorMap(*graph);

	return 0;
}