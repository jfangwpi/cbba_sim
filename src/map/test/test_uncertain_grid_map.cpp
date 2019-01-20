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