// standard libaray
#include <stdio.h>
#include <vector>
#include <ctime>
#include <tuple>
#include <string>

// opencv
#include "opencv2/opencv.hpp"

// user
#include "vis/graph_vis.hpp"
#include "map/square_grid.hpp"
#include "graph/algorithms/astar.hpp"


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

	/*** 2. Construct a graph from the square grid ***/
	std::shared_ptr<Graph_t<SquareCell*>> graph = GraphFromGrid::BuildGraphFromSquareGrid(grid, false, true);


	/*** Visualize the map and graph ***/
	Mat vis_img;

	GraphVis::VisSquareGrid(*grid, vis_img);
	GraphVis::VisSquareGridGraph(*graph, vis_img, vis_img, true);

	// display visualization result
	namedWindow("Processed Image", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
	imshow("Processed Image", vis_img);

	waitKey(0);

	/*** uncomment this line if you want to save result into an image ***/
	//imwrite( "examples_result.jpg", vis_img);

	return 0;
}