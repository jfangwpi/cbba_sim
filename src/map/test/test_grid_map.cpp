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
	// for(int i = 1; i<=143; i++)
	// {
	// 	int ID = i;
	// 	int L  = 15;

	// 	std::vector<double> real_coord = grid->GetRealCoordinateFromID(ID,L);
	// 	int64_t id = grid->GetIDFromRealCoordinate(real_coord[0],real_coord[1],L);
	// 	//std::vector<float> test_vector_output = test_output.toVector();
	// 	std::printf("Real Coordinates for ID = %d and side length = %dm,\n x:%f y:%f\n",ID,L,real_coord[0],real_coord[1]);
	// 	std::printf("Calculated ID from coordinates: %d\n-----------------------------------\n", id);
	// }
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
	imwrite( "examples_result.jpg", vis_img);

	return 0;
}