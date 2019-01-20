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
#include "cbta/graph_lift.hpp"

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
	Mat input_image;

	/*** 1. Create a empty square grid ***/
	int row_num = 3;
	int col_num = 3;

	std::shared_ptr<SquareGrid> grid = GraphFromGrid::CreateSquareGrid(row_num,col_num,50);

	// // assign properties of square cells
    // for (int i = 71; i < 77; i++){
    //     grid->SetObstacleRegionLabel(i,1);
    // }

    // grid->SetInterestedRegionLabel(11,2);
	// grid->SetInterestedRegionLabel(65,3);

	/*** 2. Construct a graph from the square grid ***/
    
	std::shared_ptr<Graph_t<SquareCell*>> graph = GraphFromGrid::BuildGraphFromSquareGrid(grid, false, true);


    /*** 4. Construct a lifted graph ***/
	int historyH = 2;
	//std::shared_ptr<Graph_t<LiftedSquareCell>> lifted_graph = GraphLifter::CreateLiftedGraph(HistoryH, graph);
    std::cout << "Start lifted graph " << std::endl;
    std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph = GraphLifter::BuildLiftedGraph(historyH, graph);
    std::cout << "End lifted graph" << std::endl;
   
    

	

	/************************************************************************************/
	/* Below this point, a SquareGrid object should be available for graph construction */
	/************************************************************************************/

	/*** Construct a graph from the square grid ***/
	/*** the second argument determines if move along diagonal is allowed ***/
	

	/*** Visualize the map and graph ***/
	// Mat vis_img;
	// /*** Image Layouts: (map) -> square grid -> graph -> path ***/
	// /*** you can visualize the squre grid by itself or overlay it on the map image ***/
	
	// GraphVis::VisSquareGrid(*grid, vis_img);
	// GraphVis::VisSquareGridGraph(*graph, vis_img, vis_img, true);

	// // display visualization result
	// namedWindow("Processed Image", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
	// imshow("Processed Image", vis_img);

	waitKey(0);

	/*** uncomment this line if you want to save result into an image ***/
	//imwrite( "examples_result.jpg", vis_img);

	return 0;
}