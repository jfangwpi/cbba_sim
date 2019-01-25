/*
 * cbba_example.cpp
 *
 *  Created on: Feb 15, 2017
 *      Author: jfang
 */

/*** This code is only for independent tasks ***/
// standard libaray
#include <stdio.h>
#include <vector>
#include <ctime>
#include <tuple>
#include <algorithm>
#include <bitset>
// opencv
#include "opencv2/opencv.hpp"

// self-defined library
#include "graph/graph.hpp"
#include "graph/algorithms/astar.hpp"
#include "map/square_grid.hpp"
#include "vis/graph_vis.hpp"

#include "ltl/buchi_automaton.hpp"
#include "ltl/product_automaton.hpp"

#include "cbba/cbba_agent.hpp"
#include "cbba/cbba_task.hpp"

using namespace cv;
using namespace librav;

int main(int argc, char** argv )
{

	/************************************************************************************************************/
	/*********************************          Initialize: Map         *****************************************/
	/************************************************************************************************************/
	/*** 1. Create a empty square grid ***/
	std::shared_ptr<SquareGrid> grid = GraphFromGrid::CreateSquareGrid();
	
	/*** 3. Define the target regions ***/
	// Define LTL specification
	LTLFormula Global_LTL;
	for (int i = 0; i < Global_LTL.task_info.size();i++)
		grid->SetInterestedRegionLabel(Global_LTL.task_info[i].pos_,Global_LTL.task_info[i].idx_);	
	// Decompose the global LTL_expression
	LTLDecomposition::GlobalLTLDecomposition(Global_LTL);
	
	/*** 4. Initialize agents ***/
	std::vector<cbba_Agent> agents_group = CBBA::InitializeAgents();
	int num_agents = agents_group.size();

    /*** 5. Construct a graph from the square grid ***/
	std::shared_ptr<Graph_t<SquareCell *>> grid_graph = GraphFromGrid::BuildGraphFromSquareGrid(grid,false, false);


    std::vector<Vertex_t<SquareCell*>*> path_origin[num_agents];
    Path_t<SquareCell *> path_vis[num_agents];
    for (int i=0; i < num_agents; i++){
        Vertex_t<SquareCell *> * start_node_origin = grid_graph->GetVertexFromID(agents_group[i].start_node_);
        path_origin[i].push_back(start_node_origin);

        for (auto &ee: path_origin[i]){
            path_vis[i].push_back(ee->state_);
        }
    }

	/*** 9.Visualize the map and graph ***/
	// Image Layouts: square grid -> graph -> path
	GraphVis vis;
	Mat vis_img;
	vis.VisSquareGrid(*grid, vis_img);
	vis.VisSquareGridGraph(*grid_graph, vis_img, vis_img, true);
	// // put the path on top of the graph
	vis.VisSquareGridPath(path_vis[0], vis_img, vis_img);
	vis.VisSquareGridPath(path_vis[1], vis_img, vis_img);
	vis.VisSquareGridPath(path_vis[2], vis_img, vis_img);
	// vis.VisSquareGridPath(path_vis[3], vis_img, vis_img);
	
	// display visualization result
	//namedWindow("Processed Image", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
	//imshow("Processed Image", vis_img);
	imwrite("result_map_pi.jpg",vis_img);

	waitKey(0);

	return 0;

}