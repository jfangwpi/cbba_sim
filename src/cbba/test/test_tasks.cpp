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
	TasksList tasks(Global_LTL);
    tasks.GetAllTasks();
	
    std::vector<int> bundle = {0,2,1};
    std::string local_formula = tasks.local_task_recreator(bundle);
    std::cout << local_formula << std::endl;
	waitKey(0);

	return 0;

}