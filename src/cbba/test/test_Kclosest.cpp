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
	
	/*** 4. Initialize agents ***/
	std::vector<Agent> agents_group = TaskAssignment::InitializeAgents();
	int num_agents = agents_group.size();


    /*** 5. Construct a graph from the square grid ***/
	std::shared_ptr<Graph_t<SquareCell *>> grid_graph = GraphFromGrid::BuildGraphFromSquareGrid(grid, false, false);

    agents_group[0].winning_bids_matrix_(1,0) = 91;
    agents_group[0].winning_bids_matrix_(1,1) = 90;
    agents_group[0].winning_bids_matrix_(1,2) = 15;
    agents_group[0].winning_bids_matrix_(1,3) = 7;
    agents_group[0].winning_bids_matrix_(1,4) = 90;
    agents_group[0].winning_bids_matrix_(1,5) = 18;

    agents_group[0].assignment_matrix_(1,2) = 1;
    agents_group[0].assignment_matrix_(1,5) = 1;

    cbba_Task task(1, {10}, TaskType::VISIT, 2, "<> p2", "([] p0) && ([] !p1)", {"p2"});


    agents_group[0].optimal_group_finder(task);
	std::vector<int> winners = agents_group[0].winners_finder(task);
	std::cout << "Result of optimal group is " << std::endl;
	std::cout << agents_group[0].assignment_matrix_ << std::endl;
	std::cout << "The index of winners are ";
	for (auto &e: winners){
		std::cout << e << ", ";
	}
	std::cout << std::endl;
	



	return 0;

}