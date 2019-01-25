/*
 * test_syn_v3.cpp
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

	CBBATasks tasks(Global_LTL);
    tasks.GetAllTasks();

	std::cout << "# of independent " << Global_LTL.Num_Tasks_In << std::endl;
	std::cout << "# of dependent " << Global_LTL.Num_Tasks_De << std::endl;
	
	/*** 4. Initialize agents ***/
	std::vector<cbba_Agent> agents_group = CBBA::InitializeAgents();
	int num_agents = agents_group.size();
	

    /*** 5. Construct a graph from the square grid ***/
	std::shared_ptr<Graph_t<SquareCell *>> grid_graph = GraphFromGrid::BuildGraphFromSquareGrid(grid,false, false);

	/************************************************************************************************************/
	/**************************************          CBBA         ***********************************************/
	/************************************************************************************************************/
	bool succFlag = 0;
	clock_t cbba_time;
	cbba_time = clock();
	while (succFlag != 1){
	//while(agent[0].Iter < 2){
		/*** 6. Communication among neighbors ***/
		std::cout << "Iteration is " << agents_group[0].iter_ << "********************************" << std::endl;
		CBBA::communicate(agents_group);
		//Update the history of iteration neighbors
		for (int i = 0; i < agents_group[0].num_agents_; i++)
			agents_group[i].history_.iter_neighbors_his.push_back(agents_group[i].iteration_neighbors_);

		/*** 7. Bundle Operations ***/
		/*** 7.1 Remove the out-bid task from the bundle ***/
		CBBA::bundle_remove(agents_group);
		/*** 7.2 Keep inserting tasks which have not been assigned into the bundle ***/
		CBBA::bundle_add(Global_LTL,grid_graph,agents_group);
		/*** 7.3 Check whether the assignment converge or not ***/
		succFlag = CBBA::success_checker(agents_group);
		std::cout << "The Flag for success is " << succFlag <<std::endl;
		/*** 7.4 Update the number of interation ***/
		
		// Increase the iteration
		for (int i = 0; i < agents_group[0].num_agents_; i++)
			agents_group[i].iter_++;

	}
	cbba_time = clock()-cbba_time;
	std::cout << "CBBA iteration is " << agents_group[0].iter_ << std::endl;
	std::cout << "The running time required is " << double(cbba_time)/CLOCKS_PER_SEC << std::endl;


	std::cout << "RESULT OF CBBA " << std::endl;
	for(int i = 0; i < agents_group[0].num_agents_; i++){
		std::cout << "Vehicle " << i << std::endl;
		std::cout << "Winners for tasks are " << std::endl;
		for (auto &e: agents_group[i].cbba_z_)
			std::cout << e << " ";
		std::cout << std::endl;
		std::cout << "Highest reward are " << std::endl;
		for(auto &t: agents_group[i].cbba_y_)
			std::cout << t << " ";
		std::cout << std::endl;
		std::cout << "Reward for each vehicle is " << std::endl;
		for(auto &y: agents_group[i].cbba_award_)
			std::cout << y << " ";
		std::cout << std::endl;
		std::cout << "Path info is " << std::endl;
		for(auto &b: agents_group[i].cbba_path_)
			std::cout << b << " ";
		std::cout << std::endl;
	}



	/************************************************************************************************************/
	/**************************************    Synchronization    ***********************************************/
	/************************************************************************************************************/
    std::vector<int> dependent_tasks_idx = {4, 5};
    for (auto &agent: agents_group){
        agent.bundle_add_dependent(tasks, dependent_tasks_idx, grid_graph);
		agent.update_dependent_path_length(tasks, dependent_tasks_idx, grid_graph);
    }

	for (auto &agent:agents_group){
		for (auto &neighbor: agents_group){
			agent.winning_bids_matrix_(4,neighbor.idx_) = neighbor.winning_bids_matrix_(4, neighbor.idx_);
			agent.winning_bids_matrix_(5,neighbor.idx_) = neighbor.winning_bids_matrix_(5, neighbor.idx_);
		}
	}

	for (auto &agent: agents_group){
		agent.dependent_tasks_convergence(4, 2, agents_group);
		agent.dependent_tasks_convergence(5, 3, agents_group);
	}

    for (auto &agent: agents_group){
        std::cout << "For vehicle " << agent.idx_ << std::endl;
		std::cout << "reward is " << std::endl;
		for(auto &c:agent.cbba_award_){
			std::cout << c << " ";
		}
		std::cout << std::endl;

		std::cout << "winning_bids_matrix is " << std::endl;
		std::cout << agent.winning_bids_matrix_ << std::endl;

		for (auto &b: agent.cbba_path_){
            std::cout << b << " ";
        }
        std::cout << std::endl;
		
		// std::cout << "assignment matrix is  " << std::endl;
		// std::cout << agent.assignment_matrix_ << std::endl;

		std::cout << "============================================== " << std::endl;
    }




    return 0;
}