/*
 * test_path_length.cpp
 *
 *  Created on: Aug 25, 2018
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

    std::vector<int> path_ = {0,2};
	int idx = 1;
	std::vector<std::vector<int>> path_copy(path_.size()+1, path_);
	for (int i = 0; i < path_copy.size(); i++){
		path_copy[i].insert(path_copy[i].begin()+i, idx);
	}

	// Case 1: original method
	std::vector<float> c_reward_origin = {};
	for (int j = 0; j < path_copy.size(); j++){
		float length = CBBA::PathLengthCalculation(Global_LTL,grid_graph, path_copy[j] ,agents_group[0].start_node_);
		c_reward_origin.push_back(length);
	}

	float c_reward_min = 1000.0;
	int insert_pos = -1;
	for(int k = 0; k < path_copy.size(); k++){
		if(c_reward_origin[k] <= c_reward_min){
			c_reward_min = c_reward_origin[k];
			insert_pos = k;
		}
	}

	std::cout << "=====================================" << std::endl;
	std::cout << "The original way: " << std::endl;
	std::cout << "The minimum reward is " << c_reward_min << " , the best insert pos is " << insert_pos << std::endl;



	// Case 2: 
	float original_length = CBBA::PathLengthCalculation(Global_LTL, grid_graph, path_,agents_group[0].start_node_);
	std::vector<float> c_reward_new = {};
	for (int m = 0; m < path_copy.size(); m++){
		float length = CBBA::PathLengthCalculation(Global_LTL, grid_graph, path_copy[m],agents_group[0].start_node_);
		float reward = length - original_length;
		std::cout << "The reward is " << reward << std::endl;
		c_reward_new.push_back(reward);
	}

	float c_reward_max = 0.0;
	int insert_pos_new = -1;
	for (int l = 0; l < path_copy.size(); l++){
		if(c_reward_new[l] >= c_reward_max){
			c_reward_max = c_reward_new[l];
			insert_pos_new = l;
		}
	}

	std::cout << "=====================================" << std::endl;
	std::cout << "The new way: " << std::endl;
	std::cout << "The minimum reward is " << c_reward_max << " , the best insert pos is " << insert_pos_new << std::endl;

	
}