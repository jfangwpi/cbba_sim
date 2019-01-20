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
	for (int i = 0; i < Global_LTL.task_info.size();i++){
		std::cout << "Task " << i << ": " << Global_LTL.task_info[i].pos_ << ", " << Global_LTL.task_info[i].idx_ << std::endl;
		grid->SetInterestedRegionLabel(Global_LTL.task_info[i].pos_,Global_LTL.task_info[i].idx_);
	}	
	// Decompose the global LTL_expression
	LTLDecomposition::GlobalLTLDecomposition(Global_LTL);
	TasksList tasks(Global_LTL);
    tasks.GetAllTasks();
	
	/*** 4. Initialize agents ***/
	std::vector<Agent> agents_group = TaskAssignment::InitializeAgents();
	int num_agents = agents_group.size();

    /*** 5. Construct a graph from the square grid ***/
	std::shared_ptr<Graph_t<SquareCell *>> grid_graph = GraphFromGrid::BuildGraphFromSquareGrid(grid, false, false);

	auto tt = grid_graph->FindVertex(224)->state_->GetCellLabels();
	std::cout << "TT is " << tt << std::endl;

    std::vector<int> path_ = {6};

	/*** 8.2 Generate the corresponding buchi regions ***/
	std::string ltl_formula = tasks.local_formula_recreator(path_);
	std::vector<std::vector<std::string>> buchi_regions = LTLDecomposition::ObtainBuchiRegion({ltl_formula});
    std::shared_ptr<Graph_t<BuchiState *, std::vector<int64_t>>> buchi_graph = BuchiAutomaton::BuildBuchiGraph(ltl_formula,buchi_regions.front());
    
	std::vector<int64_t> buchi_acc = (*buchi_graph->FindVertex(0)).state_->acc_state_idx;
	//std::shared_ptr<Graph_t<BuchiState>> buchi_graph = BuchiAutomaton::CreateBuchiGraph(ltl_formula,buchi_regions.front());

	/*** 8.4 Construct a product graph ***/
    std::shared_ptr<Graph_t<ProductState *>> product_graph = std::make_shared<Graph_t<ProductState *>>();
    int64_t start_id_grid = agents_group[1].init_pos_;
    int64_t virtual_start_state_id = ProductAutomaton::SetVirtualStartState(product_graph, buchi_graph, grid_graph, start_id_grid);
	std::cout << "The virtual start state id is " << virtual_start_state_id << std::endl;
	//std::shared_ptr<Graph_t<ProductState>> product_graph_new = std::make_shared<Graph_t<ProductState>>();

    GetProductNeighbor product_neighbor(grid_graph, buchi_graph);
    auto path = AStar::ProductIncSearch(product_graph, virtual_start_state_id, buchi_acc, GetNeighbourFunc_t<ProductState*, double>(product_neighbor));
    Path_t<SquareCell*> path_origin;
	if (!path_.empty()){
        for(auto &e: path){
            path_origin.push_back(e->grid_vertex_->state_);
       	}
		std::cout << "The path length for vehicle "  << agents_group[1].idx_ << " is " << path_origin.size() << std::endl;
	}
	else{
		path_origin.push_back(path[0]->grid_vertex_->state_);
	}

	
	double l_length = TaskAssignment::PathLengthCalculationBasedType(tasks, grid_graph, path_ ,agents_group[1].init_pos_);
	std::cout << "Length is " << l_length << std::endl;


	/*** Visualize the map and graph ***/
	Mat vis_img;
	/*** Image Layouts: (map) -> square grid -> graph -> path ***/
	/*** you can visualize the squre grid by itself or overlay it on the map image ***/
	GraphVis::VisSquareGrid(*grid, vis_img);
	GraphVis::VisSquareGridGraph(*grid_graph, vis_img, vis_img, true);
	GraphVis::VisSquareGridPath(path_origin, vis_img, vis_img);

	// display visualization result
	namedWindow("Processed Image", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
	imshow("Processed Image", vis_img);

	waitKey(0);

	// int idx = 1;
	// std::vector<std::vector<int>> path_copy(path_.size()+1, path_);
	// for (int i = 0; i < path_copy.size(); i++){
	// 	path_copy[i].insert(path_copy[i].begin()+i, idx);
	// }



	// // Case 1: original method
	// std::vector<float> c_reward_origin = {};
	// for (int j = 0; j < path_copy.size(); j++){
	// 	float length = CBBA::PathLengthCalculation(Global_LTL, grid_graph, path_copy[j] ,agents_group[0].init_pos_);
	// 	c_reward_origin.push_back(length);
	// }

	// float c_reward_min = 1000.0;
	// int insert_pos = -1;
	// for(int k = 0; k < path_copy.size(); k++){
	// 	if(c_reward_origin[k] <= c_reward_min){
	// 		c_reward_min = c_reward_origin[k];
	// 		insert_pos = k;
	// 	}
	// }

	// std::cout << "=====================================" << std::endl;
	// std::cout << "The original way: " << std::endl;
	// std::cout << "The minimum reward is " << c_reward_min << " , the best insert pos is " << insert_pos << std::endl;



	// // Case 2: 
	// float original_length = CBBA::PathLengthCalculation(Global_LTL, grid_graph, path_,agents_group[0].init_pos_);
	// std::vector<float> c_reward_new = {};
	// for (int m = 0; m < path_copy.size(); m++){
	// 	float length = CBBA::PathLengthCalculation(Global_LTL, grid_graph, path_copy[m],agents_group[0].init_pos_);
	// 	float reward = length - original_length;
	// 	std::cout << "The reward is " << reward << std::endl;
	// 	c_reward_new.push_back(reward);
	// }

	// float c_reward_max = 0.0;
	// int insert_pos_new = -1;
	// for (int l = 0; l < path_copy.size(); l++){
	// 	if(c_reward_new[l] >= c_reward_max){
	// 		c_reward_max = c_reward_new[l];
	// 		insert_pos_new = l;
	// 	}
	// }

	// std::cout << "=====================================" << std::endl;
	// std::cout << "The new way: " << std::endl;
	// std::cout << "The minimum reward is " << c_reward_max << " , the best insert pos is " << insert_pos_new << std::endl;

	
}