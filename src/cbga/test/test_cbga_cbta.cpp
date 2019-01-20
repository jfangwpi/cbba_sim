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

#include "cbga/cbga_agent.hpp"
#include "cbga/cbga_task.hpp"
#include "cbba/cbba_task.hpp"

#include "cbta/graph_lift.hpp"
#include "cbta/hcost_interface.hpp"


using namespace cv;
using namespace librav;

int main(int argc, char** argv){
   	/*** 0. Preprocessing CBTA data ***/
	double r_min = 4;
	TileTraversalData tile_traversal_data = HCost::hcost_preprocessing(r_min);

    /************************************************************************************************************/
	/*********************************          Initialize: Map         *****************************************/
	/************************************************************************************************************/
	/*** 1. Create a empty square grid ***/
	std::shared_ptr<SquareGrid> grid = GraphFromGrid::CreateSquareGrid();
	
	/*** 2. Define the target regions ***/
	// Define LTL specification
	LTLFormula Global_LTL;
	for (int i = 0; i < Global_LTL.task_info.size();i++)
		grid->SetInterestedRegionLabel(Global_LTL.task_info[i].pos_,Global_LTL.task_info[i].idx_);	
	// Decompose the global LTL_expression
	LTLDecomposition::GlobalLTLDecomposition(Global_LTL);

    CBGATasks tasks(Global_LTL);
    tasks.GetAllTasks();

	/*** 3. Construct a graph from the square grid ***/
	std::shared_ptr<Graph_t<SquareCell *>> grid_graph = GraphFromGrid::BuildGraphFromSquareGrid(grid, false, true);

	/*** 4. Initialize the info of agents ***/
	std::vector<cbga_Agent> agents =  CBGA::InitializeAgents();


    /************************************************************************************************************/
	/*********************************          Pre-Porcess for CBTA         ************************************/
	/************************************************************************************************************/
	int historyH = 4;
	//std::shared_ptr<Graph_t<LiftedSquareCell>> lifted_graph = GraphLifter::CreateLiftedGraph(HistoryH, graph);
    std::cout << "Start lifted graph " << std::endl;
    std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph = GraphLifter::BuildLiftedGraph(historyH, grid_graph);
	std::cout << "End lifted graph" << std::endl;


    /************************************************************************************************************/
	/******************************************          CBGA         *******************************************/
	/************************************************************************************************************/
	/*** 5. Start CBGA ***/
	bool flag = false;
	while(flag == false){
		CBGA::bundle_construction(tasks, Global_LTL, grid_graph, lifted_graph, tile_traversal_data, grid, agents);
		
		// Increase the iteration
		for (int i = 0; i < agents[0].num_agents_; i++){
			agents[i].iter_++;
			agents[i].iteration_neighbors_[agents[i].idx_] = agents[i].iter_;
			agents[i].history_.iter_neighbors_his.push_back(agents[i].iteration_neighbors_);
		}
		
		CBGA::consensus(agents, tasks);
		CBGA::update_iteration_number(agents);
		CBGA::bundle_remove(agents);
		flag = CBGA::success_checker(agents, tasks);
		std::cout << "Iteration is " << agents[0].iter_ << std::endl;

	}

	std::cout << "Final Result " << std::endl;
	for (auto &agent:agents){
		std::cout << "For agent " << agent.idx_ << std::endl;
		std::cout << "Path is " << std::endl;
		for (auto &e: agent.cbga_path_){
			std::cout << e << " ";
		}
		std::cout << std::endl;
		// std::cout << "winning bid matrix is " << std::endl;
		// std::cout << agent.winning_bids_matrix_ << std::endl;
		// std::cout << "assignment matrix is " << std::endl;
		// std::cout << agent.assignment_matrix_ << std::endl;
	}
	return 0;	
}