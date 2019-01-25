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
    std::vector<int> dependent_tasks_idx = {3};
    for (auto &agent:agents_group){
        agent.iteration_neighbors_ = std::vector<int>(agent.num_agents_, 0);
        agent.history_.iter_neighbors_his = {agent.iteration_neighbors_};
    }
	
    std::cout << "=================================================================================== " << std::endl;
    std::cout << "=================================================================================== " << std::endl;
    std::cout << "=================================================================================== " << std::endl;
    std::cout << "=================================================================================== " << std::endl;
    
    for (auto &agent: agents_group){
        agent.bundle_add_dependent(tasks, dependent_tasks_idx, grid_graph);
		agent.update_dependent_path_length(tasks, dependent_tasks_idx, grid_graph);
        agent.iteration_neighbors_[agent.idx_]++;

        agent.history_.assignment_.push_back(agent.assignment_matrix_);
        agent.history_.iter_neighbors_his.push_back(agent.iteration_neighbors_);
        agent.history_.winning_bids_.push_back(agent.winning_bids_matrix_);
    }

    std::cout << "At the beginning of SYN " << std::endl;
    for (auto &agent: agents_group){
        std::cout << "================================================================ " << std::endl;
        std::cout << "Agent " << agent.idx_ << std::endl;
        std::cout << "winning bid is " << std::endl;
        std::cout << agent.winning_bids_matrix_ << std::endl;
        std::cout << "assignment is " << std::endl;
        std::cout << agent.assignment_matrix_ << std::endl;

        std::cout << "PATH pi skr skr skr skr skr skr skr " << std::endl;
        for (auto &bb: agent.cbba_path_){
            std::cout << bb << " ";
        }
        std::cout << std::endl;
    }

    int communicate_dia = 3;
    int syn_task_idx = 3;
    int count = 0;
    cbba_Task& task = tasks.FindTask(syn_task_idx);
    while (count < communicate_dia){
        std::cout << "ITERATION " << count << "================================================================================ " << std::endl;
        CBBA::communicate_dependent_decentralized(agents_group, task);
        count++;
    }


    // std::cout << "TASK 5 +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ " << std::endl;
    // syn_task_idx = 5;
    // count = 0;
    // cbba_Task& task1 = tasks.FindTask(syn_task_idx);
    // while (count < communicate_dia){
    //     CBBA::communicate_dependent_decentralized(agents_group, task1);
    //     count++;
    // }

    std::cout << "After COMM " << std::endl;
    for (auto &agent: agents_group){
        agent.assignment_update(dependent_tasks_idx);
        std::cout << "================================================================ " << std::endl;
        std::cout << "Agent " << agent.idx_ << std::endl;
        std::cout << "winning bid is " << std::endl;
        std::cout << agent.winning_bids_matrix_ << std::endl;
        std::cout << "assignment is " << std::endl;
        std::cout << agent.assignment_matrix_ << std::endl;

        std::cout << "PATH pi" << std::endl;
        for (auto &bb: agent.cbba_path_){
            std::cout << bb << " ";
        }
        std::cout << std::endl;
    }


    /*** 8. Visualization ***/
	Path_t<SquareCell*> path_origin[num_agents];
	for (auto it_ag = agents_group.begin(); it_ag != agents_group.end(); it_ag++){
		
		/*** 8.1 Rebuild the local LTL specification based on current bundle/path ***/
		std::string ltl_formula = LTLDecomposition::subtask_recreator((*it_ag).cbba_path_,true, Global_LTL);
		std::cout << "The specification is " << ltl_formula << std::endl;
		std::vector<std::vector<std::string>> buchi_regions = LTLDecomposition::ObtainBuchiRegion({ltl_formula});

		/*** 8.2 Generate the corresponding buchi regions ***/
        std::shared_ptr<Graph_t<BuchiState *, std::vector<int64_t>>> buchi_graph = BuchiAutomaton::BuildBuchiGraph(ltl_formula,buchi_regions.front());
        std::vector<int64_t> buchi_acc = (*buchi_graph->FindVertex(0)).state_->acc_state_idx;
		//std::shared_ptr<Graph_t<BuchiState>> buchi_graph = BuchiAutomaton::CreateBuchiGraph(ltl_formula,buchi_regions.front());

		/*** 8.4 Construct a product graph ***/
        std::shared_ptr<Graph_t<ProductState *>> product_graph = std::make_shared<Graph_t<ProductState *>>();
        int64_t start_id_grid = (*it_ag).start_node_;
        int64_t virtual_start_state_id = ProductAutomaton::SetVirtualStartState(product_graph, buchi_graph, grid_graph, start_id_grid);
		//std::shared_ptr<Graph_t<ProductState>> product_graph_new = std::make_shared<Graph_t<ProductState>>();

        GetProductNeighbor product_neighbor(grid_graph, buchi_graph);
        auto path = AStar::ProductIncSearch(product_graph, virtual_start_state_id, buchi_acc, GetNeighbourFunc_t<ProductState*, double>(product_neighbor));
        
		if (!(*it_ag).cbba_path_.empty()){
        	for(auto &e: path){
            	path_origin[(*it_ag).idx_].push_back(e->grid_vertex_->state_);
       	 	}
			std::cout << "The path length for vehicle "  << (*it_ag).idx_ << " is " << path_origin[(*it_ag).idx_].size() << std::endl;
		}
		else{
			path_origin[(*it_ag).idx_].push_back(path[0]->grid_vertex_->state_);
		}
		
	}



	/*** 9.Visualize the map and graph ***/
	// // Image Layouts: square grid -> graph -> path
	// GraphVis vis;
	// Mat vis_img;
	// vis.VisSquareGrid(*grid, vis_img);
	// vis.VisSquareGridGraph(*grid_graph, vis_img, vis_img, true);
	// // // put the path on top of the graph
	// vis.VisSquareGridPath(path_origin[0], vis_img, vis_img);
	// vis.VisSquareGridPath(path_origin[1], vis_img, vis_img);
	// vis.VisSquareGridPath(path_origin[2], vis_img, vis_img);
	// vis.VisSquareGridPath(path_origin[3], vis_img, vis_img);
	
	// display visualization result
	//namedWindow("Processed Image", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
	//imshow("Processed Image", vis_img);
	// imwrite("result_cbba.jpg",vis_img);


    for (auto &agent:agents_group){
        GraphVis vis;
        Mat vis_img;
        vis.VisSquareGrid(*grid, vis_img);
        vis.VisSquareGridGraph(*grid_graph, vis_img, vis_img, true);
        // // put the path on top of the graph
        vis.VisSquareGridPath(path_origin[agent.idx_], vis_img, vis_img);
        std::string jgp_name = "result_syn_" + std::to_string(agent.idx_) + ".jpg";
        imwrite(jgp_name,vis_img);
    }

	waitKey(0);




    

    
    






    return 0;
}