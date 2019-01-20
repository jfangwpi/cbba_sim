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

	TasksList tasks(Global_LTL);
    tasks.GetAllTasks();

	std::cout << "# of independent " << Global_LTL.Num_Tasks_In << std::endl;
	std::cout << "# of dependent " << Global_LTL.Num_Tasks_De << std::endl;
	
	/*** 4. Initialize agents ***/
	std::vector<Agent> agents_group = TaskAssignment::InitializeAgents();
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
		TaskAssignment::communicate(agents_group);
		//Update the history of iteration neighbors
		for (int i = 0; i < agents_group[0].num_agents_; i++)
			agents_group[i].history_.iter_neighbors_his.push_back(agents_group[i].iteration_neighbors_);

		/*** 7. Bundle Operations ***/
		/*** 7.1 Remove the out-bid task from the bundle ***/
		TaskAssignment::bundle_remove(agents_group);
		/*** 7.2 Keep inserting tasks which have not been assigned into the bundle ***/
		TaskAssignment::bundle_add(Global_LTL,grid_graph,agents_group);
		/*** 7.3 Check whether the assignment converge or not ***/
		// Increase the iteration
		for (int i = 0; i < agents_group[0].num_agents_; i++)
			agents_group[i].iter_++;
		succFlag = TaskAssignment::success_checker(agents_group);
		std::cout << "The Flag for success is " << succFlag <<std::endl;
		/*** 7.4 Update the number of interation ***/
		
	}
	cbba_time = clock()-cbba_time;
	std::cout << "CBBA iteration is " << agents_group[0].iter_ << std::endl;
	std::cout << "The running time required is " << double(cbba_time)/CLOCKS_PER_SEC << std::endl;


	std::cout << "RESULT OF CBBA " << std::endl;
	for(int i = 0; i < agents_group[0].num_agents_; i++){
		std::cout << "Vehicle " << i << std::endl;
			std::cout << "Winners for tasks are " << std::endl;
			std::cout << agents_group[i].cbba_z_ << std::endl;
			std::cout << "Highest reward are " << std::endl;
			std::cout << agents_group[i].cbba_y_ << std::endl;
			std::cout << "Path info is " << std::endl;
			std::cout << "Path info is " << std::endl;
			for(auto &b: agents_group[i].cbba_path_)
				std::cout << b << " ";
			std::cout << std::endl;
			std::cout << "Reward for each vehicle is " << std::endl;
			std::cout << agents_group[i].cbba_reward_ << std::endl;
			std::string ltl_formula = tasks.local_formula_recreator(agents_group[i].cbba_path_);
			std::cout << "The specification is " << ltl_formula << std::endl;
			std::cout << std::endl;
	}

	/************************************************************************************************************/
	/**************************************    Synchronization    ***********************************************/
	/************************************************************************************************************/
   
    for (auto &agent:agents_group){
        agent.iteration_neighbors_ = Eigen::MatrixXd::Zero(1, num_agents);
        agent.history_.iter_neighbors_his = {agent.iteration_neighbors_};
    }
	
    std::cout << "=================================================================================== " << std::endl;
    std::cout << "=================================================================================== " << std::endl;
    std::cout << "=================================================================================== " << std::endl;
    std::cout << "=================================================================================== " << std::endl;
    
    long count = 0;
	bool flag = false;
    while (count < 15550){
		std::cout << "=================================================================================== " << std::endl;
   		std::cout << "=================================================================================== " << std::endl;
        std::cout << "Syn iteration t = " << count << std::endl;
		std::cout << "Agent " << agents_group[2].idx_ << std::endl;
		for (auto &b: agents_group[2].cbba_path_){
			std::cout << b << ", ";
		}
		std::cout << std::endl;

        TaskAssignment::bundle_add_dependent(tasks, grid_graph, agents_group);

        std::cout << "After bundle add " << std::endl;
        for (auto &agent: agents_group){
            std::cout << "agent " << agent.idx_ << std::endl;
            std::cout << "winning bids matrix is " << std::endl;
            std::cout << agent.winning_bids_matrix_ << std::endl;
            std::cout << "assignment matrix is " << std::endl;
            std::cout << agent.assignment_matrix_ << std::endl;
			std::cout << "Iteration infor is " << std::endl;
			std::cout << agent.iteration_neighbors_ << std::endl;
            std::cout << "current path is " << std::endl;
            for (auto &b: agent.cbba_path_){
                std::cout << b << ", ";
            }
            std::cout << std::endl;
        }
		
		std::cout << std::endl;
        // CBBA::communicate_dependent_decentralized(agents_group, tasks);
        TaskAssignment::communicate_dependent(agents_group, tasks);
        std::cout << "After communication " << std::endl;
        for (auto &agent: agents_group){
            std::cout << "agent " << agent.idx_ << std::endl;
            std::cout << "winning bids matrix is " << std::endl;
            std::cout << agent.winning_bids_matrix_ << std::endl;
            std::cout << "assignment matrix is " << std::endl;
            std::cout << agent.assignment_matrix_ << std::endl;
			std::cout << "Iteration infor is " << std::endl;
			std::cout << agent.iteration_neighbors_ << std::endl;
            std::cout << "current path is " << std::endl;
            for (auto &b: agent.cbba_path_){
                std::cout << b << ", ";
            }
            std::cout << std::endl;
        }

        TaskAssignment::path_remove_dependent(tasks, agents_group);
        
		std::cout << "After bundle remove " << std::endl;
        for (auto &agent: agents_group){
            std::cout << "agent " << agent.idx_ << std::endl;
            std::cout << "winning bids matrix is " << std::endl;
            std::cout << agent.winning_bids_matrix_ << std::endl;
            std::cout << "assignment matrix is " << std::endl;
            std::cout << agent.assignment_matrix_ << std::endl;
			std::cout << "Iteration infor is " << std::endl;
			std::cout << agent.iteration_neighbors_ << std::endl;
            std::cout << "current path is " << std::endl;
            for (auto &b: agent.cbba_path_){
                std::cout << b << ", ";
            }
            std::cout << std::endl;
        }

        count = count + 1;

		flag = TaskAssignment::success_checker_dependent(agents_group, tasks);
		std::cout << "THE SUCCESS FLAG IS " << flag << std::endl;
		if(flag == true){
			std::cout << "Iteration " << count << std::endl;
			break;
		}
		
    }

	TaskAssignment::WaitingTimeCalculation(grid_graph, agents_group, tasks);


    /*** 8. Visualization ***/
	Path_t<SquareCell*> path_origin[num_agents];
	for (auto it_ag = agents_group.begin(); it_ag != agents_group.end(); it_ag++){
		
		/*** 8.1 Rebuild the local LTL specification based on current bundle/path ***/
		std::string ltl_formula = tasks.local_formula_recreator((*it_ag).cbba_path_);
		std::cout << "The specification is " << ltl_formula << std::endl;
		std::vector<std::vector<std::string>> buchi_regions = LTLDecomposition::ObtainBuchiRegion({ltl_formula});

		/*** 8.2 Generate the corresponding buchi regions ***/
        std::shared_ptr<Graph_t<BuchiState *, std::vector<int64_t>>> buchi_graph = BuchiAutomaton::BuildBuchiGraph(ltl_formula,buchi_regions.front());
        std::vector<int64_t> buchi_acc = (*buchi_graph->FindVertex(0)).state_->acc_state_idx;
		//std::shared_ptr<Graph_t<BuchiState>> buchi_graph = BuchiAutomaton::CreateBuchiGraph(ltl_formula,buchi_regions.front());

		/*** 8.4 Construct a product graph ***/
        std::shared_ptr<Graph_t<ProductState *>> product_graph = std::make_shared<Graph_t<ProductState *>>();
        int64_t start_id_grid = (*it_ag).init_pos_;
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
	// Image Layouts: square grid -> graph -> path
	GraphVis vis;
	Mat vis_img;
	vis.VisSquareGrid(*grid, vis_img);
	vis.VisSquareGridGraph(*grid_graph, vis_img, vis_img, false);
	// // put the path on top of the graph
	for (int i = 0; i < agents_group[0].num_agents_; i++){
		vis.VisSquareGridPath(path_origin[i], vis_img, vis_img, i);
	}
	
	// display visualization result
	namedWindow("Processed Image", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
	imshow("Processed Image", vis_img);
	std::string pic_name = "result_cbba_syn_" + std::to_string(agents_group[0].num_agents_) + "v.jpg";
	imwrite(pic_name,vis_img);

	waitKey(0);
    return 0;
}