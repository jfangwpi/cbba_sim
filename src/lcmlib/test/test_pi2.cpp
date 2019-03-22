// standard library
#include <stdio.h>
#include <vector>
#include <ctime>
#include <tuple>
#include <algorithm>
#include <bitset>
#include <iostream>
#include <fstream>

// self-defined library
#include "graph/graph.hpp"
#include "graph/algorithms/astar.hpp"
#include "map/square_grid.hpp"
#include "vis/graph_vis.hpp"

#include "ltl/buchi_automaton.hpp"
#include "ltl/product_automaton.hpp"

#include "cbba/cbba_agent.hpp"
#include "cbba/cbba_task.hpp"

#include "lcmlib/msg_exchange.hpp"
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/cbba_lcm_msgs.hpp"

using namespace librav;
int main(int argc, char** argv )
{   
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
    CBBATasks tasks(Global_LTL);
    tasks.GetAllTasks();
	
	/*** 3. Initialize agents ***/
    int agent_id = 1;
    int agent_init_pos = 24;
    std::vector<int> comm = {1, 1};
    int num_tasks_inde = 2;
    int num_tasks_de = 0;
    cbba_Agent agent = cbba_Agent(agent_id, agent_init_pos, comm, num_tasks_inde, num_tasks_de);
    
    /*** 4. Construct a graph from the square grid ***/
	std::shared_ptr<Graph_t<SquareCell *>> grid_graph = GraphFromGrid::BuildGraphFromSquareGrid(grid,false, false);

    std::cout << "================== Vehicle Information ==================" << std::endl;
    std::cout << "Vehicle index: " << agent.idx_ << std::endl;
    std::cout << "Initial position: " << agent.start_node_ << std::endl;
    std::cout << "=========================================================" << std::endl;
    std::cout << std::endl;
    /************************************************************************************************************/
	/*************************************          Communicate         *****************************************/
	/************************************************************************************************************/
    int count = 0;
    int communicate_dia = 3;
    std::cout << "=========================================================" << std::endl;
    std::cout << "========================== CBBA ========================="<<std::endl;
    std::cout << "=========================================================" << std::endl;
    while (count < communicate_dia){
        std::cout << "++++++++++++++ Iteration " << count << " ++++++++++++++" << std::endl;
        //----------------------------- Reward Update -----------------------------//
        //-------------------------------------------------------------------------//
        //-------------------------------------------------------------------------//
        /*** 7. Bundle Operations ***/
		/*** 7.1 Remove the out-bid task from the bundle ***/
		CBBA::bundle_remove(agent);
		/*** 7.2 Keep inserting tasks which have not been assigned into the bundle ***/
		CBBA::bundle_add(Global_LTL,grid_graph,agent);

        /*** Print out the reward ***/
        std::cout << "Bundle Construction Phase: " << std::endl;
        std::cout << "Highest rewards of independent tasks that vehicle " << agent.idx_ << " had: "<< std::endl; 
        for(auto &e: agent.cbba_y_){
            std::cout << e << ", ";
        }
        std::cout << std::endl;
        std::cout << "Assignment of independent tasks that vehcile " << agent.idx_ << " had: " << std::endl;
         for(auto &t: agent.cbba_z_){
            std::cout << t << ", ";
        }
        std::cout << std::endl;
        std::cout << "Current task path for vehcile " << agent.idx_ << " is: ";
        for (auto &b: agent.cbba_path_){
            std::cout << b << ", ";
        }
        std::cout << std::endl;


        //----------------------------- Communication -----------------------------//
        //-------------------------------------------------------------------------//
        //-------------------------------------------------------------------------//
        MSGHandler handlerObject({0});
        handlerObject.MSGExchange(agent);

        bool flag = 0;
        std::cout << "Enter 1, if all vehicles are ready for next iteration. " << std::endl;
        while (flag == 0){
            std::cin >> flag;
        }
       
        for (auto &neg: handlerObject.neigh_list){
            CBBA::communicate_lcm(agent, neg, handlerObject.y_his[neg], handlerObject.z_his[neg], handlerObject.iter_nei_his[neg]);    
        }
        agent.history_.iter_neighbors_his.push_back(agent.iteration_neighbors_);
        agent.iter_ = agent.iter_ + 1;

        std::cout << std::endl;
        std::cout << "After Consensus Phase: " << std::endl;
        std::cout << "Highest rewards of independent tasks that vehicle " << agent.idx_ << " had: "<< std::endl; 
        for(auto &e: agent.cbba_y_){
            std::cout << e << ", ";
        }
        std::cout << std::endl;
        std::cout << "Assignment of independent tasks that vehcile " << agent.idx_ << " had: " << std::endl;
         for(auto &t: agent.cbba_z_){
            std::cout << t << ", ";
        }
        std::cout << std::endl;
        std::cout << "Current task path for vehcile " << agent.idx_ << " is: ";
        for (auto &b: agent.cbba_path_){
            std::cout << b << ", ";
        }
        std::cout << std::endl;
        std::cout << std::endl;

        count ++;
    }

std::cout << std::endl;
    std::cout << "=============== Task Assignment Completed ===============" << std::endl;
    std::cout << "The local specification for vehicle " << agent.idx_ << " is: " << std::endl;
    std::string local_ltl = LTLDecomposition::subtask_recreator(agent.cbba_path_,true, Global_LTL);
    std::cout << local_ltl << std::endl;

    Path_t<SquareCell*> path_origin;
    /*** 8.2 Generate the corresponding buchi regions ***/
    std::vector<std::vector<std::string>> buchi_regions = LTLDecomposition::ObtainBuchiRegion({local_ltl});
    std::shared_ptr<Graph_t<BuchiState *, std::vector<int64_t>>> buchi_graph = BuchiAutomaton::BuildBuchiGraph(local_ltl,buchi_regions.front());
    std::vector<int64_t> buchi_acc = (*buchi_graph->FindVertex(0)).state_->acc_state_idx;
    //std::shared_ptr<Graph_t<BuchiState>> buchi_graph = BuchiAutomaton::CreateBuchiGraph(ltl_formula,buchi_regions.front());

    /*** 8.4 Construct a product graph ***/
    std::shared_ptr<Graph_t<ProductState *>> product_graph = std::make_shared<Graph_t<ProductState *>>();
    int64_t start_id_grid = agent.start_node_;
    int64_t virtual_start_state_id = ProductAutomaton::SetVirtualStartState(product_graph, buchi_graph, grid_graph, start_id_grid);
    //std::shared_ptr<Graph_t<ProductState>> product_graph_new = std::make_shared<Graph_t<ProductState>>();

    GetProductNeighbor product_neighbor(grid_graph, buchi_graph);
    auto path = AStar::ProductIncSearch(product_graph, virtual_start_state_id, buchi_acc, GetNeighbourFunc_t<ProductState*, double>(product_neighbor));
    
    if (!agent.cbba_path_.empty()){
        for(auto &e: path){
            path_origin.push_back(e->grid_vertex_->state_);
        }
        std::cout << "The path length for vehicle "  << agent.idx_ << " is " << path_origin.size() << std::endl;
    }
    else{
        path_origin.push_back(path[0]->grid_vertex_->state_);
    }


    std::cout << std::endl;
	std::cout << "Path information for testing is " << std::endl;
	for (auto &p_cell: path_origin){
		std::cout << p_cell->id_ << " ->";
    }
	std::cout << std::endl;

    std::ofstream pfile;
    std::string file_name = "path_" + std::to_string(agent.idx_ + 1) + ".txt";
    pfile.open(file_name);
    for (auto &p_cell: path_origin){
		pfile << p_cell->id_ << "\n";
    }
    pfile.close();



    return 0;
}
