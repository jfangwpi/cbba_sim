// standard library
#include <stdio.h>
#include <vector>
#include <ctime>
#include <tuple>
#include <algorithm>
#include <bitset>

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
    int agent_id = 2;
    int agent_init_pos = 11;
    std::vector<int> comm = {0, 1, 1};
    int num_tasks_inde = 4;
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
        MSGHandler handlerObject({1});
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
    
    return 0;
}

