/*
 * cbga_Agent.h
 *
 *  Created on: Aug 29, 2018
 *      Author: jfang
 */

#ifndef CBGA_AGENT_HPP
#define CBGA_AGENT_HPP


// standard library
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

#include "graph/graph.hpp"
#include "graph/algorithms/astar.hpp"
#include "map/square_grid.hpp"

#include "ltl/buchi_automaton.hpp"
#include "ltl/product_automaton.hpp"
#include "cbba/cbba_task.hpp"
#include "cbta/graph_lift.hpp"
#include "cbta/hcost_interface.hpp"
#include "cbga/cbga_task.hpp"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SparseCore>


namespace librav{

const double eps = 0.0;
const double reward_benefit = 200.0;

/*** Memory info required by CBGA ***/
typedef struct
{
	std::vector<std::vector<int>> iter_neighbors_his;

	std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> assignment_;
	std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> winning_bids_;

}Memo; 

class cbga_Agent
{
public:
	// Constructor
	cbga_Agent(int id, int startID, std::vector<int> com, int num_tasks);
	cbga_Agent(int id, int startID, int num_tasks_inde, int num_tasks_de, int num_agents);

	// Destructor
	~cbga_Agent(){};

public:
	// The index of agent (agent 0,1,...)
	int idx_;
	// Initial position of each agent
	int init_pos_;
	// The award of agent
	std::vector<double> cbga_reward_;
	// The set of available task
	std::vector<cbga_Task> h_avai_;
	// Bundle
	std::vector<int> cbga_bundle_;
	// Path
	std::vector<int> cbga_path_;
	// Iteration that the agent talk to neighbors
	std::vector<int> iteration_neighbors_;
	std::vector<int> insert_pos_;

    // Required for complex tasks (multi-agents tasks)
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> assignment_matrix_;
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> winning_bids_matrix_;
	
	int max_tasks_;
	int num_agents_;
	int num_tasks_;
	int num_tasks_inde_;
	int num_tasks_de_;

	Memo history_;

	// Communication Topology
	std::vector<int> comm_topo_;
	// The neighbor of the agent
	std::vector<int> neighbors_;

	int iter_;

	double waiting_t_;

public:
	void bids_update(CBGATasks tasks_group, const std::shared_ptr<Graph_t<SquareCell*>> graph);
	void bids_update(LTLFormula Global_LTL, std::shared_ptr<Graph_t<SquareCell*>> graph, std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph, 
		TileTraversalData tile_traversal_data, std::shared_ptr<SquareGrid> grid);
	std::vector<int> find_winners(cbga_Task& task);
	std::vector<double> find_winner_bids(cbga_Task& task);
	void neighbors_finder();

	void UpdateWaitingTime(std::shared_ptr<Graph_t<SquareCell*>> graph, CBGATasks tasks);
};

namespace CBGA
{

	double CalcHeuristic(SquareCell *node1, SquareCell *node2);
	std::vector<cbga_Agent> InitializeAgents();
	double ScoreCalculation(CBGATasks tasks,std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<int> bundle ,cbga_Agent* agent);
    double ScoreCalculation(LTLFormula Global_LTL, std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<int> bundle, 
    	int start_idx_, std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph, TileTraversalData tile_traversal_data, 
    	std::shared_ptr<SquareGrid> grid);
	double PathLengthCalculation(CBGATasks tasks, std::shared_ptr<Graph_t<SquareCell*>> graph,std::vector<int> bundle,cbga_Agent* agent);
	double PathLengthCalculationBasedType(CBGATasks tasks_list, std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<int> bundle, int start_idx_);
	void available_tasks_finder(CBGATasks tasks, cbga_Agent& agent);
	int desired_task_finder(CBGATasks tasks, cbga_Agent& agent);
	void bundle_remove(std::vector<cbga_Agent>& agents);
	void bundle_remove(cbga_Agent& agent);
	void path_remove(std::vector<cbga_Agent>& agents);
	void path_remove(cbga_Agent& agent);
	void bundle_construction(CBGATasks tasks, std::shared_ptr<Graph_t<SquareCell*>> graph, cbga_Agent& agent);
	void bundle_construction(CBGATasks tasks, std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<cbga_Agent>& agents);
	void bundle_construction(CBGATasks tasks, LTLFormula Global_LTL,const std::shared_ptr<Graph_t<SquareCell*>> graph,
		std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph, TileTraversalData tile_traversal_data, 
        std::shared_ptr<SquareGrid> grid, cbga_Agent& agent);
	void bundle_construction(CBGATasks tasks, LTLFormula Global_LTL,const std::shared_ptr<Graph_t<SquareCell*>> graph,
		std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph, TileTraversalData tile_traversal_data, 
        std::shared_ptr<SquareGrid> grid, std::vector<cbga_Agent>& agents);
	
	bool success_checker(std::vector<cbga_Agent>& agents, CBGATasks tasks);
	void communicate(std::vector<cbga_Agent>& agents, cbga_Task task);
	void communicate_dependent(std::vector<cbga_Agent>& agents, cbga_Task task);
	void consensus(std::vector<cbga_Agent>& agents, CBGATasks tasks);
	// void update_rewards(CBGATasks tasks_group, cbga_Agent& agent,  std::shared_ptr<Graph_t<SquareCell*>> graph);
	void update_iteration_number(std::vector<cbga_Agent>& agents);
	cbga_Agent& FindAgent(std::vector<cbga_Agent>& agents, int idx);



	double WaitingTimeCalculation(std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<cbga_Agent>& agents, CBGATasks tasks);
	int FindNumAppearance(std::vector<int> list, int target);
	std::vector<int> FindWinners(std::vector<cbga_Agent> agents, int task_idx);
	double PathLengthCalculationWithWaiting(std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<int> bundle, int init_pos, CBGATasks tasks);
	double PathLengthCalculationForWaiting(std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<int> bundle, int init_pos, CBGATasks tasks);
		
};

}



#endif /* CBGA_AGENT_HPP */