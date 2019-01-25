/*
 * cbba_Agent.h
 *
 *  Created on: Feb 15, 2017
 *      Author: jfang
 */

#ifndef CBBA_AGENT_HPP
#define CBBA_AGENT_HPP


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


namespace librav{

// The number of helpers required
const float eps = 0.0;

const float cbba_benefit = 100.0;

/*** General CBBA ***/
typedef struct
{
	std::vector<std::vector<int>> x_history;
	std::vector<std::vector<float>> y_history;
	std::vector<std::vector<int>> z_history;
	std::vector<std::vector<int>> iter_neighbors_his;

	std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> assignment_;
	std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> winning_bids_;
}Memo;

class cbba_Agent
{
public:
	// Constructor
	cbba_Agent(int id, int startID, std::vector<int> com, int num_tasks_inde, int num_tasks_de);

	// Destructor
	~cbba_Agent(){};

public:
	// The index of agent (agent 0,1,...)
	int idx_;
	int start_node_;
	// The award of agent
	std::vector<float> cbba_award_;
	// The set of available task
	std::vector<int> h_avai_;
	// Bundle
	std::vector<int> cbba_bundle_;
	// Path
	std::vector<int> cbba_path_;
	// Iteration that the agent talk to neighbors
	std::vector<int> iteration_neighbors_;
	// The highest bid for certain task that agent knows
	std::vector<float> cbba_y_;
	// The winner for certain task that agent knows
	std::vector<int> cbba_z_;
	// The assignment of task
	std::vector<int> cbba_x_;
	// Best insert position for task
	std::vector<int> insert_pos_;

	// Maximum tasks each agent can be assigned
	int max_tasks_;
	// Number of agents in the group
	int num_agents_;
	// Total number of tasks 
	int num_tasks_;
	// Number of independent tasks
	int num_tasks_inde_;
	// Number of dependent tasks
	int num_tasks_de_;

	Memo history_;

	// Communication Topology
	std::vector<int> comm_topo_;
	// The neighbor of the agent
	std::vector<int> neighbors_;

	int iter_;

	// Decentralized Synchronization
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> assignment_matrix_;
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> winning_bids_matrix_;
	

public:

	void assignment_update();

	std::vector<int> award_update(LTLFormula Global_LTL,const std::shared_ptr<Graph_t<SquareCell*>> graph);
	std::vector<int> award_update_CBTA(LTLFormula Global_LTL,const std::shared_ptr<Graph_t<SquareCell*>> graph, std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph,  
			TileTraversalData tile_traversal_data, std::shared_ptr<SquareGrid> grid);
	void update_reward_syn(CBBATasks tasks, std::shared_ptr<Graph_t<SquareCell*>> graph, int task_idx_syn);
	void neighbor_finder();
	void assignment_update(cbba_Task& task, bool flag);
	void assignment_update(std::vector<int> dependent_task_idx);
	
	void update_reward_dependent(CBBATasks tasks, std::vector<int> dependent_tasks_idx, std::shared_ptr<Graph_t<SquareCell*>> graph);
	void bundle_add_dependent(CBBATasks tasks, std::vector<int> dependent_tasks_idx, std::shared_ptr<Graph_t<SquareCell*>> graph);
	void update_dependent_path_length(CBBATasks tasks, std::vector<int> dependent_tasks_idx, std::shared_ptr<Graph_t<SquareCell*>> graph);

	void dependent_tasks_convergence(int task_idx, int num_agent, std::vector<cbba_Agent>& agents);

	std::vector<int> winners_finder(cbba_Task& task);
	std::vector<int> his_winners_finder(cbba_Task& task);
	int winners_count(cbba_Task& task);
	std::pair<int, float> min_bid_finder(cbba_Task& task);
	std::map<int, float> cross_bid_finder(cbba_Task& task, float new_bid);
	std::pair<int, float> max_bid_finder(cbba_Task& task);
	std::pair<int, float> second_min_bid_finder(cbba_Task& task);
	std::pair<int, float> second_max_bid_finder(cbba_Task& task);


	// CBTA
	void update_reward_dependent(CBBATasks tasks, std::vector<int> dependent_tasks_idx, LTLFormula Global_LTL, 
		std::shared_ptr<Graph_t<SquareCell*>> graph, std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph, TileTraversalData tile_traversal_data, 
		std::shared_ptr<SquareGrid> grid);


	void bundle_add_dependent(CBBATasks tasks, std::vector<int> dependent_task_idx, LTLFormula Global_LTL, 
		std::shared_ptr<Graph_t<SquareCell*>> graph, std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph, TileTraversalData tile_traversal_data, 
		std::shared_ptr<SquareGrid> grid);


	void update_dependent_path_length(CBBATasks tasks, std::vector<int> dependent_task_idx, LTLFormula Global_LTL, 
		std::shared_ptr<Graph_t<SquareCell*>> graph, std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph, TileTraversalData tile_traversal_data, 
		std::shared_ptr<SquareGrid> grid);

};

namespace CBBA
{

	double CalcHeuristic(SquareCell *node1, SquareCell *node2);
	std::vector<cbba_Agent> InitializeAgents();

    float PathLengthCalculation(LTLFormula Global_LTL,std::shared_ptr<Graph_t<SquareCell*>> graph,std::vector<int> bundle_copy_onecase,int start_idx);
	float PathLengthCalculationWithDelay(CBBATasks tasks, std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<int> bundle, int start_idx);
	float PathLengthCalculationCBTA(LTLFormula Global_LTL, std::shared_ptr<Graph_t<SquareCell*>> graph,
		std::vector<int> bundle_copy_onecase,int start_idx_,std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph, 
		TileTraversalData tile_traversal_data, std::shared_ptr<SquareGrid> grid);

	void communicate(std::vector<cbba_Agent>& agents);
	void communicate_lcm(cbba_Agent& agent, int neig_idx, std::vector<float> neig_y_his, std::vector<int> neigh_z_his, std::vector<int> neigh_iter_nei_his);
	void communicate_dependent_lcm(cbba_Agent& agent, cbba_Task& task, int neighbor_idx, std::vector<std::vector<float>> bids_matrix, std::vector<std::vector<int>> assignment_matrix, std::vector<int> iteration_neighbors);
	void available_tasks_finder(cbba_Agent& agent_sig);
	int desired_task_finder(cbba_Agent& agent_sig);
	void bundle_remove(std::vector<cbba_Agent>& agent);
	void bundle_remove(cbba_Agent& agent);
	void path_remove(std::vector<cbba_Agent>& agent);
	void path_remove(cbba_Agent& agent);
	void bundle_add(LTLFormula Global_LTL,const std::shared_ptr<Graph_t<SquareCell*>> graph,std::vector<cbba_Agent>& agent);
	void bundle_add(LTLFormula Global_LTL,const std::shared_ptr<Graph_t<SquareCell*>> graph, cbba_Agent& agent);
	void bundle_add(LTLFormula Global_LTL,const std::shared_ptr<Graph_t<SquareCell*>> graph,std::vector<cbba_Agent>& agent,
		std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph, TileTraversalData tile_traversal_data, 
		std::shared_ptr<SquareGrid> grid);
	bool success_checker(std::vector<cbba_Agent> agent);

	// // Decentralized Syn
	void communicate_dependent_centralized(std::vector<cbba_Agent>& agents, cbba_Task& task);
	int find_num_agents(LTLFormula Global_LTL, int task_syn);
	void CommunicationSyn(std::vector<cbba_Agent>& agents, LTLFormula Global_LTL, int task_syn);
	std::vector<int> DesiredInserPosFinder(cbba_Agent agent, int task_syn, LTLFormula Global_LTL,std::shared_ptr<Graph_t<SquareCell*>> graph,int n_helper);
	std::vector<std::vector<int>> PossibleInsertPosFinder(std::vector<int> current_path, int task_syn);
	cbba_Agent& FindAgent(std::vector<cbba_Agent>& agents, int idx);
	void update_iteration_number(std::vector<cbba_Agent>& agents);


	std::vector<float> findKclosest(std::vector<cbba_Agent>& agents, float leader_bid, int leader_idx, int K, int n, cbba_Task task);
	int findCrossOver(std::vector<float> ordered_bids, int low, int high, int leader_bid);
	std::vector<int> findKclosest_idx(std::vector<float> Kclosest_bids, std::vector<cbba_Agent>& agents, cbba_Task task);
	void leader_select_centralized(std::vector<cbba_Agent>& agents, CBBATasks& tasks, int task_idx);

	void communicate_dependent_decentralized(std::vector<cbba_Agent>& agents, cbba_Task& task);
	
	// void group_vehicles_selection(std::vector<cbba_Agent>& agents, int K, cbba_Task& task);
};

}



#endif /* CBBA_AGENT_HPP */