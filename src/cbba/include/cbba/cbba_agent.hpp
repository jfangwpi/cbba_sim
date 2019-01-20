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
const double eps = 0.0;

const double cbba_benefit = 200.0;
const double syn_benefit = 200.0;

/*** General CBBA ***/
typedef struct
{	
	// Required by CBBA
	std::vector<Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>> x_history;
	std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> y_history;
	std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> z_history;
	std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> iter_neighbors_his;
	// Required by synchronization algorithm
	std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> assignment_;
	std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> winning_bids_;
}Memo;

class Agent
{
public:
	// Constructor
	Agent(int id, int startID, int num_tasks_inde, int num_tasks_de, int num_agents);

	// Destructor
	~Agent(){};

public:
	/*** General information of vehicles ***/
	// The index of agent (agent 0,1,...)
	int idx_;
	int init_pos_;
	// Iteration that the agent talk to neighbors
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> iteration_neighbors_;
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
	// Communication Topology
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> comm_topo_;
	// The neighbor of the agent
	std::vector<int> neighbors_;
	// Current iteration
	int iter_;
	// Best insert position for all tasks (Independent + Dependent)
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> insert_pos_;
	// History of information required by CBBA and Synchronization Algorithm
	Memo history_;


	/*** Information required by CBBA ***/
	// The reward of agent
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> cbba_reward_;
	// The set of available independent tasks
	std::vector<int> h_avai_;
	// Task Bundle
	std::vector<int> cbba_bundle_;
	// Task Path
	std::vector<int> cbba_path_;
	// The highest bid for certain task that agent knows
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> cbba_y_;
	// The winner for certain task that agent knows
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> cbba_z_;
	// The assignment of task
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> cbba_x_;
	
	
	/*** Information required by synchronization algorithm ***/
	// Required by synchronization algorithm
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> syn_c_;
	// The set of available dependent tasks
	std::vector<int> h_avai_de_;
	// Assignment matrix
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> assignment_matrix_;
	// Reward matrix 
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> winning_bids_matrix_;
	// Total waiting time required by the vehicle 
	double waiting_t_;
	// Insert position threshold for dependent tasks
	int last_pos_dependent_;

	
public:
	/*** Required by CBBA ***/
	// Add independent tasks into task bundle
	void bundle_add(TasksList tasks, const std::shared_ptr<Graph_t<SquareCell*>> graph);
	// Find the neighbors for vehicle based on comm_topo_
	void neighbor_finder();
	// Update the reward of independent tasks
	void reward_update(TasksList tasks_list, const std::shared_ptr<Graph_t<SquareCell*>> graph);
	// Find the available independent tasks based on current task path
	void available_tasks_finder();
	// Find the desired task from a list of available indenpendent tasks
	int desired_task_finder();
	// Remove the outbid independent tasks from task bundle
	void bundle_remove();
	// Remove the outbid independent tasks from task path
	void path_remove();
	

	/*** Required by Synchronization Algorithm ***/
	// Update the syn_c and the optimal insert position 
	void update_reward_dependent(TasksList tasks, std::shared_ptr<Graph_t<SquareCell*>> graph);
	// Find the available dependent task based on current task path
	void available_dep_tasks_finder(TasksList tasks, std::shared_ptr<Graph_t<SquareCell*>> graph);
	// Find the desired dependent task from the list of available tasks
	int desired_dep_task_finder();
	// Add dependent task into task path
	void bundle_add_dependent(TasksList task, const std::shared_ptr<Graph_t<SquareCell*>> graph);
	// Update the optimal group for the given task
	void optimal_group_finder(cbba_Task task);
	// Find the vehicles based on the bids
	std::vector<int> FindVehicleFrombid(cbba_Task task, std::vector<double> winners_bid);
	// Find the winners for the given task
	std::vector<int> winners_finder(cbba_Task task);
	// Find the number of vehicle 
	int winners_count(cbba_Task task);
	// Remove outbid dependent task from task path
	void path_remove_dependent(TasksList tasks);
	// Remove outbid dependent task from task bundle
	void bundle_remove_dependent();
	// Find the winning bids for the given task
	std::vector<double> winning_bids_finder(cbba_Task task);
	// Compute the waiting time required by all dependent tasks
	void UpdateWaitingTime(std::shared_ptr<Graph_t<SquareCell*>> graph, TasksList tasks);


	/*** Required by CBTA-CBBA ***/
	// Update the reward for independent tasks
	void reward_update_CBTA(TasksList tasks, const std::shared_ptr<Graph_t<SquareCell*>> graph, std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph,  
		TileTraversalData tile_traversal_data, std::shared_ptr<SquareGrid> grid);
	// Insert independent tasks into task bundle/path
	void bundle_add(TasksList tasks,const std::shared_ptr<Graph_t<SquareCell*>> graph,std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph, 
		TileTraversalData tile_traversal_data, std::shared_ptr<SquareGrid> grid);


	/*** Required by CBTA-Synchronization algoeirhm ***/
	// Update syn_c and optimal insert position for dependent task
	void update_reward_dependent(TasksList tasks, std::shared_ptr<Graph_t<SquareCell*>> graph, std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph, 
		TileTraversalData tile_traversal_data, std::shared_ptr<SquareGrid> grid);
	// Find the available dependent tasks
	void available_dep_tasks_finder(TasksList tasks, std::shared_ptr<Graph_t<SquareCell*>> graph,std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph,
		TileTraversalData tile_traversal_data, std::shared_ptr<SquareGrid> grid);
	// Insert dependent tasks into task bundle/path
	void bundle_add_dependent(TasksList tasks, std::shared_ptr<Graph_t<SquareCell*>> graph, std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph, 
		TileTraversalData tile_traversal_data, std::shared_ptr<SquareGrid> grid);

};

namespace TaskAssignment
{
	/*** General functions for agents ***/
	double CalcHeuristic(SquareCell *node1, SquareCell *node2);
	// Read agents information from config file
	std::vector<Agent> InitializeAgents();
	// Reurn all possible task path after inserting new task into current task path
	std::vector<std::vector<int>> PossibleInsertPosFinder(std::vector<int> current_path, int task_syn);
	// Return agent based on index
	Agent& FindAgent(std::vector<Agent>& agents, int64_t idx);
	// Compute the length of path with given task path
	double PathLengthCalculation(TasksList tasks_list, std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<int> bundle, int start_idx_);
	double PathLengthCalculationFromLTL(TasksList tasks, std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<int> bundle, int start_idx_);
	double PathLengthCalculationBasedType(TasksList tasks, std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<int> bundle, int start_idx_);
	double PathLengthCalculationCBTA(TasksList tasks, std::shared_ptr<Graph_t<SquareCell*>> graph,
		std::vector<int> bundle, int start_idx_,std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph, 
		TileTraversalData tile_traversal_data, std::shared_ptr<SquareGrid> grid);

	/*** Functions for CBBA ***/
	void bundle_add(TasksList tasks, const std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<Agent>& agents);
	void bundle_remove(std::vector<Agent>& agents);
	void communicate(std::vector<Agent>& agents);
	bool success_checker(std::vector<Agent> agent);

	
	/*** Functions for Synchronization algorithm ***/
	// Insert dependent tasks into task bundle/path and update iteration
	void bundle_add_dependent(TasksList tasks, const std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<Agent>& agents);
	// Remove dependent tasks from task path/bundle
	void path_remove_dependent(TasksList tasks, std::vector<Agent>& agents);
	// Communicate with neighbors
	void communicate_dependent(std::vector<Agent>& agents, TasksList tasks);
	// Find the K closest reward based on current reward
	std::vector<double> KClosestFinder(std::vector<double> ordered_bids, int x, int k, int n);
	int findCrossOver(std::vector<double> ordered_bids, int low, int high, int x);
	double maximum_bid_finder(std::vector<double> winning_bids);
	double minimum_bid_finder(std::vector<double> winning_bids);
	// Check whether the convergence of synchronization algorithm is achieved
	bool success_checker_dependent(std::vector<Agent> agents, TasksList tasks);


	/*** Functions for CBTA-CBBA ***/
	// Insert independent tasks into task bundle and task path by CBBA
	void bundle_add(TasksList tasks,const std::shared_ptr<Graph_t<SquareCell*>> graph,std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph, 
	TileTraversalData tile_traversal_data, std::shared_ptr<SquareGrid> grid,std::vector<Agent>& agents);
	// Insert independent tasks into task bundle and path with different radius of turn by CBBA
	void bundle_add(TasksList tasks,const std::shared_ptr<Graph_t<SquareCell*>> graph,std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph, 
		std::map<int, TileTraversalData> tile_traversal_data, std::shared_ptr<SquareGrid> grid, std::vector<Agent>& agents);

	
	/*** Functions for CBTA-Synchronization algorithm ***/
	// Insert dependent tasks into task bundle and task path by synchronization algorithm
	void bundle_add_dependent(TasksList tasks, std::shared_ptr<Graph_t<SquareCell*>> graph, std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph, 
		TileTraversalData tile_traversal_data, std::shared_ptr<SquareGrid> grid,std::vector<Agent>& agents);
	void bundle_add_dependent(TasksList tasks, std::shared_ptr<Graph_t<SquareCell*>> graph, std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph, 
		std::map<int, TileTraversalData> tile_traversal_data, std::shared_ptr<SquareGrid> grid,std::vector<Agent>& agents);


	/*** Compute the waiting time required by vehicle ***/
	double WaitingTimeCalculation(std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<Agent>& agents, TasksList tasks);
	int FindNumAppearance(std::vector<int> list, int target);
	std::vector<int> FindWinners(std::vector<Agent> agents, int task_idx);
	// Find the longest path for all dependent tasks
	double MaximumRewardCalculation(std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<int> bundle, int init_pos, TasksList tasks);
	// Compute certain length of path by considerring the waiting time
	double PathLengthCalculationWithWaiting(std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<int> bundle, int init_pos, TasksList tasks);
};

}



#endif /* CBBA_AGENT_HPP */