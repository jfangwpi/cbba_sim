/*
 * cbba_Agent.cpp
 *
 *  Created on: Feb 15, 2017
 *      Author: jfang
 */

#include "cbba/cbba_agent.hpp"
#include "config_reader/config_reader.hpp"

using namespace librav;

// Initialize agent
Agent::Agent(int id, int startID, int num_tasks_inde, int num_tasks_de, int num_agents)
{
	idx_ = id;
	init_pos_ = startID;
	comm_topo_ = Eigen::MatrixXd::Zero(1, num_agents);
	iter_ = 0;
	cbba_bundle_ = {};
	cbba_path_ = {};

	num_agents_ = num_agents;
	num_tasks_inde_ = num_tasks_inde;
	num_tasks_de_ = num_tasks_de;
	max_tasks_ = num_tasks_inde + num_tasks_de;
	num_tasks_ = num_tasks_inde_ + num_tasks_de_;
	
	iteration_neighbors_ = Eigen::MatrixXd::Zero(1, num_agents);
	history_.iter_neighbors_his = {iteration_neighbors_};
	
	cbba_y_ = Eigen::MatrixXd::Zero(1, num_tasks_inde_);
	cbba_z_ = -1 * Eigen::MatrixXd::Ones(1, num_tasks_inde_);

	history_.y_history = {cbba_y_};
	history_.z_history = {cbba_z_};
	
	cbba_reward_ = -1 * Eigen::MatrixXd::Ones(1, num_tasks_);
	syn_c_ = -1 * Eigen::MatrixXd::Ones(1, num_tasks_);
	insert_pos_ = -1 * Eigen::MatrixXd::Ones(1, num_tasks_);

	// Initialize dependent tasks
	assignment_matrix_ = Eigen::MatrixXd::Zero(num_tasks_, num_agents_);
	winning_bids_matrix_ = -1 * Eigen::MatrixXd::Ones(num_tasks_, num_agents_);

};

// Initialize Agents: Read agents from config file 
std::vector<Agent> TaskAssignment::InitializeAgents(){
	
	ConfigReader config_reader("../../src/config/agent.ini");
    if (config_reader.CheckError()){
        std::cout << "Reading config file failed." << std::endl;
    }

    int32_t num_agents = config_reader.GetReal("num_agents", 0);
	std::vector<Agent> agents;

	for (int i= 0; i < num_agents; i++){
		std::string agent_name = "agent"+ std::to_string(i);
		std::string agent_idx = agent_name + "_idx";
		std::string agent_init_pos = agent_name + "_init";
		std::string agent_neighbors = agent_name + "_neighbors";

		// Read config from agent.ini
		int idx = i;
		int init_pos = config_reader.GetReal(agent_init_pos, 0);
		int num_tasks_inde = config_reader.GetReal("num_tasks_inde", 0);
		std::vector<int> comm = config_reader.GetVectorInt(agent_neighbors, {});
		int num_tasks_de = config_reader.GetReal("num_tasks_de", 0);

		// Initialize the agent and communication network
		Agent agent = Agent(idx, init_pos, num_tasks_inde, num_tasks_de, num_agents);
		for (int i = 0; i < num_agents; i++){
			if (comm[i] == 1){
				agent.comm_topo_(0, i) = 1;
			}
		}
		agents.push_back(agent);
	}
	return agents;
}


double TaskAssignment::CalcHeuristic(SquareCell *node1, SquareCell *node2)
{
    int32_t dist_row = node1->coordinate_.x - node2->coordinate_.x;
    int32_t dist_col = node1->coordinate_.y - node2->coordinate_.y;

    return std::sqrt(dist_row * dist_row + dist_col * dist_col);
}


double TaskAssignment::PathLengthCalculation(TasksList tasks_list, std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<int> bundle, int start_idx_){
	Path_t<SquareCell *> path_part;
	Path_t<SquareCell *> path;
	Vertex_t<SquareCell*> * start_node;
	Vertex_t<SquareCell*> * finish_node;

	start_node = graph->GetVertexFromID(start_idx_);
	for (int i = 0; i < bundle.size(); i++){
		cbba_Task& task = tasks_list.FindTask(bundle[i]);
		for (int t_j = 0 ; t_j < task.pos_.size(); t_j++){
			finish_node = graph->GetVertexFromID(task.pos_[t_j]);
			path_part = AStar::Search(graph,start_node->state_->id_,finish_node->state_->id_, CalcHeuristicFunc_t<SquareCell *>(TaskAssignment::CalcHeuristic));
			if (i == bundle.size()-1 && t_j == task.pos_.size()-1){
				path.insert(path.end(), path_part.begin(), path_part.end());
			}
			else{
				path.insert(path.end(), path_part.begin(), path_part.end()-1);
			}
			start_node = finish_node;
		}
	}

	double path_length = path.size();
	return path_length;
}

double TaskAssignment::PathLengthCalculationFromLTL(TasksList tasks, std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<int> bundle, int start_idx_){
	// Create the local ltl formula 
	std::string local_ltl = tasks.local_formula_recreator(bundle);
	std::cout << "The local formula is " << local_ltl << std::endl;
	std::vector<std::vector<std::string>> buchi_regions = LTLDecomposition::ObtainBuchiRegion({local_ltl});

	// std::vector<std::string> buchi_regions;
	std::shared_ptr<Graph_t<BuchiState *, std::vector<int64_t>>> buchi_graph = BuchiAutomaton::BuildBuchiGraph(local_ltl,buchi_regions.front());
    std::vector<int64_t> buchi_acc = (*buchi_graph->FindVertex(0)).state_->acc_state_idx;

	std::shared_ptr<Graph_t<ProductState *>> product_graph = std::make_shared<Graph_t<ProductState *>>();
    int64_t virtual_start_state_id = ProductAutomaton::SetVirtualStartState(product_graph, buchi_graph, graph, start_idx_);

	GetProductNeighbor product_neighbor(graph, buchi_graph);
    auto path = AStar::ProductIncSearch(product_graph, virtual_start_state_id, buchi_acc, GetNeighbourFunc_t<ProductState*, double>(product_neighbor));
    Path_t<SquareCell *> path_vis;
    for(auto &e: path){
        path_vis.push_back(e->grid_vertex_->state_);
    }

	/**** Debug for path planning ****/
    // for (auto &ee:path_vis){
    //     std::cout << ee->id_ << " ";
    // }
    // std::cout << std::endl;
	
	double path_length = 0.0;
	if(!path_vis.empty()){
		path_length = path_vis.size();
	}
	else{
		path_length = cbba_benefit;
	}

	return path_length;
}

double TaskAssignment::PathLengthCalculationBasedType(TasksList tasks_list, std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<int> bundle, int start_idx_){
	Path_t<SquareCell *> path_part;
	Path_t<SquareCell *> path;
	Vertex_t<SquareCell*> * start_node;
	Vertex_t<SquareCell*> * finish_node;
	int64_t final_pos;
	bool flag = false;

	std::vector<int64_t> pos_seq;
	start_node = graph->GetVertexFromID(start_idx_);
	for (int i = 0; i < bundle.size(); i++){
		cbba_Task task = tasks_list.FindTaskFromID(bundle[i]);
		if(task.type_ == TaskType::VISIT){
			if(task.pos_.size() == 1){
				pos_seq.push_back(task.pos_[0]);
			}
			else{
				for (int j = 0; j < task.pos_.size(); j++){
					pos_seq.push_back(task.pos_[j]);
				}
			}
		}
		else if (task.type_ ==  TaskType::SEARCH){
			if (task.pos_.size() != task.buchi_regions_.size()){
				pos_seq.push_back(task.pos_[0]);
			}
			else{
				final_pos = task.pos_.back();
				pos_seq.push_back(task.pos_[0]);
			}
		}
	}
	
	start_node = graph->GetVertexFromID(start_idx_);
	for (int k = 0; k < pos_seq.size(); k++){
		finish_node = graph->GetVertexFromID(pos_seq[k]);
		path_part = AStar::Search(graph,start_node->state_->id_,finish_node->state_->id_, CalcHeuristicFunc_t<SquareCell *>(TaskAssignment::CalcHeuristic));
		if (k == pos_seq.size() - 1){
			path.insert(path.end(), path_part.begin(), path_part.end());
				
		}
		else{
			path.insert(path.end(), path_part.begin(), path_part.end()-1);
		}
		start_node = finish_node;
	}

	// std::cout << "+++++++++++++++++++++++++++++++++++++ " << std::endl;
	// for (auto &e: path){
	// 	std::cout << e->id_ << ", ";
	// }
	// std::cout << std::endl;
	// std::cout << "+++++++++++++++++++++++++++++++++++++ " << std::endl;
	double path_length = path.size();
	return path_length;
}

double TaskAssignment::PathLengthCalculationCBTA(TasksList tasks, std::shared_ptr<Graph_t<SquareCell*>> graph,
	std::vector<int> bundle, int start_idx_,std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph, 
	TileTraversalData tile_traversal_data, std::shared_ptr<SquareGrid> grid){
		std::cout << "================================ DEBUG FOR CBTA ====================================" << std::endl;
		std::shared_ptr<Graph_t<ProductState *>> product_graph_new = std::make_shared<Graph_t<ProductState *>>();

		std::string ltl_formula = tasks.local_task_recreator(bundle);
		std::vector<std::vector<std::string>> buchi_regions = tasks.obtain_buchi_regions({ltl_formula});

        std::shared_ptr<Graph_t<BuchiState *, std::vector<int64_t>>> buchi_graph = BuchiAutomaton::BuildBuchiGraph(ltl_formula,buchi_regions.front());
        std::vector<int64_t> buchi_acc = (*buchi_graph->FindVertex(0)).state_->acc_state_idx;

		Vertex_t<SquareCell *> * start_node_origin = graph->GetVertexFromID(start_idx_);
		std::vector<double> zta0 = {0.45,0.0};
		std::vector<int> rgn_idx_init = {HCost::zta02rgn_idx(zta0,tile_traversal_data.region_bd)};
		
		int64_t virtual_start_id = ProductAutomaton::SetVirtualStartIncre(start_node_origin, rgn_idx_init, lifted_graph, buchi_graph, product_graph_new);
		
		GetProductCBTANeighbour get_product_cbta_neighbour(lifted_graph, buchi_graph, tile_traversal_data, grid);
		auto path = AStar::ProductIncSearchCBTA(product_graph_new, virtual_start_id, buchi_acc,  GetNeighbourFunc_t<ProductState*, double>(get_product_cbta_neighbour));
		float path_CBTA_length;
		if (!path.empty()){
			std::vector<Vertex_t<SquareCell*>*> path_origin;
    		for (auto cell = path.begin()+1; cell!= path.end(); cell++){
        		path_origin.push_back((*cell)->lifted_vertex_->state_->history.front());
    		}

    		path_origin.insert(path_origin.end(),path.back()->lifted_vertex_->state_->history.begin()+1,path.back()->lifted_vertex_->state_->history.end());
     		Path_t<SquareCell *> path_vis;
    		for(auto &ee: path_origin){
       	 		path_vis.push_back(ee->state_);
    		}
			path_CBTA_length = float(path_vis.size());
		}
		else{
			path_CBTA_length = 1000.0;
		}

		std::cout << "================================ DEBUG ====================================" << std::endl;
		std::cout << "start idx: " << start_idx_ << std::endl;
		std::cout << "The specification is " << ltl_formula << std::endl;
		std::cout << "The FINAL length of path is: ！! " << path_CBTA_length << std::endl;
		std::cout << "================================ DEBUG ====================================" << std::endl;
		return path_CBTA_length;
}

/*** Find certain agent with given index ***/
Agent& TaskAssignment::FindAgent(std::vector<Agent>& agents, int64_t idx){
	for (auto &agent: agents){
		if (agent.idx_ == idx){
			return agent;
		}
	}
}

/*** Find possible results after inserting certain task into current task bundle ***/
std::vector<std::vector<int>> TaskAssignment::PossibleInsertPosFinder(std::vector<int> current_path, int task_syn){
	// Find all possibilities to insert task_syn for current agent
	std::vector<std::vector<int>> path_poss;
	path_poss.clear();
	// If the size of cbba_bundle is n, then there are n+1 positions to insert each task in the tasks_NotInBundle
	for (int k = 0; k < current_path.size()+1; k++)
		path_poss.push_back(current_path);
	// Insert one task (task_NotInBundle[j]) into all the possible position
	// Each item in the path_copy represents a possible position to insert the task
	//int task_syn = 0;
	for (int m = 0; m < path_poss.size(); m++){
		auto it_pos_other_ag = path_poss[m].begin();
		path_poss[m].insert(it_pos_other_ag+m,task_syn);
	}

	return path_poss;
}


/****************************************************************************************************/
/****************************************************************************************************/
/******************************************* CBBA ***************************************************/
/****************************************************************************************************/
/****************************************************************************************************/
/*** Update reward of independent tasks ***/
void Agent::reward_update(TasksList tasks_list, const std::shared_ptr<Graph_t<SquareCell*>> graph){
	// Find the task which is not in the bundle
	std::vector<int> tasks_NotInBundle = {};
	for (auto &task: tasks_list.tasks_){
		if (task.num_agents_ == 1){
			tasks_NotInBundle.push_back(task.idx_);
		}
	}
	// Find all tasks which have not been inserted into the bundle
	for (int j = 0; j < cbba_bundle_.size(); j++){
		// Erase all tasks which have been added into the bundle based on erase-remove idiom
		tasks_NotInBundle.erase(std::remove(tasks_NotInBundle.begin(),tasks_NotInBundle.end(),cbba_bundle_[j]),tasks_NotInBundle.end());
	}

	// Prepare all cases after inserting the task at each possible position
	for (int j = 0; j < tasks_NotInBundle.size();j++){
		// Clear c_award and bundle_copy for each task in the tasks_NotInBundle
		// c_award and bundle_copy are only for one task with different insert position
		std::vector<double> c_reward = {};
		
		// If the size of cbba_bundle is n, then there are n+1 positions to insert each task in the tasks_NotInBundle
		std::vector<std::vector<int>> path_copy(cbba_path_.size()+1, cbba_path_);

		// Insert one task (task_NotInBundle[j]) into all the possible position
		// Each item in the path_copy represents a possible position to insert the task
		for (int m = 0; m < path_copy.size(); m++){
			std::vector<int>::iterator it = path_copy[m].begin();
			path_copy[m].insert(it+m,tasks_NotInBundle[j]);

			// Calculate the Astar length for each possible insert position
			c_reward.push_back(TaskAssignment::PathLengthCalculationBasedType(tasks_list,graph,path_copy[m],init_pos_));
			// c_reward.push_back(CBBA::PathLengthCalculationFromLTL(tasks_list,graph,path_copy[m],init_pos_));
		}

		float c_reward_min = cbba_benefit;
		for (int i = 0; i < c_reward.size(); i++)
			if (c_reward[i] < c_reward_min){
				c_reward_min = c_reward[i];
				insert_pos_(0, tasks_NotInBundle[j]) = i;
			}

		// Update the reward
		cbba_reward_(0, tasks_NotInBundle[j]) = cbba_benefit - c_reward_min;
	}

	// std::cout << "============================= DEBUG FOR AWARD UPDATES ================================= " << std::endl;
	// std::cout << "For vehicle " << idx_ << std::endl;
	// std::cout << "The Reward are !!!!!!!!!!!!!!!!!!!!" << std::endl;
	// for (int j = 0; j < num_agents_; j++)
	// 	std::cout << cbba_reward[j] << ' ';
	// std::cout << '\n';

	// std::cout << "Best Position for each task should be:" <<std::endl;
	// for (int j = 0; j < best_position.size(); j++)
	// 	std::cout << best_position[j] << ' ';
	// std::cout << std::endl;

}

/*** Find the available tasks ***/
void Agent::available_tasks_finder(){
	//Initialize the available tasks
	h_avai_.clear();
	bool condition_1;
	bool condition_2;

	for (int j = 0; j < num_tasks_inde_; j++){
		// Initialize the condition for each task
		condition_1 = 0;
		condition_2 = 0;

		if(cbba_reward_(0, j) - cbba_y_(0, j) > eps)
			condition_1 = 1;
		else if (abs(cbba_reward_(0, j) - cbba_y_(0, j)) <= eps)
			if (idx_ < cbba_z_(0, j))
				condition_2 = 1;

		if (condition_1 == 1 || condition_2 == 1)
			h_avai_.push_back(j);
	}

};


/*** Find the desired independent task ***/
int Agent::desired_task_finder(){
	int max = 0;
	int desired_Index = -1;

	// Update the available tasks for agent
	available_tasks_finder();

	if (!h_avai_.empty()){
		// If for certain agent, there are more than one desired tasks, select the one with smaller task index
		for (int j = 0; j < h_avai_.size();j++){
			if (cbba_reward_(0, h_avai_[j]) > max){
		    	max = cbba_reward_(0, h_avai_[j]);
		    	desired_Index = h_avai_[j];
			}
		}
	}
	return desired_Index;
};


/*** Add desired independent tasks into task bundle and task path ***/
void Agent::bundle_add(TasksList tasks, const std::shared_ptr<Graph_t<SquareCell*>> graph){
	bool bundleFull = -1;

	// Check whether the bundle is full or not
	if (cbba_bundle_.size() > max_tasks_){
		std::cout << "The bundle is full" << std::endl;
		bundleFull = 1;
	}
	else
		bundleFull = 0;

	while (bundleFull == 0){
		reward_update(tasks, graph);
		int desired_idx = desired_task_finder();
		if (desired_idx == -1){
			break;
		}

		// update assignment
		cbba_z_(0, desired_idx) = idx_;
		cbba_y_(0, desired_idx) = cbba_reward_(0, desired_idx);

		// Insert desired task into task path
		std::vector<int>::iterator it = cbba_path_.begin();
		cbba_path_.insert(it+insert_pos_(0, desired_idx), desired_idx);

		cbba_bundle_.push_back(desired_idx);

		if (cbba_bundle_.size() >= max_tasks_){
			bundleFull = 1;
		}
	}

	history_.y_history.push_back(cbba_y_);
	history_.z_history.push_back(cbba_z_);
}

/*** bundle add for all vehicles in the team ***/
void TaskAssignment::bundle_add(TasksList tasks, const std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<Agent>& agents){
	for (auto &agent: agents){
		agent.bundle_add(tasks, graph);
	}
};


/*** Find the neighbors based on current communicate network ***/
void Agent::neighbor_finder()
{	
	neighbors_.clear();
	for (int i = 0; i < num_agents_; i++)
		if (comm_topo_(0, i) == 1 && i != idx_)
			neighbors_.push_back(i);
		else
			continue;
};


/*** Communicate with neighbors ***/
void TaskAssignment::communicate(std::vector<Agent>& agents)
{
	//CBBA::neighbor_finder(agent);
	// sender: itself k
	// receiver: i
	// task : j
	for (auto &agent: agents){
		agent.neighbor_finder();
		for (int i = 0; i < agent.neighbors_.size();i++){
			Agent neighbor = TaskAssignment::FindAgent(agents, agent.neighbors_[i]);
			for (int j = 0; j < agent.num_tasks_inde_; j++){
				// Entries 1 to 4
				// if current agent k thinks that the winner of task j is itself k
				if (neighbor.history_.z_history.back()(0, j) == neighbor.idx_){

					/***************************************** Entry 1 ***************************************/
					// Entry 1: Update or leave
					// If the receiver (neighbor) i thinks the winner of task j is also itself i
					if (agent.cbba_z_(0, j) == agent.idx_){
						// Update
						if (neighbor.history_.y_history.back()(0,j) - agent.cbba_y_(0,j) > eps){
							//std::cout << "case 1" << std::endl;
							agent.cbba_z_(0, j) = neighbor.history_.z_history.back()(0, j);
							agent.cbba_y_(0, j) = neighbor.history_.y_history.back()(0, j);
						}
					    // Equal score: require to break the tie
						else if(abs(neighbor.history_.y_history.back()(0,j) - agent.cbba_y_(0,j)) <= eps){
							// select the winner of task j as the agent with smaller index
							if (agent.cbba_z_(0, j) > neighbor.history_.z_history.back()(0, j)){
								//std::cout << "case 2" << std::endl;
								agent.cbba_z_(0, j) = neighbor.history_.z_history.back()(0, j);
								agent.cbba_y_(0, j) = neighbor.history_.y_history.back()(0, j);
							}
						}
					}

					/***************************************** Entry 2 ***************************************/
					// Entry 2: Update
					// If the receiver i thinks the winner of task j is also agent k (sender)
					// Update
					else if(agent.cbba_z_(0, j) == neighbor.idx_){
						//std::cout << "case 3" << std::endl;
						agent.cbba_z_(0, j) = neighbor.history_.z_history.back()(0, j);
						agent.cbba_y_(0, j) = neighbor.history_.y_history.back()(0, j);
					}

					/***************************************** Entry 3 ***************************************/

					// Entry 3: Update or Leave
					// If the receiver i thinks the winner of task j is not k or itself i but other agent
					else if (agent.cbba_z_(0, j) >= 0){
						// Compare the iteration of task j, find which one is the least information of task j
						// Update
						if (neighbor.history_.iter_neighbors_his.back()(0, agent.cbba_z_(0, j)) > agent.iteration_neighbors_(0, agent.cbba_z_(0, j))){
							//std::cout << "case 4" << std::endl;
							agent.cbba_z_(0, j) = neighbor.history_.z_history.back()(0, j);
							agent.cbba_y_(0, j) = neighbor.history_.y_history.back()(0, j);
						}
						// Update
						else if(neighbor.history_.y_history.back()(0, j) - agent.cbba_y_(0, j) > eps){
							//std::cout << "case 5" << std::endl;
							agent.cbba_z_(0, j) = neighbor.history_.z_history.back()(0, j);
							agent.cbba_y_(0, j) = neighbor.history_.y_history.back()(0, j);
						}
						// Equal scores: break the tie by selecting the winner as the agent with smaller index
						else if (abs(neighbor.history_.y_history.back()(0, j) - agent.cbba_y_(0, j)) <= eps){
							if (agent.cbba_z_(0, j) > neighbor.history_.z_history.back()(0, j)){
								//std::cout << "case 6" << std::endl;
								agent.cbba_z_(0, j) = neighbor.history_.z_history.back()(0, j);
								agent.cbba_y_(0, j) = neighbor.history_.y_history.back()(0, j);
							}
						}
					}

					/***************************************** Entry 4 ***************************************/
					// Entry 4: Update
					// If the agent i (receiver) has no idea about the winner
					else if(agent.cbba_z_(0, j) == -1){
						//std::cout << "case 7" << std::endl;
						agent.cbba_z_(0, j) = neighbor.history_.z_history.back()(0, j);
						agent.cbba_y_(0, j) = neighbor.history_.y_history.back()(0, j);
					}

					else
						std::cout << "Unknown winner value 1" << std::endl;
				}



		        /*********************************************************************************************************/
				/*********************************************************************************************************/
				/*********************************************************************************************************/

				// Entries 5 to 8
				// If current agent i (sender) thinks the winner of task j is agent k (receiver)
				else if (neighbor.history_.z_history.back()(0, j) == agent.idx_){

					/***************************************** Entry 5 ***************************************/
					// Entry 5
				    // if agent i (receiver) also agree with agent k (sender): agent i thinks the winner of task j is also itself i
				    // Leave
				    if (agent.cbba_z_(0, j) == agent.idx_)
				    	std::cout << "Do nothing Entry 5" << std::endl;

				    /***************************************** Entry 6 ***************************************/
				    // Entry 6
					// If agent i (receiver) thinks the winner of task j is agent k (sender)
					// Reset (Because the agent k will definitely be the first one to know its own information )
					else if (agent.cbba_z_(0, j) == neighbor.idx_){
						//std::cout << "case 7" << std::endl;
						agent.cbba_z_(0, j) = -1;
						agent.cbba_y_(0, j) = -1;
					}

				    /***************************************** Entry 7 ***************************************/
				    // Entry 7
					// If agent i thinks the winner of task j is not itself (agent k thinks), but other agent
					else if(agent.cbba_z_(0, j) >= 0){
						// Compare the iteration of agent k and i, find which one has the least information
						// Reset
						if (neighbor.history_.iter_neighbors_his.back()(0, agent.cbba_z_(0, j)) > agent.iteration_neighbors_(0, agent.cbba_z_(0, j))){
							// agent k should have the updated information
							//std::cout << "case 8" << std::endl;
							agent.cbba_z_(0, j) = -1;
							agent.cbba_y_(0, j) = -1;
						}
					}

				    /***************************************** Entry 8 ***************************************/
				    // Entry 8
					else if(agent.cbba_z_(0, j) == -1)
						std::cout <<"Do nothing Entry 8" << std::endl;

					else
						std::cout << "Unknown winner value 2" << std::endl;

				}

				/*********************************************************************************************************/
				/*********************************************************************************************************/
				/*********************************************************************************************************/
				// Entries 9 to 13
				// If agent k (sender) thinks the winner of task j is not itself k and not receiver i,but other agent
				else if (neighbor.history_.z_history.back()(0, j) >= 0){
					/***************************************** Entry 9 ***************************************/
					// Entry 9
					// if agent i (receiver) thinks the winner of task j should be itself i
					if (agent.cbba_z_(0, j) == agent.idx_){
						// compare the iteration that agent k and i talk to the winner that agent k thinks
						// If agent k (sender) has the least information
						if (neighbor.history_.iter_neighbors_his.back()(0, neighbor.history_.z_history.back()(0, j)) > agent.iteration_neighbors_(0, neighbor.history_.z_history.back()(0, j))){
							// Update
							if (neighbor.history_.y_history.back()(0, j) - agent.cbba_y_(0, j) > eps){
								//std::cout << "case 8" << std::endl;
								agent.cbba_z_(0, j) = neighbor.history_.z_history.back()(0, j);
								agent.cbba_y_(0, j) = neighbor.history_.y_history.back()(0, j);
							}
							
							// If we have a tie: break the tie by selecting the agent with smaller index as the winner
							else if (abs(neighbor.history_.y_history.back()(0, j) - agent.cbba_y_(0, j)) <= eps){
								if (agent.cbba_z_(0, j) > neighbor.history_.z_history.back()(0, j)){
									// Update
									//std::cout << "case 9" << std::endl;
									agent.cbba_z_(0, j) = neighbor.history_.z_history.back()(0, j);
									agent.cbba_y_(0, j) = neighbor.history_.y_history.back()(0, j);
								}
							}
						}
					}

					/***************************************** Entry 10 ***************************************/
					// Entry 10
					// if agent i (receiver) thinks the winner of task j is agent k (sender)
					else if (agent.cbba_z_(0, j) == neighbor.idx_){
						// Compare the iteration of agent k and i, which one has the least information about the winner that agent k thinks
						if (neighbor.history_.iter_neighbors_his.back()(0, neighbor.history_.z_history.back()(0, j)) > agent.iteration_neighbors_(0, neighbor.history_.z_history.back()(0, j))){
							//std::cout << "case 10" << std::endl;
							agent.cbba_z_(0, j) = neighbor.history_.z_history.back()(0, j);
							agent.cbba_y_(0, j) = neighbor.history_.y_history.back()(0, j);
						}
						else{
							// Reset
							//std::cout << "case 11" << std::endl;
							agent.cbba_z_(0, j) = -1;
							agent.cbba_y_(0, j) = -1;
						}
					}

					/***************************************** Entry 11 ***************************************/
					// Entry 11
					// If agent i (receiver) agree with agent k and thinks the winner of task j is not i k, but other agent history_z(j)
					else if(agent.cbba_z_(0, j) == neighbor.history_.z_history.back()(0, j)){
						// Update
						if (neighbor.history_.iter_neighbors_his.back()(0, neighbor.history_.z_history.back()(0, j)) > agent.iteration_neighbors_(0, neighbor.history_.z_history.back()(0, j))){
							//std::cout << "case 12" << std::endl;
							agent.cbba_z_(0, j) = neighbor.history_.z_history.back()(0, j);
							agent.cbba_y_(0, j) = neighbor.history_.y_history.back()(0, j);
						}
					}

					/***************************************** Entry 12 ***************************************/
					// Entry 12
					// If agent i (receiver) thinks the winner of task j is not itself, agent k, the one that agent k thinks
					else if (agent.cbba_z_(0, j) >= 0){
						if (neighbor.history_.iter_neighbors_his.back()(0, neighbor.history_.z_history.back()(0, j)) > agent.iteration_neighbors_(0, neighbor.history_.z_history.back()(0, j))){
							// Update
							if (neighbor.history_.iter_neighbors_his.back()(0, agent.cbba_z_(0, j)) > agent.iteration_neighbors_(0, agent.cbba_z_(0, j))){
								//std::cout << "case 13" << std::endl;
								agent.cbba_z_(0, j) = neighbor.history_.z_history.back()(0, j);
								agent.cbba_y_(0, j) = neighbor.history_.y_history.back()(0, j);
							}
							// Update
							if (neighbor.history_.y_history.back()(0, j) - agent.cbba_y_(0, j) > eps){
								//std::cout << "case 14" << std::endl;
								agent.cbba_z_(0, j) = neighbor.history_.z_history.back()(0, j);
								agent.cbba_y_(0, j) = neighbor.history_.y_history.back()(0, j);
							}
							else if (abs(neighbor.history_.y_history.back()(0, j) - agent.cbba_y_(0, j)) <= eps){
								if(agent.cbba_y_(0, j) > neighbor.history_.y_history.back()(0, j)){
									// std::cout << "case 15" << std::endl;
									agent.cbba_z_(0, j) = neighbor.history_.z_history.back()(0, j);
									agent.cbba_y_(0, j) = neighbor.history_.y_history.back()(0, j);
								}
							}
							else
								std::cout << "Should not be here Entry 12" << std::endl;
						}
						else{
							if (neighbor.history_.iter_neighbors_his.back()(0, agent.cbba_z_(0, j)) > agent.iteration_neighbors_(0, agent.cbba_z_(0, j))){
								// Reset
								// std::cout << "case 15" << std::endl;
								agent.cbba_z_(0, j) = -1;
								agent.cbba_y_(0, j) = -1;
							}
						}
					}
					/***************************************** Entry 13 ***************************************/
					// Entry 13
					else if(agent.cbba_z_(0, j) == -1){
						// Update
						if (neighbor.history_.iter_neighbors_his.back()(0, neighbor.history_.z_history.back()(0, j)) > agent.iteration_neighbors_(0, neighbor.history_.z_history.back()(0, j))){
							//std::cout << "case 17" << std::endl;
							agent.cbba_z_(0, j) = neighbor.history_.z_history.back()(0, j);
							agent.cbba_y_(0, j) = neighbor.history_.y_history.back()(0, j);
						}
					}
					else
						std::cout << "Unknown winner value Entry 13" << std::endl;

				}

				/*********************************************************************************************************/
				/*********************************************************************************************************/
				/*********************************************************************************************************/
				// Entries 14 to 17
				else if (neighbor.history_.z_history.back()(0, j) == -1){

					/***************************************** Entry 14 ***************************************/
					// Entry 14
					// Leave
					if (agent.cbba_z_(0, j) == agent.idx_)
						std::cout << "Do nothing Entry 14" << std::endl;

					/***************************************** Entry 15 ***************************************/
					// Entry 15
					// Update
					else if (agent.cbba_z_(0, j) == neighbor.idx_){
						//std::cout << "case 18" << std::endl;
						agent.cbba_z_(0, j) = neighbor.history_.z_history.back()(0, j);
						agent.cbba_y_(0, j) = neighbor.history_.y_history.back()(0, j);
					}

					/***************************************** Entry 16 ***************************************/
					// Entry 16
					// Update
					else if (agent.cbba_z_(0, j) >= 0){
						// Update
						if (neighbor.history_.iter_neighbors_his.back()(0, agent.cbba_z_(0, j)) > agent.iteration_neighbors_(0, agent.cbba_z_(0, j))){
							//std::cout << "case 19" << std::endl;
							agent.cbba_z_(0, j) = neighbor.history_.z_history.back()(0, j);
							agent.cbba_y_(0, j) = neighbor.history_.y_history.back()(0, j);
						}
					}

					/***************************************** Entry 17 ***************************************/
					// Entry 17
					// Leave
					else if(agent.cbba_z_(0, j) == -1)
						std::cout<< "Do noting Entry 17" << std::endl;

					else
						std::cout << "Unknown winner value Entry 17" <<std::endl;
				}

				else
					std::cout << "Unknown winner value end of communicate" <<std::endl;
			}

			for (int n = 0; n < agent.num_agents_; n++){
				if (n != neighbor.idx_ && agent.iteration_neighbors_(0, n) < neighbor.history_.iter_neighbors_his.back()(0, n)){
					agent.iteration_neighbors_(0, n) = neighbor.history_.iter_neighbors_his.back()(0, n);
				}
			}
			agent.iteration_neighbors_(0, neighbor.idx_) = agent.iter_;
		}
			
	}
};


/*** Remove outbid independent tasks from task bundle and task path ***/
void Agent::bundle_remove(){
	bool outbidForTask = 0;
	if (!cbba_bundle_.empty()){
		// Check whether agent is outbid by other agent for the tasks which are in the bundle
		for (int k = 0; k < cbba_bundle_.size(); k++){
			if (cbba_z_(0, cbba_bundle_[k]) != idx_)
				outbidForTask = 1;

			// Remove the tasks added after the outbid task
			if (outbidForTask == 1){
				if (cbba_z_(0, cbba_bundle_[k]) == idx_){
					cbba_z_(0, cbba_bundle_[k]) = -1;
					cbba_y_(0, cbba_bundle_[k]) = -1;
				}

				// Once the path is taken into consideration
				// we need to find the position of task in the path
				// remove the task from the bundle
				cbba_bundle_[k] = -1;
			}
		}
	}

	// Remove the task which has -1
	cbba_bundle_.erase(std::remove(cbba_bundle_.begin(), cbba_bundle_.end(),-1), cbba_bundle_.end());

	path_remove();
};

/*** Remove independent tasks from task path ***/
void Agent::path_remove(){
	// Remove the tasks in the path but not in the bunlde (after bundle_remove)
	std::vector<int> cbba_path_copy = cbba_path_;
	for (int j = 0; j < cbba_path_copy.size(); j++){
		std::vector<int>::iterator it = std::find(cbba_bundle_.begin(), cbba_bundle_.end(), cbba_path_copy[j]);
		if(it == cbba_bundle_.end()){
			cbba_path_.erase(std::remove(cbba_path_.begin(), cbba_path_.end(),cbba_path_copy[j]),cbba_path_.end());
		}
	}	
}

/*** Remove outbid independent tasks from all vehicles' task bundle and task path ***/
void TaskAssignment::bundle_remove(std::vector<Agent>& agents){
	for (auto &agent: agents){
		agent.bundle_remove();
	}
}

/*** Check is the convergence of CBBA is achieved ***/
bool TaskAssignment::success_checker(std::vector<Agent> agents){
	bool flag = true;
	for (int j = 0; j < agents[0].num_tasks_inde_; j++){
		double winner = -1.0;
		double bid = -1.0;
		for (auto &agent: agents){
			
			if(winner == -1.0 && bid == -1.0){
				winner = agent.cbba_z_(0, j);
				bid = agent.cbba_y_(0,j);
			}
			else{
				if(agent.cbba_z_(0,j) != winner || agent.cbba_y_(0,j) != bid){
					flag = false;
					break;
				}
				else{
					continue;
				}
			}
			if(flag == false){break;}
		}
		if(flag == false){break;}
	}
	return flag;
};



/****************************************************************************************************/
/****************************************************************************************************/
/*********************************** Synchronization Algorithm **************************************/
/****************************************************************************************************/
/****************************************************************************************************/
/*** Update syn_c for dependent tasks ***/
void Agent::update_reward_dependent(TasksList tasks, std::shared_ptr<Graph_t<SquareCell*>> graph){
	for (auto &task: tasks.tasks_){
		if (task.num_agents_ > 1){
			std::vector<int>::iterator it = std::find(cbba_path_.begin(), cbba_path_.end(), task.idx_);
			if(it == cbba_path_.end()){
				std::vector<std::vector<int>> path_poss = TaskAssignment::PossibleInsertPosFinder(cbba_path_, task.idx_);
				std::vector<double> c_rewards = {};
				for (auto &path_case: path_poss){
					c_rewards.push_back(TaskAssignment::PathLengthCalculationBasedType(tasks, graph, path_case, init_pos_));	
				}
				

				double min_reward = syn_benefit;
				double best_pos = -1;
				for (int i = 0; i < c_rewards.size(); i++){
					if(c_rewards[i] < min_reward){
						min_reward = c_rewards[i];
						best_pos = i;
					}
				}
				
				syn_c_(0, task.idx_) = syn_benefit - min_reward;
				insert_pos_(0, task.idx_) = best_pos;
			}
		}
	}
}

/*** Find the available dependent tasks ***/
void Agent::available_dep_tasks_finder(TasksList tasks, std::shared_ptr<Graph_t<SquareCell*>> graph){
	update_reward_dependent(tasks, graph);
	h_avai_de_ = {};
	for (auto &task: tasks.tasks_){
		if (task.num_agents_ > 1){
			int de_pos_ = -1;
	
			int num_winners = winners_count(task);
			std::vector<int>::iterator it = std::find(cbba_bundle_.begin(), cbba_bundle_.end(), task.idx_);
			if (last_pos_dependent_ == -1){
				de_pos_ = insert_pos_(0, task.idx_);
			}
			else{
				de_pos_ = last_pos_dependent_;
			}

			if(it == cbba_bundle_.end()){
				if (num_winners < task.num_agents_ && insert_pos_(0, task.idx_) >= de_pos_){
					h_avai_de_.push_back(task.idx_);
				}
				else if(num_winners < task.num_agents_ && insert_pos_(0, task.idx_) < de_pos_){
					insert_pos_(0, task.idx_) = last_pos_dependent_;
					h_avai_de_.push_back(task.idx_);
				}
				else if(num_winners == task.num_agents_ && insert_pos_(0, task.idx_) >= de_pos_){
					h_avai_de_.push_back(task.idx_);
				}
			}
		}
	}	
}

/*** Find the desired dependent task from the list of available dependent tasks ***/
int Agent::desired_dep_task_finder(){
	int desired_idx = -1;
	if (!h_avai_de_.empty()){
		double max_reward = 0.0;
		for (auto &t_idx: h_avai_de_){
			if(syn_c_(0, t_idx) > max_reward){
				max_reward = syn_c_(0, t_idx);
				desired_idx = t_idx;
			}
		}
	}
	return desired_idx;
}

/*** Insert dependent tasks into task bundle and task path ***/
void Agent::bundle_add_dependent(TasksList tasks, std::shared_ptr<Graph_t<SquareCell*>> graph){
	bool bundleFull = 0;
	if(cbba_bundle_.size() >= max_tasks_){
		bundleFull = 1;
	}

	last_pos_dependent_ = -1;
	if (!cbba_path_.empty()){
		for (int i = cbba_path_.size()-1; i >= 0; i--){
			if(cbba_path_[i] >= num_tasks_inde_){
				last_pos_dependent_ = i + 1;
				break;
			}
		}	
	}
	while (bundleFull == 0){
		available_dep_tasks_finder(tasks, graph);
		int desired_idx = desired_dep_task_finder();
		if(desired_idx == -1){
			break;
		}

		cbba_bundle_.push_back(desired_idx);
		std::vector<int>::iterator it = cbba_path_.begin();
		cbba_path_.insert(it+insert_pos_(0, desired_idx), desired_idx);
		
		if(cbba_bundle_.size() > max_tasks_){
			bundleFull = 1;
		}
	}
	
	/* ========================================= Debug =====================================*/
	// std::cout << "After inserting all dependent tasks, the optimal sequence is " << std::endl;
	// for (auto &b: cbba_path_){
	// 	std::cout << b << ", ";
	// }
	// std::cout << std::endl;
	// std::cout << "Current winning bids matrix is " << std::endl;
	// std::cout << winning_bids_matrix_ << std::endl;
	// std::cout << "The assignment matrix is " << std::endl;
	// std::cout << assignment_matrix_ << std::endl;

	std::vector<int> current_p = cbba_path_;
	for (int t_idx = 0; t_idx < current_p.size(); t_idx++){
		cbba_Task& task = tasks.FindTask(current_p[t_idx]);
		if(task.num_agents_ > 1){
			std::vector<int> path_copy = cbba_path_;
			std::vector<int>::iterator it = std::find(cbba_path_.begin(), cbba_path_.end(), task.idx_);
			int task_length = it - cbba_path_.begin();
			path_copy.resize(task_length+1);	

			double path_part = syn_benefit - TaskAssignment::PathLengthCalculationBasedType(tasks, graph, path_copy, init_pos_);
			cbba_reward_(0, task.idx_) = path_part;
			winning_bids_matrix_(task.idx_, idx_) = path_part;

			optimal_group_finder(task);

			if (assignment_matrix_(task.idx_,idx_) == 0){
				cbba_path_.erase(std::remove(cbba_path_.begin(), cbba_path_.end(), task.idx_), cbba_path_.end());
				cbba_bundle_.erase(std::remove(cbba_bundle_.begin(), cbba_bundle_.end(), task.idx_), cbba_bundle_.end());
			}
			
			/*** ======================================== DEBUG =============================== ***/
			//std::cout << "============================= DEBUG =============================== " << std::endl;
			// std::cout << "The Result of converging optimal group of vehicle about task " << task.idx_ << " is: " << std::endl;
			// for (auto &b: cbba_path_){
			// 	std::cout << b << ", ";
			// }
			// std::cout << std::endl;
			// std::cout << "Now winning bids matrix becomes " << std::endl;
			// std::cout << winning_bids_matrix_ << std::endl;
			// std::cout << "The assignment matrix becomes " << std::endl;
			// std::cout << assignment_matrix_ << std::endl;
		}
	}

	history_.assignment_.push_back(assignment_matrix_);
	history_.winning_bids_.push_back(winning_bids_matrix_);
}

/*** Update the optimal group based on current knowledge of reward ***/
void Agent::optimal_group_finder(cbba_Task task){
	// Find the ordered map
	std::multimap<double, int> winning_bids_collect_ = {};
	for (int i = 0; i < num_agents_; i++){
		if(winning_bids_matrix_(task.idx_, i) > 0){
			winning_bids_collect_.insert(std::pair<double,int>(winning_bids_matrix_(task.idx_,i),i));
		}
	}

	int n = winning_bids_collect_.size();
	int k = task.num_agents_;

	std::multimap<double, int>::iterator it;
	std::vector<double> ordered_bids;
	for (auto it = winning_bids_collect_.begin(); it != winning_bids_collect_.end(); it++){
		ordered_bids.push_back((*it).first);
	}
	std::cout << std::endl;

	std::multimap<double, int>::iterator it1;
	double waiting_time_min = syn_benefit;
	std::vector<double> winners_bids;
	for (auto it1 = winning_bids_collect_.begin(); it1 != winning_bids_collect_.end(); it1++){
		double x = (*it1).first;
		std::vector<double> winning_bids = TaskAssignment::KClosestFinder(ordered_bids, x, k, n);
		double max_bid = TaskAssignment::maximum_bid_finder(winning_bids);
		double min_bid = TaskAssignment::minimum_bid_finder(winning_bids);
		double waiting_time = max_bid - min_bid;
		if(waiting_time <= waiting_time_min){
			waiting_time_min = waiting_time;
			winners_bids = winning_bids;
		}
	}

	std::vector<int> winners = FindVehicleFrombid(task, winners_bids);
	for (int i = 0; i < num_agents_;i++){
		assignment_matrix_(task.idx_, i) = 0;
	}
	for (auto &e: winners){
		assignment_matrix_(task.idx_, e) = 1;
	}
	if (assignment_matrix_(task.idx_, idx_) == 0){
		winning_bids_matrix_(task.idx_, idx_) = 0.0;
	}
}

/*** Find the lists of index of vehicles with given list of bids ***/
std::vector<int> Agent::FindVehicleFrombid(cbba_Task task, std::vector<double> winners_bid){
	std::vector<int> winners;
	for (auto &bid: winners_bid){
		for (int i = 0; i < num_agents_; i++){
			std::vector<int>::iterator exist = std::find(winners.begin(), winners.end(), i);
			if(bid == winning_bids_matrix_(task.idx_, i) && winning_bids_matrix_(task.idx_, i) != 0.0 && exist == winners.end()){
				winners.push_back(i);
				break;
			}
		}
	}
	return winners;
}

/*** Find the winners of given dependent task ***/
std::vector<int> Agent::winners_finder(cbba_Task task){
	std::vector<int> winners_ = {};
	for (int i =0; i < num_agents_; i++){
		if(assignment_matrix_(task.idx_, i) == 1 && winning_bids_matrix_(task.idx_, i) > 0){
			winners_.push_back(i);
		}
	}
	return winners_;
}

/*** Find the number of winners of given dependent task ***/
int Agent::winners_count(cbba_Task task){
	std::vector<int> winners = winners_finder(task);
	return winners.size();
}

/*** Remove the outbid dependent tasks from task path ***/
void Agent::path_remove_dependent(TasksList tasks){
	bool outbidForTask = 0;
	for (int t_idx = 0; t_idx < cbba_path_.size(); t_idx++){
		cbba_Task& task = tasks.FindTask(cbba_path_[t_idx]);
		if (task.num_agents_ > 1 && assignment_matrix_(cbba_path_[t_idx], idx_) == 0){
			outbidForTask = 1;
			cbba_path_[t_idx] = -1;
			winning_bids_matrix_(task.idx_, idx_) = 0.0;
		}

		if(task.num_agents_ > 1 && outbidForTask == 1){
			// cbba_path_[t_idx] = -1;
			winning_bids_matrix_(task.idx_, idx_) = 0.0;
			assignment_matrix_(task.idx_, idx_) = 0;
		}
	}
	cbba_path_.erase(std::remove(cbba_path_.begin(), cbba_path_.end(),-1), cbba_path_.end());
	bundle_remove_dependent();
}

/*** Remove outbit dependent tasks from task bundle ***/
void Agent::bundle_remove_dependent(){
	std::vector<int> cbba_bundle_copy = cbba_bundle_;
	for (int j = 0; j < cbba_bundle_copy.size(); j++){
		std::vector<int>::iterator it = std::find(cbba_path_.begin(), cbba_path_.end(), cbba_bundle_copy[j]);
		if(it == cbba_path_.end()){
			cbba_bundle_.erase(std::remove(cbba_bundle_.begin(), cbba_bundle_.end(), cbba_bundle_copy[j]), cbba_bundle_.end());
		}
	}
}

/*** Remove dependent tasks from task path for all vehicles ***/
void TaskAssignment::path_remove_dependent(TasksList tasks, std::vector<Agent>& agents){
	for (auto &agent: agents){
		agent.path_remove_dependent(tasks);
	}
}

/*** Communicate with neighbors for dependent tasks ***/
void TaskAssignment::communicate_dependent(std::vector<Agent>& agents, TasksList tasks){
	for (auto &agent: agents){
		agent.neighbor_finder();
		for (auto &neighbor_idx: agent.neighbors_){
			Agent& neighbor = TaskAssignment::FindAgent(agents, neighbor_idx);
			// std::cout << " ============================= Debug =========================== " << std::endl;
			// std::cout << "Before communication " << std::endl;
			// std::cout << "Vehicle " << agent.idx_ << std::endl;
			// std::cout << "The winning_bids matrix is " << std::endl;
			// std::cout << agent.winning_bids_matrix_ << std::endl;
			// std::cout << "The assignment matrix is " << std::endl;
			// std::cout << agent.assignment_matrix_ << std::endl;

			// std::cout << "Neighbor " << neighbor.idx_ << std::endl;
			// std::cout << "The winning_bids matrix is " << std::endl;
			// std::cout << neighbor.history_.winning_bids_.back() << std::endl;
			// std::cout << "The assignment matrix is " << std::endl;
			// std::cout << neighbor.history_.assignment_.back() << std::endl;
			// std::cout << "Neighbors iteration is "<< std::endl;
			// std::cout << neighbor.history_.iter_neighbors_his.back() << std::endl;
			for (int i = 0; i < agent.num_agents_; i++){
				for (auto &task: tasks.tasks_){
					if(task.num_agents_ > 1){
						if(i != agent.idx_ && i != neighbor.idx_ && neighbor.history_.iter_neighbors_his.back()(0, i) >= agent.iteration_neighbors_(0, i)){
							agent.winning_bids_matrix_(task.idx_, i) = neighbor.history_.winning_bids_.back()(task.idx_,i);	
						}
						else if(i == neighbor.idx_){
							agent.winning_bids_matrix_(task.idx_, i) = neighbor.history_.winning_bids_.back()(task.idx_,i);
						}
					}
				}
				if (i != agent.idx_ && agent.iteration_neighbors_(0, i) <= neighbor.history_.iter_neighbors_his.back()(0, i)){
					agent.iteration_neighbors_(0, i) = neighbor.history_.iter_neighbors_his.back()(0, i);
				}
			}
			// std::cout << "The result for vehicle " << agent.idx_ << " talking to neighbor " << neighbor.idx_ << " is: " << std::endl;
			// std::cout << "The winning bids matrix is " << std::endl;
			// std::cout << agent.winning_bids_matrix_ << std::endl;
			// std::cout << "The iteration becomes " << std::endl;
			// std::cout << agent.iteration_neighbors_ << std::endl;
			// std::cout << "The assignment matrix is " << std::endl;
			// std::cout << agent.assignment_matrix_ << std::endl;
		}
		for (auto &task: tasks.tasks_){
			if(task.num_agents_ > 1){
				agent.optimal_group_finder(task);
			}
		}
		// std::cout << "Assignment Update for " << agent.idx_ << " based on latest information is " << std::endl;
		// 	std::cout << "The winning bids matrix is " << std::endl;
		// 	std::cout << agent.winning_bids_matrix_ << std::endl;
		// 	std::cout << "The assignment matrix is " << std::endl;
		// 	std::cout << agent.assignment_matrix_ << std::endl;
	}

	for (auto &agent: agents){
		agent.history_.winning_bids_.push_back(agent.winning_bids_matrix_);
		agent.history_.assignment_.push_back(agent.assignment_matrix_);
	}
}

/*** Find the maximum bid with given list of bids ***/
double TaskAssignment::maximum_bid_finder(std::vector<double> winning_bids){
	double max_bid = 0.0;
	for (auto &e:winning_bids){
		if(e > max_bid){
			max_bid = e;
		}
	}
	return max_bid;
}

/*** Find the minimum bid with given list of bids ***/
double TaskAssignment::minimum_bid_finder(std::vector<double> winning_bids){
	double min_bid = cbba_benefit;
	for (auto &e: winning_bids){
		if(e < min_bid){
			min_bid = e;
		}
	}
	return min_bid;
}

/*** Find the K closest reward ***/
std::vector<double> TaskAssignment::KClosestFinder(std::vector<double> ordered_bids, int x, int k, int n){
	std::vector<double> cloest_set;
	int l = TaskAssignment::findCrossOver(ordered_bids, 0, n-1, x);
	int r = l + 1;
	int count = 0;

	if(ordered_bids[l] == x){
		cloest_set.push_back(ordered_bids[l]);
		count = count + 1;
		l--;
	}

	while (l >= 0 && r < n && count < k){
		if(x - ordered_bids[l] < ordered_bids[r] - x){
			cloest_set.push_back(ordered_bids[l--]);
		}
		else{
			cloest_set.push_back(ordered_bids[r++]);
		}
		count ++;
	}

	while (count < k && l >= 0){
		cloest_set.push_back(ordered_bids[l--]);
		count ++;
	}

	while (count < k && r < n){
		cloest_set.push_back(ordered_bids[r++]);
		count ++;
	}
	return cloest_set;
}
/*** Implement to find the K closest reward ***/
int TaskAssignment::findCrossOver(std::vector<double> ordered_bids, int low, int high, int x){

	if (ordered_bids[high] <= x){
		return high;
	}
	if (ordered_bids[low] > x){
		return low;
	}

	int mid = (low + high)/2;

	if(ordered_bids[mid] <= x && ordered_bids[mid+1] > x){
		return mid;
	}

	if(ordered_bids[mid] < x){
		return TaskAssignment::findCrossOver(ordered_bids, mid+1, high, x);
	}

	return TaskAssignment::findCrossOver(ordered_bids, low, mid-1, x);
}

/*** Bundle add for all vehicles ***/
void TaskAssignment::bundle_add_dependent(TasksList tasks, const std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<Agent>& agents){
	for (auto &agent: agents){
		agent.bundle_add_dependent(tasks, graph);

		agent.iteration_neighbors_(0, agent.idx_)++;
		agent.history_.iter_neighbors_his.push_back(agent.iteration_neighbors_);
	}
}

/*** Find the winning bids for given dependent task ***/
std::vector<double> Agent::winning_bids_finder(cbba_Task task){
	std::vector<int> winners = winners_finder(task);
	std::vector<double> winning_bids = {};
	if (!winners.empty()){
		for(auto & vehicle: winners){
			winning_bids.push_back(winning_bids_matrix_(task.idx_, vehicle));
		}
	}
	return winning_bids;
}

/*** Implement to check is the convergence of synchronization algorithm is achieved ***/
bool TaskAssignment::success_checker_dependent(std::vector<Agent> agents, TasksList tasks){
    bool success = true;
    for (auto &task: tasks.tasks_){
		if (task.num_agents_ > 1){
			std::vector<int> winners = {};
			std::vector<double> bids = {};
			for (auto &agent: agents){
				if (winners.empty() && bids.empty()){
					winners = agent.winners_finder(task);
					bids = agent.winning_bids_finder(task);

					if (winners.size() != task.num_agents_){
						success = false;
						break;
					}
				}
				else{
					std::vector<int> new_winners = agent.winners_finder(task);
					std::vector<double> new_bids = agent.winning_bids_finder(task);
					
					if (new_winners.size() != winners.size() || new_bids.size() != bids.size()){
						success = false;
						break;
					}
					else{
						for (int i = 0; i < winners.size(); i++){
							if (new_winners[i] != winners[i]){
								success = false;
								break;
							}
						}
						for (int j = 0; j < bids.size(); j++){
							if(new_bids[j] != bids[j]){
								success = false;
								break;
							}
						}
					}
				}
			}
		}
    }
    return success;
}



/****************************************************************************************************/
/****************************************************************************************************/
/************************************ Waiting time computation **************************************/
/****************************************************************************************************/
/****************************************************************************************************/
/*** Compute the waiting time required by certain task assignment ***/
double TaskAssignment::WaitingTimeCalculation(std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<Agent>& agents, TasksList tasks){
	std::vector<int> dep_list;
	// Prepare tasks path only contain dependent tasks
	std::map<int, std::vector<int>> vehicle_path;

	while (dep_list.size() != agents[0].num_tasks_de_){
		std::vector<int> first_tasks = {};
		for (auto& agent: agents){
			for (auto& t_idx: agent.cbba_path_){
				std::vector<int>::iterator it = std::find(dep_list.begin(), dep_list.end(), t_idx);
				if (t_idx >= agent.num_tasks_inde_ && it == dep_list.end()){
					first_tasks.push_back(t_idx);
					break;
				}
			}
		}

		for (auto& t: first_tasks){
			cbba_Task& task = tasks.FindTask(t);
			int num_appearance = FindNumAppearance(first_tasks, t);
			std::vector<int>::iterator it1 = std::find(dep_list.begin(), dep_list.end(), task.idx_);
			if(num_appearance == task.num_agents_ && it1 == dep_list.end()){
				dep_list.push_back(task.idx_);
			}
		}
	}

	std::cout << "The result of sequence of dependent list is " << std::endl;
	for (auto& m: dep_list){
		std::cout << m << ", ";
	}
	std::cout << std::endl;


	// Update the reward for all dependent tasks
	for(auto& b: dep_list){
		double max_reward = 0.0;
		cbba_Task& task = tasks.FindTask(b);
		std::vector<int> winners = FindWinners(agents, b);
		std::cout << "Winner for task " << b << " is: ";
		for (auto& k: winners){
			std::cout << k << ", ";
		}
		std::cout << std::endl;
		for (auto& a: winners){
			Agent& agent = TaskAssignment::FindAgent(agents, a);
			std::vector<int> path = agent.cbba_path_;
			std::vector<int>::iterator it = std::find(agent.cbba_path_.begin(), agent.cbba_path_.end(), b);
			int task_length = it - agent.cbba_path_.begin();
			path.resize(task_length+1);

			std::cout << "The path info is " << std::endl;
			for (auto& i: path){
				std::cout << i << ", ";
			}
			std::cout << std::endl;
			

			double r = TaskAssignment::MaximumRewardCalculation(graph, path, agent.init_pos_, tasks);
			if(r > max_reward){
				max_reward = r;
			}
		}
		task.max_reward_ = max_reward;
	}

	std::cout << "Test max reward for each dependent task " << std::endl;
	for(auto& bb: dep_list){
		cbba_Task& task = tasks.FindTask(bb);
		std::cout << "For task " << bb << " ,the max reward is " << task.max_reward_ << std::endl;
	}

	double total_waiting = 0;
	for (auto& agent: agents){
		agent.UpdateWaitingTime(graph, tasks);
		total_waiting = total_waiting + agent.waiting_t_;
		std::cout << "For agent " << agent.idx_ << " the waiting time is " << agent.waiting_t_ << std::endl;
	}
	
	std::cout << "TOTAL WAITING TIME IS "<< total_waiting << std::endl;
	
	return 0.0;
}

/*** Compute the waiting time for each vehicle ***/
void Agent::UpdateWaitingTime(std::shared_ptr<Graph_t<SquareCell*>> graph, TasksList tasks){
	waiting_t_ = 0.0;
	for (int i = 0; i < cbba_path_.size(); i++){
		cbba_Task task = tasks.FindTaskFromID(cbba_path_[i]);
		if(task.num_agents_ > 1){
			std::vector<int> path = cbba_path_;
			path.resize(i+1);
			double r = TaskAssignment::PathLengthCalculationWithWaiting(graph, path, init_pos_, tasks);
			if (r < task.max_reward_){
				waiting_t_ = waiting_t_ + (task.max_reward_ - r);
			}
		}
	}
}

/*** Compute the length of certain path by considering the waiting time ***/
double TaskAssignment::PathLengthCalculationWithWaiting(std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<int> bundle, int init_pos, TasksList tasks){
	Vertex_t<SquareCell*> * start_node = graph->GetVertexFromID(init_pos);
	std::vector<int64_t> pos_seq;
	std::vector<int64_t> task_seq;
	Path_t<SquareCell *> path_part;
	Path_t<SquareCell *> path;
	
	for (int i = 0; i < bundle.size(); i++){
		cbba_Task task = tasks.FindTaskFromID(bundle[i]);
		if(task.type_ == TaskType::VISIT){
			if(task.pos_.size() == 1){
				pos_seq.push_back(task.pos_[0]);
				task_seq.push_back(task.idx_);
			}
			else{
				for (int j = 0; j < task.pos_.size(); j++){
					pos_seq.push_back(task.pos_[j]);
					task_seq.push_back(task.idx_);
				}
			}
		}
		else if (task.type_ ==  TaskType::SEARCH){
			pos_seq.push_back(task.pos_[0]);
			task_seq.push_back(task.idx_);
		}
		else{
			std::cout << "Unknown task type. " << std::endl;
		}
	}
	
	for (int k = 0; k < task_seq.size(); k++){
		cbba_Task task = tasks.FindTaskFromID(task_seq[k]);
		Vertex_t<SquareCell*> * finish_node = graph->GetVertexFromID(pos_seq[k]);
		if(task.num_agents_ > 1){
			Path_t<SquareCell *> path_part = AStar::Search(graph,start_node->state_->id_,finish_node->state_->id_, CalcHeuristicFunc_t<SquareCell *>(TaskAssignment::CalcHeuristic));
			if (k == task_seq.size() - 1){
				path.insert(path.end(), path_part.begin(), path_part.end());
	
			}
			else{
				path.insert(path.end(), path_part.begin(), path_part.end()-1);

				if (task.max_reward_ != 0.0){
					while ( (task.max_reward_ - 1) > path.size()){
						path.push_back(finish_node->state_);
					}
				}
			}
		}
		else{
			Path_t<SquareCell *> path_part = AStar::Search(graph,start_node->state_->id_,finish_node->state_->id_, CalcHeuristicFunc_t<SquareCell *>(TaskAssignment::CalcHeuristic));
			if (k == task_seq.size() - 1){
				path.insert(path.end(), path_part.begin(), path_part.end());	
			}
			else{
				path.insert(path.end(), path_part.begin(), path_part.end()-1);
			}

		}
		start_node = finish_node;
	}

	double path_length = path.size();
	return path_length;
}

/*** Compute the maximum reward for each dependent tasks ***/
double TaskAssignment::MaximumRewardCalculation(std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<int> bundle, int init_pos, TasksList tasks){
	Vertex_t<SquareCell*> * start_node = graph->GetVertexFromID(init_pos);
	std::vector<int64_t> pos_seq;
	std::vector<int64_t> task_seq;
	Path_t<SquareCell *> path_part;
	Path_t<SquareCell *> path;
	
	for (int i = 0; i < bundle.size(); i++){
		cbba_Task task = tasks.FindTaskFromID(bundle[i]);
		if(task.type_ == TaskType::VISIT){
			if(task.pos_.size() == 1){
				pos_seq.push_back(task.pos_[0]);
				task_seq.push_back(task.idx_);
			}
			else{
				for (int j = 0; j < task.pos_.size(); j++){
					pos_seq.push_back(task.pos_[j]);
					task_seq.push_back(task.idx_);
				}
			}
		}
		else if (task.type_ ==  TaskType::SEARCH){
			pos_seq.push_back(task.pos_[0]);
			task_seq.push_back(task.idx_);
		}
		else{
			std::cout << "Unknown task type. " << std::endl;
		}
	}
	
	for (int k = 0; k < task_seq.size(); k++){
		cbba_Task task = tasks.FindTaskFromID(task_seq[k]);
		Vertex_t<SquareCell*> * finish_node = graph->GetVertexFromID(pos_seq[k]);
		if(task.num_agents_ > 1){
			Path_t<SquareCell *> path_part = AStar::Search(graph,start_node->state_->id_,finish_node->state_->id_, CalcHeuristicFunc_t<SquareCell *>(TaskAssignment::CalcHeuristic));
			if (k == task_seq.size() - 1){
				path.insert(path.end(), path_part.begin(), path_part.end());
				if (task.max_reward_ != 0.0){
					while ( task.max_reward_ > path.size()){
						path.push_back(finish_node->state_);
					}
				}	
			}
			else{
				path.insert(path.end(), path_part.begin(), path_part.end()-1);

				if (task.max_reward_ != 0.0){
					while ( (task.max_reward_ - 1) > path.size()){
						path.push_back(finish_node->state_);
					}
				}
			}
		}
		else{
			Path_t<SquareCell *> path_part = AStar::Search(graph,start_node->state_->id_,finish_node->state_->id_, CalcHeuristicFunc_t<SquareCell *>(TaskAssignment::CalcHeuristic));
			if (k == task_seq.size() - 1){
				path.insert(path.end(), path_part.begin(), path_part.end());	
			}
			else{
				path.insert(path.end(), path_part.begin(), path_part.end()-1);
			}

		}
		start_node = finish_node;
	}

	double path_length = path.size();
	return path_length;
}

/*** Find the num of appearance for certain target from a given list ***/
int TaskAssignment::FindNumAppearance(std::vector<int> list, int target){
	int num_appearance = 0;
	for (int i = 0; i < list.size(); i++){
		if(list[i] == target){
			num_appearance++;
		}
	}
	return num_appearance;
}

/*** Find the winners for certain dependent tasks ***/
std::vector<int> TaskAssignment::FindWinners(std::vector<Agent> agents, int task_idx){
	std::vector<int> winners_list;
	for (auto& agent: agents){
		std::vector<int>::iterator it = std::find(agent.cbba_path_.begin(), agent.cbba_path_.end(), task_idx);
		if(it != agent.cbba_path_.end()){
			winners_list.push_back(agent.idx_);
		}
	}
	return winners_list;
}



/****************************************************************************************************/
/****************************************************************************************************/
/******************************************** CBTA-CBBA *********************************************/
/****************************************************************************************************/
/****************************************************************************************************/
/*** Update the reward of independent tasks by CBTA ***/
void Agent::reward_update_CBTA(TasksList tasks, const std::shared_ptr<Graph_t<SquareCell*>> graph, std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph,  
			TileTraversalData tile_traversal_data, std::shared_ptr<SquareGrid> grid){

	// Find the task which is not in the bundle
	std::vector<int> tasks_NotInBundle = {};
	for (auto &task: tasks.tasks_){
		if (task.num_agents_ == 1){
			tasks_NotInBundle.push_back(task.idx_);
		}
	}
	// Find all tasks which have not been inserted into the bundle
	for (int j = 0; j < cbba_bundle_.size(); j++){
		// Erase all tasks which have been added into the bundle based on erase-remove idiom
		tasks_NotInBundle.erase(std::remove(tasks_NotInBundle.begin(),tasks_NotInBundle.end(),cbba_bundle_[j]),tasks_NotInBundle.end());
	}

	// Prepare all cases after inserting the task at each possible position
	for (int j = 0; j < tasks_NotInBundle.size();j++){
		// Clear c_award and bundle_copy for each task in the tasks_NotInBundle
		// c_award and bundle_copy are only for one task with different insert position
		std::vector<double> c_reward = {};
		
		// If the size of cbba_bundle is n, then there are n+1 positions to insert each task in the tasks_NotInBundle
		std::vector<std::vector<int>> path_copy(cbba_path_.size()+1, cbba_path_);

		// Insert one task (task_NotInBundle[j]) into all the possible position
		// Each item in the path_copy represents a possible position to insert the task
		for (int m = 0; m < path_copy.size(); m++){
			std::vector<int>::iterator it = path_copy[m].begin();
			path_copy[m].insert(it+m,tasks_NotInBundle[j]);

			// Calculate the Astar length for each possible insert position
			c_reward.push_back(TaskAssignment::PathLengthCalculationCBTA(tasks,graph,path_copy[m],init_pos_,lifted_graph,tile_traversal_data, grid));
		}

		float c_reward_min = cbba_benefit;
		for (int i = 0; i < c_reward.size(); i++)
			if (c_reward[i] < c_reward_min){
				c_reward_min = c_reward[i];
				insert_pos_(0, tasks_NotInBundle[j]) = i;
			}

		// Update the reward
		cbba_reward_(0, tasks_NotInBundle[j]) = cbba_benefit - c_reward_min;
	}

}

/*** Insert independent tasks into task bundle and task path by CBTA ***/
void Agent::bundle_add(TasksList tasks,const std::shared_ptr<Graph_t<SquareCell*>> graph,std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph, 
	TileTraversalData tile_traversal_data, std::shared_ptr<SquareGrid> grid){

	bool bundleFull = -1;

	// Check whether the bundle is full or not
	if (cbba_bundle_.size() > max_tasks_){
		std::cout << "The bundle is full" << std::endl;
		bundleFull = 1;
	}
	else
		bundleFull = 0;

	while (bundleFull == 0){
		reward_update_CBTA(tasks, graph, lifted_graph, tile_traversal_data, grid);
		int desired_idx = desired_task_finder();
		if (desired_idx == -1){
			break;
		}

		// update assignment
		cbba_z_(0, desired_idx) = idx_;
		cbba_y_(0, desired_idx) = cbba_reward_(0, desired_idx);

		// Insert desired task into task path
		std::vector<int>::iterator it = cbba_path_.begin();
		cbba_path_.insert(it+insert_pos_(0, desired_idx), desired_idx);

		cbba_bundle_.push_back(desired_idx);

		if (cbba_bundle_.size() >= max_tasks_){
			bundleFull = 1;
		}
	}

	history_.y_history.push_back(cbba_y_);
	history_.z_history.push_back(cbba_z_);
};

/*** Bundle add for all vehicles in the team by CBTA with same radius of turn ***/
void TaskAssignment::bundle_add(TasksList tasks,const std::shared_ptr<Graph_t<SquareCell*>> graph,std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph, 
	TileTraversalData tile_traversal_data, std::shared_ptr<SquareGrid> grid,std::vector<Agent>& agents){

	for (auto &agent: agents){
		agent.bundle_add(tasks,graph,lifted_graph,tile_traversal_data,grid);
	}
}

/*** Bundle add for all vehicles in the team by CBTA with different radius of turn ***/
void TaskAssignment::bundle_add(TasksList tasks, const std::shared_ptr<Graph_t<SquareCell*>> graph,std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph,
	std::map<int, TileTraversalData> tile_traversal_data, std::shared_ptr<SquareGrid> grid,std::vector<Agent>& agents){

	for (auto &agent: agents){
		agent.bundle_add(tasks,graph,lifted_graph,tile_traversal_data[agent.idx_],grid);
	}
}


/****************************************************************************************************/
/****************************************************************************************************/
/******************************************** CBTA-Syn **********************************************/
/****************************************************************************************************/
/****************************************************************************************************/
/*** Update syn_c for dependent tasks by CBTA ***/
void Agent::update_reward_dependent(TasksList tasks, std::shared_ptr<Graph_t<SquareCell*>> graph, std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph, 
	TileTraversalData tile_traversal_data, std::shared_ptr<SquareGrid> grid){
	for (auto &task: tasks.tasks_){
		if (task.num_agents_ > 1){
			std::vector<int>::iterator it = std::find(cbba_path_.begin(), cbba_path_.end(), task.idx_);
			if(it == cbba_path_.end()){
				std::vector<std::vector<int>> path_poss = TaskAssignment::PossibleInsertPosFinder(cbba_path_, task.idx_);
				std::vector<double> c_rewards = {};
				for (auto &path_case: path_poss){
					c_rewards.push_back(TaskAssignment::PathLengthCalculationCBTA(tasks, graph, path_case, init_pos_, lifted_graph, tile_traversal_data, grid));
				}
				double min_reward = syn_benefit;
				double best_pos = -1;
				for (int i = 0; i < c_rewards.size(); i++){
					if(c_rewards[i] < min_reward){
						min_reward = c_rewards[i];
						best_pos = i;
					}
				}
				syn_c_(0, task.idx_) = syn_benefit - min_reward;
				insert_pos_(0, task.idx_) = best_pos;
			}
		}
	}
}

/*** Find the available dependent tasks by CBTA ***/
void Agent::available_dep_tasks_finder(TasksList tasks, std::shared_ptr<Graph_t<SquareCell*>> graph,std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph,
	TileTraversalData tile_traversal_data, std::shared_ptr<SquareGrid> grid){
	
	update_reward_dependent(tasks, graph,lifted_graph,tile_traversal_data,grid);
	h_avai_de_ = {};
	for (auto &task: tasks.tasks_){
		if (task.num_agents_ > 1){
			int de_pos_ = -1;
	
			int num_winners = winners_count(task);
			std::vector<int>::iterator it = std::find(cbba_bundle_.begin(), cbba_bundle_.end(), task.idx_);
			if (last_pos_dependent_ == -1){
				de_pos_ = insert_pos_(0, task.idx_);
			}
			else{
				de_pos_ = last_pos_dependent_;
			}
			if(it == cbba_bundle_.end()){
				if (num_winners < task.num_agents_ && insert_pos_(0, task.idx_) >= de_pos_){
					h_avai_de_.push_back(task.idx_);
				}
				else if(num_winners < task.num_agents_ && insert_pos_(0, task.idx_) < de_pos_){
					insert_pos_(0, task.idx_) = last_pos_dependent_;
					h_avai_de_.push_back(task.idx_);
				}
				else if(num_winners == task.num_agents_ && insert_pos_(0, task.idx_) >= de_pos_){
					h_avai_de_.push_back(task.idx_);
				}
			}
		}
	}
}
/*** Add dependent tasks into task bundle and task path ***/
void Agent::bundle_add_dependent(TasksList tasks, std::shared_ptr<Graph_t<SquareCell*>> graph, std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph, 
		TileTraversalData tile_traversal_data, std::shared_ptr<SquareGrid> grid){
	
	bool bundleFull = 0;
	if(cbba_bundle_.size() >= max_tasks_){
		bundleFull = 1;
	}

	int last_pos_dependent_ = -1;
	for (int i = cbba_path_.size()-1; i >= 0; i--){
		if(cbba_path_[i] >= num_tasks_inde_){
			last_pos_dependent_ = i + 1;
			break;
		}
	}

	while (bundleFull == 0){
		available_dep_tasks_finder(tasks, graph,lifted_graph,tile_traversal_data,grid);
		int desired_idx = desired_dep_task_finder();
		if(desired_idx == -1){
			break;
		}

		cbba_bundle_.push_back(desired_idx);
		std::vector<int>::iterator it = cbba_path_.begin();
		cbba_path_.insert(it+insert_pos_(0, desired_idx), desired_idx);
		
		if(cbba_bundle_.size() > max_tasks_){
			bundleFull = 1;
		}
	}

	std::vector<int> current_p = cbba_path_;
	for (int t_idx = 0; t_idx < current_p.size(); t_idx++){
		cbba_Task task = tasks.FindTaskFromID(current_p[t_idx]);
		if(task.num_agents_ > 1){
			std::vector<int> path_copy = cbba_path_;
			std::vector<int>::iterator it = std::find(cbba_path_.begin(), cbba_path_.end(), task.idx_);
			int task_length = it - cbba_path_.begin();
			path_copy.resize(task_length+1);

			double path_part = syn_benefit - TaskAssignment::PathLengthCalculationCBTA(tasks, graph, path_copy, init_pos_,lifted_graph, tile_traversal_data, grid);
			cbba_reward_(0, task.idx_) = path_part;
			winning_bids_matrix_(task.idx_, idx_) = path_part;

			optimal_group_finder(task);
			if(assignment_matrix_(task.idx_, idx_) == 0){
				cbba_path_.erase(std::remove(cbba_path_.begin(), cbba_path_.end(), task.idx_), cbba_path_.end());
				cbba_bundle_.erase(std::remove(cbba_bundle_.begin(), cbba_bundle_.end(), task.idx_), cbba_bundle_.end());
			}
		}
	}

	history_.assignment_.push_back(assignment_matrix_);
	history_.winning_bids_.push_back(winning_bids_matrix_);
	
}

/*** Bundle add for all vehicles by CBTA with same radius of turn ***/
void TaskAssignment::bundle_add_dependent(TasksList tasks, std::shared_ptr<Graph_t<SquareCell*>> graph, std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph, 
		TileTraversalData tile_traversal_data, std::shared_ptr<SquareGrid> grid,std::vector<Agent>& agents){
	
	for (auto &agent:agents){
		agent.bundle_add_dependent(tasks,graph,lifted_graph,tile_traversal_data,grid);

		agent.iteration_neighbors_(0, agent.idx_)++;
		agent.history_.iter_neighbors_his.push_back(agent.iteration_neighbors_);
	}
}

/*** Bundle add for all vehicles by CBTA with different radius of turn ***/
void TaskAssignment::bundle_add_dependent(TasksList tasks, std::shared_ptr<Graph_t<SquareCell*>> graph, std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph, 
	std::map<int, TileTraversalData> tile_traversal_data, std::shared_ptr<SquareGrid> grid,std::vector<Agent>& agents){
	
	for (auto &agent:agents){
		agent.bundle_add_dependent(tasks,graph,lifted_graph,tile_traversal_data[agent.idx_],grid);

		agent.iteration_neighbors_(0, agent.idx_)++;
		agent.history_.iter_neighbors_his.push_back(agent.iteration_neighbors_);
	}
}
