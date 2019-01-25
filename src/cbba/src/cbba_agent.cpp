/*
 * cbba_Agent.cpp
 *
 *  Created on: Feb 15, 2017
 *      Author: jfang
 */

#include "cbba/cbba_agent.hpp"
#include "config_reader/config_reader.hpp"

using namespace librav;

cbba_Agent::cbba_Agent(int id, int startID, std::vector<int> com, int num_tasks_inde, int num_tasks_de)
{
	idx_ = id;
	start_node_ = startID;
	comm_topo_ = com;
	iter_ = 0;
	cbba_bundle_ = {};
	cbba_path_ = {};
	
	iteration_neighbors_ = std::vector<int>(com.size(),0);
	history_.iter_neighbors_his = {iteration_neighbors_};
	num_tasks_inde_ = num_tasks_inde;
	num_tasks_de_ = num_tasks_de;
	num_tasks_ = num_tasks_inde_ + num_tasks_de_;
	cbba_y_ = std::vector<float>(num_tasks_, 0.0);
	cbba_z_ = std::vector<int>(num_tasks_, -1);
	history_.y_history = {cbba_y_};
	history_.z_history = {cbba_z_};
	cbba_award_ = std::vector<float>(num_tasks_, -1.0);
	insert_pos_ = std::vector<int>(num_tasks_, -1);

	num_agents_ = com.size();
	
	max_tasks_ = num_tasks_inde + num_tasks_de;
	

	// Initialize dependent tasks
	assignment_matrix_ =   Eigen::MatrixXd::Zero(num_tasks_, num_agents_);
	winning_bids_matrix_ = -1 * Eigen::MatrixXd::Zero(num_tasks_, num_agents_);
	history_.assignment_ = {assignment_matrix_};
	history_.winning_bids_ = {winning_bids_matrix_};

};

std::vector<int> cbba_Agent::award_update(LTLFormula Global_LTL,const std::shared_ptr<Graph_t<SquareCell*>> graph){
	// Initialize the best position for all tasks
	std::vector<int> best_position(num_tasks_inde_, -1);
	// Initialize the tasks_NotInBundle
	std::vector<int> tasks_NotInBundle = {};
	// Put all the task index into the tasks_NotInBundle
	// std::cout << "******************************************************* num_tasks_inde_ " << num_tasks_inde_ << std::endl;
	for(int j = 0; j < num_tasks_inde_; j++){
		tasks_NotInBundle.push_back(j);
	}
		

	// Find all tasks which have not been inserted into the bundle
	for (int j = 0; j < cbba_bundle_.size(); j++)
		// Erase all tasks which have been added into the bundle based on erase-remove idiom
		tasks_NotInBundle.erase(std::remove(tasks_NotInBundle.begin(),tasks_NotInBundle.end(),cbba_bundle_[j]),tasks_NotInBundle.end());

	// Initialize bundle_copy
	// There are cbba_bundle.size()+1 possible position where task can be inserted into
	std::vector<std::vector<int>> path_copy;

	//std::cout << "The size of bundle_copy is" << bundle_copy.size() <<std::endl;

	// Find all possibilities each task in task_NotInBundle can insert into
	std::vector<int>::iterator it;
	std::vector<float> c_award;
	for (int j = 0; j < tasks_NotInBundle.size();j++){

		// Clear c_award and bundle_copy for each task in the tasks_NotInBundle
		// c_award and bundle_copy are only for one task with different insert position
		c_award.clear();
		
		// If the size of cbba_bundle is n, then there are n+1 positions to insert each task in the tasks_NotInBundle
		std::vector<std::vector<int>> path_copy(cbba_path_.size()+1, cbba_path_);

		// Insert one task (task_NotInBundle[j]) into all the possible position
		// Each item in the path_copy represents a possible position to insert the task
		// std::cout << "Task " << j << "+++++++++++++++" << std::endl;
		// std::cout << "Current path is " << std::endl;
		// for (auto &item: cbba_path_){
		// 	std::cout << item << " " ;
		// }
		// std::cout << std::endl;

		for (int m = 0; m < path_copy.size(); m++){
			it = path_copy[m].begin();
			path_copy[m].insert(it+m,tasks_NotInBundle[j]);
		}

		// Calculate the Astar length for each possible insert position
		for (int i = 0; i < path_copy.size(); i++)
			c_award.push_back(CBBA::PathLengthCalculation(Global_LTL,graph,path_copy[i],start_node_));
			
		float c_award_min= 200.0;
		for (int i = 0; i < c_award.size(); i++)
			if (c_award[i] < c_award_min){
				c_award_min = c_award[i];
				best_position[tasks_NotInBundle[j]] = i;
			}

		// Update the award
		cbba_award_[tasks_NotInBundle[j]] = cbba_benefit - c_award_min;

	}

	// std::cout << "============================= DEBUG FOR AWARD UPDATES ================================= " << std::endl;
	// std::cout << "For vehicle " << idx_ << std::endl;
	// std::cout << "The Awards are !!!!!!!!!!!!!!!!!!!!" << std::endl;
	// for (int j = 0; j < M; j++)
	// 	std::cout << cbba_award[j] << ' ';
	// std::cout << '\n';

	// std::cout << "Best Position for each task should be:" <<std::endl;
	// for (int j = 0; j < best_position.size(); j++)
	// 	std::cout << best_position[j] << ' ';
	// std::cout << std::endl;

	return best_position;
};


std::vector<int> cbba_Agent::award_update_CBTA(LTLFormula Global_LTL,const std::shared_ptr<Graph_t<SquareCell*>> graph, std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph,  
			TileTraversalData tile_traversal_data, std::shared_ptr<SquareGrid> grid){

	std::vector<int> best_position(num_tasks_, -1);
	// Initialize the tasks_NotInBundle
	std::vector<int> tasks_NotInBundle = {};
	// Put all the task index into the tasks_NotInBundle
	for(int j = 0; j < num_tasks_; j++)
		tasks_NotInBundle.push_back(j);

	// Find all tasks which have not been inserted into the bundle
	for (int j = 0; j < cbba_bundle_.size(); j++)
		// Erase all tasks which have been added into the bundle
		tasks_NotInBundle.erase(std::remove(tasks_NotInBundle.begin(),tasks_NotInBundle.end(),cbba_bundle_[j]),tasks_NotInBundle.end());


	// Initialize bundle_copy
	// There are cbba_bundle.size()+1 possible position where task can be inserted into
	std::vector<std::vector<int>> path_copy;

	//std::cout << "The size of bundle_copy is" << bundle_copy.size() <<std::endl;

	// Find all possibilities each task in task_NotInBundle can insert into
	std::vector<int>::iterator it;
	std::vector<float> c_award;
	for (int j = 0; j < tasks_NotInBundle.size();j++){

		// Clear c_award and bundle_copy for each task in the tasks_NotInBundle
		// c_award and bundle_copy are only for one task with different insert position
		c_award.clear();
	
		// If the size of cbba_bundle is n, then there are n+1 positions to insert each task in the tasks_NotInBundle
		std::vector<std::vector<int>> path_copy(cbba_path_.size()+1, cbba_path_);

		// Insert one task (task_NotInBundle[j]) into all the possible position
		// Each item in the path_copy represents a possible position to insert the task
		std::cout << "Task " << j << "+++++++++++++++" << std::endl;
		std::cout << "Current path is " << std::endl;
		for (auto &item: cbba_path_){
			std::cout << item << " " ;
		}
		std::cout << std::endl;

		for (int m = 0; m < path_copy.size(); m++){
			it = path_copy[m].begin();
			path_copy[m].insert(it+m,tasks_NotInBundle[j]);
			// path_copy[m].erase(it+m+1, path_copy[m].end());

			// for (auto &item: path_copy[m]){
			// 	std::cout << item << " " ;
			// }
			// std::cout << std::endl;
		}

		

		// Calculate the Astar length for each possible insert position
		for (int i = 0; i < path_copy.size(); i++)
			c_award.push_back(CBBA::PathLengthCalculationCBTA(Global_LTL, graph, path_copy[i], start_node_, lifted_graph, tile_traversal_data, grid));

		float c_award_min = 1000.0;
		for (int i = 0; i < c_award.size(); i++)
			if (c_award[i] <= c_award_min){
				c_award_min = c_award[i];
				best_position[tasks_NotInBundle[j]] = i;
			}

		// Update the award
		cbba_award_[tasks_NotInBundle[j]] = cbba_benefit-c_award_min;
	}

	// std::cout << "============================= DEBUG FOR AWARD UPDATES ================================= " << std::endl;
	// std::cout << "For vehicle " << idx_ << std::endl;
	// std::cout << "The Awards are !!!!!!!!!!!!!!!!!!!!" << std::endl;
	// for (int j = 0; j < M; j++)
	// 	std::cout << cbba_award[j] << ' ';
	// std::cout << '\n';

	// std::cout << "Best Position for each task should be:" <<std::endl;
	// for (int j = 0; j < best_position.size(); j++)
	// 	std::cout << best_position[j] << ' ';
	// std::cout << std::endl;

	return best_position;
};

// Update the assignment from bundle information
void cbba_Agent::assignment_update(){
	if (!cbba_bundle_.empty())
		for (int j = 0; j < cbba_bundle_.size();j++)
			cbba_x_[cbba_bundle_[j]] = 1;
};


double CBBA::CalcHeuristic(SquareCell *node1, SquareCell *node2)
{
    int32_t dist_row = node1->coordinate_.x - node2->coordinate_.x;
    int32_t dist_col = node1->coordinate_.y - node2->coordinate_.y;

    return std::sqrt(dist_row * dist_row + dist_col * dist_col);
}

float CBBA::PathLengthCalculation(LTLFormula Global_LTL,std::shared_ptr<Graph_t<SquareCell*>> graph,std::vector<int> bundle_copy_onecase,int start_idx_){
	Path_t<SquareCell *> path_part;
	Path_t<SquareCell *> path;
	Vertex_t<SquareCell*> * start_node;
	Vertex_t<SquareCell*> * finish_node;


	// Initialize the start_node which should be initial position of agent
	start_node = graph->GetVertexFromID(start_idx_);
	for (int i = 0; i < bundle_copy_onecase.size(); i++){
		for(int j = 0; j < Global_LTL.sub_buchi_regions.sub_buchi_regions_Liveness_T[bundle_copy_onecase[i]].size();j++){
			for (int m = 0; m < Global_LTL.task_info.size();m++){
				if(Global_LTL.sub_buchi_regions.sub_buchi_regions_Liveness_T[bundle_copy_onecase[i]][j] == Global_LTL.task_info[m].P_idx_){
					finish_node = graph->GetVertexFromID(Global_LTL.task_info[m].pos_);
					path_part = AStar::Search(graph,start_node->state_->id_,finish_node->state_->id_, CalcHeuristicFunc_t<SquareCell *>(CBBA::CalcHeuristic));
					if (i == bundle_copy_onecase.size()-1 && j == Global_LTL.sub_buchi_regions.sub_buchi_regions_Liveness_T[bundle_copy_onecase[i]].size()-1){
						path.insert(path.end(), path_part.begin(), path_part.end());
					}
					else{
						path.insert(path.end(), path_part.begin(), path_part.end()-1);
					}
					start_node = finish_node;

					break;
				}
			}
		}
	}

	float path_length = path.size();
	return path_length;
};


float CBBA::PathLengthCalculationCBTA(LTLFormula Global_LTL, std::shared_ptr<Graph_t<SquareCell*>> graph,
	std::vector<int> bundle_copy_onecase,int start_idx_,std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph, 
	TileTraversalData tile_traversal_data, std::shared_ptr<SquareGrid> grid){
		std::cout << "================================ DEBUG FOR CBTA ====================================" << std::endl;
		std::shared_ptr<Graph_t<ProductState *>> product_graph_new = std::make_shared<Graph_t<ProductState *>>();

		std::string ltl_formula = LTLDecomposition::subtask_recreator(bundle_copy_onecase,true, Global_LTL);
		std::vector<std::vector<std::string>> buchi_regions = LTLDecomposition::ObtainBuchiRegion({ltl_formula});

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
			//path_CBTA_length = float(path.size());
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

		std::cout << "start idx: " << start_idx_ << std::endl;
		std::cout << "The specification is " << ltl_formula << std::endl;
		// std::cout << "Act state is: " << start_idx_ << ". Virtual state is: " << virtual_start_id << std::endl;
		std::cout << "The FINAL length of path is: ！! " << path_CBTA_length << std::endl;
		std::cout << "================================ DEBUG ====================================" << std::endl;
		return path_CBTA_length;
}



// Find the neighbors of agent and update neighbors
void cbba_Agent::neighbor_finder()
{
	neighbors_.clear();
	for (int i = 0; i < comm_topo_.size(); i++)
		if (comm_topo_[i] == 1 && i != idx_)
				neighbors_.push_back(i);
		else
			continue;
};

// Initialize Agents: Read agents from config file 
std::vector<cbba_Agent> CBBA::InitializeAgents(){

	ConfigReader config_reader("../../src/config/agent.ini");
    if (config_reader.CheckError()){
        std::cout << "Reading config file failed." << std::endl;
    }

    int32_t num_agents = config_reader.GetReal("num_agents", 0);
	std::vector<cbba_Agent> agents;

	for (int i= 0; i < num_agents; i++){
		std::string agent_name = "agent"+ std::to_string(i);
		std::string agent_idx = agent_name + "_idx";
		std::string agent_init_pos = agent_name + "_init";
		std::string agent_neighbors = agent_name + "_neighbors";

		// Read config from agent.ini
		int idx = i;
		int init_pos = config_reader.GetReal(agent_init_pos, 0);
		std::vector<int> comm = config_reader.GetVectorInt(agent_neighbors,{});
		int num_tasks_inde = config_reader.GetReal("num_tasks_inde", 0);
		int num_tasks_de = config_reader.GetReal("num_tasks_de", 0);


		// Initialize the agent
		agents.push_back(cbba_Agent(idx, init_pos, comm, num_tasks_inde, num_tasks_de));
	}

	return agents;
    
}

// Let agent communicate with its neighbors
void CBBA::communicate(std::vector<cbba_Agent>& agents)
{
	//CBBA::neighbor_finder(agent);
	// sender: itself k
	// receiver: i
	// task : j

	for (auto it_ag = agents.begin(); it_ag != agents.end(); it_ag++){
		(*it_ag).neighbor_finder();
		for (int i = 0; i < (*it_ag).neighbors_.size();i++){
			std::cout << "Task independent is " << (*it_ag).num_tasks_inde_ << std::endl;
			for (int j = 0; j < (*it_ag).num_tasks_inde_; j++){
				// Entries 1 to 4
				// if current agent k thinks that the winner of task j is itself k
				if ((*it_ag).history_.z_history.back()[j] == (*it_ag).idx_){

					/***************************************** Entry 1 ***************************************/
					// Entry 1: Update or leave
					// If the receiver (neighbor) i thinks the winner of task j is also itself i
					if (agents[(*it_ag).neighbors_[i]].cbba_z_[j] == (*it_ag).neighbors_[i]){
						// Update
						if ((*it_ag).history_.y_history.back()[j] - agents[(*it_ag).neighbors_[i]].cbba_y_[j] > eps){
							//std::cout << "case 1" << std::endl;
							agents[(*it_ag).neighbors_[i]].cbba_z_[j] = (*it_ag).history_.z_history.back()[j];
							agents[(*it_ag).neighbors_[i]].cbba_y_[j] = (*it_ag).history_.y_history.back()[j];
						}
					    // Equal score: require to break the tie
						else if(abs((*it_ag).history_.y_history.back()[j] - agents[(*it_ag).neighbors_[i]].cbba_y_[j]) <= eps){
							// select the winner of task j as the agent with smaller index
							if (agents[(*it_ag).neighbors_[i]].cbba_z_[j] > (*it_ag).history_.z_history.back()[j]){
								//std::cout << "case 2" << std::endl;
								agents[(*it_ag).neighbors_[i]].cbba_z_[j] = (*it_ag).history_.z_history.back()[j];
								agents[(*it_ag).neighbors_[i]].cbba_y_[j] = (*it_ag).history_.y_history.back()[j];
							}
						}
					}

					/***************************************** Entry 2 ***************************************/
					// Entry 2: Update
					// If the receiver i thinks the winner of task j is also agent k (sender)
					// Update
					else if(agents[(*it_ag).neighbors_[i]].cbba_z_[j] == (*it_ag).idx_){
						//std::cout << "case 3" << std::endl;
						agents[(*it_ag).neighbors_[i]].cbba_z_[j] = (*it_ag).history_.z_history.back()[j];
						agents[(*it_ag).neighbors_[i]].cbba_y_[j] = (*it_ag).history_.y_history.back()[j];
					}

					/***************************************** Entry 3 ***************************************/

					// Entry 3: Update or Leave
					// If the receiver i thinks the winner of task j is not k or itself i but other agent
					else if (agents[(*it_ag).neighbors_[i]].cbba_z_[j] >= 0){
						// Compare the iteration of task j, find which one is the least information of task j
						// Update
						if ((*it_ag).history_.iter_neighbors_his.back()[agents[(*it_ag).neighbors_[i]].cbba_z_[j]] > agents[(*it_ag).neighbors_[i]].iteration_neighbors_[agents[(*it_ag).neighbors_[i]].cbba_z_[j]]){
							//std::cout << "case 4" << std::endl;
							agents[(*it_ag).neighbors_[i]].cbba_z_[j] = (*it_ag).history_.z_history.back()[j];
							agents[(*it_ag).neighbors_[i]].cbba_y_[j] = (*it_ag).history_.y_history.back()[j];
						}
						// Update
						else if((*it_ag).history_.y_history.back()[j] - agents[(*it_ag).neighbors_[i]].cbba_y_[j] > eps){
							//std::cout << "case 5" << std::endl;
							agents[(*it_ag).neighbors_[i]].cbba_z_[j] = (*it_ag).history_.z_history.back()[j];
							agents[(*it_ag).neighbors_[i]].cbba_y_[j] = (*it_ag).history_.y_history.back()[j];
						}
						// Equal scores: break the tie by selecting the winner as the agent with smaller index
						else if (abs((*it_ag).history_.y_history.back()[j] - agents[(*it_ag).neighbors_[i]].cbba_y_[j]) <= eps){
							if (agents[(*it_ag).neighbors_[i]].cbba_z_[j] > (*it_ag).history_.z_history.back()[j]){
								//std::cout << "case 6" << std::endl;
								agents[(*it_ag).neighbors_[i]].cbba_z_[j] = (*it_ag).history_.z_history.back()[j];
								agents[(*it_ag).neighbors_[i]].cbba_y_[j] = (*it_ag).history_.y_history.back()[j];
							}
						}
					}

					/***************************************** Entry 4 ***************************************/
					// Entry 4: Update
					// If the agent i (receiver) has no idea about the winner
					else if(agents[(*it_ag).neighbors_[i]].cbba_z_[j] == -1){
						//std::cout << "case 7" << std::endl;
						agents[(*it_ag).neighbors_[i]].cbba_z_[j] = (*it_ag).history_.z_history.back()[j];
						agents[(*it_ag).neighbors_[i]].cbba_y_[j] = (*it_ag).history_.y_history.back()[j];
					}

					else
						std::cout << "Unknown winner value 1" << std::endl;
				}



		        /*********************************************************************************************************/
				/*********************************************************************************************************/
				/*********************************************************************************************************/

				// Entries 5 to 8
				// If current agent i (sender) thinks the winner of task j is agent k (receiver)
				else if ((*it_ag).history_.z_history.back()[j] == agents[(*it_ag).neighbors_[i]].idx_){

					/***************************************** Entry 5 ***************************************/
					// Entry 5
				    // if agent i (receiver) also agree with agent k (sender): agent i thinks the winner of task j is also itself i
				    // Leave
				    if (agents[(*it_ag).neighbors_[i]].cbba_z_[j] == agents[(*it_ag).neighbors_[i]].idx_)
				    	std::cout << "Do nothing Entry 5" << std::endl;

				    /***************************************** Entry 6 ***************************************/
				    // Entry 6
					// If agent i (receiver) thinks the winner of task j is agent k (sender)
					// Reset (Because the agent k will definitely be the first one to know its own information )
					else if (agents[(*it_ag).neighbors_[i]].cbba_z_[j] == (*it_ag).idx_){
						//std::cout << "case 7" << std::endl;
						agents[(*it_ag).neighbors_[i]].cbba_z_[j] = -1;
						agents[(*it_ag).neighbors_[i]].cbba_y_[j] = -1;
					}

				    /***************************************** Entry 7 ***************************************/
				    // Entry 7
					// If agent i thinks the winner of task j is not itself (agent k thinks), but other agent
					else if(agents[(*it_ag).neighbors_[i]].cbba_z_[j] >= 0){
						// Compare the iteration of agent k and i, find which one has the least information
						// Reset
						if ((*it_ag).history_.iter_neighbors_his.back()[agents[(*it_ag).neighbors_[i]].cbba_z_[j]] > agents[(*it_ag).neighbors_[i]].iteration_neighbors_[agents[(*it_ag).neighbors_[i]].cbba_z_[j]]){
							// agent k should have the updated information
							//std::cout << "case 8" << std::endl;
							agents[(*it_ag).neighbors_[i]].cbba_z_[j] = -1;
							agents[(*it_ag).neighbors_[i]].cbba_y_[j] = -1;
						}
					}

				    /***************************************** Entry 8 ***************************************/
				    // Entry 8
					else if(agents[(*it_ag).neighbors_[i]].cbba_z_[j] == -1)
						std::cout <<"Do nothing Entry 8" << std::endl;

					else
						std::cout << "Unknown winner value 2" << std::endl;

				}

				/*********************************************************************************************************/
				/*********************************************************************************************************/
				/*********************************************************************************************************/
				// Entries 9 to 13
				// If agent k (sender) thinks the winner of task j is not itself k and not receiver i,but other agent
				else if ((*it_ag).history_.z_history.back()[j] >= 0){
					/***************************************** Entry 9 ***************************************/
					// Entry 9
					// if agent i (receiver) thinks the winner of task j should be itself i
					if (agents[(*it_ag).neighbors_[i]].cbba_z_[j] == agents[(*it_ag).neighbors_[i]].idx_){
						// compare the iteration that agent k and i talk to the winner that agent k thinks
						// If agent k (sender) has the least information
						if ((*it_ag).history_.iter_neighbors_his.back()[(*it_ag).history_.z_history.back()[j]] > agents[(*it_ag).neighbors_[i]].iteration_neighbors_[(*it_ag).history_.z_history.back()[j]]){
							// Update
							if ((*it_ag).history_.y_history.back()[j] - agents[(*it_ag).neighbors_[i]].cbba_y_[j] > eps){
								//std::cout << "case 8" << std::endl;
								agents[(*it_ag).neighbors_[i]].cbba_z_[j] = (*it_ag).history_.z_history.back()[j];
								agents[(*it_ag).neighbors_[i]].cbba_y_[j] = (*it_ag).history_.y_history.back()[j];
							}
							
							// If we have a tie: break the tie by selecting the agent with smaller index as the winner
							else if (abs((*it_ag).history_.y_history.back()[j] - agents[(*it_ag).neighbors_[i]].cbba_y_[j]) <= eps){
								if (agents[(*it_ag).neighbors_[i]].cbba_z_[j] > (*it_ag).history_.z_history.back()[j]){
									// Update
									//std::cout << "case 9" << std::endl;
									agents[(*it_ag).neighbors_[i]].cbba_z_[j] = (*it_ag).history_.z_history.back()[j];
									agents[(*it_ag).neighbors_[i]].cbba_y_[j] = (*it_ag).history_.y_history.back()[j];
								}
							}
						}
					}

					/***************************************** Entry 10 ***************************************/
					// Entry 10
					// if agent i (receiver) thinks the winner of task j is agent k (sender)
					else if (agents[(*it_ag).neighbors_[i]].cbba_z_[j] == (*it_ag).idx_){
						// Compare the iteration of agent k and i, which one has the least information about the winner that agent k thinks
						if ((*it_ag).history_.iter_neighbors_his.back()[(*it_ag).history_.z_history.back()[j]] > agents[(*it_ag).neighbors_[i]].iteration_neighbors_[(*it_ag).history_.z_history.back()[j]]){
							//std::cout << "case 10" << std::endl;
							agents[(*it_ag).neighbors_[i]].cbba_z_[j] = (*it_ag).history_.z_history.back()[j];
							agents[(*it_ag).neighbors_[i]].cbba_y_[j] = (*it_ag).history_.y_history.back()[j];
						}
						else{
							// Reset
							//std::cout << "case 11" << std::endl;
							agents[(*it_ag).neighbors_[i]].cbba_z_[j] = -1;
							agents[(*it_ag).neighbors_[i]].cbba_y_[j] = -1;
						}
					}

					/***************************************** Entry 11 ***************************************/
					// Entry 11
					// If agent i (receiver) agree with agent k and thinks the winner of task j is not i k, but other agent history_z(j)
					else if(agents[(*it_ag).neighbors_[i]].cbba_z_[j] == (*it_ag).history_.z_history.back()[j]){
						// Update
						if ((*it_ag).history_.iter_neighbors_his.back()[(*it_ag).history_.z_history.back()[j]] > agents[(*it_ag).neighbors_[i]].iteration_neighbors_[(*it_ag).history_.z_history.back()[j]]){
							//std::cout << "case 12" << std::endl;
							agents[(*it_ag).neighbors_[i]].cbba_z_[j] = (*it_ag).history_.z_history.back()[j];
							agents[(*it_ag).neighbors_[i]].cbba_y_[j] = (*it_ag).history_.y_history.back()[j];
						}
					}

					/***************************************** Entry 12 ***************************************/
					// Entry 12
					// If agent i (receiver) thinks the winner of task j is not itself, agent k, the one that agent k thinks
					else if (agents[(*it_ag).neighbors_[i]].cbba_z_[j] >= 0){
						if ((*it_ag).history_.iter_neighbors_his.back()[agents[(*it_ag).neighbors_[i]].cbba_z_[j]] > agents[(*it_ag).neighbors_[i]].iteration_neighbors_[agents[(*it_ag).neighbors_[i]].cbba_z_[j]]){
							// Update
							if ((*it_ag).history_.iter_neighbors_his.back()[(*it_ag).history_.z_history.back()[j]] >= agents[(*it_ag).neighbors_[i]].iteration_neighbors_[(*it_ag).history_.z_history.back()[j]]){
								//std::cout << "case 13" << std::endl;
								agents[(*it_ag).neighbors_[i]].cbba_z_[j] = (*it_ag).history_.z_history.back()[j];
								agents[(*it_ag).neighbors_[i]].cbba_y_[j] = (*it_ag).history_.y_history.back()[j];
							}
							// Reset
							else if ((*it_ag).history_.iter_neighbors_his.back()[(*it_ag).history_.z_history.back()[j]] < agents[(*it_ag).neighbors_[i]].iteration_neighbors_[(*it_ag).history_.z_history.back()[j]]){
								//std::cout << "case 14" << std::endl;
								agents[(*it_ag).neighbors_[i]].cbba_z_[j] = -1;
								agents[(*it_ag).neighbors_[i]].cbba_y_[j] = -1;
							}
							else
								std::cout << "Should not be here Entry 12" << std::endl;
						}
						else{
							if ((*it_ag).history_.iter_neighbors_his.back()[(*it_ag).history_.z_history.back()[j]] > agents[(*it_ag).neighbors_[i]].iteration_neighbors_[(*it_ag).history_.z_history.back()[j]]){
								// Update
								if ((*it_ag).history_.y_history.back()[j] - agents[(*it_ag).neighbors_[i]].cbba_y_[j] > eps){
									//std::cout << "case 15" << std::endl;
									agents[(*it_ag).neighbors_[i]].cbba_z_[j] = (*it_ag).history_.z_history.back()[j];
									agents[(*it_ag).neighbors_[i]].cbba_y_[j] = (*it_ag).history_.y_history.back()[j];
								}
								// If we have tie, break the tie by selecting the agent as the winner with smaller index
								else if (abs((*it_ag).history_.y_history.back()[j] - agents[(*it_ag).neighbors_[i]].cbba_y_[j]) <= eps){
									if (agents[(*it_ag).neighbors_[i]].cbba_z_[j] > (*it_ag).history_.z_history.back()[j]){
										//std::cout << "case 16" << std::endl;
										agents[(*it_ag).neighbors_[i]].cbba_z_[j] = (*it_ag).history_.z_history.back()[j];
										agents[(*it_ag).neighbors_[i]].cbba_y_[j] = (*it_ag).history_.y_history.back()[j];
									}
								}
							}
						}
					}
					/***************************************** Entry 13 ***************************************/
					// Entry 13
					else if(agents[(*it_ag).neighbors_[i]].cbba_z_[j] == -1){
						// Update
						if ((*it_ag).history_.iter_neighbors_his.back()[(*it_ag).history_.z_history.back()[j]] > agents[(*it_ag).neighbors_[i]].iteration_neighbors_[(*it_ag).history_.z_history.back()[j]]){
							//std::cout << "case 17" << std::endl;
							agents[(*it_ag).neighbors_[i]].cbba_z_[j] = (*it_ag).history_.z_history.back()[j];
							agents[(*it_ag).neighbors_[i]].cbba_y_[j] = (*it_ag).history_.y_history.back()[j];
						}
					}
					else
						std::cout << "Unknown winner value Entry 13" << std::endl;

				}

				/*********************************************************************************************************/
				/*********************************************************************************************************/
				/*********************************************************************************************************/
				// Entries 14 to 17
				else if ((*it_ag).history_.z_history.back()[j] == -1){

					/***************************************** Entry 14 ***************************************/
					// Entry 14
					// Leave
					if (agents[(*it_ag).neighbors_[i]].cbba_z_[j] == agents[(*it_ag).neighbors_[i]].idx_)
						std::cout << "Do nothing Entry 14" << std::endl;

					/***************************************** Entry 15 ***************************************/
					// Entry 15
					// Update
					else if (agents[(*it_ag).neighbors_[i]].cbba_z_[j] == (*it_ag).idx_){
						//std::cout << "case 18" << std::endl;
						agents[(*it_ag).neighbors_[i]].cbba_z_[j] = (*it_ag).history_.z_history.back()[j];
						agents[(*it_ag).neighbors_[i]].cbba_y_[j] = (*it_ag).history_.y_history.back()[j];
					}

					/***************************************** Entry 16 ***************************************/
					// Entry 16
					// Update
					else if (agents[(*it_ag).neighbors_[i]].cbba_z_[j] >= 0){
						// Update
						if ((*it_ag).history_.iter_neighbors_his.back()[agents[(*it_ag).neighbors_[i]].cbba_z_[j]] > agents[(*it_ag).neighbors_[i]].iteration_neighbors_[agents[(*it_ag).neighbors_[i]].cbba_z_[j]]){
							//std::cout << "case 19" << std::endl;
							agents[(*it_ag).neighbors_[i]].cbba_z_[j] = (*it_ag).history_.z_history.back()[j];
							agents[(*it_ag).neighbors_[i]].cbba_y_[j] = (*it_ag).history_.y_history.back()[j];
						}
					}

					/***************************************** Entry 17 ***************************************/
					// Entry 17
					// Leave
					else if(agents[(*it_ag).neighbors_[i]].cbba_z_[j] == -1)
						std::cout<< "Do noting Entry 17" << std::endl;

					else
						std::cout << "Unknown winner value Entry 17" <<std::endl;
				}

				else
					std::cout << "Unknown winner value end of communicate" <<std::endl;
			}

			for (int n = 0; n < agents[0].num_agents_; n++){
				if (n != (*it_ag).neighbors_[i] && agents[(*it_ag).neighbors_[i]].iteration_neighbors_[n] < (*it_ag).history_.iter_neighbors_his.back()[n]){
					agents[(*it_ag).neighbors_[i]].iteration_neighbors_[n] = (*it_ag).history_.iter_neighbors_his.back()[n];
				}
			}

			agents[(*it_ag).neighbors_[i]].iteration_neighbors_[(*it_ag).idx_] = (*it_ag).iter_;
		}

	}
};


void CBBA::communicate_lcm(cbba_Agent& agent, int neig_idx, std::vector<float> neig_y_his, std::vector<int> neig_z_his, std::vector<int> neig_iter_nei_his){
	for (int j = 0; j < agent.num_tasks_inde_; j++){
		// std::cout << "task " << j << "(*)(*)(*)(*)(*)(*)(*) " << std::endl;
		// std::cout << "neig_z_his[j] " << neig_z_his[j] << std::endl;
		// std::cout << "agent.cbba_z_[j] " << agent.cbba_z_[j] << std::endl;
		// std::cout << "neig_iter_nei_his ";
		// for (auto &ee: neig_iter_nei_his){
		// 	std::cout << ee << ", ";
		// }
		// std::cout << std::endl;
		// std::cout << "agent hist iteration is ";
		// for (auto &ed: agent.iteration_neighbors_){
		// 	std::cout << ed << ", ";
		// }
		// std::cout << std::endl;
		// Entries 1 to 4
		// if current agent k thinks that the winner of task j is itself k
		if (neig_z_his[j] == neig_idx){

			/***************************************** Entry 1 ***************************************/
			// Entry 1: Update or leave
			// If the receiver (neighbor) i thinks the winner of task j is also itself i
			if (agent.cbba_z_[j] == agent.idx_){
				// Update
				if (neig_y_his[j] - agent.cbba_y_[j] > eps){
					// std::cout << "case 1" << std::endl;
					agent.cbba_z_[j] = neig_z_his[j];
					agent.cbba_y_[j] = neig_y_his[j];
				}
				// Equal score: require to break the tie
				else if(abs(neig_y_his[j] - agent.cbba_y_[j]) <= eps){
					// select the winner of task j as the agent with smaller index
					if (agent.cbba_z_[j] > neig_z_his[j]){
						// std::cout << "case 2" << std::endl;
						agent.cbba_z_[j] = neig_z_his[j];
						agent.cbba_y_[j] = neig_y_his[j];
					}
				}
			}


			/***************************************** Entry 2 ***************************************/
			// Entry 2: Update
			// If the receiver i thinks the winner of task j is also agent k (sender)
			// Update
			else if(agent.cbba_z_[j] == neig_idx){
				// std::cout << "case 3" << std::endl;
				agent.cbba_z_[j] = neig_z_his[j];
				agent.cbba_y_[j] = neig_y_his[j];
			}

			/***************************************** Entry 3 ***************************************/
			// Entry 3: Update or Leave
			// If the receiver i thinks the winner of task j is not k or itself i but other agent
			else if (agent.cbba_z_[j] >= 0){
				// Compare the iteration of task j, find which one is the least information of task j
				// Update
				if (neig_iter_nei_his[agent.cbba_z_[j]] > agent.iteration_neighbors_[agent.cbba_z_[j]]){
					// std::cout << "case 4" << std::endl;
					agent.cbba_z_[j] = neig_z_his[j];
					agent.cbba_y_[j] = neig_y_his[j];
				}
				// Update
				else if(neig_y_his[j] - agent.cbba_y_[j] > eps){
					// std::cout << "case 5" << std::endl;
					agent.cbba_z_[j] = neig_z_his[j];
					agent.cbba_y_[j] = neig_y_his[j];
				}
				// Equal scores: break the tie by selecting the winner as the agent with smaller index
				else if (abs(neig_y_his[j] - agent.cbba_y_[j]) <= eps){
					if (agent.cbba_z_[j] > neig_z_his[j]){
						// std::cout << "case 6" << std::endl;
						agent.cbba_z_[j] = neig_z_his[j];
						agent.cbba_y_[j] = neig_y_his[j];
					}
				}
			}

			/***************************************** Entry 4 ***************************************/
			// Entry 4: Update
			// If the agent i (receiver) has no idea about the winner
			else if(agent.cbba_z_[j] == -1){
				// std::cout << "case 7" << std::endl;
				agent.cbba_z_[j] = neig_z_his[j];
				agent.cbba_y_[j] = neig_y_his[j];
			}

			else
				std::cout << "Unknown winner value 1" << std::endl;
		}



		/*********************************************************************************************************/
		/*********************************************************************************************************/
		/*********************************************************************************************************/
		// Entries 5 to 8
		// If current agent i (sender) thinks the winner of task j is agent k (receiver)
		else if (neig_z_his[j] == agent.idx_){

			/***************************************** Entry 5 ***************************************/
			// Entry 5
			// if agent i (receiver) also agree with agent k (sender): agent i thinks the winner of task j is also itself i
			// Leave
			if (agent.cbba_z_[j] == agent.idx_){
				continue;
				// std::cout << "Do nothing Entry 5" << std::endl;
			}
				

			/***************************************** Entry 6 ***************************************/
			// Entry 6
			// If agent i (receiver) thinks the winner of task j is agent k (sender)
			// Reset (Because the agent k will definitely be the first one to know its own information )
			else if (agent.cbba_z_[j] == neig_idx){
				// std::cout << "case 7" << std::endl;
				agent.cbba_z_[j] = -1;
				agent.cbba_y_[j] = -1;
			}

			/***************************************** Entry 7 ***************************************/
			// Entry 7
			// If agent i thinks the winner of task j is not itself (agent k thinks), but other agent
			else if(agent.cbba_z_[j] >= 0){
				// Compare the iteration of agent k and i, find which one has the least information
				// Reset
				if (neig_iter_nei_his[agent.cbba_z_[j]] > agent.iteration_neighbors_[agent.cbba_z_[j]]){
						// agent k should have the updated information
						// std::cout << "case 8" << std::endl;
						agent.cbba_z_[j] = -1;
						agent.cbba_y_[j] = -1;
				}
			}

			/***************************************** Entry 8 ***************************************/
			// Entry 8
			else if(agent.cbba_z_[j] == -1){
				continue;
				// std::cout <<"Do nothing Entry 8" << std::endl;
			}
			else{
				continue;
				// std::cout << "Unknown winner value 2" << std::endl;
			}
				

		}
			
		/*********************************************************************************************************/
		/*********************************************************************************************************/
		/*********************************************************************************************************/
		// Entries 9 to 13
		// If agent k (sender) thinks the winner of task j is not itself k and not receiver i,but other agent
		else if (neig_z_his[j] >= 0){
			/***************************************** Entry 9 ***************************************/
			// Entry 9
			// if agent i (receiver) thinks the winner of task j should be itself i
			if (agent.cbba_z_[j] == agent.idx_){
				// compare the iteration that agent k and i talk to the winner that agent k thinks
				// If agent k (sender) has the least information
				if (neig_iter_nei_his[neig_z_his[j]] > agent.iteration_neighbors_[neig_z_his[j]]){
					// Update
					if (neig_y_his[j] - agent.cbba_y_[j] > eps){
						// std::cout << "case 8" << std::endl;
						agent.cbba_z_[j] = neig_z_his[j];
						agent.cbba_y_[j] = neig_y_his[j];
					}
					// If we have a tie: break the tie by selecting the agent with smaller index as the winner
					else if (abs(neig_y_his[j] - agent.cbba_y_[j]) <= eps){
						if (agent.cbba_z_[j] > neig_z_his[j]){
							// Update
							// std::cout << "case 9" << std::endl;
							agent.cbba_z_[j] = neig_z_his[j];
							agent.cbba_y_[j] = neig_y_his[j];
						}
					}
				}
			}

			/***************************************** Entry 10 ***************************************/
			// Entry 10
			// if agent i (receiver) thinks the winner of task j is agent k (sender)
			else if (agent.cbba_z_[j] == neig_idx){
				// Compare the iteration of agent k and i, which one has the least information about the winner that agent k thinks
				if (neig_iter_nei_his[neig_z_his[j]] > agent.iteration_neighbors_[neig_z_his[j]]){
					// std::cout << "case 10" << std::endl;
					agent.cbba_z_[j] = neig_z_his[j];
					agent.cbba_y_[j] = neig_y_his[j];
				}
				else{
					// Reset
					// std::cout << "case 11" << std::endl;
					agent.cbba_z_[j] = -1;
					agent.cbba_y_[j] = -1;
				}
			}


			/***************************************** Entry 11 ***************************************/
			// Entry 11
			// If agent i (receiver) agree with agent k and thinks the winner of task j is not i k, but other agent history_z(j)
			else if(agent.cbba_z_[j] ==neig_z_his[j]){
				// Update
				if (neig_iter_nei_his[neig_z_his[j]] > agent.iteration_neighbors_[neig_z_his[j]]){
					// std::cout << "case 12" << std::endl;
					agent.cbba_z_[j] = neig_z_his[j];
					agent.cbba_y_[j] = neig_y_his[j];
				}
			}




			/***************************************** Entry 12 ***************************************/
			// Entry 12
			// If agent i (receiver) thinks the winner of task j is not itself, agent k, the one that agent k thinks
			else if (agent.cbba_z_[j] >= 0){
				if (neig_iter_nei_his[agent.cbba_z_[j]] > agent.iteration_neighbors_[agent.cbba_z_[j]]){
					// Update
					if (neig_iter_nei_his[neig_z_his[j]] > agent.iteration_neighbors_[neig_z_his[j]]){
						// std::cout << "case 13" << std::endl;
						agent.cbba_z_[j] = neig_z_his[j];
						agent.cbba_y_[j] = neig_y_his[j];
					}
					// Reset
					else if (neig_iter_nei_his[neig_z_his[j]] < agent.iteration_neighbors_[neig_z_his[j]]){
						// std::cout << "case 14" << std::endl;
						agent.cbba_z_[j] = -1;
						agent.cbba_y_[j] = -1;
					}
					else{
						continue;
						// std::cout << "Should not be here Entry 12" << std::endl;
					}
						
				}
				else{
					if (neig_iter_nei_his[neig_z_his[j]] > agent.iteration_neighbors_[neig_z_his[j]]){
						// Update
						if (neig_y_his[j] - agent.cbba_y_[j] > eps){
							// std::cout << "case 15" << std::endl;
							agent.cbba_z_[j] = neig_z_his[j];
							agent.cbba_y_[j] = neig_y_his[j];
						}
						// If we have tie, break the tie by selecting the agent as the winner with smaller index
						else if (abs(neig_y_his[j] - agent.cbba_y_[j]) <= eps){
							if (agent.cbba_z_[j] > neig_z_his[j]){
								// std::cout << "case 16" << std::endl;
								agent.cbba_z_[j] = neig_z_his[j];
								agent.cbba_y_[j] = neig_y_his[j];
							}
						}
					}
				}
			}

			/***************************************** Entry 13 ***************************************/
			// Entry 13
			else if(agent.cbba_z_[j] == -1){
				// Update
				if (neig_iter_nei_his[neig_z_his[j]] > agent.iteration_neighbors_[neig_z_his[j]]){
					// std::cout << "case 17" << std::endl;
					agent.cbba_z_[j] = neig_z_his[j];
					agent.cbba_y_[j] = neig_y_his[j];
				}
			}
			else{
				continue;
				// std::cout << "Unknown winner value Entry 13" << std::endl;
			}
				
		}


		/*********************************************************************************************************/
		/*********************************************************************************************************/
		/*********************************************************************************************************/
		// Entries 14 to 17
		else if (neig_z_his[j] == -1){

			/***************************************** Entry 14 ***************************************/
			// Entry 14
			// Leave
			if (agent.cbba_z_[j] == agent.idx_){
				continue;
				// std::cout << "Do nothing Entry 14" << std::endl;
			}
				

			/***************************************** Entry 15 ***************************************/
			// Entry 15
			// Update
			else if (agent.cbba_z_[j] == neig_idx){
				// std::cout << "case 18" << std::endl;
				agent.cbba_z_[j] = neig_z_his[j];
				agent.cbba_y_[j] = neig_y_his[j];
			}

			/***************************************** Entry 16 ***************************************/
			// Entry 16
			// Update
			else if (agent.cbba_z_[j] >= 0){
				// Update
				if (neig_iter_nei_his[agent.cbba_z_[j]] > agent.iteration_neighbors_[agent.cbba_z_[j]]){
					// std::cout << "case 19" << std::endl;
					agent.cbba_z_[j] = neig_z_his[j];
					agent.cbba_y_[j] = neig_y_his[j];
				}
			}

			/***************************************** Entry 17 ***************************************/
			// Entry 17
			// Leave
			else if(agent.cbba_z_[j] == -1){
				continue;
				// std::cout<< "Do noting Entry 17" << std::endl;
			}
				

			else{
				continue;
				// std::cout << "Unknown winner value Entry 17" <<std::endl;
			}
				
		}

		else{
			continue;
			// std::cout << "Unknown winner value end of communicate" <<std::endl;
		}
			
	}

	for (int n = 0; n < agent.num_agents_; n++){
		if (n != agent.idx_ && agent.iteration_neighbors_[n] < neig_iter_nei_his[n]){
			agent.iteration_neighbors_[n] = neig_iter_nei_his[n];
		}
	}

	//agent.iteration_neighbors[(*neig_agent).Index] = (*neig_agent).Iter;
    agent.iteration_neighbors_[neig_idx] = agent.iter_ + 1;
}

// Find the available tasks for agent
void CBBA::available_tasks_finder(cbba_Agent& agent_sig){
	//Initialize the available tasks
	agent_sig.h_avai_.clear();
	bool condition_1;
	bool condition_2;

	for (int j = 0; j < agent_sig.num_tasks_inde_; j++){
		// Initialize the condition for each task
		condition_1 = 0;
		condition_2 = 0;

		if(agent_sig.cbba_award_[j] - agent_sig.cbba_y_[j] > eps)
			condition_1 = 1;
		else if (abs(agent_sig.cbba_award_[j] - agent_sig.cbba_y_[j]) <= eps)
			if (agent_sig.idx_ < agent_sig.cbba_z_[j])
				condition_2 = 1;

		if (condition_1 == 1 || condition_2 == 1)
			agent_sig.h_avai_.push_back(j);
	}

};


// Find the desired task for agent
int CBBA::desired_task_finder(cbba_Agent& agent_sig){
	int max = 0;
	int desired_Index = -1;

	// Update the available tasks for agent
	CBBA::available_tasks_finder(agent_sig);

	if (!agent_sig.h_avai_.empty()){
		// If for certain agent, there are more than one desired tasks, select the one with smaller task index
		for (int j = 0; j < agent_sig.h_avai_.size();j++)
			if (agent_sig.cbba_award_[agent_sig.h_avai_[j]] > max){
		    	max = agent_sig.cbba_award_[agent_sig.h_avai_[j]];
		    	desired_Index = agent_sig.h_avai_[j];
			}
	}

	return desired_Index;
};

void CBBA::bundle_remove(cbba_Agent& agent){
	bool outbidForTask = 0;
	if (!agent.cbba_bundle_.empty()){
		// Check whether agent is outbid by other agent for the tasks which are in the bundle
		for (int k = 0; k < agent.cbba_bundle_.size(); k++){
			if (agent.cbba_z_[agent.cbba_bundle_[k]] != agent.idx_)
				outbidForTask = 1;

			// Remove the tasks added after the outbid task
			if (outbidForTask == 1){
				if (agent.cbba_z_[agent.cbba_bundle_[k]] == agent.idx_){
					agent.cbba_z_[agent.cbba_bundle_[k]] = -1;
					agent.cbba_y_[agent.cbba_bundle_[k]] = -1;
				}

				// Once the path is taken into consideration
				// we need to find the position of task in the path
				// remove the task from the bundle
				agent.cbba_bundle_[k] = -1;
			}
		}
	}

	// Remove the task which has -1
	agent.cbba_bundle_.erase(std::remove(agent.cbba_bundle_.begin(),agent.cbba_bundle_.end(),-1),agent.cbba_bundle_.end());
	
	CBBA::path_remove(agent);

}


// Remove the task from bundle and all the tasks added after it
void CBBA::bundle_remove(std::vector<cbba_Agent>& agents){
	for (auto it_ag = agents.begin(); it_ag != agents.end(); it_ag++){
		CBBA::bundle_remove((*it_ag));
	}

	CBBA::path_remove(agents);
};


void CBBA::path_remove(cbba_Agent& agent){
	std::vector<bool> existence(agent.cbba_path_.size(), 0);
	std::vector<int> cbba_path_copy = agent.cbba_path_;
	for (int j = 0; j < cbba_path_copy.size(); j++){
		std::vector<int>::iterator it = std::find(agent.cbba_bundle_.begin(), agent.cbba_bundle_.end(), cbba_path_copy[j]);
		if(it == agent.cbba_bundle_.end()){
			agent.cbba_path_.erase(std::remove(agent.cbba_path_.begin(),agent.cbba_path_.end(),cbba_path_copy[j]),agent.cbba_path_.end());
		}
	}
}


void CBBA::path_remove(std::vector<cbba_Agent>& agents){
	// Remove the tasks in the path but not in the bunlde (after bundle_remove)
	for(auto it_ag = agents.begin(); it_ag != agents.end(); it_ag++){
		CBBA::path_remove((*it_ag));
	}
}


void CBBA::bundle_add(LTLFormula Global_LTL,const std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<cbba_Agent>& agents){

	for (auto it_ag = agents.begin(); it_ag != agents.end(); it_ag++){
		CBBA::bundle_add(Global_LTL, graph, (*it_ag));
	}

};


void CBBA::bundle_add(LTLFormula Global_LTL,const std::shared_ptr<Graph_t<SquareCell*>> graph, cbba_Agent& agent){
	bool bundleFull = -1;

	// The best position for all tasks which have not been added into the bundle
	std::vector<int> best_position;

	// Check whether the bundle is full or not
	if (agent.cbba_bundle_.size() > agent.max_tasks_){
		std::cout << "The bundle is full" << std::endl;
		bundleFull = 1;
	}
	else
		bundleFull = 0;

	while(bundleFull == 0){
		std::vector<int> best_position = agent.award_update(Global_LTL, graph);
		int desired_Index = CBBA::desired_task_finder(agent);
		if (desired_Index == -1)
			break;

		// Update the assignment
		agent.cbba_z_[desired_Index] = agent.idx_;
		agent.cbba_y_[desired_Index] = agent.cbba_award_[desired_Index];

		// Insert the desired task into the path with best insert position
		std::vector<int>::iterator it;
		it = agent.cbba_path_.begin();
		agent.cbba_path_.insert(it+best_position[desired_Index],desired_Index);

		// Insert the desired task into the bundle
		agent.cbba_bundle_.push_back(desired_Index);


		if (agent.cbba_bundle_.size() > agent.max_tasks_)
			bundleFull = 1;

		// std::cout << "=================================== DEBUG FOR BUNDLE ADD =======================================" << std::endl;
		// std::cout << "For vehicle "<< agent.idx_ << " , Add task " << desired_Index << " at pos " << best_position[desired_Index] << std::endl;
	}

	agent.history_.z_history.push_back(agent.cbba_z_);
	agent.history_.y_history.push_back(agent.cbba_y_);
}


void CBBA::bundle_add(LTLFormula Global_LTL,const std::shared_ptr<Graph_t<SquareCell*>> graph,std::vector<cbba_Agent>& agents,
		std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph, TileTraversalData tile_traversal_data, 
		std::shared_ptr<SquareGrid> grid){

	for (auto it_ag = agents.begin(); it_ag != agents.end(); it_ag++){
		bool bundleFull = -1;
	
		// The best position for all tasks which have not been added into the bundle
		// The index of the best_position is same as the index of task
		std::vector<int> best_position;
		if ((*it_ag).cbba_bundle_.size() > (*it_ag).max_tasks_){
			std::cout << "The bundle is full" << std::endl;
			bundleFull = 1;
		}
		else
			bundleFull = 0;

		while(bundleFull == 0){
			//award_update;
			std::vector<int> best_position = (*it_ag).award_update_CBTA(Global_LTL, graph, lifted_graph, tile_traversal_data, grid);
			int desired_Index = CBBA::desired_task_finder((*it_ag));
			if (desired_Index == -1)
				break;

			// Update the assignment
			(*it_ag).cbba_z_[desired_Index] = (*it_ag).idx_;
			(*it_ag).cbba_y_[desired_Index] = (*it_ag).cbba_award_[desired_Index];


			// Insert the desired task into the path with best insert position
			std::vector<int>::iterator it;
			it = (*it_ag).cbba_path_.begin();
			(*it_ag).cbba_path_.insert(it+best_position[desired_Index],desired_Index);
			// Insert the desired task into the bundle
			(*it_ag).cbba_bundle_.push_back(desired_Index);

			if ((*it_ag).cbba_bundle_.size() > (*it_ag).max_tasks_)
				bundleFull = 1;
		}

		(*it_ag).history_.z_history.push_back((*it_ag).cbba_z_);
		(*it_ag).history_.y_history.push_back((*it_ag).cbba_y_);
	}

};


// Check whether assignment is over or not
bool CBBA::success_checker(std::vector<cbba_Agent> agent){

	std::vector<int> BundlesForAllAgents;
	BundlesForAllAgents.clear();
	bool successFlag;
	successFlag = 0;
	int Num;
	int AppearOnce = 0;

	for (int i = 0; i < agent.size(); i++)
		for (int j = 0; j < agent[i].cbba_bundle_.size(); j++)
			BundlesForAllAgents.push_back(agent[i].cbba_bundle_[j]);

	for (int j = 0; j < agent[0].num_tasks_inde_ ;j++){
		Num = std::count(BundlesForAllAgents.begin(),BundlesForAllAgents.end(),j);
		if (Num == 1)
			AppearOnce = AppearOnce + 1;
	}

	if (AppearOnce == agent[0].num_tasks_inde_)
		successFlag = 1;

	return successFlag;
};


// Synchronization task assignment 
void cbba_Agent::update_reward_syn(CBBATasks tasks, std::shared_ptr<Graph_t<SquareCell*>> graph, int task_idx_syn){
	std::cout << "++++++++++++++++++++++++++++++ Update the reward ++++++++++++++++++++++++++++++++++++ " << std::endl;
	cbba_Task task = tasks.FindTaskFromID(task_idx_syn);
	std::cout << "task" << task.idx_ << "number of agents required " << task.num_agents_ << std::endl;
	std::vector<int>::iterator it = std::find(task.winners_.begin(), task.winners_.end(), idx_);
	if(it != task.winners_.end()){
		std::cout << "Have been selected " << std::endl;
		cbba_award_[task.idx_] = 0.0;
		winning_bids_matrix_(task.idx_, idx_) = 0.0;
	}
	else{
		float length_origin = CBBA::PathLengthCalculationWithDelay(tasks,graph,cbba_path_,start_node_);
		std::cout << "The original length is " << length_origin << std::endl;

		// Find all possible insert positions
		std::vector<std::vector<int>> path_poss = CBBA::PossibleInsertPosFinder(cbba_path_, task.idx_);
		std::vector<float> bids(path_poss.size(), 0.0);
		for (int k=0; k < path_poss.size(); k++){
			
			std::cout << "The possible path is ";
			for (auto &e: path_poss[k]){
				std::cout << e << " ";
			}
			std::cout << std::endl;
		
			// Compute the extra steps required
			float length_current = CBBA::PathLengthCalculationWithDelay(tasks,graph,path_poss[k],start_node_);
			float extra_steps = fabs(length_current - length_origin);
			std::cout << "The extra steps is " << extra_steps << std::endl;
			bids[k] = extra_steps;
		}

		float min_bid = 1000.0;
		int best_pos = -1;
		for(int m = 0; m < bids.size(); m++){
			if(bids[m] < min_bid){
				min_bid = bids[m];
				best_pos = m;
			}
		}

		// Update award, winning_matrix, assignment, 
		std::vector<int> path_best = path_poss[best_pos];
		path_best.erase(path_best.begin() + best_pos + 1, path_best.end());
		std::cout << "The best inserted path is ";
		for(auto &e: path_best){
			std::cout << e << " ";
		}
		std::cout << std::endl;
		float length_path = CBBA::PathLengthCalculationWithDelay(tasks, graph, path_best, start_node_);
		cbba_award_[task.idx_] = cbba_benefit - length_path;
		winning_bids_matrix_(task.idx_, idx_) = cbba_award_[task.idx_];
		assignment_matrix_(task.idx_, idx_) = 1;

		cbba_path_.insert(cbba_path_.begin() + best_pos, task.idx_);
		cbba_bundle_.push_back(task.idx_);

		std::cout << "At the end of update reward, the path of vehicle " << idx_ << std::endl;
		for (auto &e: cbba_path_){
			std::cout << e << " ";
		}
		std::cout << std::endl;
		std::cout << "The reward is " << winning_bids_matrix_(task.idx_, idx_) << std::endl;

	}

	history_.winning_bids_.push_back(winning_bids_matrix_);
	history_.assignment_.push_back(assignment_matrix_);
}


float CBBA::PathLengthCalculationWithDelay(CBBATasks tasks, std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<int> bundle, int start_idx){
	float length = 0.0;

	Path_t<SquareCell *> path_part;
	Path_t<SquareCell *> path;
	Vertex_t<SquareCell*> * start_node;
	Vertex_t<SquareCell*> * finish_node;

	// Initialize the start_node which should be initial position of agent
	start_node = graph->GetVertexFromID(start_idx);
	for (int i = 0; i < bundle.size(); i++){
        auto task = tasks.FindTaskFromID(bundle[i]);
        if(task.num_agents_ == 1){
			for (int j = 0; j < task.pos_.size(); j++){
				finish_node = graph->GetVertexFromID(task.pos_[j]);
				path_part = AStar::Search(graph,start_node->state_->id_,finish_node->state_->id_, CalcHeuristicFunc_t<SquareCell *>(CBBA::CalcHeuristic));
				if (i == bundle.size()-1 && j == task.pos_.size()-1){
					path.insert(path.end(), path_part.begin(), path_part.end());
				}
				else{
					path.insert(path.end(), path_part.begin(), path_part.end()-1);
				}
				start_node = finish_node;
			}
        }

        else{
            for (int j = 0; j < task.pos_.size(); j++){
                finish_node = graph->GetVertexFromID(task.pos_[j]);
                path_part = AStar::Search(graph,start_node->state_->id_,finish_node->state_->id_, CalcHeuristicFunc_t<SquareCell *>(CBBA::CalcHeuristic));
				float length_current = 0.0;
                if (i == bundle.size()-1 && j == task.pos_.size()-1){
                    path.insert(path.end(), path_part.begin(), path_part.end());
					length_current = path.size();
                }
                else{
                    path.insert(path.end(), path_part.begin(), path_part.end()-1);
					length_current = path.size() + 1;
                }

                start_node = finish_node;

                // // Compare
				// int t_wait = 0;
				// // if(task.min_bid_ > 0 && length_current < cbba_benefit - task.min_bid_){
				// if(task.min_bid_ > 0){
				// 	std::cout << "length_current " << length_current << std::endl;
				// 	std::cout << "task.min_bid_ " << task.min_bid_ << std::endl;
				// 	// t_wait = abs(std::round(length_current - cbba_benefit + task.min_bid_));
				// 	if (length_current < cbba_benefit - task.min_bid_){
				// 		t_wait = abs(std::round(length_current - cbba_benefit + task.min_bid_));
				// 	}
				// 	else{
				// 		t_wait = 0;
				// 	}
					
				// 	std::cout << "waiting time is " << t_wait << std::endl;
				// }
                // if (t_wait != 0){
				// 	std::vector<SquareCell *> waiting_cell(t_wait, path_part.back());
				// 	path.insert(path.end(), waiting_cell.begin(), waiting_cell.end());
				// }
				
            }

        }
            
	}

    // std::cout << "Current bundle is " << std::endl;
    // for (auto  &e: bundle){
    //     std::cout << e << ", ";
    // }
    // std::cout << std::endl;
    // std::cout << "The length of bundle is " << path.size() << std::endl;

	double path_length = path.size();
	return path_length;
}



int CBBA::find_num_agents(LTLFormula Global_LTL, int task_syn){
	for (int i= 0; i < Global_LTL.task_info.size(); i++){
		if(Global_LTL.sub_buchi_regions.sub_buchi_regions_Liveness_T[task_syn][0] == Global_LTL.task_info[i].P_idx_){
			return Global_LTL.task_info[i].num_agents_;
		}
	}
}


cbba_Agent& CBBA::FindAgent(std::vector<cbba_Agent>& agents, int idx){
	for (auto &agent: agents){
		if (agent.idx_ == idx){
			return agent;
		}
	}
}

std::vector<std::vector<int>> CBBA::PossibleInsertPosFinder(std::vector<int> current_path, int task_syn){

	// // Find all possibilities to insert task_syn for current agent
	// std::vector<std::vector<int>> path_poss;
	// path_poss.clear();
	// // If the size of cbba_bundle is n, then there are n+1 positions to insert each task in the tasks_NotInBundle
	// for (int k = 0; k < current_path.size()+1; k++)
	// 	path_poss.push_back(current_path);
	std::vector<std::vector<int>> path_poss(current_path.size()+1, current_path);
	// Insert one task (task_NotInBundle[j]) into all the possible position
	// Each item in the path_copy represents a possible position to insert the task
	//int task_syn = 0;
	for (int m = 0; m < path_poss.size(); m++){
		auto it_pos_other_ag = path_poss[m].begin();
		path_poss[m].insert(it_pos_other_ag+m,task_syn);
	}

	return path_poss;
}

void cbba_Agent::assignment_update(cbba_Task& task, bool flag){
	if (flag == 1){
		std::vector<int>::iterator it = std::find(task.winners_.begin(), task.winners_.end(), idx_);
		if(it == task.winners_.end()){
			cbba_bundle_.pop_back();
			cbba_path_.erase(std::remove(cbba_path_.begin(), cbba_path_.end(), task.idx_), cbba_path_.end());
		}
	}
}




void CBBA::update_iteration_number(std::vector<cbba_Agent>& agents){
    
    for(auto &agent: agents){
        for (int i = 0; i < agent.neighbors_.size(); i++){
            cbba_Agent& neighbor = CBBA::FindAgent(agents, agent.neighbors_[i]);

            // update the iteration information for neighbor 
            for (int idx = 0; idx < agent.num_agents_; idx++){
                if (idx != neighbor.idx_ && neighbor.iteration_neighbors_[idx] < agent.history_.iter_neighbors_his.back()[idx]){
                    neighbor.iteration_neighbors_[idx] = agent.history_.iter_neighbors_his.back()[idx];
                }
            }

            neighbor.iteration_neighbors_[agent.idx_] = agent.iter_;
        }
    }

    for (auto &agent: agents){
			agent.history_.iter_neighbors_his.push_back(agent.iteration_neighbors_);
	}
}


std::vector<float> CBBA::findKclosest(std::vector<cbba_Agent>& agents, float leader_bid, int leader_idx, int K, int n, cbba_Task task){
	std::multimap<float, int> ordered_bids_map;
	for (auto &agent: agents){
		if (agent.idx_ != leader_idx)
			ordered_bids_map.insert(std::pair<float, int>(agent.winning_bids_matrix_(task.idx_, agent.idx_), agent.idx_));
	}

	std::vector<float> ordered_bids;
	for (auto &bid: ordered_bids_map){
		ordered_bids.push_back(bid.first);
		std::cout << bid.first << " ";
	}
	std::cout << std::endl;

	// Find the crossover point
	int num_elements = ordered_bids.size();
	int left = CBBA::findCrossOver(ordered_bids, 0, num_elements-1, leader_bid);
	int right = left + 1;
	int count = 0;

	std::vector<float> Kclosest_bids = {};
	if (ordered_bids[left] == leader_bid){
		Kclosest_bids.push_back(ordered_bids[left]);
		count ++;
		left --;
	}

	
	while (left >= 0 && right < num_elements && count < K){
		if(leader_bid - ordered_bids[left] < ordered_bids[right] - leader_bid){
			Kclosest_bids.push_back(ordered_bids[left--]);
		}
		else{
			Kclosest_bids.push_back(ordered_bids[right++]);
		}
		count ++;
	}


	while (count < K && left >= 0){
		Kclosest_bids.push_back(ordered_bids[left--]);
		count ++;
	}

	while (count < K && right < num_elements){
		Kclosest_bids.push_back(ordered_bids[right++]);
		count ++;
	}

	std::cout << "The close K elements are " << std::endl;
	for (auto &e: Kclosest_bids){
		std::cout << e << " ";
	}
	std::cout << std::endl;

	return Kclosest_bids;
}

std::vector<int> CBBA::findKclosest_idx(std::vector<float> Kclosest_bids, std::vector<cbba_Agent>& agents, cbba_Task task){
	std::vector<int> idxs = {};
	for (int i = 0; i < agents.size(); i++){
		std::vector<int>::iterator it=std::find(task.winners_.begin(), task.winners_.end(), agents[i].idx_);
		if (it == task.winners_.end()){
			for (auto &bid: Kclosest_bids){
				if (bid == agents[i].winning_bids_matrix_(task.idx_, agents[i].idx_)){
					idxs.push_back(agents[i].idx_);
					break;
				}
			}
		}
	}

	std::cout << "The K closest agents' idx are " << std::endl;
	std::cout << "The size of idxs are "<< idxs.size() << std::endl;
	for (auto &idx: idxs){
		std::cout << idx << " ";
	}
	std::cout << std::endl;
	return idxs;
	
}


int CBBA::findCrossOver(std::vector<float> ordered_bids, int low, int high, int leader_bid){
	if (ordered_bids[high] <= leader_bid){
		return high;
	}
	if(ordered_bids[low] > leader_bid){
		return low;
	}

	// FInd the middle point
	int mid = low + (high - low)/2;

	if(ordered_bids[mid] <= leader_bid && ordered_bids[mid+1] > leader_bid){
		return mid;
	}

	if(ordered_bids[mid] < leader_bid){
		return findCrossOver(ordered_bids, mid+1, high, leader_bid);
	}
	
	return findCrossOver(ordered_bids, low, mid-1, leader_bid);
}


void cbba_Agent::update_reward_dependent(CBBATasks tasks, std::vector<int> dependent_tasks_idx, std::shared_ptr<Graph_t<SquareCell*>> graph){
	for (auto &t_idx: dependent_tasks_idx){
		std::vector<std::vector<int>> path_poss = CBBA::PossibleInsertPosFinder(cbba_path_, t_idx);
		std::vector<float> c_rewards = {};
		for (auto &path_case: path_poss){
			c_rewards.push_back(CBBA::PathLengthCalculationWithDelay(tasks, graph, path_case, start_node_));
		}

		float min_reward = 1000.0;
		int best_pos = -1;
		for (int i = 0; i < c_rewards.size(); i++){
			if(c_rewards[i] < min_reward){
				min_reward = c_rewards[i];
				best_pos = i;
			}
		}
		cbba_award_[t_idx] = cbba_benefit - min_reward;
		insert_pos_[t_idx] = best_pos;
	}
}


void cbba_Agent::update_reward_dependent(CBBATasks tasks, std::vector<int> dependent_tasks_idx, LTLFormula Global_LTL, 
std::shared_ptr<Graph_t<SquareCell*>> graph, std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph, TileTraversalData tile_traversal_data, 
std::shared_ptr<SquareGrid> grid){
	for (auto &t_idx:dependent_tasks_idx){
		std::vector<std::vector<int>> path_poss = CBBA::PossibleInsertPosFinder(cbba_path_, t_idx);

		std::vector<float> c_rewards = {};
		for (auto &path_case: path_poss){
			c_rewards.push_back(CBBA::PathLengthCalculationCBTA(Global_LTL, graph, path_case, start_node_, lifted_graph, tile_traversal_data, grid));
		}
	

		float min_reward = 1000.0;
		int best_pos = -1;
		for(int i = 0; i < c_rewards.size(); i++){
			if(c_rewards[i] < min_reward){
				min_reward = c_rewards[i];
				best_pos = i;
			}
		}

		cbba_award_[t_idx] = cbba_benefit - min_reward;
		insert_pos_[t_idx] = best_pos;
	}
}

void cbba_Agent::bundle_add_dependent(CBBATasks tasks, std::vector<int> dependent_tasks_idx, std::shared_ptr<Graph_t<SquareCell*>> graph){
	std::vector<int> available_tasks = dependent_tasks_idx;
	while (!available_tasks.empty()){
		update_reward_dependent(tasks, available_tasks, graph);
		float max_reward = 0.0;
		int best_idx = -1;
		for (auto &t_idx: available_tasks){
			if (cbba_award_[t_idx] > max_reward){
				max_reward = cbba_award_[t_idx];
				best_idx = t_idx;
			}
		}
		// std::cout << "Desired task idx is "<< best_idx << std::endl;
		// Update bundle and path
		std::vector<int>::iterator it = cbba_path_.begin();
		if (best_idx != -1){
			cbba_path_.insert(it+insert_pos_[best_idx],best_idx);
			cbba_bundle_.push_back(best_idx);

			available_tasks.erase(std::remove(available_tasks.begin(), available_tasks.end(), best_idx), available_tasks.end());
		}
		else{
			std::cout << "Can not find feasible solution becasue of kinematic constraint. " << std::endl;
			available_tasks.erase(std::remove(available_tasks.begin(), available_tasks.end(), best_idx), available_tasks.end());
		}
		
	}
}


void cbba_Agent::bundle_add_dependent(CBBATasks tasks, std::vector<int> dependent_task_idx, LTLFormula Global_LTL, 
std::shared_ptr<Graph_t<SquareCell*>> graph, std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph, TileTraversalData tile_traversal_data, 
std::shared_ptr<SquareGrid> grid){
	std::vector<int> available_tasks = dependent_task_idx;
	while(! available_tasks.empty()){
		update_reward_dependent(tasks,available_tasks,Global_LTL,graph,lifted_graph,tile_traversal_data,grid);

		float max_reward = 0.0;
		int best_idx = -1;
		for (auto &t_idx:available_tasks){
			if(cbba_award_[t_idx] > max_reward){
				max_reward = cbba_award_[t_idx];
				best_idx = t_idx;
			}
		}
		std::cout << "Desired task idx is " << best_idx << std::endl;

		//update the bundle and path
		std::vector<int>::iterator it = cbba_path_.begin();
		cbba_path_.insert(it+insert_pos_[best_idx],best_idx);
		cbba_bundle_.push_back(best_idx);

		available_tasks.erase(std::remove(available_tasks.begin(), available_tasks.end(), best_idx), available_tasks.end());
	}
}

void cbba_Agent::update_dependent_path_length(CBBATasks tasks, std::vector<int> dependent_tasks_idx, std::shared_ptr<Graph_t<SquareCell*>> graph){
	for (auto &t_idx: dependent_tasks_idx){
		std::vector<int>::iterator it = std::find(cbba_path_.begin(), cbba_path_.end(), t_idx);
		if (it != cbba_path_.end()){
			auto task_syn_pos = it - cbba_path_.begin();
			std::vector<int> path_part = cbba_path_;
			path_part.erase(path_part.begin()+task_syn_pos+1, path_part.end());
			float path_length_part = CBBA::PathLengthCalculationWithDelay(tasks, graph, path_part, start_node_);
			winning_bids_matrix_(t_idx, idx_) = path_length_part;
			assignment_matrix_(t_idx, idx_) = 1;
		}
	}
}


void cbba_Agent::update_dependent_path_length(CBBATasks tasks, std::vector<int> dependent_task_idx, LTLFormula Global_LTL, 
std::shared_ptr<Graph_t<SquareCell*>> graph, std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph, TileTraversalData tile_traversal_data, 
std::shared_ptr<SquareGrid> grid){
	for (auto &t_idx:dependent_task_idx){
		std::vector<int>::iterator it = std::find(cbba_path_.begin(), cbba_path_.end(), t_idx);
		if (it != cbba_path_.end()){
			auto task_syn_pos = it - cbba_path_.begin();
			std::vector<int> path_part = cbba_path_;
			path_part.erase(path_part.begin()+task_syn_pos+1, path_part.end());
			float path_length_part = CBBA::PathLengthCalculationCBTA(Global_LTL, graph, path_part, start_node_,lifted_graph, tile_traversal_data, grid);
			winning_bids_matrix_(t_idx, idx_) = path_length_part;
			assignment_matrix_(t_idx, idx_) = 1;
		}
	}
}

void cbba_Agent::dependent_tasks_convergence(int task_idx, int num_agent, std::vector<cbba_Agent>& agents){
	std::multimap<float, int> ordered_bids_map = {};
	for (auto &agent: agents){
		ordered_bids_map.insert(std::pair<float, int>(winning_bids_matrix_(task_idx,agent.idx_), agent.idx_));
	}
	std::multimap<float, int>::iterator it_start = ordered_bids_map.begin();
	int count = 0;
	float diff_min = 1000.0;
	std::multimap<float, int>::iterator opt_start = ordered_bids_map.begin();
	std::multimap<float, int>::iterator opt_end = ordered_bids_map.begin();
	while (count < ordered_bids_map.size() - num_agent + 1){
		std::cout << "it start is " << (*it_start).first << std::endl;
		std::multimap<float, int>::iterator it_end = it_start;
		std::advance(it_end, num_agent - 1);
		std::cout << "The end bid is " << (*it_end).first << std::endl;
		float diff = abs((*it_end).first - (*it_start).first);

		if (diff < diff_min){
			diff_min = diff;
			opt_start = it_start; 
			opt_end = it_end;
		}

		std::cout << "The diff is " << diff << std::endl;
		std::cout << "============================================ " << std::endl;

		it_start ++;
		count ++;
	}


	// Update the assignment
	std::vector<int> winners = {};
	while (winners.size() < num_agent){
		winners.push_back((*opt_start).second);
		opt_start ++;
	}

	std::vector<int>::iterator it = std::find(winners.begin(), winners.end(), idx_);
	if(it == winners.end()){
		cbba_path_.erase(std::remove(cbba_path_.begin(),cbba_path_.end(),task_idx),cbba_path_.end());
		cbba_bundle_.erase(std::remove(cbba_bundle_.begin(), cbba_bundle_.end(), task_idx), cbba_bundle_.end());
		// assignment_matrix_(task_idx, idx_) = 0;
	}



	std::cout << "winners are " << std::endl;
	for (auto &e: winners){
		std::cout << e << " ";
	}
	std::cout << std::endl;

}


void CBBA::communicate_dependent_lcm(cbba_Agent& agent, cbba_Task& task, int neighbor_idx, std::vector<std::vector<float>> bids_matrix, std::vector<std::vector<int>> assignment_matrix, std::vector<int> iteration_neighbors){
	std::vector<int> winners_agent = agent.winners_finder(task);
	std::vector<int> winners_neighbor = {};
	for (int idx = 0; idx < agent.num_agents_; idx++){
		if(assignment_matrix[task.idx_][idx] == 1 && bids_matrix[task.idx_][idx] > 0){
			winners_neighbor.push_back(idx);
		}
	}

	for (auto &m: winners_agent){
		// std::cout << "neighbor.history_.iter_neighbors_his.back()[m] " << iteration_neighbors[m] << std::endl;
		// std::cout << "agent.iteration_neighbors_[m] " << agent.iteration_neighbors_[m] << std::endl;
		if(m == neighbor_idx || iteration_neighbors[m] > agent.iteration_neighbors_[m]){
			// std::cout << "Case 1 " << std::endl;
			agent.winning_bids_matrix_(task.idx_, m) = bids_matrix[task.idx_][m];
			agent.assignment_matrix_(task.idx_, m) = assignment_matrix[task.idx_][m];
		}
	}

	int num_winners_agent = agent.winners_count(task);
	for (auto &m: winners_neighbor){
		std::vector<int>::iterator it = std::find(winners_agent.begin(), winners_agent.end(), m);
		if (m != agent.idx_ && iteration_neighbors[m] > agent.iteration_neighbors_[m] && it == winners_agent.end()){
			// std::cout << "Enter here " << std::endl;
			if (num_winners_agent < task.num_agents_){
				// std::cout << "Case 2 " << std::endl;
				agent.winning_bids_matrix_(task.idx_, m) = bids_matrix[task.idx_][m];
				agent.assignment_matrix_(task.idx_, m) = assignment_matrix[task.idx_][m];
			}
			else{
				// Find max, mid bid
				// std::cout << "case 3 " << std::endl;
				std::pair<int, float> max_bid_pair_ = agent.max_bid_finder(task);
				std::pair<int,float> min_bid_pair_ = agent.min_bid_finder(task);
				// std::cout << "max_bid_pair is " << max_bid_pair_.second << std::endl;
				// std::cout << "min_bid_pair is " << min_bid_pair_.second << std::endl;
				// std::cout << "neighbor.history_.winning_bids_.back()(task.idx_,m) " << iteration_neighbors[m] << std::endl;
				float diff = abs(max_bid_pair_.second - min_bid_pair_.second);

				if(min_bid_pair_.second <= bids_matrix[task.idx_][m] && bids_matrix[task.idx_][m] <= max_bid_pair_.second){
					if(abs(min_bid_pair_.second - bids_matrix[task.idx_][m]) <= abs(max_bid_pair_.second - bids_matrix[task.idx_][m])){
						// std::cout << "case 4 " << std::endl;
						agent.winning_bids_matrix_(task.idx_, m) = bids_matrix[task.idx_][m];
						agent.assignment_matrix_(task.idx_, m) = assignment_matrix[task.idx_][m];

						agent.winning_bids_matrix_(task.idx_, max_bid_pair_.first) = 0.0;
						agent.assignment_matrix_(task.idx_, max_bid_pair_.first) = 0;
					}
					else{
						// std::cout << "case 5 " << std::endl;
						agent.winning_bids_matrix_(task.idx_, m) = bids_matrix[task.idx_][m];
						agent.assignment_matrix_(task.idx_, m) = assignment_matrix[task.idx_][m];

						agent.winning_bids_matrix_(task.idx_, min_bid_pair_.first) = 0.0;
						agent.assignment_matrix_(task.idx_, min_bid_pair_.first) = 0;
					}
				}
				else if(bids_matrix[task.idx_][m] < min_bid_pair_.second){
					// find second max bid
					std::pair<int, float> sec_max_bid_pair_ = agent.second_max_bid_finder(task);
					// std::cout << "sec_max_bid_pair is " << sec_max_bid_pair_.second << "()()(()()())()()()()() " << std::endl;
					if (abs(sec_max_bid_pair_.second - bids_matrix[task.idx_][m]) <= diff){
						// std::cout << "case 6 " << std::endl;
						agent.winning_bids_matrix_(task.idx_, m) = bids_matrix[task.idx_][m];
						agent.assignment_matrix_(task.idx_, m) = assignment_matrix[task.idx_][m];

						agent.winning_bids_matrix_(task.idx_, max_bid_pair_.first) = 0.0;
						agent.assignment_matrix_(task.idx_, max_bid_pair_.first) = 0;
					}
				}

				else if(bids_matrix[task.idx_][m] > max_bid_pair_.second){
					std::pair<int, float> sec_min_bid_pair_ = agent.second_min_bid_finder(task);
					// std::cout << "Second minimum bid is " << sec_min_bid_pair_.second << std::endl;
					if(abs(sec_min_bid_pair_.second - bids_matrix[task.idx_][m]) <= diff){
						// std::cout << "case 7 " << std::endl;
						agent.winning_bids_matrix_(task.idx_, m) = bids_matrix[task.idx_][m];
						agent.assignment_matrix_(task.idx_, m) = assignment_matrix[task.idx_][m];

						agent.winning_bids_matrix_(task.idx_, min_bid_pair_.first) = 0.0;
						agent.assignment_matrix_(task.idx_, min_bid_pair_.first) = 0;
					}
				}
						
			}
		}
	}

	// std::cout << "The result of talking to neighbor " << neighbor_idx << " is " << std::endl;
	// std::cout << "Winning bids are " << std::endl;
	// std::cout << agent.winning_bids_matrix_ << std::endl;
	// std::cout << "assignmeny matrix is " << std::endl;
	// std::cout << agent.assignment_matrix_ << std::endl;
	// std::cout << std::endl;

}



void CBBA::communicate_dependent_decentralized(std::vector<cbba_Agent>& agents, cbba_Task& task){
	for (auto &agent: agents){
		agent.neighbor_finder();
		std::cout << "SKR, SKR, SKR, SKR, SKR~~~~~~~~~~" << std::endl;
		for (auto &neighbor_idx: agent.neighbors_){
			cbba_Agent& neighbor = CBBA::FindAgent(agents, neighbor_idx);
			/*** ======================================== Debug ======================================= ***/
			std::cout << "agent " << agent.idx_ << std::endl;
			std::cout << "winning bids matrix is " << std::endl;
			std::cout << agent.winning_bids_matrix_ << std::endl;
			std::cout << "assignment matrix is " << std::endl;
			std::cout << agent.assignment_matrix_ << std::endl;
			std::cout << "=========================================================== " << std::endl;
			std::cout << std::endl;
			std::cout << "neighbor " << neighbor.idx_ << std::endl;
			std::cout << "winning bids matrix is " << std::endl;
			std::cout << neighbor.history_.winning_bids_.back() << std::endl;
			std::cout << "assignment matrix is " << std::endl;
			std::cout << neighbor.history_.assignment_.back() << std::endl;
			std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
			std::cout << std::endl; 

			std::vector<int> winners_agent = agent.winners_finder(task);
			std::vector<int> winners_neighbor = neighbor.his_winners_finder(task);
			

			for (auto &m: winners_agent){
				std::cout << "neighbor.history_.iter_neighbors_his.back()[m] " << neighbor.history_.iter_neighbors_his.back()[m] << std::endl;
				std::cout << "agent.iteration_neighbors_[m] " << agent.iteration_neighbors_[m] << std::endl;
				if(m == neighbor.idx_ || neighbor.history_.iter_neighbors_his.back()[m] > agent.iteration_neighbors_[m]){
					std::cout << "Case 1 " << std::endl;
					agent.winning_bids_matrix_(task.idx_, m) = neighbor.history_.winning_bids_.back()(task.idx_,m);
					agent.assignment_matrix_(task.idx_, m) = neighbor.history_.assignment_.back()(task.idx_, m);
				}
			}

			int num_winners_agent = agent.winners_count(task);
			for (auto &m: winners_neighbor){
				std::vector<int>::iterator it = std::find(winners_agent.begin(), winners_agent.end(), m);
				if (m != agent.idx_ && neighbor.history_.iter_neighbors_his.back()[m] > agent.iteration_neighbors_[m] && it == winners_agent.end()){
					std::cout << "Enter here " << std::endl;
					if (num_winners_agent < task.num_agents_){
						std::cout << "Case 2 " << std::endl;
						agent.winning_bids_matrix_(task.idx_, m) = neighbor.history_.winning_bids_.back()(task.idx_,m);
						agent.assignment_matrix_(task.idx_, m) = neighbor.history_.assignment_.back()(task.idx_, m);
					}
					else{
						// Find max, mid bid
						std::cout << "case 3 " << std::endl;
						std::pair<int, float> max_bid_pair_ = agent.max_bid_finder(task);
						std::pair<int,float> min_bid_pair_ = agent.min_bid_finder(task);
						std::cout << "max_bid_pair is " << max_bid_pair_.second << std::endl;
						std::cout << "min_bid_pair is " << min_bid_pair_.second << std::endl;
						std::cout << "neighbor.history_.winning_bids_.back()(task.idx_,m) " << neighbor.history_.winning_bids_.back()(task.idx_,m) << std::endl;
						float diff = abs(max_bid_pair_.second - min_bid_pair_.second);

						if(min_bid_pair_.second <= neighbor.history_.winning_bids_.back()(task.idx_,m) && neighbor.history_.winning_bids_.back()(task.idx_,m) <= max_bid_pair_.second){
							if(abs(min_bid_pair_.second - neighbor.history_.winning_bids_.back()(task.idx_,m)) <= abs(max_bid_pair_.second - neighbor.history_.winning_bids_.back()(task.idx_,m))){
								std::cout << "case 4 " << std::endl;
								agent.winning_bids_matrix_(task.idx_, m) = neighbor.history_.winning_bids_.back()(task.idx_,m);
								agent.assignment_matrix_(task.idx_, m) = neighbor.history_.assignment_.back()(task.idx_, m);

								agent.winning_bids_matrix_(task.idx_, max_bid_pair_.first) = 0.0;
								agent.assignment_matrix_(task.idx_, max_bid_pair_.first) = 0;
							}
							else{
								std::cout << "case 5 " << std::endl;
								agent.winning_bids_matrix_(task.idx_, m) = neighbor.history_.winning_bids_.back()(task.idx_,m);
								agent.assignment_matrix_(task.idx_, m) = neighbor.history_.assignment_.back()(task.idx_, m);

								agent.winning_bids_matrix_(task.idx_, min_bid_pair_.first) = 0.0;
								agent.assignment_matrix_(task.idx_, min_bid_pair_.first) = 0;
							}
						}
						else if(neighbor.history_.winning_bids_.back()(task.idx_,m) < min_bid_pair_.second){
							// find second max bid
							std::pair<int, float> sec_max_bid_pair_ = agent.second_max_bid_finder(task);
							std::cout << "sec_max_bid_pair is " << sec_max_bid_pair_.second << "()()(()()())()()()()() " << std::endl;
							if (abs(sec_max_bid_pair_.second - neighbor.history_.winning_bids_.back()(task.idx_,m)) <= diff){
								std::cout << "case 6 " << std::endl;
								agent.winning_bids_matrix_(task.idx_, m) = neighbor.history_.winning_bids_.back()(task.idx_,m);
								agent.assignment_matrix_(task.idx_, m) = neighbor.history_.assignment_.back()(task.idx_, m);

								agent.winning_bids_matrix_(task.idx_, max_bid_pair_.first) = 0.0;
								agent.assignment_matrix_(task.idx_, max_bid_pair_.first) = 0;
							}
						}

						else if(neighbor.history_.winning_bids_.back()(task.idx_, m) > max_bid_pair_.second){
							std::pair<int, float> sec_min_bid_pair_ = agent.second_min_bid_finder(task);
							std::cout << "Second minimum bid is " << sec_min_bid_pair_.second << std::endl;
							if(abs(sec_min_bid_pair_.second - neighbor.history_.winning_bids_.back()(task.idx_,m)) <= diff){
								std::cout << "case 7 " << std::endl;
								agent.winning_bids_matrix_(task.idx_, m) = neighbor.history_.winning_bids_.back()(task.idx_,m);
								agent.assignment_matrix_(task.idx_, m) = neighbor.history_.assignment_.back()(task.idx_, m);

								agent.winning_bids_matrix_(task.idx_, min_bid_pair_.first) = 0.0;
								agent.assignment_matrix_(task.idx_, min_bid_pair_.first) = 0;
							}
						}
						
						

					}
				}
			}

			std::cout << "The result of talking to neighbor " << neighbor.idx_ << " is " << std::endl;
			std::cout << "Winning bids are " << std::endl;
			std::cout << agent.winning_bids_matrix_ << std::endl;
			std::cout << "assignmeny matrix is " << std::endl;
			std::cout << agent.assignment_matrix_ << std::endl;
			std::cout << std::endl;
		}

	
	
		std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ " << std::endl;
	}


	// Update info 
	for (auto &agent: agents){
		for (auto &neighbor_idx: agent.neighbors_){
			cbba_Agent& neighbor = CBBA::FindAgent(agents, neighbor_idx);
			agent.iteration_neighbors_[neighbor.idx_]++;
			for (int i = 0; i < agent.num_agents_; i++){
				if (i != neighbor.idx_ && i != agent.idx_ && agent.iteration_neighbors_[i] < neighbor.history_.iter_neighbors_his.back()[i]){
					agent.iteration_neighbors_[i] = neighbor.history_.iter_neighbors_his.back()[i];
				}
			}

		}
		agent.iteration_neighbors_[agent.idx_]++;
	}


	for (auto &agent: agents){
		agent.history_.iter_neighbors_his.push_back(agent.iteration_neighbors_);
		agent.history_.winning_bids_.push_back(agent.winning_bids_matrix_);
		agent.history_.assignment_.push_back(agent.assignment_matrix_);
	}
	
}

std::vector<int> cbba_Agent::winners_finder(cbba_Task& task){
	std::vector<int> winners_ = {};

	for (int i =0; i < num_agents_; i++){
		if(assignment_matrix_(task.idx_, i) == 1 && winning_bids_matrix_(task.idx_, i) > 0){
			winners_.push_back(i);
		}
	}

	return winners_;
}

std::vector<int> cbba_Agent::his_winners_finder(cbba_Task& task){
	std::vector<int> winners_ = {};
	for(int i = 0; i < num_agents_; i++){
		if(history_.assignment_.back()(task.idx_, i) == 1 && history_.winning_bids_.back()(task.idx_, i) > 0){
			winners_.push_back(i);
		} 
	}

	return winners_;
}

int cbba_Agent::winners_count(cbba_Task& task){
	int num_winners = 0;
	for (int i= 0; i < num_agents_; i++){
		if(assignment_matrix_(task.idx_, i) == 1 ){
			num_winners++;
		}
	}
	return num_winners;
}


std::pair<int, float> cbba_Agent::min_bid_finder(cbba_Task& task){
	std::pair<int, float> min_bid_pair_;
	float min_bid = 1000.0;
	int min_idx = -1;
	for (int i= 0; i < num_agents_; i++){
		if(assignment_matrix_(task.idx_,i) == 1 && winning_bids_matrix_(task.idx_, i) < min_bid){
			min_bid = winning_bids_matrix_(task.idx_, i);
			min_idx = i;
		}
	}

	min_bid_pair_ = std::make_pair(min_idx, min_bid);
	return min_bid_pair_;
}

std::pair<int, float> cbba_Agent::max_bid_finder(cbba_Task& task){
	std::pair<int, float> max_bid_pair_;
	float max_bid = 0.0;
	int max_idx = -1;
	for(int i=0; i < num_agents_; i++){
		if(assignment_matrix_(task.idx_, i) == 1 && winning_bids_matrix_(task.idx_, i) > max_bid){
			max_bid = winning_bids_matrix_(task.idx_, i);
			max_idx = i;
		}
	}

	max_bid_pair_ = std::make_pair(max_idx, max_bid);
	return max_bid_pair_;
} 

std::pair<int, float> cbba_Agent::second_min_bid_finder(cbba_Task& task){
	std::multimap<float, int> bids_pair_ = {};
	std::pair<int, float> sec_bid_pair_;
	for (int i = 0; i < num_agents_; i++){
		if(assignment_matrix_(task.idx_, i) == 1){
			bids_pair_.insert(std::pair<float,int>(winning_bids_matrix_(task.idx_,i),i));
		}
	}

	// std::cout << "Enter the second minimum bid finder function  " << std::endl;
	// for (auto e = bids_pair_.begin(); e != bids_pair_.end(); e++){
	// 	std::cout << (*e).first << " ";
	// }
	// std::cout << std::endl;

	std::map<float, int>::iterator it = bids_pair_.begin();
	std::map<float, int>::iterator it_sec = std::next(it,1);

	sec_bid_pair_ = std::make_pair((*it_sec).second, (*it_sec).first);

	return sec_bid_pair_;
}


std::pair<int, float> cbba_Agent::second_max_bid_finder(cbba_Task& task){
	std::multimap<float, int> bids_pair_ = {};
	std::pair<int, float> sec_bid_pair_;
	for (int i = 0; i < num_agents_; i++){
		if(assignment_matrix_(task.idx_, i) == 1){
			bids_pair_.insert(std::pair<float,int>(winning_bids_matrix_(task.idx_,i),i));
		}
	}

	// std::cout << "WWWWWWW trying to find second max bid " << std::endl;
	// for (auto e = bids_pair_.begin(); e != bids_pair_.end(); e++){
	// 	std::cout << (*e).first << ", ";
	// }
	// std::cout << std::endl;

	std::map<float, int>::iterator it = bids_pair_.end();
	std::map<float, int>::iterator it_prev = std::prev(it, 2);
	
	sec_bid_pair_ = std::make_pair((*it_prev).second, (*it_prev).first);
	
	return sec_bid_pair_;
}

std::map<int, float> cbba_Agent::cross_bid_finder(cbba_Task& task, float new_bid){
	std::map<int,float> cross_bid = {};
	float bid_ = 0.0;
	int idx_ = -1;
	for (int i = 0; i < num_agents_; i++){
		if(assignment_matrix_(task.idx_, i) == 1 && winning_bids_matrix_(task.idx_, i) < new_bid){
			cross_bid[i] = winning_bids_matrix_(task.idx_, i);
		}
	}

	// Find the maxum item
	float max_bid = 0.0;
	int idx_max = -1;
	for (auto &item: cross_bid){
		if (item.second > max_bid){
			max_bid = item.second;
			idx_max = item.first;
		}
	}
	std::map<int, float> cross_bid_opt = {};
	cross_bid_opt[idx_max] = max_bid;

	return cross_bid_opt;
}


void cbba_Agent::assignment_update(std::vector<int> dependent_task_idx){
	for (auto &task_idx: dependent_task_idx){
		if (assignment_matrix_(task_idx, idx_) == 1){
			continue;
		} 
		else{
			cbba_path_.erase(std::remove(cbba_path_.begin(),cbba_path_.end(),task_idx),cbba_path_.end());
			cbba_bundle_.erase(std::remove(cbba_bundle_.begin(), cbba_bundle_.end(), task_idx), cbba_bundle_.end());
		}
	}
}









