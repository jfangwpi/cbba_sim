/* 
 * cbga_agent.cpp
 *
 * Created on Aug 29, 2018
 *     Author: jfang
 *
 */


#include "cbga/cbga_agent.hpp"
#include "config_reader/config_reader.hpp"

using namespace librav;

cbga_Agent::cbga_Agent(int id, int init_pos, std::vector<int> com, int num_tasks):
idx_(id),
init_pos_(init_pos),
comm_topo_(com),
iter_(0){
    cbga_bundle_ = {};
    cbga_path_ = {};

    iteration_neighbors_ = std::vector<int>(com.size(), 0);
    history_.iter_neighbors_his = {iteration_neighbors_};

    cbga_reward_ = std::vector<double>(num_tasks, -1.0);

    num_agents_ = com.size();
    num_tasks_ = num_tasks;
    max_tasks_ = num_tasks;

    insert_pos_ = std::vector<int>(num_tasks, -1);

    assignment_matrix_ = Eigen::MatrixXd::Zero(num_tasks_, num_agents_);
    winning_bids_matrix_ = -1 * Eigen::MatrixXd::Zero(num_tasks_, num_agents_);

    history_.assignment_ = {assignment_matrix_};
    history_.winning_bids_ = {winning_bids_matrix_};
    
}

// Return agent based on the idx
cbga_Agent& CBGA::FindAgent(std::vector<cbga_Agent>& agents, int idx){

    for (auto &agent: agents){
        if(agent.idx_ == idx){
            return agent;
        }
    }
}

// Read Agents from config file
std::vector<cbga_Agent> CBGA::InitializeAgents(){

	ConfigReader config_reader("../../src/config/agent.ini");
    if (config_reader.CheckError()){
        std::cout << "Reading config file failed." << std::endl;
    }

    int32_t num_agents = config_reader.GetReal("num_agents", 0);
	std::vector<cbga_Agent> agents;

	for (int i= 0; i < num_agents; i++){
		std::string agent_name = "agent"+ std::to_string(i);
		std::string agent_idx = agent_name + "_idx";
		std::string agent_init_pos = agent_name + "_init";
		std::string agent_neighbors = agent_name + "_neighbors";

		// Read config from agent.ini
		int idx = i;
		int init_pos = config_reader.GetReal(agent_init_pos, 0);
		std::vector<int> comm = config_reader.GetVectorInt(agent_neighbors,{});
		int num_tasks = config_reader.GetReal("num_tasks", 0);

		// Initialize the agent
		agents.push_back(cbga_Agent(idx, init_pos, comm, num_tasks));
	}

	return agents;
    
}

// Update the bids for vehicle i
void cbga_Agent::bids_update(CBGATasks tasks_group, const std::shared_ptr<Graph_t<SquareCell*>> graph){
    std::cout << "Before bids_update, the bundle info is " << std::endl;
    for(auto &e: cbga_bundle_){
        std::cout << e << " ";
    }
    std::cout << std::endl;
    // Find all tasks which have not been inserted into t he bundle (path)
    std::vector<int> tasks_NotInBundle = {};
    for (int task_idx = 0; task_idx < num_tasks_; task_idx++){
        std::vector<int>::iterator it = std::find(cbga_bundle_.begin(), cbga_bundle_.end(), task_idx);
        if(it == cbga_bundle_.end()){
            tasks_NotInBundle.push_back(task_idx);
        }
    }

    for (auto &task_idx: tasks_NotInBundle){
        cbga_Task task = tasks_group.FindTaskFromID(task_idx);
        /*** Indenpendent tasks ***/
        /*** Find possible insert positions ***/
        std::vector<std::vector<int>> path_copy(cbga_path_.size()+1, cbga_path_);
        for (int m = 0; m < path_copy.size(); m++){
            std::vector<int>::iterator it = path_copy[m].begin();
            path_copy[m].insert(it+m,task_idx);
        }

        /*** Compute the bid for each insert case ***/
        std::vector<double> bids(path_copy.size(), 0.0);
        for (int i = 0; i < path_copy.size(); i++){
            std::cout << "currently the path_copy is " << std::endl;
            for (auto &e: path_copy[i]){
                std::cout << e << " ";
            }
            std::cout << std::endl;
            // bids[i] = CBGA::ScoreCalculation(tasks_group, graph, path_copy[i], this) - origin_score;
            // bids[i] = CBGA::ScoreCalculation(tasks_group, graph, path_copy[i], this);
            bids[i] = reward_benefit - CBGA::PathLengthCalculationBasedType(tasks_group, graph, path_copy[i], init_pos_);   
        }

        /*** Find the maximum bids ***/
        /*** Update the best insert pos for the task ***/
        double bid_max = 0.0;
        for (int i= 0; i < bids.size(); i++){
            if(bids[i] > bid_max){
                bid_max = bids[i];
                insert_pos_[task_idx] = i;
            }
        }

        cbga_reward_[task_idx] = bid_max;
   
    }

}


void cbga_Agent::bids_update(LTLFormula Global_LTL, std::shared_ptr<Graph_t<SquareCell*>> graph,
    std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph, TileTraversalData tile_traversal_data, 
    std::shared_ptr<SquareGrid> grid){
    std::cout << "Before bids update, the bundle info is " << std::endl;
    for (auto &e: cbga_bundle_){
        std::cout << e << " ";
    }
    std::cout << std::endl;
    
    // Find all tasks whcih have not been inserted int the bundle 
    std::vector<int> tasks_NotInBundle = {};
    for(int task_idx = 0; task_idx < num_tasks_; task_idx++){
        std::vector<int>::iterator it = std::find(cbga_bundle_.begin(), cbga_bundle_.end(), task_idx);
        if (it == cbga_bundle_.end()){
            tasks_NotInBundle.push_back(task_idx);
        }
    }

    for (auto &task_idx: tasks_NotInBundle){
        // Find all possible insert positions 
        std::vector<std::vector<int>> path_copy(cbga_path_.size()+1, cbga_path_);
        for (int m = 0; m < path_copy.size(); m++){
            std::vector<int>::iterator it = path_copy[m].begin();
            path_copy[m].insert(it+m,task_idx);
        }

        // Compute the bid for each insert case
        std::vector<double> bids(path_copy.size(), 0.0);
        for(int i= 0; i < path_copy.size(); i++){
            std::cout << "currently the path copy is " << std::endl;
            for (auto &e: path_copy[i]){
                std::cout << e << " ";
            }
            std::cout << std::endl;

            bids[i] = cbga_benefit - CBGA::ScoreCalculation(Global_LTL, graph, path_copy[i], this->init_pos_, lifted_graph, tile_traversal_data, grid);
        }

        // Find the maximum bids 
        double bid_max = 0.0;
        for (int i = 0; i < bids.size(); i++){
            if(bids[i] > bid_max){
                bid_max = bids[i];
                insert_pos_[task_idx] = i;
            }
        }

        cbga_reward_[task_idx] = bid_max;
    }


}


double CBGA::ScoreCalculation(LTLFormula Global_LTL, std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<int> bundle, 
    int start_idx_, std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph, TileTraversalData tile_traversal_data, 
    std::shared_ptr<SquareGrid> grid){
        std::cout << "========================================= DEBUG FOR CBTA ================================ " << std::endl;
        std::shared_ptr<Graph_t<ProductState *>> product_graph_new = std::make_shared<Graph_t<ProductState *>>();
        std::string ltl_formula = LTLDecomposition::subtask_recreator(bundle, true, Global_LTL);
        std::vector<std::vector<std::string>> buchi_regions = LTLDecomposition::ObtainBuchiRegion({ltl_formula});

        // Build buchi graph
        std::shared_ptr<Graph_t<BuchiState *, std::vector<int64_t>>> buchi_graph = BuchiAutomaton::BuildBuchiGraph(ltl_formula, buchi_regions.front());
        std::vector<int64_t> buchi_acc = (*buchi_graph->FindVertex(0)).state_->acc_state_idx;

        Vertex_t<SquareCell *>* start_node_origin = graph ->GetVertexFromID(start_idx_);

        std::vector<double> zta0 = {0.45, 0.0};
        std::vector<int> rgn_idx_init = {HCost::zta02rgn_idx(zta0, tile_traversal_data.region_bd)};

        int64_t virtual_start_id = ProductAutomaton::SetVirtualStartIncre(start_node_origin, rgn_idx_init, lifted_graph, buchi_graph, product_graph_new);
        GetProductCBTANeighbour get_product_cbta_neighbour(lifted_graph, buchi_graph, tile_traversal_data, grid);
        auto path = AStar::ProductIncSearchCBTA(product_graph_new, virtual_start_id, buchi_acc, GetNeighbourFunc_t<ProductState*, double>(get_product_cbta_neighbour));

        float path_CBTA_length;
        if(!path.empty()){
            std::vector<Vertex_t<SquareCell*>*> path_origin;
            for (auto cell = path.begin()+1; cell != path.end(); cell++){
                path_origin.push_back((*cell)->lifted_vertex_->state_->history.front());
            }

            path_origin.insert(path_origin.end(), path.back()->lifted_vertex_->state_->history.begin()+1, path.back()->lifted_vertex_->state_->history.end());
            Path_t<SquareCell *> path_vis;
            for (auto &ee: path_origin){
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






/*** Find the neighbrs ***/
void cbga_Agent::neighbors_finder(){
    neighbors_.clear();
    for (int i= 0; i < comm_topo_.size(); i++){
        if(i != idx_ && comm_topo_[i] == 1){
            neighbors_.push_back(i);
        }
    }
}

/*** Given certain bundle (path), compute the score which is the sum of cost for each task in the bundle ***/
/*** The cost of each task j equals to r_j which is some constant value minus the distance required by vehicle i travels to task j ***/
double CBGA::ScoreCalculation(CBGATasks tasks,std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<int> bundle ,cbga_Agent* agent){
    double score = 0.0;

    // for (int i=0; i < bundle.size(); i++){
    //     std::vector<int>::iterator first_ = bundle.begin();
    //     std::vector<int>::iterator last_ = bundle.begin() + i +1;
    //     std::vector<int> bundle(first_, last_);

    /*** Compute the distance for each task i in the bundle and compute the sum of them which is the score with given bundle ***/
    std::cout << "The start id is " << (*agent).idx_ << std::endl;
    score = reward_benefit - CBGA::PathLengthCalculationBasedType(tasks, graph, bundle, (*agent).idx_);
    std::cout << "score is " << score << std::endl;
    // }

    return score;
}

double CBGA::PathLengthCalculation(CBGATasks tasks, std::shared_ptr<Graph_t<SquareCell*>> graph,std::vector<int> bundle,cbga_Agent* agent){
    Path_t<SquareCell *> path_part;
	Path_t<SquareCell *> path;
	Vertex_t<SquareCell*> * start_node;
	Vertex_t<SquareCell*> * finish_node;

	// Initialize the start_node which should be initial position of agent
	start_node = graph->GetVertexFromID(agent->init_pos_);
	for (int i = 0; i < bundle.size(); i++){
        auto task = tasks.FindTaskFromID(bundle[i]);
        // if(task.num_agents_ == 1){
        for (int j = 0; j < task.pos_.size(); j++){
            finish_node = graph->GetVertexFromID(task.pos_[j]);
            path_part = AStar::Search(graph,start_node->state_->id_,finish_node->state_->id_, CalcHeuristicFunc_t<SquareCell *>(CBGA::CalcHeuristic));
            if (i == bundle.size()-1 && j == task.pos_.size()-1){
                path.insert(path.end(), path_part.begin(), path_part.end());
            }
            else{
                path.insert(path.end(), path_part.begin(), path_part.end()-1);
            }
            start_node = finish_node;
        }
        // }

        // else{
        //     for (int j = 0; j < task.pos_.size(); j++){
        //         finish_node = graph->GetVertexFromID(task.pos_[j]);
        //         path_part = AStar::Search(graph,start_node->state_->id_,finish_node->state_->id_, CalcHeuristicFunc_t<SquareCell *>(CBGA::CalcHeuristic));
                
        //         if (i == bundle.size()-1 && j == task.pos_.size()-1){
        //             path.insert(path.end(), path_part.begin(), path_part.end());
        //         }
        //         else{
        //             path.insert(path.end(), path_part.begin(), path_part.end()-1);
        //         }

        //         start_node = finish_node;

                // Compare
                // double length_max = path.size();
                // for (int k = 0; k < agent->num_agents_; k++){
                //     if(agent->assignment_matrix_(task.idx_, k) == 1 && reward_benefit - agent->winning_bids_matrix_(task.idx_, k) > length_max && k != agent->idx_){
                //         length_max = reward_benefit - agent->winning_bids_matrix_(task.idx_, k);
                //     }
                // }
                
                // if(agent->min_bid_ != 0.0){
                //     double t_wait = reward_benefit - agent->min_bid_ - path.size();
                //     std::cout << "t_wait is " << t_wait << std::endl;
                //     for (int m = 0; m < t_wait; m++){
                //         path.push_back(path_part.back());
                //     }
                // }
                
                    
            // }

        // }
            
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

double CBGA::PathLengthCalculationBasedType(CBGATasks tasks, std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<int> bundle, int start_idx_){
	Path_t<SquareCell *> path_part;
	Path_t<SquareCell *> path;
	Vertex_t<SquareCell*> * start_node;
	Vertex_t<SquareCell*> * finish_node;
	int final_pos;
	bool flag = false;

	std::vector<int> pos_seq;
	start_node = graph->GetVertexFromID(start_idx_);
    if (bundle.empty()){
        return 0.0;
    }
  
	for (int i = 0; i < bundle.size(); i++){
		cbga_Task task = tasks.FindTaskFromID(bundle[i]);
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
				std::cout << "Target not found " << std::endl;
				pos_seq.push_back(task.pos_[0]);
			}
			else{
				std::cout << "Target found " << std::endl;
				flag = true;
				final_pos = task.pos_.back();
				pos_seq.push_back(task.pos_[0]);
			}
        }
	}
	
	if(flag == true){
		pos_seq.push_back(final_pos);
	}

   
	start_node = graph->GetVertexFromID(start_idx_);
     std::cout << "The start node is " << start_node->state_->id_ << std::endl;
	for (int k = 0; k < pos_seq.size(); k++){
		finish_node = graph->GetVertexFromID(pos_seq[k]);
		path_part = AStar::Search(graph,start_node->state_->id_,finish_node->state_->id_, CalcHeuristicFunc_t<SquareCell *>(CBGA::CalcHeuristic));
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



double CBGA::CalcHeuristic(SquareCell *node1, SquareCell *node2)
{
    int32_t dist_row = node1->coordinate_.x - node2->coordinate_.x;
    int32_t dist_col = node1->coordinate_.y - node2->coordinate_.y;

    return std::sqrt(dist_row * dist_row + dist_col * dist_col);
}

void CBGA::available_tasks_finder(CBGATasks tasks, cbga_Agent& agent){
    agent.h_avai_.clear();
    
    for (auto &task: tasks.tasks_){
        if(task.num_agents_ == 1){
            bool condition_1 = 0;
            bool condition_2 = 0;
            
            int winner_idx = -1;
            for (int i = 0; i < agent.num_agents_; i++){
                if(agent.assignment_matrix_(task.idx_,i) == 1)
                    winner_idx = i;
                    break;
            }
            
            if(agent.cbga_reward_[task.idx_] - agent.winning_bids_matrix_(task.idx_, agent.idx_) > eps){
                condition_1 = 1;
            }
            else if(abs(agent.cbga_reward_[task.idx_] - agent.winning_bids_matrix_(task.idx_, agent.idx_)) <= eps){
                if (agent.idx_ < winner_idx){
                    condition_2 = 1;
                }
            }

            if(condition_1 == 1 || condition_2 ==1){
                agent.h_avai_.push_back(task);
            }
        }
        else{
            int num_winners = 0;
            std::vector<double> winner_bids = {};
            std::vector<int> winners = {};
            for (int i = 0; i < agent.num_agents_; i++){
                if(agent.assignment_matrix_(task.idx_, i) == 1){
                    num_winners += 1;
                    winners.push_back(i);
                    winner_bids.push_back(agent.winning_bids_matrix_(task.idx_, i));
                }
            }

            std::vector<int>::iterator it = std::find(winners.begin(), winners.end(), agent.idx_);
            bool exist = 0;
            if(it != winners.end()){
                exist = 1;
            }
            double bid_min;
            if (num_winners == 0){
                bid_min = 0.0;
            }
            else{
                bid_min = 1000.0;
                for (auto &b: winner_bids){
                    if(b < bid_min){
                        bid_min = b;
                    }
                }
            }

            if(num_winners < task.num_agents_ && exist==0){
                agent.h_avai_.push_back(task);
            }
            else if(agent.cbga_reward_[task.idx_] > bid_min && exist==0){
                agent.h_avai_.push_back(task);
            }
        }
    }

    // std::cout << "skr skr skr skr skr~~~~~~~~~ " << std::endl;
    // std::cout << "The available task for vehicle " << agent.idx_ << std::endl;
    // for (auto &e: agent.h_avai_){
    //     std::cout << e.idx_ << " ";
    // }
    // std::cout << std::endl;
    
}


int CBGA::desired_task_finder(CBGATasks tasks, cbga_Agent& agent){
    double bid_max = 0.0;
    int desired_task_idx = -1;

    CBGA::available_tasks_finder(tasks, agent);
    if (!agent.h_avai_.empty()){
        for (auto &task: agent.h_avai_){
            if(agent.cbga_reward_[task.idx_] > bid_max){
                bid_max = agent.cbga_reward_[task.idx_];
                desired_task_idx = task.idx_;
            }
        }
    }
    return desired_task_idx;
}

void CBGA::bundle_remove(cbga_Agent& agent){
    
    bool outbid = 0;
    for (int idx = 0; idx < agent.cbga_bundle_.size(); idx++){
        if(agent.assignment_matrix_(agent.cbga_bundle_[idx], agent.idx_) != 1){
            outbid = 1;
        }

        if(outbid == 1){
            if(agent.assignment_matrix_(agent.cbga_bundle_[idx], agent.idx_) == 1){
                agent.winning_bids_matrix_(agent.cbga_bundle_[idx], agent.idx_) = -1;
                agent.assignment_matrix_(agent.cbga_bundle_[idx], agent.idx_) = -1;
            }
            
            // agent.winning_bids_matrix_(agent.cbga_bundle_[idx], idx) = -1;
            // agent.assignment_matrix_(agent.cbga_bundle_[idx], idx) = -1;
            agent.cbga_bundle_[idx] = -1;
        }
    }

    agent.cbga_bundle_.erase(std::remove(agent.cbga_bundle_.begin(), agent.cbga_bundle_.end(), -1), agent.cbga_bundle_.end());


    CBGA::path_remove(agent);
    
}

void CBGA::bundle_remove(std::vector<cbga_Agent>& agents){
    for (auto &agent: agents){
        CBGA::bundle_remove(agent);
    }
}


void CBGA::path_remove(cbga_Agent& agent){
    std::vector<int> path_copy = agent.cbga_path_;
    for (int i = 0; i < path_copy.size(); i++){
        std::vector<int>::iterator it = std::find(agent.cbga_bundle_.begin(), agent.cbga_bundle_.end(), path_copy[i]);

        if (it == agent.cbga_bundle_.end()){
            agent.cbga_path_.erase(std::remove(agent.cbga_path_.begin(), agent.cbga_path_.end(), path_copy[i]), agent.cbga_path_.end());
        }
    }
}

void CBGA::path_remove(std::vector<cbga_Agent>& agents){
    for (auto &agent:agents){
        CBGA::path_remove(agent);
    }
}


void CBGA::bundle_construction(CBGATasks tasks, std::shared_ptr<Graph_t<SquareCell*>> graph, cbga_Agent& agent){
    std::cout << "==========================================================================================" << std::endl;
    std::cout << "bundle constructure " << std::endl;
    bool bundle_full = 0;
    if(agent.cbga_bundle_.size() >= agent.max_tasks_){
        bundle_full = 1;
    }

    while(bundle_full == 0){
        // Updates bids based on current bundle and path info
        agent.bids_update(tasks, graph);
        int desired_task_idx = CBGA::desired_task_finder(tasks, agent);
        
        //std::cout << "Based on the task reward for agent " << agent.idx_ << std::endl;
        //for(auto &e: agent.cbga_reward_){
            //std::cout << e << " ";
        //}
        //std::cout << std::endl;
        //std::cout << "The desired idx is " << desired_task_idx << std::endl;

        if(desired_task_idx == -1){
            break;
        }

        // Update the assignment
        cbga_Task task = tasks.FindTaskFromID(desired_task_idx);
        agent.winning_bids_matrix_(desired_task_idx, agent.idx_) = agent.cbga_reward_[desired_task_idx];
        if (task.num_agents_ == 1){
            for (int idx = 0; idx < agent.num_agents_; idx++){
                if (agent.assignment_matrix_(desired_task_idx, idx) == 1){
                    agent.assignment_matrix_(desired_task_idx, idx) = 0;
                }
            }
            agent.assignment_matrix_(desired_task_idx, agent.idx_) = 1;
        }
        else{
            std::vector<int> winners_agent = {};
            std::vector<double> bids_agent = {};
            int bid_min = 1000.0;
            int idx_min = -1;
            for (int idx = 0; idx < agent.num_agents_; idx++){
                if (agent.assignment_matrix_(desired_task_idx, idx) == 1){
                    winners_agent.push_back(idx);
                    bids_agent.push_back(agent.winning_bids_matrix_(desired_task_idx, idx));
                }
            }
            for (int i=0; i < winners_agent.size(); i++){
                if(bids_agent[i] < bid_min){
                    bid_min = bids_agent[i];
                    idx_min = winners_agent[i];
                }
            }
            if (winners_agent.size() >= task.num_agents_){
                if (agent.cbga_reward_[desired_task_idx] > bid_min){
                    agent.assignment_matrix_(desired_task_idx, idx_min) = 0;
                    agent.assignment_matrix_(desired_task_idx, agent.idx_) = 1;
                }
            }
            else{
                agent.assignment_matrix_(desired_task_idx, agent.idx_) = 1;
            }
            
        }
        
        // Update the bundle and path
        std::vector<int>::iterator it = agent.cbga_path_.begin();
        int insert_pos = agent.insert_pos_[task.idx_];
        agent.cbga_path_.insert(it + insert_pos, desired_task_idx);

        //std::cout << "Best insert position is " << insert_pos << std::endl;
        

        agent.cbga_bundle_.push_back(desired_task_idx);

        std::cout << "Now the bundle becomes " << std::endl;
        for (auto &e: agent.cbga_bundle_){
            std::cout << e << " ";
        }
        std::cout << std::endl;

        std::cout << "Now the path becomes " << std::endl;
        for (auto &e: agent.cbga_path_){
            std::cout << e << " ";
        }
        std::cout << std::endl;


        if(agent.cbga_bundle_.size() > agent.max_tasks_){
            bundle_full = 1;
        }
    }

    // Update the reward based on the latest bundle and path info
    // CBGA::update_rewards(tasks, agent, graph);


    agent.history_.winning_bids_.push_back(agent.winning_bids_matrix_);
    agent.history_.assignment_.push_back(agent.assignment_matrix_);

    std::cout << "==========================================================================================" << std::endl;
}

void CBGA::bundle_construction(CBGATasks tasks, std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<cbga_Agent>& agents){
    for (auto &agent: agents){
        CBGA::bundle_construction(tasks, graph, agent);
    }
    
}

void CBGA::bundle_construction(CBGATasks tasks, LTLFormula Global_LTL,const std::shared_ptr<Graph_t<SquareCell*>> graph,
		std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph, TileTraversalData tile_traversal_data, 
        std::shared_ptr<SquareGrid> grid, std::vector<cbga_Agent>& agents){
            for(auto &agent: agents){
                CBGA::bundle_construction(tasks, Global_LTL, graph, lifted_graph, tile_traversal_data, grid, agent);
            }
        }


void CBGA::bundle_construction(CBGATasks tasks, LTLFormula Global_LTL,const std::shared_ptr<Graph_t<SquareCell*>> graph,
		std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph, TileTraversalData tile_traversal_data, 
        std::shared_ptr<SquareGrid> grid, cbga_Agent& agent){
            std::cout << "=================================================================================== " << std::endl;
            std::cout << "bundle construction " << std::endl;
            bool bundle_full = 0;
            if (agent.cbga_bundle_.size() >= agent.max_tasks_){
                bundle_full = 1;
            }
            while (bundle_full == 0){
                agent.bids_update(Global_LTL,graph,lifted_graph,tile_traversal_data,grid);
                int desired_task_idx = CBGA::desired_task_finder(tasks, agent);
                if (desired_task_idx == -1){
                    break;
                }

                cbga_Task task = tasks.FindTaskFromID(desired_task_idx);
                agent.winning_bids_matrix_(desired_task_idx, agent.idx_) = agent.cbga_reward_[desired_task_idx];
                if (task.num_agents_ == 1){
                    for (int idx = 0; idx < agent.num_agents_; idx++){
                        if (agent.assignment_matrix_(desired_task_idx, idx) == 1){
                            agent.assignment_matrix_(desired_task_idx, idx) = 0;
                        }
                    }
                    agent.assignment_matrix_(desired_task_idx, agent.idx_) = 1;
                }
                else{
                    std::vector<int> winners_agent = {};
                    std::vector<double> bids_agent = {};

                    double bid_min = 1000.0;
                    int idx_min = -1;
                    for (int idx = 0; idx < agent.num_agents_; idx++){
                       if (agent.assignment_matrix_(desired_task_idx, idx) == 1){
                           winners_agent.push_back(idx);
                           bids_agent.push_back(agent.winning_bids_matrix_(desired_task_idx, idx));
                       }
                    }

                    for (int i = 0; i < winners_agent.size(); i++){
                        if(bids_agent[i] < bid_min){
                            bid_min = bids_agent[i];
                            idx_min = winners_agent[i];
                        }
                    }
                    if (winners_agent.size() >= task.num_agents_){
                        if (agent.cbga_reward_[desired_task_idx] > bid_min){
                            agent.assignment_matrix_(desired_task_idx, idx_min) = 0;
                            agent.assignment_matrix_(desired_task_idx, agent.idx_) = 1;
                        }
                    }
                    else{
                        agent.assignment_matrix_(desired_task_idx, agent.idx_) = 1;
                    }
                }

                // Update the bundle and path
                std::vector<int>::iterator it = agent.cbga_path_.begin();
                int insert_pos = agent.insert_pos_[task.idx_];
                agent.cbga_path_.insert(it + insert_pos, desired_task_idx);

                agent.cbga_bundle_.push_back(desired_task_idx);

                std::cout << "Now the bundle becomes " << std::endl;
                for (auto &e: agent.cbga_bundle_){
                    std::cout << e << " ";
                }
                std::cout << std::endl;

                std::cout << "Now the path becomes " << std::endl;
                for (auto &e: agent.cbga_path_){
                    std::cout << e << " ";
                }
                std::cout << std::endl;

                if (agent.cbga_bundle_.size() > agent.max_tasks_){
                    bundle_full = 1;
                }

            }

            agent.history_.winning_bids_.push_back(agent.winning_bids_matrix_);
            agent.history_.assignment_.push_back(agent.assignment_matrix_);

            std::cout << "==========================================================================================" << std::endl;

}


// void CBGA::update_rewards(CBGATasks tasks_group, cbga_Agent& agent,  std::shared_ptr<Graph_t<SquareCell*>> graph){
//     for(int task_idx = 0; task_idx < agent.cbga_path_.size(); task_idx++){
//         cbga_Task task = tasks_group.FindTaskFromID(agent.cbga_path_[task_idx]);
//         if(task.num_agents_ > 1){
//             std::vector<int>::iterator first_ = agent.cbga_path_.begin();
//             std::vector<int>::iterator last_ = agent.cbga_path_.begin() + task_idx + 1;
//             std::vector<int> path_part(first_, last_);
//             agent.cbga_reward_[agent.cbga_path_[task_idx]] = reward_benefit - CBGA::PathLengthCalculation(tasks_group, graph, path_part, &agent);
//         }
        
//         agent.winning_bids_matrix_(agent.cbga_path_[task_idx], agent.idx_) = agent.cbga_reward_[agent.cbga_path_[task_idx]];
//         agent.assignment_matrix_(agent.cbga_path_[task_idx], agent.idx_) = 1;
//     }

//     std::cout << "Once the path is decided " << std::endl;
//     std::cout << "For vehicle " << agent.idx_ << std::endl;
//     std::cout << "The new reward for each task is " << std::endl;
//     for (auto &e: agent.cbga_reward_){
//         std::cout << e << " ";
//     }
//     std::cout << std::endl;
//     std::cout << "The winning bids matrix is " << std::endl;
//     std::cout << agent.winning_bids_matrix_ << std::endl;
//     std::cout << "The assignment matrix is " << std::endl;
//     std::cout << agent.assignment_matrix_ << std::endl;
// }

bool CBGA::success_checker(std::vector<cbga_Agent>& agents, CBGATasks tasks){
    bool success = true;
    for (auto &task: tasks.tasks_){
        std::vector<int> winners = {};
        std::vector<double> bids = {};
        for (auto &agent: agents){
            if (winners.empty() && bids.empty()){
                winners = agent.find_winners(task);
                bids = agent.find_winner_bids(task);
            }
            else{
                std::vector<int> new_winners = agent.find_winners(task);
                std::vector<double> new_bids = agent.find_winner_bids(task);
                
                if (new_winners.size() != winners.size() || new_bids.size() != bids.size()){
                    success = false;
                }
                else{
                    for (int i = 0; i < winners.size(); i++){
                        if (new_winners[i] != winners[i]){
                            std::cout << "new_winners " << new_winners[i] << std::endl;
                            std::cout << "winners " << winners[i] << std::endl;
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
    return success;
}

std::vector<int> cbga_Agent::find_winners(cbga_Task& task){
    std::vector<int> winners = {};
    for (int idx = 0; idx < num_agents_; idx++){
        if (assignment_matrix_(task.idx_, idx) == 1){
            winners.push_back(idx);
        }
    }

    return winners;
}


std::vector<double> cbga_Agent::find_winner_bids(cbga_Task& task){
    std::vector<double> bids = {};
    if (task.num_agents_ == 1){
        bids.push_back(winning_bids_matrix_(task.idx_, idx_));
        return bids;
    }
    else{
        for (int idx = 0; idx < num_agents_; idx++){
            if (assignment_matrix_(task.idx_, idx) == 1){
                bids.push_back(winning_bids_matrix_(task.idx_, idx));
            }
        }
        return bids;
    }
}



void CBGA::communicate(std::vector<cbga_Agent>& agents, cbga_Task task){
    // Update the winning bids and assignment based on the table from CBBA
    for (auto &agent: agents){
        // Update the neighbor
        agent.neighbors_finder();
        
        for (int i = 0; i < agent.neighbors_.size(); i++){
            cbga_Agent& neighbor = CBGA::FindAgent(agents, agent.neighbors_[i]);
            // neighbor.iteration_neighbors_[agent.idx_] = agent.iter_;
            // std::cout << "neighbor " << neighbor.idx_ << std::endl;
            

            // Find the winner of the task that agent , neighbor thinks
            int winner_idx_neighbor = -1;
            int winner_idx_agent = -1;
            for (int m = 0 ; m < agent.num_agents_ ; m++){
                if(neighbor.assignment_matrix_(task.idx_, m) == 1){
                    winner_idx_neighbor = m;
                }

                if(agent.history_.assignment_.back()(task.idx_, m) == 1){
                    winner_idx_agent = m;
                }
            }

            std::cout << "Before communicate " << std::endl;
            std::cout << "Agent info is " << std::endl;
            std::cout << agent.history_.winning_bids_.back() << std::endl;
            std::cout << agent.history_.assignment_.back() << std::endl;
            std::cout << "neighbor info is " << std::endl;
            std::cout << neighbor.winning_bids_matrix_ << std::endl;
            std::cout << neighbor.assignment_matrix_ << std::endl;

            // Entried 1 to 4
            // If current agent k thinks that the winner of task j is itself k
            if(agent.history_.assignment_.back()(task.idx_, agent.idx_) == 1){

                /************************* Entry 1 ********************************/
                // If the receiver (neighbor) i thinks the winner of task j is also itself i
                if(neighbor.assignment_matrix_(task.idx_,neighbor.idx_) == 1){
                    // Update 
                    if(agent.history_.winning_bids_.back()(task.idx_, agent.idx_) - neighbor.winning_bids_matrix_(task.idx_, neighbor.idx_) > eps){
                        std::cout << "case 1 " << std::endl;
                        neighbor.winning_bids_matrix_(task.idx_, neighbor.idx_) = agent.history_.winning_bids_.back()(task.idx_, agent.idx_);
                        neighbor.assignment_matrix_(task.idx_, agent.idx_) = 1;
                        neighbor.assignment_matrix_(task.idx_, neighbor.idx_) = 0;
                    }
                    // Equal score: require to break the tire
                    else if(abs(agent.history_.winning_bids_.back()(task.idx_, agent.idx_) - neighbor.winning_bids_matrix_(task.idx_, neighbor.idx_)) <= eps){
                        // Select the winner of task j as the agent with smaller index
                        if(neighbor.idx_ > agent.idx_){
                            std::cout << "case 2 " << std::endl;
                            neighbor.winning_bids_matrix_(task.idx_, neighbor.idx_) = agent.history_.winning_bids_.back()(task.idx_, agent.idx_);
                            neighbor.assignment_matrix_(task.idx_, agent.idx_) = 1;
                            neighbor.assignment_matrix_(task.idx_, neighbor.idx_) = 0;
                        }
                    }
                }

                /***************************************** Entry 2 ***************************************/
				// Entry 2: Update
				// If the receiver i thinks the winner of task j is also agent k (sender)
                // Update
                else if(neighbor.assignment_matrix_(task.idx_, agent.idx_) == 1){
                    std::cout << "case 3 " << std::endl;
                    neighbor.winning_bids_matrix_(task.idx_, neighbor.idx_) = agent.history_.winning_bids_.back()(task.idx_, agent.idx_);
                    neighbor.assignment_matrix_(task.idx_, agent.idx_) = 1;
                    neighbor.assignment_matrix_(task.idx_, neighbor.idx_) = 0;
                }

                /***************************************** Entry 3 ***************************************/
                // Entry 3: Update or Leave
                // If the receiver i thinks the winner of task j is not k or itself i but other agent
                else if(winner_idx_neighbor >= 0){
                    // Compare the iteration of communicate with neighbor j, and figure it out who has the latest information of task j
                    // Update (if agent has the latest info)
                    if(agent.history_.iter_neighbors_his.back()[winner_idx_neighbor] > neighbor.iteration_neighbors_[winner_idx_neighbor]){
                        std::cout << "case 4 " << std::endl;
                        neighbor.winning_bids_matrix_(task.idx_, neighbor.idx_) = agent.history_.winning_bids_.back()(task.idx_, agent.idx_);
                        neighbor.assignment_matrix_(task.idx_, agent.idx_) = 1;
                        neighbor.assignment_matrix_(task.idx_, winner_idx_neighbor) = 0;
                    }

                    // Update 
                    else if(agent.history_.winning_bids_.back()(task.idx_, agent.idx_) - neighbor.winning_bids_matrix_(task.idx_, neighbor.idx_) > eps){
                        std::cout << "case 5 " << std::endl;
                        neighbor.winning_bids_matrix_(task.idx_, neighbor.idx_) = agent.history_.winning_bids_.back()(task.idx_, agent.idx_);
                        neighbor.assignment_matrix_(task.idx_, agent.idx_) = 1;
                        neighbor.assignment_matrix_(task.idx_, winner_idx_neighbor) = 0;
                    }

                    // Equal scores: break the tie by selecting the winner with xmaller idx
                    else if(abs(agent.history_.winning_bids_.back()(task.idx_, agent.idx_) - neighbor.winning_bids_matrix_(task.idx_, neighbor.idx_)) <= eps){
                        if(winner_idx_neighbor > agent.idx_){
                            std::cout << "case 6 " << std::endl;
                            neighbor.winning_bids_matrix_(task.idx_, neighbor.idx_) = agent.history_.winning_bids_.back()(task.idx_, agent.idx_);
                            neighbor.assignment_matrix_(task.idx_, agent.idx_) = 1;
                            neighbor.assignment_matrix_(task.idx_, winner_idx_neighbor) = 0;
                        }
                    }

                }

                // Entry 4: Update
                // If the agent i (neighbor) has no idea about the winner
                else if(winner_idx_neighbor == -1){
                    std::cout << "case 7 " << std::endl;
                    neighbor.winning_bids_matrix_(task.idx_, neighbor.idx_) = agent.history_.winning_bids_.back()(task.idx_, agent.idx_);
                    neighbor.assignment_matrix_(task.idx_, agent.idx_) = 1;
                            
                }

                else{
                    std::cout << "Unknown case 1 " << std::endl;
                }
            }
            
            /*********************************************************************************************************/
			/*********************************************************************************************************/
			/*********************************************************************************************************/
            // Entries 5 to 8
            // If current agent thinks the winner of task j is neighbor
            else if(agent.history_.assignment_.back()(task.idx_, neighbor.idx_) == 1){

                // Entry 5
                // neighbor thinks that the winner of task j is itself
                // Leave 
                if(neighbor.assignment_matrix_(task.idx_, neighbor.idx_) == 1){
                    std::cout << "Do nothing Entry 5 " << std::endl;
                } 

                // Entry 6
                // If neighbor thinks the winner if task j is agent 
                // reset 
                else if(neighbor.assignment_matrix_(task.idx_, agent.idx_) == 1){
                    std::cout << "case 8 " << std::endl;
                    neighbor.winning_bids_matrix_(task.idx_, neighbor.idx_) = -1;
                    neighbor.assignment_matrix_(task.idx_, agent.idx_) = 0;
                }


                // Entry 7
                else if(winner_idx_neighbor >= 0){
                    // Compare the interation between agent and winner_idx_neighbor, neighbor and winner_idx_neighbor, 
                    // Trust the latest information
                    // Rest
                    if(agent.history_.iter_neighbors_his.back()[winner_idx_neighbor] > neighbor.iteration_neighbors_[winner_idx_neighbor]){
                        std::cout << "case 9 " << std::endl;
                        neighbor.winning_bids_matrix_(task.idx_, neighbor.idx_) = -1;
                        neighbor.assignment_matrix_(task.idx_, winner_idx_neighbor) = 0;
                    }
                }


                // Entry 8
                else if(winner_idx_neighbor = -1){
                    std::cout << "Do nothing Entry 5 " << std::endl;
                }

                else{
                    std::cout << "Unknown case 2 " << std::endl;
                }

            }


            /*********************************************************************************************************/
			/*********************************************************************************************************/
			/*********************************************************************************************************/
			// Entries 9 to 13
            // If agent thinks the winner of task j is not either itself nor neighbor but other agent
            else if(winner_idx_agent >= 0){
                // Entry 9
                // If neighbor thinks the winner of task j should be itself
                if(neighbor.assignment_matrix_(task.idx_, neighbor.idx_) == 1){
                    // Fisrt, compare the iteration between agent and winnder_idx_agent, neighbor and winner_idx_agent
                    if(agent.history_.iter_neighbors_his.back()[winner_idx_agent] > neighbor.iteration_neighbors_[winner_idx_agent]){
                        // Update
                        if(agent.history_.winning_bids_.back()(task.idx_, agent.idx_) - neighbor.winning_bids_matrix_(task.idx_, neighbor.idx_) > eps){
                            std::cout << "case 10 " << std::endl;
                            neighbor.winning_bids_matrix_(task.idx_, neighbor.idx_) = agent.history_.winning_bids_.back()(task.idx_, agent.idx_);
                            neighbor.assignment_matrix_(task.idx_, winner_idx_agent) = 1;
                            neighbor.assignment_matrix_(task.idx_, neighbor.idx_) = 0;
                        }

                        // Equal score:
                        else if(abs(agent.history_.winning_bids_.back()(task.idx_, agent.idx_) - neighbor.winning_bids_matrix_(task.idx_, neighbor.idx_)) <= eps){
                            if(neighbor.idx_ > winner_idx_agent){
                                // Update
                                std::cout << "case 11 " << std::endl;
                                neighbor.winning_bids_matrix_(task.idx_, neighbor.idx_) = agent.history_.winning_bids_.back()(task.idx_,agent.idx_);
                                neighbor.assignment_matrix_(task.idx_, winner_idx_agent) = 1;
                                neighbor.assignment_matrix_(task.idx_, neighbor.idx_) = 0;
                            }
                        }
                    }
                    
                }
                // Entry 10
                // If neighbor thinks the winner of task j is agent
                else if(neighbor.assignment_matrix_(task.idx_, agent.idx_) == 1){
                    for (int idx=0; idx<4; idx++){
                        std::cout << neighbor.iteration_neighbors_[idx] << " ";
                    }
                    std::cout << std::endl;
                    // Compare the iteration between agent and winner_idx_agent, neighbor and winner_idx_agent
                    if (agent.history_.iter_neighbors_his.back()[winner_idx_agent] > neighbor.iteration_neighbors_[winner_idx_agent]){
                        std::cout << "case 12" << std::endl;
                        neighbor.winning_bids_matrix_(task.idx_, neighbor.idx_) = agent.history_.winning_bids_.back()(task.idx_, agent.idx_);
                        neighbor.assignment_matrix_(task.idx_, winner_idx_agent) = 1;
                        neighbor.assignment_matrix_(task.idx_, agent.idx_) = 0;
                    }

                    else{
                        // Reset
                        std::cout << "case 13 " << std::endl;
                        neighbor.winning_bids_matrix_(task.idx_, neighbor.idx_) = -1;
                        neighbor.assignment_matrix_(task.idx_, agent.idx_) = 0;
                    }
                }

                // Entry 11
                // If agent and neighbor make an agreement here about the winner of task j
                else if(winner_idx_agent == winner_idx_neighbor){
                    // Update 
                    if(agent.history_.iter_neighbors_his.back()[winner_idx_agent] > neighbor.iteration_neighbors_[winner_idx_agent]){
                        std::cout << "case 14 " << std::endl;
                        neighbor.winning_bids_matrix_(task.idx_, neighbor.idx_) = agent.history_.winning_bids_.back()(task.idx_, agent.idx_);
                        neighbor.assignment_matrix_(task.idx_, winner_idx_agent) = 1;
                    }
                }

                // Entry 12 
                // If neighbor thinks the winner of task j is not agent, itself, and winner_idx_agent
                else if(winner_idx_neighbor >= 0){
                    // Compare the iteration between agent winner_idx_neighbor, neighbor and winner_idx_neighbor
                    if(agent.history_.iter_neighbors_his.back()[winner_idx_neighbor] > neighbor.iteration_neighbors_[winner_idx_neighbor]){
                        // Compare the iteration between agent, winner_idx_agent and neighbor, winnder_idx_agent
                        if (agent.history_.iter_neighbors_his.back()[winner_idx_agent] > neighbor.iteration_neighbors_[winner_idx_agent]){
                            //update
                            std::cout << "case 15 " << std::endl;
                            neighbor.winning_bids_matrix_(task.idx_, neighbor.idx_) = agent.history_.winning_bids_.back()(task.idx_, agent.idx_);
                            neighbor.assignment_matrix_(task.idx_, winner_idx_neighbor) = 0;
                            neighbor.assignment_matrix_(task.idx_, winner_idx_agent) = 1;
                        }

                        else if(agent.history_.iter_neighbors_his.back()[winner_idx_agent] <= neighbor.iteration_neighbors_[winner_idx_agent]){
                            // Reset
                            std::cout << "case 16 " << std::endl;
                            neighbor.winning_bids_matrix_(task.idx_, neighbor.idx_) = -1;
                            neighbor.assignment_matrix_(task.idx_, winner_idx_neighbor) = 0;
                        }

                        else{
                            std::cout << "Should not be here Entry 12" << std::endl;
                        }
                    }
                    else{
                        // Compare the iteration between agent, winner_idx_agent and neighbor, winnder_idx_agent
                        if(agent.history_.iter_neighbors_his.back()[winner_idx_agent] > neighbor.iteration_neighbors_[winner_idx_agent]){
                            // Update
                            if(agent.history_.winning_bids_.back()(task.idx_, agent.idx_) - neighbor.winning_bids_matrix_(task.idx_, neighbor.idx_) > eps){
                                std::cout << "case 17 " << std::endl;
                                neighbor.winning_bids_matrix_(task.idx_, neighbor.idx_) = agent.history_.winning_bids_.back()(task.idx_, agent.idx_);
                                neighbor.assignment_matrix_(task.idx_, winner_idx_neighbor) = 0;
                                neighbor.assignment_matrix_(task.idx_, winner_idx_agent) = 1; 
                            }

                            // Equal score:
                            else if(abs(agent.history_.winning_bids_.back()(task.idx_, agent.idx_) - neighbor.winning_bids_matrix_(task.idx_, neighbor.idx_)) <= eps){
                                if(winner_idx_neighbor > winner_idx_agent){
                                    std::cout << "case 18 " << std::endl;
                                    neighbor.winning_bids_matrix_(task.idx_, neighbor.idx_) = agent.history_.winning_bids_.back()(task.idx_, agent.idx_);
                                    neighbor.assignment_matrix_(task.idx_, winner_idx_neighbor) = 0;
                                    neighbor.assignment_matrix_(task.idx_, winner_idx_agent) = 1; 
                                }
                            }
                        }

                    }
                }


                // Entry 13
                else if(winner_idx_neighbor == -1){
                    // update
                    // Compare the iteration between agent, winner_idx_agent and neighbor, winner_idx_agent
                    if(agent.history_.iter_neighbors_his.back()[winner_idx_agent] > neighbor.iteration_neighbors_[winner_idx_agent]){
                        std::cout << "case 19 " << std::endl;
                        neighbor.winning_bids_matrix_(task.idx_, neighbor.idx_) = agent.history_.winning_bids_.back()(task.idx_, agent.idx_);
                        neighbor.assignment_matrix_(task.idx_, winner_idx_agent) = 1;
                    }
                }

                else{
                    std::cout << "Unknown case 3 " << std::endl;
                }

            }

            /*********************************************************************************************************/
			/*********************************************************************************************************/
			/*********************************************************************************************************/
            // Entries 14 to 17
            // If agent has no idea about who is the winner
            else if(winner_idx_agent == -1){
                // Entry 14
                // Leave 
                // If neighbor thinks the winner is itself
                if (neighbor.assignment_matrix_(task.idx_, neighbor.idx_) == 1){
                    std::cout << "Do nothing Entry 14 " << std::endl;
                }

                // Entry 15
                // Update
                else if(neighbor.assignment_matrix_(task.idx_, agent.idx_) == 1){
                    std::cout << "case 20 " << std::endl;
                    neighbor.winning_bids_matrix_(task.idx_, neighbor.idx_) = agent.history_.winning_bids_.back()(task.idx_, agent.idx_);
                    // neighbor.assignment_matrix_(task.idx_, winner_idx_agent) = 0;
                }

                // Entry 16
                // If neighbor thinks the winner of task j is not itself, agent, but other vehicle
                else if(winner_idx_neighbor >= 0){
                    // Update 
                    // Compare the iteration between agent, winner_idx_neighbor and neighbor, winner_idx_neighbor
                    if(agent.history_.iter_neighbors_his.back()[winner_idx_neighbor] > neighbor.iteration_neighbors_[winner_idx_neighbor]){
                        std::cout << "case 21 " << std::endl;
                        neighbor.winning_bids_matrix_(task.idx_, neighbor.idx_) = agent.history_.winning_bids_.back()(task.idx_, agent.idx_);
                    }
                }

                // Entry 17
                // Leave
                else if(winner_idx_neighbor = -1){
                    std::cout << "Do nothing Entry 17 " << std::endl;
                }

                else{
                    std::cout << "Unknown case 4 " << std::endl;
                }
            }

            else{
                std::cout << "Unknown case 5 " << std::endl;
            }



            std::cout << "After communicate " << std::endl;
            std::cout << "neighbor info is " << std::endl;
            std::cout << neighbor.winning_bids_matrix_ << std::endl;
            std::cout << neighbor.assignment_matrix_ << std::endl;




        }
    }
    
}

void CBGA::update_iteration_number(std::vector<cbga_Agent>& agents){
    
    for(auto &agent: agents){
        for (int i = 0; i < agent.neighbors_.size(); i++){
            cbga_Agent& neighbor = CBGA::FindAgent(agents, agent.neighbors_[i]);

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

void CBGA::communicate_dependent(std::vector<cbga_Agent>& agents, cbga_Task task){
    for (auto &agent:agents){
        // Update the neighbor
        agent.neighbors_finder();
        
        for (int idx = 0; idx < agent.neighbors_.size(); idx++){
            cbga_Agent& neighbor = CBGA::FindAgent(agents, agent.neighbors_[idx]);

            // Find the winners of task j that agent believes
            std::vector<int> winners_idx_agent = {};
            std::vector<int> winners_idx_neighbor = {};
            std::vector<double> bids_agent = {};
            for (int m = 0; m < agent.num_agents_; m++){
                if(agent.assignment_matrix_(task.idx_, m) == 1){
                    winners_idx_agent.push_back(m);
                    bids_agent.push_back(agent.winning_bids_matrix_(task.idx_, m));
                }
        

                if(neighbor.history_.assignment_.back()(task.idx_, m) == 1){
                    winners_idx_neighbor.push_back(m);
                }
            }

            std::cout << "*******************************************************" << std::endl;
            std::cout << "Before communicate dependent " << std::endl;
            std::cout << "Agent is " << agent.idx_ << std::endl;
            std::cout << agent.winning_bids_matrix_ << std::endl;
            std::cout << agent.assignment_matrix_ << std::endl;

            std::cout << "Neighbor " << neighbor.idx_ << std::endl;
            std::cout << neighbor.history_.winning_bids_.back() << std::endl;
            std::cout << neighbor.history_.assignment_.back() << std::endl;

            for (auto &assigned_idx: winners_idx_agent){
                if (assigned_idx == neighbor.idx_ || neighbor.history_.iter_neighbors_his.back()[assigned_idx] > agent.iteration_neighbors_[assigned_idx]){
                    if(assigned_idx != agent.idx_){
                        std::cout << "assigned_idx " << assigned_idx << std::endl;
                        std::cout << "dependent case 1 " << std::endl;
                        agent.winning_bids_matrix_(task.idx_, assigned_idx) = neighbor.history_.winning_bids_.back()(task.idx_, assigned_idx);
                        agent.assignment_matrix_(task.idx_, assigned_idx) = neighbor.history_.assignment_.back()(task.idx_, assigned_idx);
                    } 
                }

            }

            // update winners infor for agent
            winners_idx_agent = {};
            bids_agent = {};
            for (int m = 0; m < agent.num_agents_; m++){
                if(agent.assignment_matrix_(task.idx_, m) == 1){
                        winners_idx_agent.push_back(m);
                        bids_agent.push_back(agent.winning_bids_matrix_(task.idx_, m));
                }
            }  
            int bid_min = 0.0;
            int idx_n = -1;
            if (!bids_agent.empty()){
                bid_min = 1000.0;
                for(int k = 0; k < bids_agent.size(); k++){
                    if(bids_agent[k] < bid_min){
                        bid_min = bids_agent[k];
                        idx_n = winners_idx_agent[k];
                    }
                }
            }
        

            for (auto &assigned_idx: winners_idx_neighbor){
                if(assigned_idx != agent.idx_ && agent.assignment_matrix_(task.idx_, assigned_idx) == 0 && neighbor.history_.iter_neighbors_his.back()[assigned_idx] > agent.iteration_neighbors_[assigned_idx]){
                    if (winners_idx_agent.size() < task.num_agents_){
                        if (assigned_idx != agent.idx_){
                            std::cout << "dependent case 2 " << std::endl;
                            agent.winning_bids_matrix_(task.idx_, assigned_idx) = neighbor.history_.winning_bids_.back()(task.idx_, assigned_idx);
                            agent.assignment_matrix_(task.idx_, assigned_idx) = neighbor.history_.assignment_.back()(task.idx_, assigned_idx);
                        }
                    }
                    else if(neighbor.history_.winning_bids_.back()(task.idx_, assigned_idx) > bid_min){
                        std::cout << "vehicle " << agent.idx_ << std::endl;
                        std::cout << "idx_n removed is " << idx_n << std::endl;
                        std::cout << "assigned_idx " << assigned_idx << std::endl;
                        std::cout << "dependent case 3 " << std::endl;
                        agent.winning_bids_matrix_(task.idx_, idx_n) = 0;
                        agent.assignment_matrix_(task.idx_, idx_n) = 0;

                        agent.winning_bids_matrix_(task.idx_, assigned_idx) = neighbor.history_.winning_bids_.back()(task.idx_, assigned_idx);
                        agent.assignment_matrix_(task.idx_, assigned_idx) = neighbor.history_.assignment_.back()(task.idx_, assigned_idx);
                    }
                }

                // update winners infor for agent
                winners_idx_agent = {};
                bids_agent = {};
                for (int m = 0; m < agent.num_agents_; m++){
                    if(agent.assignment_matrix_(task.idx_, m) == 1){
                        winners_idx_agent.push_back(m);
                        bids_agent.push_back(agent.winning_bids_matrix_(task.idx_, m));
                    }
                }  
                bid_min = 0.0;
                idx_n = -1;
                if (!bids_agent.empty()){
                    bid_min = 1000.0;
                    for(int k = 0; k < bids_agent.size(); k++){
                        if(bids_agent[k] < bid_min){
                            bid_min = bids_agent[k];
                            idx_n = winners_idx_agent[k];
                        }
                    }
                }
            }

            std::cout << "After communicate +++++++++++++++++++++++++++++ " << std::endl;
            std::cout << "agent " << agent.idx_ << std::endl;
            std::cout << agent.winning_bids_matrix_ << std::endl;
            std::cout << agent.assignment_matrix_ << std::endl;

        } 
    }


}


void CBGA::consensus(std::vector<cbga_Agent>& agents, CBGATasks tasks){
    for (auto &task: tasks.tasks_){
        if(task.num_agents_ == 1){
            std::cout << " ============= task " << task.idx_ << std::endl;
            CBGA::communicate(agents, task);
        }
        else{
            std::cout << " ============= task " << task.idx_ << std::endl;
            CBGA::communicate_dependent(agents, task);
        }
    }

   
}




double CBGA::WaitingTimeCalculation(std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<cbga_Agent>& agents, CBGATasks tasks){
	std::vector<int> dep_list;
	// Prepare tasks path only contain dependent tasks
	std::map<int, std::vector<int>> vehicle_path;
    int num_tasks_de_ = 8;
    int num_tasks_inde_ = 5;

	while (dep_list.size() != num_tasks_de_){
		std::vector<int> first_tasks = {};
		for (auto& agent: agents){
			for (auto& t_idx: agent.cbga_path_){
				std::vector<int>::iterator it = std::find(dep_list.begin(), dep_list.end(), t_idx);
				if (t_idx >= num_tasks_inde_ && it == dep_list.end()){
					first_tasks.push_back(t_idx);
					break;
				}
			}
		}

		for (auto& t: first_tasks){
			cbga_Task task = tasks.FindTaskFromID(t);
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
		cbga_Task& task = tasks.FindTask(b);
		std::vector<int> winners = FindWinners(agents, b);
		std::cout << "Winner for task " << b << " is: ";
		for (auto& k: winners){
			std::cout << k << ", ";
		}
		std::cout << std::endl;
		for (auto& a: winners){
			cbga_Agent& agent = CBGA::FindAgent(agents, a);
			std::vector<int> path = agent.cbga_path_;
			std::vector<int>::iterator it = std::find(agent.cbga_path_.begin(), agent.cbga_path_.end(), b);
			int task_length = it - agent.cbga_path_.begin();
			path.resize(task_length+1);

			std::cout << "The path info is " << std::endl;
			for (auto& i: path){
				std::cout << i << ", ";
			}
			std::cout << std::endl;
			

			double r = CBGA::PathLengthCalculationWithWaiting(graph, path, agent.init_pos_, tasks);
			std::cout << "the r is " << r << std::endl;
			if(r > max_reward){
				max_reward = r;
			}
		}
		task.max_reward_ = max_reward;
	}

	std::cout << "Test max reward for each dependent task " << std::endl;
	for(auto& bb: dep_list){
		cbga_Task task = tasks.FindTaskFromID(bb);
		std::cout << "For task " << bb << " ,the max reward is " << task.max_reward_ << std::endl;
	}


    double total_waiting = 0.0;
	for (auto& agent: agents){
		agent.UpdateWaitingTime(graph, tasks);
		std::cout << "For agent " << agent.idx_ << " the waiting time is " << agent.waiting_t_ << std::endl;
        total_waiting = total_waiting + agent.waiting_t_;
    }
	
	std::cout << "TOTAL WAITING TIME IS "<< total_waiting << std::endl;
	return 0.0;
}

void cbga_Agent::UpdateWaitingTime(std::shared_ptr<Graph_t<SquareCell*>> graph, CBGATasks tasks){
	waiting_t_ = 0.0;
	for (int i = 0; i < cbga_path_.size(); i++){
		cbga_Task task = tasks.FindTaskFromID(cbga_path_[i]);
		if(task.num_agents_ > 1){
			std::vector<int> path = cbga_path_;
			path.resize(i+1);
			double r = CBGA::PathLengthCalculationForWaiting(graph, path, init_pos_, tasks);
			if (r < task.max_reward_){
				waiting_t_ = waiting_t_ + (task.max_reward_ - r);
			}
		}
	}
}

double CBGA::PathLengthCalculationForWaiting(std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<int> bundle, int init_pos, CBGATasks tasks){
	Vertex_t<SquareCell*> * start_node = graph->GetVertexFromID(init_pos);
	std::vector<int64_t> pos_seq;
	std::vector<int64_t> task_seq;
	Path_t<SquareCell *> path_part;
	Path_t<SquareCell *> path;
	
	for (int i = 0; i < bundle.size(); i++){
		cbga_Task task = tasks.FindTaskFromID(bundle[i]);
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
		cbga_Task task = tasks.FindTaskFromID(task_seq[k]);
		Vertex_t<SquareCell*> * finish_node = graph->GetVertexFromID(pos_seq[k]);
		if(task.num_agents_ > 1){
			Path_t<SquareCell *> path_part = AStar::Search(graph,start_node->state_->id_,finish_node->state_->id_, CalcHeuristicFunc_t<SquareCell *>(CBGA::CalcHeuristic));
			if (k == task_seq.size() - 1){
				path.insert(path.end(), path_part.begin(), path_part.end());

				// if (task.max_reward_ != 0.0){
				// 	while ( (task.max_reward_) != path.size()){
				// 		path.push_back(finish_node->state_);
				// 	}
				// }	
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
			Path_t<SquareCell *> path_part = AStar::Search(graph,start_node->state_->id_,finish_node->state_->id_, CalcHeuristicFunc_t<SquareCell *>(CBGA::CalcHeuristic));
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


double CBGA::PathLengthCalculationWithWaiting(std::shared_ptr<Graph_t<SquareCell*>> graph, std::vector<int> bundle, int init_pos, CBGATasks tasks){
	Vertex_t<SquareCell*> * start_node = graph->GetVertexFromID(init_pos);
	std::vector<int64_t> pos_seq;
	std::vector<int64_t> task_seq;
	Path_t<SquareCell *> path_part;
	Path_t<SquareCell *> path;
	
	for (int i = 0; i < bundle.size(); i++){
		cbga_Task task = tasks.FindTaskFromID(bundle[i]);
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
		cbga_Task task = tasks.FindTaskFromID(task_seq[k]);
		Vertex_t<SquareCell*> * finish_node = graph->GetVertexFromID(pos_seq[k]);
		if(task.num_agents_ > 1){
			Path_t<SquareCell *> path_part = AStar::Search(graph,start_node->state_->id_,finish_node->state_->id_, CalcHeuristicFunc_t<SquareCell *>(CBGA::CalcHeuristic));
			if (k == task_seq.size() - 1){
				path.insert(path.end(), path_part.begin(), path_part.end());

				if (task.max_reward_ != 0.0){
					while ( (task.max_reward_) > path.size()){
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
			Path_t<SquareCell *> path_part = AStar::Search(graph,start_node->state_->id_,finish_node->state_->id_, CalcHeuristicFunc_t<SquareCell *>(CBGA::CalcHeuristic));
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




int CBGA::FindNumAppearance(std::vector<int> list, int target){
	int num_appearance = 0;
	for (int i = 0; i < list.size(); i++){
		if(list[i] == target){
			num_appearance++;
		}
	}

	return num_appearance;
}


std::vector<int> CBGA::FindWinners(std::vector<cbga_Agent> agents, int task_idx){
	std::vector<int> winners_list;
	for (auto& agent: agents){
		std::vector<int>::iterator it = std::find(agent.cbga_path_.begin(), agent.cbga_path_.end(), task_idx);
		if(it != agent.cbga_path_.end()){
			winners_list.push_back(agent.idx_);
		}
	}
	return winners_list;
}


