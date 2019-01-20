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
#include "cbta/graph_lift.hpp"
#include "cbta/hcost_interface.hpp"

using namespace cv;
using namespace librav;

int main(int argc, char** argv){
    /*** 0. Preprocessing CBTA data ***/
	std::vector<double> radius_ = {4, 3, 3, 2};
	std::map<int, TileTraversalData> tile_travelsal_data;
	int idx = 0;
	for(auto &r_min: radius_){
		tile_travelsal_data[idx] = HCost::hcost_preprocessing(r_min);
		idx++;
	}

	// 
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

	
	/*** 4. Initialize agents ***/
	std::vector<Agent> agents_group = TaskAssignment::InitializeAgents();
	int num_agents = agents_group.size();

    /*** 5. Construct a graph from the square grid ***/
	std::shared_ptr<Graph_t<SquareCell *>> grid_graph = GraphFromGrid::BuildGraphFromSquareGrid(grid, false, true);

	/************************************************************************************************************/
	/*********************************          Pre-Porcess for CBTA         ************************************/
	/************************************************************************************************************/
	int historyH = 4;
	//std::shared_ptr<Graph_t<LiftedSquareCell>> lifted_graph = GraphLifter::CreateLiftedGraph(HistoryH, graph);
    std::cout << "Start lifted graph " << std::endl;
    std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph = GraphLifter::BuildLiftedGraph(historyH, grid_graph);
	std::cout << "End lifted graph" << std::endl;
	

	// /************************************************************************************************************/
	// /**************************************          CBBA         ***********************************************/
	// /************************************************************************************************************/
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

		// std::cout << "================================= AFTER COMMUNICATION ================================== " << std::endl;
		// for(int i = 0; i < agents_group[0].num_agents_; i++){
		// 	std::cout << "Vehicle " << i << std::endl;
		// 	std::cout << "Winners for tasks are " << std::endl;
		// 	std::cout << agents_group[i].cbba_z_ << std::endl;
		// 	std::cout << "Highest reward are " << std::endl;
		// 	std::cout << agents_group[i].cbba_y_ << std::endl;
		// 	std::cout << "Path info is " << std::endl;
		// 	for(auto &b: agents_group[i].cbba_path_)
		// 		std::cout << b << " ";
		// 	std::cout << std::endl;
		// 	std::cout << "Reward for each vehicle is " << std::endl;
		// 	std::cout << agents_group[i].cbba_reward_ << std::endl;
		
		// }

		/*** 7. Bundle Operations ***/
		/*** 7.1 Remove the out-bid task from the bundle ***/
		TaskAssignment::bundle_remove(agents_group);
		// std::cout << "===================================== AFTER BUNDLE REMOVE ======================================" << std::endl;
		// for(int i = 0; i < agents_group[0].num_agents_; i++){
		// 	std::cout << "Vehicle " << i << std::endl;
		// 	std::cout << "Path info is " << std::endl;
		// 	for(auto &b: agents_group[i].cbba_path_)
		// 		std::cout << b << " ";
		// 	std::cout << std::endl;
		// }
		/*** 7.2 Keep inserting tasks which have not been assigned into the bundle ***/
		TaskAssignment::bundle_add(tasks,grid_graph,lifted_graph, tile_travelsal_data,grid, agents_group);
		/*** 7.3 Check whether the assignment converge or not ***/
		succFlag = TaskAssignment::success_checker(agents_group);
		std::cout << "The Flag for success is " << succFlag <<std::endl;
		/*** 7.4 Update the number of interation ***/
		
		// Increase the iteration
		for (int i = 0; i < agents_group[0].num_agents_; i++)
			agents_group[i].iter_++;

		// std::cout << "====================================== AFTER INSERT NEW TASKS ========================================== " << std::endl;
		// for(int i = 0; i < agents_group[0].num_agents_; i++){
		// 	std::cout << "Vehicle " << i << std::endl;
		// 	std::cout << "Winners for tasks are " << std::endl;
		// 	std::cout << agents_group[i].cbba_z_ << std::endl;
		// 	std::cout << "Highest reward are " << std::endl;
		// 	std::cout << agents_group[i].cbba_y_ << std::endl;
		// 	std::cout << "Path info is " << std::endl;
		// 	std::cout << "Path info is " << std::endl;
		// 	for(auto &b: agents_group[i].cbba_path_)
		// 		std::cout << b << " ";
		// 	std::cout << std::endl;
		// 	std::cout << "Reward for each vehicle is " << std::endl;
		// 	std::cout << agents_group[i].cbba_reward_ << std::endl;
		// }
	}
	cbba_time = clock()-cbba_time;
	std::cout << "CBBA iteration is " << agents_group[0].iter_ << std::endl;
	std::cout << "The running time required is " << double(cbba_time)/CLOCKS_PER_SEC << std::endl;

	std::cout << "FINAL RESULT OF CBBA " << std::endl;
	for (auto &agent: agents_group){
		std::cout << "FOR AGENT " << agent.idx_ << std::endl;
		std::cout << "Highest reward is " << std::endl;
		std::cout << agent.cbba_y_ << std::endl;
		std::cout << "Winners " << std::endl;
		std::cout << agent.cbba_z_ << std::endl;
		std::cout << "Path is " << std::endl;
		for (auto &b: agent.cbba_path_){
			std::cout << b << ", ";
		}
		std::cout << std::endl;
		std::cout << std::endl;
	}



	/************************************************************************************************************/
	/**************************************          Visualization         **************************************/
	/************************************************************************************************************/
	// agents_group[0].cbba_path_ = {1,4};
	// agents_group[1].cbba_path_ = {5,0};
	// agents_group[2].cbba_path_ = {2};
	// agents_group[3].cbba_path_ = {3};
	std::cout << "Finish the task allocation!!! Start to visualize the result " << std::endl;
	// /*** 8. Visualization ***/
	// std::vector<Vertex_t<SquareCell*>*> path_origin[num_agents];
	// Path_t<SquareCell *> path_vis[num_agents];
	// for (auto it_ag = agents_group.begin(); it_ag != agents_group.end(); it_ag++){
		
	// 	/*** 8.1 Rebuild the local LTL specification based on current bundle/path ***/
	// 	std::string ltl_formula = LTLDecomposition::subtask_recreator((*it_ag).cbba_path_,true, Global_LTL);
	// 	std::cout << "The specification is " << ltl_formula << std::endl;
	// 	std::vector<std::vector<std::string>> buchi_regions = LTLDecomposition::ObtainBuchiRegion({ltl_formula});

	// 	/*** 8.2 Generate the corresponding buchi regions ***/
    //     std::shared_ptr<Graph_t<BuchiState *, std::vector<int64_t>>> buchi_graph = BuchiAutomaton::BuildBuchiGraph(ltl_formula,buchi_regions.front());
    //     std::vector<int64_t> buchi_acc = (*buchi_graph->FindVertex(0)).state_->acc_state_idx;
	// 	//std::shared_ptr<Graph_t<BuchiState>> buchi_graph = BuchiAutomaton::CreateBuchiGraph(ltl_formula,buchi_regions.front());

	// 	/*** 8.4 Construct a product graph ***/
    //     std::shared_ptr<Graph_t<ProductState *>> product_graph = std::make_shared<Graph_t<ProductState *>>();
    //     int64_t start_id_grid = (*it_ag).init_pos_;
	// 	Vertex_t<SquareCell *> * start_node_origin = grid_graph->GetVertexFromID(start_id_grid);


	// 	std::vector<double> zta0 = {0.45,0.0};
	// 	std::vector<int> rgn_idx_init = {HCost::zta02rgn_idx(zta0,tile_traversal_data.region_bd)};
	// 	std::cout << "Start to look for virtual id " << std::endl;
	// 	int64_t virtual_start_id = ProductAutomaton::SetVirtualStartIncre(start_node_origin, rgn_idx_init, lifted_graph, buchi_graph, product_graph);
	// 	std::cout << "The virtual start id is " << virtual_start_id << std::endl;

	// 	GetProductCBTANeighbour get_product_cbta_neighbour(lifted_graph, buchi_graph, tile_traversal_data, grid);
	// 	std::cout << "AStar productIncSearch is called " << std::endl;
	// 	auto path = AStar::ProductIncSearchCBTA(product_graph, virtual_start_id, buchi_acc,  GetNeighbourFunc_t<ProductState*, double>(get_product_cbta_neighbour));
	// 	std::cout << "A star complete " << std::endl;

	// 	if (!(*it_ag).cbba_path_.empty()){
    //     	for(auto it = path.begin()+1; it != path.end(); it++){
	// 			//std::cout << " e->lifted_vertex_->state_->history.front() " << e->lifted_vertex_->state_->history.front()->id_<< std::endl;
    //         	path_origin[(*it_ag).idx_].push_back((*it)->lifted_vertex_->state_->history.front());
    //    	 	}
			
	// 		path_origin[(*it_ag).idx_].insert(path_origin[(*it_ag).idx_].end(),path.back()->lifted_vertex_->state_->history.begin()+1,path.back()->lifted_vertex_->state_->history.end());
	// 		std::cout << "The path length for vehicle "  << (*it_ag).idx_ << " is " << path_origin[(*it_ag).idx_].size() << std::endl;

	// 		for(auto &ee: path_origin[(*it_ag).idx_]){
	// 			path_vis[(*it_ag).idx_].push_back(ee->state_);
	// 		}
	// 	}
	// 	else{	
	// 		path_vis[(*it_ag).idx_].push_back(start_node_origin->state_);
	// 	}
		
	// }


	// /*** 9.Visualize the map and graph ***/
	// // Image Layouts: square grid -> graph -> path
	// GraphVis vis;
	// Mat vis_img;
	// vis.VisSquareGrid(*grid, vis_img);
	// vis.VisSquareGridGraph(*grid_graph, vis_img, vis_img, true);
	// // // put the path on top of the graph
	// vis.VisSquareGridPath(path_vis[0], vis_img, vis_img);
	// vis.VisSquareGridPath(path_vis[1], vis_img, vis_img);
	// vis.VisSquareGridPath(path_vis[2], vis_img, vis_img);
	// vis.VisSquareGridPath(path_vis[3], vis_img, vis_img);
	
	// // display visualization result
	// //namedWindow("Processed Image", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
	// //imshow("Processed Image", vis_img);
	// imwrite("result_cbba_cbta.jpg",vis_img);

	waitKey(0);
    return 0;
}