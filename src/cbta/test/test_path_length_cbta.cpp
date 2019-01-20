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

int main(int argc, char** argv )
{
    /************************************************************************************************************/
	/*********************************          Initialize: Map         *****************************************/
	/************************************************************************************************************/
	double r_min = 4;
    TileTraversalData tile_traversal_data = HCost::hcost_preprocessing(r_min);
    /*** 1. Create a empty square grid ***/
	std::shared_ptr<SquareGrid> grid = GraphFromGrid::CreateSquareGrid();
	
	/*** 3. Define the target regions ***/
	// Define LTL specification
	LTLFormula Global_LTL;
	for (int i = 0; i < Global_LTL.task_info.size();i++)
		grid->SetInterestedRegionLabel(Global_LTL.task_info[i].pos_,Global_LTL.task_info[i].idx_);	
	// Decompose the global LTL_expression
	LTLDecomposition::GlobalLTLDecomposition(Global_LTL);
	
	/*** 4. Initialize agents ***/
	std::vector<Agent> agents_group = TaskAssignment::InitializeAgents();
	int num_agents = agents_group.size();

    /*** 5. Construct a graph from the square grid ***/
	std::shared_ptr<Graph_t<SquareCell *>> grid_graph = GraphFromGrid::BuildGraphFromSquareGrid(grid,false, true);

    int historyH = 4;
    std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph = GraphLifter::BuildLiftedGraph(historyH, grid_graph);


    std::vector<int> path_ = {2};
    std::string ltl_formula = LTLDecomposition::subtask_recreator(path_,true, Global_LTL);
    std::vector<std::vector<std::string>> buchi_regions = LTLDecomposition::ObtainBuchiRegion({ltl_formula});

    /*** 8.2 Generate the corresponding buchi regions ***/
    std::shared_ptr<Graph_t<BuchiState *, std::vector<int64_t>>> buchi_graph = BuchiAutomaton::BuildBuchiGraph(ltl_formula,buchi_regions.front());
    std::vector<int64_t> buchi_acc = (*buchi_graph->FindVertex(0)).state_->acc_state_idx;

    /*** 8.4 Construct a product graph ***/
    std::shared_ptr<Graph_t<ProductState *>> product_graph = std::make_shared<Graph_t<ProductState *>>();
    int64_t start_id_grid = agents_group[1].init_pos_;
    Vertex_t<SquareCell *> * start_node_origin = grid_graph->GetVertexFromID(start_id_grid);

    std::vector<double> zta0 = {0.45,0.0};
    std::vector<int> rgn_idx_init = {HCost::zta02rgn_idx(zta0,tile_traversal_data.region_bd)};
    int64_t virtual_start_id = ProductAutomaton::SetVirtualStartIncre(start_node_origin, rgn_idx_init, lifted_graph, buchi_graph, product_graph);

    GetProductCBTANeighbour get_product_cbta_neighbour(lifted_graph, buchi_graph, tile_traversal_data, grid);
    auto path = AStar::ProductIncSearchCBTA(product_graph, virtual_start_id, buchi_acc,  GetNeighbourFunc_t<ProductState*, double>(get_product_cbta_neighbour));
    
    std::vector<Vertex_t<SquareCell*>*> path_origin;
    for (auto cell = path.begin()+1; cell!= path.end(); cell++){
        path_origin.push_back((*cell)->lifted_vertex_->state_->history.front());
    }

    path_origin.insert(path_origin.end(),path.back()->lifted_vertex_->state_->history.begin()+1,path.back()->lifted_vertex_->state_->history.end());
     Path_t<SquareCell *> path_vis;
    for(auto &ee: path_origin){
        path_vis.push_back(ee->state_);
    }
    
    std::cout << "LTL formula is " << ltl_formula << std::endl;
    std::cout << "start state: " << start_id_grid << ", Virtual state: " << virtual_start_id << std::endl;
    std::cout << "The length of cbta path from CBTA is " << path.size() << std::endl;
    std::cout << "The length of cbta path after projection is " << path_vis.size() << std::endl;
    for(auto &cell: path_vis){
        std::cout << cell->id_ << ", ";
    }
    std::cout << std::endl;



	/*** 9.Visualize the map and graph ***/
	// Image Layouts: square grid -> graph -> path
	GraphVis vis;
	Mat vis_img;
	vis.VisSquareGrid(*grid, vis_img);
	vis.VisSquareGridGraph(*grid_graph, vis_img, vis_img, false);
	// // put the path on top of the graph
	vis.VisSquareGridPath(path_vis, vis_img, vis_img, 0);

	
	// display visualization result
	namedWindow("Processed Image", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
	imshow("Processed Image", vis_img);
	imwrite("result_length_cbta.jpg",vis_img);


}