// standard libaray
#include <stdio.h>
#include <vector>
#include <ctime>
#include <tuple>
#include <algorithm>
#include <bitset>
// opencv
#include "opencv2/opencv.hpp"

// user
#include "graph/graph.hpp"
#include "graph/algorithms/astar.hpp"
#include "map/square_grid.hpp"
#include "vis/graph_vis.hpp"

#include "ltl/buchi_automaton.hpp"
#include "ltl/product_automaton.hpp"
#include "cbta/graph_lift.hpp"
#include "cbta/hcost_interface.hpp"

using namespace cv;
using namespace librav;

int main(int argc, char** argv )
{
	/*** 0. Preprocessing CBTA data ***/
	std::map<double, TileTraversalData> tile_travelsal_data;
	double r_min_1 = 3.0;
	tile_travelsal_data[r_min_1] = HCost::hcost_preprocessing(r_min_1);

	double r_min_2 = 2.0;
	tile_travelsal_data[r_min_2] = HCost::hcost_preprocessing(r_min_2);


	/*** 1. Create a empty square grid ***/
	int row_num = 10;
	int col_num = 10;

	std::shared_ptr<SquareGrid> grid = GraphFromGrid::CreateSquareGrid(row_num,col_num,1);

	// assign properties of square cells
    for (int i = 70; i < 77; i++){
        grid->SetObstacleRegionLabel(i,1);
    }

    grid->SetInterestedRegionLabel(11,2);
	grid->SetInterestedRegionLabel(65,3);

	/*** 2. Construct a graph from the square grid ***/
    
	std::shared_ptr<Graph_t<SquareCell*>> graph = GraphFromGrid::BuildGraphFromSquareGrid(grid, false, true);


    /*** 4. Construct a lifted graph ***/
	int historyH = 4;
	//std::shared_ptr<Graph_t<LiftedSquareCell>> lifted_graph = GraphLifter::CreateLiftedGraph(HistoryH, graph);
    std::cout << "Start lifted graph " << std::endl;
    std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph = GraphLifter::BuildLiftedGraph(historyH, graph);
    std::cout << "End lifted graph" << std::endl;


	/*** 3. Construct a graph from the buchi automata ***/
	std::string ltl_formula = "[] p0 && [] !p1 && <> (p2 && <> p3)";
	std::vector<std::string> buchi_regions;
	buchi_regions.push_back("p0");
	buchi_regions.push_back("p1");
	buchi_regions.push_back("p2");
	buchi_regions.push_back("p3");
    std::cout << "Start buchi graph" << std::endl;
    std::shared_ptr<Graph_t<BuchiState *, std::vector<int64_t>>> buchi_graph = BuchiAutomaton::BuildBuchiGraph(ltl_formula,buchi_regions);
	std::cout << "End buchi graph " << std::endl;
    //std::shared_ptr<Graph_t<BuchiState>> buchi_graph = BuchiAutomaton::CreateBuchiGraph(ltl_formula,buchi_regions);



	/*** 5. Construct a product graph ***/
	clock_t     total_time;
	clock_t		product_time;
	total_time = clock();
	product_time = clock();
	std::shared_ptr<Graph_t<ProductState *>> product_graph_new1 = std::make_shared<Graph_t<ProductState *>>();
	std::shared_ptr<Graph_t<ProductState *>> product_graph_new2 = std::make_shared<Graph_t<ProductState *>>();
	product_time = clock() - product_time;

	/*** 6. Search in the product graph ***/
	// Set start node in original graph
	Vertex_t<SquareCell *> * start_node_origin = graph->GetVertexFromID(80);

    std::cout << "Start node origin is " << start_node_origin->state_->lifted_vertices_id_.size() << std::endl;
    for (auto it = start_node_origin->state_->lifted_vertices_id_.begin(); it != start_node_origin->state_->lifted_vertices_id_.end(); it++)
        std::cout << (*it) << " ";
    std::cout << std::endl;

	// Convert from original node to product node
	std::vector<double> zta0 = {0.45,0.0};
	std::map<double, std::vector<int>> rgn_idx_init;
	rgn_idx_init[r_min_1] = {HCost::zta02rgn_idx(zta0,tile_travelsal_data[r_min_1].region_bd)};
	rgn_idx_init[r_min_2] = {HCost::zta02rgn_idx(zta0,tile_travelsal_data[r_min_2].region_bd)};
    std::cout << "Start to look for virtual id " << std::endl;
	std::map<double, int64_t> virtual_start_id;
	virtual_start_id[r_min_1] = ProductAutomaton::SetVirtualStartIncre(start_node_origin, rgn_idx_init[r_min_1], lifted_graph, buchi_graph, product_graph_new1);
	virtual_start_id[r_min_2] = ProductAutomaton::SetVirtualStartIncre(start_node_origin, rgn_idx_init[r_min_2], lifted_graph, buchi_graph, product_graph_new2);
    std::cout << "The virtual start id is " << virtual_start_id[r_min_1] << std::endl;
	std::cout << "The virtual start id is " << virtual_start_id[r_min_2] << std::endl;
	// Search in product graph
	std::vector<int64_t>  buchi_acc = buchi_graph->GetVertexFromID(0)->state_->acc_state_idx;

	GetProductCBTANeighbour get_product_cbta_neighbour1(lifted_graph, buchi_graph, tile_travelsal_data[r_min_1], grid);
	GetProductCBTANeighbour get_product_cbta_neighbour2(lifted_graph, buchi_graph, tile_travelsal_data[r_min_2], grid);

	std::cout << "Start A star " << std::endl;
	auto path1 = AStar::ProductIncSearchCBTA(product_graph_new1, virtual_start_id[r_min_1], buchi_acc,  GetNeighbourFunc_t<ProductState*, double>(get_product_cbta_neighbour1));
	auto path2 = AStar::ProductIncSearchCBTA(product_graph_new2, virtual_start_id[r_min_2], buchi_acc,  GetNeighbourFunc_t<ProductState*, double>(get_product_cbta_neighbour2));

	total_time = clock() - total_time;
	std::cout << "Total time in " << double(total_time)/CLOCKS_PER_SEC << " s." << std::endl;


	// Map path in the product graph back to the square grid graph
	std::vector<Vertex_t<SquareCell*>*> path_origin1;
	std::vector<Vertex_t<SquareCell*>*> path_origin2;
    //path_t<SquareCell *> path_vis;
	for (auto it = path1.begin()+1; it != path1.end(); it++){
		path_origin1.push_back((*it)->lifted_vertex_->state_->history.front());
	}
	path_origin1.insert(path_origin1.end(),path1.back()->lifted_vertex_->state_->history.begin()+1,path1.back()->lifted_vertex_->state_->history.end());
    

	for (auto it = path2.begin()+1; it != path2.end(); it++){
		path_origin2.push_back((*it)->lifted_vertex_->state_->history.front());
	}
	path_origin2.insert(path_origin2.end(),path2.back()->lifted_vertex_->state_->history.begin()+1,path2.back()->lifted_vertex_->state_->history.end());


    Path_t<SquareCell *> path_vis1;
	Path_t<SquareCell *> path_vis2;
    for (auto &e: path_origin1){
        path_vis1.push_back(e->state_);
    }

	for (auto &e: path_origin2){
        path_vis2.push_back(e->state_);
    }

	for (auto &ee:path_vis1){
        std::cout << ee->id_ << " ";
    }
    std::cout << std::endl;


	// /*** 7. Visualize the map and graph ***/
    /*** Visualize the map and graph ***/
	Mat vis_img;
	/*** Image Layouts: (map) -> square grid -> graph -> path ***/
	/*** you can visualize the squre grid by itself or overlay it on the map image ***/
	GraphVis::VisSquareGrid(*grid, vis_img);
	//GraphVis::VisSquareGrid(*grid, vis_img, vis_img, true);
	GraphVis::VisSquareGridPath(path_vis1, vis_img, vis_img);
	GraphVis::VisSquareGridPath(path_vis2, vis_img, vis_img);
	// display visualization result
	namedWindow("Processed Image", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
	imshow("Processed Image", vis_img);

	waitKey(0);

	return 0;
}
