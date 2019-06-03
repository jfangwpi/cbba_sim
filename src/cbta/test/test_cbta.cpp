/*** Simple example of CBTA 
 * by Jie Fang
 * ***/
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
	TileTraversalData tile_traversal_data = HCost::hcost_preprocessing();

	/*** 1. Create a empty square grid ***/
	int row_num = 7;
	int col_num = 7;
	std::shared_ptr<SquareGrid> grid = GraphFromGrid::CreateSquareGrid(row_num,col_num,1);

	// assign properties of square cells
    for (int i = 35; i < 40; i++){
        grid->SetObstacleRegionLabel(i,1);
    }
    grid->SetInterestedRegionLabel(28,2);
	// grid->SetInterestedRegionLabel(10,3);

	/*** 2. Construct a graph from the square grid ***/
	std::shared_ptr<Graph_t<SquareCell*>> graph = GraphFromGrid::BuildGraphFromSquareGrid(grid, false, true);

    /*** 4. Construct a lifted graph ***/
	int historyH = 4;
    std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph = GraphLifter::BuildLiftedGraph(historyH, graph);
   
	/*** 3. Construct a graph from the buchi automata ***/
	/*** Visit region p2 ***/
	std::string ltl_formula = "[] p0 && [] !p1 && <> p2";
	std::vector<std::string> buchi_regions;
	buchi_regions.push_back("p0");
	buchi_regions.push_back("p1");
	buchi_regions.push_back("p2");
	buchi_regions.push_back("p3");
    std::shared_ptr<Graph_t<BuchiState *, std::vector<int64_t>>> buchi_graph = BuchiAutomaton::BuildBuchiGraph(ltl_formula,buchi_regions);
	
	/*** 5. Construct a product graph ***/
	clock_t     total_time;
	clock_t		product_time;
	total_time = clock();
	product_time = clock();
	std::shared_ptr<Graph_t<ProductState *>> product_graph_new = std::make_shared<Graph_t<ProductState *>>();
	product_time = clock() - product_time;

	/*** 6. Search in the product graph ***/
	// Set start node in original graph
	Vertex_t<SquareCell *> * start_node_origin = graph->GetVertexFromID(42);

    std::cout << "Start node origin is " << start_node_origin->state_->lifted_vertices_id_.size() << std::endl;
    for (auto it = start_node_origin->state_->lifted_vertices_id_.begin(); it != start_node_origin->state_->lifted_vertices_id_.end(); it++)
        std::cout << (*it) << " ";
    std::cout << std::endl;

	// Convert from original node to product node
	std::vector<double> zta0 = {0.45,0.0};
	std::vector<int> rgn_idx_init = {HCost::zta02rgn_idx(zta0,tile_traversal_data.region_bd)};
    std::cout << "Start to look for virtual id " << std::endl;
	int64_t virtual_start_id = ProductAutomaton::SetVirtualStartIncre(start_node_origin, rgn_idx_init, lifted_graph, buchi_graph, product_graph_new);
    std::cout << "The virtual start id is " << virtual_start_id << std::endl;
	// Search in product graph
	std::vector<int64_t>  buchi_acc = buchi_graph->GetVertexFromID(0)->state_->acc_state_idx;

	GetProductCBTANeighbour get_product_cbta_neighbour(lifted_graph, buchi_graph, tile_traversal_data, grid);

	std::cout << "Start A star " << std::endl;
	auto path = AStar::ProductIncSearchCBTA(product_graph_new, virtual_start_id,buchi_acc,  GetNeighbourFunc_t<ProductState*, double>(get_product_cbta_neighbour));
	total_time = clock() - total_time;
	std::cout << "Total time in " << double(total_time)/CLOCKS_PER_SEC << " s." << std::endl;

	// Map path in the product graph back to the square grid graph
	std::vector<Vertex_t<SquareCell*>*> path_origin;
    //path_t<SquareCell *> path_vis;
	for (auto it = path.begin()+1; it != path.end(); it++){
		path_origin.push_back((*it)->lifted_vertex_->state_->history.front());
	}
	path_origin.insert(path_origin.end(),path.back()->lifted_vertex_->state_->history.begin()+1,path.back()->lifted_vertex_->state_->history.end());
    
    Path_t<SquareCell *> path_vis;
    for (auto &e: path_origin){
        path_vis.push_back(e->state_);
    }

	for (auto &ee:path_vis){
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
	GraphVis::VisSquareGridPath(path_vis, vis_img, vis_img);

	// display visualization result
	namedWindow("Processed Image", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
	imshow("Processed Image", vis_img);

	waitKey(0);

	return 0;
}
