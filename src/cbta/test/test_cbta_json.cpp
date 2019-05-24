// standard libaray
#include <stdio.h>
#include <vector>
#include <ctime>
#include <tuple>
#include <algorithm>
#include <bitset>
// opencv
#include "opencv2/opencv.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

// user
#include "vis/graph_vis.hpp"
#include "graph/graph.hpp"
#include "graph/algorithms/astar.hpp"
#include "map/square_grid.hpp"
#include "vis/graph_vis.hpp"

#include "ltl/buchi_automaton.hpp"
#include "ltl/product_automaton.hpp"
#include "cbta/graph_lift.hpp"
#include "cbta/hcost_interface.hpp"
#include "nlohmann/json.hpp"

using nlohmann::json;

namespace ns {
	void from_json(const json& j, librav::TileTraversalData& ttd, int desired_H){

		/********** N_REGION **********/
		j.at("N_REGION_W").get_to(ttd.N_REGION_W);
		j.at("N_REGION_PSI").get_to(ttd.N_REGION_PSI);
		j.at("N_REGION_SPD").get_to(ttd.N_REGION_SPD);
		j.at("N_REGION_TOTAL").get_to(ttd.N_REGION_TOTAL);

		/********** region_bd **********/
		j.at("REGION_BD").at("region_psi_lower").get_to(ttd.region_bd.region_psi_lower);
		j.at("REGION_BD").at("region_psi_upper").get_to(ttd.region_bd.region_psi_upper);
		j.at("REGION_BD").at("region_w_lower").get_to(ttd.region_bd.region_w_lower);	
		j.at("REGION_BD").at("region_w_upper").get_to(ttd.region_bd.region_w_upper);	
		j.at("REGION_BD").at("region_vel_lower").get_to(ttd.region_bd.region_vel_lower);	
		j.at("REGION_BD").at("region_vel_upper").get_to(ttd.region_bd.region_vel_upper);
		
		/********** Hlevels **********/
		unsigned int H;
		unsigned int n_tiles;
		Eigen::Matrix<int,Eigen::Dynamic,Eigen::Dynamic> unique_tiles;
		j.at("Hlevels").at("H").get_to(H);
		j.at("Hlevels").at("n_tiles").get_to(n_tiles);
		std::string unique_tiles_str;
		j.at("Hlevels").at("unique_tiles").get_to(unique_tiles_str);
		std::shared_ptr<librav::Hlevel> current_Hlevel = std::make_shared<librav::Hlevel>(H,n_tiles,unique_tiles_str);
		

		/********** Tiles **********/
		std::string cell_edge_str;
		std::string cell_vertices_str;
		std::string cell_xform_str;
		std::string channel_data_str;
		std::string connectivity_str;
		std::string traversal_type_str;
		std::string traversal_faces_str;

		std::string alfa_str;
		std::string bta_str;
		std::string w_lower_str;
		std::string w_upper_str;
		std::string x_str;
		std::string w_str;
		std::string w_sol_str;

		for(int n = 0; n < n_tiles; n++){

			std::stringstream str;
			str << "Tile_" << n;
			std::string title = str.str();
			const char *current_tile_name = title.c_str();
			std::cout << str.str() << std::endl;
			// Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> cell_edge;
			// Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> cell_vertices;
			// Eigen::Matrix<int,Eigen::Dynamic,Eigen::Dynamic> cell_xform;
			// Eigen::Matrix<int,Eigen::Dynamic,Eigen::Dynamic> channel_data;
			j.at(current_tile_name).at("traversal_type").get_to(traversal_type_str);
			j.at(current_tile_name).at("traversal_faces").get_to(traversal_faces_str);			
			j.at(current_tile_name).at("cell_edge").get_to(cell_edge_str);
			j.at(current_tile_name).at("cell_vertices").get_to(cell_vertices_str);
			j.at(current_tile_name).at("cell_xform").get_to(cell_xform_str);
			j.at(current_tile_name).at("channel_data").get_to(channel_data_str);
			j.at(current_tile_name).at("connectivity").get_to(connectivity_str);

			std::cout << traversal_type_str << std::endl;

			std::shared_ptr<librav::Tile> newTile = std::make_shared<librav::Tile>(desired_H,traversal_type_str,traversal_faces_str,cell_xform_str,channel_data_str,cell_edge_str,cell_vertices_str,connectivity_str);
			current_Hlevel->Tiles.insert({n, newTile});

			/********** Tile Block **********/
			// j.at(current_tile_name).at("tile_block").get_to();
			j.at(current_tile_name).at("tile_block").at("alfa").get_to(alfa_str);
			j.at(current_tile_name).at("tile_block").at("bta").get_to(bta_str);			
			j.at(current_tile_name).at("tile_block").at("w_lower").get_to(w_lower_str);
			j.at(current_tile_name).at("tile_block").at("w_upper").get_to(w_upper_str);
			j.at(current_tile_name).at("tile_block").at("x").get_to(x_str);
			j.at(current_tile_name).at("tile_block").at("w").get_to(w_str);
			j.at(current_tile_name).at("tile_block").at("w_sol").get_to(w_sol_str);
		}
		ttd.Hlevels.insert({desired_H,current_Hlevel});

	}

} // namespace ns


using namespace cv;
using namespace librav;

int main(int argc, char *argv[] )
{		
	int desired_Hlevel = 3;
	/*** 0. Preprocessing CBTA data ***/
	// TileTraversalData tile_traversal_data = HCost::hcost_preprocessing();
	TileTraversalData tile_traversal_data;

	std::ifstream i("tile_traversal_data_H3_R3.txt");
	nlohmann::json j;
	i >> j;

	ns::from_json(j, tile_traversal_data, desired_Hlevel);
	// std::cout << tile_traversal_data.Hlevels.at(desired_Hlevel)->n_tiles << std::endl;


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
	std::shared_ptr<Graph_t<ProductState *>> product_graph_new = std::make_shared<Graph_t<ProductState *>>();
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
