/*
 * hcost_interface.h
 *
 *  Created on: 18 Nov 2016
 *      Author: zetian
 */

#ifndef HCOST_INTERFACE_HPP
#define HCOST_INTERFACE_HPP

#include <vector>
#include <map>
#include <memory>
#include <tuple>

#include "graph/graph.hpp"
#include "map/square_grid.hpp"
#include "cbta/hcost_tile_library.hpp"

namespace librav
{
struct TransitionData{
	std::shared_ptr<Tile> tile_data;
	double cost;
	std::vector<int> rgn_idx_next;
};

namespace HCost {

//double get_lifted_transition(int H, std::vector<Vertex<SquareCell>> tile_vertices_Vertex, std::vector<int> rgn_idx_next, std::map<int, std::shared_ptr<Hlevel>>& Hlevels, int N_REGION_TOTAL);
TransitionData get_lifted_transition(int H, Eigen::Matrix<int,Eigen::Dynamic,4> tile_vertices, const std::vector<int>& rgn_idx_current, const std::map<unsigned int, std::shared_ptr<Hlevel>>& Hlevels, int N_REGION_TOTAL);


int zta02rgn_idx(std::vector<double> zta0, REGION_BD region_bd);

TileTraversalData hcost_preprocessing();
}

}



#endif /* HCOST_INTERFACE_HPP */

