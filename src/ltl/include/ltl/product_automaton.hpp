#ifndef PRODUCT_AUTOMATON_HPP
#define PRODUCT_AUTOMATON_HPP

#include <iostream>
#include <sstream>
#include <cstring>
#include <cstdint>
#include <map>
#include <vector>
#include <algorithm>
#include <memory>
#include <tuple>

#include "graph/graph.hpp"
#include "map/square_grid.hpp"
#include "graph/vertex.hpp"
#include "ltl/buchi_automaton.hpp"
#include "cbta/graph_lift.hpp"
#include "cbta/hcost_tile_library.hpp"
#include "cbta/hcost_interface.hpp"

namespace librav{

class ProductState
{
    public:
        ProductState(int64_t _id):
            id_(_id),
            grid_vertex_(nullptr),
            buchi_vertex_(nullptr),
            lifted_vertex_(nullptr),
            rgn_idx_init_()
        {};
        ~ProductState(){};

        int64_t id_;
        Vertex_t<SquareCell *> *  grid_vertex_;
        Vertex_t<BuchiState *, std::vector<int64_t>> * buchi_vertex_;

        Vertex_t<LiftedSquareCell *> * lifted_vertex_;
        std::vector<int> rgn_idx_init_;
        
    public:
        int64_t GetUniqueID() const{return id_;};

};

class ProductAutomaton
{
    public:
        static std::shared_ptr<Graph_t<ProductState *>> BuildProductGraph(std::shared_ptr<Graph_t<SquareCell *>> GridGraph, std::shared_ptr<Graph_t<BuchiState *, std::vector<int64_t>>> BuchiGraph);
        static int64_t SetVirtualStartState(std::shared_ptr<Graph_t<ProductState *>> product_graph, std::shared_ptr<Graph_t<BuchiState *, std::vector<int64_t>>> buchi_graph, std::shared_ptr<Graph_t<SquareCell* >> grid_graph, int64_t grid_start_id);

        static int64_t SetVirtualStartIncre(Vertex_t<SquareCell *> * start_node_origin, std::vector<int> rgn_idx_init, std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph, std::shared_ptr<Graph_t<BuchiState *, std::vector<int64_t>>> buchi_graph, std::shared_ptr<Graph_t<ProductState *>> product_graph);
};

class GetProductNeighbor
{
    public: 
        GetProductNeighbor(std::shared_ptr<Graph_t<SquareCell *>> GridGraph, std::shared_ptr<Graph_t<BuchiState *, std::vector<int64_t>>> BuchiGraph);
        ~GetProductNeighbor(){};
    
    private:
        std::shared_ptr<Graph_t<SquareCell *>> grid_graph_;
        std::shared_ptr<Graph_t<BuchiState *, std::vector<int64_t>>> buchi_graph_;

    public:
        void InfoCheck();
        std::vector<std::tuple<ProductState*, double>> operator()(ProductState* cell);
};

class GetProductCBTANeighbour {
public:
	GetProductCBTANeighbour(std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph, std::shared_ptr<Graph_t<BuchiState *, std::vector<int64_t>>> buchi_graph, TileTraversalData tile_traversal_data, std::shared_ptr<SquareGrid> grid){
		lifted_graph_ = lifted_graph;
		buchi_graph_ = buchi_graph;
		tile_traversal_data_ = tile_traversal_data;
//		h_levels_ = h_levels;
		grid_ = grid;
		history_H_ = lifted_graph->GetVertexFromID(0)->state_->history.size() - 1;
		Eigen::MatrixXi verts_cd(grid->num_row_*grid->num_col_,4);
		int total_rows = 0;
		for (int m2 = 0; m2 < grid->num_row_; m2++)
			for (int m1 = 0; m1 < grid->num_col_; m1++){
				verts_cd.row(total_rows) << m1, m2, 1, 1;
				total_rows++;
			}
		verts_cd_ = verts_cd;
	}

private:
	std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph_;
	std::shared_ptr<Graph_t<BuchiState *, std::vector<int64_t>>> buchi_graph_;
	TileTraversalData tile_traversal_data_;
//	std::map<unsigned int,std::shared_ptr<Hlevel>> h_levels_;
	int history_H_;
	std::shared_ptr<SquareGrid> grid_;
	Eigen::MatrixXi verts_cd_;

public:
	std::vector<std::tuple<ProductState*, double>> operator()(ProductState* cell);
};
}


#endif /* PRODUCT_AUTOMATON_HPP */