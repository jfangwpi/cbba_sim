#include <iostream>
#include <sstream>
#include <cstring>
#include <cstdint>
#include <map>
#include <vector>
#include <algorithm>
#include <bitset>

#include "ltl/product_automaton.hpp"

using namespace librav;

std::shared_ptr<Graph_t<ProductState *>> ProductAutomaton::BuildProductGraph(std::shared_ptr<Graph_t<SquareCell *>> GridGraph, std::shared_ptr<Graph_t<BuchiState *, std::vector<int64_t>>> BuchiGraph)
{
    std::shared_ptr<Graph_t<ProductState *>> product_graph = std::make_shared<Graph_t<ProductState *>>();
    // Obtain all grid vertex
    std::vector<Vertex_t<SquareCell *> *> grid_vertices = GridGraph->GetAllVertex();

    //Obtain all buchi vertex
    std::vector<Vertex_t<BuchiState *, std::vector<int64_t>> *> buchi_vertices = BuchiGraph->GetAllVertex();
    
    int num_buchi_states = buchi_vertices.size();
    std::vector<int64_t> alphabet_set = (*BuchiGraph->FindVertex(0)).state_->alphabet_set;

    int64_t node_2;

    for (auto gs = grid_vertices.begin(); gs!= grid_vertices.end(); gs++){
        // Find all neighbors
        std::vector<Vertex_t<SquareCell *> *> neighbor_from_grid = (*gs)->GetNeighbours();
        for (auto ngs = neighbor_from_grid.begin(); ngs != neighbor_from_grid.end(); ngs++){
            for (auto bs1 = buchi_vertices.begin(); bs1 != buchi_vertices.end(); bs1++){
                for (auto bs2 = buchi_vertices.begin(); bs2 != buchi_vertices.end(); bs2++){
                    std::vector<int64_t> buchi_transition = (*bs1)->GetEdgeCost(*bs2);
                    int32_t cell_bit_map = (*ngs)->state_->GetCellBitMap();
                    for(auto &e_buchi_trans: buchi_transition){
                        if(e_buchi_trans > alphabet_set.size()-1){
                            continue;
                        }
                        if(alphabet_set[e_buchi_trans] == cell_bit_map){
                            
                            int64_t node_1_id = (*gs)->state_->id_ * num_buchi_states + (*bs1)->state_->id_;
                            int64_t node_2_id = (*ngs)->state_->id_ * num_buchi_states + (*bs2) -> state_->id_;

                            ProductState* new_product_state_1 =  new ProductState(node_1_id);
                            new_product_state_1->grid_vertex_ = (*gs);
                            new_product_state_1->buchi_vertex_ = (*bs1);

                            ProductState* new_product_state_2 = new ProductState(node_2_id);
                            new_product_state_2->grid_vertex_ = (*ngs);
                            new_product_state_2->buchi_vertex_ = (*bs2);

                            product_graph->AddEdge(new_product_state_1,new_product_state_2,1.0);
                            break;

                            // ==================================== DEBUG ======================================================//
                            // std::cout << "Physical vertex transition is " << (*gs)->state_->id_ << " -> " << (*ngs) -> state_->id_ << std::endl;
                            // std::cout << "The dst bit map is " << cell_bit_map << std::endl;
                            // std::cout << "Buchi transition is " << (*bs1)->state_->id_ << " -> " << (*bs2)->state_->id_ << std::endl;
                            // std::cout << "The buchi transition is ";
                            // for (auto &e: buchi_transition){
                            //     std::cout << e << " ";
                            // }
                            // std::cout << std::endl;
                            // ==================================== DEBUG ======================================================//
                        }
                           
                    }
                                       
                }
            }
        }

    };

    return product_graph;
};

int64_t ProductAutomaton::SetVirtualStartState(std::shared_ptr<Graph_t<ProductState *>> product_graph, std::shared_ptr<Graph_t<BuchiState *, std::vector<int64_t>>> buchi_graph, std::shared_ptr<Graph_t<SquareCell* >> grid_graph, int64_t grid_start_id)
{
    int64_t virtual_start_id;

    // Buchi State
    std::vector<Vertex_t<BuchiState *, std::vector<int64_t>> *> buchi_vertices = buchi_graph->GetAllVertex();
    int num_buchi_states = buchi_vertices.size();
    int64_t buchi_init_id = (*buchi_graph->FindVertex(0)).state_->init_state_idx_;
    Vertex_t<BuchiState *, std::vector<int64_t>> * buchi_init = buchi_graph->GetVertexFromID(buchi_init_id);

    // Grid State
    Vertex_t<SquareCell *> * grid_init = grid_graph->GetVertexFromID(grid_start_id);

    virtual_start_id = grid_init->state_->id_ * num_buchi_states + buchi_init->state_->id_;

    ProductState* virtual_start = new ProductState(virtual_start_id);
    virtual_start->grid_vertex_ = grid_init;
    virtual_start->buchi_vertex_ = buchi_init;

    product_graph->AddVertex(virtual_start);
    return virtual_start_id;
};

int64_t ProductAutomaton::SetVirtualStartIncre(Vertex_t<SquareCell *> * start_node_origin, std::vector<int> rgn_idx_init, std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph, std::shared_ptr<Graph_t<BuchiState *, std::vector<int64_t>>> buchi_graph, std::shared_ptr<Graph_t<ProductState *>> product_graph){
    std::vector<Vertex_t<LiftedSquareCell* >*> start_node_lifted;
	for (auto it = start_node_origin->state_->lifted_vertices_id_.begin();it != start_node_origin->state_->lifted_vertices_id_.end(); it++){
        Vertex_t<LiftedSquareCell *> * lifted_cell = lifted_graph->GetVertexFromID(*it);
        start_node_lifted.push_back(lifted_cell);
	}


	auto init_buchi_idx_ = buchi_graph->GetVertexFromID(0)->state_->init_state_idx_;
	std::vector<Vertex_t<BuchiState *, std::vector<int64_t>> *> buchi_vertices = buchi_graph->GetAllVertex();
    int num_buchi_states = buchi_vertices.size();
	std::vector<int64_t> alphabet_set =buchi_graph->GetVertexFromID(0)->state_->alphabet_set;
	std::vector<ProductState *> start_product;
	for(auto it = start_node_lifted.begin(); it != start_node_lifted.end(); it++){
		int32_t RegionBitMap = (*it)->state_->history[0]->state_->GetCellBitMap();
		for (int i = 1; i < (*it)->state_->history.size(); i++){
			auto bitmap = (*it)->state_->history[i]->state_->GetCellBitMap();
			RegionBitMap = RegionBitMap|bitmap;
		}
        
		for (auto ite = buchi_vertices.begin();ite != buchi_vertices.end();ite++){
			std::vector<int64_t> buchi_transition = buchi_graph->GetVertexFromID(init_buchi_idx_)->GetEdgeCost(*ite);
			for(auto it_buchi_trans = buchi_transition.begin(); it_buchi_trans != buchi_transition.end(); it_buchi_trans++) {
				if (alphabet_set[(*it_buchi_trans)] == RegionBitMap){
					auto product_start_id = (*it)->vertex_id_*num_buchi_states + (*ite)->vertex_id_;

					ProductState* product_start = new ProductState(product_start_id);
					product_start->buchi_vertex_ = *ite;
					product_start->lifted_vertex_ = *it;
					product_start->rgn_idx_init_ = rgn_idx_init;
					start_product.push_back(product_start);
				}
			}
		}
	}
	int64_t virtual_start_id = buchi_graph->GetAllVertex().size()*lifted_graph->GetAllVertex().size();

	ProductState* virtual_start = new ProductState(virtual_start_id);

	virtual_start->buchi_vertex_ = buchi_graph->GetVertexFromID(init_buchi_idx_);
	for (auto it = start_product.begin(); it!=start_product.end();it++){
		product_graph->AddEdge(virtual_start, (*it), 1.0);
	}


    return virtual_start_id;
}

GetProductNeighbor::GetProductNeighbor(std::shared_ptr<Graph_t<SquareCell *>> GridGraph, std::shared_ptr<Graph_t<BuchiState *, std::vector<int64_t>>> BuchiGraph):
grid_graph_(GridGraph),
buchi_graph_(BuchiGraph){};


void GetProductNeighbor::InfoCheck()
{
    std::vector<Vertex_t<BuchiState *, std::vector<int64_t>> *> buchi_vertices = buchi_graph_->GetAllVertex();
    for (auto &e: buchi_vertices){
        std::cout << e->state_->id_ << " ";
    }
    std::cout << std::endl;
}


std::vector<std::tuple<ProductState*, double>> GetProductNeighbor::operator()(ProductState* cell)
{
    std::vector<std::tuple<ProductState*, double>> neighbors;

    //Obtain all buchi vertex
    std::vector<Vertex_t<BuchiState *, std::vector<int64_t>> *> buchi_vertices = buchi_graph_->GetAllVertex();
    
    int num_buchi_states = buchi_vertices.size();
    std::vector<int64_t> alphabet_set = (*buchi_graph_->FindVertex(0)).state_->alphabet_set;

    Vertex_t<SquareCell *>* current_grid_state =  cell->grid_vertex_;
    std::vector<Vertex_t<SquareCell *> *> neighbor_from_grid = current_grid_state->GetNeighbours();
    Vertex_t<BuchiState *, std::vector<int64_t>> * current_buchi_state = cell->buchi_vertex_;

    for (auto ngs = neighbor_from_grid.begin(); ngs != neighbor_from_grid.end(); ngs++){
        for (auto bs = buchi_vertices.begin(); bs != buchi_vertices.end(); bs++){
            std::vector<int64_t> buchi_transition = (current_buchi_state)->GetEdgeCost(*bs);
            int32_t cell_bit_map = (*ngs)->state_->GetCellBitMap();
            for(auto &e_buchi_trans: buchi_transition){
                if(e_buchi_trans > alphabet_set.size()-1){
                    continue;
                }
                if(alphabet_set[e_buchi_trans] == cell_bit_map){
                        
                    int64_t new_node_id = (*ngs)->state_->id_ * num_buchi_states + (*bs) -> state_->id_;
                    ProductState* new_product_state = new ProductState(new_node_id);
                    new_product_state->grid_vertex_ = (*ngs);
                    new_product_state->buchi_vertex_ = (*bs);

                    neighbors.emplace_back(new_product_state,1.0);
                
                    // //==================================== DEBUG ======================================================//
                    // std::cout << "Physical vertex transition is " << cell->grid_vertex_->state_->id_ << " -> " << (*ngs) -> state_->id_ << std::endl;
                    // std::cout << "The dst bit map is " << cell_bit_map << std::endl;
                    // std::cout << "Buchi transition is " << cell->buchi_vertex_->state_->id_ << " -> " << (*bs)->state_->id_ << std::endl;
                    // std::cout << "The buchi transition is ";
                    // for (auto &e: buchi_transition){
                    //     std::cout << e << " ";
                    // }
                    // std::cout << std::endl;
                    // //==================================== DEBUG ======================================================//
                    break;
                }
                           
            }
                                       
        }
    }
    return neighbors;
}

std::vector<std::tuple<ProductState*, double>> GetProductCBTANeighbour::operator()(ProductState* cell){
	std::vector<std::tuple<ProductState*, double>> adjacent_cells;
	std::vector<int64_t> alphabet_set = buchi_graph_->GetVertexFromID(0)->state_->alphabet_set;
	std::vector<int64_t> buchi_transition;
	Vertex_t<SquareCell*>* last_cell;
	int64_t cell_region_bit_map;
	int64_t node_id_new;
	double edge_cost;
	std::vector<int> rgn_idx_init = cell->rgn_idx_init_;
	Vertex_t<LiftedSquareCell *>* current_lifted_vertex = cell->lifted_vertex_;
	Vertex_t<BuchiState*,std::vector<int64_t>>* current_buchi_vertex = cell->buchi_vertex_;
	std::vector<Vertex_t<LiftedSquareCell *>*> neighbour_from_lift = current_lifted_vertex->GetNeighbours();
	std::vector<Vertex_t<BuchiState *, std::vector<int64_t>>*> buchi_vertices = buchi_graph_->GetAllVertex();
	int num_buchi_states = buchi_vertices.size();

	Eigen::MatrixXi tile_vertices(history_H_ + 2,4);
	for (auto it_lifted_neighbour = neighbour_from_lift.begin(); it_lifted_neighbour!= neighbour_from_lift.end(); it_lifted_neighbour++){
		for (auto it_buchi = buchi_vertices.begin();it_buchi != buchi_vertices.end();it_buchi++){
			buchi_transition = current_buchi_vertex->GetEdgeCost(*it_buchi);
			last_cell = (*it_lifted_neighbour)->state_->history.back();
			cell_region_bit_map = last_cell->state_->GetCellBitMap();
			for(auto it_buchi_trans = buchi_transition.begin(); it_buchi_trans != buchi_transition.end(); it_buchi_trans++) {
				if ((*it_buchi_trans)>alphabet_set.size()-1)
					continue;
				if (alphabet_set[(*it_buchi_trans)] == cell_region_bit_map){
					std::vector<int64_t> vertSeq;
					for (auto it = current_lifted_vertex->state_->history.begin(); it != current_lifted_vertex->state_->history.end(); it++){
						vertSeq.push_back((*it)->state_->GetUniqueID());
					}
					vertSeq.push_back(last_cell->state_->GetUniqueID());
					for (int ii = 0; ii < history_H_ + 2; ii++){
						tile_vertices.row(ii) << verts_cd_.row(vertSeq[ii]);
					}
					TransitionData transition_data = HCost::get_lifted_transition(history_H_, tile_vertices, rgn_idx_init, tile_traversal_data_.Hlevels, tile_traversal_data_.N_REGION_TOTAL);
					edge_cost = transition_data.cost;
					if (edge_cost >1e6){
						continue;
					}
					node_id_new = (*it_lifted_neighbour)->vertex_id_*num_buchi_states + (*it_buchi)->vertex_id_;
					ProductState* new_product_state = new ProductState(node_id_new);
					new_product_state->lifted_vertex_ = *it_lifted_neighbour;
					new_product_state->buchi_vertex_ = *it_buchi;
					new_product_state->rgn_idx_init_ = transition_data.rgn_idx_next;
					adjacent_cells.emplace_back(new_product_state, edge_cost);
					break;
				}
			}
		}
	}
	return adjacent_cells;
}
