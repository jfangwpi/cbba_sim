#ifndef GRAPH_LIFT_HPP
#define GRAPH_LIFT_HPP

#include <iostream>
#include <sstream>
#include <cstring>
#include <cstdint>
#include <map>
#include <vector>
#include <algorithm>
#include <memory>

#include "map/square_grid.hpp"
#include "graph/graph.hpp"
#include "graph/vertex.hpp"
#define EIGEN_DONT_ALIGN_STATICALLY


namespace librav{
    class LiftedSquareCell{
        public:
            LiftedSquareCell(int64_t id):id_(id){};
            ~LiftedSquareCell(){};

        public:
            int64_t id_;
            std::vector<Vertex_t<SquareCell* > *> history;

            double GetHeuristic(const LiftedSquareCell& other_cell) const{
                return 0.0;
            }

            int64_t GetUniqueID() const {return id_;};

    };

    class GraphLifter{

        public:
            static std::shared_ptr<Graph_t<LiftedSquareCell *>> BuildLiftedGraph(int historyH, std::shared_ptr<Graph_t<SquareCell *>> graph);
            static std::vector<std::vector<Vertex_t<SquareCell *> *>> GetHistories(Vertex_t<SquareCell *>* actNode, int historyH);
    };
}

#endif /* GRAPH_LIFT_HPP */