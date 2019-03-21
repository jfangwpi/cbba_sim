#ifndef SQUARE_GRID_HPP
#define SQUARE_GRID_HPP

#include <map>
#include <cmath>
#include <vector>
#include <cstdint>
#include <string>
#include <memory>
#include <vector>

#include "map/common_types.hpp"
#include "ltl/cell_label.hpp"
#include "graph/graph.hpp"


namespace librav{
    class SquareCell{
        public: 
            SquareCell(int64_t id, int32_t col, int32_t row, OccupancyType _occu):
                id_(id),
                occu_(_occu)
            {
                // Coordinate in the square grid map
                coordinate_.x = col;
                coordinate_.y = row;
            }
            ~SquareCell(){};
        
            // abstract square cell attributes
            int64_t id_;
            Position2D coordinate_;
            OccupancyType occu_;

            // additional information when associated with an image map
            Position2D position_;
            PhysicalPosition2D physical_position_;
            BoundingBox<int32_t> bbox_;

            // For buchi automaton
            CellLabel cell_labels_;

            // For lifted graph
            std::vector<int64_t> lifted_vertices_id_;

            // Information Gain
            float info_gain_;
           
        public:
            double GetHeuristic(const SquareCell& other_cell) const{
                double x1, x2, y1, y2;

                // const after function name means the "this" point is constant
                x1 = this->position_.x;
                y1 = this->position_.y;

                x2 = other_cell.position_.x;
                y2 = other_cell.position_.y;

                long x_error = static_cast<long>(x1) - static_cast<long>(x2);
                long y_error = static_cast<long>(y1) - static_cast<long>(y2);

                double cost = std::abs(x_error) + std::abs(y_error);

                //std::cout << "The heuristic cost is " << cost << std::endl;

                return cost;
            }

            int64_t GetUniqueID() const {return id_;};
            void UpdateMapInfo(int32_t row_size, int32_t col_size, double side_size, int32_t pixel_per_meter);

            std::string GetCellLabels();
            int32_t GetCellBitMap();



    };

    class SquareGrid{
        public:
            SquareGrid(int32_t row_num, int32_t col_num, double cell_size = 0.1, int32_t pixel_per_meter = 100, int32_t default_label = 0);
            
            ~SquareGrid();

            friend class SquareCell;

        public: 
            std::vector<std::vector<SquareCell *>> grid_cells_;

        public: 
            int32_t num_row_;
            int32_t num_col_;
            double cell_size_;
            int32_t pixel_per_meter_;

        
            
        public: 
            Position2D GetCoordinateFromID(int64_t id);
            PhysicalPosition2D GetRealCoordinateFromID(int64_t id, double real_side_length);
            void SetCellOccupancy(int32_t x_col, int32_t y_row, OccupancyType occ);
            void SetCellOccupancy(int64_t id, OccupancyType occ);

            int64_t GetIDFromCoordinate(int32_t x_col, int32_t y_row);
            int64_t GetIDFromRealCoordinate(double x_col_real, double y_row_real, double real_side_length);
	        SquareCell* GetCellFromID(int64_t id);

	        std::vector<SquareCell*> GetNeighbors(int64_t id, bool allow_diag);
            std::vector<SquareCell*> GetNeighbors(int32_t x_col, int32_t y_row, bool allow_diag);

            /*** Set region label***/
            void SetObstacleRegionLabel(int64_t id, int8_t label);
            void SetBaseRegionLabel(int64_t id, int8_t label);
            void SetInterestedRegionLabel(int64_t id, int8_t label);
       
    };

    class GraphFromGrid{

        public:
            // Create gird map with given row and colum num
            static std::shared_ptr<SquareGrid> CreateSquareGrid(int32_t row_size, int32_t col_size, double cell_size);
            // Create grid map by reading from config file: map.ini
            static std::shared_ptr<SquareGrid> CreateSquareGrid();

            // Create a graph based on the given grid
	        static std::shared_ptr<Graph_t<SquareCell *>> BuildGraphFromSquareGrid(std::shared_ptr<SquareGrid> grid, bool allow_diag_move, bool ignore_obs);
            
    };

}


#endif /* SQUARE_GRID_HPP */