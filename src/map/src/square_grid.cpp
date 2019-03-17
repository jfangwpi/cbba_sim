#include "map/square_grid.hpp"
#include "config_reader/config_reader.hpp"
#include <vector>

using namespace librav;

void SquareCell::UpdateMapInfo(int32_t row_size, int32_t col_size, double side_size, int32_t pixel_per_meter)
{
    int32_t vis_side_size = side_size * pixel_per_meter;

    bbox_.x.min = coordinate_.x * vis_side_size;
    bbox_.x.max = bbox_.x.min + vis_side_size - 1;
    bbox_.y.min = coordinate_.y * vis_side_size;
    bbox_.y.max = bbox_.y.min + vis_side_size - 1;

    //std::cout << "The state id is " << id_ << std::endl;
    position_.x = coordinate_.x * vis_side_size + vis_side_size/2;
    //std::cout << "pos x is " << coordinate_.x << std::endl;
	position_.y = coordinate_.y * vis_side_size + vis_side_size/2;
    //std::cout << "pos y is " << coordinate_.y << std::endl;
}

std::string SquareCell::GetCellLabels(){
    return cell_labels_.GetCellLabels();
}

int32_t SquareCell::GetCellBitMap(){
    return cell_labels_.GetCellBitMap();
}

SquareGrid::SquareGrid(int32_t row_num, int32_t col_num, double cell_size, int32_t pixel_per_meter, int32_t default_label):
                num_row_(row_num),
                num_col_(col_num),
                cell_size_(cell_size),
                pixel_per_meter_(pixel_per_meter)
{
    grid_cells_.resize(col_num);

    for (auto &grid_col : grid_cells_)
        grid_col.resize(row_num);

    for (int32_t y = 0; y < row_num; y++){
        for (int32_t x = 0; x < col_num; x++){
            int64_t new_id = y*col_num + x;
            SquareCell *new_cell = new SquareCell(new_id, x, y, OccupancyType::FREE);
            grid_cells_[x][y] = new_cell;
            grid_cells_[x][y]->UpdateMapInfo(num_row_, num_col_, cell_size_, pixel_per_meter_);
           
            grid_cells_[x][y]->cell_labels_.SetDefaultRegionLabel(default_label);
        }
    }

}

SquareGrid::~SquareGrid()
{
    // Remove all Square cells
    for (auto &grid_col: grid_cells_){
        for (auto &cell: grid_col)
            delete cell;
    }
}

Position2D SquareGrid::GetCoordinateFromID(int64_t id)
{
    int32_t y = id/num_col_;
    int32_t x = id%num_col_;

    return Position2D(x,y);

}

Position2D SquareGrid::GetRealCoordinateFromID(int64_t id, double real_side_length)
{
    double y_real = (id/num_col_)*real_side_length/num_row_;
    double x_real = (id%num_col_)*real_side_length/num_col_;
    //coord.push_back(x_real);
    //coord.push_back(y_real);
    return Position2D(x_real,y_real);

}

void SquareGrid::SetCellOccupancy(int32_t x_col, int32_t y_row, OccupancyType occ)
{
    grid_cells_[x_col][y_row]->occu_ = occ;
}

void SquareGrid::SetCellOccupancy(int64_t id, OccupancyType occ)
{
    Position2D coord = GetCoordinateFromID(id);

    SetCellOccupancy(coord.x, coord.y, occ);
}

int64_t SquareGrid::GetIDFromCoordinate(int32_t x_col, int32_t y_row)
{
    return y_row * num_col_ + x_col;
}

int64_t SquareGrid::GetIDFromRealCoordinate(double x_col_real, double y_row_real, double real_side_length)
{
    double y_sim = y_row_real * (num_row_/real_side_length);
    double x_sim = x_col_real * (num_col_/real_side_length);

    return y_sim * num_col_ + x_sim;
}

SquareCell* SquareGrid::GetCellFromID(int64_t id)
{
    Position2D coord = GetCoordinateFromID(id);

    return grid_cells_[coord.x][coord.y];
}

std::vector<SquareCell*> SquareGrid::GetNeighbors(int64_t id, bool allow_diag)
{
    auto cell = GetCellFromID(id);

    return GetNeighbors(cell->coordinate_.x, cell->coordinate_.y, allow_diag);
}

std::vector<SquareCell *> SquareGrid::GetNeighbors(int32_t x_col, int32_t y_row, bool allow_diag)
{
    std::vector<SquareCell *> neighbors;

    if (allow_diag)
    {
        for (int32_t x = x_col - 1; x <= x_col + 1; ++x)
        {
            for (int32_t y = y_row - 1; y <= y_row + 1; ++y)
            {
                if (x == x_col && y == y_row)
                    continue;

                if (x >= 0 && x < num_col_ && y >= 0 && y < num_row_)
                    neighbors.push_back(grid_cells_[x][y]);
            }
        }
    }
    else
    {
        Position2D pos[4];

        pos[0].x = x_col;
        pos[0].y = y_row + 1;

        pos[1].x = x_col;
        pos[1].y = y_row - 1;

        pos[2].x = x_col + 1;
        pos[2].y = y_row;

        pos[3].x = x_col - 1;
        pos[3].y = y_row;

        for (int i = 0; i < 4; i++)
        {
            if (pos[i].x >= 0 && pos[i].x < num_col_ &&
                pos[i].y >= 0 && pos[i].y < num_row_){
                    neighbors.push_back(grid_cells_[pos[i].x][pos[i].y]);
            }
        }
    }

    // for (auto &ne: neighbors){
    //     std::cout << "Neighbor is " << ne->id_ << std::endl;
    //     std::cout << "Info about bit map is " << ne->GetCellBitMap() << std::endl;
    // }

    return neighbors;

}

void SquareGrid::SetObstacleRegionLabel(int64_t id, int8_t label){
    this->SetCellOccupancy(id, OccupancyType::OCCUPIED);

    SquareCell* current_cell = this->GetCellFromID(id);
    
    current_cell->cell_labels_.RemoveRegionLabel(current_cell->cell_labels_.GetDefaultRegionLabel());

    current_cell->cell_labels_.AssignRegionLabel(label);
}

void SquareGrid::SetBaseRegionLabel(int64_t id, int8_t label){
    this->SetCellOccupancy(id, OccupancyType::INTERESTED);

    SquareCell* current_cell = this->GetCellFromID(id);
    current_cell->cell_labels_.AssignRegionLabel(label);
}

void SquareGrid::SetInterestedRegionLabel(int64_t id, int8_t label){
    this->SetCellOccupancy(id, OccupancyType::INTERESTED);
    SquareCell* current_cell = this->GetCellFromID(id);
    current_cell->cell_labels_.AssignRegionLabel(label);
}


std::shared_ptr<SquareGrid> GraphFromGrid::CreateSquareGrid(int32_t row_size, int32_t col_size, double cell_size)
{
    return std::make_shared<SquareGrid>(row_size,col_size,cell_size);
}

std::shared_ptr<SquareGrid> GraphFromGrid::CreateSquareGrid()
{
    
    ConfigReader config_reader("../../src/config/map.ini");
    if (config_reader.CheckError()){
        std::cout << "Reading config file failed." << std::endl;
    }

    int32_t num_row = config_reader.GetReal("grid_row", 0);
    int32_t num_col = config_reader.GetReal("grid_column", 0);
    double cell_size = config_reader.GetReal("cell_size", 0.1);
    int32_t pixel_per_meter = config_reader.GetReal("pixel_per_meter", 100);
    int32_t default_label = config_reader.GetReal("default_label", 0);

    std::shared_ptr<SquareGrid> grid = std::make_shared<SquareGrid>(num_row, num_col, cell_size, pixel_per_meter, default_label);

    int32_t num_obs_block = config_reader.GetReal("obs_block_num", 0);
    for (int i = 0; i < num_obs_block; i++){
        std::string obs_block_start = "obs"+std::to_string(i)+"_start";
        std::string obs_block_end = "obs"+std::to_string(i)+"_end";
        int32_t obs_start = config_reader.GetReal(obs_block_start, 0);
        int32_t obs_end = config_reader.GetReal(obs_block_end, 0);
        for (int j = obs_start; j < obs_end; j++){
            grid->SetObstacleRegionLabel(j,1);
        }
    }

    int32_t num_obs_cell = config_reader.GetReal("obs_cell_num", 0);
    for(int i = 0; i < num_obs_cell; i++){
        std::string obs_cell_str = "obs"+std::to_string(i);
        int obs_cell = config_reader.GetReal(obs_cell_str, 0);
        grid->SetObstacleRegionLabel(obs_cell,1);
    }


    return  grid;
}


std::shared_ptr<Graph_t<SquareCell *>> GraphFromGrid::BuildGraphFromSquareGrid(std::shared_ptr<SquareGrid> grid, bool allow_diag_move, bool ignore_obs)
{
    std::shared_ptr<Graph_t<SquareCell *>> graph = std::make_shared<Graph_t<SquareCell *>>();

    if (ignore_obs == false){
        for (auto &cell_col : grid->grid_cells_){
		    for (auto &cell : cell_col)
		    {
			    if(cell->occu_ != OccupancyType::OCCUPIED){
                    int64_t current_nodeid = cell->id_;
			        std::vector<SquareCell *> neighbour_list = grid->GetNeighbors(current_nodeid, allow_diag_move);

			        for (auto &neighbour : neighbour_list)
			        {
                        if(neighbour->occu_ != OccupancyType::OCCUPIED){
                            double error_x, error_y, cost = 0;
				            error_x = std::abs(neighbour->position_.x - cell->position_.x);
				            error_y = std::abs(neighbour->position_.y - cell->position_.y);
				            cost = std::sqrt(error_x * error_x + error_y * error_y);

				            graph->AddEdge(cell, neighbour, cost);
                        }
			        }		
                }
            
		    }
        }
    }
    else{
        for (auto &cell_col : grid->grid_cells_){
		    for (auto &cell : cell_col){
                int64_t current_nodeid = cell->id_;
			    std::vector<SquareCell *> neighbour_list = grid->GetNeighbors(current_nodeid, allow_diag_move);

			    for (auto &neighbour : neighbour_list){
                    double error_x, error_y, cost = 0;
				    error_x = std::abs(neighbour->position_.x - cell->position_.x);
				    error_y = std::abs(neighbour->position_.y - cell->position_.y);
				    cost = std::sqrt(error_x * error_x + error_y * error_y);

				    graph->AddEdge(cell, neighbour, cost);
			    }		
		    }
        }
    }


	return graph;
}



    