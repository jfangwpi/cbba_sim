/*
 *  json_access.hpp
 *
 *  area to determine source of error when 
 * 	running CBTA on the Raspberry Pi
 * 
 *  Created on: June 4th, 2019
 *      Author: Jack O'Neill
 *      Questions/comments: (203) 558-2221
 */

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

using namespace Eigen;
using namespace librav;

int main(int argc, char** argv){


std::shared_ptr<Eigen::SparseMatrix<int,Eigen::RowMajor>> connectivity = std::make_shared<Eigen::SparseMatrix<int,Eigen::RowMajor>>(2500,2500);



}
