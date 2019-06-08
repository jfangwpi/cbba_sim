/*
 *  create_json_file.cpp
 *
 *  Created on: May 30th, 2019
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
#include <memory>
#include <math.h>
// opencv
#include "opencv2/opencv.hpp"
#include <iostream>
#include <fstream>

// user
#include "graph/graph.hpp"
#include "graph/algorithms/astar.hpp"
#include "map/square_grid.hpp"
#include "vis/graph_vis.hpp"

#include "ltl/buchi_automaton.hpp"
#include "ltl/product_automaton.hpp"
#include "cbta/graph_lift.hpp"
#include "cbta/hcost_interface.hpp"

#include "nlohmann/json.hpp"
#include "cbta/json_access.hpp"

using namespace cv;
using namespace librav;

int main(int argc, char** argv )
{
	int start_hlevel = 1;
	int end_hlevel = 3;
	jsonReadWrite::get_to_json(start_hlevel, end_hlevel, 3);
	return 0;
}

