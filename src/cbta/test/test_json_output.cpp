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

#include "nlohmann/json.hpp"

using nlohmann::json;
	// void to_json(json& j, const TileTraversalData& data){
	// 	j = json{"Hlevels", data.Hlevels};
	// 	std::string test_string = j.dump();
	// 	std::cout << test_string << std::endl;
	// }



using namespace cv;
using namespace librav;

int main(int argc, char** argv )
{
	/*** 0. Preprocessing CBTA data ***/
	TileTraversalData tile_traversal_data = HCost::hcost_preprocessing();
	const TileTraversalData& ttd = tile_traversal_data;

	using json = nlohmann::json;

	json j;
		j["region_w_lower"] = ttd.region_bd.region_w_lower;
		j["region_w_upper"] = ttd.region_bd.region_w_upper;
		j["region_psi_lower"] = ttd.region_bd.region_psi_lower;
		j["region_psi_upper"] = ttd.region_bd.region_psi_upper;
		j["region_vel_lower"] = ttd.region_bd.region_vel_lower;
		j["region_vel_upper"] = ttd.region_bd.region_vel_upper;

	j["N_REGION_W"] = tile_traversal_data.N_REGION_W;
	j["N_REGION_PSI"] = tile_traversal_data.N_REGION_PSI;
	j["N_REGION_SPD"] = tile_traversal_data.N_REGION_SPD;
	j["N_REGION_TOTAL"] = tile_traversal_data.N_REGION_TOTAL;


	std::string s = j.dump();
	std::cout << "------- JSON Output -------" << std::endl;
	std::cout << s << std::endl;



	return 0;
}
