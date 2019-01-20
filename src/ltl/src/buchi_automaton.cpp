#include <iostream>
#include <sstream>
#include <algorithm>
#include <iomanip>
#include <bitset>
#include <cmath>

#include "ltl/buchi_automaton.hpp"

using namespace librav;
std::shared_ptr<Graph_t<BuchiState *, std::vector<int64_t>>> BuchiAutomaton::BuildBuchiGraph(std::string ltl_str, std::vector<std::string> ltl_states)
{
	SpotHoaInterpreter ltl2ba_lib;

	// generate buchi data structure using ltl2ba library
	BAStruct ba = ltl2ba_lib.GetBuchi(ltl_str,ltl_states);

	std::vector<BuchiState *> buchi_states;

	// create new buchi graph
	std::shared_ptr<Graph_t<BuchiState *, std::vector<int64_t>>> buchi_graph = std::make_shared<Graph_t<BuchiState *, std::vector<int64_t>>>();
	for(int i = 0; i < ba.state_num; i++) {
		auto buchi_state = new BuchiState(static_cast<int64_t>(i));
        for (auto &alp: ba.alphabet_set){
            (*buchi_state).alphabet_set.push_back(static_cast<int64_t>(alp));
        }
 		
        for (auto &acc: ba.acc_state_idx){
            (*buchi_state).acc_state_idx.push_back(static_cast<int64_t>(acc));
        }

 		(*buchi_state).init_state_idx_ = static_cast<int64_t> (ba.init_state_idx);
// //		buchi_state.region_to_id = ba.region_to_id;
 		buchi_states.push_back(buchi_state);
	}

	for(int i = 0; i < ba.state_num; i++) {
	 	for(int j = 0; j < ba.state_num; j++)
	 	{
	 		std::vector<int64_t> ref;
            for (auto &tc_: ba.trans_con[i][j]){
                ref.push_back(static_cast<int64_t>(tc_));
            };
	 		if (!ba.trans_con[i][j].empty()) {
	 			buchi_graph->AddEdge(buchi_states[i], buchi_states[j], ref);
	 		}
	 	}
	}

	return buchi_graph;
}