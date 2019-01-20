#ifndef BUCHI_AUTOMATON_HPP
#define BUCHI_AUTOMATON_HPP

#include <cstring>
#include <cstdint>
#include <vector>
#include <memory>

#include "graph/graph.hpp"
#include "ltl/region_label.hpp"
#include "ltl/spot_hoa_interpreter.hpp"

namespace librav{

/// BuchiState is used to construct graph of buchi automata.
class BuchiState
{
	public:
		BuchiState(int64_t _id):
			id_(_id),
			init_state_idx_(0)
			{
				alphabet_set.empty();
				acc_state_idx.empty();
			};
		~BuchiState(){};

	public:
		std::vector<int64_t> alphabet_set;
		std::vector<int64_t> acc_state_idx;
		int64_t init_state_idx_;
		int64_t id_;
//	std::map<spot::formula, int> region_to_id;

		int64_t GetUniqueID() const{return id_;};
		double GetHeuristic(const BuchiState& other_struct) const {
			return 0.0;
	
		};
};

/// BuchiAutomata class contains functions to create a Buchi graph from LTL statement.
class BuchiAutomaton{

public:
	static std::shared_ptr<Graph_t<BuchiState *, std::vector<int64_t>>> BuildBuchiGraph(std::string ltl_str, std::vector<std::string> ltl_states);

};

}

#endif /* BUCHI_AUTOMATON_HPP*/
