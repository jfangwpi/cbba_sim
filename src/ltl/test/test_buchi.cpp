// C++ STL header
#include <stdio.h>
#include <vector>
#include <cstring>
#include <iostream>
#include <ctime>
#include <bitset>

// User-defined header
#include "ltl/buchi_automaton.hpp"

using namespace librav;

int main(int argc, char** argv )
{

	// std::cout << "Test Buchi Automata:\n" << std::endl;

	// //BuchiAutomaton buchi2;

	// std::vector<std::string> states1;
	// std::vector<std::string> states2;
	// std::vector<std::string> states3;
	// std::vector<std::string> states4;

	// std::string ltl_stm1 = "<> p1";
	// states1.push_back("p1");

	// std::string ltl_stm2 = "[]<>p0 || <>[]p1";
	// states2.push_back("p0");
	// states2.push_back("p1");

	// std::string ltl_stm3 = "([] p1) && ([] !p2) && (<> p3) && (<> p4)";
	// states3.push_back("p1");
	// states3.push_back("p2");
	// states3.push_back("p3");
	// states3.push_back("p4");

	// std::string ltl_stm4 = "([] p1) && ([] !p2) && (<> (p3 && <> p4))";
	// states4.push_back("p1");
	// states4.push_back("p2");
	// states4.push_back("p3");
	// states4.push_back("p4");

	// std::cout << "\n ------------ test case 1 ------------\n" << std::endl;
	// buchi2.GetBuchi(ltl_stm1);
	// buchi2.GetBuchi(ltl_stm1, states1);
//
//	std::cout << "\n ------------ test case 2 ------------\n" << std::endl;
////	buchi2.GetBuchi(ltl_stm2);
//	buchi2.GetBuchi(ltl_stm2, states2);
//
//	std::cout << "\n ------------ test case 3 ------------\n" << std::endl;
	// buchi2.GetBuchi(ltl_stm3);
	// buchi2.GetBuchi(ltl_stm3, states3);
//
//	std::cout << "\n ------------ test case 4 ------------\n" << std::endl;
////	buchi2.GetBuchi(ltl_stm4);
//	buchi2.GetBuchi(ltl_stm4, states4);

//	std::cout << "\n ------------ test case 5 ------------\n" << std::endl;
//	buchi2.GetBuchi(ltl_stm4);

//	buchi2.TranslateTransCon("(p1 && !p2 && p3) || (!p4)");
//	std::vector<std::string> states;
//	std::vector<uint32_t> ab_set;
//	states.push_back("p1");
//	states.push_back("p2");
//	states.push_back("p3");
//	states.push_back("p4");
//	buchi2.GetAlphabetSet(ab_set, states);
//	for(auto it = ab_set.begin(); it != ab_set.end(); it++)
//			std::cout<< std::bitset<32>(*it) << std::endl;

//	buchi2.GetBuchi(ltl_stm4, states);

	std::cout << "\n ------------ test buchi graph ------------\n" << std::endl;
//	Graph<BuchiState>* buchi_graph = buchi2.CreateBuchiGraph(ltl_stm3,states3);

	//BuchiAutomaton buchi_autaton;
	std::vector<std::string> buchi_regions;
	std::string ltl_formula = "([] p0) && ([] !p1) && (<> p2) && (<> p3)";
	//buchi_regions.push_back("p1");
	buchi_regions.push_back("p0");
	buchi_regions.push_back("p1");
	buchi_regions.push_back("p2");
	//buchi_regions.push_back("p3");
	std::shared_ptr<Graph_t<BuchiState *, std::vector<int64_t>>> buchi_graph = BuchiAutomaton::BuildBuchiGraph(ltl_formula,buchi_regions);

   
    /*** Test Buchi Graph ***/
    std::cout << "Initial state is " << (*buchi_graph->vertex_begin()).state_->init_state_idx_ << std::endl;
    std::cout << "Acc state is ";
    for (auto &it_acc: (*buchi_graph->vertex_begin()).state_->acc_state_idx)
        std::cout << it_acc << " " << std::endl;

	std::cout << "Alphabet set is " << std::endl;
	for (auto &as: (*buchi_graph->vertex_begin()).state_->alphabet_set)
		std::cout << as << " ";
	std::cout << std::endl;


    std::cout << "All edges info is " << std::endl;
    auto all_edges = buchi_graph->GetAllEdges();
    for (auto &e: all_edges){
        std::cout << "src state is " << (*e).src_->state_->id_<< std::endl;
        std::cout << "dst state is " << (*e).dst_->state_->id_<< std::endl;
        for(auto &cc: (*e).cost_)
            std::cout << cc << " ";
        std::cout << std::endl;
    }
        

    

//	buchi_autaton.GetBuchi(ltl_formula,buchi_regions);
//	Ltl2baWrapper ltl2ba_lib;
//	ba = ltl2ba_lib.GetBuchi(ltl_formula,buchi_regions);

//	delete buchi_graph;

    return 0;
}
