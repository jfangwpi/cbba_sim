#include <iostream>
#include <stdio.h>
#include <vector>
#include <ctime>
#include <tuple>
#include <algorithm>
#include <bitset>
#include <map>

#include "cbba/cbba_agent.hpp"

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/cbba_lcm_msgs.hpp"


namespace librav{
    class MSGHandler 
    {
        public:
            std::vector<int> idx;
            std::map<int, std::vector<float>> y_his;
            std::map<int, std::vector<int>> z_his;
            std::map<int, std::vector<int>> iter_nei_his;
            std::map<int, std::vector<std::vector<float>>> bids_matrix;
            std::map<int, std::vector<std::vector<int>>> assignment_matrix;
            std::map<int, bool>  is_data_received_self;
            std::map<int, bool> is_data_received_neigh;

            std::vector<int> neigh_list;

            bool is_all_data_received_self = false;
            bool is_all_data_received_neigh = false;

        public:
            MSGHandler(std::vector<int> neig_idx);
            ~MSGHandler() {};

            void handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const cbba_lcm_msgs::Agent* msg);
            void MSGExchange(cbba_Agent& agent);
            void ReceiveItselfCheck();
            void ReceiveNeighCheck();

    };
}