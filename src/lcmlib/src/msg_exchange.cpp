// standard library
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

#include "lcmlib/msg_exchange.hpp"
#include "cbba/cbba_agent.hpp"


using namespace librav;

MSGHandler::MSGHandler(std::vector<int> neig_idx):
    neigh_list(neig_idx)
    {
    // Initialize every parameters
    idx.clear();
    y_his.clear();
    z_his.clear();
    iter_nei_his.clear();
    bids_matrix.clear();
    assignment_matrix.clear();
    is_data_received_self.clear();
    is_data_received_neigh.clear();

    for (auto &id: neig_idx){
        idx.push_back(id);
        y_his[id] = {};
        z_his[id] = {};
        iter_nei_his[id] = {};
        is_data_received_self[id] = false;
        is_data_received_neigh[id] = false;
    }
};

void MSGHandler::handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const cbba_lcm_msgs::Agent* msg)
{
    // update self data
    //std::cout << "SELF MESSAGE FROM " << msg->idx << " RECEIVED." << std::endl;
    is_data_received_self[msg->idx] = true;
    if(msg->is_data_received_self == 1){
        is_data_received_neigh[msg->idx] = true;
        //std::cout << "NEIGHBOR " << msg->idx <<" MESSAGE RECEIVED TOO" << std::endl;
    }
    else
        is_data_received_neigh[msg->idx] = true;
    
    std::vector<float> y_his_sig = {};
    for (auto it=msg->y_his.begin();it !=msg->y_his.end(); it++){
        y_his_sig.push_back(*it);
    }
    y_his[msg->idx] = y_his_sig;

    std::vector<int> z_his_sig = {};
    for(auto it=msg->z_his.begin();it !=msg->z_his.end(); it++){
        z_his_sig.push_back(*it);
    }
    z_his[msg->idx] = z_his_sig;

    std::vector<std::vector<float>> bids_ = {};
    for(auto it_task = msg->bids_matrix.begin(); it_task!= msg->bids_matrix.end(); it_task++){
        std::vector<float> bids_per_task = {};
        for (auto it_veh = (*it_task).begin(); it_veh != (*it_task).end(); it_veh++){
            bids_per_task.push_back((*it_veh));
        }
        bids_.push_back(bids_per_task);
    }
    bids_matrix[msg->idx] = bids_;

    std::vector<std::vector<int>> assignment_ = {};
    for(auto it_task = msg->assignment_matrix.begin(); it_task!= msg->assignment_matrix.end(); it_task++){
        std::vector<int> assignment_per_task = {};
        for (auto it_veh = (*it_task).begin(); it_veh != (*it_task).end(); it_veh++){
            assignment_per_task.push_back((*it_veh));
        }
        assignment_.push_back(assignment_per_task);
    }
    assignment_matrix[msg->idx] = assignment_;

    std::vector<int> iter_nei_his_sig = {};
    for (auto it=msg->iter_nei_his.begin(); it != msg->iter_nei_his.end(); it++){
        iter_nei_his_sig.push_back(*it);
    }
    iter_nei_his[msg->idx] = iter_nei_his_sig;
};

void MSGHandler::MSGExchange(cbba_Agent& agent){
    lcm::LCM lcm;
    for (auto &neig_idx: neigh_list){
        std::string chanel_sub = "Chanel"+std::to_string(neig_idx)+std::to_string(agent.idx_);
        // std::cout << "Subscribe chanel is " << chanel_sub << std::endl;
        lcm.subscribe(chanel_sub, &MSGHandler::handleMessage, this);
    }
    

    while(is_all_data_received_self==false || is_all_data_received_neigh == false){
              
        lcm.handleTimeout(1);

        if (!lcm.good())
            break;
        
        cbba_lcm_msgs::Agent sender_his;
        
        sender_his.idx = agent.idx_;
        sender_his.num_tasks_inde = agent.num_tasks_inde_;
        sender_his.num_tasks_de = agent.num_tasks_de_;
        sender_his.num_tasks = agent.num_tasks_;
        sender_his.num_agent = agent.num_agents_;
        sender_his.y_his.resize(sender_his.num_tasks);
        sender_his.z_his.resize(sender_his.num_tasks);
        sender_his.bids_matrix.resize(sender_his.num_tasks, std::vector<float>(sender_his.num_agent));
        sender_his.assignment_matrix.resize(sender_his.num_tasks, std::vector<int>(sender_his.num_agent));
        for (int i = 0; i < sender_his.num_tasks; i++){
            sender_his.y_his[i] = agent.history_.y_history.back()[i];
            sender_his.z_his[i] = agent.history_.z_history.back()[i];
        }
        //std::cout << "SKR SKR SKR SKR SKR " << std::endl;

        for (int t_idx = 0; t_idx < sender_his.num_tasks; t_idx++){
            std::vector<float> bids_per_task = {};
            std::vector<int> assignment_per_task = {};
            for (int v_idx=0; v_idx < sender_his.num_agent; v_idx++){
                //std::cout << "t_idx is " << t_idx << " and v_idx is " << v_idx << std::endl;
                //std::cout << agent.history_.winning_bids_.back()(t_idx, v_idx) << std::endl;
                bids_per_task.push_back(agent.history_.winning_bids_.back()(t_idx, v_idx));
                assignment_per_task.push_back(agent.history_.assignment_.back()(t_idx, v_idx));
            }
            sender_his.bids_matrix[t_idx] = bids_per_task;
            sender_his.assignment_matrix[t_idx] = assignment_per_task;
        }
        //std::cout << "SKR SKR SKR SKR SKR 2" << std::endl;

        sender_his.iter_nei_his.resize(sender_his.num_agent);
        for(int j = 0; j < sender_his.num_agent; j++){
            sender_his.iter_nei_his[j] = agent.history_.iter_neighbors_his.back()[j];
        }

        for (auto &neig_idx: neigh_list){
            std::string chanel_pub = "Chanel"+std::to_string(agent.idx_)+std::to_string(neig_idx);
            // std::cout << "Publish chanel is " << chanel_pub << std::endl;

            if (is_data_received_self[neig_idx] == false)
                sender_his.is_data_received_self = 0;
            else
                sender_his.is_data_received_self = 1;

            lcm.publish(chanel_pub, &sender_his); 
        }

        // Update the data
        ReceiveItselfCheck();
        ReceiveNeighCheck();
    }
}

void MSGHandler::ReceiveItselfCheck(){
    int num_receive = 0;
    for (auto &it:is_data_received_self){
        if (it.second == true){
            // std::cout << "Data from vehcile " << it.first << " is received" << std::endl;
            num_receive++;
        }
        else
            continue;
            // std::cout << "Data from " << it.first << " IS NOT received" << std::endl;

    }
    if (num_receive != is_data_received_self.size())
        is_all_data_received_self = false;
    else{ 
        is_all_data_received_self = true;
        // std::cout << "ALL DATA IS RECEIVED BY ITSELF" << std::endl;
    }
}

void MSGHandler::ReceiveNeighCheck(){
    int num_receive = 0;
    for (auto &it: is_data_received_neigh){
        if(it.second == true){
            // std::cout << "Data send to vehicle " << it.first << " has been received" << std::endl;
            num_receive++;
        }
        else 
            continue;
            // std::cout << "Data send to " << it.first << " HAS NOT BEEN received" << std::endl;
        
    }
    if(num_receive != is_data_received_neigh.size())
        is_all_data_received_neigh = false;
    else{
        is_all_data_received_neigh = true;
        // std::cout << "All data from neighbors is received. " << std::endl;
    }
}