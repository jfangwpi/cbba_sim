/*
 * cbga_task.cpp
 *
 *  Created on: Aug 30, 2018
 *      Author: jfang
 */

// standard library
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <map>

#include "cbga/cbga_task.hpp"

using namespace librav;

// Initialize cbga task
cbga_Task::cbga_Task(int64_t id, std::vector<int64_t> pos, TaskType type, int64_t num_agents, std::string liveness, std::string safety, std::vector<std::string> buchi_regions):
idx_(id),
pos_(pos),
type_(type),
num_agents_(num_agents),
specification_liveness_(liveness),
specification_safety_(safety),
buchi_regions_(buchi_regions){};


void CBGATasks::SetTasksFromLTL(LTLFormula Global_LTL){
    tasks_ = {};
    for (int i = 0; i < Global_LTL.sub_buchi_regions.sub_buchi_regions_Liveness_T.size(); i++){
        int64_t id = i;
        std::vector<int64_t> pos = {};
        int num_agents = 1;
        TaskType type;
        for (int j = 0; j < Global_LTL.sub_buchi_regions.sub_buchi_regions_Liveness_T[i].size(); j++){
            for(int m = 0; m < Global_LTL.task_info.size(); m++){
                if(Global_LTL.sub_buchi_regions.sub_buchi_regions_Liveness_T[i][j] == Global_LTL.task_info[m].P_idx_){
                    pos.push_back(Global_LTL.task_info[m].pos_);
                    if(num_agents <= Global_LTL.task_info[m].num_agents_){
                        num_agents = Global_LTL.task_info[m].num_agents_;
                    }
                    type = Global_LTL.task_info[m].type_;
                    break;
                }
            }
        }
        // TaskType type = TaskType::VISIT;
        std::string liveness = Global_LTL.LTL_expression.sub_LTL_expression_Liveness_T[id];
        std::string safety = Global_LTL.LTL_expression_Safety;
        std::vector<std::string> buchi_regions = Global_LTL.sub_buchi_regions.sub_buchi_regions_Liveness_T[id];

        cbga_Task task = cbga_Task(id, pos, type, num_agents, liveness, safety, buchi_regions);
        tasks_.push_back(task);
    }
    specification_safety_ = Global_LTL.LTL_expression_Safety;
}


cbga_Task CBGATasks::FindTaskFromID(int64_t idx){
    for (auto &task: tasks_){
        if(task.idx_ == idx)
            return task;
    }
}


void CBGATasks::GetAllTasks(){
    for (auto &task: tasks_){
        std::cout << "============================== " << std::endl;
        std::cout << "task " << task.idx_ << std::endl;
        std::cout << "pos: ";
        for (auto &p: task.pos_)
            std::cout <<  p << " ";
        std::cout << std::endl;
        std::cout << "num of agents required " << task.num_agents_ << std::endl;
        std::cout << "safety specification is " << task.specification_safety_ << std::endl;
        std::cout << "liveness specification is " << task.specification_liveness_ << std::endl;
        std::cout << "buchi regions are "  << std::endl;
        for (auto &e: task.buchi_regions_)
            std::cout << e << " ";
        std::cout << std::endl;
    }
}


std::string CBGATasks::local_formula_recreator(std::vector<int> bundle){
	std::string SubtaskFromBundle;
	std::string sub_task_bundled;

	if(bundle.size() == 1){
		cbga_Task task = CBGATasks::FindTaskFromID(bundle[0]);
		std::cout << "There is no need to recreate the subtask" << std::endl;
		SubtaskFromBundle = task.specification_liveness_;
		// std::cout << "~========================DEBUG after=============================" << std::endl;
	}
	else if(bundle.size() > 1){
		for (int j  = bundle.size()-1; j >= 0;j--){
			cbga_Task task = CBGATasks::FindTaskFromID(bundle[j]);
			if (j == bundle.size()-1){
				SubtaskFromBundle = task.specification_liveness_;
				// SubtaskFromBundle = "<> ( " + task.buchi_regions_.front
			}
			else{
				if (task.buchi_regions_.size() == 1)
					SubtaskFromBundle = "<> ( " + task.buchi_regions_.front() + " && " + SubtaskFromBundle + ")";
				else {
					if(task.type_ == TaskType::VISIT){
						for (int it_sub_task_idx = task.buchi_regions_.size()-1;it_sub_task_idx >= 0 ; it_sub_task_idx--)
							SubtaskFromBundle = "<> ( " + task.buchi_regions_[it_sub_task_idx] + " && " + SubtaskFromBundle + ")";
					}
					else if(task.type_ == TaskType::SEARCH){
						int length = task.specification_liveness_.size();
						std::string key_part = task.specification_liveness_.substr(5,length-7);
						SubtaskFromBundle = "<> ( " + key_part + " && " + SubtaskFromBundle + ")";
					}
					else
						std::cout << "NO TASK TYPE DEFINED" << std::endl;
				}
					//SubtaskFromBundle = "( " + sub_LTL_expression_Liveness[bundle[j]] + " && " + SubtaskFromBundle + ")";
			}
		}
	}
	else{
		std::cout << "No task in the bundle" <<std::endl;
		int length = specification_safety_.size();
		sub_task_bundled = specification_safety_.substr(0,length-3);
		
	}

	if (!specification_safety_.empty())
		sub_task_bundled = specification_safety_ + SubtaskFromBundle;
	else
		sub_task_bundled = SubtaskFromBundle;

	return sub_task_bundled;
}

cbga_Task& CBGATasks::FindTask(int64_t idx){
    for (auto &task: tasks_){
        if(task.idx_ == idx)
            return task;
    }
}