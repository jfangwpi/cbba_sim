/*
 * ltl_task.cpp
 *
 *  Created on: Feb 15, 2017
 *      Author: jfang
 */
// standard library
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>


// self-define library
#include "cbba/cbba_task.hpp"
#include "config_reader/config_reader.hpp"


using namespace librav;

void LTLFormula::InitializeGlobalLTL(){

	ConfigReader config_reader("../../src/config/task.ini");
    if (config_reader.CheckError()){
        std::cout << "Reading config file failed." << std::endl;
    }
	// Read Safety and Liveness
	LTL_expression_Safety = config_reader.GetString("safety", "");
	LTL_expression_Liveness = config_reader.GetString("liveness", "");

    int32_t num_AP = config_reader.GetReal("num_AP", 0);

	for (int i = 0; i < num_AP; i++){
		AP_Info ap;
		std::string AP_idx = "AP" + std::to_string(i);
		std::string AP_pos = AP_idx + "_pos";
		std::string AP_num_agents = AP_idx + "_num_agents";

		ap.P_idx_ = config_reader.GetString(AP_idx, "");
		
		ap.idx_ = stoi(ap.P_idx_.substr(1));
		ap.pos_ = config_reader.GetReal(AP_pos, 0);
		ap.num_agents_ = config_reader.GetReal(AP_num_agents, 1);

		task_info.push_back(ap);
	}
}

std::vector<std::vector<std::string>> LTLDecomposition::ObtainBuchiRegion(std::vector<std::string> expression){

	std::string IntegerNum = "p0123456789";
    std::size_t found;

    std::vector<std::vector<std::string>> BuchiRegions = {};

    for(int i = 0; i < expression.size(); i++){
    	std::vector<std::size_t> position_p = {0};
        std::vector<std::string> buchi_regions;

        while (position_p.back() != std::string::npos)
        	position_p.push_back(expression[i].find("p",position_p.back()+1,1));

        // Remove the first and the last element in the position vector
        position_p.erase(position_p.begin());
        position_p.pop_back();

        buchi_regions.clear();
        for (int k = 0; k < position_p.size();k++){
        	int j = 0;
            std::string sub_sub_LTL_expression;

            // Obtain the sub_sub_LTL_expression
            if (k != position_p.size()-1)
            	sub_sub_LTL_expression = expression[i].substr(position_p[k],position_p[k+1]-position_p[k]-1);
            else
            	sub_sub_LTL_expression = expression[i].substr(position_p[k]);

            // Only work on the sub_sub_LTL_expression
            while (found != std::string::npos){
            	found = IntegerNum.find(sub_sub_LTL_expression[j]);
            	j++;
            }

            buchi_regions.push_back(sub_sub_LTL_expression.substr(0,j-1));
            found = {0};
        }

        BuchiRegions.push_back(buchi_regions);
    }

    return BuchiRegions;
}


void LTLDecomposition::GlobalLTLDecomposition(LTLFormula& formula){

	std::vector<std::string> sub_expression_Liveness = LTLDecomposition::Decomposition(formula.LTL_expression_Liveness);
	std::vector<std::vector<std::string>> sub_buchi_regions_total = LTLDecomposition::ObtainBuchiRegion(sub_expression_Liveness);
	
	// Determine whether the task is independent or dependent
	for (auto &br: sub_buchi_regions_total){
		for(auto &ti: formula.task_info){
			if(br[0] == ti.P_idx_){
				if(ti.num_agents_ > 1){
					formula.sub_buchi_regions.sub_buchi_regions_Liveness_De.push_back(br);
				}
				else{
					formula.sub_buchi_regions.sub_buchi_regions_Liveness_In.push_back(br);
				}
				break;
			}
		}
	}

	// Update the total buchi regions
	formula.sub_buchi_regions.sub_buchi_regions_Liveness_T = formula.sub_buchi_regions.sub_buchi_regions_Liveness_In;
	for (auto it_d = formula.sub_buchi_regions.sub_buchi_regions_Liveness_De.begin(); it_d != formula.sub_buchi_regions.sub_buchi_regions_Liveness_De.end(); it_d++)
		formula.sub_buchi_regions.sub_buchi_regions_Liveness_T.push_back(*it_d);

	// Construct sub_expression for independent and dependent tasks
	// Expression of Independent: Liveness
	for (int idx = 0; idx < formula.sub_buchi_regions.sub_buchi_regions_Liveness_In.size(); idx++)
		for (auto it1 = sub_expression_Liveness.begin();it1 != sub_expression_Liveness.end(); it1++)
			if((*it1).find(formula.sub_buchi_regions.sub_buchi_regions_Liveness_In[idx][0]) != std::string::npos)
				formula.LTL_expression.sub_LTL_expression_Liveness_In.push_back(*it1);
			else
				continue;

	// Expression of Dependent: Liveness
	for (int idx = 0; idx < formula.sub_buchi_regions.sub_buchi_regions_Liveness_De.size(); idx++)
		for (auto it1 = sub_expression_Liveness.begin();it1 != sub_expression_Liveness.end(); it1++)
			if((*it1).find(formula.sub_buchi_regions.sub_buchi_regions_Liveness_De[idx][0]) != std::string::npos)
				formula.LTL_expression.sub_LTL_expression_Liveness_De.push_back(*it1);
			else
				continue;

	if (!formula.LTL_expression_Safety.empty())
		formula.LTL_expression_Safety.append(" && ");
		
	// Expression of Independent: Safety + Liveness
	for(int i = 0; i < formula.LTL_expression.sub_LTL_expression_Liveness_In.size(); i++){
		formula.LTL_expression.sub_LTL_expression_In.push_back(formula.LTL_expression_Safety+formula.LTL_expression.sub_LTL_expression_Liveness_In[i]);
		formula.LTL_expression.sub_LTL_expression_Liveness_T.push_back(formula.LTL_expression.sub_LTL_expression_Liveness_In[i]);
		formula.LTL_expression.sub_LTL_expression_T.push_back(formula.LTL_expression_Safety+formula.LTL_expression.sub_LTL_expression_Liveness_In[i]);
	}

	// Expression of Dependent: Safety + Liveness
	for(int i = 0; i < formula.LTL_expression.sub_LTL_expression_Liveness_De.size(); i++){
		formula.LTL_expression.sub_LTL_expression_De.push_back(formula.LTL_expression_Safety+formula.LTL_expression.sub_LTL_expression_Liveness_De[i]);
		formula.LTL_expression.sub_LTL_expression_Liveness_T.push_back(formula.LTL_expression.sub_LTL_expression_Liveness_De[i]);
		formula.LTL_expression.sub_LTL_expression_T.push_back(formula.LTL_expression_Safety+formula.LTL_expression.sub_LTL_expression_Liveness_De[i]);
	}

	// Update the number of Independent and Dependent tasks
	formula.Num_Tasks_In = formula.LTL_expression.sub_LTL_expression_In.size();
	formula.Num_Tasks_De = formula.LTL_expression.sub_LTL_expression_De.size();

	// std::cout << "=========================================== DEBUG FOR INDEPENDENT AND DEPENDENT TASKS =========================================" << std::endl;
	// std::cout << "Expression of Independent " << std::endl;
	// for (auto it = formula.LTL_expression.sub_LTL_expression_In.begin(); it != formula.LTL_expression.sub_LTL_expression_In.end(); it++)
	// 	std::cout << (*it) << std::endl;

	// std::cout << "Expression of Dependent " << std::endl;
	// for (auto it = formula.LTL_expression.sub_LTL_expression_De.begin(); it != formula.LTL_expression.sub_LTL_expression_De.end(); it++)
	// 	std::cout << *it << std::endl;

	// std::cout << "Dependent Tasks are:" << std::endl;
	// for (auto it = formula.sub_buchi_regions.sub_buchi_regions_Liveness_De.begin(); it != formula.sub_buchi_regions.sub_buchi_regions_Liveness_De.end(); it++){
	// 	for (auto it1 = (*it).begin(); it1 != (*it).end(); it1++)
	// 		std::cout << *it1 << " ";
	//     std::cout << std::endl;
	// }

	// std::cout << "Independent Tasks are:" << std::endl;
	// for (auto it = formula.sub_buchi_regions.sub_buchi_regions_Liveness_In.begin(); it != formula.sub_buchi_regions.sub_buchi_regions_Liveness_In.end(); it++){
	// 	for (auto it1 = (*it).begin(); it1 != (*it).end(); it1++)
	// 		std::cout << *it1 << " ";
	//     std::cout << std::endl;
	// }

	// std::cout << "============== Task information of LTL ==============" << std::endl;
	// // std::cout << "The safety specification is: " << formula.LTL_expression_Safety << std::endl; 
	// std::cout << "The decomposed liveness tasks are: " << std::endl;
	// int count = 0;
	// for (auto it = formula.LTL_expression.sub_LTL_expression_Liveness_T.begin(); it != formula.LTL_expression.sub_LTL_expression_Liveness_T.end(); it++){
	// 	std::cout << "Task " << count << ": ";
	// 	std::cout << *it << std::endl;
	// }

}

std::vector<std::string> LTLDecomposition::Decomposition(std::string expression){

	std::vector<std::string> sub_expression;
	sub_expression.clear();

	std::vector<std::size_t> position = {0};

	while(position.back() != std::string::npos)
	{
		position.push_back(expression.find(") && (",position.back()+1,6));

	}

	// Remove the first and the last element in the position vector
	//position.erase(position.begin());
	position.pop_back();

	// Decompose the Liveness Property
	for (std::vector<std::size_t>::iterator it = position.begin();it != position.end();it++)
	{
		if (it == position.begin())
		{
			//copy_sub_LTL = LTL_expression_Liveness.substr(*it,*(it+1)-(*it) +1);
			sub_expression.push_back(expression.substr(*it,*(it+1)-(*it) +1));
			continue;
		}
		else if(it == position.end())
		{
			//copy_sub_LTL = LTL_expression_Liveness.substr(*it);
			sub_expression.push_back(expression.substr(*it));
			continue;
		}
		else
		{
			//copy_sub_LTL = LTL_expression_Liveness.substr((*it)+5,*(it+1)-((*it)+5)+1);
			sub_expression.push_back(expression.substr((*it)+5,*(it+1)-((*it)+5)+1));
			continue;

		}
	}

	return sub_expression;
}

std::string LTLDecomposition::subtask_recreator(std::vector<int> bundle, bool succFlag, LTLFormula& formula){
	std::string SubtaskFromBundle;
	std::string sub_task_bundled;
	sub_task_bundled.clear();
	SubtaskFromBundle.clear();

	if (succFlag == 1){
		if(bundle.size() == 1){
			std::cout << "There is no need to recreate the subtask" << std::endl;
		    SubtaskFromBundle = formula.LTL_expression.sub_LTL_expression_Liveness_T[bundle[0]];
		}
		else if(bundle.size() > 1){
			for (int j  = bundle.size()-1; j >= 0;j--){
				if (j == bundle.size()-1 )
					SubtaskFromBundle = formula.LTL_expression.sub_LTL_expression_Liveness_T[bundle[j]];
				else{
//        					std::string mid_sub_LTL_expression_Liveness = sub_LTL_expression_Liveness[bundle[j]];
//        					mid_sub_LTL_expression_Liveness.erase(mid_sub_LTL_expression_Liveness.begin());
//        					mid_sub_LTL_expression_Liveness.pop_back();
					if (formula.sub_buchi_regions.sub_buchi_regions_Liveness_T[bundle[j]].size() == 1)
						SubtaskFromBundle = "<> ( " + formula.sub_buchi_regions.sub_buchi_regions_Liveness_T[bundle[j]].front() + " && " + SubtaskFromBundle + ")";
					else {
						for (int it_sub_task_idx = formula.sub_buchi_regions.sub_buchi_regions_Liveness_T[bundle[j]].size()-1;it_sub_task_idx >= 0 ; it_sub_task_idx--)
							SubtaskFromBundle = "<> ( " + formula.sub_buchi_regions.sub_buchi_regions_Liveness_T[bundle[j]][it_sub_task_idx] + " && " + SubtaskFromBundle + ")";
					}
					//SubtaskFromBundle = "( " + sub_LTL_expression_Liveness[bundle[j]] + " && " + SubtaskFromBundle + ")";
				}
			}
		}
		else
			std::cout << "No task in the bundle" <<std::endl;

	}
	else{
		if(bundle.size() == 1){
			std::cout << "There is no need to recreate the subtask" << std::endl;
		    SubtaskFromBundle = formula.LTL_expression.sub_LTL_expression_Liveness_T[bundle[0]];
		}
		else if(bundle.size() > 1){
			SubtaskFromBundle = formula.LTL_expression.sub_LTL_expression_Liveness_T[bundle.back()];
		}
		else
			std::cout << "No task in the bundle" <<std::endl;
	}

	sub_task_bundled = formula.LTL_expression_Safety + SubtaskFromBundle;
	return sub_task_bundled;
}


std::string LTLDecomposition::subtask_recreator(std::vector<int> bundle, LTLFormula& formula){
	std::string SubtaskFromBundle;
	std::string sub_task_bundled;
	sub_task_bundled.clear();
	SubtaskFromBundle.clear();


	if(bundle.size() == 1){
			std::cout << "There is no need to recreate the subtask" << std::endl;
			SubtaskFromBundle = formula.LTL_expression.sub_LTL_expression_Liveness_T[bundle[0]];
			// std::cout << "~========================DEBUG after=============================" << std::endl;
	}

	else if(bundle.size() > 1){
		for (int j  = bundle.size()-1; j >= 0;j--){
			if (j == bundle.size()-1 )
				SubtaskFromBundle = formula.LTL_expression.sub_LTL_expression_Liveness_T[bundle[j]];
			else{
				if (formula.sub_buchi_regions.sub_buchi_regions_Liveness_T[bundle[j]].size() == 1)
					SubtaskFromBundle = "<> ( " + formula.sub_buchi_regions.sub_buchi_regions_Liveness_T[bundle[j]].front() + " && " + SubtaskFromBundle + ")";
				else {
					if(formula.task_type_[bundle[j]] == TaskType::VISIT){
						for (int it_sub_task_idx = formula.sub_buchi_regions.sub_buchi_regions_Liveness_T[bundle[j]].size()-1;it_sub_task_idx >= 0 ; it_sub_task_idx--)
							SubtaskFromBundle = "<> ( " + formula.sub_buchi_regions.sub_buchi_regions_Liveness_T[bundle[j]][it_sub_task_idx] + " && " + SubtaskFromBundle + ")";
					}
					else if(formula.task_type_[bundle[j]] == TaskType::SEARCH){
						std::size_t pos_p = formula.LTL_expression.sub_LTL_expression_Liveness_T[bundle[j]].find('p');
						int str_size = formula.LTL_expression.sub_LTL_expression_Liveness_T[bundle[j]].size();
						std::string key_part = formula.LTL_expression.sub_LTL_expression_Liveness_T[bundle[j]].substr(pos_p-1);
						key_part.pop_back();
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
		sub_task_bundled = formula.LTL_expression_Safety.substr(0,9);
		
	}

	if (!formula.LTL_expression_Safety.empty())
		sub_task_bundled = formula.LTL_expression_Safety + SubtaskFromBundle;
	else
		sub_task_bundled = SubtaskFromBundle;

	return sub_task_bundled;
}


void LTLDecomposition::get_safety_properties(LTLFormula& formula, std::string safty) {
	formula.LTL_expression_Safety = safty;
}

void LTLDecomposition::get_liveness_properties(LTLFormula& formula,std::string liveness) {
	formula.LTL_expression_Liveness = liveness;
}

void LTLDecomposition::get_tasks_properties(LTLFormula& formula, std::map<int, TaskType> task_info){
	for (auto &it: task_info){
		formula.task_type_[it.first] = it.second;
	}
}

// Initialize cbga task
cbba_Task::cbba_Task(int64_t id, std::vector<int64_t> pos, TaskType type, int64_t num_agents, std::string liveness, std::string safety, std::vector<std::string> buchi_regions):
idx_(id),
pos_(pos),
type_(type),
num_agents_(num_agents),
specification_liveness_(liveness),
specification_safety_(safety),
buchi_regions_(buchi_regions){
	min_bid_ = -1;
};


void CBBATasks::SetTasksFromLTL(LTLFormula Global_LTL){
    tasks_ = {};
    for (int i = 0; i < Global_LTL.sub_buchi_regions.sub_buchi_regions_Liveness_T.size(); i++){
        int64_t id = i;
        std::vector<int64_t> pos = {};
        int num_agents = 1;
        for (int j = 0; j < Global_LTL.sub_buchi_regions.sub_buchi_regions_Liveness_T[i].size(); j++){
            for(int m = 0; m < Global_LTL.task_info.size(); m++){
                if(Global_LTL.sub_buchi_regions.sub_buchi_regions_Liveness_T[i][j] == Global_LTL.task_info[m].P_idx_){
                    pos.push_back(Global_LTL.task_info[m].pos_);
                    num_agents = Global_LTL.task_info[m].num_agents_;
                    break;
                }
            }
        }
        TaskType type = TaskType::VISIT;
        std::string liveness = Global_LTL.LTL_expression.sub_LTL_expression_T[id];
        std::string safety = Global_LTL.LTL_expression_Safety;
        std::vector<std::string> buchi_regions = Global_LTL.sub_buchi_regions.sub_buchi_regions_Liveness_T[id];

        cbba_Task task = cbba_Task(id, pos, type, num_agents, liveness, safety, buchi_regions);
        tasks_.push_back(task);
    }
}


cbba_Task CBBATasks::FindTaskFromID(int64_t idx){
    for (auto &task: tasks_){
        if(task.idx_ == idx)
            return task;
    }
}


cbba_Task& CBBATasks::FindTask(int64_t idx){
    for (auto &task: tasks_){
        if(task.idx_ == idx)
            return task;
    }
}


void CBBATasks::GetAllTasks(){
	std::cout << "================ Task information of LTL ================" << std::endl;
    for (auto &task: tasks_){
        std::cout << "++++++++++++++++++++++++++++++++++" <<std::endl;
        std::cout << "Task " << task.idx_ << std::endl;
        std::cout << "Position of task: ";
        for (auto &p: task.pos_)
            std::cout <<  p << " ";
        std::cout << std::endl;
        std::cout << "Number of vehicles required to satisfy the task: " << task.num_agents_ << std::endl;
        // std::cout << "safety specification is " << task.specification_safety_ << std::endl;
        std::cout << "Corresponding specification of the task: " << std::endl;
		std::cout << task.specification_liveness_ << std::endl;
        // std::cout << "buchi regions are "  << std::endl;
        // for (auto &e: task.buchi_regions_)
        //     std::cout << e << " ";
        // std::cout << std::endl;
    }
				  
	std::cout << "=========================================================" << std::endl;
	std::cout << std::endl;
}
