/*
 * cbba_task.h
 *
 *  Created on: Feb 15, 2017
 *      Author: jfang
 */

#ifndef CBBA_TASK_HPP
#define CBBA_TASK_HPP

// standard library
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <map>

namespace librav{

enum class TaskType
{
    VISIT,
    SEARCH
};

typedef struct
{
	std::string P_idx_;
	int idx_;
	int pos_;
	int num_agents_;
    TaskType type_;
}AP_Info;


typedef struct
{

    std::vector<std::vector<std::string>> sub_buchi_regions_Safety;
    std::vector<std::vector<std::string>> sub_buchi_regions_Liveness_In;
    std::vector<std::vector<std::string>> sub_buchi_regions_Liveness_De;
    std::vector<std::vector<std::string>> sub_buchi_regions_Liveness_T;
}Buchi_Regions;

typedef struct
{

	std::vector<std::string> sub_LTL_expression_Liveness_In;
	std::vector<std::string> sub_LTL_expression_In;

	std::vector<std::string> sub_LTL_expression_Liveness_De;
	std::vector<std::string> sub_LTL_expression_De;

	std::vector<std::string> sub_LTL_expression_Liveness_T;
	std::vector<std::string> sub_LTL_expression_T;

}Expression;


class LTLFormula{

	public:
		LTLFormula():
			task_info(),
			task_type_(){InitializeGlobalLTL();};
		~LTLFormula(){};


	public:
		std::string LTL_expression_Safety;
		std::string LTL_expression_Liveness;

		Expression LTL_expression;
		Buchi_Regions sub_buchi_regions;

		int Num_Tasks_In;
		int Num_Tasks_De;
		std::vector<AP_Info> task_info;

		std::map<int, TaskType> task_type_;


	public:
		void InitializeGlobalLTL();

};

namespace LTLDecomposition
{

void GlobalLTLDecomposition(LTLFormula& formula);
//LTLFormula Divide_Dependent_Independent(LTLFormula formula);
std::vector<std::string> Decomposition(std::string expression);
std::vector<std::vector<std::string>> ObtainBuchiRegion(std::vector<std::string> expressions);
std::string subtask_recreator(std::vector<int> bundle, bool succFlag, LTLFormula& formula);
std::string subtask_recreator(std::vector<int> bundle, LTLFormula& formula);

void get_safety_properties(LTLFormula& formula, std::string safty);
void get_liveness_properties(LTLFormula& formula, std::string liveness);
void get_tasks_properties(LTLFormula& formula, std::map<int, TaskType> task_info);
void set_tasks_details(LTLFormula& formula);

}


class cbba_Task{

        public:
            cbba_Task(int64_t id, std::vector<int64_t> pos, TaskType type, int64_t num_agents, std::string liveness, std::string safety, std::vector<std::string> buchi_regions);

            ~cbba_Task(){};

        public:
            int64_t idx_;
            std::vector<int64_t> pos_;
            TaskType type_;
            int64_t num_agents_;
            double max_reward_;
            
            std::string specification_liveness_;
            std::string specification_safety_;
            std::vector<std::string> buchi_regions_;
			
    };

    class TasksList{

        public: 
            TasksList(LTLFormula Global_LTL){SetTasksFromLTL(Global_LTL);};
            ~TasksList(){};


        public: 
            std::vector<cbba_Task> tasks_;
            std::string specification_safety_;

        public:
            void SetTasksFromLTL(LTLFormula Global_LTL);
            cbba_Task FindTaskFromID(int64_t idx);
			cbba_Task& FindTask(int64_t idx);
            void GetAllTasks();
            std::string local_task_recreator(std::vector<int> bundle);
            std::string local_formula_recreator(std::vector<int> bundle);
            std::vector<std::vector<std::string>> obtain_buchi_regions(std::vector<std::string> expressions);
    };
}



#endif /* CBBA_TASK_HPP */