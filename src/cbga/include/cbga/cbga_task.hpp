/*
 * cbga_task.hpp
 *
 *  Created on: Aug 30, 2018
 *      Author: jfang
 */

#ifndef CBGA_TASK_HPP
#define CBGA_TASK_HPP

// standard library
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <map>

#include "cbba/cbba_task.hpp"

namespace librav{

const float cbga_benefit = 1000.0;

    class cbga_Task{

        public:
            cbga_Task(int64_t id, std::vector<int64_t> pos, TaskType type, int64_t num_agents, std::string liveness, std::string safety, std::vector<std::string> buchi_regions);

            ~cbga_Task(){};

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

    class CBGATasks{

        public: 
            CBGATasks(LTLFormula Global_LTL){SetTasksFromLTL(Global_LTL);};
            ~CBGATasks(){};


        public: 
            std::vector<cbga_Task> tasks_;
            std::string specification_safety_;
            

        public:
            void SetTasksFromLTL(LTLFormula Global_LTL);
            std::string local_formula_recreator(std::vector<int> bundle);
            cbga_Task FindTaskFromID(int64_t idx);
            cbga_Task& FindTask(int64_t idx);
            void GetAllTasks();


    };
}

#endif /* CBGA_TASK_HPP */