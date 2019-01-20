#ifndef CELL_LABEL_HPP
#define CELL_LABEL_HPP

#include <cstdint>
#include <string>
#include <vector>

#include "ltl/region_label.hpp"

namespace librav{

class CellLabel{
    public:
        CellLabel(){};
        ~CellLabel(){};

    private:
        std::vector<RegionLabel> labels_;
        int32_t default_label_;
    
    public:
        void SetDefaultRegionLabel(int32_t lab){
            default_label_ = lab;
            if(labels_.empty()){
                RegionLabel new_label(lab);
                labels_.push_back(new_label);
            }
        }

        int32_t GetDefaultRegionLabel() const {return default_label_;};

        void AssignRegionLabel(int32_t label){
            bool label_exist = false;
            for (auto &lab: labels_)
            {
                if(lab.GetRegionLabelID() == label){
                    label_exist = true;
                    break;
                }
            }
            if(label_exist == false){
                RegionLabel new_label(label);
                labels_.push_back(new_label);
            }

        }

        void RemoveRegionLabel(int32_t label){
            for(auto it = labels_.begin(); it != labels_.end(); it++){
                if((*it).GetRegionLabelID() == label){
                    labels_.erase(it);
                    break;
                }
            }
        }

        std::string GetCellLabels() const
        {
            std::string cell_labs;
            for(auto &lab: labels_)
                cell_labs += lab.GetRegionLabelName();
            
            return cell_labs;
        }

        int32_t GetCellBitMap() const
        {
            int32_t bit_map = 0;
            for(auto &lab: labels_)
                bit_map = bit_map | lab.GetRegionLabelBitMap();

            return bit_map;
        }


};
}


#endif /* CELL_LABEL_HPP */