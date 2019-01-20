#ifndef SRC_H2C_TASK_REGION_H_
#define SRC_H2C_TASK_REGION_H_

#include <cstdint>
#include <string>

namespace librav {

/// A type used to specify task regions. A region can be labeled from 0 to 31, which corresponds to "p0" to "p31".
class RegionLabel {
public:
	RegionLabel():region_label_(0),region_name_("p0"),bit_map_(0){};
	RegionLabel(int8_t rid):region_label_(rid){
		region_name_ = "p" + std::to_string(rid);
		bit_map_ = 0x01 << rid;
	};
	~RegionLabel(){};

	const static int8_t max_label_num = 32;

private:
	int8_t region_label_;
	std::string region_name_;
	int32_t bit_map_;

public:
	void SetRegionLabelID(uint8_t new_label){
		if(new_label > max_label_num)
			new_label = max_label_num;

		region_label_ = new_label;
		region_name_ = "p" + std::to_string(new_label);
		bit_map_ = 0x01 << new_label;
	};

	int32_t GetRegionLabelID() const {
		return region_label_;
	}

	std::string GetRegionLabelName() const {
		return region_name_;
	}

	int32_t GetRegionLabelBitMap() const {
		return bit_map_;
	}
};

}

#endif /* SRC_H2C_TASK_REGION_H_ */
