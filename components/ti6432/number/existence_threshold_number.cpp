#include "existence_threshold_number.h"

namespace esphome {
namespace ti6432 {

void ExistenceThresholdNumber::control(float value) { this->parent_->set_existence_threshold(value); }

}  // namespace ti6432
}  // namespace esphome
