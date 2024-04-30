#include "existence_threshold_number.h"

namespace esphome {
namespace ti6342 {

void ExistenceThresholdNumber::control(float value) { this->parent_->set_existence_threshold(value); }

}  // namespace ti6342
}  // namespace esphome
