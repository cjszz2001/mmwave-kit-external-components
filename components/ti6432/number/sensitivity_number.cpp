#include "sensitivity_number.h"

namespace esphome {
namespace ti6432 {

void SensitivityNumber::control(float value) { this->parent_->set_sensitivity(value); }

}  // namespace ti6432
}  // namespace esphome
