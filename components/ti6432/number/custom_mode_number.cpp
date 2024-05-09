#include "custom_mode_number.h"

namespace esphome {
namespace ti6432 {

void CustomModeNumber::control(float value) {
  this->publish_state(value);
  this->parent_->set_custom_mode(value);
}

}  // namespace ti6432
}  // namespace esphome
