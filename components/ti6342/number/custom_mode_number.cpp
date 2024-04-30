#include "custom_mode_number.h"

namespace esphome {
namespace ti6342 {

void CustomModeNumber::control(float value) {
  this->publish_state(value);
  this->parent_->set_custom_mode(value);
}

}  // namespace ti6342
}  // namespace esphome
