#include "underlyFuc_switch.h"

namespace esphome {
namespace ti6432 {

void UnderlyOpenFunctionSwitch::write_state(bool state) {
  this->publish_state(state);
  this->parent_->set_underlying_open_function(state);
}

}  // namespace ti6432
}  // namespace esphome
