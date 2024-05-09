#include "motion_boundary_select.h"

namespace esphome {
namespace ti6432 {

void MotionBoundarySelect::control(const std::string &value) {
  this->publish_state(value);
  auto index = this->index_of(value);
  if (index.has_value()) {
    this->parent_->set_motion_boundary(index.value());
  }
}

}  // namespace ti6432
}  // namespace esphome
