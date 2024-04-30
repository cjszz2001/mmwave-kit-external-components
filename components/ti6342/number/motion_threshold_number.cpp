#include "motion_threshold_number.h"

namespace esphome {
namespace ti6342 {

void MotionThresholdNumber::control(float value) { this->parent_->set_motion_threshold(value); }

}  // namespace ti6342
}  // namespace esphome
