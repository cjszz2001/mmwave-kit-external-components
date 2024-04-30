#include "motion_trigger_time_number.h"

namespace esphome {
namespace ti6342 {

void MotionTriggerTimeNumber::control(float value) { this->parent_->set_motion_trigger_time(value); }

}  // namespace ti6342
}  // namespace esphome
