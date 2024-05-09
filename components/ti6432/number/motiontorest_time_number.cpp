#include "motiontorest_time_number.h"

namespace esphome {
namespace ti6432 {

void MotionToRestTimeNumber::control(float value) { this->parent_->set_motion_to_rest_time(value); }

}  // namespace ti6432
}  // namespace esphome
