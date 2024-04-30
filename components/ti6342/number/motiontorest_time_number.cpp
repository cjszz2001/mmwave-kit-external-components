#include "motiontorest_time_number.h"

namespace esphome {
namespace ti6342 {

void MotionToRestTimeNumber::control(float value) { this->parent_->set_motion_to_rest_time(value); }

}  // namespace ti6342
}  // namespace esphome
