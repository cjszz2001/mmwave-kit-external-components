#include "custom_unman_time_number.h"

namespace esphome {
namespace ti6342 {

void CustomUnmanTimeNumber::control(float value) { this->parent_->set_custom_unman_time(value); }

}  // namespace ti6342
}  // namespace esphome
