#include "custom_unman_time_number.h"

namespace esphome {
namespace ti6432 {

void CustomUnmanTimeNumber::control(float value) { this->parent_->set_custom_unman_time(value); }

}  // namespace ti6432
}  // namespace esphome
