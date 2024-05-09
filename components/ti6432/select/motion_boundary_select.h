#pragma once

#include "esphome/components/select/select.h"
#include "../ti6432.h"

namespace esphome {
namespace ti6432 {

class MotionBoundarySelect : public select::Select, public Parented<TI6432Component> {
 public:
  MotionBoundarySelect() = default;

 protected:
  void control(const std::string &value) override;
};

}  // namespace ti6432
}  // namespace esphome
