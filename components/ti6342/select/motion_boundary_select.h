#pragma once

#include "esphome/components/select/select.h"
#include "../ti6342.h"

namespace esphome {
namespace ti6342 {

class MotionBoundarySelect : public select::Select, public Parented<TI6342Component> {
 public:
  MotionBoundarySelect() = default;

 protected:
  void control(const std::string &value) override;
};

}  // namespace ti6342
}  // namespace esphome
