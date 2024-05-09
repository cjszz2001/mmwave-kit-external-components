#pragma once

#include "esphome/components/number/number.h"
#include "../ti6432.h"

namespace esphome {
namespace ti6432 {

class MotionThresholdNumber : public number::Number, public Parented<TI6432Component> {
 public:
  MotionThresholdNumber() = default;

 protected:
  void control(float value) override;
};

}  // namespace ti6432
}  // namespace esphome