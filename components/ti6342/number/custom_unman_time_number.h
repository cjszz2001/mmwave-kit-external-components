#pragma once

#include "esphome/components/number/number.h"
#include "../ti6342.h"

namespace esphome {
namespace ti6342 {

class CustomUnmanTimeNumber : public number::Number, public Parented<TI6342Component> {
 public:
  CustomUnmanTimeNumber() = default;

 protected:
  void control(float value) override;
};

}  // namespace ti6342
}  // namespace esphome
