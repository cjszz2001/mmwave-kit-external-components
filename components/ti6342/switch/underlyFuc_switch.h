#pragma once

#include "esphome/components/switch/switch.h"
#include "../ti6342.h"

namespace esphome {
namespace ti6342 {

class UnderlyOpenFunctionSwitch : public switch_::Switch, public Parented<TI6342Component> {
 public:
  UnderlyOpenFunctionSwitch() = default;

 protected:
  void write_state(bool state) override;
};

}  // namespace ti6342
}  // namespace esphome
