#pragma once

#include "esphome/components/switch/switch.h"
#include "../ti6432.h"

namespace esphome {
namespace ti6432 {

class UnderlyOpenFunctionSwitch : public switch_::Switch, public Parented<TI6432Component> {
 public:
  UnderlyOpenFunctionSwitch() = default;

 protected:
  void write_state(bool state) override;
};

}  // namespace ti6432
}  // namespace esphome
