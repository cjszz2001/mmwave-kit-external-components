#pragma once

#include "esphome/components/button/button.h"
#include "../ti6342.h"

namespace esphome {
namespace ti6342 {

class RestartButton : public button::Button, public Parented<TI6342Component> {
 public:
  RestartButton() = default;

 protected:
  void press_action() override;
};

}  // namespace ti6342
}  // namespace esphome
