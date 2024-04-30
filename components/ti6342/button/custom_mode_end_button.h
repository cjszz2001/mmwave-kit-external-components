#pragma once

#include "esphome/components/button/button.h"
#include "../ti6342.h"

namespace esphome {
namespace ti6342 {

class CustomSetEndButton : public button::Button, public Parented<TI6342Component> {
 public:
  CustomSetEndButton() = default;

 protected:
  void press_action() override;
};

}  // namespace ti6342
}  // namespace esphome
