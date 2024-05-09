#pragma once

#include "esphome/components/button/button.h"
#include "../ti6432.h"

namespace esphome {
namespace ti6432 {

class CustomSetEndButton : public button::Button, public Parented<TI6432Component> {
 public:
  CustomSetEndButton() = default;

 protected:
  void press_action() override;
};

}  // namespace ti6432
}  // namespace esphome
