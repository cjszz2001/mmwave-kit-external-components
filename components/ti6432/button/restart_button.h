#pragma once

#include "esphome/components/button/button.h"
#include "../ti6432.h"

namespace esphome {
namespace ti6432 {

class RestartButton : public button::Button, public Parented<TI6432Component> {
 public:
  RestartButton() = default;

 protected:
  void press_action() override;
};

}  // namespace ti6432
}  // namespace esphome
