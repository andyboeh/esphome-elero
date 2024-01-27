#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/cover/cover.h"
#include "esphome/components/elero/elero.h"

namespace esphome {
namespace elero {

class EleroCover : public cover::Cover, public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override;
  
  cover::CoverTraits get_traits() override;
  
  void set_elero_parent(Elero *parent) { this->parent_ = parent; }
  void set_blind_address(uint32_t address) { this->blind_address_ = address; }
  void set_channel(uint8_t channel) { this->channel_ = channel; }
  void set_remote_address(uint32_t remote) { this->remote_address_ = remote; }
  void set_name(std::string name) { this->name_ = name; }
  
 protected:
  void control(const cover::CoverCall &call) override;

  uint32_t blind_address_{0};
  uint32_t remote_address_{0};
  uint8_t channel_{0};
  std::string name_;
  Elero *parent_;
};

} // namespace elero
} // namespace esphome

