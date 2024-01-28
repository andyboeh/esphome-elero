#include "EleroCover.h"
#include "esphome/core/log.h"
#include "esphome/components/elero/elero.h"

namespace esphome {
namespace elero {

static const char *const TAG = "elero.cover";

void EleroCover::dump_config() {
  LOG_COVER("", "Elero Cover", this);
}

void EleroCover::setup() {
  this->parent_->register_cover(this);

}

void EleroCover::loop() {


}

float EleroCover::get_setup_priority() const { return setup_priority::DATA; }

cover::CoverTraits EleroCover::get_traits() {
  auto traits = cover::CoverTraits();
  traits.set_supports_stop(true);
  traits.set_supports_position(true);
  traits.set_supports_toggle(true);
  traits.set_is_assumed_state(false);
  return traits;
}

void EleroCover::set_rx_state(uint8_t state) {
  ESP_LOGD(TAG, "Got state: 0x%02x for blind 0x%02x", state, this->blind_address_);
}

void EleroCover::control(const cover::CoverCall &call) {

}

} // namespace elero
} // namespace esphome
