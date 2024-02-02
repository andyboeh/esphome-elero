#include "EleroCover.h"
#include "esphome/core/log.h"
#include "esphome/components/elero/elero.h"

namespace esphome {
namespace elero {

using namespace esphome::cover;

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
  traits.set_supports_position(false);
  traits.set_supports_toggle(false);
  traits.set_is_assumed_state(true);
  return traits;
}

void EleroCover::set_rx_state(uint8_t state) {
  ESP_LOGD(TAG, "Got state: 0x%02x for blind 0x%02x", state, this->blind_address_);
}

void EleroCover::increase_counter() {
  if(this->counter_ == 0xff)
    this->counter_ = 1;
  else
    this->counter_ += 1;
}

void EleroCover::send_command(uint8_t command) {
  this->parent_->send_command(command, this->counter_, this->blind_address_, this->remote_address_, this->channel_);
  this->increase_counter();
}

void EleroCover::control(const cover::CoverCall &call) {
  if (call.get_stop()) {
    this->send_command(ELERO_COMMAND_COVER_STOP);
  }
  if (call.get_position().has_value()) {
    auto pos = *call.get_position();
    if(pos == COVER_OPEN) {
      ESP_LOGD(TAG, "Sending OPEN command");
      this->send_command(elero::ELERO_COMMAND_COVER_UP);
    } else if(pos == COVER_CLOSED) {
      ESP_LOGD(TAG, "Sending CLOSE command");
      this->send_command(elero::ELERO_COMMAND_COVER_DOWN);
    }
  }
}

} // namespace elero
} // namespace esphome
