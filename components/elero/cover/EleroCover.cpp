#include "EleroCover.h"
#include "esphome/core/log.h"

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
  uint32_t intvl = ELERO_POLL_INTERVAL;
  if(this->current_operation != COVER_OPERATION_IDLE) {
    if((millis() - ELERO_TIMEOUT_MOVEMENT) < this->movement_start_) // do not poll frequently for an extended period of time
      intvl = ELERO_POLL_INTERVAL_MOVING;
  }

  if((millis() > this->poll_offset_) && (millis() - this->poll_offset_ - this->last_poll_) > intvl) {
    this->send_command(this->command_check_);
    this->last_poll_ = millis() - this->poll_offset_;
  }
}

float EleroCover::get_setup_priority() const { return setup_priority::DATA; }

cover::CoverTraits EleroCover::get_traits() {
  auto traits = cover::CoverTraits();
  traits.set_supports_stop(true);
  traits.set_supports_position(false);
  traits.set_supports_toggle(false);
  traits.set_is_assumed_state(false);
  return traits;
}

void EleroCover::set_rx_state(uint8_t state) {
  ESP_LOGD(TAG, "Got state: 0x%02x for blind 0x%02x", state, this->command_.blind_addr);
  switch(state) {
  case ELERO_STATE_TOP:
    this->position = COVER_OPEN;
    this->current_operation = COVER_OPERATION_IDLE;
    break;
  case ELERO_STATE_BOTTOM:
    this->position = COVER_CLOSED;
    this->current_operation = COVER_OPERATION_IDLE;
    break;
  case ELERO_STATE_START_MOVING_UP:
  case ELERO_STATE_MOVING_UP:
    this->current_operation = COVER_OPERATION_OPENING;
    break;
  case ELERO_STATE_START_MOVING_DOWN:
  case ELERO_STATE_MOVING_DOWN:
    this->current_operation = COVER_OPERATION_CLOSING;
    break;
  case ELERO_STATE_STOPPED:
    this->current_operation = COVER_OPERATION_IDLE;
    break;
  default:
    this->current_operation = COVER_OPERATION_IDLE;
  }
  this->publish_state();
}

void EleroCover::increase_counter() {
  if(this->command_.counter == 0xff)
    this->command_.counter = 1;
  else
    this->command_.counter += 1;
}

void EleroCover::send_command(uint8_t command) {
  this->command_.payload[4] = command;

  for(uint8_t i=0; i<ELERO_SEND_PACKETS; i++) {
    for(uint8_t i=0; i<ELERO_SEND_RETRIES; i++) {
      if(this->parent_->send_command(&this->command_))
        break;
    }
    delay_microseconds_safe(ELERO_DELAY_SEND_PACKETS);
  }
  this->increase_counter();
}

void EleroCover::control(const cover::CoverCall &call) {
  if (call.get_stop()) {
    this->send_command(this->command_stop_);
  }
  if (call.get_position().has_value()) {
    auto pos = *call.get_position();
    if(pos == COVER_OPEN) {
      ESP_LOGD(TAG, "Sending OPEN command");
      this->send_command(this->command_up_);
      this->current_operation = COVER_OPERATION_OPENING;
      this->position = COVER_OPEN;
      this->movement_start_ = millis();
      this->publish_state();
    } else if(pos == COVER_CLOSED) {
      ESP_LOGD(TAG, "Sending CLOSE command");
      this->send_command(this->command_down_);
      this->current_operation = COVER_OPERATION_CLOSING;
      this->position = COVER_CLOSED;
      this->movement_start_ = millis();
      this->publish_state();
    }
  }
}

} // namespace elero
} // namespace esphome
