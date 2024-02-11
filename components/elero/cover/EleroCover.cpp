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
  auto restore = this->restore_state_();
  if (restore.has_value()) {
    restore->apply(this);
  } else {
    if((this->open_duration_ > 0) && (this->close_duration_ > 0))
      this->position = 0.5f;
  }
}

void EleroCover::loop() {
  uint32_t intvl = this->poll_intvl_;
  uint32_t now = millis();
  if(this->current_operation != COVER_OPERATION_IDLE) {
    if((now - ELERO_TIMEOUT_MOVEMENT) < this->movement_start_) // do not poll frequently for an extended period of time
      intvl = ELERO_POLL_INTERVAL_MOVING;
  }

  if((now > this->poll_offset_) && (now - this->poll_offset_ - this->last_poll_) > intvl) {
    this->commands_to_send_.push(this->command_check_);
    this->last_poll_ = now - this->poll_offset_;
  }

  this->handle_commands(now);

  if((this->current_operation != COVER_OPERATION_IDLE) && (this->open_duration_ > 0) && (this->close_duration_ > 0)) {
    this->recompute_position();
    if(this->is_at_target()) {
      this->commands_to_send_.push(this->command_stop_);
      this->current_operation = COVER_OPERATION_IDLE;
    }

    // Publish position every second
    if(now - this->last_publish_ > 1000) {
      this->publish_state(false);
      this->last_publish_ = now;
    }
  }
}

bool EleroCover::is_at_target() {
  // We return false as we don't want to send a stop command for completely open or
  // close - this is handled by the cover
  if((this->target_position_ == COVER_OPEN) || (this->target_position_ == COVER_CLOSED))
    return false;

  switch (this->current_operation) {
    case COVER_OPERATION_OPENING:
      return this->position >= this->target_position_;
    case COVER_OPERATION_CLOSING:
      return this->position <= this->target_position_;
    case COVER_OPERATION_IDLE:
    default:
      return true;
  }
}

void EleroCover::handle_commands(uint32_t now) {
  if((now - this->last_command_) > ELERO_DELAY_SEND_PACKETS) {
    if(this->commands_to_send_.size() > 0) {
      this->command_.payload[4] = this->commands_to_send_.front();
      if(this->parent_->send_command(&this->command_)) {
        this->send_packets_++;
        this->send_retries_ = 0;
        if(this->send_packets_ >= ELERO_SEND_PACKETS) {
          this->commands_to_send_.pop();
          this->send_packets_ = 0;
          this->increase_counter();
        }
      } else {
        this->send_retries_++;
        if(this->send_retries_ > ELERO_SEND_RETRIES) {
          this->send_retries_ = 0;
          this->commands_to_send_.pop();
        }
      }
      this->last_command_ = now;
    }
  }
}

float EleroCover::get_setup_priority() const { return setup_priority::DATA; }

cover::CoverTraits EleroCover::get_traits() {
  auto traits = cover::CoverTraits();
  traits.set_supports_stop(true);
  if((this->open_duration_ > 0) && (this->close_duration_ > 0))
    traits.set_supports_position(true);
  else
    traits.set_supports_position(false);
  traits.set_supports_toggle(false);
  traits.set_is_assumed_state(true);
  return traits;
}

void EleroCover::set_rx_state(uint8_t state) {
  ESP_LOGD(TAG, "Got state: 0x%02x for blind 0x%02x", state, this->command_.blind_addr);
  float pos = this->position;
  CoverOperation op = this->current_operation;

  switch(state) {
  case ELERO_STATE_TOP:
    pos = COVER_OPEN;
    op = COVER_OPERATION_IDLE;
    break;
  case ELERO_STATE_BOTTOM:
    pos = COVER_CLOSED;
    op = COVER_OPERATION_IDLE;
    break;
  case ELERO_STATE_START_MOVING_UP:
  case ELERO_STATE_MOVING_UP:
    op = COVER_OPERATION_OPENING;
    break;
  case ELERO_STATE_START_MOVING_DOWN:
  case ELERO_STATE_MOVING_DOWN:
    op = COVER_OPERATION_CLOSING;
    break;
  case ELERO_STATE_STOPPED:
    op = COVER_OPERATION_IDLE;
    break;
  default:
    op = COVER_OPERATION_IDLE;
  }

  if((pos != this->position) || (op != this->current_operation)) {
    this->position = pos;
    this->current_operation = op;
    this->publish_state();
  }
}

void EleroCover::increase_counter() {
  if(this->command_.counter == 0xff)
    this->command_.counter = 1;
  else
    this->command_.counter += 1;
}

void EleroCover::control(const cover::CoverCall &call) {
  if (call.get_stop()) {
    this->commands_to_send_.push(this->command_stop_);
    this->start_movement(COVER_OPERATION_IDLE);
  }
  if (call.get_position().has_value()) {
    auto pos = *call.get_position();
    this->target_position_ = pos;
    if((pos > this->position) || (pos == COVER_OPEN)) {
      ESP_LOGD(TAG, "Sending OPEN command");
      this->commands_to_send_.push(this->command_up_);
      this->start_movement(COVER_OPERATION_OPENING);

    } else {
      ESP_LOGD(TAG, "Sending CLOSE command");
      this->commands_to_send_.push(this->command_down_);
      this->start_movement(COVER_OPERATION_CLOSING);
    }
  }
}

void EleroCover::start_movement(CoverOperation dir) {
  if(dir == this->current_operation)
    return;

  this->current_operation = dir;
  this->movement_start_ = millis();
  this->last_recompute_time_ = millis();
  this->publish_state();
}

void EleroCover::recompute_position() {
  if(this->current_operation == COVER_OPERATION_IDLE)
    return;


  float dir;
  float action_dur;
  switch (this->current_operation) {
    case COVER_OPERATION_OPENING:
      dir = 1.0f;
      action_dur = this->open_duration_;
      break;
    case COVER_OPERATION_CLOSING:
      dir = -1.0f;
      action_dur = this->close_duration_;
      break;
    default:
      return;
  }

  const uint32_t now = millis();
  this->position += dir * (now - this->last_recompute_time_) / action_dur;
  this->position = clamp(this->position, 0.0f, 1.0f);

  this->last_recompute_time_ = now;

}

} // namespace elero
} // namespace esphome
