#include "pulse_meter_sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace pulse_meter {

static const char *const TAG = "pulse_meter";

void PulseMeterSensor::setup() {
  this->pin_->setup();
  this->isr_pin_ = pin_->to_isr();
  this->pin_->attach_interrupt(PulseMeterSensor::gpio_intr, this, gpio::INTERRUPT_RISING_EDGE);

  this->last_detected_edge_us_ = 0;
  this->last_valid_low_edge_us_ = 0;
  this->last_valid_high_edge_us_ = 0;
  this->debaunce = 0;
  this->sensor_is_high_ = this->isr_pin_.digital_read();
}

void PulseMeterSensor::loop() {

  
  if ((this->debaunce < micros() - 240000000){
    this->publish_state(0);
  }

  if(this->last_detected_edge_us_ < this->total_pulses_ && this->total_pulses_  > 1){
    this->last_detected_edge_us_ = this->total_pulses_;

    // We quantize our pulse widths to 1 ms to avoid unnecessary jitter
    const uint32_t pulse_width_ms = this->pulse_width_us_ / 1000;
    this->publish_state((60.0f * 1000.0f) / pulse_width_ms);

    if (this->total_sensor_ != nullptr) {
        const uint32_t total = this->total_pulses_;
        if (this->total_dedupe_.next(total)) {
        this->total_sensor_->publish_state(total);
        }
    }
  }
}

void PulseMeterSensor::set_total_pulses(uint32_t pulses) { this->total_pulses_ = pulses; }

void PulseMeterSensor::dump_config() {
  LOG_SENSOR("", "Pulse Meter", this);
  LOG_PIN("  Pin: ", this->pin_);
  if (this->filter_mode_ == FILTER_EDGE) {
    ESP_LOGCONFIG(TAG, "  Filtering rising edges less than %u µs apart", this->filter_us_);
  } else {
    ESP_LOGCONFIG(TAG, "  Filtering pulses shorter than %u µs", this->filter_us_);
  }
  ESP_LOGCONFIG(TAG, "  Assuming 0 pulses/min after not receiving a pulse for %us", this->timeout_us_ / 1000000);
}

void IRAM_ATTR PulseMeterSensor::gpio_intr(PulseMeterSensor *sensor) {
  // This is an interrupt handler - we can't call any virtual method from this method

  // Get the current time before we do anything else so the measurements are consistent
  const uint32_t now = micros();
  if(sensor->debaunce < now - 2000000){
    sendor->pulse_width_us_ = now - sensor->debaunce;
    sensor->debaunce = now;
    sensor->total_pulses_++;
  }

}

}  // namespace pulse_meter
}  // namespace esphome
