#pragma once

#include "esphome/core/component.h"
#include "esphome/components/ble_client/ble_client.h"
//#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/defines.h"
#include "esphome/core/helpers.h"

#ifdef USE_TIME
#include "esphome/components/time/real_time_clock.h"
#else
#include <ctime>
#endif

//#include <esp_gattc_api.h>

namespace esphome::powerpal_ble {

struct PowerpalMeasurement {
  uint16_t pulses;
  time_t timestamp;
  uint32_t watt_hours;
};

class Powerpal : public esphome::ble_client::BLEClientNode, public Component {

 public:
  void setup() override;

  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param) override;
  void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) override;

  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  void set_pairing_code(uint32_t pairing_code) {
    pairing_code_[0] = (pairing_code & 0x000000FF);
    pairing_code_[1] = (pairing_code & 0x0000FF00) >> 8;
    pairing_code_[2] = (pairing_code & 0x00FF0000) >> 16;
    pairing_code_[3] = (pairing_code & 0xFF000000) >> 24;
  }
  void set_pulses_per_kwh(uint16_t pulses_per_kwh) { pulses_per_kwh_ = pulses_per_kwh; }

  void set_notification_interval(uint8_t reading_batch_size) { reading_batch_size_[0] = reading_batch_size; }
  void set_apikey(std::string powerpal_apikey) { powerpal_apikey_ = powerpal_apikey; }
  void set_device_id(std::string powerpal_device_id) { powerpal_device_id_ = powerpal_device_id; }

  void set_battery(sensor::Sensor *battery) { battery_ = battery; }
  void set_daily_energy_sensor(sensor::Sensor *daily_energy_sensor) { daily_energy_sensor_ = daily_energy_sensor; }
  void set_daily_pulses_sensor(sensor::Sensor *daily_pulses_sensor) { daily_pulses_sensor_ = daily_pulses_sensor;}
  void set_energy_sensor(sensor::Sensor *energy_sensor) { energy_sensor_ = energy_sensor; }
  void set_power_sensor(sensor::Sensor *power_sensor) { power_sensor_ = power_sensor; }
  void set_pulses_sensor(sensor::Sensor *pulses_sensor) { pulses_sensor_ = pulses_sensor;}
  void set_timestamp(sensor::Sensor *timestamp_sensor) { timestamp_sensor_ = timestamp_sensor;}
  void set_uptime(sensor::Sensor *uptime_sensor) { uptime_sensor_ = uptime_sensor;}
  void set_watt_hours(sensor::Sensor *watt_hours_sensor) {watt_hours_sensor_ = watt_hours_sensor;}

#ifdef USE_TIME
  void set_time(time::RealTimeClock *time) { time_ = time; }
#endif

 protected:
  void on_connect();
  void on_disconnect();

  void parse_battery_(const uint8_t *data, uint16_t length);
  void parse_measurement_(const uint8_t *data, uint16_t length);

  void request_subscription_(const char *trigger_reason);
  void reset_connection_state_();

  bool subscription_retry_scheduled_{false};
  bool establish_handles_retry_scheduled_{false};

  enum StateCodes {
     DISCONNECTED = 0,
     CONNECTION_PENDING,
     ESTABLISH_HANDLES_PENDING,
     SUBSCRIPTION_PENDING,
     SUBSCRIPTION_IN_PROGRESS,
     AUTHENICATED,
     FAILED_TO_AUTHENICATE,
  } powerpal_state_{DISCONNECTED};

  float pulse_multiplier_;

  sensor::Sensor *battery_{nullptr};
  sensor::Sensor *daily_energy_sensor_{nullptr};
  sensor::Sensor *daily_pulses_sensor_{nullptr};
  sensor::Sensor *energy_sensor_{nullptr};
  sensor::Sensor *power_sensor_{nullptr};
  sensor::Sensor *pulses_sensor_{nullptr};
  sensor::Sensor *timestamp_sensor_{nullptr};
  sensor::Sensor *uptime_sensor_{nullptr};
  sensor::Sensor *watt_hours_sensor_{nullptr};

  std::string powerpal_apikey_;
  std::string powerpal_device_id_;

  uint8_t pairing_code_[4];
  uint8_t reading_batch_size_[4] = {0x01, 0x00, 0x00, 0x00};

  uint16_t current_year_{0};
  uint16_t day_of_last_measurement_{0};

  // configured in YAML
  uint16_t pulses_per_kwh_;

  // handles
  uint16_t pairing_code_char_handle_{0};
  uint16_t reading_batch_size_char_handle_{0};

  uint16_t battery_char_handle_{0};
  uint16_t firmware_char_handle_{0};
  uint16_t led_sensitivity_char_handle_{0};
  uint16_t measurement_char_handle_{0};
  uint16_t serial_number_char_handle_{0};
  uint16_t uuid_char_handle_{0};

  uint64_t daily_pulses_{0};
  uint64_t total_pulses_{0};

#ifdef USE_TIME
  optional<time::RealTimeClock *> time_{};
#endif

  time_t start_unix_time_;

};

} // namespace esphome::powerpal_ble
