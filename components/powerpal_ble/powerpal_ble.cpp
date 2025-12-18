#include "powerpal_ble.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

#ifdef USE_ESP32
namespace esphome {
namespace powerpal_ble {

static const char *const TAG = "powerpal_ble";

static const uint16_t PAIRING_CODE_DEFAULT_HANDLE         = 0x2E;
static const uint16_t READING_BATCH_SIZE_DEFAULT_HANDLE   = 0x33;

static const uint16_t BATTERY_CHAR_DEFAULT_HANDLE         = 0x10;
static const uint16_t FIRMWARE_CHAR_DEFAULT_HANDLE        = 0x3B;
static const uint16_t LED_SENSITIVITY_CHAR_DEFAULT_HANDLE = 0x25;
static const uint16_t MEASUREMENT_CHAR_DEFAULT_HANDLE     = 0x14;
static const uint16_t SERIAL_NUMBER_CHAR_DEFAULT_HANDLE   = 0x2B;
static const uint16_t UUID_CHAR_DEFAULT_HANDLE            = 0x28;

namespace espbt = esphome::esp32_ble_tracker;

static const espbt::ESPBTUUID POWERPAL_SERVICE_UUID =
    espbt::ESPBTUUID::from_raw("59DAABCD-12F4-25A6-7D4F-55961DCE4205");
static const espbt::ESPBTUUID POWERPAL_CHARACTERISTIC_PAIRING_CODE_UUID =
    espbt::ESPBTUUID::from_raw("59DA0011-12F4-25A6-7D4F-55961DCE4205");  // indicate, notify, read, write
static const espbt::ESPBTUUID POWERPAL_CHARACTERISTIC_READING_BATCH_SIZE_UUID =
    espbt::ESPBTUUID::from_raw("59DA0013-12F4-25A6-7D4F-55961DCE4205");  // indicate, notify, read, write
static const espbt::ESPBTUUID POWERPAL_CHARACTERISTIC_MEASUREMENT_UUID =
    espbt::ESPBTUUID::from_raw("59DA0001-12F4-25A6-7D4F-55961DCE4205");  // notify, read, write
static const espbt::ESPBTUUID POWERPAL_CHARACTERISTIC_UUID_UUID =
    espbt::ESPBTUUID::from_raw("59DA0009-12F4-25A6-7D4F-55961DCE4205");  // indicate, notify, read, write
static const espbt::ESPBTUUID POWERPAL_CHARACTERISTIC_SERIAL_UUID =
    espbt::ESPBTUUID::from_raw("59DA0010-12F4-25A6-7D4F-55961DCE4205");  // indicate, notify, read, write

static const espbt::ESPBTUUID POWERPAL_BATTERY_SERVICE_UUID = espbt::ESPBTUUID::from_uint16(0x180F);
static const espbt::ESPBTUUID POWERPAL_BATTERY_CHARACTERISTIC_UUID = espbt::ESPBTUUID::from_uint16(0x2A19);

static const float SECONDS_IN_MINUTE = 60.0;        // seconds
static const float KW_TO_W_CONVERSION  = 1000.0;    // conversion ratio

void Powerpal::setup() {
  this->authenticated_ = false;
  this->pulse_multiplier_ =
    ((SECONDS_IN_MINUTE * (float)(this->reading_batch_size_[0])) / ((float)(this->pulses_per_kwh_) / KW_TO_W_CONVERSION));

    // gurrier
  this->reset_connection_state_();
 }

void Powerpal::dump_config() {
  ESP_LOGCONFIG(TAG, "Powerpal:");
  ESP_LOGCONFIG(TAG, "  Pulses/kwh: %i", this->pulses_per_kwh_);
  ESP_LOGCONFIG(TAG, "  Interval: %i min", this->reading_batch_size_[0]);

  LOG_SENSOR("  ", "Battery", this->battery_);
  LOG_SENSOR("  ", "Power", this->power_sensor_);
  LOG_SENSOR("  ", "Daily Energy", this->daily_energy_sensor_);
  LOG_SENSOR("  ", "Total Energy", this->energy_sensor_);
}

// gurrier
void Powerpal::reset_connection_state_() {
  this->authenticated_ = false;
  this->pending_subscription_ = false;
  this->subscription_in_progress_ = false;
  this->subscription_retry_scheduled_ = false;

  this->pairing_code_char_handle_ = 0;
  this->reading_batch_size_char_handle_ = 0;
  this->measurement_char_handle_ = 0;
  this->battery_char_handle_ = 0;
  this->led_sensitivity_char_handle_ = 0;
  this->firmware_char_handle_ = 0;
  this->uuid_char_handle_ = 0;
  this->serial_number_char_handle_ = 0;

  this->stored_measurements_count_ = 0;
  this->stored_measurements_.clear();
  this->last_measurement_timestamp_s_ = 0;
  this->reconnect_pending_ = false;
  this->client_connected_ = false;
}

void Powerpal::on_connect() {
  ESP_LOGI(TAG, "[%s] Connected to Powerpal GATT server", this->parent_->address_str());
  this->client_connected_ = true;
  this->pending_subscription_ = true;
  this->subscription_in_progress_ = false;
  this->subscription_retry_scheduled_ = false;
  this->reconnect_pending_ = false;
  this->stored_measurements_.clear();
  this->stored_measurements_count_ = 0;
  this->last_measurement_timestamp_s_ = 0;
  this->authenticated_ = false;

  this->set_timeout(1000, [this]() { this->request_subscription_("post-connect"); });
}

void Powerpal::on_disconnect() {
  ESP_LOGW(TAG, "[%s] Disconnected from Powerpal GATT server", this->parent_->address_str());
  this->reset_connection_state_();

  if (!this->reconnect_pending_) {
    this->reconnect_pending_ = true;
    this->set_timeout(10000, [this]() {
      this->reconnect_pending_ = false;
      if (this->parent_ == nullptr)
        return;
      if (this->client_connected_) {
        ESP_LOGD(TAG, "[%s] Reconnect timer fired but client already connected", this->parent_->address_str());
        return;
      }
      ESP_LOGI(TAG, "[%s] Attempting BLE reconnect", this->parent_->address_str());
      this->pending_subscription_ = true;
      this->parent_->connect();
    });
  }
}


void Powerpal::parse_battery_(const uint8_t *data, uint16_t length) {
  ESP_LOGD(TAG, "Battery: DEC(%d): 0x%s", length, format_hex(data, length).c_str());
  if (length == 1) {
    this->battery_->publish_state(data[0]);
  }
}

void Powerpal::parse_measurement_(const uint8_t *data, uint16_t length) {
  //gurrier
  if (length < 6) {
    ESP_LOGW(TAG, "parse_measurement_: packet too short (%hu)", length);
    return;
  }

  ESP_LOGD(TAG, "Meaurement: DEC(%d): 0x%s", length, format_hex(data, length).c_str());
  if (length >= 6) {
    time_t unix_time = data[0];
    unix_time += (data[1] << 8);
    unix_time += (data[2] << 16);
    unix_time += (data[3] << 24);
    long int new_time = unix_time;

    struct tm *date_local = ::localtime(&unix_time);
    if (date_local->tm_year > this->current_year_) {
      this->start_unix_time_ = unix_time;
      this->current_year_ = date_local->tm_year;
    }

    uint16_t pulses_within_interval = data[4];
    pulses_within_interval += data[5] << 8;
    this->daily_pulses_ += pulses_within_interval;

    float avg_watts_within_interval = pulses_within_interval * this->pulse_multiplier_;

    ESP_LOGI(TAG, "Timestamp: %ld, Pulses: %" PRIu64 ", Average Watts within interval: %f W, Daily Pulses: %" PRIu64, unix_time, pulses_within_interval,
             avg_watts_within_interval, this->daily_pulses_);

    if (this->power_sensor_ != nullptr) {
      this->power_sensor_->publish_state(avg_watts_within_interval);
    }

    if (this->pulses_sensor_ != nullptr) {
      this->pulses_sensor_->publish_state(pulses_within_interval);
    }

    if (this->watt_hours_sensor_ != nullptr) {
       int mywatt_hrs = (uint32_t)roundf(pulses_within_interval * (this->pulses_per_kwh_ / KW_TO_W_CONVERSION));
       this->watt_hours_sensor_->publish_state(mywatt_hrs);
    }

    if (this->timestamp_sensor_ != nullptr) {
      this->timestamp_sensor_->publish_state(new_time);
    }

    if (this->uptime_sensor_ != nullptr) {
      int32_t seconds_since_start = (int32_t)(unix_time - this->start_unix_time_);
      float uptime_minutes = (float)(seconds_since_start) / SECONDS_IN_MINUTE;
      this->uptime_sensor_->publish_state(uptime_minutes);
    }

    if (this->energy_sensor_ != nullptr) {
      this->total_pulses_ += pulses_within_interval;
      float energy = (float)(this->total_pulses_) / (float)(this->pulses_per_kwh_);
      this->energy_sensor_->publish_state(energy);
    }

    if (this->daily_energy_sensor_ != nullptr) {
      // even if new day, publish last measurement window before resetting
      float energy = (float)(this->daily_pulses_) / (float)(this->pulses_per_kwh_);
      this->daily_energy_sensor_->publish_state(energy);

      if (this->daily_pulses_sensor_ != nullptr) {
        this->daily_pulses_sensor_->publish_state(daily_pulses_);
      }
      // if esphome device has a valid time component set up, use that (preferred)
      // else, use the powerpal measurement timestamps
#ifdef USE_TIME
      auto *time_ = *this->time_;
      esphome::ESPTime date_of_measurement = time_->now();
      if (date_of_measurement.is_valid()) {
        if (this->day_of_last_measurement_ == 0) { this->day_of_last_measurement_ = date_of_measurement.day_of_year;}
        else if (this->day_of_last_measurement_ != date_of_measurement.day_of_year) {
          this->daily_pulses_ = 0;
          this->day_of_last_measurement_ = date_of_measurement.day_of_year;
        }
      } else {
        // if !date_of_measurement.is_valid(), user may have a bare "time:" in their yaml without a specific platform selected, so fallback to date of powerpal measurement
#else
        // avoid using ESPTime here so we don't need a time component in the config
        struct tm *date_of_measurement = ::localtime(&unix_time);
        // date_of_measurement.tm_yday + 1 because we are matching ESPTime day of year (1-366 instead of 0-365), which lets us catch a day_of_last_measurement_ of 0 as uninitialised
        if (this->day_of_last_measurement_ == 0) {
          this->day_of_last_measurement_ = date_of_measurement->tm_yday + 1;
        }
        else if (this->day_of_last_measurement_ != date_of_measurement->tm_yday + 1) {
          this->daily_pulses_ = 0;
          this->day_of_last_measurement_ = date_of_measurement->tm_yday + 1;
        }
#endif
#ifdef USE_TIME
      }
#endif
    }
  }
} // parse_battery_

void Powerpal::decode_(const uint8_t *data, uint16_t length) {
  //ESP_LOGD(TAG, "DEC(%d): 0x%s", length, this->pkt_to_hex_(data, length).c_str());
  ESP_LOGD(TAG, "DEC(%d): 0x%s", length, format_hex(data, length).c_str());
}

// std::string Powerpal::pkt_to_hex_(const uint8_t* data, uint16_t len) {
  // char buf[64];
  // memset(buf, 0, 64);
  // for (int i = 0; i < len; i++)
  //   sprintf(&buf[i * 2], "%02x", data[i]);
  // std::string ret = buf;
  // return ret;

  //gurrier
  // if (data == nullptr || len == 0)
  //   return {};

  // return format_hex(data, len);

  // static constexpr char HEXMAP[] = "0123456789abcdef";
  // std::string ret;
  // ret.reserve(static_cast<size_t>(len) * 2);
  // for (uint16_t i = 0; i < len; i++) {
  //   uint8_t byte = data[i];
  //   ret.push_back(HEXMAP[(byte >> 4) & 0x0F]);
  //   ret.push_back(HEXMAP[byte & 0x0F]);
  // }
  // return ret;
// }
// std::string Powerpal::uuid_to_device_id_(const uint8_t *data, uint16_t length) {
  // const char* hexmap[] = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "a", "b", "c", "d", "e", "f"};
  // std::string device_id;
  // for (int i = length-1; i >= 0; i--) {
  //   device_id.append(hexmap[(data[i] & 0xF0) >> 4]);
  //   device_id.append(hexmap[data[i] & 0x0F]);
  // }
  // return device_id;
//   if (data == nullptr || length == 0)
//     return {};

//   static constexpr char HEXMAP[] = "0123456789abcdef";
//   std::string device_id;
//   device_id.reserve(static_cast<size_t>(length) * 2);
//   for (int i = static_cast<int>(length) - 1; i >= 0; i--) {
//     uint8_t byte = data[i];
//     device_id.push_back(HEXMAP[(byte & 0xF0) >> 4]);
//     device_id.push_back(HEXMAP[byte & 0x0F]);
//   }
//   return device_id;
// }

// std::string Powerpal::serial_to_apikey_(const uint8_t *data, uint16_t length) {
  // const char* hexmap[] = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "a", "b", "c", "d", "e", "f"};
  // std::string api_key;
  // for (int i = 0; i < length; i++) {
  //   if ( i == 4 || i == 6 || i == 8 || i == 10 ) {
  //     api_key.append("-");
  //   }
  //   api_key.append(hexmap[(data[i] & 0xF0) >> 4]);
  //   api_key.append(hexmap[data[i] & 0x0F]);
  // }
  // return api_key;
//   if (data == nullptr || length == 0)
//     return {};

//   static constexpr char HEXMAP[] = "0123456789abcdef";
//   std::string api_key;
//   api_key.reserve(static_cast<size_t>(length) * 2 + 4);
//   for (uint16_t i = 0; i < length; i++) {
//     if (i == 4 || i == 6 || i == 8 || i == 10) {
//       api_key.push_back('-');
//     }
//     uint8_t byte = data[i];
//     api_key.push_back(HEXMAP[(byte & 0xF0) >> 4]);
//     api_key.push_back(HEXMAP[byte & 0x0F]);
//   }
//   return api_key;
// }

void Powerpal::request_subscription_(const char *trigger_reason) {
  if (!this->pending_subscription_)
    return;

  if (this->subscription_in_progress_) {
    ESP_LOGV(TAG, "[%s] Subscription already in progress, ignoring trigger '%s'", this->parent_->address_str(), trigger_reason);
    return;
  }

  if (this->pairing_code_char_handle_ == 0 || this->reading_batch_size_char_handle_ == 0 || this->measurement_char_handle_ == 0) {
    ESP_LOGD(TAG, "[%s] GATT handles not ready, waiting to subscribe (%s)", this->parent_->address_str(), trigger_reason);
    if (!this->subscription_retry_scheduled_) {
      this->subscription_retry_scheduled_ = true;
      this->set_timeout(500, [this]() {
        this->subscription_retry_scheduled_ = false;
        this->request_subscription_("wait-handles");
      });
    }
    return;
  }

  ESP_LOGI(TAG, "[%s] Writing pairing code to resume notifications (%s)", this->parent_->address_str(), trigger_reason);
  auto status = esp_ble_gattc_write_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                         this->pairing_code_char_handle_, sizeof(this->pairing_code_),
                                         this->pairing_code_, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
  if (status != ESP_OK) {
    ESP_LOGW(TAG, "[%s] Failed to submit pairing write (%s), status=%d", this->parent_->address_str(), trigger_reason, status);
    if (!this->subscription_retry_scheduled_) {
      this->subscription_retry_scheduled_ = true;
      this->set_timeout(2000, [this]() {
        this->subscription_retry_scheduled_ = false;
        this->request_subscription_("retry");
      });
    }
    return;
  }

  this->subscription_in_progress_ = true;
}

void Powerpal::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                   esp_ble_gattc_cb_param_t *param) {
  switch (event) {
    case ESP_GATTC_OPEN_EVT: {
      if (param->open.status == ESP_GATT_OK) {
        ESP_LOGD(TAG, "[%s] ESP_GATTC_OPEN_EVT", this->parent_->address_str());
        this->on_connect();
      } else {
        ESP_LOGW(TAG, "[%s] ESP_GATTC_OPEN_EVT failed, status=%d", this->parent_->address_str(),
                 param->open.status);
        this->reset_connection_state_();
      }
      break;
    }
    case ESP_GATTC_DISCONNECT_EVT: {
      ESP_LOGW(TAG, "[%s] ESP_GATTC_DISCONNECT_EVT", this->parent_->address_str());
      this->on_disconnect();
      break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT: {
      ESP_LOGI(TAG, "POWERPAL: services discovered, looking up characteristic handles");

      // Pairing Code
      if (auto *ch = this->parent_->get_characteristic(POWERPAL_SERVICE_UUID, POWERPAL_CHARACTERISTIC_PAIRING_CODE_UUID)) {
        this->pairing_code_char_handle_ = ch->handle;
        ESP_LOGI(TAG, "  → pairing_code handle = 0x%02x", ch->handle);
      } else {
        ESP_LOGE(TAG, "  Cannot discover characteristic: pairing code - setting to default");
        this->pairing_code_char_handle_ = PAIRING_CODE_DEFAULT_HANDLE;
      }

      // Reading Batch Size
      if (auto *ch = this->parent_->get_characteristic(POWERPAL_SERVICE_UUID, POWERPAL_CHARACTERISTIC_READING_BATCH_SIZE_UUID)) {
        this->reading_batch_size_char_handle_ = ch->handle;
        ESP_LOGI(TAG, "  → reading_batch_size handle = 0x%02x", ch->handle);
      } else {
        ESP_LOGE(TAG, "   Cannot discover characteristic: reading batch size - setting to default");
        this->reading_batch_size_char_handle_ = READING_BATCH_SIZE_DEFAULT_HANDLE;
      }

      // Measurement
      if (auto *ch = this->parent_->get_characteristic(POWERPAL_SERVICE_UUID, POWERPAL_CHARACTERISTIC_MEASUREMENT_UUID)) {
        this->measurement_char_handle_ = ch->handle;
        ESP_LOGI(TAG, "  → measurement handle = 0x%02x", ch->handle);
      } else {
        ESP_LOGE(TAG, "  Cannot discover characteristic: measurement - setting to default");
        this->measurement_char_handle_ = MEASUREMENT_CHAR_DEFAULT_HANDLE;
      }

      // (optional) UUID & serial if you need them:
      if (auto *ch = this->parent_->get_characteristic(POWERPAL_SERVICE_UUID, POWERPAL_CHARACTERISTIC_UUID_UUID)) {
        this->uuid_char_handle_ = ch->handle;
        ESP_LOGI(TAG, "  → uuid handle = 0x%02x", ch->handle);
      } else {
        ESP_LOGE(TAG, "  Cannot discover characteristic: uuid - setting to default");
        this->uuid_char_handle_ = UUID_CHAR_DEFAULT_HANDLE;
      }

      if (auto *ch = this->parent_->get_characteristic(POWERPAL_SERVICE_UUID, POWERPAL_CHARACTERISTIC_SERIAL_UUID)) {
        this->serial_number_char_handle_ = ch->handle;
        ESP_LOGI(TAG, "  → serial handle = 0x%02x", ch->handle);
       } else {
        ESP_LOGE(TAG, "  Cannot discover characteristic: serial - setting to default");
        this->serial_number_char_handle_ = SERIAL_NUMBER_CHAR_DEFAULT_HANDLE;
      }

      // set daults with no discovery
      battery_char_handle_ = BATTERY_CHAR_DEFAULT_HANDLE;
      firmware_char_handle_ = FIRMWARE_CHAR_DEFAULT_HANDLE;
      led_sensitivity_char_handle_ = LED_SENSITIVITY_CHAR_DEFAULT_HANDLE;

      this->pending_subscription_ = true;
      this->request_subscription_("service discovery");
      break;
    }
    case ESP_GATTC_READ_CHAR_EVT: {
      ESP_LOGD(TAG, "[%s] ESP_GATTC_READ_CHAR_EVT (Received READ)", this->parent_->address_str());
      if (param->read.status != ESP_GATT_OK) {
        ESP_LOGW(TAG, "Error reading char at handle %d, status=%d", param->read.handle, param->read.status);
        break;
      }
      // reading batch size
      if (param->read.handle == this->reading_batch_size_char_handle_) {
        ESP_LOGD(TAG, "Received reading_batch_size read event");
        this->decode_(param->read.value, param->read.value_len);
        if (param->read.value_len == 4) {
          if (param->read.value[0] != this->reading_batch_size_[0]) {
            // reading batch size needs changing, so write
            auto status =
                esp_ble_gattc_write_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                         this->reading_batch_size_char_handle_, sizeof(this->reading_batch_size_),
                                         this->reading_batch_size_, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
            if (status) {
              ESP_LOGW(TAG, "Error sending write request for batch_size, status=%d", status);
            }
          } else {
            // reading batch size is set correctly so subscribe to measurement notifications
            auto status = esp_ble_gattc_register_for_notify(this->parent_->get_gattc_if(), this->parent_->get_remote_bda(),
                                                            this->measurement_char_handle_);
            if (status) {
              ESP_LOGW(TAG, "[%s] esp_ble_gattc_register_for_notify failed, status=%d",
                       this->parent_->address_str(), status);
            }
          }
        } else {
          // error, length should be 4
        }
        break;
      }

      // battery
      if (param->read.handle == this->battery_char_handle_) {
        ESP_LOGD(TAG, "Received battery read event");
        this->parse_battery_(param->read.value, param->read.value_len);
        break;
      }

      // firmware
      if (param->read.handle == this->firmware_char_handle_) {
        ESP_LOGD(TAG, "Received firmware read event");
        this->decode_(param->read.value, param->read.value_len);
        break;
      }

      // led sensitivity
      if (param->read.handle == this->led_sensitivity_char_handle_) {
        ESP_LOGD(TAG, "Received led sensitivity read event");
        this->decode_(param->read.value, param->read.value_len);
        break;
      }

      // serialNumber
      if (param->read.handle == this->serial_number_char_handle_) {
        ESP_LOGI(TAG, "Received serial_number read event");
        this->powerpal_device_id_ = format_hex(param->read.value, param->read.value_len);
        ESP_LOGI(TAG, "Powerpal device id: %s", this->powerpal_device_id_.c_str());

        break;
      }

      // uuid
      if (param->read.handle == this->uuid_char_handle_) {
        ESP_LOGI(TAG, "Received uuid read event");
        this->powerpal_apikey_ = format_hex(param->read.value, param->read.value_len);
        ESP_LOGI(TAG, "Powerpal apikey: %s", this->powerpal_apikey_.c_str());

        break;
      }

      break;
    }

    case ESP_GATTC_WRITE_CHAR_EVT: {
      ESP_LOGD(TAG, "[%s] ESP_GATTC_WRITE_CHAR_EVT (Write confirmed)", this->parent_->address_str());

      if (param->write.handle == this->pairing_code_char_handle_) {
        this->subscription_in_progress_ = false;
        if (param->write.status != ESP_GATT_OK) {
          ESP_LOGW(TAG, "Error writing pairing code at handle %d, status=%d", param->write.handle, param->write.status);
          this->pending_subscription_ = true;
          if (!this->subscription_retry_scheduled_) {
            this->subscription_retry_scheduled_ = true;
            this->set_timeout(2000, [this]() {
              this->subscription_retry_scheduled_ = false;
              this->request_subscription_("retry-after-fail");
            });
          }
          break;
        }

        this->authenticated_ = true;
        this->pending_subscription_ = false;
        this->subscription_retry_scheduled_ = false;

        auto read_reading_batch_size_status =
            esp_ble_gattc_read_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                    this->reading_batch_size_char_handle_, ESP_GATT_AUTH_REQ_NONE);
        if (read_reading_batch_size_status) {
          ESP_LOGW(TAG, "Error sending read request for reading batch size, status=%d", read_reading_batch_size_status);
        }

        if (!this->powerpal_apikey_.length()) {
          // read uuid (apikey)
          auto read_uuid_status = esp_ble_gattc_read_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                                          this->uuid_char_handle_, ESP_GATT_AUTH_REQ_NONE);
          if (read_uuid_status) {
            ESP_LOGW(TAG, "Error sending read request for powerpal uuid, status=%d", read_uuid_status);
          }
        }
        if (!this->powerpal_device_id_.length()) {
          // read serial number (device id)
          auto read_serial_number_status = esp_ble_gattc_read_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                                                  this->serial_number_char_handle_, ESP_GATT_AUTH_REQ_NONE);
          if (read_serial_number_status) {
            ESP_LOGW(TAG, "Error sending read request for powerpal serial number, status=%d", read_serial_number_status);
          }
        }

        if (this->battery_ != nullptr) {
          // read battery
          auto read_battery_status = esp_ble_gattc_read_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                                             this->battery_char_handle_, ESP_GATT_AUTH_REQ_NONE);
          if (read_battery_status) {
            ESP_LOGW(TAG, "Error sending read request for battery, status=%d", read_battery_status);
          }
          // Enable notifications for battery
          auto notify_battery_status = esp_ble_gattc_register_for_notify(
              this->parent_->get_gattc_if(), this->parent_->get_remote_bda(), this->battery_char_handle_);
          if (notify_battery_status) {
            ESP_LOGW(TAG, "[%s] esp_ble_gattc_register_for_notify failed, status=%d",
                     this->parent_->address_str(), notify_battery_status);
          }
        }

        // read firmware version
        auto read_firmware_status =
            esp_ble_gattc_read_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                    this->firmware_char_handle_, ESP_GATT_AUTH_REQ_NONE);
        if (read_firmware_status) {
          ESP_LOGW(TAG, "Error sending read request for led sensitivity, status=%d", read_firmware_status);
        }

        // read led sensitivity
        auto read_led_sensitivity_status =
            esp_ble_gattc_read_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                    this->led_sensitivity_char_handle_, ESP_GATT_AUTH_REQ_NONE);
        if (read_led_sensitivity_status) {
          ESP_LOGW(TAG, "Error sending read request for led sensitivity, status=%d", read_led_sensitivity_status);
        }

        break;
      }

      if (param->write.status != ESP_GATT_OK) {
        ESP_LOGW(TAG, "Error writing value to char at handle %d, status=%d", param->write.handle, param->write.status);
        break;
      }

      if (param->write.handle == this->reading_batch_size_char_handle_) {
        // reading batch size is now set correctly so subscribe to measurement notifications
        auto status = esp_ble_gattc_register_for_notify(this->parent_->get_gattc_if(), this->parent_->get_remote_bda(),
                                                        this->measurement_char_handle_);
        if (status) {
          ESP_LOGW(TAG, "[%s] esp_ble_gattc_register_for_notify failed, status=%d",
                   this->parent_->address_str(), status);
        }
        break;
      }

      ESP_LOGW(TAG, "[%s] Missed all handle matches: %d",
               this->parent_->address_str(), param->write.handle);
      break;
    }  // ESP_GATTC_WRITE_CHAR_EVT
    case ESP_GATTC_NOTIFY_EVT: {
      ESP_LOGD(TAG, "[%s] Received Notification", this->parent_->address_str());

      // battery
      if (param->notify.handle == this->battery_char_handle_) {
        ESP_LOGD(TAG, "Received battery notify event");
        this->parse_battery_(param->notify.value, param->notify.value_len);
        break;
      }

      // measurement
      if (param->notify.handle == this->measurement_char_handle_) {
        ESP_LOGD(TAG, "Received measurement notify event");
        this->parse_measurement_(param->notify.value, param->notify.value_len);
        break;
      }
      break;  // registerForNotify
    }
    default:
      break;
  }
}

void Powerpal::gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  switch (event) {
    // This event is sent once authentication has completed
    case ESP_GAP_BLE_AUTH_CMPL_EVT: {
      if (param->ble_security.auth_cmpl.success) {
        ESP_LOGI(TAG, "[%s] Authentication completed", this->parent_->address_str());
        this->pending_subscription_ = true;
        this->subscription_in_progress_ = false;
        this->subscription_retry_scheduled_ = false;
        this->request_subscription_("auth-complete");
      } else {
        ESP_LOGW(TAG, "[%s] Authentication failed, reason=0x%02x", this->parent_->address_str(),
                 param->ble_security.auth_cmpl.fail_reason);
        this->pending_subscription_ = false;
        this->subscription_in_progress_ = false;
        this->subscription_retry_scheduled_ = false;
      }
      break;
    }
    default:
      break;
  }
}

}  // namespace powerpal_ble
}  // namespace esphome

#endif
