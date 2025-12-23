#include "powerpal_ble.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome::powerpal_ble {

namespace espbt = esphome::esp32_ble_tracker;

static const char *const TAG = "powerpal_ble";

static const espbt::ESPBTUUID POWERPAL_SERVICE_UUID = espbt::ESPBTUUID::from_raw("59DAABCD-12F4-25A6-7D4F-55961DCE4205");
static const espbt::ESPBTUUID POWERPAL_CHARACTERISTIC_PAIRING_CODE_UUID = espbt::ESPBTUUID::from_raw("59DA0011-12F4-25A6-7D4F-55961DCE4205");
static const espbt::ESPBTUUID POWERPAL_CHARACTERISTIC_READING_BATCH_SIZE_UUID = espbt::ESPBTUUID::from_raw("59DA0013-12F4-25A6-7D4F-55961DCE4205");
static const espbt::ESPBTUUID POWERPAL_CHARACTERISTIC_MEASUREMENT_UUID = espbt::ESPBTUUID::from_raw("59DA0001-12F4-25A6-7D4F-55961DCE4205");
static const espbt::ESPBTUUID POWERPAL_CHARACTERISTIC_UUID_UUID = espbt::ESPBTUUID::from_raw("59DA0009-12F4-25A6-7D4F-55961DCE4205");
static const espbt::ESPBTUUID POWERPAL_CHARACTERISTIC_SERIAL_UUID = espbt::ESPBTUUID::from_raw("59DA0010-12F4-25A6-7D4F-55961DCE4205");

static const espbt::ESPBTUUID POWERPAL_BATTERY_SERVICE_UUID = espbt::ESPBTUUID::from_uint16(0x180F);
static const espbt::ESPBTUUID POWERPAL_BATTERY_CHARACTERISTIC_UUID = espbt::ESPBTUUID::from_uint16(0x2A19);

static const float SECONDS_IN_MINUTE   = 60.0;
static const float KW_TO_W_CONVERSION  = 1000.0;    // conversion ratio

static const uint8_t BATTERY_STATUS_LENGTH = 1;
static const uint8_t MEASUREMENT_LENGTH    = 6;

static const uint16_t PAIRING_CODE_DEFAULT_HANDLE         = 0x2E;
static const uint16_t READING_BATCH_SIZE_DEFAULT_HANDLE   = 0x33;

static const uint16_t BATTERY_CHAR_DEFAULT_HANDLE         = 0x10;
static const uint16_t FIRMWARE_CHAR_DEFAULT_HANDLE        = 0x3B;
static const uint16_t LED_SENSITIVITY_CHAR_DEFAULT_HANDLE = 0x25;
static const uint16_t MEASUREMENT_CHAR_DEFAULT_HANDLE     = 0x14;
static const uint16_t SERIAL_NUMBER_CHAR_DEFAULT_HANDLE   = 0x2B;
static const uint16_t UUID_CHAR_DEFAULT_HANDLE            = 0x28;


void Powerpal::setup() {
  this->pulse_multiplier_ =
    ((SECONDS_IN_MINUTE * (float)(this->reading_batch_size_[0])) / ((float)(this->pulses_per_kwh_) / KW_TO_W_CONVERSION));

  this->reset_connection_state_();
 }

void Powerpal::dump_config() {
  ESP_LOGCONFIG(TAG, "Powerpal:");
  ESP_LOGCONFIG(TAG, "  Pulses / kWh: %i", this->pulses_per_kwh_);
  ESP_LOGCONFIG(TAG, "  Measurement Interval: %imin", this->reading_batch_size_[0]);

  LOG_SENSOR("  ", "Battery", this->battery_);
  LOG_SENSOR("  ", "Power", this->power_sensor_);
  LOG_SENSOR("  ", "Daily Energy", this->daily_energy_sensor_);
  LOG_SENSOR("  ", "Total Energy", this->energy_sensor_);
}

void Powerpal::reset_connection_state_() {
  this->powerpal_state_ = CONNECTION_PENDING;
  this->establish_handles_retry_scheduled_ = false;
  this->subscription_retry_scheduled_ = false;
}

void Powerpal::on_connect() {
  ESP_LOGI(TAG, "[%s] Connected to Powerpal GATT server", this->parent_->address_str());
  this->powerpal_state_ = ESTABLISH_HANDLES_PENDING;
  this->set_timeout(1000, [this]() { this->request_subscription_("Establish Handles Pending"); });
}

void Powerpal::request_subscription_(const char *trigger_reason) {
  if ((this->powerpal_state_ == CONNECTION_PENDING) || (this->powerpal_state_ == AUTHENICATED)) return;

  if (this->powerpal_state_ == SUBSCRIPTION_IN_PROGRESS) {
    ESP_LOGV(TAG, "[%s] Subscription already in progress, ignoring trigger '%s'", this->parent_->address_str(), trigger_reason);
    return;
  }

  if (this->powerpal_state_ == ESTABLISH_HANDLES_PENDING) {
    ESP_LOGD(TAG, "[%s] GATT handles not ready, waiting to subscribe (%s)", this->parent_->address_str(), trigger_reason);
    if (!this->establish_handles_retry_scheduled_) {
      this->establish_handles_retry_scheduled_ = true;
      this->set_timeout(500, [this]() {
        // still waiting for handles - nothing changes just fire off another delay before calling request subscription again
        this->establish_handles_retry_scheduled_ = false;
        this->request_subscription_("retry establish handles");
      });
    }
    return;
  }

  // SUBSCRITION_PENDING
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
        this->request_subscription_("retry subscription");
      });
    }
    return;
  }
// if write successful subscription is in progress
  this->powerpal_state_ = SUBSCRIPTION_IN_PROGRESS;
}


void Powerpal::on_disconnect() {
  ESP_LOGW(TAG, "[%s] Disconnected from Powerpal GATT server", this->parent_->address_str());
  this->subscription_retry_scheduled_ = false;
   this->establish_handles_retry_scheduled_ = false;
  this->powerpal_state_ = DISCONNECTED;

  this->set_timeout(10000, [this]() {
    if (this->parent_ == nullptr) return;
    if (this->powerpal_state_ >= ESTABLISH_HANDLES_PENDING) {
      ESP_LOGD(TAG, "[%s] Reconnect timer fired but client already connected", this->parent_->address_str());
      return;
    }
    ESP_LOGI(TAG, "[%s] Attempting BLE reconnect", this->parent_->address_str());
    this->powerpal_state_ = CONNECTION_PENDING;
    this->parent_->connect();
  });

}

void Powerpal::parse_battery_(const uint8_t *data, uint16_t length) {
  ESP_LOGI(TAG, "Battery state: 0x%s", format_hex(data, length).c_str());
  if (length == BATTERY_STATUS_LENGTH) {
    this->battery_->publish_state(data[0]);
  }
}

void Powerpal::parse_measurement_(const uint8_t *data, uint16_t length) {
  //gurrier
  if (length < MEASUREMENT_LENGTH ) {
    ESP_LOGW(TAG, "Skip parsing measuerment - packet length too short: (%hu)", length);
    return;
  }

  ESP_LOGD(TAG, "Measurement: 0x%s", format_hex(data, length).c_str());

  time_t unix_time = data[0];
  unix_time += (data[1] << 8);
  unix_time += (data[2] << 16);
  unix_time += (data[3] << 24);

  // When YAML configures time component, use that time when valid otherwise use measurement timestamp
  uint16_t today; // day to decide when day rolls over
  uint16_t year;  // year to decide when year rolls over
  bool have_valid_day{false};

#ifdef USE_TIME
  auto *time_ = *this->time_;
  esphome::ESPTime date_of_measurement = time_->now();
  if (date_of_measurement.is_valid()) {
    today = date_of_measurement.day_of_year;
    year = date_of_measurement.year;
    have_valid_day = true;
  } else {
    ESP_LOGD(TAG, "External Time invalid - use measurement timestamp instead");
  }
#endif

  if (!have_valid_day) {
    struct tm *date_of_measurement = ::localtime(&unix_time);
    today = date_of_measurement->tm_yday + 1;
    year = date_of_measurement->tm_year;
  }

  if (this->day_of_last_measurement_ == 0) { // first measurement
    this->day_of_last_measurement_ = today;
  } else if (this->day_of_last_measurement_ != today) { // new day
    this->day_of_last_measurement_ = today;
    this->daily_pulses_ = 0;
  }

  uint16_t pulses_within_interval = (uint16_t)data[4] + ((uint16_t)data[5] << 8);
  this->daily_pulses_ += pulses_within_interval;

  if (this->power_sensor_ != nullptr) {
    float avg_watts_within_interval = (float)(pulses_within_interval) * this->pulse_multiplier_;
    this->power_sensor_->publish_state(avg_watts_within_interval);
  }

  if (this->pulses_sensor_ != nullptr) {
    this->pulses_sensor_->publish_state(pulses_within_interval);
  }

  if (this->watt_hours_sensor_ != nullptr) {
      uint32_t mywatt_hrs = (uint32_t)(roundf((float)pulses_within_interval * ((float)this->pulses_per_kwh_) / KW_TO_W_CONVERSION));
      this->watt_hours_sensor_->publish_state(mywatt_hrs);
  }

  if (this->timestamp_sensor_ != nullptr) {
    this->timestamp_sensor_->publish_state((long int)unix_time);
  }

  if (this->uptime_sensor_ != nullptr) {
    // reset uptime when new year rolls over
    if (year > this->current_year_) {
      this->start_unix_time_ = unix_time;
      this->current_year_ = year;
    }

    int32_t seconds_since_start = (int32_t)(unix_time - this->start_unix_time_);
    float uptime_minutes = (float)seconds_since_start / SECONDS_IN_MINUTE;
    this->uptime_sensor_->publish_state(uptime_minutes);
  }

  if (this->energy_sensor_ != nullptr) {
    this->total_pulses_ += pulses_within_interval;
    float energy = (float)this->total_pulses_ / (float)this->pulses_per_kwh_;
    this->energy_sensor_->publish_state(energy);
  }

  if (this->daily_energy_sensor_ != nullptr) {
    float energy = (float)this->daily_pulses_ / (float)this->pulses_per_kwh_;
    this->daily_energy_sensor_->publish_state(energy);
  }

  if (this->daily_pulses_sensor_ != nullptr) {
      this->daily_pulses_sensor_->publish_state(this->daily_pulses_);
  }


}

void Powerpal::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param) {
  switch (event) {

    case ESP_GATTC_OPEN_EVT: {
      if (param->open.status == ESP_GATT_OK) {
        ESP_LOGD(TAG, " Open Event [%s] ", this->parent_->address_str());
        this->on_connect();
      } else {
        ESP_LOGW(TAG, "Open failed [%s], error=%d", this->parent_->address_str(), param->open.status);
        this->reset_connection_state_();
      }
      break;
    }

    case ESP_GATTC_DISCONNECT_EVT: {
      ESP_LOGW(TAG, "Disconnect Event [%s]", this->parent_->address_str());
      this->on_disconnect();
      break;
    }

    case ESP_GATTC_SEARCH_CMPL_EVT: {
      ESP_LOGI(TAG, "Search Event: establishing characteristic handles");

      // Pairing Code Handle
      if (auto *ch = this->parent_->get_characteristic(POWERPAL_SERVICE_UUID, POWERPAL_CHARACTERISTIC_PAIRING_CODE_UUID)) {
        this->pairing_code_char_handle_ = ch->handle;
        ESP_LOGI(TAG, "  Pairing Code handle: 0x%02x", ch->handle);
      } else {
        ESP_LOGE(TAG, "  Getting Pairing Code handle failed - setting default");
        this->pairing_code_char_handle_ = PAIRING_CODE_DEFAULT_HANDLE;
      }

      // Reading Batch Size Handle
      if (auto *ch = this->parent_->get_characteristic(POWERPAL_SERVICE_UUID, POWERPAL_CHARACTERISTIC_READING_BATCH_SIZE_UUID)) {
        this->reading_batch_size_char_handle_ = ch->handle;
        ESP_LOGI(TAG, "  Reading Batch Size handle: 0x%02x", ch->handle);
      } else {
        ESP_LOGE(TAG, "   Getting Reading Batch Size handle failed - setting default");
        this->reading_batch_size_char_handle_ = READING_BATCH_SIZE_DEFAULT_HANDLE;
      }

      // Measurement Handle
      if (auto *ch = this->parent_->get_characteristic(POWERPAL_SERVICE_UUID, POWERPAL_CHARACTERISTIC_MEASUREMENT_UUID)) {
        this->measurement_char_handle_ = ch->handle;
        ESP_LOGI(TAG, "  Measurement handle: 0x%02x", ch->handle);
      } else {
        ESP_LOGE(TAG, "  Getting Measurement handle failed - setting default");
        this->measurement_char_handle_ = MEASUREMENT_CHAR_DEFAULT_HANDLE;
      }

      // UUID Handle
      if (auto *ch = this->parent_->get_characteristic(POWERPAL_SERVICE_UUID, POWERPAL_CHARACTERISTIC_UUID_UUID)) {
        this->uuid_char_handle_ = ch->handle;
        ESP_LOGI(TAG, "  UUID handle: 0x%02x", ch->handle);
      } else {
        ESP_LOGE(TAG, "  Getting UUID handle failed - setting default");
        this->uuid_char_handle_ = UUID_CHAR_DEFAULT_HANDLE;
      }

      // Serial Number Handle
      if (auto *ch = this->parent_->get_characteristic(POWERPAL_SERVICE_UUID, POWERPAL_CHARACTERISTIC_SERIAL_UUID)) {
        this->serial_number_char_handle_ = ch->handle;
        ESP_LOGI(TAG, "  Serial Number handle: 0x%02x", ch->handle);
       } else {
        ESP_LOGE(TAG, "  Getting Serial Number handle failed - setting to default");
        this->serial_number_char_handle_ = SERIAL_NUMBER_CHAR_DEFAULT_HANDLE;
      }

      // Set defaults with no discovery
      this->battery_char_handle_ = BATTERY_CHAR_DEFAULT_HANDLE;
      this->firmware_char_handle_ = FIRMWARE_CHAR_DEFAULT_HANDLE;
      this->led_sensitivity_char_handle_ = LED_SENSITIVITY_CHAR_DEFAULT_HANDLE;

      this->powerpal_state_ = SUBSCRIPTION_PENDING;
      this->request_subscription_("Got Handles now pending subscription");
      break;
    }

    case ESP_GATTC_READ_CHAR_EVT: {
      ESP_LOGD(TAG, "Read Event [%s]", this->parent_->address_str());
      if (param->read.status != ESP_GATT_OK) {
        ESP_LOGW(TAG, "Read Event failed at handle %d, error=%d", param->read.handle, param->read.status);
        break;
      }

      // reading batch size
      if (param->read.handle == this->reading_batch_size_char_handle_) {
        ESP_LOGI(TAG, "Reading Batch Size: 0x%s", format_hex(param->read.value, param->read.value_len).c_str());
        if (param->read.value_len != 4) {
          ESP_LOGW(TAG, "Reading Batch Size has incorrect length: %d", param->read.value_len);
          break;
        }
        if (param->read.value[0] != this->reading_batch_size_[0]) {
          // reading batch size needs changing, so write
          auto write_status =
              esp_ble_gattc_write_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                        this->reading_batch_size_char_handle_, sizeof(this->reading_batch_size_),
                                        this->reading_batch_size_, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
          if (write_status) {
            ESP_LOGW(TAG, "Reading Batch Size Write request failed, errors=%d", write_status);
          }
        } else {
          // reading batch size is set correctly so subscribe to measurement notifications
          auto notify_status = esp_ble_gattc_register_for_notify(this->parent_->get_gattc_if(), this->parent_->get_remote_bda(), this->measurement_char_handle_);
          if (notify_status) {
            ESP_LOGW(TAG, "Register for Measurement notifications failed [%s], error=%d", this->parent_->address_str(), notify_status);
          }
        }
        break;
      }

      // battery
      if (param->read.handle == this->battery_char_handle_) {
        this->parse_battery_(param->read.value, param->read.value_len);
        break;
      }

      // firmware
      if (param->read.handle == this->firmware_char_handle_) {
        ESP_LOGI(TAG, "Firmware: 0x%s", format_hex(param->read.value, param->read.value_len).c_str());
        break;
      }

      // led sensitivity
      if (param->read.handle == this->led_sensitivity_char_handle_) {
        ESP_LOGI(TAG, "Led Sensitivity: 0x%s", format_hex(param->read.value, param->read.value_len).c_str());
        break;
      }

      // serialNumber
      if (param->read.handle == this->serial_number_char_handle_) {
        this->powerpal_device_id_ = format_hex(param->read.value, param->read.value_len);
        ESP_LOGI(TAG, "Device ID: %s", this->powerpal_device_id_.c_str());

        break;
      }

      // uuid
      if (param->read.handle == this->uuid_char_handle_) {
        this->powerpal_apikey_ = format_hex(param->read.value, param->read.value_len);
        ESP_LOGI(TAG, "API Key: %s", this->powerpal_apikey_.c_str());

        break;
      }

      break;
    }

    case ESP_GATTC_WRITE_CHAR_EVT: {
      ESP_LOGD(TAG, "Write Event [%s]", this->parent_->address_str());

      if (param->write.handle == this->pairing_code_char_handle_) {
        if (param->write.status != ESP_GATT_OK) {
          ESP_LOGW(TAG, "Pairing Code - write event failed at handle %d, error=%d", param->write.handle, param->write.status);
          this->powerpal_state_ = SUBSCRIPTION_PENDING;
          if (!this->subscription_retry_scheduled_) {
            this->subscription_retry_scheduled_ = true;
            this->set_timeout(2000, [this]() {
              this->subscription_retry_scheduled_ = false;
              this->request_subscription_("Retry subscription after pairing failed");
            });
          }
          break;
        }
        this->powerpal_state_ = AUTHENICATED;

        auto read_status =
            esp_ble_gattc_read_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(), this->reading_batch_size_char_handle_, ESP_GATT_AUTH_REQ_NONE);
        if (read_status) {
          ESP_LOGW(TAG, "Reading Batch Size - read request failed, error=%d", read_status);
        }

        if (!this->powerpal_apikey_.length()) {
          // read uuid
          read_status = esp_ble_gattc_read_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(), this->uuid_char_handle_, ESP_GATT_AUTH_REQ_NONE);
          if (read_status) {
            ESP_LOGW(TAG, "UUID - read request failed, error=%d", read_status);
          }
        }

        if (!this->powerpal_device_id_.length()) {
          // read serial number
          read_status = esp_ble_gattc_read_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(), this->serial_number_char_handle_, ESP_GATT_AUTH_REQ_NONE);
          if (read_status) {
            ESP_LOGW(TAG, "Serial Number - read request failed, error=%d", read_status);
          }
        }

        if (this->battery_ != nullptr) {
          // read battery
          read_status = esp_ble_gattc_read_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(), this->battery_char_handle_, ESP_GATT_AUTH_REQ_NONE);
          if (read_status) {
            ESP_LOGW(TAG, "Battery status - read request failed, error=%d", read_status);
          }
          // Enable notifications for battery
          auto battery_notify_status = esp_ble_gattc_register_for_notify(
                                       this->parent_->get_gattc_if(), this->parent_->get_remote_bda(), this->battery_char_handle_);
          if (battery_notify_status) {
            ESP_LOGW(TAG, "Battery status - register for notify failed [%s], error=%d", this->parent_->address_str(), battery_notify_status);
          }
        }

        // read firmware version
        read_status =
          esp_ble_gattc_read_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(), this->firmware_char_handle_, ESP_GATT_AUTH_REQ_NONE);
        if (read_status) {
          ESP_LOGW(TAG, "Firmware - read request failed, error=%d", read_status);
        }

        // read led sensitivity
        read_status =
          esp_ble_gattc_read_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(), this->led_sensitivity_char_handle_, ESP_GATT_AUTH_REQ_NONE);
        if (read_status) {
          ESP_LOGW(TAG, "Led sensitivity - read request failed, error=%d", read_status);
        }

        break;
      }

      if (param->write.status != ESP_GATT_OK) {
        ESP_LOGW(TAG, "Writing value failed at handle %d, error=%d", param->write.handle, param->write.status);
        break;
      }

      if (param->write.handle == this->reading_batch_size_char_handle_) {
        // reading batch size is now set correctly so subscribe to measurement notifications
        auto batch_size_notify_status = esp_ble_gattc_register_for_notify(this->parent_->get_gattc_if(), this->parent_->get_remote_bda(), this->measurement_char_handle_);
        if (batch_size_notify_status) {
          ESP_LOGW(TAG, "Reading Batch Size - register for notify failed [%s], error=%d", this->parent_->address_str(), batch_size_notify_status);
        }
        break;
      }

      ESP_LOGW(TAG, "Missed handle match [%s]: %d", this->parent_->address_str(), param->write.handle);
      break;
    }  // ESP_GATTC_WRITE_CHAR_EVT

    case ESP_GATTC_NOTIFY_EVT: {
      ESP_LOGD(TAG, "Notify Event [%s]", this->parent_->address_str());

      // battery
      if (param->notify.handle == this->battery_char_handle_) {
        this->parse_battery_(param->notify.value, param->notify.value_len);
        break;
      }

      // measurement
      if (param->notify.handle == this->measurement_char_handle_) {
        this->parse_measurement_(param->notify.value, param->notify.value_len);
        break;
      }
      break;
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
        ESP_LOGI(TAG, "Authentication completed [%s]", this->parent_->address_str());
        this->powerpal_state_ = SUBSCRIPTION_PENDING;
        this->request_subscription_("Authenication Complete");
      } else {
        ESP_LOGW(TAG, "Authentication failed [%s], reason=0x%02x", this->parent_->address_str(), param->ble_security.auth_cmpl.fail_reason);
        this->powerpal_state_ = FAILED_TO_AUTHENICATE;
        this->mark_failed();
      }
      break;
    }
    default:
      break;
  }
}

}  // namespace esphome::powerpal_ble
