// Copyright (c) Microsoft Corporation. All rights reserved.
// SPDX-License-Identifier: MIT

#include <stdarg.h>
#include <stdlib.h>
#include <DHT.h>
#include <az_core.h>
#include <az_iot.h>
#include "AzureIoT.h"
#include "Azure_IoT_PnP_Template.h"
#include "iot_configs.h"
#include <az_precondition_internal.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h> 
#include <RTClib.h>

static DHT dht(DHT_PIN,DHT_TYPE);
/* --- Defines --- */
#define AZURE_PNP_MODEL_ID "dtmi:azureiot:devkit:freertos:Esp32AzureIotKit;1"
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

#define SAMPLE_DEVICE_INFORMATION_NAME "deviceInformation"
#define SAMPLE_MANUFACTURER_PROPERTY_NAME "manufacturer"
#define SAMPLE_MODEL_PROPERTY_NAME "model"
#define SAMPLE_SOFTWARE_VERSION_PROPERTY_NAME "swVersion"
#define SAMPLE_OS_NAME_PROPERTY_NAME "osName"
#define SAMPLE_PROCESSOR_ARCHITECTURE_PROPERTY_NAME "processorArchitecture"
#define SAMPLE_PROCESSOR_MANUFACTURER_PROPERTY_NAME "processorManufacturer"
#define SAMPLE_TOTAL_STORAGE_PROPERTY_NAME "totalStorage"
#define SAMPLE_TOTAL_MEMORY_PROPERTY_NAME "totalMemory"

#define SAMPLE_MANUFACTURER_PROPERTY_VALUE "ESPRESSIF"
#define SAMPLE_MODEL_PROPERTY_VALUE "ESP32 Azure IoT Kit"
#define SAMPLE_VERSION_PROPERTY_VALUE "1.0.0"
#define SAMPLE_OS_NAME_PROPERTY_VALUE "FreeRTOS"
#define SAMPLE_ARCHITECTURE_PROPERTY_VALUE "ESP32 WROVER-B"
#define SAMPLE_PROCESSOR_MANUFACTURER_PROPERTY_VALUE "ESPRESSIF"
// The next couple properties are in KiloBytes.
#define SAMPLE_TOTAL_STORAGE_PROPERTY_VALUE 4096
#define SAMPLE_TOTAL_MEMORY_PROPERTY_VALUE 8192

#define TELEMETRY_PROP_NAME_TEMPERATURE "temperature"
#define TELEMETRY_PROP_NAME_HUMIDITY "humidity"
#define TELEMETRY_PROP_NAME_LIGHT "light"
#define TELEMETRY_PROP_NAME_QUALITY "quality"
#define TELEMETRY_PROP_NAME_MOTION "motion"

static az_span COMMAND_NAME_TOGGLE_LED_1 = AZ_SPAN_FROM_STR("ToggleLed1");
static az_span COMMAND_NAME_TOGGLE_LED_2 = AZ_SPAN_FROM_STR("ToggleLed2");
static az_span COMMAND_NAME_DISPLAY_TEXT = AZ_SPAN_FROM_STR("DisplayText");
#define COMMAND_RESPONSE_CODE_ACCEPTED 202
#define COMMAND_RESPONSE_CODE_REJECTED 404

#define WRITABLE_PROPERTY_TELEMETRY_FREQ_SECS "telemetryFrequencySecs"
#define WRITABLE_PROPERTY_RESPONSE_SUCCESS "success"

#define DOUBLE_DECIMAL_PLACE_DIGITS 2

/* --- Function Checks and Returns --- */
#define RESULT_OK 0
#define RESULT_ERROR __LINE__

#define EXIT_IF_TRUE(condition, retcode, message, ...) \
  do                                                   \
  {                                                    \
    if (condition)                                     \
    {                                                  \
      LogError(message, ##__VA_ARGS__);                \
      return retcode;                                  \
    }                                                  \
  } while (0)

#define EXIT_IF_AZ_FAILED(azresult, retcode, message, ...) \
  EXIT_IF_TRUE(az_result_failed(azresult), retcode, message, ##__VA_ARGS__)

/* --- Data --- */
#define DATA_BUFFER_SIZE 1024
static uint8_t data_buffer[DATA_BUFFER_SIZE];
static uint32_t telemetry_send_count = 0;

static size_t telemetry_frequency_in_seconds = 10; // With default frequency of once in 10 seconds.
static time_t last_telemetry_send_time = INDEFINITE_TIME;

static bool led1_on = false;
static bool led2_on = false;

/* --- Function Prototypes --- */
/* Please find the function implementations at the bottom of this file */
static int generate_telemetry_payload(
    uint8_t* payload_buffer,
    size_t payload_buffer_size,
    size_t* payload_buffer_length);
static int generate_device_info_payload(
    az_iot_hub_client const* hub_client,
    uint8_t* payload_buffer,
    size_t payload_buffer_size,
    size_t* payload_buffer_length);
static int consume_properties_and_generate_response(
    azure_iot_t* azure_iot,
    az_span properties,
    uint8_t* buffer,
    size_t buffer_size,
    size_t* response_length);

/* --- Public Functions --- */ 
void azure_pnp_init() { 
  dht.begin(); 
  pinMode(LED_MDL, OUTPUT);

  pinMode(MDL, INPUT);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  delay(2000);
  display.clearDisplay();
  display.setTextColor(WHITE);
}


const az_span azure_pnp_get_model_id() { return AZ_SPAN_FROM_STR(AZURE_PNP_MODEL_ID); }

void azure_pnp_set_telemetry_frequency(size_t frequency_in_seconds)
{
  telemetry_frequency_in_seconds = frequency_in_seconds;
  LogInfo("Telemetry frequency set to once every %d seconds.", telemetry_frequency_in_seconds);
}

/* Application-specific data section */

int azure_pnp_send_telemetry(azure_iot_t* azure_iot)
{

  _az_PRECONDITION_NOT_NULL(azure_iot);
  int sensorValue = analogRead(MDL);

  // Convert the analog value to a voltage (optional)
  float voltage = sensorValue * (3.3 / 4095.0);
  display.setTextSize(2);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Temp:");
  display.print(dht.readTemperature());
  display.display();
  delay(2000);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Hum:");
  display.print(dht.readHumidity());
  display.display();

   
  int rainThreshold = 2000; // Adjust this threshold based on your sensor and testing

  if (sensorValue < rainThreshold) {
digitalWrite(LED_MDL, HIGH);
display.print ("\nRain");
  } else {
digitalWrite(LED_MDL, LOW);

display.print("\nNotRain");
  }
display.display();
  // Add a delay before the next reading
  delay(1000);
 
  time_t now = time(NULL);

  if (now == INDEFINITE_TIME)
  {
    LogError("Failed getting current time for controlling telemetry.");
    return RESULT_ERROR;
  }
  else if (
      last_telemetry_send_time == INDEFINITE_TIME
      || difftime(now, last_telemetry_send_time) >= telemetry_frequency_in_seconds)
  {
    size_t payload_size;

    last_telemetry_send_time = now;

    if (generate_telemetry_payload(data_buffer, DATA_BUFFER_SIZE, &payload_size) != RESULT_OK)
    {
      LogError("Failed generating telemetry payload.");
      return RESULT_ERROR;
    }

    if (azure_iot_send_telemetry(azure_iot, az_span_create(data_buffer, payload_size)) != 0)
    {
      LogError("Failed sending telemetry.");
      return RESULT_ERROR;
    }
  }

  return RESULT_OK;
}

int azure_pnp_send_device_info(azure_iot_t* azure_iot, uint32_t request_id)
{
  _az_PRECONDITION_NOT_NULL(azure_iot);

  int result;
  size_t length;

  result = generate_device_info_payload(
      &azure_iot->iot_hub_client, data_buffer, DATA_BUFFER_SIZE, &length);
  EXIT_IF_TRUE(result != RESULT_OK, RESULT_ERROR, "Failed generating telemetry payload.");

  result = azure_iot_send_properties_update(
      azure_iot, request_id, az_span_create(data_buffer, length));
  EXIT_IF_TRUE(result != RESULT_OK, RESULT_ERROR, "Failed sending reported properties update.");

  return RESULT_OK;
}

int azure_pnp_handle_command_request(azure_iot_t* azure_iot, command_request_t command)
{
  _az_PRECONDITION_NOT_NULL(azure_iot);

  uint16_t response_code;

  if (az_span_is_content_equal(command.command_name, COMMAND_NAME_TOGGLE_LED_1))
  {
    led1_on = !led1_on;
    LogInfo("LED 1 state: %s", (led1_on ? "ON" : "OFF"));
    response_code = COMMAND_RESPONSE_CODE_ACCEPTED;
  }
  else if (az_span_is_content_equal(command.command_name, COMMAND_NAME_TOGGLE_LED_2))
  {
    led2_on = !led2_on;
    LogInfo("LED 2 state: %s", (led2_on ? "ON" : "OFF"));
    response_code = COMMAND_RESPONSE_CODE_ACCEPTED;
  }
  else if (az_span_is_content_equal(command.command_name, COMMAND_NAME_DISPLAY_TEXT))
  {
    // The payload comes surrounded by quotes, so to remove them we offset the payload by 1 and its
    // size by 2.
    LogInfo(
        "OLED display: %.*s", az_span_size(command.payload) - 2, az_span_ptr(command.payload) + 1);
    response_code = COMMAND_RESPONSE_CODE_ACCEPTED;
  }
  else
  {
    LogError(
        "Command not recognized (%.*s).",
        az_span_size(command.command_name),
        az_span_ptr(command.command_name));
    response_code = COMMAND_RESPONSE_CODE_REJECTED;
  }

  return azure_iot_send_command_response(
      azure_iot, command.request_id, response_code, AZ_SPAN_EMPTY);
}

int azure_pnp_handle_properties_update(
    azure_iot_t* azure_iot,
    az_span properties,
    uint32_t request_id)
{
  _az_PRECONDITION_NOT_NULL(azure_iot);
  _az_PRECONDITION_VALID_SPAN(properties, 1, false);

  int result;
  size_t length;

  result = consume_properties_and_generate_response(
      azure_iot, properties, data_buffer, DATA_BUFFER_SIZE, &length);
  EXIT_IF_TRUE(result != RESULT_OK, RESULT_ERROR, "Failed generating properties ack payload.");

  result = azure_iot_send_properties_update(
      azure_iot, request_id, az_span_create(data_buffer, length));
  EXIT_IF_TRUE(result != RESULT_OK, RESULT_ERROR, "Failed sending reported properties update.");

  return RESULT_OK;
}

/* --- Internal Functions --- */
static float simulated_get_temperature() { return dht.readTemperature();  }

static float simulated_get_humidity() { return dht.readHumidity(); }
 
static float simulated_get_ambientLight() { return 700.0; }
RTC_DS3231 rtc;
 
 
 


static int generate_telemetry_payload(
    uint8_t* payload_buffer,
    size_t payload_buffer_size,
    size_t* payload_buffer_length)
{
  az_json_writer jw;
  az_result rc;
  az_span payload_buffer_span = az_span_create(payload_buffer, payload_buffer_size);
  az_span json_span;
  float temperature, humidity, light, quality;
  bool motion;

  // Acquiring the simulated data.
  temperature = simulated_get_temperature();
  humidity = simulated_get_humidity();
  light = simulated_get_ambientLight(); 
  rc = az_json_writer_init(&jw, payload_buffer_span, NULL);
  EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Failed initializing json writer for telemetry.");

  rc = az_json_writer_append_begin_object(&jw);
  EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Failed setting telemetry json root.");

  rc = az_json_writer_append_property_name(&jw, AZ_SPAN_FROM_STR(TELEMETRY_PROP_NAME_TEMPERATURE));
  EXIT_IF_AZ_FAILED(
      rc, RESULT_ERROR, "Failed adding temperature property name to telemetry payload.");
  rc = az_json_writer_append_double(&jw, temperature, DOUBLE_DECIMAL_PLACE_DIGITS);
  EXIT_IF_AZ_FAILED(
      rc, RESULT_ERROR, "Failed adding temperature property value to telemetry payload.");

  rc = az_json_writer_append_property_name(&jw, AZ_SPAN_FROM_STR(TELEMETRY_PROP_NAME_HUMIDITY));
  EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Failed adding humidity property name to telemetry payload.");
  rc = az_json_writer_append_double(&jw, humidity, DOUBLE_DECIMAL_PLACE_DIGITS);
  EXIT_IF_AZ_FAILED(
      rc, RESULT_ERROR, "Failed adding humidity property value to telemetry payload.");

  rc = az_json_writer_append_property_name(&jw, AZ_SPAN_FROM_STR(TELEMETRY_PROP_NAME_LIGHT));
  EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Failed adding light property name to telemetry payload.");
  rc = az_json_writer_append_double(&jw, light, DOUBLE_DECIMAL_PLACE_DIGITS);
  EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Failed adding light property value to telemetry payload.");


  rc = az_json_writer_append_property_name(&jw, AZ_SPAN_FROM_STR(TELEMETRY_PROP_NAME_QUALITY));
  EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Failed adding quality property name to telemetry payload.");
  rc = az_json_writer_append_double(&jw, quality, DOUBLE_DECIMAL_PLACE_DIGITS);
  EXIT_IF_AZ_FAILED(
      rc, RESULT_ERROR, "Failed adding quality property value to telemetry payload.");

  rc = az_json_writer_append_property_name(&jw, AZ_SPAN_FROM_STR(TELEMETRY_PROP_NAME_MOTION));
  EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Failed adding ismotion property name to telemetry payload.");
  rc = az_json_writer_append_bool(&jw, motion);
  EXIT_IF_AZ_FAILED(
      rc, RESULT_ERROR, "Failed adding ismotion property value to telemetry payload.");
  rc = az_json_writer_append_end_object(&jw);
  EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Failed closing telemetry json payload.");

  json_span = az_json_writer_get_bytes_used_in_destination(&jw);

  if ((payload_buffer_size - az_span_size(json_span)) < 1)
  {
    LogError("Insufficient space for telemetry payload null terminator.");
    return RESULT_ERROR;
  }

  payload_buffer[az_span_size(json_span)] = '\0';
  *payload_buffer_length = az_span_size(json_span);

  return RESULT_OK;
}

static int generate_device_info_payload(
    az_iot_hub_client const* hub_client,
    uint8_t* payload_buffer,
    size_t payload_buffer_size,
    size_t* payload_buffer_length)
{
  az_result rc;
  az_json_writer jw;
  az_span payload_span = az_span_create(payload_buffer, payload_buffer_size);

  rc = az_json_writer_init(&jw, payload_span, NULL);
  EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Could not initialize the json writer");

  rc = az_json_writer_append_begin_object(&jw);
  EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Could not begin the json object");

  rc = az_json_writer_append_property_name(&jw, AZ_SPAN_FROM_STR(SAMPLE_DEVICE_INFORMATION_NAME));
  EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Could not append the property name");

  rc = az_json_writer_append_begin_object(&jw);
  EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Could not append the property value");

  rc = az_json_writer_append_property_name(&jw, AZ_SPAN_FROM_STR(SAMPLE_MANUFACTURER_PROPERTY_NAME));
  EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Could not append the property name");
  rc = az_json_writer_append_string(&jw, AZ_SPAN_FROM_STR(SAMPLE_MANUFACTURER_PROPERTY_VALUE));
  EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Could not append the property value");

  rc = az_json_writer_append_property_name(&jw, AZ_SPAN_FROM_STR(SAMPLE_MODEL_PROPERTY_NAME));
  EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Could not append the property name");
  rc = az_json_writer_append_string(&jw, AZ_SPAN_FROM_STR(SAMPLE_MODEL_PROPERTY_VALUE));
  EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Could not append the property value");

  rc = az_json_writer_append_property_name(&jw, AZ_SPAN_FROM_STR(SAMPLE_SOFTWARE_VERSION_PROPERTY_NAME));
  EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Could not append the property name");
  rc = az_json_writer_append_string(&jw, AZ_SPAN_FROM_STR(SAMPLE_VERSION_PROPERTY_VALUE));
  EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Could not append the property value");

  rc = az_json_writer_append_property_name(&jw, AZ_SPAN_FROM_STR(SAMPLE_OS_NAME_PROPERTY_NAME));
  EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Could not append the property name");
  rc = az_json_writer_append_string(&jw, AZ_SPAN_FROM_STR(SAMPLE_OS_NAME_PROPERTY_VALUE));
  EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Could not append the property value");

  rc = az_json_writer_append_property_name(&jw, AZ_SPAN_FROM_STR(SAMPLE_PROCESSOR_ARCHITECTURE_PROPERTY_NAME));
  EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Could not append the property name");
  rc = az_json_writer_append_string(&jw, AZ_SPAN_FROM_STR(SAMPLE_ARCHITECTURE_PROPERTY_VALUE));
  EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Could not append the property value");

  rc = az_json_writer_append_property_name(&jw, AZ_SPAN_FROM_STR(SAMPLE_PROCESSOR_MANUFACTURER_PROPERTY_NAME));
  EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Could not append the property name");
  rc = az_json_writer_append_string(&jw, AZ_SPAN_FROM_STR(SAMPLE_PROCESSOR_MANUFACTURER_PROPERTY_VALUE));
  EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Could not append the property value");

  rc = az_json_writer_append_property_name(&jw, AZ_SPAN_FROM_STR(SAMPLE_TOTAL_STORAGE_PROPERTY_NAME));
  EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Could not append the property name");
  rc = az_json_writer_append_double(&jw, SAMPLE_TOTAL_STORAGE_PROPERTY_VALUE, DOUBLE_DECIMAL_PLACE_DIGITS);
  EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Could not append the property value");

  rc = az_json_writer_append_property_name(&jw, AZ_SPAN_FROM_STR(SAMPLE_TOTAL_MEMORY_PROPERTY_NAME));
  EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Could not append the property name");
  rc = az_json_writer_append_double(&jw, SAMPLE_TOTAL_MEMORY_PROPERTY_VALUE, DOUBLE_DECIMAL_PLACE_DIGITS);
  EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Could not append the property value");

  rc = az_json_writer_append_end_object(&jw);
  EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Could not append end object");

  rc = az_json_writer_append_end_object(&jw);
  EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Could not append end object");

  payload_span = az_json_writer_get_bytes_used_in_destination(&jw);

  if ((payload_buffer_size - az_span_size(payload_span)) < 1)
  {
    LogError("Insufficient space for telemetry payload null terminator.");
    return RESULT_ERROR;
  }

  payload_buffer[az_span_size(payload_span)] = '\0';
  *payload_buffer_length = az_span_size(payload_span);

  return RESULT_OK;
}

static int consume_properties_and_generate_response(
    azure_iot_t* azure_iot,
    az_span properties,
    uint8_t* buffer,
    size_t buffer_size,
    size_t* response_length)
{
  az_result rc;
  az_json_reader jr;
  az_json_writer jw;
  int32_t incoming_frequency = 0;
  bool frequency_found = false;

  rc = az_json_reader_init(&jr, properties, NULL);
  EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Could not initialize json reader");

  rc = az_json_writer_init(&jw, az_span_create(buffer, buffer_size), NULL);
  EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Could not initialize json writer");

  rc = az_json_writer_append_begin_object(&jw);
  EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Could not begin json object");

  while (az_result_succeeded(rc = az_json_reader_next_token(&jr)))
  {
    if (jr.token.kind == AZ_JSON_TOKEN_PROPERTY_NAME
        && az_json_token_is_text_equal(&jr.token, AZ_SPAN_FROM_STR(WRITABLE_PROPERTY_TELEMETRY_FREQ_SECS)))
    {
      rc = az_json_reader_next_token(&jr);
      EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Could not read property value");

      rc = az_json_token_get_int32(&jr.token, &incoming_frequency);
      EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Could not get int32 from property value");

      frequency_found = true;

      rc = az_json_writer_append_property_name(&jw, AZ_SPAN_FROM_STR(WRITABLE_PROPERTY_TELEMETRY_FREQ_SECS));
      EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Could not append property name");

      rc = az_json_writer_append_int32(&jw, incoming_frequency);
      EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Could not append property value");
    }
  }

  if (frequency_found)
  {
    azure_pnp_set_telemetry_frequency(incoming_frequency);
  }

  rc = az_json_writer_append_property_name(&jw, AZ_SPAN_FROM_STR("response"));
  EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Could not append property name");

  rc = az_json_writer_append_begin_object(&jw);
  EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Could not append begin object");

  rc = az_json_writer_append_property_name(&jw, AZ_SPAN_FROM_STR(WRITABLE_PROPERTY_RESPONSE_SUCCESS));
  EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Could not append property name");

  rc = az_json_writer_append_bool(&jw, true);
  EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Could not append property value");

  rc = az_json_writer_append_end_object(&jw);
  EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Could not append end object");

  rc = az_json_writer_append_end_object(&jw);
  EXIT_IF_AZ_FAILED(rc, RESULT_ERROR, "Could not append end object");

  *response_length = az_span_size(az_json_writer_get_bytes_used_in_destination(&jw));

  return RESULT_OK;
}
