#ifndef __ME2_CO_SENSOR_H__
#define __ME2_CO_SENSOR_H__

#include "sensor.h"
#include <SoftwareSerial.h>

constexpr const unsigned long WARMUPTIME_ME2_MS = 3000;   // time needed to "warm up" the sensor
constexpr const unsigned long SAMPLETIME_ME2_MS = 1000;   // sample time for ME2 sensor
constexpr const unsigned long ME2_SENSOR_MIN_TIMEOUT = 30000UL; // minimum timeout for ME2

class ME2COSensor : public Sensor {

public:
  ME2COSensor(unsigned long sending_timeout = 30000UL);
  
  bool begin() override;

private:
    SoftwareSerial* me2Serial;
    void _fetch(JsonDocument &data) override;
    
    unsigned long ME2_error_count = 0;
    float last_co_concentration = -1.0;
    String last_co_str;
    unsigned long last_request_time = 0;
    unsigned long last_measure_time = 0;
    bool sensor_initialized = false;
    bool data_available = false;
    bool read_flag = false;
    
    // Protocol handling
    uint8_t receive_buffer[9];
    uint8_t buffer_index = 0;
    uint8_t checksum = 0;
    bool receiving_data = false;
    
    // Protocol constants
    static const uint8_t START_BYTE = 0xFF;
    
    // Helper methods
    bool checksum_valid(const uint8_t (&data)[9]);
    void send_command(const uint8_t* cmd, size_t len);
    void setup_qa_mode();
    void request_data();
    bool process_response();
    void reset_buffer();
};

#endif // __ME2_CO_SENSOR_H__