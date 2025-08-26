#include "me2_co_sensor.h"
#include "../utils.h"
#include "../defines.h"
#include "sensor_names.h"
#include "../config_manager/config_helpers.h"

// ME2 Protocol command arrays
static const uint8_t CMD_SWITCH_QA[] = {0xFF, 0x01, 0x78, 0x41, 0x00, 0x00, 0x00, 0x00, 0x46};
static const uint8_t CMD_REQUEST_DATA[] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};

ME2COSensor::ME2COSensor(unsigned long sending_timeout)
    : Sensor(sending_timeout) {
    if (sending_timeout > ME2_SENSOR_MIN_TIMEOUT) {
        this->sending_timeout = sending_timeout;
    } else {
        this->sending_timeout = ME2_SENSOR_MIN_TIMEOUT;
    }
    timeout = SAMPLETIME_ME2_MS;
    sensor_name = ME2_SENSOR_NAME;
    last_co_concentration = -1.0; // Initialize to invalid value to detect if no real data comes
    
    // Create SoftwareSerial instance - try reversed pins: 19 (RX), 18 (TX)
    me2Serial = new SoftwareSerial(19, 18);
}

bool ME2COSensor::begin() {
    me2Serial->begin(9600);
    me2Serial->setTimeout(1000);
    delay(500);
    
    debug_outln_info(F("Initializing ME2 CO sensor on pins 18-19..."));
    debug_outln_info(F("ME2 sensor requires proper 5V power supply"));
    
    // Set sensor to Q&A mode
    setup_qa_mode();
    delay(100);
    
    // Initialize timing
    last_measure_time = millis() - (sending_timeout - (WARMUPTIME_ME2_MS + 5000));
    last_request_time = 0;
    
    sensor_initialized = true;
    debug_outln_info(F("ME2 CO sensor initialized with fetch interval (sec): "), String(sending_timeout/1000));
    
    return sensor_initialized;
}

void ME2COSensor::_fetch(JsonDocument &data) {
    if (!sensor_initialized) {
        addValueToJSON(data, F("CO"), (float)-1.0, F("co1"), F("ppm"));
        return;
    }
    
    me2Serial->begin(9600);
    me2Serial->setTimeout(1000);
    
    unsigned long current_time = millis();
    
    // Check if it's time to make a new measurement cycle
    if (msSince(last_measure_time) >= sending_timeout) {
        debug_outln_info(F("Starting ME2 measurement cycle"));
        last_measure_time = current_time;
        last_request_time = 0; // Reset request timing
        data_available = false;
    }
    
    // During measurement window, request data periodically
    if (msSince(last_measure_time) < (sending_timeout - WARMUPTIME_ME2_MS)) {
        // We're in measurement phase
        if (current_time - last_request_time >= 3000) { // Request every 3 seconds
            request_data();
            last_request_time = current_time;
            debug_outln_verbose(F("ME2 data requested"));
        }
        
        // Process any incoming response
        if (process_response()) {
            debug_outln_info(F("ME2 CO (ppm): "), String(last_co_concentration));
            addValueToJSON(data, F("CO"), last_co_concentration, F("co1"), F("ppm"));
            data_available = true;
        }
    } else {
        // Measurement window ended, provide final result
        if (data_available) {
            addValueToJSON(data, F("CO"), last_co_concentration, F("co1"), F("ppm"));
        } else {
            // No data received during this cycle
            ME2_error_count++;
            debug_outln_error(F("ME2 CO: No data received in measurement window"));
            debug_outln_info(F("ME2 error_count: "), String(ME2_error_count));
            addValueToJSON(data, F("CO"), (float)-1.0, F("co1"), F("ppm"));
        }
    }
}

void ME2COSensor::setup_qa_mode() {
    debug_outln_info(F("ME2 switching to Q&A mode..."));
    send_command(CMD_SWITCH_QA, sizeof(CMD_SWITCH_QA));
    debug_outln_info(F("ME2 Q&A mode command sent"));
}

void ME2COSensor::request_data() {
    send_command(CMD_REQUEST_DATA, sizeof(CMD_REQUEST_DATA));
    reset_buffer();
}

void ME2COSensor::send_command(const uint8_t* cmd, size_t len) {
    debug_outln_info(F("ME2 sending command of length: "), String(len));
    // Print command bytes for debugging
    String cmd_str = "ME2 CMD: ";
    for (size_t i = 0; i < len; i++) {
        cmd_str += "0x" + String(cmd[i], HEX) + " ";
    }
    debug_outln_info(cmd_str);
    
    me2Serial->write(cmd, len);
    me2Serial->flush(); // Ensure data is sent
    debug_outln_info(F("ME2 command sent"));
}

bool ME2COSensor::process_response() {
    int available_bytes = me2Serial->available();
    if (available_bytes > 0) {
        debug_outln_verbose(F("ME2 received bytes: "), String(available_bytes));
    }
    
    while (me2Serial->available() > 0) {
        uint8_t byte = me2Serial->read();
        
        // Look for start byte
        if (!receiving_data && byte == START_BYTE) {
            reset_buffer();
            receiving_data = true;
            buffer_index = 0;
            checksum = 0;
        }
        
        if (receiving_data) {
            receive_buffer[buffer_index] = byte;
            
            // Calculate checksum for bytes 1-7 (exclude start byte and checksum byte)
            if (buffer_index > 0 && buffer_index < 8) {
                checksum += byte;
            }
            
            buffer_index++;
            
            // Complete packet received
            if (buffer_index >= 9) {
                receiving_data = false;
                
                // Validate checksum: checksum = ~(sum of bytes 1-7) + 1
                uint8_t expected_checksum = (~checksum) + 1;
                
                if (receive_buffer[8] == expected_checksum) {
                    // Valid packet - extract concentration
                    // According to protocol: concentration is in bytes 2 and 3 for Q&A mode response
                    uint16_t raw_concentration = (receive_buffer[2] << 8) | receive_buffer[3];
                    last_co_concentration = (float)raw_concentration * 0.1f; // Convert to ppm
                    
                    debug_outln_verbose(F("ME2 valid response (ppm): "), String(last_co_concentration));
                    return true;
                } else {
                    debug_outln_error(F("ME2 checksum error"));
                    debug_outln_verbose(F("ME2 checksum expected: "), String(expected_checksum, HEX));
                    debug_outln_verbose(F("ME2 checksum got: "), String(receive_buffer[8], HEX));
                    ME2_error_count++;
                }
                
                reset_buffer();
            }
        }
    }
    
    return false;
}

void ME2COSensor::reset_buffer() {
    buffer_index = 0;
    checksum = 0;
    receiving_data = false;
    memset(receive_buffer, 0, sizeof(receive_buffer));
}

bool ME2COSensor::checksum_valid(const uint8_t (&data)[9]) {
    uint8_t sum = 0;
    for (unsigned i = 1; i < 8; ++i) {
        sum += data[i];
    }
    uint8_t expected = (~sum) + 1;
    return (data[8] == expected);
}