/******************************************************************************

    written by stefan krueger (s-light),
        github@s-light.eu, http://s-light.eu, https://github.com/s-light/

******************************************************************************/
/******************************************************************************
    The MIT License (MIT)

    Copyright (c) 2015 Stefan Krüger

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
******************************************************************************/


/** Includes Core Arduino functionality **/
#if ARDUINO
    #if ARDUINO < 100
        #include <WProgram.h>
    #else
        #include <Arduino.h>
    #endif
#endif

#include <Wire.h>
#include "slight_CAP1188.h"
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// definitions
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// slight_CAP1188_TWI functions
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// slight_CAP1188_TWI::slight_CAP1188_TWI(const uint8_t reset_pin_new) {
//
// }

slight_CAP1188_TWI::slight_CAP1188_TWI(
    const uint8_t reset_pin_new,
    uint8_t twi_address_new
) :
    twi_address(twi_address_new),
    reset_pin(reset_pin_new)
{
    ready = false;

    sensor_input_status = 0;
    sensor_input_status_old = 0;

    timestamp_lastread = 0;
    update_interval = 90;
}


bool slight_CAP1188_TWI::begin() {
    if (ready == false) {
        // set registers to default configuration.
        // takes ~300ms
        sensor_HW_reset();

        // setup TWI
        Wire.begin();

        // set ready
        // otherwise write and read commands are not executed
        ready = true;

        // first ask sensor for some information so that all is ready.
        // uint8_t general_status = general_status_get();
        // Serial.print(F("general status: "));
        // Serial.print(general_status, BIN);
        // Serial.println();

        sensor_default_configuration();
    }
    return ready;
}

void slight_CAP1188_TWI::update() {
    sensor_input_status_update();
}

void slight_CAP1188_TWI::sensor_HW_reset() {
    if (reset_pin != NO_PIN) {
        // toggle reset pin for a Hardware reset of CAP1188
        pinMode(reset_pin, OUTPUT);
        digitalWrite(reset_pin, LOW);
        delay(100);
        digitalWrite(reset_pin, HIGH);
        delay(100);
        digitalWrite(reset_pin, LOW);
        delay(100);
    }
}

void slight_CAP1188_TWI::sensor_default_configuration() {
    // setup 'good' starting configuration
    // Link all leds to touch inputs
    sensor_input_led_linking_set(B11111111);
    // allow multiple touches
    multiple_touch_blocking_enable_set(0);
}


void slight_CAP1188_TWI::update_interval_set_autofit() {
    // TODO(s-light): implement update calculation with help from datasheet
    // 90ms is fine for default configuration
    update_interval = 90;
}

void slight_CAP1188_TWI::update_interval_set(uint32_t interval) {
    update_interval = interval;
}

uint32_t slight_CAP1188_TWI::update_interval_get() {
    return update_interval;
}


void slight_CAP1188_TWI::touch_event_set_callback(
    callback_t callback_function
) {
    callback_touch_event = callback_function;
}

// private
void slight_CAP1188_TWI::sensor_input_status_update() {
    // check if sensor is present
    if (ready) {
        // poll sensor every 90ms
        // at default configuration
        // this is the cycle time till all sensors are read.
        uint32_t duration = millis() - timestamp_lastread;
        if (duration > update_interval) {
            timestamp_lastread =  millis();

            sensor_input_status = sensor_input_status_get_raw();
            interrupt_clear();

            // filter for changes
            if (sensor_input_status != sensor_input_status_old) {
                sensor_input_status_old = sensor_input_status;

                // change in touch detected
                touch_event_callback();
                // Serial.print(F("touched: "));
                // for (size_t i=0; i<8; i++) {
                //     if (sensor_input_status & (1 << i)) {
                //         Serial.print("1");
                //     } else {
                //         Serial.print("0");
                //     }
                // }
                // Serial.println();
            }  // filter for changes
        }  // update_interval
    }  // if ready
}



void slight_CAP1188_TWI::touch_event_callback() {
    if (callback_touch_event) {
        callback_touch_event(this);
    }
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// register helper
// capitle title numers are referencing to:
// http://www.adafruit.com/datasheets/CAP1188.pdf
// in newer revisions the numers have changed to 6.n
// http://www.microchip.com/wwwproducts/Devices.aspx?product=CAP1188
// http://ww1.microchip.com/downloads/en/DeviceDoc/00001620C.pdf
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 5.1 Main Control Register

// Interrupt Bit
void slight_CAP1188_TWI::interrupt_clear() {
    // read register
    uint8_t reg = read_register(REG_Main_Control);
    // clear interrupt bit
    reg = reg & (~intterupt_mask);
    write_register(REG_Main_Control, reg);
}

bool slight_CAP1188_TWI::interrupt_get() {
    // read register
    uint8_t reg = read_register(REG_Main_Control);
    // isolate
    reg = reg & intterupt_mask;
    // convert to bool
    bool result = false;
    if (reg > 0) {
        result = true;
    }
    return result;
}

// Gain Settings
void slight_CAP1188_TWI::gain_set(gain_setting_t value) {
    // read Main Control Register
    uint8_t reg = read_register(REG_Main_Control);
    // info
    //     11000000 bits we want to change
    //
    //     01001001 main register
    //     11000000 input
    //
    //     110000000 input
    //   ~ 001111111 inv
    //
    //     01001001 Input
    //   & 001111111 Mask invertiert
    //     000110000 Output

    // clear gain bits
    reg = reg & (~gain_mask);
    // set gain bits
    reg = reg | value;
    // write Main control Register
    write_register(REG_Main_Control, reg);
}

slight_CAP1188_TWI::gain_setting_t slight_CAP1188_TWI::gain_get() {
    // read Main Control Register
    uint8_t reg = read_register(REG_Main_Control);
    // isolate gain bits
    gain_setting_t result = (gain_setting_t)(reg & gain_mask);
    return result;
}

void slight_CAP1188_TWI::gain_print(Print &out) {
    gain_print(out, gain_get());
}

void slight_CAP1188_TWI::gain_print(
    Print &out,
    slight_CAP1188_TWI::gain_setting_t value
) {
    switch (value) {
        case gain_1: {
            out.print(F("gain 1"));
        } break;
        case gain_2: {
            out.print(F("gain 2"));
        } break;
        case gain_4: {
            out.print(F("gain 4"));
        } break;
        case gain_8: {
            out.print(F("gain 8"));
        } break;
    }

    // out.print(F("gain "));
    // out.print(value >> 6, DEC);
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 5.2 Status Registers

// 5.2.1 General Status
uint8_t slight_CAP1188_TWI::general_status_get() {
    // read register
    uint8_t reg = read_register(REG_General_Status);
    return reg;
}

// 5.2.2 Sensor Input Status
uint8_t slight_CAP1188_TWI::sensor_input_status_get_raw() {
    // read register
    uint8_t reg = read_register(REG_Sensor_Input_Status);
    return reg;
}

uint8_t slight_CAP1188_TWI::sensor_input_status_get() {
    return sensor_input_status;
}

bool slight_CAP1188_TWI::sensor_input_status_get(uint8_t sensor) {
    if (sensor > 8) {
        sensor = 8;
    }
    if (sensor < 1) {
        sensor = 1;
    }
    sensor = sensor -1;

    uint8_t value = 0;
    // isolate
    value = sensor_input_status & (1 << sensor);

    // convert to bool
    bool result = false;
    if (value > 0) {
        result = true;
    }
    return result;
}

// 5.2.3 LED Status
uint8_t slight_CAP1188_TWI::led_status_get() {
    // read register
    uint8_t reg = read_register(REG_Sensor_Input_Status);
    return reg;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 5.3 noise flags

uint8_t slight_CAP1188_TWI::noise_flags_get() {
    // read register
    uint8_t reg = read_register(REG_Noise_Flag_Status);
    return reg;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 5.5 Sensitivity Control Register

void slight_CAP1188_TWI::sensitivity_set(
    slight_CAP1188_TWI::sensitivity_t value
) {
    // read register
    uint8_t reg = read_register(REG_Sensitivity_Control);
    // clear bits
    reg = reg & (~sensitivity_mask);
    // set bits
    reg = reg | value;
    // write register
    write_register(REG_Sensitivity_Control, reg);
}

slight_CAP1188_TWI::sensitivity_t slight_CAP1188_TWI::sensitivity_get() {
    // read register
    uint8_t reg = read_register(REG_Sensitivity_Control);
    // isolate bits
    uint8_t value = reg & sensitivity_mask;
    sensitivity_t result;
    result = (sensitivity_t)value;
    return result;
}

void slight_CAP1188_TWI::sensitivity_print(Print &out) {
    sensitivity_print(out, sensitivity_get());
}

void slight_CAP1188_TWI::sensitivity_print(
    Print &out,
    slight_CAP1188_TWI::sensitivity_t value
) {
    out.print(F("sensitivity "));
    switch (value) {
        case sensitivity_128x: {
            out.print(F("128x"));
        } break;
        case sensitivity_64x: {
            out.print(F("64x"));
        } break;
        case sensitivity_32x: {
            out.print(F("32x"));
        } break;
        case sensitivity_16x: {
            out.print(F("16x"));
        } break;
        case sensitivity_8x: {
            out.print(F("8x"));
        } break;
        case sensitivity_4x: {
            out.print(F("4x"));
        } break;
        case sensitivity_2x: {
            out.print(F("2x"));
        } break;
        case sensitivity_1x: {
            out.print(F("1x"));
        } break;
        default: {
            out.print(F("?x"));
        } break;
    }

    // out.print(F("gain "));
    // out.print(value >> 6, DEC);
}

slight_CAP1188_TWI::sensitivity_t slight_CAP1188_TWI::sensitivity_convert(
    uint8_t value
) {
    sensitivity_t result = sensitivity_64x;
    switch (value) {
        case 128: {
            result = sensitivity_128x;
        } break;
        case 64: {
            result = sensitivity_64x;
        } break;
        case 32: {
            result = sensitivity_32x;
        } break;
        case 16: {
            result = sensitivity_16x;
        } break;
        case 8: {
            result = sensitivity_8x;
        } break;
        case 4: {
            result = sensitivity_4x;
        } break;
        case 2: {
            result = sensitivity_2x;
        } break;
        case 1: {
            result = sensitivity_1x;
        } break;
    }
    return result;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 5.4 Sensor Input Delta Count Registers

uint8_t  slight_CAP1188_TWI::sensor_input_delta_count_get(uint8_t sensor) {
    if (sensor > 8) {
        sensor = 8;
    }
    if (sensor < 1) {
        sensor = 1;
    }
    sensor = sensor -1;
    // read register
    uint8_t reg = read_register(REG_Sensor_Input_1_Delta_Count + (sensor));
    return reg;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 5.7 Sensor Input Enable Registers

void slight_CAP1188_TWI::sensor_input_enable_set_sensor(
    uint8_t sensor,
    bool enable
) {
    if (sensor > 8) {
        sensor = 8;
    }
    if (sensor < 1) {
        sensor = 1;
    }
    sensor = sensor -1;
    // read register
    uint8_t reg = read_register(REG_Sensor_Input_Enable);
    // create mask
    uint8_t mask = 1 << sensor;
    // clear bit
    reg = reg & (~mask);
    // set bit
    reg = reg | (enable << sensor);
    // write register
    write_register(REG_Sensor_Input_Enable, reg);
}

void slight_CAP1188_TWI::sensor_input_enable_set(uint8_t sensor_enable) {
    // write register
    write_register(REG_Sensor_Input_Enable, sensor_enable);
}

uint8_t slight_CAP1188_TWI::sensor_input_enable_get() {
    // read register
    uint8_t reg = read_register(REG_Sensor_Input_Enable);
    return reg;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 5.8 Sensor Input Configuration Register
// 5.9 Sensor Input Configuration 2 Register

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 5.10 Averaging and Sampling Configuration Register

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 5.11 Calibration Activate Register

void slight_CAP1188_TWI::calibration_activate_sensor(
    uint8_t sensor
) {
    bool enable = 1;
    if (sensor > 8) {
        sensor = 8;
    }
    if (sensor < 1) {
        sensor = 1;
    }
    sensor = sensor -1;
    // read register
    uint8_t reg = read_register(REG_Calibration_Activate);
    // create mask
    // uint8_t mask = 1 << sensor;
    // clear bit
    // reg = reg & (~mask);
    // set bit
    reg = reg | (enable << sensor);
    // write register
    write_register(REG_Calibration_Activate, reg);
}

void slight_CAP1188_TWI::calibration_activate(uint8_t sensor_activate) {
    // write register
    write_register(REG_Calibration_Activate, sensor_activate);
}

uint8_t slight_CAP1188_TWI::calibration_activate_get() {
    // read register
    uint8_t reg = read_register(REG_Calibration_Activate);
    return reg;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 5.12 Interrupt Enable Register
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 5.13 Repeat Rate Enable Register

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 5.14 Multiple Touch Configuration Register

void slight_CAP1188_TWI::multiple_touch_blocking_enable_set(
    bool enable
) {
    // read register
    uint8_t reg = read_register(REG_Multiple_Touch_Configuration);
    // clear bit
    reg = reg & (~multiple_touch_blocking_enable_mask);
    // set bit
    reg = reg | enable;
    // write register
    write_register(REG_Multiple_Touch_Configuration, reg);
}

bool slight_CAP1188_TWI::multiple_touch_blocking_enable_get() {
    // read register
    uint8_t reg = read_register(REG_Multiple_Touch_Configuration);
    // isolate bits
    reg = reg & multiple_touch_blocking_enable_mask;
    // convert to boolean
    bool result = false;
    if (reg > 0) {
        result = true;
    }
    return result;
}

slight_CAP1188_TWI::multiple_touch_simultaneous_t slight_CAP1188_TWI::multiple_touch_simultaneous_convert(
    uint8_t value
) {
    multiple_touch_simultaneous_t result = simultaneous_1;
    // switch (value) {
    //     case 1: {
    //         result = simultaneous_1;
    //     } break;
    //     case 2: {
    //         result = simultaneous_2;
    //     } break;
    //     case 3: {
    //         result = simultaneous_3;
    //     } break;
    //     case 4: {
    //         result = simultaneous_4;
    //     } break;
    // }
    if (value < 1) {
        value = 1;
    }
    if (value > 4) {
        value = 4;
    }
    value = value -1;
    result = (multiple_touch_simultaneous_t)(value << 2);
    return result;
}

void slight_CAP1188_TWI::multiple_touch_simultaneous_set(
    multiple_touch_simultaneous_t value
) {
    // read register
    uint8_t reg = read_register(REG_Multiple_Touch_Configuration);
    // clear bits
    reg = reg & (~multiple_touch_simultaneous_mask);
    // set bits
    reg = reg | value;
    // write register
    write_register(REG_Multiple_Touch_Configuration, reg);
}

slight_CAP1188_TWI::multiple_touch_simultaneous_t slight_CAP1188_TWI::multiple_touch_simultaneous_get() {
    uint8_t reg = read_register(REG_Multiple_Touch_Configuration);
    // isolate bits
    uint8_t value = reg & multiple_touch_simultaneous_mask;
    // shift bits right
    value = (value >> 2);
    // add offset
    value = value + 1;
    multiple_touch_simultaneous_t result;
    // cast
    result = (multiple_touch_simultaneous_t)value;
    return result;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 5.15 Multiple Touch Pattern Configuration Register
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 5.16 Multiple Touch Pattern Register

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 5.17 Recalibration Configuration Register

//
// void slight_CAP1188_TWI::recalibration_configuration_set(
//     uint8_t sensor_activate
// ) {
//     // write register
//     write_register(REG_Recalibration_Configuration, sensor_activate);
// }
//
// uint8_t slight_CAP1188_TWI::recalibration_configuration_get() {
//     // read register
//     uint8_t reg = read_register(REG_Recalibration_Configuration);
//     return reg;
// }

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 5.18 Sensor Input Threshold Registers

void slight_CAP1188_TWI::sensor_input_threshold_set(
    uint8_t sensor,
    uint8_t value
) {
    if (sensor > 8) {
        sensor = 8;
    }
    if (sensor < 1) {
        sensor = 1;
    }
    sensor = sensor -1;
    if (value > B01111111) {
        value = 127;
    }
    write_register(REG_Sensor_Input_1_Threshold + (sensor), value);
}

uint8_t slight_CAP1188_TWI::sensor_input_threshold_get(uint8_t sensor) {
    // read register
    uint8_t reg = read_register(REG_Sensor_Input_1_Threshold + (sensor));
    return reg;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 5.19 Sensor Input Noise Threshold Register

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 5.20 Standby Channel Register
// 5.21 Standby Configuration Register
// 5.22 Standby Sensitivity Register
// 5.23 Standby Threshold Register

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 5.24 Sensor Input Base Count Registers

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 5.25 LED Output Type Register
void slight_CAP1188_TWI::led_output_type_set(uint8_t type_config) {
    write_register(REG_LED_Output_Type, type_config);
}

void slight_CAP1188_TWI::led_output_type_set_led(
    uint8_t led,
    bool type_config
) {
    if (led > 8) {
        led = 8;
    }
    if (led < 1) {
        led = 1;
    }
    led = led -1;
    // read register
    uint8_t reg = read_register(REG_LED_Output_Type);
    // create mask
    uint8_t mask = 1 << led;
    // clear bit
    reg = reg & (~mask);
    // set bit
    reg = reg | (type_config << led);
    // write register
    write_register(REG_LED_Output_Type, reg);
}

uint8_t slight_CAP1188_TWI::led_output_type_get() {
    // read register
    uint8_t reg = read_register(REG_LED_Output_Type);
    return reg;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 5.26 Sensor Input LED Linking Register

void slight_CAP1188_TWI::sensor_input_led_linking_set(uint8_t linking) {
    write_register(REG_Sensor_Input_LED_Linking, linking);
}

void slight_CAP1188_TWI::sensor_input_led_linking_set_led(
    uint8_t led,
    bool link
) {
    if (led > 8) {
        led = 8;
    }
    if (led < 1) {
        led = 1;
    }
    led = led -1;
    // read register
    uint8_t reg = read_register(REG_Sensor_Input_LED_Linking);
    // create mask
    uint8_t mask = 1 << led;
    // clear bit
    reg = reg & (~mask);
    // set bit
    reg = reg | (link << led);
    // write register
    write_register(REG_Sensor_Input_LED_Linking, reg);
}

uint8_t slight_CAP1188_TWI::sensor_input_led_linking_get() {
    // read register
    uint8_t reg = read_register(REG_Sensor_Input_LED_Linking);
    return reg;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 5.27 LED Polarity Register

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 5.28 LED Output Control Register

void slight_CAP1188_TWI::led_output_control_set(uint8_t enable) {
    write_register(REG_LED_Output_Control, enable);
}

void slight_CAP1188_TWI::led_output_control_set_led(
    uint8_t led,
    bool enable
) {
    if (led > 8) {
        led = 8;
    }
    if (led < 1) {
        led = 1;
    }
    led = led -1;
    // read register
    uint8_t reg = read_register(REG_LED_Output_Control);
    // create mask
    uint8_t mask = 1 << led;
    // clear bit
    reg = reg & (~mask);
    // set bit
    reg = reg | (enable << led);
    // write register
    write_register(REG_LED_Output_Control, reg);
}

uint8_t slight_CAP1188_TWI::led_output_control_get() {
    // read register
    uint8_t reg = read_register(REG_LED_Output_Control);
    return reg;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 6.31 LED Behavior Registers
void slight_CAP1188_TWI::led_behavior_set(
    uint8_t led,
    led_behavior_t behavior
) {
    if (led > 8) {
        led = 8;
    }
    if (led < 1) {
        led = 1;
    }

    // find right register to use:
    register_name_t reg_behavior_n = REG_LED_Behavior_1;
    if (led > 4) {
        reg_behavior_n = REG_LED_Behavior_2;
        // recaluclate led position in register
        led = led - 4;
    }

    // calculat how fare we have to move the bits
    uint8_t move_count = ((led-1) * 2);

    // read register
    uint8_t reg = read_register(reg_behavior_n);
    // create mask
    uint8_t mask = led_behavior_mask << move_count;
    // clear bits
    reg = reg & (~mask);
    // set bits
    reg = reg | (behavior << move_count);
    // Serial.print(F("reg: "));
    // Serial.print(reg, BIN);
    // Serial.println();
    // write register
    write_register(reg_behavior_n, reg);
}

slight_CAP1188_TWI::led_behavior_t slight_CAP1188_TWI::led_behavior_get(
    uint8_t led
) {
    if (led > 8) {
        led = 8;
    }
    if (led < 1) {
        led = 1;
    }

    // find right register to use:
    register_name_t reg_behavior_n = REG_LED_Behavior_1;
    if (led > 4) {
        reg_behavior_n = REG_LED_Behavior_2;
        // recaluclate led position in register
        led = led - 4;
    }

    // calculat how fare we have to move the bits
    uint8_t move_count = ((led-1) * 2);

    // read register
    uint8_t reg = read_register(reg_behavior_n);
    // create mask
    uint8_t mask = led_behavior_mask << move_count;
    // isolate
    reg = reg & mask;
    // move to neutral position
    reg = (reg >> move_count);
    // create temp result variable
    led_behavior_t result = behavior_direct;
    // cast to result type
    result = (led_behavior_t)reg;
    return result;
}

void slight_CAP1188_TWI::led_behavior_print(
    Print &out,
    led_behavior_t behavior
) {
    switch (behavior) {
        case behavior_direct: {
            out.print(F("direct"));
        } break;
        case behavior_pulse1: {
            out.print(F("pulse 1"));
        } break;
        case behavior_pulse2: {
            out.print(F("pulse 2"));
        } break;
        case behavior_breathe: {
            out.print(F("breathe"));
        } break;
    }
}

void slight_CAP1188_TWI::led_behavior_print_all(
    Print &out
) {
    out.println(F("LED Behavior:"));
    for (size_t led = 1; led <= 8; led++) {
        led_behavior_t behavior;
        behavior = led_behavior_get(led);
        out.print(F("\t"));
        out.print(led);
        out.print(F(" : "));
        led_behavior_print(out, behavior);
        out.println();
    }
}


// there are more...


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 5.39 Sensor Input Calibration Registers
uint16_t slight_CAP1188_TWI::sensor_input_calibration_value_get(
  uint8_t sensor
) {
    if (sensor > 8) {
        sensor = 8;
    }
    if (sensor < 1) {
        sensor = 1;
    }

    uint8_t reg_raw = read_register(
        REG_Sensor_Input_1_Calibration  + (sensor-1)
    );

    uint16_t result = reg_raw << 2;
    uint8_t reg_lsb = 0;
    uint8_t lsb_offset = 0;
    if (sensor < 5) {
        reg_lsb = read_register(REG_Sensor_Input_Calibration_LSB_1);
        lsb_offset = sensor * 2;
    } else {
        reg_lsb = read_register(REG_Sensor_Input_Calibration_LSB_2);
        lsb_offset = (sensor-4) * 2;
    }

    uint8_t lsb_sensor = 0;
    // isolate
    lsb_sensor = reg_lsb & (1 << lsb_offset);

    result = result & lsb_sensor;


    return result;
}

// there are more...



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// basic read write operations
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void slight_CAP1188_TWI::write_register(uint8_t reg_name, uint8_t value) {
    if (ready) {
        twi_state = TWI_STATE_undefined;
        // set register
        Wire.beginTransmission(twi_address);
        Wire.write(reg_name);
        Wire.write(value);
        twi_state = (twi_state_t)Wire.endTransmission();
        if (twi_state == TWI_STATE_success) {
            // all fine.
        } else {
            // print_transmission_state(Serial, twi_state);
        }
    }
}

void slight_CAP1188_TWI::write_register(
    register_name_t reg_name,
    uint8_t value
) {
    write_register((uint8_t)reg_name, value);
}

uint8_t slight_CAP1188_TWI::read_register(uint8_t reg_name) {
    uint8_t result_value = 0;
    if (ready) {
        twi_state = TWI_STATE_undefined;
        // set register
        Wire.beginTransmission(twi_address);
        Wire.write(reg_name);
        twi_state = (twi_state_t)Wire.endTransmission();
        if (twi_state == TWI_STATE_success) {
            // read data
            Wire.requestFrom(twi_address, (uint8_t)1);
            result_value = Wire.read();
        } else {
            // print_transmission_state(Serial, twi_state);
        }
    }
    return result_value;
}

uint8_t slight_CAP1188_TWI::read_register(register_name_t reg_name) {
    return read_register((uint8_t)reg_name);
}


void slight_CAP1188_TWI::twi_state_print(Print &out) {
    twi_state_print(out, twi_state);
}

void slight_CAP1188_TWI::twi_state_print(Print &out, twi_state_t state) {
    switch (state) {
        case TWI_STATE_success: {
            out.print(F("success"));
        } break;
        case TWI_STATE_data_to_long: {
            out.print(F("data too long to fit in transmit buffer"));
        } break;
        case TWI_STATE_rec_NACK_on_address: {
            out.print(F("received NACK on transmit of address"));
        } break;
        case TWI_STATE_rec_NACK_on_data: {
            out.print(F("received NACK on transmit of data"));
        } break;
        case TWI_STATE_other_error: {
            out.print(F("other error"));
        } break;
        default: {
            out.print(F("??"));
        }
    }
}
