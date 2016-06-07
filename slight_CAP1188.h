/******************************************************************************

    arduino library for CAP1188 touch sensor.
    tested with adafruit CAP1188 breakoutboard
        (https://www.adafruit.com/products/1602)

    this library currently only supports TWI (= I2C) interface.

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



#ifndef SLIGHT_CAP1188_TWI_H_
#define SLIGHT_CAP1188_TWI_H_

/** Includes Core Arduino functionality **/
#if ARDUINO
    #if ARDUINO < 100
        #include <WProgram.h>
    #else
        #include <Arduino.h>
    #endif
#endif


class slight_CAP1188_TWI {
public:

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // constructor

    // slight_CAP1188_TWI(uint8_t reset_pin);
    slight_CAP1188_TWI(uint8_t reset_pin, uint8_t twi_address);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // attributes

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // public types

    static const uint8_t NO_PIN = 255;

    enum twi_state_t {
        TWI_STATE_success = 0,
        TWI_STATE_data_to_long = 1,
        TWI_STATE_rec_NACK_on_address = 2,
        TWI_STATE_rec_NACK_on_data = 3,
        TWI_STATE_other_error = 4,
        TWI_STATE_undefined = 99,
    };

    // registers
    enum register_name_t {
        // read / write
        REG_Main_Control = 0x00,
        // read only
        REG_General_Status = 0x02,
        REG_Sensor_Input_Status = 0x03,
        REG_LED_Status = 0x04,
        REG_Noise_Flag_Status = 0x0A,
        REG_Sensor_Input_1_Delta_Count = 0x10,
        REG_Sensor_Input_2_Delta_Count = 0x11,
        REG_Sensor_Input_3_Delta_Count = 0x12,
        REG_Sensor_Input_4_Delta_Count = 0x13,
        REG_Sensor_Input_5_Delta_Count = 0x14,
        REG_Sensor_Input_6_Delta_Count = 0x15,
        REG_Sensor_Input_7_Delta_Count = 0x16,
        REG_Sensor_Input_8_Delta_Count = 0x17,
        // read / write
        REG_Sensitivity_Control = 0x1F,
        REG_Sensor_Input_Enable = 0x21,
        REG_Sensor_Input_Configuration = 0x22,
        REG_Sensor_Input_Configuration_2 = 0x23,
        REG_Averaging_and_Sampling_Config = 0x24,
        REG_Calibration_Activate = 0x26,
        REG_Interrupt_Enable = 0x27,
        REG_Repeat_Rate_Enable = 0x28,
        REG_Multiple_Touch_Configuration = 0x2A,
        REG_Multiple_Touch_Pattern_Configuration = 0x2B,
        REG_Multiple_Touch_Pattern = 0x2D,
        REG_Recalibration_Configuration = 0x2F,
        REG_Sensor_Input_1_Threshold = 0x30,
        REG_Sensor_Input_2_Threshold = 0x31,
        REG_Sensor_Input_3_Threshold = 0x32,
        REG_Sensor_Input_4_Threshold = 0x33,
        REG_Sensor_Input_5_Threshold = 0x34,
        REG_Sensor_Input_6_Threshold = 0x35,
        REG_Sensor_Input_7_Threshold = 0x36,
        REG_Sensor_Input_8_Threshold = 0x37,
        REG_Sensor_Input_Noise_Threshold = 0x38,
        // read / write
        REG_Standby_Channel = 0x40,
        REG_Standby_Configuration = 0x41,
        REG_Standby_Sensitivity = 0x42,
        REG_Standby_Threshold = 0x43,
        REG_Configuration_2 = 0x44,
        // read only
        REG_Sensor_Input_1_Base_Count = 0x50,
        REG_Sensor_Input_2_Base_Count = 0x51,
        REG_Sensor_Input_3_Base_Count = 0x52,
        REG_Sensor_Input_4_Base_Count = 0x53,
        REG_Sensor_Input_5_Base_Count = 0x54,
        REG_Sensor_Input_6_Base_Count = 0x55,
        REG_Sensor_Input_7_Base_Count = 0x56,
        REG_Sensor_Input_8_Base_Count = 0x57,
        // read / write
        REG_LED_Output_Type = 0x71,
        REG_Sensor_Input_LED_Linking = 0x72,
        REG_LED_Polarity = 0x73,
        REG_LED_Output_Control = 0x74,
        REG_LED_Linked_Transition_Control = 0x77,
        REG_LED_Mirror_Control = 0x79,
        REG_LED_Behavior_1 = 0x81,
        REG_LED_Behavior_2 = 0x82,
        REG_LED_Pulse_1_Period = 0x84,
        REG_LED_Pulse_2_Period = 0x85,
        REG_LED_Breathe_Period = 0x86,
        REG_LED_Config = 0x88,
        REG_LED_Pulse_1_Duty_Cycle = 0x90,
        REG_LED_Pulse_2_Duty_Cycle = 0x91,
        REG_LED_Pulse_Breathe_Duty_Cycle = 0x92,
        REG_LED_Pulse_Direct_Duty_Cycle = 0x93,
        REG_LED_Pulse_Direct_Ramp_Rates = 0x94,
        REG_LED_Off_Delay = 0x95,
        // read only
        REG_Sensor_Input_1_Calibration = 0xB1,
        REG_Sensor_Input_2_Calibration = 0xB2,
        REG_Sensor_Input_3_Calibration = 0xB3,
        REG_Sensor_Input_4_Calibration = 0xB4,
        REG_Sensor_Input_5_Calibration = 0xB5,
        REG_Sensor_Input_6_Calibration = 0xB6,
        REG_Sensor_Input_7_Calibration = 0xB7,
        REG_Sensor_Input_8_Calibration = 0xB8,
        REG_Sensor_Input_Calibration_LSB_1 = 0xB9,
        REG_Sensor_Input_Calibration_LSB_2 = 0xBA,
        REG_Product_ID = 0xFD,
        REG_Manufacturer_ID = 0xFE,
        REG_Revision_ID = 0xFF,
    };

    // callback
    typedef void (* callback_t) (slight_CAP1188_TWI *instance);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // public functions
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    bool begin();
    void update();

    void sensor_HW_reset();
    void sensor_default_configuration();

    void update_interval_set_autofit();
    void update_interval_set(uint32_t interval);
    uint32_t update_interval_get();

    void touch_event_set_callback(callback_t);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // register helper functions

    // 5.1 Main Control Register

    // Interrupt Bit
    static const uint8_t intterupt_mask = B00000001;

    void interrupt_clear();
    bool interrupt_get();

    // Gain Settings
    enum gain_setting_t {
        gain_1 = B00000000,
        gain_2 = B01000000,
        gain_4 = B10000000,
        gain_8 = B11000000,
    };
    static const uint8_t gain_mask = B11000000;

    void gain_set(gain_setting_t value);
    gain_setting_t gain_get();
    void gain_print(Print& out);
    static void gain_print(Print &out, gain_setting_t value);

    // 5.2.1 General Status
    uint8_t general_status_get();
    // 5.2.2 Sensor Input Status
    uint8_t sensor_input_status_get_raw();
    uint8_t sensor_input_status_get();
    bool sensor_input_status_get(uint8_t sensor);
    // 5.2.3 LED Status
    uint8_t led_status_get();

    // 5.3 noise flags
    uint8_t noise_flags_get();

    // 5.4 Sensor Input Delta Count Registers
    int8_t sensor_input_delta_count_get(uint8_t sensor);

    // 5.5 Sensitivity Control Register
    enum sensitivity_t {
        sensitivity_128x = B00000000,   // most sensitiv
        sensitivity_64x =  B00010000,
        sensitivity_32x =  B00100000,   // default
        sensitivity_16x =  B00110000,
        sensitivity_8x =   B01000000,
        sensitivity_4x =   B01010000,
        sensitivity_2x =   B01100000,
        sensitivity_1x =   B01110000,   // least sensitiv
    };
    static const uint8_t sensitivity_mask = B01110000;

    void sensitivity_set(sensitivity_t value);
    sensitivity_t sensitivity_get();
    sensitivity_t sensitivity_convert(uint8_t value);
    static void sensitivity_print(Print &out, sensitivity_t value);
    void sensitivity_print(Print &out);

    // 5.6.1 Configuration
    // 5.6.2 Configuration 2

    // 5.7 Sensor Input Enable Registers
    void sensor_input_enable_set_sensor(uint8_t sensor, bool enable);
    void sensor_input_enable_set(uint8_t sensor_enable);
    uint8_t sensor_input_enable_get();

    // 5.8 Sensor Input Configuration Register
    // 5.9 Sensor Input Configuration 2 Register

    // 5.10 Averaging and Sampling Configuration Register

    // 5.11 Calibration Activate Register
    void calibration_activate_sensor(uint8_t sensor);
    void calibration_activate(uint8_t sensor_enable);
    uint8_t calibration_activate_get();

    // 5.12 Interrupt Enable Register
    // 5.13 Repeat Rate Enable Register

    // 5.14 Multiple Touch Configuration Register
    enum multiple_touch_simultaneous_t {
        simultaneous_1 = B00000000,
        simultaneous_2 = B00000100,
        simultaneous_3 = B00001000,
        simultaneous_4 = B00001100,
    };
    static const uint8_t multiple_touch_simultaneous_mask = B00001100;
    static const uint8_t multiple_touch_blocking_enable_mask = B10000000;

    void multiple_touch_blocking_enable_set(bool enable);
    bool multiple_touch_blocking_enable_get();

    multiple_touch_simultaneous_t multiple_touch_simultaneous_convert(uint8_t value);
    void multiple_touch_simultaneous_set(uint8_t value);
    void multiple_touch_simultaneous_set(multiple_touch_simultaneous_t value);
    multiple_touch_simultaneous_t multiple_touch_simultaneous_get();

    // 5.15 Multiple Touch Pattern Configuration Register
    // 5.16 Multiple Touch Pattern Register

    // 5.17 Recalibration Configuration Register
    // void recalibration_configuration_set(uint8_t sensor_enable);
    // uint8_t recalibration_configuration_get();

    // 5.18 Sensor Input Threshold Registers
    void sensor_input_threshold_set(uint8_t sensor, uint8_t value);
    uint8_t sensor_input_threshold_get(uint8_t sensor);

    // 5.19 Sensor Input Noise Threshold Register
    enum sensor_input_noise_threshold_t {
        threshold_25 =   B00000000,
        threshold_37_5 = B00000001, // default
        threshold_50 =   B00000010,
        threshold_62_5 = B00000011,
    };
    static const uint8_t sensor_input_noise_threshold_mask = B00000011;

    static sensor_input_noise_threshold_t sensor_input_noise_threshold_convert(
        uint8_t value
    );
    static void sensor_input_noise_threshold_print(
        Print &out,
        sensor_input_noise_threshold_t value
    );

    void sensor_input_noise_threshold_print(
        Print &out
    );

    void sensor_input_noise_threshold_set(
        sensor_input_noise_threshold_t value
    );
    void sensor_input_noise_threshold_set(
        uint8_t value
    );

    sensor_input_noise_threshold_t sensor_input_noise_threshold_get();

    // 5.20 Standby Channel Register
    // 5.21 Standby Configuration Register
    // 5.22 Standby Sensitivity Register
    // 5.23 Standby Threshold Register

    // 5.24 Sensor Input Base Count Registers
    uint8_t sensor_input_base_count_get(uint8_t sensor);

    // 5.25 LED Output Type Register
    // '0' = open drain with external pullup (only can drive low & highZ)
    // '1' = push-pull output (drives high & low)
    void led_output_type_set(uint8_t type_config);
    void led_output_type_set_led(uint8_t led, bool type_config);
    uint8_t led_output_type_get();

    // 5.26 Sensor Input LED Linking Register
    // '0' = stand alone (default)
    // '1' = linked with sensor input
    void sensor_input_led_linking_set(uint8_t linking);
    void sensor_input_led_linking_set_led(uint8_t led, bool link);
    uint8_t sensor_input_led_linking_get();

    // 5.27 LED Polarity Register

    // 5.28 LED Output Control Register
    // '0' = off
    // '1' = activated
    void led_output_control_set(uint8_t enable);
    void led_output_control_set_led(uint8_t led, bool enable);
    uint8_t led_output_control_get();

    // 5.29 Linked LED Transition Control Register
    // 5.30 LED Mirror Control Register

    // 5.31 LED Behavior Registers
    // 5.31.1 LED Behavior 1
    // 5.31.2 LED Behavior 2
    enum led_behavior_t {
        behavior_direct =  B00000000,
        behavior_pulse1 =  B00000001,
        behavior_pulse2 =  B00000010,
        behavior_breathe = B00000011,
    };
    static const uint8_t led_behavior_mask = B00000011;

    void led_behavior_set(uint8_t led, led_behavior_t behavior);
    led_behavior_t led_behavior_get(uint8_t led);
    static void led_behavior_print(Print &out, led_behavior_t behavior);
    void led_behavior_print_all(Print &out);

    // 5.32 ...
    // led things...

    // 5.39 Sensor Input Calibration Registers
    uint16_t sensor_input_calibration_value_get(uint8_t sensor);

    // 5.40 Product ID Register
    uint8_t product_ID_get();

    // 5.41 Manufacturer ID Register
    uint8_t manufacturer_ID_get();

    // 5.42 Revision Register
    uint8_t revision_get();

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // basic read write operations
    void write_register(register_name_t, uint8_t);
    uint8_t read_register(register_name_t);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // twi state helper
    twi_state_t twi_state_get();
    void twi_state_print(Print &out);
    static void twi_state_print(Print &out, twi_state_t state);

private:

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // private functions
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    void sensor_input_status_update();

    void touch_event_callback();

    // basic read write operations
    void write_register(uint8_t, uint8_t);
    uint8_t read_register(uint8_t);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // attributes

    bool ready;
    twi_state_t twi_state;

    const uint8_t twi_address;
    const uint8_t reset_pin;

    uint8_t sensor_input_status;
    uint8_t sensor_input_status_old;

    uint32_t timestamp_lastread;
    uint32_t update_interval;

    callback_t callback_touch_event;

};  // class slight_CAP1188_TWI

#endif  // SLIGHT_CAP1188_TWI_H_
