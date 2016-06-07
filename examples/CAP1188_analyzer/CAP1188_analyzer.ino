/******************************************************************************

    CAP1188_analyzer.ino
        sketch for analyzing and tweaking CAP1188 settings.
        debugout on usbserial interface: 115200baud

    hardware:
        Board:
            Arduino compatible (with serial port)
            (tested with Leonardo compatible)
            LED on pin 13
            CAP1188
              (tested with CAP1188 sensor breakout form adafruit)
                TWI (SDA & SCL)
                reset on SCK


    libraries used:
        ~ slight_CAP1188_TWI
        ~ slight_DebugMenu
            written by stefan krueger (s-light),
                github@s-light.eu, http://s-light.eu, https://github.com/s-light/
            license: MIT

    written by stefan krueger (s-light),
        github@s-light.eu, http://s-light.eu, https://github.com/s-light/

    changelog / history
        02.06.2016 16:10 created (based on CAP1188_TWI_wMenu.ino)
        02.06.2016 16:10

    TO DO:
        ~ enjoy your life ;-)


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

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Includes
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// use "file.h" for files in same directory as .ino
// #include "file.h"
// use <file.h> for files in library directory
// #include <file.h>

#include <slight_DebugMenu.h>

#include <Wire.h>
#include <slight_CAP1188.h>

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Info
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sketchinfo_print(Print &out) {
    out.println();
    //             "|~~~~~~~~~|~~~~~~~~~|~~~..~~~|~~~~~~~~~|~~~~~~~~~|"
    out.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
    out.println(F("|                       ^ ^                      |"));
    out.println(F("|                      (0,0)                     |"));
    out.println(F("|                      ( _ )                     |"));
    out.println(F("|                       \" \"                      |"));
    out.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
    out.println(F("| CAP1188_analyzer.ino"));
    out.println(F("|   sketch for analyzing and tweaking"));
    out.println(F("|   CAP1188 settings."));
    out.println(F("|   CAP1188 sensor breakout form adafruit."));
    out.println(F("|"));
    out.println(F("| This Sketch has a debug-menu:"));
    out.println(F("| send '?'+Return for help"));
    out.println(F("|"));
    out.println(F("| dream on & have fun :-)"));
    out.println(F("|"));
    out.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
    out.println(F("|"));
    //out.println(F("| Version: Nov 11 2013  20:35:04"));
    out.print(F("| version: "));
    out.print(F(__DATE__));
    out.print(F("  "));
    out.print(F(__TIME__));
    out.println();
    out.println(F("|"));
    out.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
    out.println();

    //out.println(__DATE__); Nov 11 2013
    //out.println(__TIME__); 20:35:04
}


// Serial.print to Flash: Notepad++ Replace RegEx
//     Find what:        Serial.print(.*)\("(.*)"\);
//     Replace with:    Serial.print\1\(F\("\2"\)\);



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// definitions (global)
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Debug Output

boolean infoled_state = 0;
const byte infoled_pin = 13; //D9

unsigned long debugOut_LiveSign_TimeStamp_LastAction = 0;
const uint16_t debugOut_LiveSign_UpdateInterval = 1000; //ms

boolean debugOut_LiveSign_Serial_Enabled = 0;
boolean debugOut_LiveSign_LED_Enabled = 1;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Menu

// slight_DebugMenu(Stream &in_ref, Print &out_ref, uint8_t input_length_new);
slight_DebugMenu myDebugMenu(Serial, Serial, 15);

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// CAP1188

unsigned long debugOut_SensorInfo_TimeStamp_LastAction = 0;
uint16_t debugOut_SensorInfo_UpdateInterval = 1000; //ms

boolean debugOut_SensorInfo_Serial_Enabled = 0;
boolean debugOut_SensorInfo_Terminal_Special = 0;

slight_CAP1188_TWI myTouchSensor(
    SCK,  // HW reset pin
    0x29  // TWI address
);


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// functions
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// debug things

// freeRam found at
// http://forum.arduino.cc/index.php?topic=183790.msg1362282#msg1362282
// posted by mrburnette
int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Menu System

void debugOut_SensorInfo_Terminal_Type(Print &out) {
    if (debugOut_SensorInfo_Terminal_Special) {
        out.print(F("special"));
    } else {
        out.print(F("normal"));
    }
}

// Main Menu
void handleMenu_Main(slight_DebugMenu *pInstance) {
    Print &out = pInstance->get_stream_out_ref();
    char *command = pInstance->get_command_current_pointer();
    // out.print("command: '");
    // out.print(command);
    // out.println("'");
    switch (command[0]) {
        case 'h':
        case 'H':
        case '?': {
            // help
            out.println(F("____________________________________________________________"));
            out.println();
            out.println(F("Help for Commands:"));
            out.println();
            out.println(F("\t '?': this help"));
            out.println(F("\t 'i': sketch info"));
            out.println(F("\t 'Y': toggle DebugOut livesign print"));
            // out.println(F("\t 'Y': toggle DebugOut livesign LED"));
            // out.println(F("\t 'x': tests"));
            out.println(F("\t 'x': SensorInfo toggle"));
            out.print(F("\t 'x': SensorInfo set terminal type (0=normal 1=special) 'y1'; "));
            debugOut_SensorInfo_Terminal_Type(out);
            out.println();
            out.print(F("\t 'X': SensorInfo set update interval 'X1000'; "));
            out.print(debugOut_SensorInfo_UpdateInterval);
            out.println(F("ms"));
            // ------------------------------------------
            out.println();
            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // 5.1 Main Control Registers - Gain
            out.print(F("\t 'g': gain set [1..8] 's8'; "));
            myTouchSensor.gain_print(out);
            out.println();
            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // 5.5 Sensitivity Control Register
            out.print(F("\t 's': sensitivity set [1, 2, .., 64, 128] 's128'; "));
            myTouchSensor.sensitivity_print(out);
            out.println();
            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // 5.14 Multiple Touch Configuration Register
            out.print(F("\t 'b': multiple touch blocked enable set 'b1'; "));
            out.print(
                myTouchSensor.multiple_touch_blocking_enable_get()
            );
            out.println();
            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // 5.18 Sensor Input Threshold Registers
            out.print(F("\t 't': threshold set (0..127) 't1:127'; "));
                out.print(F(" 1:"));
                out.print(myTouchSensor.sensor_input_threshold_get(1));
                out.print(F(" 2:"));
                out.print(myTouchSensor.sensor_input_threshold_get(2));
            out.println();
            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // 5.19 Sensor Input Noise Threshold Register
            out.print(F("\t 'n': noise threshold set (0..255) 't37'; "));
                myTouchSensor.sensor_input_noise_threshold_print(out);
                out.print(F("%"));
            out.println();
            // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // 5.24 Sensor Input Base Count Registers
            out.print(F("\t 'a': calibration activate sensor 'a1'; "));
            slight_DebugMenu::print_Binary_8(
                out,
                myTouchSensor.calibration_activate_get()
            );
            out.println();
            out.print(F("\t 'e': sensor input enable set sensor'e0:1'; "));
            // out.print(F("\t 'e': sensor input enable set'e255' (3=S1+S2 en); "));
            out.print(F("\t sensor input enabled: "));
            slight_DebugMenu::print_Binary_8(
                out,
                myTouchSensor.sensor_input_enable_get()
            );
            out.println();
            // out.println(F("\t 'E': sensor input enable get"));
            out.println();
            out.println(F("\t 'r': soft reset of sensor 'r1'"));
            out.println(F("\t 'R': HW sensor reset "));
            // out.println();
            // out.println(F("\t 'l': setup leds special"));
            // out.println(F("\t 'L': print leds special"));
            // out.println(F("\t 'a': activate leds special"));
            // out.println(F("\t 'A': deactivate leds special"));
            // out.println();
            out.println(F("____________________________________________________________"));
        } break;
        case 'i': {
            sketchinfo_print(out);
        } break;
        case 'y': {
            out.println(F("\t toggle DebugOut livesign Serial:"));
            debugOut_LiveSign_Serial_Enabled = !debugOut_LiveSign_Serial_Enabled;
            out.print(F("\t debugOut_LiveSign_Serial_Enabled:"));
            out.println(debugOut_LiveSign_Serial_Enabled);
        } break;
        // case 'Y': {
        //     out.println(F("\t toggle DebugOut livesign LED:"));
        //     debugOut_LiveSign_LED_Enabled = !debugOut_LiveSign_LED_Enabled;
        //     out.print(F("\t debugOut_LiveSign_LED_Enabled:"));
        //     out.println(debugOut_LiveSign_LED_Enabled);
        // } break;
        // case 'x': {
        //     // get state
        //     out.println(F("__________"));
        //     out.println(F("Tests:"));
        //
        //     out.println(F("nothing to do."));
        //
        //     // uint16_t wTest = 65535;
        //     uint16_t wTest = atoi(&command[1]);
        //     out.print(F("wTest: "));
        //     out.print(wTest);
        //     out.println();
        //
        //     out.print(F("1: "));
        //     out.print((byte)wTest);
        //     out.println();
        //
        //     out.print(F("2: "));
        //     out.print((byte)(wTest>>8));
        //     out.println();
        //
        //     out.println();
        //
        //     // set leds
        //     myTouchSensor.led_output_control_set_led(7, 1);
        //     myTouchSensor.led_output_control_set_led(8, 0);
        //
        //     out.println(F("__________"));
        // } break;
        case 'x': {
            out.print(F("\t SensorInfo Serial '"));
            out.print(&command[1]);
            out.print(F("' "));
            // out.print(command[1], HEX);
            // out.print(F(" "));
            if (command[1] != '\0') {
                // handle set type
                out.print(F("set type: "));
                uint8_t value = atoi(&command[1]);
                out.print(value);
                out.print(F(" "));
                if (value == 1) {
                    debugOut_SensorInfo_Terminal_Special = 1;
                } else {
                    debugOut_SensorInfo_Terminal_Special = 0;
                }
                debugOut_SensorInfo_Terminal_Type(out);
                out.println();
            } else {
                // handle toggle
                out.print(F("toggle: "));
                debugOut_SensorInfo_Serial_Enabled = !debugOut_SensorInfo_Serial_Enabled;
                out.print(F("\t debugOut_SensorInfo_Serial_Enabled:"));
                out.print(debugOut_SensorInfo_Serial_Enabled);
                out.println();
                if (debugOut_SensorInfo_Serial_Enabled) {
                    myTouchSensor_debugOut_print_start(out);
                }
            }
        } break;
        case 'X': {
            out.print(F("\t SensorInfo set update interval: "));
            uint16_t value = atoi(&command[1]);
            out.print(value);
            debugOut_SensorInfo_UpdateInterval = value;
            out.println();
        } break;
        //---------------------------------------------------------------------
        case 'g': {
            out.print(F("\t gain set "));
            // convert part of string to int
            // (up to first char that is not a number)
            uint8_t value = atoi(&command[1]);
            out.print(value);
            out.print(F(" --> "));
            myTouchSensor.gain_set(value);
            myTouchSensor.gain_print(out);
            out.println();
        } break;
        case 's': {
            out.print(F("\t sensitivity set "));
            // convert part of string to int
            // (up to first char that is not a number)
            uint8_t value = atoi(&command[1]);
            out.print(value);
            out.print(F(" --> "));
            myTouchSensor.sensitivity_set(value);
            myTouchSensor.sensitivity_print(out);
            out.println();
        } break;
        case 't': {
            out.print(F("\t threshold set "));
            // t0:127
            uint8_t input = atoi(&command[1]);
            uint8_t value = atoi(&command[3]);
            out.print(input);
            out.print(F(":"));
            out.print(value);
            myTouchSensor.sensor_input_threshold_set(input, value);
            out.println();
        } break;
        case 'n': {
            out.print(F("\t noise threshold set "));
            // t0:127
            uint8_t value = atoi(&command[1]);
            out.print(value);
            myTouchSensor.sensor_input_noise_threshold_set(value);
            out.print(F(" --> "));
            myTouchSensor.sensor_input_noise_threshold_print(out);
            out.print(F("%"));
            out.println();
        } break;
        case 'b': {
            out.print(F("\t multiple touch blocked enable set "));
            uint8_t value = atoi(&command[1]);
            out.print(value);
            myTouchSensor.multiple_touch_blocking_enable_set(value);
            out.println();
        } break;
        case 'a': {
            out.print(F("\t calibration activate sensor set "));
            // a0
            uint8_t input = atoi(&command[1]);
            // uint8_t value = atoi(&command[3]);
            out.print(input);
            // out.print(F(":"));
            // out.print(value);
            myTouchSensor.calibration_activate_sensor(input);
            out.println();
        } break;
        case 'e': {
            out.print(F("\t sensor input enable set sensor "));
            // a0:1
            uint8_t input = atoi(&command[1]);
            uint8_t value = atoi(&command[3]);
            out.print(input);
            out.print(F(":"));
            out.print(value);
            myTouchSensor.sensor_input_enable_set_sensor(input, value);
            out.println();
        } break;
        // case 'e': {
        //     out.print(F("\t sensor input enable set "));
        //     // a0:1
        //     uint8_t value = atoi(&command[1]);
        //     out.print(value);
        //     myTouchSensor.sensor_input_enable_set(value);
        //     out.println();
        // } break;
        // case 'E': {
        //     out.print(F("\t calibration activate sensor get "));
        //     slight_DebugMenu::print_Binary_8(
        //         out,
        //         myTouchSensor.sensor_input_enable_get()
        //     );
        //     out.println();
        // } break;
        case 'r': {
            out.print(F("\t reset without loading defaults."));
            // out.print(F(" -- TODO --"));
            // out.println();
            myTouchSensor.sensor_HW_reset();
            out.print(F(". done"));
            out.println();
            out.print(F("\t write app settings: "));
            // myTouchSensor_init(out);
            myTouchSensor_write_appsettings(out);
            out.print(F("done"));
            out.println();
        } break;
        case 'R': {
            out.print(F("\t HW sensor reset .."));
            myTouchSensor.sensor_HW_reset();
            out.print(F("\t load app defaults: "));
            myTouchSensor.sensor_default_configuration();
            out.print(F(". done"));
            out.println();
            out.print(F("\t write app settings: "));
            myTouchSensor_init(out);
            out.print(F("done"));
            out.println();
        } break;
        //---------------------------------------------------------------------
        default: {
            if(strlen(command) > 0) {
                out.print(F("command '"));
                out.print(command);
                out.println(F("' not recognized. try again."));
            }
            pInstance->get_command_input_pointer()[0] = '?';
                pInstance->set_flag_EOC(true);
        }
    } // end switch

    // end Command Parser
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// CAP1188


void myTouchSensor_init(Print &out) {
    out.println(F("setup CAP1188:")); {
        out.println(F("\t connect to sensor ..."));
        bool ready = myTouchSensor.begin();
        out.print("\t ");
        if (!ready) {
            out.print("not ");
        }
        out.print("found!");
        out.println();

      myTouchSensor_write_appsettings(out);
    }
    out.println(F("\t finished."));
}

void myTouchSensor_write_appsettings(Print &out) {
    out.println(F("setup CAP1188:")); {
        out.println("\t sensor information:");
        out.print("\t   Product ID: ");
        out.print("0x");
        // out.println(
        //     myTouchSensor.read_register(
        //         slight_CAP1188_TWI::REG_Product_ID
        //     ),
        //     HEX
        // );
        out.println(
            myTouchSensor.product_ID_get(),
            HEX
        );
        out.print("\t   Manufacturer ID: ");
        out.print("0x");
        out.println(
            myTouchSensor.manufacturer_ID_get(),
            HEX
        );
        out.print("\t   Revision: ");
        out.print("0x");
        out.println(
            myTouchSensor.revision_get(),
            HEX
        );

        // register callback function.
        // myTouchSensor.touch_event_set_callback(touch_event);

        out.println("\t change config: ");

        out.println("\t   activate only Sensor1&2");
        myTouchSensor.sensor_input_enable_set(0b00000011);
        // setup RECALIBRATION CONFIGURATION REGISTERS
        // default: 0b10001010
        // set BUT_LD_TH to 0. (Each Sensor Input X Threshold register is
        //                      updated individually.)
        myTouchSensor.write_register(
            slight_CAP1188_TWI::REG_Recalibration_Configuration,
            0b00001010
        );
        // setup AVERAGING AND SAMPLING CONFIGURATION REGISTER
        // default: 0b00111001
        // set
        // AVG (B6, B5, B4) to 32 (101) (default: 8 011)
        // SAMP_TIME (B3, B2) to 2.56ms (11) (default: 1.28ms 10).
        // CYCLE_TIME (B1, B0) to 35ms (00) (default: 70ms 01)
        myTouchSensor.write_register(
            slight_CAP1188_TWI::REG_Averaging_and_Sampling_Config,
            0b01010000
        );
        // setup 6.19 Sensor Input Noise Threshold Register
        // default: 0b00000001
        // mask: 0b00000011
        // 00 = 25%
        // 01 = 37.5% (default)
        // 10 = 50%
        // 11 = 62.5%
        // CYCLE_TIME (B1, B0) to 35ms (00) (default: 70ms 01)
        // myTouchSensor.write_register(
        //     slight_CAP1188_TWI::REG_??,
        //     0b00000011
        // );


        out.println("\t   LED config: ");
        // enable linked control
        myTouchSensor.sensor_input_led_linking_set_led(1, 1);
        myTouchSensor.sensor_input_led_linking_set_led(2, 1);
        // enable stand alone host control
        // out.println("\t   * enable host control for led 3 .. 8");
        // myTouchSensor.sensor_input_led_linking_set_led(3, 0);
        // myTouchSensor.sensor_input_led_linking_set_led(4, 0);
        // myTouchSensor.sensor_input_led_linking_set_led(5, 0);
        // myTouchSensor.sensor_input_led_linking_set_led(6, 0);
        // myTouchSensor.sensor_input_led_linking_set_led(7, 0);
        // myTouchSensor.sensor_input_led_linking_set_led(8, 0);
        // // set output as push-pull
        // out.println("\t   * set output type to push-pull for led 3 .. 8");
        // myTouchSensor.led_output_type_set_led(3, 1);
        // myTouchSensor.led_output_type_set_led(4, 1);
        // myTouchSensor.led_output_type_set_led(5, 1);
        // myTouchSensor.led_output_type_set_led(6, 1);
        // myTouchSensor.led_output_type_set_led(7, 1);
        // myTouchSensor.led_output_type_set_led(8, 1);
        // set led behavior to breathe
        // default configs for pulse1 pulse2 and breathe
        // pulse1: pulse 5 times
        // pulse2: pulse endless
        // breathe: breathe (slow pulse) endless
        // out.println("\t   * set behavior to Pulse1 for led 5");
        // myTouchSensor.led_behavior_set(5, slight_CAP1188_TWI::behavior_pulse1);
        // myTouchSensor.led_behavior_set(6, slight_CAP1188_TWI::behavior_breathe);
        // out.println("\t   * set behavior to breathe for led 7, 8");
        // myTouchSensor.led_behavior_set(7, slight_CAP1188_TWI::behavior_pulse1);
        // myTouchSensor.led_behavior_set(8, slight_CAP1188_TWI::behavior_pulse2);

        out.print("\t get sensitivity: ");
        // slight_CAP1188_TWI::sensitivity_print(out, myTouchSensor.sensitivity_get());
        myTouchSensor.sensitivity_print(out);
        out.println();
        // out.print("\t   set sensitivity to ");
        // myTouchSensor.sensitivity_set(slight_CAP1188_TWI::sensitivity_2x);
        // // slight_CAP1188_TWI::sensitivity_print(out, myTouchSensor.sensitivity_get());
        // myTouchSensor.sensitivity_print(out);
        // out.println();

        out.print(F("\t get threshold:"));
        out.println();
        out.print(F("\t   0:"));
        out.print(myTouchSensor.sensor_input_threshold_get(0));
        out.println();
        out.print(F("\t   1:"));
        out.print(myTouchSensor.sensor_input_threshold_get(1));
        out.println();

        // out.print("\t   set sensor threshold to ");
        // myTouchSensor.sensor_input_threshold_set(0, 100);
        // out.print(myTouchSensor.sensor_input_threshold_get(1));
        // out.println();
    }
    out.println(F("\t finished."));
}

void touch_event(slight_CAP1188_TWI *instance) {
    Serial.print(F("touched: "));
    for (size_t i=0; i<8; i++) {
        if (instance->sensor_input_status_get() & (1 << i)) {
            Serial.print("1");
        } else {
            Serial.print("0");
        }
    }
    Serial.println();
}

void myTouchSensor_debugOut_print_start(Print &out) {
    if (debugOut_SensorInfo_Terminal_Special) {
        out.println();
        // remember cursor:

    } else {
        out.println();
        out.println();
        out.println();
        out.println();
        out.println(F("BaseCount, deltaCount, calibration"));
    }
}

void myTouchSensor_debugOut_print(Print &out) {
    if (debugOut_SensorInfo_Terminal_Special) {
        // set cursor back to start:

    }
    // Line 1
    // out.print(millis());
    // out.print(F("ms;"));
    // // out.print(F("  free RAM = "));
    // // out.print(freeRam());
    // // out.print(F(";"));
    // // out.println();
    // // global things:
    // out.print(F(" Noise_Flag: "));
    // slight_DebugMenu::print_Binary_8(
    //     out,
    //     myTouchSensor.noise_flags_get()
    // );
    // out.print(F(" sensitivity: "));
    // myTouchSensor.sensitivity_print(out, myTouchSensor.sensitivity_get());
    // out.println();
    // Line 2
    // out.println(F("\t list: BaseC, deltaC, cali"));
    // Line 3
    // out.print(F("\t\t1: "));
    out.print(
        myTouchSensor.sensor_input_base_count_get(1)
    );
    out.print(F(", "));
    out.print(
        myTouchSensor.sensor_input_delta_count_get(1)
    );
    out.print(F(", "));
    out.print(
        myTouchSensor.sensor_input_calibration_value_get(1)
    );
    out.println();
    // // Line 4
    // out.print(F("\t\t2: "));
    // out.print(
    //     myTouchSensor.sensor_input_base_count_get(2)
    // );
    // out.print(F(", "));
    // out.print(
    //     myTouchSensor.sensor_input_delta_count_get(2)
    // );
    // out.print(F(", "));
    // out.print(
    //     myTouchSensor.sensor_input_calibration_value_get(2)
    // );
    // out.println();
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// setup
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void setup() {
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // initialise PINs

        //LiveSign
        pinMode(infoled_pin, OUTPUT);
        digitalWrite(infoled_pin, HIGH);

        // as of arduino 1.0.1 you can use INPUT_PULLUP

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // initialise serial

        // for ATmega32U4 devices:
        #if defined (__AVR_ATmega32U4__)
            // wait for arduino IDE to release all serial ports after upload.
            delay(2000);
        #endif

        Serial.begin(115200);

        // for ATmega32U4 devices:
        #if defined (__AVR_ATmega32U4__)
            // Wait for Serial Connection to be Opend from Host or
            // timeout after 6second
            uint32_t timeStamp_Start = millis();
            while( (! Serial) && ( (millis() - timeStamp_Start) < 6000 ) ) {
                // nothing to do
            }
        #endif

        Serial.println();

        Serial.print(F("# Free RAM = "));
        Serial.println(freeRam());

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // print welcome

        sketchinfo_print(Serial);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // setup CAP1188

        Serial.print(F("# Free RAM = "));
        Serial.println(freeRam());

        myTouchSensor_init(Serial);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // show serial commands

        myDebugMenu.set_callback(handleMenu_Main);
        myDebugMenu.begin(true);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // go

        Serial.println(F("Loop:"));

} /** setup **/

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// main loop
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void loop() {
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // menu input
        myDebugMenu.update();

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // timed things

        // every XXXXms
        // if ( ( millis() - ulTimeStamp_LastAction ) > cwUpdateInterval) {
        //     ulTimeStamp_LastAction =  millis();
        //     // do something
        // }


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TouchSensor things

        myTouchSensor.update();

        if ( debugOut_SensorInfo_Serial_Enabled ) {
            if (
                (millis() - debugOut_SensorInfo_TimeStamp_LastAction) >
                debugOut_SensorInfo_UpdateInterval
            ) {
                debugOut_SensorInfo_TimeStamp_LastAction = millis();

                myTouchSensor_debugOut_print(Serial);
            }
        }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // debug output

        if (
            (millis() - debugOut_LiveSign_TimeStamp_LastAction) >
            debugOut_LiveSign_UpdateInterval
        ) {
            debugOut_LiveSign_TimeStamp_LastAction = millis();

            if ( debugOut_LiveSign_Serial_Enabled ) {
                Serial.print(millis());
                Serial.print(F("ms;"));
                Serial.print(F("  free RAM = "));
                Serial.println(freeRam());
            }

            if ( debugOut_LiveSign_LED_Enabled ) {
                infoled_state = ! infoled_state;
                if (infoled_state) {
                    //set LED to HIGH
                    digitalWrite(infoled_pin, HIGH);
                } else {
                    //set LED to LOW
                    digitalWrite(infoled_pin, LOW);
                }
            }

        }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // other things

} /** loop **/

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// THE END
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
