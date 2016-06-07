/******************************************************************************

    CAP1188_TWI_wMenu.ino
        sketch for extended test with the CAP1188 sensor breakout form adafruit.
        debugout on usbserial interface: 115200baud

    hardware:
        Board:
            Arduino compatible (with serial port)
            LED on pin 13
            CAP1188
                TWI (SDA & SCL)
                reset on MOSI


    libraries used:
        ~ slight_CAP1188_TWI
        ~ slight_DebugMenu
            written by stefan krueger (s-light),
                github@s-light.eu, http://s-light.eu, https://github.com/s-light/
            license: MIT

    written by stefan krueger (s-light),
        github@s-light.eu, http://s-light.eu, https://github.com/s-light/

    changelog / history
        24.11.2015 16:30 created (based on DebugMenu_Simple.ino)
        27.11.2015 16:30 renamed/structured

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
    out.println(F("| CAP1188_TWI_wMenu.ino"));
    out.println(F("|   sketch for extended test with the"));
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

slight_CAP1188_TWI myTouchSensor(
    MOSI,  // HW reset pin
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
            out.println(F("\t 'y': toggle DebugOut livesign print"));
            out.println(F("\t 'Y': toggle DebugOut livesign LED"));
            out.println(F("\t 'x': tests"));
            out.println();
            out.println(F("\t 's': sensitivity set [1, 2, .., 64, 128] 's128'"));
            out.println(F("\t 'S': sensitivity get "));
            out.println(F("\t 't': threshold set (0..127) 't127'"));
            out.println(F("\t 'T': threshold get "));
            out.println(F("\t 'r': soft reset of sensor 'r1'"));
            out.println(F("\t 'R': HW sensor reset "));
            out.println(F("\t 'b': multiple touch blocked enable set 'b1'"));
            out.println(F("\t 'B': multiple touch blocked enable get"));
            out.println();
            out.println(F("\t 'l': setup leds special"));
            out.println(F("\t 'L': print leds special"));
            out.println(F("\t 'a': activate leds special"));
            out.println(F("\t 'A': deactivate leds special"));
            out.println();
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
        case 'Y': {
            out.println(F("\t toggle DebugOut livesign LED:"));
            debugOut_LiveSign_LED_Enabled = !debugOut_LiveSign_LED_Enabled;
            out.print(F("\t debugOut_LiveSign_LED_Enabled:"));
            out.println(debugOut_LiveSign_LED_Enabled);
        } break;
        case 'x': {
            // get state
            out.println(F("__________"));
            out.println(F("Tests:"));

            out.println(F("nothing to do."));

            // uint16_t wTest = 65535;
            uint16_t wTest = atoi(&command[1]);
            out.print(F("wTest: "));
            out.print(wTest);
            out.println();

            out.print(F("1: "));
            out.print((byte)wTest);
            out.println();

            out.print(F("2: "));
            out.print((byte)(wTest>>8));
            out.println();

            out.println();

            // set leds
            myTouchSensor.led_output_control_set_led(7, 1);
            myTouchSensor.led_output_control_set_led(8, 0);

            out.println(F("__________"));
        } break;
        //---------------------------------------------------------------------
        case 's': {
            out.print(F("\t sensitivity set "));
            // convert part of string to int
            // (up to first char that is not a number)
            uint8_t value = atoi(&command[1]);
            out.print(value);
            out.print(F(" = "));
            slight_CAP1188_TWI::sensitivity_t sens = myTouchSensor.sensitivity_convert(value);
            myTouchSensor.sensitivity_set(sens);
            myTouchSensor.sensitivity_print(Serial, sens);
            Serial.println();
        } break;
        case 'S': {
            out.print(F("\t sensitivity get "));
            myTouchSensor.sensitivity_print(Serial, myTouchSensor.sensitivity_get());
            Serial.println();
        } break;
        case 't': {
            out.print(F("\t threshold set "));
            uint8_t value = atoi(&command[1]);
            out.print(value);
            myTouchSensor.sensor_input_threshold_set(0, value);
            Serial.println();
        } break;
        case 'T': {
            out.print(F("\t threshold get "));
            out.print(myTouchSensor.sensor_input_threshold_get(0));
            Serial.println();
        } break;
        case 'r': {
            out.print(F("\t reset "));
            out.print(F(" -- TODO --"));
            Serial.println();
        } break;
        case 'R': {
            out.print(F("\t HW sensor reset .."));
            myTouchSensor.sensor_HW_reset();
            myTouchSensor.sensor_default_configuration();
            out.print(F(". done"));
            Serial.println();
        } break;
        case 'b': {
            out.print(F("\t multiple touch blocked enable set "));
            uint8_t value = atoi(&command[1]);
            out.print(value);
            myTouchSensor.multiple_touch_blocking_enable_set(value);
            Serial.println();
        } break;
        case 'B': {
            out.print(F("\t multiple touch blocked enable get "));
            out.print(
                myTouchSensor.multiple_touch_blocking_enable_get()
            );
            Serial.println();
        } break;
        case 'l': {
            out.print(F("\t setup leds special "));
            // enable stand alone host control
            myTouchSensor.sensor_input_led_linking_set_led(7, 0);
            myTouchSensor.sensor_input_led_linking_set_led(8, 0);
            // // set output as push-pull
            myTouchSensor.led_output_type_set_led(7, 1);
            myTouchSensor.led_output_type_set_led(8, 1);
            // set led behavior to breathe
            myTouchSensor.led_behavior_set(
                7,
                slight_CAP1188_TWI::behavior_breathe
            );
            Serial.println();
        } break;
        case 'L': {
            out.print(F("\t print leds special "));
            Serial.println();
            Serial.print(F("\t linking: "));
            slight_DebugMenu::print_Binary_8(
                Serial,
                myTouchSensor.sensor_input_led_linking_get()
            );
            Serial.println();
            Serial.print(F("\t output type: "));
            slight_DebugMenu::print_Binary_8(
                Serial,
                myTouchSensor.led_output_type_get()
            );
            Serial.println();
            Serial.print(F("\t output control: "));
            slight_DebugMenu::print_Binary_8(
                Serial,
                myTouchSensor.led_output_control_get()
            );
            Serial.println();
            Serial.print(F("\t behavior: "));
            myTouchSensor.led_behavior_print(
                Serial,
                myTouchSensor.led_behavior_get(7)
            );
            Serial.println();
            myTouchSensor.led_behavior_print_all(Serial);
            Serial.println();
        } break;
        case 'a': {
            out.print(F("\t activate leds special "));
            // set leds
            myTouchSensor.led_output_control_set_led(7, 0);
            myTouchSensor.led_output_control_set_led(8, 1);
            Serial.println();
        } break;
        case 'A': {
            out.print(F("\t deactivate leds special "));
            // set leds
            myTouchSensor.led_output_control_set_led(7, 1);
            myTouchSensor.led_output_control_set_led(8, 0);
            Serial.println();
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

        out.println("\t sensor information:");
        out.print("\t   Product ID: ");
        out.print("0x");
        out.println(
            myTouchSensor.read_register(
                slight_CAP1188_TWI::REG_Product_ID
            ),
            HEX
        );
        out.print("\t   Manufacturer ID: ");
        out.print("0x");
        out.println(
            myTouchSensor.read_register(
                slight_CAP1188_TWI::REG_Manufacturer_ID
            ),
            HEX
        );
        out.print("\t   Revision: ");
        out.print("0x");
        out.println(
            myTouchSensor.read_register(
                slight_CAP1188_TWI::REG_Revision_ID
            ),
            HEX
        );

        myTouchSensor.touch_event_set_callback(touch_event);

        out.println("\t change config: ");


        // out.print("\t   get sensitivity: ");
        // // slight_CAP1188_TWI::sensitivity_print(out, myTouchSensor.sensitivity_get());
        // myTouchSensor.sensitivity_print(out);
        // out.println();
        //
        // out.print("\t   set sensitivity to ");
        // myTouchSensor.sensitivity_set(slight_CAP1188_TWI::sensitivity_2x);
        // // slight_CAP1188_TWI::sensitivity_print(out, myTouchSensor.sensitivity_get());
        // myTouchSensor.sensitivity_print(out);
        // out.println();
        //
        // out.print("\t   get sensor threshold: ");
        // out.print(myTouchSensor.sensor_input_threshold_get(0));
        // out.println();
        //
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

        myTouchSensor.update();

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
