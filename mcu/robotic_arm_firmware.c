#include "config.h"
#include "stepper.h"
#include "move.h"
#include "pin.h"

#include <stdio.h>
#ifndef DEBUG
    #include <tusb.h>
#endif
#include "pico/stdlib.h"

int main() {
    stdio_init_all();
    #ifndef DEBUG
    while (!stdio_usb_connected()) {
        sleep_ms(10);
    }
    #endif

    pin_init();
    stepper_init();
    move_init();
    
    while (true) {
        #ifndef DEBUG
        if (tud_cdc_available()) {
        #endif
            uint8_t op_code = stdio_getchar();
            uint8_t id = stdio_getchar();
            uint8_t object_conf = id & 0b00001111;
            uint8_t command_id = id >> 4;
            switch (op_code) {
            // Reset controller
            case 0b00000000: {  // reset controller
                move_cache_enabled = true;
                move_force_stop();
                stepper_init();
                // Flush input buffer
                #ifndef DEBUG
                while (tud_cdc_available()) {
                    stdio_getchar();
                }
                #endif
                break;
            }
            // Actions
            case 0b00010000: {  // home all steppers
                break;
            }
            case 0b00010001: {  // home specific stepper
                break;
            }
            case 0b00010010: {  // add movement of all steppers
                uint_fast16_t position[DOF];
                uint_fast32_t speed[DOF];
                for (int i = 0; i < DOF; i++) {
                    position[i] = stdio_getchar() << 8;
                    position[i] |= stdio_getchar();
                    speed[i] = stdio_getchar() << 16;
                    speed[i] |= stdio_getchar() << 8;
                    speed[i] |= stdio_getchar();
                }
                add_move(command_id, position, speed);
                break;
            }
            case 0b00010011: {  // add movement of specific stepper
                // object_conf is stepper id
                if (object_conf > DOF) {
                    for (int i = 0; i < 5; i++) {
                        stdio_getchar();
                    }
                    break;
                }
                uint16_t position = stdio_getchar() << 8;
                position |= stdio_getchar();
                uint32_t speed = stdio_getchar() << 16;
                speed |= stdio_getchar() << 8;
                speed |= stdio_getchar();
                add_move_single(command_id, object_conf, position, speed);
                break;
            }
            case 0b00010100: {  // dwell
                uint32_t time = stdio_getchar() << 16;
                time |= stdio_getchar() << 8;
                time |= stdio_getchar();
                add_dwell(command_id, time);
                break;
            }
            case 0b00010101: {  // set gpio pin
                bool value = object_conf & 0b00000001;
                pin pin = stdio_getchar();
                pin_set_value(pin, value);
                break;
            }
            case 0b00010110: {  // disable stepper
                break;
            }
            case 0b00010111: {  // stop all steppers
                move_force_stop();
                break;
            }
            // Overrides
            case 0b00011000: {  // set stepper status homed
                // object_conf is stepper id
                if (steppers[object_conf].status == NOT_HOMED) {
                    stepper_set_status(object_conf, STOPPED);
                }
                break;
            }
            case 0b00011001: {  // set stepper position
                // object_conf is stepper id
                uint_fast16_t position;
                position = stdio_getchar() << 8;
                position |= stdio_getchar();
                stepper_set_position(object_conf, position);
                break;
            }
            // Config
            case 0b00100001: {  // set step and dir pins
                // object_conf is stepper id
                if (object_conf > DOF) {
                    stdio_getchar();
                    stdio_getchar();
                    break;
                }
                uint8_t step_pin = stdio_getchar();
                uint8_t dir_pin = stdio_getchar();
                stepper_set_step_dir_pins(object_conf, step_pin, dir_pin);
                break;
            }
            case 0b00100010: {  // set enable and diag/fault pins
                // object_conf is stepper id
                if (object_conf > DOF) {
                    stdio_getchar();
                    stdio_getchar();
                    break;
                }
                uint8_t enable_pin = stdio_getchar();
                uint8_t diag_fault_pin = stdio_getchar();
                stepper_set_enable_fault_pins(object_conf, enable_pin, diag_fault_pin);
                break;
            }
            case 0b00100011: {  // set spi cs pin and driver type
                // object_conf is stepper id
                if (object_conf > DOF) {
                    stdio_getchar();
                    stdio_getchar();
                    break;
                }
                enum EDriverType driver_type = stdio_getchar();
                uint8_t spi_cs_pin = stdio_getchar();
                stepper_set_cs_pin_driver(object_conf, spi_cs_pin, driver_type);
                break;
            }
            case 0b00100100: {  // configure multiplexer
                uint8_t mux_id = object_conf & 0b00000011;
                if (mux_id > 3) {
                    for (int i = 0; i < 6; i++) {
                        stdio_getchar();
                    }
                    break;
                }
                // wether the mux is used as input or output
                uint8_t output = object_conf >> 2 & 0b00000001;
                struct mux mux;
                mux.data = stdio_getchar();
                mux.enable = stdio_getchar();
                mux.address[0] = stdio_getchar();
                mux.address[1] = stdio_getchar();
                mux.address[2] = stdio_getchar();
                mux_init(&mux, mux_id, output);
                break;
            }
            case 0b00100101: {  // configure shift register
                uint8_t shift_register_id = object_conf & 0b00000011;
                uint8_t offset = object_conf >> 2 & 0b00000011;
                if (shift_register_id > 3) {
                    for (int i = 0; i < 5; i++) {
                        stdio_getchar();
                    }
                    break;
                }
                struct shift_register shift_register;
                shift_register.latch = stdio_getchar();
                shift_register.offset = offset;
                shift_register_init(&shift_register, shift_register_id);
                break;
            }
            case 0b00111100: {  // set cache mode
                uint8_t cache_mode = object_conf & 0b00000001;
                if (cache_mode == 0) {
                    move_reminder_time = 100;
                    move_cache_enabled = false;
                } else {
                    move_reminder_time = 500;
                    move_cache_enabled = true;
                }
                move_flush_queue();
                break;
            }
            // Debug
            case 0b01111111: {  // keep alive
                break;
            }
            default:
                break;
            }
            if (move_queue_not_full()) {
                stdio_putchar_raw(0xFF);
            }
            stdio_putchar_raw(op_code);
        #ifndef DEBUG
        }
        #endif
    }
}
