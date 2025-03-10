#include "config.h"
#include "stepper.h"
#include "move.h"

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

    stepper_init();
    move_init();
    
    while (true) {
        #ifndef DEBUG
        if (tud_cdc_available()) {
        #endif
            uint8_t op_code = stdio_getchar();
            switch (op_code) {
            case 0b00000000: {  // reset controller
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
            case 0b00000001: {  // set step and dir pins
                uint8_t stepper_id = stdio_getchar();
                if (stepper_id > DOF) {
                    stdio_getchar();
                    stdio_getchar();
                    break;
                }
                uint8_t step_pin = stdio_getchar();
                uint8_t dir_pin = stdio_getchar();
                stepper_set_step_dir_pins(stepper_id, step_pin, dir_pin);
                break;
            }
            case 0b00000010: {  // set enable and diag/fault pins
                uint8_t stepper_id = stdio_getchar();
                if (stepper_id > DOF) {
                    stdio_getchar();
                    stdio_getchar();
                    break;
                }
                uint8_t enable_pin = stdio_getchar();
                uint8_t diag_fault_pin = stdio_getchar();
                stepper_set_enable_fault_pins(stepper_id, enable_pin, diag_fault_pin);
                break;
            }
            case 0b00000011: {  // set spi cs pin and driver type
                uint8_t stepper_id = stdio_getchar();
                if (stepper_id > DOF) {
                    stdio_getchar();
                    stdio_getchar();
                    break;
                }
                enum EDriverType driver_type = stdio_getchar();
                uint8_t spi_cs_pin = stdio_getchar();
                stepper_set_cs_pin_driver(stepper_id, spi_cs_pin, driver_type);
                break;
            }
            case 0b00010000: {  // add movement of all steppers
                uint_fast16_t position[DOF];
                uint_fast32_t speed[DOF];
                for (int i = 0; i < DOF; i++) {
                    if (stdio_getchar() != 0b00000000) {
                        //TODO make this better more resistent
                        break;
                    }
                    position[i] = stdio_getchar() << 8;
                    position[i] |= stdio_getchar();
                    speed[i] = stdio_getchar() << 16;
                    speed[i] |= stdio_getchar() << 8;
                    speed[i] |= stdio_getchar();
                }
                add_move(position, speed);
                break;
            }
            case 0b00010001: {  // add movement of specific stepper
                uint8_t stepper_id = stdio_getchar();
                if (stepper_id > DOF) {
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
                add_move_single(stepper_id, position, speed);
                break;
            }
            case 0b00010100: {  // home all steppers
                break;
            }
            case 0b00010101: {  // home specific stepper
                break;
            }
            case 0b00011000: {  // stop all steppers
                move_force_stop();
                break;
            }
            case 0b00011110: {  // set stepper status homed
                uint8_t stepper_id = stdio_getchar();
                if (steppers[stepper_id].status == NOT_HOMED) {
                    stepper_set_status(stepper_id, STOPPED);
                }
                break;
            }
            case 0b00011111: {  // set stepper position
                uint8_t stepper_id = stdio_getchar();
                uint_fast16_t position;
                position = stdio_getchar() << 8;
                position |= stdio_getchar();
                stepper_set_position(stepper_id, position);
                break;
            }
            case 0b11111111: {  // keep alive
                break;
            }
            default:
                break;
            }
            stdio_putchar_raw(op_code);
            if (move_queue_not_full()) {
                stdio_putchar_raw(0x01); 
            } else {
                stdio_putchar_raw(0x00);
            }
        #ifndef DEBUG
        }
        #endif
    }
}
