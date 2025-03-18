#include "stepper.h"

#include "hardware/spi.h"

struct Stepper steppers[DOF];

void stepper_init() {
    for (size_t i = 0; i < DOF; i++) {
        create_stepper(i);
    }
}

void create_stepper(size_t stepper_id) {
    struct Stepper *stepper = &steppers[stepper_id];
    stepper->position = 0;
    stepper->status = NOT_HOMED;
    stepper->driver_type = NONE;
    stepper->step_pin = 31;
    stepper->dir_pin = 31;
    stepper->enable_pin = 31;
    stepper->diag_fault_pin = 31;
    stepper->spi_cs_pin = 31;
}

void stepper_set_step_dir_pins(size_t stepper_id, pin step_pin, pin dir_pin) {
    steppers[stepper_id].step_pin = step_pin;
    steppers[stepper_id].dir_pin = dir_pin;
    pin_output_init(step_pin);
    pin_output_init(dir_pin);
}

void stepper_set_enable_fault_pins(size_t stepper_id, pin enable_pin, pin diag_fault_pin) {
    steppers[stepper_id].enable_pin = enable_pin;
    steppers[stepper_id].diag_fault_pin = diag_fault_pin;
    pin_output_init(enable_pin);
    pin_set_value(steppers[stepper_id].enable_pin, false);
    pin_output_init(diag_fault_pin);
}

void stepper_set_cs_pin_driver(size_t stepper_id, pin cs_pin, enum EDriverType driver_type) {
    if (driver_type != GENERIC && driver_type != NONE) {
        spi_init(spi1, 1000000); // 10MHz max
        gpio_set_function(10, GPIO_FUNC_SPI);
        gpio_set_function(11, GPIO_FUNC_SPI);
        gpio_set_function(12, GPIO_FUNC_SPI);
        //gpio_set_function(13, GPIO_FUNC_SPI);
    }
    steppers[stepper_id].spi_cs_pin = cs_pin;
    steppers[stepper_id].driver_type = driver_type;
    pin_output_init(cs_pin);
}

void stepper_set_position(size_t stepper_id, uint_fast16_t position) {
    steppers[stepper_id].position = position;
}

void stepper_step(size_t stepper_id) {
    if (steppers[stepper_id].status < 0) {
        return;
    }
    pin_set_value(steppers[stepper_id].step_pin, true);
    // check stepper direction
    bool dir = pin_get_value(steppers[stepper_id].dir_pin);
    if (dir) {
        steppers[stepper_id].position++;
    } else {
        steppers[stepper_id].position--;
    }
    pin_set_value(steppers[stepper_id].step_pin, false);
}

void stepper_set_dir(size_t stepper_id, bool dir) {
    if (steppers[stepper_id].driver_type == NONE) {
        return;
    }
    pin_set_value(steppers[stepper_id].dir_pin, dir);
}

void stepper_set_status(size_t stepper_id, enum EStepperStatus status) {
    steppers[stepper_id].status = status;
    if (status < 0) {
        pin_set_value(steppers[stepper_id].enable_pin, false);
    } else {
        pin_set_value(steppers[stepper_id].enable_pin, true);
    }
}