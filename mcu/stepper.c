#include "stepper.h"

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

void _stepper_set_output_pin(uint_fast8_t pin) {
    pin = (pin & 0b01111111); // remove the invert bit
    gpio_set_function_masked(1 << pin, GPIO_FUNC_SIO);
    gpio_set_dir_out_masked(1 << pin);
}

void _stepper_set_input_pin(uint_fast8_t pin) {
    pin = (pin & 0b01111111); // remove the invert bit
    gpio_set_function_masked(1 << pin, GPIO_FUNC_SIO);
    gpio_set_dir_in_masked(1 << pin);
}

void stepper_set_step_dir_pins(size_t stepper_id, uint_fast8_t step_pin, uint_fast8_t dir_pin) {
    steppers[stepper_id].step_pin = step_pin;
    steppers[stepper_id].dir_pin = dir_pin;
    _stepper_set_output_pin(step_pin);
    _stepper_set_output_pin(dir_pin);
}

void stepper_set_enable_fault_pins(size_t stepper_id, uint_fast8_t enable_pin, uint_fast8_t diag_fault_pin) {
    steppers[stepper_id].enable_pin = enable_pin;
    steppers[stepper_id].diag_fault_pin = diag_fault_pin;
    _stepper_set_output_pin(enable_pin);
    _stepper_set_input_pin(diag_fault_pin);
}

void stepper_set_cs_pin_driver(size_t stepper_id, uint_fast8_t cs_pin, enum EDriverType driver_type) {
    steppers[stepper_id].spi_cs_pin = cs_pin;
    steppers[stepper_id].driver_type = driver_type;
    _stepper_set_output_pin(cs_pin);
}

void stepper_set_position(size_t stepper_id, uint_fast16_t position) {
    steppers[stepper_id].position = position;
}

void stepper_step(size_t stepper_id) {
    if (steppers[stepper_id].status < 0) {
        return;
    }
    gpio_set_mask(1 << steppers[stepper_id].step_pin);
    // check stepper direction
    bool inverted = steppers[stepper_id].dir_pin & 0b10000000;
    bool dir = gpio_get(steppers[stepper_id].dir_pin);
    dir = dir ^ inverted;
    if (dir) {
        steppers[stepper_id].position++;
    } else {
        steppers[stepper_id].position--;
    }
    gpio_clr_mask(1 << steppers[stepper_id].step_pin);
}

void stepper_set_dir(size_t stepper_id, bool dir) {
    if (steppers[stepper_id].driver_type == NONE) {
        return;
    }
    bool inverted = steppers[stepper_id].dir_pin & 0b10000000;
    dir = dir ^ inverted;
    if (dir) {
        gpio_set_mask(1 << (steppers[stepper_id].dir_pin & 0b01111111));
    } else {
        gpio_clr_mask(1 << (steppers[stepper_id].dir_pin & 0b01111111));
    }
}

void stepper_set_status(size_t stepper_id, enum EStepperStatus status) {
    steppers[stepper_id].status = status;
}