#include "stepper.h"

struct Stepper steppers[DOF];

void stepper_init() {
    for (size_t i = 0; i < DOF; i++) {
        create_stepper(&steppers[i]);
    }
}

void create_stepper(struct Stepper *stepper) {
    stepper->position = 0;
    stepper->status = STOPPED;
    stepper->driver_type = NONE;
    stepper->step_pin = 0;
    stepper->dir_pin = 0;
    stepper->enable_pin = 0;
    stepper->diag_fault_pin = 0;
    stepper->spi_cs_pin = 0;
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

void stepper_set_step_dir_pins(struct Stepper *stepper, uint_fast8_t step_pin, uint_fast8_t dir_pin) {
    stepper->step_pin = step_pin;
    stepper->dir_pin = dir_pin;
    _stepper_set_output_pin(step_pin);
    _stepper_set_output_pin(dir_pin);
}

void stepper_set_enable_fault_pins(struct Stepper *stepper, uint_fast8_t enable_pin, uint_fast8_t diag_fault_pin) {
    stepper->enable_pin = enable_pin;
    stepper->diag_fault_pin = diag_fault_pin;
    _stepper_set_output_pin(enable_pin);
    _stepper_set_input_pin(diag_fault_pin);
}

void stepper_set_cs_pin_driver(struct Stepper *stepper, uint_fast8_t cs_pin, enum EDriverType driver_type) {
    stepper->spi_cs_pin = cs_pin;
    stepper->driver_type = driver_type;
    _stepper_set_output_pin(cs_pin);
}

void stepper_step(struct Stepper *stepper) {
    if (stepper->driver_type == NONE) {
        return;
    }
    gpio_set_mask(1 << stepper->step_pin);
    gpio_clr_mask(1 << stepper->step_pin);
    // check stepper direction
    bool inverted = stepper->dir_pin & 0b10000000;
    bool dir = gpio_get(stepper->dir_pin);
    dir = dir ^ inverted;
    if (dir) {
        stepper->position++;
    } else {
        stepper->position--;
    }
}

void stepper_set_dir(struct Stepper *stepper, bool dir) {
    if (stepper->driver_type == NONE) {
        return;
    }
    bool inverted = stepper->dir_pin & 0b10000000;
    dir = dir ^ inverted;
    if (dir) {
        gpio_set_mask(1 << (stepper->dir_pin & 0b01111111));
    } else {
        gpio_clr_mask(1 << (stepper->dir_pin & 0b01111111));
    }
}

void stepper_set_status(struct Stepper *stepper, enum EStepperStatus status) {
    stepper->status = status;
}