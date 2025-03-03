#include "config.h"

#include "pico/stdlib.h"

#ifndef STEPPER_H
#define STEPPER_H

enum EStepperStatus
{
    STOPPED = 0,
    MOVING = 1,
    FAULT = 2
};

enum EDriverType
{
    NONE = 0,
    GENERIC = 1,
    TMC2240 = 2,
    TMC5160 = 3
};

struct Stepper
{
    uint_fast16_t position;
    enum EStepperStatus status;
    enum EDriverType driver_type;
    // all the pin parameters have the top bit set if inverted
    // and the bottom 7 bits are the pin number
    uint_fast8_t step_pin;
    uint_fast8_t dir_pin;
    uint_fast8_t enable_pin;
    uint_fast8_t diag_fault_pin;
    uint_fast8_t spi_cs_pin;
};

extern struct Stepper steppers[DOF];

void stepper_init();
void create_stepper(struct Stepper *stepper);
void stepper_set_step_dir_pins(struct Stepper *stepper, uint_fast8_t step_pin, uint_fast8_t dir_pin);
void stepper_set_enable_fault_pins(struct Stepper *stepper, uint_fast8_t enable_pin, uint_fast8_t diag_fault_pin);
void stepper_set_cs_pin_driver(struct Stepper *stepper, uint_fast8_t cs_pin, enum EDriverType driver_type);
void stepper_step(struct Stepper *stepper);
void stepper_set_dir(struct Stepper *stepper, bool dir);
void stepper_set_status(struct Stepper *stepper, enum EStepperStatus status);

#endif

