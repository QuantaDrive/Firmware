#include "config.h"
#include "pin.h"

#include "pico/stdlib.h"

#ifndef STEPPER_H
#define STEPPER_H

enum EStepperStatus
{
    FAULTED = -2,
    NOT_HOMED = -1,
    STOPPED = 0,
    MOVING = 1,
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
    pin step_pin;
    pin dir_pin;
    pin enable_pin;
    pin diag_fault_pin;
    pin spi_cs_pin;
};

extern struct Stepper steppers[DOF];

void stepper_init();
void create_stepper(size_t stepper_id);
void stepper_set_step_dir_pins(size_t stepper_id, pin step_pin, pin dir_pin);
void stepper_set_enable_fault_pins(size_t stepper_id, pin enable_pin, pin diag_fault_pin);
void stepper_set_cs_pin_driver(size_t stepper_id, pin cs_pin, enum EDriverType driver_type);
void stepper_set_position(size_t stepper_id, uint_fast16_t position);

void stepper_step(size_t stepper_id);
void stepper_set_dir(size_t stepper_id, bool dir);
void stepper_set_status(size_t stepper_id, enum EStepperStatus status);

#endif

