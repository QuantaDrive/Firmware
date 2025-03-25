#include "config.h"

#include "pico/stdlib.h"

#ifndef PIN_H
#define PIN_H

typedef uint_fast8_t pin;

struct mux {
    pin data;
    pin enable;
    pin address[3];
};

struct shift_register {
    pin latch;
    uint_fast8_t offset;
    uint_fast8_t data_stored;
};

struct interrupt_pin {
    bool debounce_value;
    bool value;
    pin pin;
};


/*
    PIN: 00ippppp
    i: inverted?
    p: pin number

    MUX: 01innaaa
    i: inverted?
    n: mux number/id
    a: pin adress in mux

    shift register: 10innaaa
    i: inverted?
    n: shift register number/id
    a: pin adress in shift register
*/


extern struct mux muxes[4];
extern struct shift_register shift_registers[4];

extern uint8_t interrupt_pin_count;
extern struct interrupt_pin interrupt_pins[MAX_INTERRUPT_PINS];

void pin_init();
void pin_clear();

void pin_output_init(pin pin);
void pin_input_init(pin pin);
void pin_interrupt_init(pin pin);
void mux_init(struct mux *mux, uint8_t id, bool output);
void shift_register_init(struct shift_register *shift_register, uint8_t id);

void pin_check_interrupt();
void pin_set_value(pin pin, bool value);
bool pin_get_value(pin pin);

#endif