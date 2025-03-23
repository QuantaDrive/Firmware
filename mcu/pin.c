#include "pin.h"

#include "hardware/spi.h"

struct mux muxes[4];
struct shift_register shift_registers[4];

void _mux_init(uint8_t id) {
    muxes[id].data = 31;
    muxes[id].enable = 31;
    muxes[id].address[0] = 31;
    muxes[id].address[1] = 31;
    muxes[id].address[2] = 31;
}

void _shift_register_init(uint8_t id) {
    shift_registers[id].latch = 31;
    shift_registers[id].offset = 0;
    shift_registers[id].data_stored = 0;
}

void pin_init() {
    spi_init(spi0, 37500000); // 37.5MHz (75MHz was too fast)
    gpio_set_function(2, GPIO_FUNC_SPI);
    gpio_set_function(3, GPIO_FUNC_SPI);
    for (size_t i = 0; i < 4; i++) {
        _mux_init(i);
        _shift_register_init(i);
    }
}

void pin_clear() {
    uint8_t data[4] = {0, 0, 0, 0};
    spi_write_blocking(spi0, data, 4);
    for (size_t i = 0; i < 4; i++) {
        shift_registers[i].data_stored = 0;
        pin_set_value(shift_registers[i].latch, true);
        pin_set_value(shift_registers[i].latch, false);
    }
}

void pin_output_init(pin pin) {
    if (pin >> 6 != 0) {
        return;
    }
    pin = (pin & 0b00011111);
    if (pin == 31) {
        return;
    }
    gpio_set_function_masked(1 << pin, GPIO_FUNC_SIO);
    gpio_set_dir_out_masked(1 << pin);
}

void pin_input_init(pin pin) {
    pin = (pin & 0b00011111);
    if (pin == 31) {
        return;
    }
    gpio_set_function_masked(1 << pin, GPIO_FUNC_SIO);
    gpio_set_dir_in_masked(1 << pin);
}

void mux_init(struct mux *mux, uint8_t id, bool output) {
    if (output) {
        pin_output_init(mux->data);
    } else {
        pin_input_init(mux->data);
    }
    pin_output_init(mux->enable);
    pin_set_value(mux->enable, false);
    for (size_t i = 0; i < 3; i++) {
        pin_output_init(mux->address[i]);
    }
    muxes[id] = *mux;
}

void shift_register_init(struct shift_register *shift_register, uint8_t id) {
    pin_output_init(shift_register->latch);
    uint8_t data[4];
    data[0] = 0;
    spi_write_blocking(spi0, data, shift_register->offset + 1);
    pin_set_value(shift_register->latch, true);
    shift_register->data_stored = 0;
    shift_registers[id] = *shift_register;
    pin_set_value(shift_register->latch, false);
}

void _pin_set_value(pin pin, bool value) {
    if (pin == 31) {
        return;
    }
    if (value) {
        gpio_set_mask(1 << pin);
    } else {
        gpio_clr_mask(1 << pin);
    }
}

void _mux_set_value(pin pin, bool value) {
    uint8_t mux_id = pin >> 3 & 0b00000011;
    struct mux *mux = &muxes[mux_id];
    if (!(value ^ mux->data & 0b00100000)) {
        // if the value to the data pin wil be false disable the mux
        pin_set_value(mux->enable, false);
        return;
    }
    pin_set_value(mux->enable, false);
    pin_set_value(mux->data, value);
    uint8_t mux_pin = pin & 0b00000111;
    for (size_t i = 0; i < 3; i++) {
        bool address_high = mux_pin >> i & 0b00000001;
        pin_set_value(mux->address[i], address_high);
    }
    pin_set_value(mux->enable, true);
}

void _shift_register_set_value(pin pin, bool value) {
    uint8_t shift_register_id = pin >> 3 & 0b00000011;
    uint8_t shift_register_pin = pin & 0b00000111;
    struct shift_register *shift_register = &shift_registers[shift_register_id];
    uint8_t new_data_stored = shift_register->data_stored;
    if (value) {
        new_data_stored |= 1 << shift_register_pin;
    } else {
        new_data_stored &= ~(1 << shift_register_pin);
    }
    if (new_data_stored == shift_register->data_stored) {
        return;
    }
    uint8_t data[4];
    data[0] = new_data_stored;
    spi_write_blocking(spi0, data, shift_register->offset + 1);
    pin_set_value(shift_register->latch, true);
    shift_register->data_stored = new_data_stored;
    pin_set_value(shift_register->latch, false);
}

void pin_set_value(pin pin, bool value) {
    uint8_t op_code = pin >> 6;
    bool invert = pin >> 5 & 0b00000001;
    pin = (pin & 0b00011111);
    value = value ^ invert;
    switch (op_code) {
    case 0b00000000:
        _pin_set_value(pin, value);
        break;
    case 0b00000001:
        _mux_set_value(pin, value);
        break;
    case 0b00000010:
        _shift_register_set_value(pin, value);
        break;
    default:
        break;
    }
}

bool _pin_get_value(pin pin, bool invert) {
    if (pin == 31) {
        return false;
    }
    return gpio_get(pin) ^ invert;
}

bool _mux_get_value(pin pin, bool invert) {
    uint8_t mux_id = pin >> 3 & 0b00000011;
    uint8_t mux_pin = pin & 0b00000111;
    struct mux *mux = &muxes[mux_id];
    for (size_t i = 0; i < 3; i++) {
        bool address_high = mux_pin >> i & 0b00000001;
        pin_set_value(mux->address[i], address_high);
    }
    pin_set_value(mux->enable, true);
    return pin_get_value(mux->data) ^ invert;
}

bool _shift_register_get_value(pin pin, bool invert) {
    uint8_t shift_register_id = pin >> 3 & 0b00000011;
    uint8_t shift_register_pin = pin & 0b00000111;
    uint8_t shift_register_data = shift_registers[shift_register_id].data_stored;
    return shift_register_data >> shift_register_pin & 0b00000001 ^ invert;
}

bool pin_get_value(pin pin) {
    uint8_t op_code = pin >> 6;
    bool invert = pin >> 5 & 0b00000001;
    pin = (pin & 0b00011111);
    switch (op_code) {
    case 0b00000000:
        return _pin_get_value(pin, invert);
        break;
    case 0b00000001:
        return _mux_get_value(pin, invert);
        break;
    case 0b00000010:
        return _shift_register_get_value(pin, invert);
        break;
    default:
        break;
    }
}