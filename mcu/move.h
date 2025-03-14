#include "config.h"

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include "pico/multicore.h"

#ifndef MOVE_H
#define MOVE_H

struct Move {
    uint_fast8_t command_id;
    bool dwell;
    uint_fast16_t stepper_status;
    uint_fast32_t time[DOF];
    uint_fast16_t pos[DOF];
    // Speed is ticks per step
    // Ticks are defined in config.h by US_PER_TICK
    // To stop a motor, set the speed to 0 all other values are ignored
    uint_fast32_t speed[DOF];
};

extern queue_t move_queue;
extern queue_t move_done_queue;
extern struct Move current_move;

void move_init();
void move_flush_queue();
void move_force_stop();
bool move_queue_not_full();
void add_move(uint8_t command_id, uint_fast16_t *pos, uint_fast32_t *speed);
void add_move_single(uint8_t command_id, uint8_t index, uint_fast16_t pos, uint_fast32_t speed);
void add_dwell(uint8_t command_id, uint_fast32_t time);

#endif
