#include "move.h"
#include "stepper.h"

bool move_cache_enabled = true;
uint64_t move_reminder_time = 500;
queue_t move_queue;
queue_t move_done_queue;
struct Move current_move;

/**
 * @brief Check if any motors need to move
 *
 * This function checks if any motors need to move by checking the speed and
 * position of each motor. If a motor needs to move, it sets the motor status
 * to MOVING and steps the motor. If all motors are done moving, it loads the
 * next movement from the queue.
 *
 * This function should be called once per tick.
 */
void _check_move() {
    // Check if any motors need to move
    for (size_t stepper_id = 0; stepper_id < DOF; stepper_id++) {
        if (current_move.dwell && current_move.stepper_status != 0) {
            current_move.time[0]--;
            if (current_move.time[0] == 0) {
                current_move.stepper_status = 0;
            }
        }
        if (current_move.speed[stepper_id] == 0 || (current_move.stepper_status & (1 << stepper_id)) == 0) {
            stepper_set_status(stepper_id, STOPPED);
            current_move.stepper_status &= ~(1 << stepper_id);
            continue;
        }
        current_move.time[stepper_id]++;
        current_move.time[stepper_id] %= current_move.speed[stepper_id];
        if (current_move.time[stepper_id] != 0) {
            continue;
        }
        if (steppers[stepper_id].position > current_move.pos[stepper_id]) {
            stepper_set_dir(stepper_id, false);
        } else if (steppers[stepper_id].position < current_move.pos[stepper_id]) {
            stepper_set_dir(stepper_id, true);
        } else {
            stepper_set_status(stepper_id, STOPPED);
            current_move.stepper_status &= ~(1 << stepper_id);
            continue;
        }
        stepper_set_status(stepper_id, MOVING);
        stepper_step(stepper_id);
        if (steppers[stepper_id].position == current_move.pos[stepper_id]) {
            stepper_set_status(stepper_id, STOPPED);
            current_move.stepper_status &= ~(1 << stepper_id);
        }
    }
    // if all steppers are done moving
    // load the next movement from the queue
    if (current_move.stepper_status == 0) {
        queue_try_remove(&move_queue, &current_move);
    }
}

void _move_queue_loop() {
    uint64_t start = time_us_64();
    uint64_t last_move_reminder = time_us_64();
    while (true) {
        if (time_us_64() - start >= US_PER_TICK) {
            start = time_us_64();
            _check_move();
            if (time_us_64() - last_move_reminder >= 150) {
                if (move_cache_enabled && queue_get_level(&move_queue) < 2) {                   
                    stdio_putchar_raw(0xFF); 
                    last_move_reminder = time_us_64();
                } else if (queue_is_empty(&move_queue)) {
                    stdio_putchar_raw(0xFF); 
                    last_move_reminder = time_us_64();
                }
                
            }
        }
    }
}

void move_init() {
    queue_init(&move_queue, sizeof(struct Move), MOVES_CACHE_SIZE);
    sleep_ms(5); // IMPORTANT otherwise core 1 starts too early and crashes
    multicore_launch_core1(_move_queue_loop);
}

void move_flush_queue() {
    queue_free(&move_queue);
    queue_init(&move_queue, sizeof(struct Move), MOVES_CACHE_SIZE);
}

void move_force_stop() {
    move_flush_queue();
    current_move.stepper_status = 0;
    for (size_t i = 0; i < DOF; i++) {
        stepper_set_status(i, STOPPED);
    }
}

bool move_queue_not_full() {
    if (!move_cache_enabled) {
        return queue_is_empty(&move_queue);
    }
    return !queue_is_full(&move_queue);
}

void add_move(uint8_t command_id, uint_fast16_t *pos, uint_fast32_t *speed) {
    // The status of the motors (0 = stopped, 1 = moving)
    // Set all the bits for the steppers to moving
    struct Move move;
    move.command_id = command_id;
    move.dwell = false;
    move.stepper_status = (1 << DOF) - 1;
    for (size_t i = 0; i < DOF; i++) {
        move.time[i] = 0;
        move.pos[i] = pos[i];
        move.speed[i] = speed[i];
    }
    queue_add_blocking(&move_queue, &move);
}

void add_move_single(uint8_t command_id, uint8_t index, uint_fast16_t pos, uint_fast32_t speed) {
    // The status of the motors (0 = stopped, 1 = moving)
    // Set all the bits for the steppers to moving
    struct Move move;
    move.command_id = command_id;
    move.dwell = false;
    move.stepper_status = (1 << DOF) - 1;
    for (size_t i = 0; i < DOF; i++) {
        move.time[i] = 0;
        move.pos[i] = 0;
        move.speed[i] = 0;
    }
    move.pos[index] = pos;
    move.speed[index] = speed;
    queue_add_blocking(&move_queue, &move);
}

void add_dwell(uint8_t command_id, uint_fast32_t time) {
    struct Move move;
    move.command_id = command_id;
    move.dwell = true;
    move.time[0] = time;
    move.stepper_status = 1;
}