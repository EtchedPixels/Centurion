
#pragma once

#include <stdint.h>

#define ONE_SECOND_NS 1000000000.0
#define ONE_MILISECOND_NS 1000000.0
#define ONE_MICROSECOND_NS 1000.0


struct event_t;

// event callbacks get called with their event and how many ns have passed
// since their scheduled event time.
// If event was dynamically allocated, the callback should free it.
typedef void (*callback_t)(struct event_t *event, int64_t late_ns);

struct event_t {
    int64_t delta_ns;
    callback_t callback;
    const char* name;

    // internal state
    struct event_t *next;
    int64_t scheduled_ns;
};

void schedule_event(struct event_t *event);
void cancel_event(struct event_t *event);
void run_scheduler(uint64_t current_time, unsigned trace);
int64_t get_current_time();
