#include "scheduler.h"
#include "cpu6.h"

#include <stdlib.h>
#include <assert.h>
#include <stdio.h>

static struct event_t* event_list = NULL;
static uint64_t next_event = UINT64_MAX;
static unsigned trace_schedule = 0;

static void update_next_event()
{
    // Update next_event
    next_event = (event_list == NULL) ? UINT64_MAX : event_list->scheduled_ns;

}

void schedule_event(struct event_t *event)
{
    int64_t now = get_current_time();
    int64_t scheduled = get_current_time() + event->delta_ns;

    if (trace_schedule) {
        long now_seconds = now / ONE_SECOND_NS;
        long now_us = (now % (int64_t)ONE_SECOND_NS) / ONE_MICROSECOND_NS;
        double delta = (double)event->delta_ns / ONE_MICROSECOND_NS;

        if (scheduled <= now) {
            fprintf(stderr, "%li.%06li: Scheduling %s immediately\n",
                now_seconds, now_us, event->name);
        } else {
            fprintf(stderr, "%li.%06li: Scheduling %s in %.3f us\n",
                now_seconds, now_us, event->name, delta);
        }
    }

    if (event->next != NULL) {
        if (trace_schedule) {
            fprintf(stderr, "%s was already scheduled.\n", event->name);
        }
        cancel_event(event);
    }

    event->scheduled_ns = scheduled;

    struct event_t** next_ptr = &event_list;

    // Insert event into sorted list.
    while (*next_ptr && (*next_ptr)->scheduled_ns < scheduled) {
        next_ptr = &((*next_ptr)->next);
    }
    event->next = *next_ptr;
    *next_ptr = event;

    update_next_event();
}

void run_scheduler(uint64_t current_time, unsigned trace)
{
    trace_schedule = trace;
    if (next_event > current_time)
        return;

    assert(event_list);

    while (next_event <= current_time) {
        // Pop event
        struct event_t* event = event_list;
        event_list = event->next;
        event->next = NULL;
        update_next_event();

        int64_t late_ns = current_time - event->scheduled_ns;

        if (trace) {
            long seconds = current_time / ONE_SECOND_NS;
            long us = (current_time % (int64_t)ONE_SECOND_NS) / ONE_MICROSECOND_NS;
            fprintf(stderr, "%li.%06li: Event %s dispatched. It was %.3f us late.\n",
                seconds, us, event->name, (double)late_ns / ONE_MICROSECOND_NS);
        }

        // Run callback
        event->callback(event, late_ns);
    }
}


void cancel_event(struct event_t *event)
{
    struct event_t** next_ptr = &event_list;

    if (trace_schedule) {
        int64_t now = get_current_time();
        long seconds = now / ONE_SECOND_NS;
        long us = (now % (int64_t)ONE_SECOND_NS) / ONE_MICROSECOND_NS;
        fprintf(stderr, "%li.%06li: Event %s canceled\n",
                seconds, us, event->name);
    }

    // find and remove event from list
    while(*next_ptr != NULL) {
        struct event_t* next = *next_ptr;
        if (next == event) {
            *next_ptr = next->next;
            event->next = NULL;
            update_next_event();
            return;
        }
        next_ptr = &next->next;
    }
}

int64_t scheduler_next()
{
    if (event_list == NULL)
        return -1;
    return next_event;
}