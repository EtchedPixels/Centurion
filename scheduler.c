#include "scheduler.h"
#include "cpu6.h"

#include <stdlib.h>
#include <assert.h>

static struct event_t* event_list = NULL;
static uint64_t next_event = UINT64_MAX;

void schedule_event(struct event_t *event)
{
    uint64_t now = get_current_time();
    int64_t scheduled = get_current_time() + event->delta_ns;
    assert(scheduled > now);

    if (event->next != NULL)
        cancel_event(event);

    event->scheduled_ns = scheduled;

    if (!event_list) {
        event->next = NULL;
        event_list = event;
        next_event = scheduled;
        return;
    }

    struct event_t* item = event_list;

    while (item && item->scheduled_ns < scheduled) {
        item = item->next;
    }
    event->next = item->next;
    item->next = event;
}

void run_scheduler(uint64_t current_time)
{
    if (next_event > current_time)
        return;

    assert(event_list);

    while (next_event <= current_time) {
        // Pop event
        struct event_t* event = event_list;
        event_list = event_list->next;
        event->next = NULL;

        // Update next_event
        next_event = (event_list == NULL) ? UINT64_MAX : event_list->scheduled_ns;

        // Run callback
        event->callback(event, current_time - event->scheduled_ns);
    }
}


void cancel_event(struct event_t *event)
{
    struct event_t** next_ptr = &event_list;

    while(*next_ptr != NULL) {
        struct event_t* next = *next_ptr;
        if (next == event) {
            *next_ptr = next->next;
            event->next = NULL;
            return;
        }
        next_ptr = &next->next;
    }
}