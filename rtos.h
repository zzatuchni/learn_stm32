#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>

#define MAX_NUM_TASKS   5
#define TASK_STACK_SIZE 512

extern volatile uint8_t   isr_lock;
void increment_isr_lock();
void decrement_isr_lock();

typedef enum {
    TASK_READY, TASK_BLOCKED
} TaskState;

typedef struct TaskData TaskData;

struct TaskData {
    void        (*funcPtr)(void);

    volatile void        *stackPtr;
    volatile TaskState   state;
    volatile uint8_t     priority;
    volatile TaskData    *nextTaskDataPtr;

    volatile uint8_t     dataPresent;

};

extern volatile bool      scheduler_enabled;
extern volatile uint64_t  tick_count;
extern volatile uint32_t  round_robin_helper;

extern volatile TaskData  task_list[MAX_NUM_TASKS];
#define GET_TASK_LIST_INDEX_FROM_MEMORY_LOCATION(x) ((uint32_t)((x) - (&task_list[0]))/((uint32_t)sizeof(TaskData)))
extern volatile TaskData  *task_list_head;
extern volatile TaskData  *current_running_task;

extern volatile uint8_t   task_list_write_pos;
extern volatile uint8_t   current_num_tasks;
extern volatile uint8_t   num_tasks_allocated;

extern volatile TaskData  *ready_task_list[MAX_NUM_TASKS];
extern volatile uint8_t   num_ready_tasks;

extern volatile uint32_t  task_stack_space[MAX_NUM_TASKS][TASK_STACK_SIZE];

typedef struct TaskTimer TaskTimer;

// NOTE: handle time_of_service overflow eventually
struct TaskTimer {
    const    TaskData   *task;
    volatile uint64_t   time_of_service;
    volatile TaskTimer  *nextTaskTimerPtr;
};

extern volatile TaskTimer task_timer_queue[MAX_NUM_TASKS];
extern volatile TaskTimer *task_timer_queue_head;
extern volatile uint8_t   task_timer_queue_write_pos;
extern volatile uint8_t   num_task_timers;

__attribute__((used)) void idle_task(void);

void load_ready_task_list();

void add_task(const void *funcPtr, uint8_t priority);

void delete_task(const void *funcPtr);

void set_current_task_state(TaskState state);

void set_task_state(TaskData *taskPtr, TaskState state);

void set_task_state_from_func(const void *funcPtr, TaskState state);

void delay_current_task(uint64_t ticks_to_wait);

__attribute__((naked)) void _on_scheduler_invoked(void);

void scheduler_init();
