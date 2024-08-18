#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>

#define MAX_NUM_TASKS 4
#define TASK_STACK_SIZE 1024

typedef enum {
    TASK_READY, TASK_BLOCKED
} TaskState;

typedef struct {
    const    void        (*funcPtr)(void);

    volatile void        *stackPtr;
    volatile TaskState   state;
    volatile uint8_t     priority;
    volatile TaskData    *nextTaskDataPtr;

    volatile uint8_t     dataPresent;

} TaskData;

__attribute__((used)) static volatile bool      scheduler_enabled   = false;
__attribute__((used)) static volatile uint64_t  tick_count          = 0;
__attribute__((used)) static volatile uint8_t   round_robin_helper  = 0;

__attribute__((used)) static volatile TaskData  task_list[MAX_NUM_TASKS];
#define GET_TASK_LIST_INDEX_FROM_MEMORY_LOCATION(x) (((x) - (&task_list[0]))/(sizeof(TaskData)))
__attribute__((used)) static volatile TaskData  *task_list_head                 = NULL;
__attribute__((used)) static volatile TaskData  *current_running_task           = NULL;

__attribute__((used)) static volatile uint8_t   task_list_write_pos             = 0;
__attribute__((used)) static volatile uint8_t   current_num_tasks               = 0;
__attribute__((used)) static volatile uint8_t   num_tasks_allocated             = 0;

__attribute__((used)) static volatile TaskData  *ready_task_list[MAX_NUM_TASKS];
__attribute__((used)) static volatile uint8_t   num_ready_tasks                 = 0;

__attribute__((used)) static volatile uint8_t   task_stack_space[MAX_NUM_TASKS][TASK_STACK_SIZE];

// NOTE: handle time_of_service overflow eventually
typedef struct {
    const    TaskData   *task;
    volatile uint64_t   time_of_service;
    volatile TaskTimer  *nextTaskTimerPtr;

} TaskTimer;

__attribute__((used)) static volatile TaskTimer task_timer_queue[MAX_NUM_TASKS];
__attribute__((used)) static volatile TaskTimer *task_timer_queue_head          = NULL;
__attribute__((used)) static volatile uint8_t   task_timer_queue_write_pos      = 0;
__attribute__((used)) static volatile uint8_t   num_task_timers                 = 0;

static void idle_task(void);

static inline void load_ready_task_list();

static inline void add_task(const void *funcPtr, uint8_t priority);

static inline void delete_task(const void *funcPtr);

static inline void set_current_task_state(TaskState state);

static inline void set_task_state(TaskData *taskPtr, TaskState state);

static inline void set_task_state_from_func(const void *funcPtr, TaskState state);

static inline void delay_current_task(uint64_t ticks_to_wait);

__attribute__((naked)) void _on_scheduler_invoked(void);

static inline void scheduler_init();
