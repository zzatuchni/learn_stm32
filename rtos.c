#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>

#define MAX_NUM_TASKS 8
#define TASK_STACK_SIZE 1024

typedef enum {
    TASK_READY, TASK_BLOCKED
} TaskState;

typedef struct {
    volatile uint8_t     dataPresent;

    volatile TaskState   state;
    volatile uint8_t     priority;
    const    void        (*funcPtr)(void);
    const    void        *stackPtr;

    volatile TaskData    *nextTaskDataPtr;

} TaskData;

__attribute__((used)) static volatile bool      scheduler_enabled = false;
__attribute__((used)) static volatile TaskData  task_list[MAX_NUM_TASKS];
#define GET_TASK_LIST_INDEX_FROM_MEMORY_LOCATION(x) (((x) - (&task_list[0]))/(sizeof(TaskData)))
__attribute__((used)) static volatile TaskData  *start_of_task_list_ptr = NULL;

__attribute__((used)) static volatile uint8_t   next_place_to_write_task_data = 0;
__attribute__((used)) static volatile uint8_t   current_num_tasks = 0;
__attribute__((used)) static volatile uint8_t   num_tasks_allocated = 0;

__attribute__((used)) static volatile TaskData  *ready_task_queue[MAX_NUM_TASKS];
__attribute__((used)) static volatile uint8_t   ready_task_queue_size = 0;
__attribute__((used)) static volatile uint8_t   round_robin_helper = 0;

__attribute__((used)) static volatile uint8_t   task_stack_space[MAX_NUM_TASKS][TASK_STACK_SIZE];

static void idle_task(void) {
    for (;;) {}
}

static inline void loadReadyTaskQueue() {
    TaskData *t1 = start_of_task_list_ptr;

    // get the first ready task
    while (t1 && t1->state != TASK_READY) { t1 = t1->nextTaskDataPtr; }

    // there is at least one ready task
    if (t1) {
        // load ready tasks with the same priority onto the queue
        TaskData *t2 = t1;
        uint8_t num_ready_tasks = 0;
        while (t2 && t2->priority == t1->priority) {
            if (t2->state == TASK_READY) {
                ready_task_queue[num_ready_tasks] == t2;
                num_ready_tasks++;
            }
        }
        ready_task_queue_size = num_ready_tasks;

    }
    else {
        ready_task_queue_size = 0;
    }

}

static inline void addTask(const void *funcPtr, uint8_t priority) {
    // task list is full
    if (current_num_tasks >= MAX_NUM_TASKS) return;

    __asm__("cpsid if");

    // write the task data into the list
    TaskData *new_task = &task_list[next_place_to_write_task_data];

    *new_task = (TaskData) { 0xFF, TASK_READY, priority, funcPtr, &task_stack_space[next_place_to_write_task_data], NULL };

    // new_task is not the only task in the list
    if (current_num_tasks > 0) {
        // change the pointers so the task is inserted into the linked list
        TaskData    *t1 = NULL;
        TaskData    *t2 = start_of_task_list_ptr;

        // if t2 has a priority lte to new_task's priority, we want
        // t1 --> new_task --> t2
        while ( t2 && t2->priority > priority ) {
            t1 = t2;
            t2 = t2->nextTaskDataPtr;
        }

        // new_task is in the middle / end of the linked list
        if (t1) t1->nextTaskDataPtr = new_task;
        // new_task is at the beginning of the linked list, i.e. it has the highest priority
        else start_of_task_list_ptr = new_task;

        // new_task is at the beginning / middle of the linked list
        if (t2) new_task->nextTaskDataPtr = t2;
    }
    // new_task IS the only task in the list
    else {
        start_of_task_list_ptr = new_task;
    }
    current_num_tasks++;

    // find the next place to write task data

    // new_task is not filling in a gap left by a deleted task, 
    // i.e. it was newly allocated at the end of the task list memory-wise
    if (next_place_to_write_task_data == num_tasks_allocated) {
        num_tasks_allocated++;
    }

    // there is still space at the end of the task_list to write more task data
    if (num_tasks_allocated < MAX_NUM_TASKS) {
        next_place_to_write_task_data = num_tasks_allocated;
    }

    // if there is no space at the end of task_list,
    // we have to find a spot that doesn't have task data
    else {
        do {
            next_place_to_write_task_data = (next_place_to_write_task_data + 1) % MAX_NUM_TASKS;
        } while ( task_list[next_place_to_write_task_data].dataPresent );
    }

    __asm__("cpsie if");
}

static inline void deleteTask(const void *funcPtr) {
    // task list is empty
    if (current_num_tasks == 0) return;
    // can't delete the idle task
    if (funcPtr == &idle_task) return;

    __asm__("cpsid if");

    // find the task to be deleted
    TaskData *t1 = NULL;
    TaskData *t2 = start_of_task_list_ptr;

    while (t2) {
        if (t2->funcPtr == funcPtr) {
            t2->dataPresent == 0x00;
            if (t1) {
                if (t2->nextTaskDataPtr) t1->nextTaskDataPtr = t2->nextTaskDataPtr; 
                else t1->nextTaskDataPtr = NULL;
            }
            current_num_tasks--;
            next_place_to_write_task_data = GET_TASK_LIST_INDEX_FROM_MEMORY_LOCATION(t2);
            break;
        }
        else {
            t1 = t2;
            t2 = t2->nextTaskDataPtr;
        }
    }

    __asm__("cpsie if");
}

static inline void setTaskState(const void *funcPtr, uint8_t priority) {
    // find the task to be deleted
    TaskData *t1 = NULL;
    TaskData *t2 = start_of_task_list_ptr;

    while (t2) {
        if (t2->funcPtr == funcPtr) {
            t2->dataPresent == 0x00;
            if (t1) {
                if (t2->nextTaskDataPtr) t1->nextTaskDataPtr = t2->nextTaskDataPtr; 
                else t1->nextTaskDataPtr = NULL;
            }
            current_num_tasks--;
            next_place_to_write_task_data = GET_TASK_LIST_INDEX_FROM_MEMORY_LOCATION(t2);
            break;
        }
        else {
            t1 = t2;
            t2 = t2->nextTaskDataPtr;
        }
    }
}


__attribute__((naked)) static inline void pushContext() {

}

__attribute__((naked)) static inline void popContext() {
    
}

__attribute__((naked)) void _on_scheduler_invoked(void) {
    if (scheduler_enabled) {
         __asm__("cpsid if");

        // there is at least one ready task
        if (ready_task_queue_size) {
            pushContext();

            TaskData *nextTask = ready_task_queue[round_robin_helper % ready_task_queue_size];
            round_robin_helper++;

            // SET THE SP TO THE NEW TASK 

            popContext();
        }

        __asm__("cpsie if");
    }
}

static inline void scheduler_init() {
    //  ADD IDLE TASK
    addTask(&idle_task, 0);

}

