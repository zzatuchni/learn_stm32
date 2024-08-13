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
    const    void        (*funcPtr)(void);

    volatile void        *stackPtr;
    volatile TaskState   state;
    volatile uint8_t     priority;
    volatile TaskData    *nextTaskDataPtr;

    volatile uint8_t     dataPresent;

} TaskData;

__attribute__((used)) static volatile bool      scheduler_enabled = false;

__attribute__((used)) static volatile TaskData  task_list[MAX_NUM_TASKS];
#define GET_TASK_LIST_INDEX_FROM_MEMORY_LOCATION(x) (((x) - (&task_list[0]))/(sizeof(TaskData)))
__attribute__((used)) static volatile TaskData  *task_list_head                 = NULL;
__attribute__((used)) static volatile TaskData  *current_running_task           = NULL;

__attribute__((used)) static volatile uint8_t   next_place_to_write_task_data   = 0;
__attribute__((used)) static volatile uint8_t   current_num_tasks               = 0;
__attribute__((used)) static volatile uint8_t   num_tasks_allocated             = 0;

__attribute__((used)) static volatile TaskData  *ready_task_queue[MAX_NUM_TASKS];
__attribute__((used)) static volatile uint8_t   ready_task_queue_size           = 0;
__attribute__((used)) static volatile uint8_t   round_robin_helper              = 0;

__attribute__((used)) static volatile uint8_t   task_stack_space[MAX_NUM_TASKS][TASK_STACK_SIZE];

static void idle_task(void) {
    for (;;) {}
}

static inline void load_ready_task_queue() {
    TaskData *t1 = task_list_head;

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
            t2 = t2->nextTaskDataPtr;
        }
        ready_task_queue_size = num_ready_tasks;

    }
    else {
        ready_task_queue_size = 0;
    }
}

static inline void add_task(const void *funcPtr, uint8_t priority) {
    // task list is full
    if (current_num_tasks >= MAX_NUM_TASKS) return;

    __asm__("cpsid if");

    // write the task data into the list
    TaskData *new_task = &task_list[next_place_to_write_task_data];

    *new_task = (TaskData) { 
        funcPtr, &task_stack_space[next_place_to_write_task_data][TASK_STACK_SIZE-16], 
        TASK_READY, priority, NULL, 0xFF 
    };

    // copy the function pointer into the new task's stack space so that
    // it's retreived while context switching (funcPtr becomes PC)
    task_stack_space[next_place_to_write_task_data][TASK_STACK_SIZE-2] = funcPtr;

    // new_task is not the only task in the list
    if (current_num_tasks > 0) {
        // change the pointers so the task is inserted into the linked list
        TaskData    *t1 = NULL;
        TaskData    *t2 = task_list_head;

        // if t2 has a priority lte to new_task's priority, we want
        // t1 --> new_task --> t2
        while ( t2 && t2->priority > priority ) {
            t1 = t2;
            t2 = t2->nextTaskDataPtr;
        }

        // new_task is in the middle / end of the linked list
        if (t1) t1->nextTaskDataPtr = new_task;
        // new_task is at the beginning of the linked list, i.e. it has the highest priority
        else task_list_head = new_task;

        // new_task is at the beginning / middle of the linked list
        if (t2) new_task->nextTaskDataPtr = t2;
    }
    // new_task IS the only task in the list
    else {
        task_list_head = new_task;
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

    // if the new task also has the highest priority,
    // load into the ready task queue
    if (new_task->priority == task_list_head->priority) {
        load_ready_task_queue();
    }

    __asm__("cpsie if");
}

static inline void delete_task(const void *funcPtr) {
    // task list is empty
    if (current_num_tasks == 0) return;
    // can't delete the idle task
    if (funcPtr == &idle_task) return;

    __asm__("cpsid if");

    // find the task to be deleted
    TaskData *t1 = NULL;
    TaskData *t2 = task_list_head;

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

static inline void set_task_state(const void *funcPtr, uint8_t priority) {
    // find the task to be deleted
    TaskData *t1 = NULL;
    TaskData *t2 = task_list_head;

    while (t2) {
        if (t2->funcPtr == funcPtr) {
            t2->dataPresent == 0x00;
            if (t1) {
                if (t2->nextTaskDataPtr) t1->nextTaskDataPtr = t2->nextTaskDataPtr; 
                else t1->nextTaskDataPtr = NULL;
            }
            current_num_tasks--;
            next_place_to_write_task_data = GET_TASK_LIST_INDEX_FROM_MEMORY_LOCATION(t2);

            // if the task has the highest priority,
            // load into the ready task queue
            if (t2->priority == task_list_head->priority) {
                load_ready_task_queue();
            }
            break;
        }
        else {
            t1 = t2;
            t2 = t2->nextTaskDataPtr;
        }
    }
}

static inline void delay_task(const void *funcPtr, uint32_t ticks_to_wait) {

}

__attribute__((naked)) void _on_scheduler_invoked(void) {

    /*
    if (scheduler_enabled && ready_task_queue_size) {    

        push_context();

        current_running_task = ready_task_queue[round_robin_helper % ready_task_queue_size];
        round_robin_helper++;

        // SET THE SP TO THE NEW TASK 

        pop_context();
    }
    */

    __asm__(
        "CPSID  IF;"
        
        // skip if scheduler isn't enabled or if
        // there are no ready tasks
        "LDR    R2, =scheduler_enabled;"
        "LDR    R1, [R2];"
        "CMP    R1, #0;"
        "BEQ    _on_scheduler_invoked_end;"

        "LDR    R2, =ready_task_queue_size;"
        "LDR    R1, [R2];"
        "CMP    R1, #0;"
        "BEQ    _on_scheduler_invoked_end;"

        // push remaining context
        "PUSH    {R4-R7};"
        "MOV     R4, R8;"
        "MOV     R5, R9;"
        "MOV     R6, R10;"
        "MOV     R7, R11;"
        "PUSH    {R4-R7};"

        // save the current stack pointer
        // R2 = address of current_running_task
        "LDR    R2, =current_running_task;"
        "MOV    R0, SP"
        "STR    R0, [R2, #4]"

        // R0 = round_robin_helper
        // R1 = ready_task_queue_size
        "LDR    R3, =round_robin_helper;"
        "LDR    R0, [R3];"
        
        // R1 = round_robin_helper % ready_task_queue_size
        "BL     __aeabi_idivmod;"

        // round_robin_helper++
        "ADD    R0, R0, #1;"
        "STR    R0, [R3];"

        // R3 = address of next task
        "LDR    R3, =ready_task_queue;"
        // multiply R1 by 4 since each item in queue is 4 bytes
        "LSL    R1, R1, #2"
        "ADD    R3, R3, R1;"
        "LDR    R3, [R3];"

        // set current_running_task
        "STR    R3, [R2]"

        // R4 = new SP
        "LDR    R4, [R3, #4]"

        // set the new SP
        "MOV    SP, R4"

        // pop the next context
        "POP     {R4-R7};"
        "MOV     R8, R4;"
        "MOV     R9, R5;"
        "MOV     R10, R6;"
        "MOV     R11, R7;"
        "POP     {R4-R7};"

        "_on_scheduler_invoked_end:"

        "CPSIE  IF;"
        "BX     LR;"
    );
}

static inline void scheduler_init() {
    //  ADD IDLE TASK
    add_task(&idle_task, 0);

}

