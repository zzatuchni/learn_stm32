#include "rtos.h"

__attribute__((used)) volatile uint8_t   isr_lock                           = 0;

__attribute__((used)) volatile bool      scheduler_enabled                  = false;
__attribute__((used)) volatile uint64_t  tick_count                         = 0;
__attribute__((used)) volatile uint32_t  round_robin_helper                 = 0;

__attribute__((used)) volatile TaskData  task_list[MAX_NUM_TASKS];
__attribute__((used)) volatile TaskData  *task_list_head                    = NULL;
__attribute__((used)) volatile TaskData  *current_running_task              = NULL;

__attribute__((used)) volatile uint8_t   task_list_write_pos                = 0;
__attribute__((used)) volatile uint8_t   current_num_tasks                  = 0;
__attribute__((used)) volatile uint8_t   num_tasks_allocated                = 0;

__attribute__((used)) volatile TaskData  *ready_task_list[MAX_NUM_TASKS];
__attribute__((used)) volatile uint8_t   num_ready_tasks                    = 0;

__attribute__((used)) volatile uint32_t  task_stack_space[MAX_NUM_TASKS][TASK_STACK_SIZE];

__attribute__((used)) volatile TaskTimer task_timer_queue[MAX_NUM_TASKS];
__attribute__((used)) volatile TaskTimer *task_timer_queue_head             = NULL;
__attribute__((used)) volatile uint8_t   task_timer_queue_write_pos         = 0;
__attribute__((used)) volatile uint8_t   num_task_timers                    = 0;

void increment_isr_lock() {
    if (!isr_lock) __asm__("cpsid if");
    isr_lock++;
}

void decrement_isr_lock() {
    isr_lock--;
    if (!isr_lock) __asm__("cpsie if");
}

void idle_task(void) {
    for (;;) {}
}

void load_ready_task_list() {
    volatile TaskData *t1 = task_list_head;

    // get the first ready task
    while (t1 && t1->state != TASK_READY) { t1 = t1->nextTaskDataPtr; }

    // there is at least one ready task
    if (t1) {
        // load ready tasks with the same priority onto the queue
        volatile TaskData *t2 = t1;
        uint8_t nrt = 0; // num ready tasks
        while (t2 && t2->priority == t1->priority) {
            if (t2->state == TASK_READY) {
                ready_task_list[nrt] = t2;
                nrt++;
            }
            t2 = t2->nextTaskDataPtr;
        }
        num_ready_tasks = nrt;

    }
    else {
        num_ready_tasks = 0;
    }
}


void add_task(const void *funcPtr, uint8_t priority) {
    increment_isr_lock();

    // task list is full
    if (current_num_tasks >= MAX_NUM_TASKS) { decrement_isr_lock(); return; }

    // write the task data into the list
    volatile TaskData *new_task = &task_list[task_list_write_pos];

    *new_task = (TaskData) { 
        funcPtr, &task_stack_space[task_list_write_pos][TASK_STACK_SIZE-16], 
        TASK_READY, priority, NULL, 0xFF 
    };

    // copy the function pointer into the new task's stack space so that
    // it's retreived while context switching (funcPtr becomes PC)
    task_stack_space[task_list_write_pos][TASK_STACK_SIZE-1] = (uint32_t)0x01000000;
    task_stack_space[task_list_write_pos][TASK_STACK_SIZE-2] = (uint32_t)funcPtr;

    // new_task is not the only task in the list
    if (current_num_tasks > 0) {
        // change the pointers so the task is inserted into the linked list
        volatile TaskData    *t1 = NULL;
        volatile TaskData    *t2 = task_list_head;

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
    if (task_list_write_pos == num_tasks_allocated) {
        num_tasks_allocated++;
    }

    // there is no more space to write a task, so no use changing the write position right now
    if (current_num_tasks >= MAX_NUM_TASKS) {}

    // there is still space at the end of the task_list to write more task data
    else if (num_tasks_allocated < MAX_NUM_TASKS) {
        task_list_write_pos = num_tasks_allocated;
    }

    // if there is no space at the end of task_list,
    // we have to find a spot that doesn't have task data
    else {
        do {
            task_list_write_pos = (uint8_t)((task_list_write_pos + 1U) % ((uint8_t) MAX_NUM_TASKS));
        } while ( task_list[task_list_write_pos].dataPresent );
    }

    load_ready_task_list();

    decrement_isr_lock();
}

void delete_task(const void *funcPtr) {
    increment_isr_lock();

    // task list is empty
    // can't delete the idle task
    if (current_num_tasks == 0 || funcPtr == &idle_task) {
        decrement_isr_lock(); return;
    }

    // find the task to be deleted
    volatile TaskData *t1 = NULL;
    volatile TaskData *t2 = task_list_head;

    while (t2) {
        if (t2->funcPtr == funcPtr) {
            t2->dataPresent = 0x00;
            if (t1) {
                if (t2->nextTaskDataPtr) t1->nextTaskDataPtr = t2->nextTaskDataPtr; 
                else t1->nextTaskDataPtr = NULL;
            }
            current_num_tasks--;
            task_list_write_pos = (uint8_t)GET_TASK_LIST_INDEX_FROM_MEMORY_LOCATION(t2);

            load_ready_task_list();
            break;
        }
        else {
            t1 = t2;
            t2 = t2->nextTaskDataPtr;
        }
    }

    decrement_isr_lock();
}

void set_current_task_state(TaskState state) {
    increment_isr_lock();
    volatile TaskData *t1 = current_running_task;
    t1->state = state;
    load_ready_task_list();
    decrement_isr_lock();

    while ( t1->state == TASK_BLOCKED ) {}
}

void set_task_state(TaskData *taskPtr, TaskState state) {
    increment_isr_lock();
    taskPtr->state = state;
    load_ready_task_list();
    decrement_isr_lock();
}

void set_task_state_from_func(const void *funcPtr, TaskState state) {
    increment_isr_lock();
    volatile TaskData *t1 = task_list_head;

    while (t1 && t1->funcPtr != funcPtr) t1 = t1->nextTaskDataPtr;

    if (t1) t1->state = state;

    load_ready_task_list();
    decrement_isr_lock();
}

void delay_current_task(uint64_t ticks_to_wait) {
    increment_isr_lock();

    if (num_task_timers >= MAX_NUM_TASKS) { decrement_isr_lock(); return; }

    // add a new timer to the timer queue
    uint64_t time_of_service = tick_count + ticks_to_wait;

    TaskData *task = (TaskData *) current_running_task;

    volatile TaskTimer *new_timer = &task_timer_queue[task_timer_queue_write_pos];

    *new_timer = (TaskTimer) { 
        task, time_of_service, NULL
    };

    // if new_timer is not the only timer in the queue 
    if (num_task_timers > 0) {
        // change the pointers so the task is inserted into the linked list
        volatile TaskTimer    *t1 = NULL;
        volatile TaskTimer    *t2 = task_timer_queue_head;

        // if t2 has a time_of_service gte to new_times's time_of_service, we want
        // t1 --> new_task --> t2
        while ( t2 && t2->time_of_service < time_of_service ) {
            t1 = t2;
            t2 = t2->nextTaskTimerPtr;
        }

        // new_timer is in the middle / end of the linked list
        if (t1) t1->nextTaskTimerPtr = new_timer;
        // new_timer is at the beginning of the linked list, i.e. it has the highest priority
        else task_timer_queue_head = new_timer;

        // new_timer is at the beginning / middle of the linked list
        if (t2) new_timer->nextTaskTimerPtr = t2;
    }
    else {
        task_timer_queue_head = new_timer;
    }
    num_task_timers++;

    // set the next place to write task data
    task_timer_queue_write_pos = (uint8_t)((task_timer_queue_write_pos + 1U) % ((uint8_t) MAX_NUM_TASKS));

    // set the task state to blocked
    set_task_state(task, TASK_BLOCKED);

    decrement_isr_lock();

    while ( task->state == TASK_BLOCKED ) {}
}

__attribute__((naked)) void _on_scheduler_invoked(void) {

    /*
    if (scheduler_enabled) {    
        if (
            num_task_timers && 
            tick_count > task_timer_queue_head->time_of_service
        ) {
            set delayed task state to ready and recalculate ready queue
        }
        if (num_ready_tasks)
            next_task = ready_task_list[round_robin_helper % num_ready_tasks];
        else
            next_task = current_task
    }
    */

    __asm__("CPSID  IF;");

    // increment isr_lock (should be 0)
    __asm__("LDR    R2, =isr_lock;");
    __asm__("LDRB   R1, [R2];");
    __asm__("ADD    R1, R1, #1;");
    __asm__("STRB   R1, [R2];");

    // skip everything if scheduler isn't enabled
    __asm__("LDR    R2, =scheduler_enabled;");
    __asm__("LDRB   R1, [R2];");
    __asm__("CMP    R1, #0;");
    __asm__("BEQ    _on_scheduler_invoked_skip_1;");

    // push remaining context
    __asm__("PUSH    {R4-R7};");
    __asm__("MOV     R4, R8;");
    __asm__("MOV     R5, R9;");
    __asm__("MOV     R6, R10;");
    __asm__("MOV     R7, R11;");
    __asm__("PUSH    {R4-R7};");

    __asm__("MOV    R7, LR");

    // if tick_count >= task_timer_queue_head->time_of_service,
    // unblock the delayed task
    // R3, R4 = tick_count
    __asm__("LDR    R2, =tick_count;");
    __asm__("LDR    R3, [R2];");
    __asm__("LDR    R4, [R2, #4];");

    // if there are no delayed tasks, go right to selecting the next task
    // from the ready tasks
    __asm__("LDR    R2, =num_task_timers;");
    __asm__("LDRB   R1, [R2];");
    __asm__("CMP    R1, #0;");
    __asm__("BEQ    _on_scheduler_invoked_skip_3;");

    // R2 = task_timer_queue_head
    // R5, R6 = task_timer_queue_head->time_of_service
    __asm__("LDR    R2, =task_timer_queue_head;");
    __asm__("LDR    R2, [R2];");
    __asm__("LDR    R5, [R2, #8];");
    __asm__("LDR    R6, [R2, #12];");

    __asm__("CMP    R4, R6;");
    __asm__("BLT    _on_scheduler_invoked_skip_3;");
    __asm__("BGT    _on_scheduler_invoked_skip_4;");

    __asm__("CMP    R3, R5;");
    __asm__("BLT    _on_scheduler_invoked_skip_3;");

    __asm__("_on_scheduler_invoked_skip_4:");

    // set the task at the head of the timer queue to ready state
    // and add it to ready_task_list if applicable
    // R0 = task_timer_queue_head->task
    // R1 = 0 = TASK_READY
    __asm__("MOV    R0, R2;");
    __asm__("LDR    R0, [R0];");
    __asm__("MOV    R1, #0;");
    __asm__("BL     set_task_state;");

    // task_timer_queue_head = task_timer_queue_head->nextTaskDataPtr
    __asm__("LDR    R2, =task_timer_queue_head;");
    __asm__("LDR    R0, [R2];");
    __asm__("LDR    R1, [R0, #16];");
    __asm__("STR    R1, [R0];");

    // num_task_timers--
    __asm__("LDR    R2, =num_task_timers;");
    __asm__("LDRB   R1, [R2];");
    __asm__("SUB    R1, #1;");
    __asm__("STRB   R1, [R2];");

    __asm__("_on_scheduler_invoked_skip_3:");

    // tick_count++
    __asm__("ADDS   R3, R3, #1;");
    __asm__("ADC    R4, R4, #0;");
    __asm__("LDR    R2, =tick_count;");
    __asm__("STR    R3, [R2];");
    __asm__("STR    R4, [R2, #4];");

    // skip choosing a new task if there are no ready tasks
    // R1 = num_ready_tasks
    __asm__("LDR    R2, =num_ready_tasks;");
    __asm__("LDRB   R1, [R2];");
    __asm__("CMP    R1, #0;");
    __asm__("BEQ    _on_scheduler_invoked_skip_2;");

    // save the current stack pointer
    // R2 = address of current_running_task
    __asm__("LDR    R5, =current_running_task;");
    __asm__("LDR    R2, [R5];");

    // check if there is no current running task yet
    __asm__("CMP    R2, #0;");
    __asm__("BEQ    _on_scheduler_invoked_skip_5;");

    __asm__("MOV    R0, SP;");
    __asm__("STR    R0, [R2, #4];");

    __asm__("_on_scheduler_invoked_skip_5:");

    // R0 = round_robin_helper
    __asm__("LDR    R3, =round_robin_helper;");
    __asm__("LDR    R0, [R3];");
    
    // R1 = round_robin_helper % num_ready_tasks
    __asm__("BL     __aeabi_idivmod;");

    // round_robin_helper++
    __asm__("ADD    R0, R0, #1;");
    __asm__("STR    R0, [R3];");

    // R3 = address of next task
    __asm__("LDR    R3, =ready_task_list;");
    // multiply R1 by 4 since each item in queue is 4 bytes
    __asm__("LSL    R1, R1, #2;");
    __asm__("ADD    R3, R3, R1;");
    __asm__("LDR    R3, [R3];");

    // set current_running_task
    __asm__("STR    R3, [R5];");

    // R4 = new SP
    __asm__("LDR    R4, [R3, #4];");

    // set the new SP
    __asm__("MOV    SP, R4;");

    __asm__("_on_scheduler_invoked_skip_2:");

    __asm__("MOV    LR, R7");

    // pop the next context
    __asm__("POP     {R4-R7};");
    __asm__("MOV     R8, R4;");
    __asm__("MOV     R9, R5;");
    __asm__("MOV     R10, R6;");
    __asm__("MOV     R11, R7;");
    __asm__("POP     {R4-R7};");

    __asm__("_on_scheduler_invoked_skip_1:");

    // decrement isr_lock (should be 1)
    __asm__("LDR    R2, =isr_lock;");
    __asm__("LDRB   R1, [R2];");
    __asm__("SUB    R1, R1, #1;");
    __asm__("STRB   R1, [R2];");

    __asm__("CPSIE  IF;");
    __asm__("BX     LR;");
}

inline void scheduler_init() {
    //  ADD IDLE TASK
    increment_isr_lock();
    add_task(&idle_task, 0);
    scheduler_enabled = true;
    decrement_isr_lock();
}
