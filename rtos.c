#include "rtos.h"

static void idle_task(void) {
    for (;;) {}
}

static inline void load_ready_task_list() {
    TaskData *t1 = task_list_head;

    // get the first ready task
    while (t1 && t1->state != TASK_READY) { t1 = t1->nextTaskDataPtr; }

    // there is at least one ready task
    if (t1) {
        // load ready tasks with the same priority onto the queue
        TaskData *t2 = t1;
        uint8_t nrt = 0; // num ready tasks
        while (t2 && t2->priority == t1->priority) {
            if (t2->state == TASK_READY) {
                ready_task_list[num_ready_tasks] = t2;
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


static inline void add_task(const void *funcPtr, uint8_t priority) {
    // task list is full
    if (current_num_tasks >= MAX_NUM_TASKS) return;

    __asm__("cpsid if");

    // write the task data into the list
    TaskData *new_task = &task_list[task_list_write_pos];

    *new_task = (TaskData) { 
        funcPtr, &task_stack_space[task_list_write_pos][TASK_STACK_SIZE-16], 
        TASK_READY, priority, NULL, 0xFF 
    };

    // copy the function pointer into the new task's stack space so that
    // it's retreived while context switching (funcPtr becomes PC)
    task_stack_space[task_list_write_pos][TASK_STACK_SIZE-2] = funcPtr;

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
    if (task_list_write_pos == num_tasks_allocated) {
        num_tasks_allocated++;
    }

    // there is still space at the end of the task_list to write more task data
    if (num_tasks_allocated < MAX_NUM_TASKS) {
        task_list_write_pos = num_tasks_allocated;
    }

    // if there is no space at the end of task_list,
    // we have to find a spot that doesn't have task data
    else {
        do {
            task_list_write_pos = (task_list_write_pos + 1) % MAX_NUM_TASKS;
        } while ( task_list[task_list_write_pos].dataPresent );
    }

    load_ready_task_list();

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
            task_list_write_pos = GET_TASK_LIST_INDEX_FROM_MEMORY_LOCATION(t2);

            load_ready_task_list();
            break;
        }
        else {
            t1 = t2;
            t2 = t2->nextTaskDataPtr;
        }
    }

    __asm__("cpsie if");
}

static inline void set_current_task_state(TaskState state) {
    current_running_task->state = state;
    load_ready_task_list();
}

static inline void set_task_state(TaskData *taskPtr, TaskState state) {
    taskPtr->state = state;
    load_ready_task_list();
}

static inline void set_task_state_from_func(const void *funcPtr, TaskState state) {
    TaskData *t1 = task_list_head;

    while (t1 && t1->funcPtr == funcPtr) t1 = t1->nextTaskDataPtr;

    if (t1) t1->state == state;

    load_ready_task_list();
}

static inline void delay_current_task(uint64_t ticks_to_wait) {
    if (num_task_timers >= MAX_NUM_TASKS) return;

    __asm__("cpsid if");

    // add a new timer to the timer queue
    uint64_t time_of_service = tick_count + ticks_to_wait;

    TaskData *task = current_running_task;

    // set the task state to blocked
    set_task_state_direct(task, TASK_BLOCKED);

    TaskTimer *new_timer = &task_timer_queue[task_timer_queue_write_pos];

    *new_timer = (TaskTimer) { 
        task, time_of_service, NULL
    };

    // if new_timer is not the only timer in the queue 
    if (num_task_timers > 0) {
        // change the pointers so the task is inserted into the linked list
        TaskTimer    *t1 = NULL;
        TaskTimer    *t2 = task_timer_queue_head;

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
    task_timer_queue_write_pos = (task_timer_queue_write_pos + 1) % MAX_NUM_TASKS;

    __asm__("cpsie if");
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

    __asm__(
        "CPSID  IF;"
        
        // skip everything if scheduler isn't enabled
        "LDR    R2, =scheduler_enabled;"
        "LDR    R1, [R2];"
        "CMP    R1, #0;"
        "BEQ    _on_scheduler_invoked_skip_1;"

        // push remaining context
        "PUSH    {R4-R7};"
        "MOV     R4, R8;"
        "MOV     R5, R9;"
        "MOV     R6, R10;"
        "MOV     R7, R11;"
        "PUSH    {R4-R7};"

        // if there are no delayed tasks, skip to choosing the next
        // task to run
        "LDR    R2, =num_task_timers;"
        "LDR    R1, [R2];"
        "CMP    R1, #0;"
        "BEQ    _on_scheduler_invoked_skip_3;"

        // if tick_count >= task_timer_queue_head->time_of_service,
        // unblock the delayed task
        // R3, R4 = tick_count
        "LDR    R2, =tick_count;"
        "LDR    R3, [R2];"
        "LDR    R4, [R2, #4];"

        // R2 = task_timer_queue_head
        // R5, R6 = task_timer_queue_head->time_of_service
        "LDR    R2, =task_timer_queue_head;"
        "LDR    R2, [R2];"
        "LDR    R5, [R2, #4];"
        "LDR    R6, [R2, #8];"

        "CMP    R4, R6;"
        "BLT    _on_scheduler_invoked_skip_3;"
        "BGT    _on_scheduler_invoked_skip_4;"

        "CMP    R3, R5;"
        "BLT    _on_scheduler_invoked_skip_3;"

        "_on_scheduler_invoked_skip_4:"

        // set the task at the head of the timer queue to ready state
        // and add it to ready_task_list if applicable
        // R0 = task_timer_queue_head->task
        // R1 = 0 = TASK_READY
        "MOV    R0, R2;"
        "LDR    R0, [R0];"
        "MOV    R1, #0;"
        "BL     set_task_state;"

        // task_timer_queue_head = task_timer_queue_head->nextTaskDataPtr
        "LDR    R2, =task_timer_queue_head;"
        "LDR    R0, [R2];"
        "LDR    R1, [R0, #12];"
        "STR    R1, [R0];"

        // num_task_timers--
        "LDR    R2, =num_task_timers;"
        "LDR    R1, [R2];"
        "SUB    R1, #1;"
        "STR    R1, [R2];"

        "_on_scheduler_invoked_skip_3:"

        // skip choosing a new task if there are no ready tasks
        // R1 = num_ready_tasks
        "LDR    R2, =num_ready_tasks;"
        "LDR    R1, [R2];"
        "CMP    R1, #0;"
        "BEQ    _on_scheduler_invoked_skip_2;"

        // save the current stack pointer
        // R2 = address of current_running_task
        "LDR    R2, =current_running_task;"
        "LDR    R2, [R2];"
        "MOV    R0, SP;"
        "STR    R0, [R2, #4];"

        // R0 = round_robin_helper
        "LDR    R3, =round_robin_helper;"
        "LDR    R0, [R3];"
        
        // R1 = round_robin_helper % num_ready_tasks
        "BL     __aeabi_idivmod;"

        // round_robin_helper++
        "ADD    R0, R0, #1;"
        "STR    R0, [R3];"

        // R3 = address of next task
        "LDR    R3, =ready_task_list;"
        // multiply R1 by 4 since each item in queue is 4 bytes
        "LSL    R1, R1, #2;"
        "ADD    R3, R3, R1;"
        "LDR    R3, [R3];"

        // set current_running_task
        "STR    R3, [R2];"

        // R4 = new SP
        "LDR    R4, [R3, #4];"

        // set the new SP
        "MOV    SP, R4;"

        "_on_scheduler_invoked_skip_2:"

        // pop the next context
        "POP     {R4-R7};"
        "MOV     R8, R4;"
        "MOV     R9, R5;"
        "MOV     R10, R6;"
        "MOV     R11, R7;"
        "POP     {R4-R7};"

        "_on_scheduler_invoked_skip_1:"

        "CPSIE  IF;"
        "BX     LR;"
    );
}

static inline void scheduler_init() {
    //  ADD IDLE TASK
    add_task(&idle_task, 0);
    scheduler_enabled = true;

}
