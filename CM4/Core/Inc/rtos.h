#ifndef RTOS_H
#define RTOS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RTOS_MAX_TASKS          4U
#define RTOS_WAIT_FOREVER       0xFFFFFFFFUL

typedef enum {
    RTOS_PRIO_IDLE   = 0,
    RTOS_PRIO_LOW    = 1,
    RTOS_PRIO_NORMAL = 2,
    RTOS_PRIO_HIGH   = 3
} rtos_prio_t;

typedef void (*rtos_task_func_t)(void *arg);

typedef struct rtos_tcb {
    volatile uint32_t *sp;          /* Offset 0: Process Stack Pointer (must be first member) */
    uint32_t           wake_tick;   /* Tick when delayed/blocked task wakes up */
    void              *wait_obj;    /* Semaphore or mutex the task is waiting on */
    uint8_t            prio;        /* Task priority */
    uint8_t            state;       /* Ready, Blocked, Delayed */
    const char        *name;        /* Task name for debugging */
} rtos_tcb_t;

typedef struct {
    volatile uint32_t count;
} rtos_sem_t;

typedef struct {
    volatile uint8_t locked;
} rtos_mutex_t;

/**
 * @brief Initialize the RTOS kernel structures
 */
void rtos_init(void);

/**
 * @brief Create a statically allocated RTOS task
 * @param tcb        Pointer to static TCB structure
 * @param name       Task name string
 * @param func       Task entry function
 * @param arg        Optional argument passed to func
 * @param stack      Pointer to static uint32_t stack array
 * @param stack_words Number of 32-bit words in stack array
 * @param prio       Task priority (RTOS_PRIO_LOW .. RTOS_PRIO_HIGH)
 */
void rtos_task_create_static(rtos_tcb_t *tcb,
                             const char *name,
                             rtos_task_func_t func,
                             void *arg,
                             uint32_t *stack,
                             size_t stack_words,
                             rtos_prio_t prio);

/**
 * @brief Start the preemptive RTOS scheduler (does not return)
 */
void rtos_start(void);

/**
 * @brief Check if the RTOS scheduler is currently running
 */
bool rtos_is_running(void);

/**
 * @brief Delay the calling task for the specified number of milliseconds
 *        and yield the CPU to other ready tasks
 */
void rtos_delay(uint32_t ms);

/**
 * @brief Initialize a binary/counting semaphore
 */
void rtos_sem_init(rtos_sem_t *sem, uint32_t initial_count);

/**
 * @brief Signal a semaphore from an Interrupt Service Routine (ISR) or thread
 *        Immediately preempts lower-priority task if a higher-priority task wakes
 */
void rtos_sem_give_from_isr(rtos_sem_t *sem);

/**
 * @brief Wait for a semaphore to be signaled
 * @param sem        Pointer to semaphore
 * @param timeout_ms Timeout in ms, or RTOS_WAIT_FOREVER
 * @return true if semaphore was acquired, false on timeout
 */
bool rtos_sem_take(rtos_sem_t *sem, uint32_t timeout_ms);

/**
 * @brief Initialize a mutex
 */
void rtos_mutex_init(rtos_mutex_t *mutex);

/**
 * @brief Lock a mutex (blocks calling task until available)
 */
void rtos_mutex_lock(rtos_mutex_t *mutex);

/**
 * @brief Unlock a mutex and wake highest-priority waiting task
 */
void rtos_mutex_unlock(rtos_mutex_t *mutex);

/**
 * @brief Called every 1 ms from SysTick_Handler
 */
void rtos_tick_isr(void);

#ifdef __cplusplus
}
#endif

#endif /* RTOS_H */

