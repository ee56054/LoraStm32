#include "rtos.h"
#include "main.h"

#define TASK_READY      0U
#define TASK_DELAYED    1U
#define TASK_WAIT_SEM   2U
#define TASK_WAIT_MUTEX 3U

/* Global pointers accessed from naked PendSV_Handler and SVC_Handler */
volatile rtos_tcb_t *g_current_tcb = NULL;
volatile rtos_tcb_t *g_next_tcb = NULL;

static rtos_tcb_t *s_tasks[RTOS_MAX_TASKS];
static uint8_t s_task_count = 0;
static volatile bool s_scheduler_running = false;

/* Static Idle Task resources */
static rtos_tcb_t s_idle_tcb;
static uint32_t s_idle_stack[128] __attribute__((aligned(8)));

static void rtos_idle_task(void *arg) {
    (void)arg;
    while (1) {
        __WFI();
    }
}

static void rtos_task_exit(void) {
    while (1) {
        __WFI();
    }
}

static void rtos_schedule_locked(void) {
    if (!s_scheduler_running) {
        return;
    }

    rtos_tcb_t *best = &s_idle_tcb;
    for (uint8_t i = 0; i < s_task_count; i++) {
        rtos_tcb_t *t = s_tasks[i];
        if (t->state == TASK_READY && t->prio >= best->prio) {
            best = t;
        }
    }

    g_next_tcb = best;
    if (g_next_tcb != g_current_tcb) {
        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    }
}

void rtos_init(void) {
    s_task_count = 0;
    s_scheduler_running = false;
    g_current_tcb = NULL;
    g_next_tcb = NULL;

    rtos_task_create_static(&s_idle_tcb,
                            "Idle",
                            rtos_idle_task,
                            NULL,
                            s_idle_stack,
                            sizeof(s_idle_stack) / sizeof(uint32_t),
                            RTOS_PRIO_IDLE);
}

void rtos_task_create_static(rtos_tcb_t *tcb,
                             const char *name,
                             rtos_task_func_t func,
                             void *arg,
                             uint32_t *stack,
                             size_t stack_words,
                             rtos_prio_t prio) {
    if (tcb == NULL || func == NULL || stack == NULL || stack_words < 32 || s_task_count >= RTOS_MAX_TASKS) {
        return;
    }

    /* Align top of stack to 8-byte boundary per ARM AAPCS */
    uintptr_t top_addr = ((uintptr_t)(stack + stack_words)) & ~((uintptr_t)0x07);
    uint32_t *sp = (uint32_t *)top_addr;

    /* Reserve 16 words for initial Cortex-M4 context frame */
    sp -= 16;

    /* Software-saved registers (R4..R11) */
    sp[0]  = 0x04040404UL; /* R4 */
    sp[1]  = 0x05050505UL; /* R5 */
    sp[2]  = 0x06060606UL; /* R6 */
    sp[3]  = 0x07070707UL; /* R7 */
    sp[4]  = 0x08080808UL; /* R8 */
    sp[5]  = 0x09090909UL; /* R9 */
    sp[6]  = 0x10101010UL; /* R10 */
    sp[7]  = 0x11111111UL; /* R11 */

    /* Hardware-saved exception frame (R0..R3, R12, LR, PC, xPSR) */
    sp[8]  = (uint32_t)arg;             /* R0 */
    sp[9]  = 0x01010101UL;              /* R1 */
    sp[10] = 0x02020202UL;              /* R2 */
    sp[11] = 0x03030303UL;              /* R3 */
    sp[12] = 0x12121212UL;              /* R12 */
    sp[13] = (uint32_t)rtos_task_exit;  /* LR */
    sp[14] = (uint32_t)func;            /* PC */
    sp[15] = 0x01000000UL;              /* xPSR (Thumb bit set) */

    tcb->sp = sp;
    tcb->wake_tick = 0;
    tcb->wait_obj = NULL;
    tcb->prio = (uint8_t)prio;
    tcb->state = TASK_READY;
    tcb->name = name;

    s_tasks[s_task_count++] = tcb;
}

bool rtos_is_running(void) {
    return s_scheduler_running;
}

void rtos_start(void) {
    __disable_irq();

    /* Select highest priority ready task to start first */
    rtos_tcb_t *best = &s_idle_tcb;
    for (uint8_t i = 0; i < s_task_count; i++) {
        rtos_tcb_t *t = s_tasks[i];
        if (t->state == TASK_READY && t->prio >= best->prio) {
            best = t;
        }
    }
    g_current_tcb = best;
    g_next_tcb = best;

    /* Set PendSV to lowest priority (15) and SysTick to 14 */
    NVIC_SetPriority(PendSV_IRQn, 0x0FUL);
    NVIC_SetPriority(SysTick_IRQn, 0x0EUL);

    s_scheduler_running = true;
    __enable_irq();

    /* Launch first task via SVC exception */
    __asm volatile("svc 0");

    while (1) {
    }
}

void rtos_delay(uint32_t ms) {
    if (!s_scheduler_running || ms == 0) {
        return;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    g_current_tcb->wake_tick = HAL_GetTick() + ms;
    g_current_tcb->state = TASK_DELAYED;
    rtos_schedule_locked();

    __set_PRIMASK(primask);
    __ISB();
}

void rtos_sem_init(rtos_sem_t *sem, uint32_t initial_count) {
    if (sem != NULL) {
        sem->count = initial_count;
    }
}

void rtos_sem_give_from_isr(rtos_sem_t *sem) {
    if (sem == NULL) {
        return;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    rtos_tcb_t *best_waiter = NULL;
    for (uint8_t i = 0; i < s_task_count; i++) {
        rtos_tcb_t *t = s_tasks[i];
        if (t->state == TASK_WAIT_SEM && t->wait_obj == (void *)sem) {
            if (best_waiter == NULL || t->prio >= best_waiter->prio) {
                best_waiter = t;
            }
        }
    }

    if (best_waiter != NULL) {
        best_waiter->wait_obj = NULL;
        best_waiter->state = TASK_READY;
        rtos_schedule_locked();
    } else {
        sem->count = 1U;
    }

    __set_PRIMASK(primask);
}

bool rtos_sem_take(rtos_sem_t *sem, uint32_t timeout_ms) {
    if (sem == NULL) {
        return false;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (sem->count > 0) {
        sem->count--;
        __set_PRIMASK(primask);
        return true;
    }

    if (!s_scheduler_running || timeout_ms == 0) {
        __set_PRIMASK(primask);
        return false;
    }

    g_current_tcb->wait_obj = (void *)sem;
    g_current_tcb->wake_tick = (timeout_ms == RTOS_WAIT_FOREVER)
                                   ? RTOS_WAIT_FOREVER
                                   : (HAL_GetTick() + timeout_ms);
    g_current_tcb->state = TASK_WAIT_SEM;
    rtos_schedule_locked();

    __set_PRIMASK(primask);
    __ISB();

    /* If wait_obj was cleared by rtos_sem_give_from_isr, acquisition succeeded */
    bool acquired = (g_current_tcb->wait_obj == NULL);
    g_current_tcb->wait_obj = NULL;
    return acquired;
}

void rtos_mutex_init(rtos_mutex_t *mutex) {
    if (mutex != NULL) {
        mutex->locked = 0U;
    }
}

void rtos_mutex_lock(rtos_mutex_t *mutex) {
    if (mutex == NULL || !s_scheduler_running) {
        return;
    }

    while (1) {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();

        if (mutex->locked == 0U) {
            mutex->locked = 1U;
            __set_PRIMASK(primask);
            return;
        }

        g_current_tcb->wait_obj = (void *)mutex;
        g_current_tcb->state = TASK_WAIT_MUTEX;
        rtos_schedule_locked();

        __set_PRIMASK(primask);
        __ISB();
    }
}

void rtos_mutex_unlock(rtos_mutex_t *mutex) {
    if (mutex == NULL || !s_scheduler_running) {
        return;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    mutex->locked = 0U;

    rtos_tcb_t *best_waiter = NULL;
    for (uint8_t i = 0; i < s_task_count; i++) {
        rtos_tcb_t *t = s_tasks[i];
        if (t->state == TASK_WAIT_MUTEX && t->wait_obj == (void *)mutex) {
            if (best_waiter == NULL || t->prio >= best_waiter->prio) {
                best_waiter = t;
            }
        }
    }

    if (best_waiter != NULL) {
        best_waiter->wait_obj = NULL;
        best_waiter->state = TASK_READY;
        rtos_schedule_locked();
    }

    __set_PRIMASK(primask);
}

void rtos_tick_isr(void) {
    if (!s_scheduler_running) {
        return;
    }

    uint32_t now = HAL_GetTick();
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    bool woke_any = false;
    for (uint8_t i = 0; i < s_task_count; i++) {
        rtos_tcb_t *t = s_tasks[i];
        if (t->state == TASK_DELAYED) {
            if ((int32_t)(now - t->wake_tick) >= 0) {
                t->state = TASK_READY;
                woke_any = true;
            }
        } else if (t->state == TASK_WAIT_SEM && t->wake_tick != RTOS_WAIT_FOREVER) {
            if ((int32_t)(now - t->wake_tick) >= 0) {
                t->state = TASK_READY;
                woke_any = true;
            }
        }
    }

    if (woke_any) {
        rtos_schedule_locked();
    }

    __set_PRIMASK(primask);
}

__attribute__((naked)) void SVC_Handler(void) {
    __asm volatile(
        "ldr r0, =g_current_tcb   \n"
        "ldr r1, [r0]             \n"
        "ldr r0, [r1]             \n"
        "ldmia r0!, {r4-r11}      \n"
        "msr psp, r0              \n"
        "isb                      \n"
        "movs r0, #0              \n"
        "msr basepri, r0          \n"
        "ldr r14, =0xFFFFFFFD     \n"
        "bx r14                   \n"
    );
}

__attribute__((naked)) void PendSV_Handler(void) {
    __asm volatile(
        "cpsid i                  \n"
        "mrs r0, psp              \n"
        "isb                      \n"
        "ldr r1, =g_current_tcb   \n"
        "ldr r2, [r1]             \n"
        "stmdb r0!, {r4-r11}      \n"
        "str r0, [r2]             \n"
        "ldr r3, =g_next_tcb      \n"
        "ldr r2, [r3]             \n"
        "str r2, [r1]             \n"
        "ldr r0, [r2]             \n"
        "ldmia r0!, {r4-r11}      \n"
        "msr psp, r0              \n"
        "isb                      \n"
        "cpsie i                  \n"
        "bx r14                   \n"
    );
}

