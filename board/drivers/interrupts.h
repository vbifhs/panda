#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bits for pre-emption priority 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bits for pre-emption priority 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority 1 bits for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority 0 bits for subpriority */

typedef struct interrupt {
  IRQn_Type irq_type;
  void (*handler)(void);
  uint32_t call_counter;
  uint32_t call_rate;
  uint32_t max_call_rate;   // Call rate is defined as the amount of calls each second
  uint32_t call_rate_fault;
  uint32_t last_call_time;
} interrupt;

void interrupt_timer_init(void);
uint32_t microsecond_timer_get(void);

void unused_interrupt_handler(void) {
  // Something is wrong if this handler is called!
  print("Unused interrupt handler called!\n");
  fault_occurred(FAULT_UNUSED_INTERRUPT_HANDLED);
}

interrupt interrupts[NUM_INTERRUPTS];

#define REGISTER_INTERRUPT(irq_num, func_ptr, call_rate_max, rate_fault) \
  interrupts[irq_num].irq_type = (irq_num); \
  interrupts[irq_num].handler = (func_ptr);  \
  interrupts[irq_num].call_counter = 0U;   \
  interrupts[irq_num].call_rate = 0U;   \
  interrupts[irq_num].max_call_rate = (call_rate_max); \
  interrupts[irq_num].call_rate_fault = (rate_fault); \
  interrupts[irq_num].last_call_time = 0U;

bool check_interrupt_rate = false;

uint8_t interrupt_depth = 0U;
uint32_t last_time = 0U;
uint32_t idle_time = 0U;
uint32_t busy_time = 0U;
float interrupt_load = 0.0f;
uint8_t highest_irq_num = 0U;
uint16_t highest_irq_rate = 0U;
uint8_t longest_irq_num = 0U;
uint32_t longest_irq_time = 0U;

void handle_interrupt(IRQn_Type irq_type) {
  ENTER_CRITICAL();
  if (interrupt_depth == 0U) {
    uint32_t time = microsecond_timer_get();
    idle_time += get_ts_elapsed(time, last_time);
    last_time = time;
  }
  interrupt_depth += 1U;
  EXIT_CRITICAL();

  interrupts[irq_type].call_counter++;
  interrupts[irq_type].handler();

  // Check that the interrupts don't fire too often
  if (check_interrupt_rate && (interrupts[irq_type].call_counter > interrupts[irq_type].max_call_rate)) {
    fault_occurred(interrupts[irq_type].call_rate_fault);
  }

  ENTER_CRITICAL();
  interrupt_depth -= 1U;
  if (interrupt_depth == 0U) {
    uint32_t time = microsecond_timer_get();
    uint32_t elapsed_time = get_ts_elapsed(time, last_time);
    busy_time += elapsed_time;
    interrupts[irq_type].last_call_time += elapsed_time;
    last_time = time;
  }
  EXIT_CRITICAL();
}

// Every second
void interrupt_timer_handler(void) {
  if (INTERRUPT_TIMER->SR != 0) {
    highest_irq_rate = 0;
    highest_irq_num = 0;
    longest_irq_time = 0;
    longest_irq_num = 0;

    for (uint16_t i = 0U; i < NUM_INTERRUPTS; i++) {
      // Log IRQ call rate faults
      if (check_interrupt_rate && (interrupts[i].call_counter > interrupts[i].max_call_rate)) {
        print("Interrupt 0x"); puth(i); print(" fired too often (0x"); puth(interrupts[i].call_counter); print("/s)!\n");
      }

      // Reset interrupt counters
      interrupts[i].call_rate = interrupts[i].call_counter;
      interrupts[i].call_counter = 0U;

      // FIXME: Exceptions aren't logged or checked by our code at all.
      if (interrupts[i].call_rate > highest_irq_rate) {
        highest_irq_rate = interrupts[i].call_rate;
        highest_irq_num = i;
      }

      if (interrupts[i].last_call_time > longest_irq_time) {
        longest_irq_time = interrupts[i].last_call_time;
        longest_irq_num = i;
      }

      interrupts[i].last_call_time = 0U;
    }

    // Calculate interrupt load
    // The bootstub does not have the FPU enabled, so can't do float operations.
#if !defined(PEDAL) && !defined(BOOTSTUB)
    interrupt_load = ((busy_time + idle_time) > 0U) ? ((float) busy_time) / (busy_time + idle_time) : 0.0f;
#endif
    idle_time = 0U;
    busy_time = 0U;
  }
  INTERRUPT_TIMER->SR = 0;
}

void init_interrupts(bool check_rate_limit) {
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  check_interrupt_rate = check_rate_limit;

  for(uint16_t i=0U; i<NUM_INTERRUPTS; i++){
    interrupts[i].handler = unused_interrupt_handler;
    NVIC_SetPriority(i, NVIC_EncodePriority(NVIC_PRIORITYGROUP_4, 2, 0));
  }

  // Communication interrupts are at the second highest priority
  #ifdef STM32H7
  NVIC_SetPriority(SPI4_IRQn, NVIC_EncodePriority(NVIC_PRIORITYGROUP_4, 1, 0));
  NVIC_SetPriority(OTG_HS_IRQn, NVIC_EncodePriority(NVIC_PRIORITYGROUP_4, 1, 0));
  #else
  NVIC_SetPriority(OTG_FS_IRQn, NVIC_EncodePriority(NVIC_PRIORITYGROUP_4, 1, 0));
  #endif
  NVIC_SetPriority(DMA2_Stream3_IRQn, NVIC_EncodePriority(NVIC_PRIORITYGROUP_4, 1, 0));
  NVIC_SetPriority(DMA2_Stream2_IRQn, NVIC_EncodePriority(NVIC_PRIORITYGROUP_4, 1, 0));

  // Exceptions are at the highest priority
  NVIC_SetPriority(MemoryManagement_IRQn, NVIC_EncodePriority(NVIC_PRIORITYGROUP_4, 0, 0));
  NVIC_SetPriority(BusFault_IRQn, NVIC_EncodePriority(NVIC_PRIORITYGROUP_4, 0, 0));
  NVIC_SetPriority(UsageFault_IRQn, NVIC_EncodePriority(NVIC_PRIORITYGROUP_4, 0, 0));
  NVIC_SetPriority(SVCall_IRQn, NVIC_EncodePriority(NVIC_PRIORITYGROUP_4, 0, 0));
  NVIC_SetPriority(DebugMonitor_IRQn, NVIC_EncodePriority(NVIC_PRIORITYGROUP_4, 0, 0));
  NVIC_SetPriority(PendSV_IRQn, NVIC_EncodePriority(NVIC_PRIORITYGROUP_4, 0, 0));
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_PRIORITYGROUP_4, 0, 0));

  // Highest priority for interrupt timer FIXME: switch to SysTick
  NVIC_SetPriority(INTERRUPT_TIMER_IRQ, NVIC_EncodePriority(NVIC_PRIORITYGROUP_4, 0, 0));

  // Init interrupt timer for a 1s interval
  interrupt_timer_init();
}
