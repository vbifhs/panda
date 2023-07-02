int countcount = 0;

uint32_t spilastts = 0;
void spilog(const char *a) {
  uint32_t now = microsecond_timer_get();
  puth(get_ts_elapsed(now, spilastts));
  print(" ");
  print(a);
  spilastts = now;
}

// master -> panda DMA start
void llspi_mosi_dma(uint8_t *addr, int len) {
  UNUSED(len);

  // disable DMA + SPI
  register_clear_bits(&(SPI4->CFG1), SPI_CFG1_RXDMAEN);
  DMA2_Stream2->CR &= ~DMA_SxCR_EN;
  register_clear_bits(&(SPI4->CR1), SPI_CR1_SPE);

  // drain the bus
  while ((SPI4->SR & SPI_SR_RXP) != 0U) {
    volatile uint8_t dat = SPI4->RXDR;
    (void)dat;
  }

  // clear all pending
  SPI4->IFCR |= (0x1FFU << 3U);
  register_set(&(SPI4->IER), 0, 0x3FFU);

  // setup destination and length
  register_set(&(DMA2_Stream2->M0AR), (uint32_t)addr, 0xFFFFFFFFU);
  DMA2_Stream2->NDTR = len;
  //DMA2_Stream2->NDTR = SPI_BUF_SIZE;

  // enable DMA + SPI
  DMA2_Stream2->CR |= DMA_SxCR_EN;
  register_set_bits(&(SPI4->CFG1), SPI_CFG1_RXDMAEN);
  register_set_bits(&(SPI4->CR1), SPI_CR1_SPE);

  if (countcount < 25) spilog("RX\n");
}

// panda -> master DMA start
void llspi_miso_dma(uint8_t *addr, int len) {
  // disable DMA + SPI
  DMA2_Stream3->CR &= ~DMA_SxCR_EN;
  register_clear_bits(&(SPI4->CFG1), SPI_CFG1_TXDMAEN);
  register_clear_bits(&(SPI4->CR1), SPI_CR1_SPE);

  // setup source and length
  register_set(&(DMA2_Stream3->M0AR), (uint32_t)addr, 0xFFFFFFFFU);
  DMA2_Stream3->NDTR = len;

  // clear under-run while we were reading
  SPI4->IFCR |= (0x1FFU << 3U);

  // setup interrupt on TXC
  register_set(&(SPI4->IER), (1U << SPI_IER_EOTIE_Pos), 0x3FFU);

  // enable DMA + SPI
  register_set_bits(&(SPI4->CFG1), SPI_CFG1_TXDMAEN);
  DMA2_Stream3->CR |= DMA_SxCR_EN;
  register_set_bits(&(SPI4->CR1), SPI_CR1_SPE);
}

// master -> panda DMA finished
void DMA2_Stream2_IRQ_Handler(void) {
  if (countcount < 25) spilog("RX DMA done\n");

  // Clear interrupt flag
  DMA2->LIFCR = DMA_LIFCR_CTCIF2;

  spi_rx_done();
}

// panda -> master DMA finished
void DMA2_Stream3_IRQ_Handler(void) {
  ENTER_CRITICAL();

  DMA2->LIFCR = DMA_LIFCR_CTCIF3;
  spi_tx_dma_done = true;

  EXIT_CRITICAL();
}

// panda TX finished
void SPI4_IRQ_Handler(void) {
  // clear flag
  SPI4->IFCR |= (0x1FFU << 3U);

  if (spi_tx_dma_done && ((SPI4->SR & SPI_SR_TXC) != 0)) {
    spi_tx_dma_done = false;
    spi_tx_done(false);
  }
}

void EXTI15_10_IRQ_Handler(void) {
  if (EXTI->PR1 & EXTI_PR1_PR11) {
    EXTI->PR1 |= EXTI_PR1_PR11;

    bool level = get_gpio_input(GPIOE, 11);

    if (countcount < 25) {
      countcount++;
      spilog("SPI EXTI "); puth2(level); print("\n");
      spilog("- NDTR "); puth(DMA2_Stream2->NDTR); print("\n");
    }
    //register_clear_bits(&(SPI4->CFG1), SPI_CFG1_RXDMAEN);

    // end RX
    if (((SPI4->CFG1 & SPI_CFG1_RXDMAEN) != 0) && (DMA2_Stream2->NDTR < SPI_BUF_SIZE)) {
      //print("RXE\n");

      /*
      // disable DMA + SPI
      register_clear_bits(&(SPI4->CR1), SPI_CR1_SPE);
      register_clear_bits(&(SPI4->CFG1), SPI_CFG1_RXDMAEN);
      DMA2_Stream2->CR &= ~DMA_SxCR_EN;

      //DMA2_Stream2_IRQ_Handler();
      spi_rx_done();
      */
    }
  }
}

void llspi_init(void) {
  REGISTER_INTERRUPT(SPI4_IRQn, SPI4_IRQ_Handler, (SPI_IRQ_RATE * 2U), FAULT_INTERRUPT_RATE_SPI)
  REGISTER_INTERRUPT(DMA2_Stream2_IRQn, DMA2_Stream2_IRQ_Handler, SPI_IRQ_RATE, FAULT_INTERRUPT_RATE_SPI_DMA)
  REGISTER_INTERRUPT(DMA2_Stream3_IRQn, DMA2_Stream3_IRQ_Handler, SPI_IRQ_RATE, FAULT_INTERRUPT_RATE_SPI_DMA)

  // Setup MOSI DMA
  register_set(&(DMAMUX1_Channel10->CCR), 83U, 0xFFFFFFFFU);
  register_set(&(DMA2_Stream2->CR), (DMA_SxCR_MINC | DMA_SxCR_TCIE), 0x1E077EFEU);
  register_set(&(DMA2_Stream2->PAR), (uint32_t)&(SPI4->RXDR), 0xFFFFFFFFU);

  // Setup MISO DMA, memory -> peripheral
  register_set(&(DMAMUX1_Channel11->CCR), 84U, 0xFFFFFFFFU);
  register_set(&(DMA2_Stream3->CR), (DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE), 0x1E077EFEU);
  register_set(&(DMA2_Stream3->PAR), (uint32_t)&(SPI4->TXDR), 0xFFFFFFFFU);

  // Enable SPI
  register_set(&(SPI4->IER), 0, 0x3FFU);
  register_set(&(SPI4->CFG1), (7U << SPI_CFG1_DSIZE_Pos), SPI_CFG1_DSIZE_Msk);
  register_set(&(SPI4->CFG2), SPI_CFG2_SSM, SPI_CFG2_SSM_Msk);
  register_set(&(SPI4->UDRDR), 0xcd, 0xFFFFU);  // set under-run value for debugging
  register_set(&(SPI4->CR1), SPI_CR1_SPE, 0xFFFFU);
  register_set(&(SPI4->CR2), 0, 0xFFFFU);

  // Setup CS IRQ, EXTI15_10
  REGISTER_INTERRUPT(EXTI15_10_IRQn, EXTI15_10_IRQ_Handler, SPI_IRQ_RATE, FAULT_INTERRUPT_RATE_SPI_CS)
  register_set(&(SYSCFG->EXTICR[2]), SYSCFG_EXTICR3_EXTI11_PE, 0xF000U);
  register_set_bits(&(EXTI->IMR1), EXTI_IMR1_IM11);
  register_set_bits(&(EXTI->RTSR1), EXTI_RTSR1_TR11);
  register_clear_bits(&(EXTI->FTSR1), EXTI_FTSR1_TR11);
  NVIC_EnableIRQ(EXTI15_10_IRQn);

  NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  NVIC_EnableIRQ(SPI4_IRQn);
}
