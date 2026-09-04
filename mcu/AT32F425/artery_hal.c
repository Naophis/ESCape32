/*
** ESCape32 AT32F425 (MOUSEF425) port -- compatibility layer, Artery-
** facing half. See artery_hal.h for the split rationale.
**
** The real MOUSEF425 target links against libopencm3 (opencm3_stm32f0)
** and its own vector table/startup, NOT the vendored
** mcu/AT32F425/vendor/startup_at32f425.s (which would duplicate/
** conflict with libopencm3's reset handler and vector table). The
** vendored Artery *driver* code (CRM/ADC/DMA/FLASH register access,
** genuinely incompatible with libopencm3's STM32F0 assumptions -- see
** config.h) is still needed, so the specific driver .c files this file
** depends on are pulled in directly (a small "unity build"), since the
** top-level CMakeLists.txt's add_target() only globs mcu/AT32F425/*.c,
** not mcu/AT32F425/vendor/*.c, and add_target() itself is shared
** upstream build infrastructure this port does not modify.
*/

#include "vendor/at32f425_crm.c"
#include "vendor/at32f425_gpio.c"
#include "vendor/at32f425_flash.c"
#include "vendor/at32f425_adc.c"
#include "vendor/at32f425_dma.c"
#include "vendor/system_at32f425.c"

// clock_config.c/.h live under bringup/ (that subdirectory is excluded
// from the real MOUSEF425 target's automatic mcu/AT32F425/*.c glob,
// which is why the bring-up harnesses' stage_*_main.c files had to
// move there too -- see bringup/ 's own layout). Pulling in the .c
// (not just the header) here is the same "unity build" approach as the
// vendor driver includes above: clock_config_96mhz() is the one
// already hardware-validated 96MHz bring-up sequence and this file
// reuses it verbatim rather than re-deriving it.
#include "bringup/clock_config.c"

#include "artery_hal.h"

void at32_clock_init(void)
{
  clock_config_96mhz();
}

void at32_dma_flexible_routing_init(void)
{
  crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
  dma_flexible_config(DMA1, FLEX_CHANNEL1, DMA_FLEXIBLE_ADC1);
  dma_flexible_config(DMA1, FLEX_CHANNEL2, DMA_FLEXIBLE_UART1_TX);
  dma_flexible_config(DMA1, FLEX_CHANNEL3, DMA_FLEXIBLE_UART1_RX);
  dma_flexible_config(DMA1, FLEX_CHANNEL4, DMA_FLEXIBLE_UART2_TX);
  dma_flexible_config(DMA1, FLEX_CHANNEL5, DMA_FLEXIBLE_UART2_RX);
}

volatile uint16_t at32_adc_buf[3]; // A(PA0), B(PA4), C(PA5) -- fixed physical order, Stage E14-E23

// PHASE_A/B/C_CHANNEL, ADC_TRIGGER config: identical to Stage E14/E23's
// hardware-validated setup. CH4/PWM_MODE_B on TIM1 (the actual trigger
// source) is configured by config.c (libopencm3, since TIM1 is
// address/vector-compatible); this function only sets up the ADC1
// ordinary group + DMA1 channel1 that CH4's rising edge fires into.
void at32_bemf_adc_dma_init(void)
{
  gpio_init_type g;
  dma_init_type d;
  adc_base_config_type b;

  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  gpio_default_para_init(&g);
  g.gpio_mode = GPIO_MODE_ANALOG;
  g.gpio_pins = GPIO_PINS_0 | GPIO_PINS_4 | GPIO_PINS_5;
  gpio_init(GPIOA, &g);

  dma_reset(DMA1_CHANNEL1);
  dma_default_para_init(&d);
  d.buffer_size = 3;
  d.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
  d.memory_base_addr = (uint32_t)at32_adc_buf;
  d.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
  d.memory_inc_enable = TRUE;
  d.peripheral_base_addr = (uint32_t)&(ADC1->odt);
  d.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
  d.peripheral_inc_enable = FALSE;
  d.priority = DMA_PRIORITY_HIGH;
  d.loop_mode_enable = TRUE;
  dma_init(DMA1_CHANNEL1, &d);
  dma_interrupt_enable(DMA1_CHANNEL1, DMA_FDT_INT, TRUE);
  dma_interrupt_enable(DMA1_CHANNEL1, DMA_DTERR_INT, TRUE);

  crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK, TRUE);
  adc_reset(ADC1);
  crm_adc_clock_div_set(CRM_ADC_DIV_4);

  adc_base_default_para_init(&b);
  b.sequence_mode = TRUE;
  b.repeat_mode = FALSE;
  b.data_align = ADC_RIGHT_ALIGNMENT;
  b.ordinary_channel_length = 3;
  adc_base_config(ADC1, &b);

  adc_ordinary_channel_set(ADC1, ADC_CHANNEL_0, 1, ADC_SAMPLETIME_13_5); // PA0 = A
  adc_ordinary_channel_set(ADC1, ADC_CHANNEL_4, 2, ADC_SAMPLETIME_13_5); // PA4 = B
  adc_ordinary_channel_set(ADC1, ADC_CHANNEL_5, 3, ADC_SAMPLETIME_13_5); // PA5 = C

  adc_ordinary_conversion_trigger_set(ADC1, ADC12_ORDINARY_TRIG_TMR1CH4, TRUE);
  adc_dma_mode_enable(ADC1, TRUE);

  adc_enable(ADC1, TRUE);
  adc_calibration_init(ADC1);
  while (adc_calibration_init_status_get(ADC1));
  adc_calibration_start(ADC1);
  while (adc_calibration_status_get(ADC1));

  dma_channel_enable(DMA1_CHANNEL1, TRUE);
}

int at32_bemf_dma_transfer_complete(void)
{
  if (dma_interrupt_flag_get(DMA1_DTERR1_FLAG) != RESET) dma_flag_clear(DMA1_DTERR1_FLAG);
  if (dma_interrupt_flag_get(DMA1_FDT1_FLAG) == RESET) return 0;
  dma_flag_clear(DMA1_FDT1_FLAG);
  return 1;
}
