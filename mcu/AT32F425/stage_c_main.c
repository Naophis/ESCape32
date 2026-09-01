/*
 * ESCape32 AT32F425 port -- Stage C bring-up (free-running 3-channel
 * BEMF-phase ADC + DMA, no motor drive, no BEMF processing yet).
 *
 * Scope (porting-plan Section 15 "Stage C", as scoped by the user):
 * prove ADC channel mapping, calibration, clock, DMA and continuous
 * scan all work, with the raw per-phase codes inspectable over SWD.
 * TIM1/MP6540HA are NOT touched by this file (Stage B's 6-PWM output
 * is a separate, independent firmware image -- this is its own
 * standalone main(), same pattern as Stage A/B).
 *
 * Pin/ADC-channel mapping -- per the user's confirmed real schematic
 * (NOT the earlier PA0/PA1/PA4/PA5 F421-style virtual-neutral guess
 * from the porting-plan report, which this board does NOT use):
 *
 *   Phase A: 15k/3k divider -> PA0 -> ADC1_IN0
 *   Phase B: 15k/3k divider -> PA4 -> ADC1_IN4
 *   Phase C: 15k/3k divider -> PA5 -> ADC1_IN5
 *
 * ADC1_IN0/IN4/IN5 confirmed from DS_AT32F425_V2.03_EN.pdf Table 5
 * (pin definitions), not guessed from the GPIO number.
 *
 * There is no dedicated virtual-neutral ADC input on this board (no
 * PA1 wiring). Stage C does not need a neutral/reference yet -- it
 * only proves the 3 raw phase-voltage channels convert correctly.
 * The BEMF reference (e.g. (VA+VB+VC)/3, or something more adapted to
 * ESCape32's existing floating-phase/PWM-sampling-timing model) is a
 * Stage D design decision, deliberately not made here.
 *
 * ADC clock: CRM_ADC_DIV_6 -> APB2(96MHz)/6 = 16MHz, matching Table 42
 * (0.6-28MHz valid range) and close to Table 43's 14MHz reference
 * point (so RAIN_max figures don't need interpolation) -- same
 * divider Artery's own repeat_conversion_loop_transfer example uses.
 * Sample time: ADC_SAMPLETIME_239_5 (the slow/safe end, same as that
 * official example) -- Stage C is about correctness, not BEMF timing;
 * the aggressive ts=13.5-cycle timing this port's BEMF budget needs
 * is a Stage D concern once PWM-synchronized triggered sampling
 * replaces this free-running scan.
 */

#include "clock_config.h"

#define PHASE_A_CHANNEL ADC_CHANNEL_0 /* PA0 */
#define PHASE_B_CHANNEL ADC_CHANNEL_4 /* PA4 */
#define PHASE_C_CHANNEL ADC_CHANNEL_5 /* PA5 */

static volatile uint16_t adc_buf[3]; /* DMA target, order: A, B, C (sequence 1,2,3) */

volatile uint32_t stage_c_heartbeat;
volatile uint16_t stage_c_adc_a;
volatile uint16_t stage_c_adc_b;
volatile uint16_t stage_c_adc_c;
volatile uint32_t stage_c_dma_count;   /* DMA transfer-complete (one full 3-ch scan) count */
volatile uint32_t stage_c_dma_error;   /* DMA_DTERR_INT occurrences (see note below) */
volatile int stage_c_adc_ready;        /* set once calibration + first trigger are done */

void _init(void) {}
void _fini(void) {}

static void gpio_config(void)
{
  gpio_init_type gpio_init_struct;

  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);

  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
  gpio_init_struct.gpio_pins = GPIO_PINS_0 | GPIO_PINS_4 | GPIO_PINS_5;
  gpio_init(GPIOA, &gpio_init_struct);
}

static void dma_config(void)
{
  dma_init_type dma_init_struct;

  crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
  nvic_irq_enable(DMA1_Channel1_IRQn, 0, 0);
  dma_reset(DMA1_CHANNEL1);

  /* AT32F425's DMA has per-channel "flexible" request routing (not a
   * fixed channel<->peripheral map like older AT32/STM32 parts) --
   * confirmed via at32f425_dma.h DMA_FLEXIBLE_ADC1 and Artery's own
   * repeat_conversion_loop_transfer example, which uses this exact call. */
  dma_flexible_config(DMA1, FLEX_CHANNEL1, DMA_FLEXIBLE_ADC1);

  dma_default_para_init(&dma_init_struct);
  dma_init_struct.buffer_size = 3;
  dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
  dma_init_struct.memory_base_addr = (uint32_t)adc_buf;
  dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
  dma_init_struct.memory_inc_enable = TRUE;
  dma_init_struct.peripheral_base_addr = (uint32_t)&(ADC1->odt);
  dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
  dma_init_struct.peripheral_inc_enable = FALSE;
  dma_init_struct.priority = DMA_PRIORITY_HIGH;
  dma_init_struct.loop_mode_enable = TRUE; /* circular: keeps overwriting adc_buf forever */
  dma_init(DMA1_CHANNEL1, &dma_init_struct);

  dma_interrupt_enable(DMA1_CHANNEL1, DMA_FDT_INT, TRUE);
  dma_interrupt_enable(DMA1_CHANNEL1, DMA_DTERR_INT, TRUE);
}

static void adc_config(void)
{
  adc_base_config_type adc_base_struct;

  crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK, TRUE);
  adc_reset(ADC1);
  crm_adc_clock_div_set(CRM_ADC_DIV_6); /* 96MHz/6 = 16MHz, within Table42's 0.6-28MHz */

  adc_base_default_para_init(&adc_base_struct);
  adc_base_struct.sequence_mode = TRUE;
  adc_base_struct.repeat_mode = TRUE; /* free-running: re-trigger automatically */
  adc_base_struct.data_align = ADC_RIGHT_ALIGNMENT;
  adc_base_struct.ordinary_channel_length = 3;
  adc_base_config(ADC1, &adc_base_struct);

  adc_ordinary_channel_set(ADC1, PHASE_A_CHANNEL, 1, ADC_SAMPLETIME_239_5);
  adc_ordinary_channel_set(ADC1, PHASE_B_CHANNEL, 2, ADC_SAMPLETIME_239_5);
  adc_ordinary_channel_set(ADC1, PHASE_C_CHANNEL, 3, ADC_SAMPLETIME_239_5);

  adc_ordinary_conversion_trigger_set(ADC1, ADC12_ORDINARY_TRIG_SOFTWARE, TRUE);
  adc_dma_mode_enable(ADC1, TRUE);

  adc_enable(ADC1, TRUE);

  adc_calibration_init(ADC1);
  while (adc_calibration_init_status_get(ADC1));
  adc_calibration_start(ADC1);
  while (adc_calibration_status_get(ADC1));
}

/* Transfer-complete = one full 3-channel scan landed in adc_buf.
 * There is no dedicated ADC "ordinary conversion overrun" flag in
 * at32f425_adc.h for this part (checked: only VMOR/CCE/PCCE/OCCS/PCCS
 * exist, no OVR-equivalent) -- so overrun-style problems (DMA falling
 * behind the ADC) are instead caught via the DMA channel's own
 * transfer-error interrupt (DMA_DTERR_INT), which IS a real,
 * documented flag. */
void DMA1_Channel1_IRQHandler(void)
{
  if (dma_interrupt_flag_get(DMA1_FDT1_FLAG) != RESET) {
    dma_flag_clear(DMA1_FDT1_FLAG);
    stage_c_adc_a = adc_buf[0];
    stage_c_adc_b = adc_buf[1];
    stage_c_adc_c = adc_buf[2];
    stage_c_dma_count++;
  }
  if (dma_interrupt_flag_get(DMA1_DTERR1_FLAG) != RESET) {
    dma_flag_clear(DMA1_DTERR1_FLAG);
    stage_c_dma_error++;
  }
}

int main(void)
{
  clock_config_96mhz();

  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
  gpio_config();
  dma_config();
  adc_config();

  dma_channel_enable(DMA1_CHANNEL1, TRUE);
  adc_ordinary_software_trigger_enable(ADC1, TRUE);
  stage_c_adc_ready = 1;

  for (;;) {
    ++stage_c_heartbeat;
  }
}
