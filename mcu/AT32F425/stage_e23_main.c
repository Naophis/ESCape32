/*
 * ESCape32 AT32F425 port -- Stage E23: FIRST closed-loop BEMF-driven
 * commutation handover. Everything upstream of this file is now
 * trusted on real hardware: TMR1 CH1-3 6-PWM (MUX_2 fix confirmed 6/6
 * correct regardless of configuration order, Stage E15D), CH4
 * PWM_MODE_B ADC trigger at ~1.2us (Stage E10), 15k/3k divider,
 * diff=floating_adc-positive_adc/2 threshold, the UNARMED->ARMED->
 * CONFIRM Schmitt-trigger ZC state machine, and the TMR2 16-bit-wrap-
 * safe delay measurement (all Stage E15, re-validated on the MUX-fixed
 * board: 9/9 buckets from 15ms down to 3ms/step came back with ZC
 * confirmed on essentially every sector, ADC saturation=0, dma_error=0,
 * and a clean, speed-independent expected_dir[6] = {RISING, FALLING,
 * RISING, FALLING, RISING, FALLING} for steps 0-5, now adopted as
 * final). NONE of that is changed in this file.
 *
 * WHAT THIS FILE ADDS: alignment -> open-loop ramp (15,13,11,9,7ms,
 * 40 steps each, identical mechanism to Stage E2/E6/E14/E15) -> hold
 * at 6ms/step (the fastest speed Stage E15 validated without stall;
 * open-loop forced acceleration to 3ms/step was shown by the user to
 * desync the real motor, so 6ms is this file's target, not 3ms) ->
 * once 6 CONSECUTIVE sectors at that hold rate confirm ZC in the
 * expected_dir[] direction, HANDOVER to closed-loop commutation.
 *
 * Closed-loop mechanism: on every direction-correct confirmed ZC, the
 * already-computed zc_delay_us (time from the last commutation edge to
 * this zero-cross, i.e. the ~30 electrical-degree interval) is reused
 * as-is for the delay from THIS zero-cross to the NEXT commutation --
 * the classic symmetric "30 degrees after ZC" scheme, deliberately not
 * doing anything more sophisticated (no filtering/prediction) for this
 * first bring-up. That delay is converted to TMR3 ticks (10us/tick,
 * same TMR3 unit Stage E15's ramp already uses) and TMR3 is reprogrammed
 * as a ONE-SHOT: paused, counter reset to 0, pr set to the new delay,
 * and restarted -- so the SAME TMR3-overflow-driven apply_step()
 * advance mechanism Stage E2 onward has always used now fires at a
 * ZC-derived instant instead of a fixed ramp period. No new commutation
 * mechanism was introduced -- only what reprograms TMR3's period, and
 * when.
 *
 * TMR3's role while MODE_CLOSED_LOOP is ALWAYS one of exactly two,
 * tracked by an explicit tmr3_purpose_t state (see its declaration
 * below) so a ZC-wait timeout can never masquerade as -- or silently
 * trigger -- a normal commutation:
 *   - TMR3_PURPOSE_TIMEOUT: armed immediately after every apply_step()
 *     (arm_timeout_watchdog()), waiting for this sector's ZC. An
 *     overflow in this state is PURELY a timeout (zc_timeout_count++)
 *     -- it does not call apply_step() and does not advance the
 *     sector; the watchdog is simply re-armed (or the motor stopped,
 *     see below) and commutation stays where it is until a real ZC
 *     arrives.
 *   - TMR3_PURPOSE_COMMUTATION: armed ONLY by a direction-correct
 *     confirmed ZC (schedule_next_commutation()), for exactly the
 *     measured 30-degree-equivalent delay. Its overflow IS the real
 *     scheduled commutation -- apply_step() runs, then TMR3 is
 *     immediately re-armed back to TMR3_PURPOSE_TIMEOUT for the new
 *     sector.
 *
 * Fault handling (kept deliberately simple, per instruction -- no
 * acceleration control or auto-restart in this first version):
 *   - A confirmed ZC in the WRONG direction (mismatch vs expected_dir)
 *     is counted (polarity_error_count) but does NOT reprogram TMR3 --
 *     the running timeout watchdog is left untouched, so a suspect
 *     reading is never acted on.
 *   - A timeout watchdog overflow (no confirmed ZC at all this sector)
 *     is counted (zc_timeout_count) and the watchdog is simply
 *     re-armed for another window -- see TMR3_PURPOSE_TIMEOUT above;
 *     it never drives a commutation by itself.
 *   - Either fault increments a shared consecutive-fault counter,
 *     reset to 0 on the next direction-correct confirm. After
 *     FAULT_STOP_THRESHOLD consecutive faults, the motor is stopped
 *     (all six gates forced low) immediately -- no automatic restart,
 *     per instruction.
 *   - The open-loop hold phase (before handover) also has a bound,
 *     MAX_STEPS_AT_TARGET: if 6 consecutive correct sectors are never
 *     achieved within that many steps at 6ms/step, the run stops
 *     safely with handover_success left at 0.
 *
 * Diagnostics (global, not per-bucket -- this file measures ONE run):
 * handover_success, handover_step_index, closed_loop_step_count,
 * zc_count, zc_timeout_count, polarity_error_count, sector_period_us
 * min/max/sum (closed-loop only), scheduled_delay_us min/max/sum
 * (closed-loop, direction-correct events only), final_step.
 *
 * STAGE E16B ADDITION (kept, unchanged): apply_step()'s per-sector
 * reset order/content was code-reviewed and found correct; the
 * instrumentation added there (stage_e23_first_closed_loop_snapshot
 * for ZC-state fields, stage_e23_first_timeout_diag for the first
 * timed-out sector's ZC/ADC summary) is retained as-is in this file.
 * Its real-hardware result narrowed the problem further: after the
 * first closed-loop commutation (step1->step2), adc_sample_count=0,
 * armed_count=0, diff_min/max never updated, but
 * dma_irq_count_delta=1 -- i.e. exactly ONE DMA FDT IRQ fired for the
 * new sector (with plausible-looking last ADC values: A=0,B=1362,
 * C=21) and then NOTHING further, for the remaining ~9ms+ of timeout
 * waiting. That single IRQ, arriving right at/before the sector
 * boundary, likely got attributed to blanking (COMMUTATION_BLANK_
 * SCANS=2 not yet exhausted) rather than counted as a "processed"
 * sample -- consistent with adc_sample_count staying 0 while
 * dma_irq_count_delta reads 1. The real question this points to: does
 * the ADC/DMA/CH4 chain keep running at all after that first sample,
 * or does something stop it right there.
 *
 * STAGE E16C ADDITION (this file; commutation timing/ZC/delay/duty are
 * NOT touched -- pure ADC/DMA/CH4 re-arm diagnostics only): two raw-
 * register snapshots, taken at the two most informative instants, to
 * see directly whether the ADC/DMA/CH4 chain is still configured to
 * keep converting after entering closed loop, instead of re-deriving
 * it from source review again:
 *
 * (1) stage_e23_regs_after_commutation: captured immediately after
 *     the FIRST closed-loop apply_step() (closed_loop_step_count
 *     becomes 1) -- DMA1_CHANNEL1->ctrl (chen and the rest),
 *     DMA1_CHANNEL1->dtcnt (remaining transfer count), DMA1->sts
 *     (fdtf1 and friends), ADC1->ctrl2 (octen = ordinary external
 *     trigger enable, ocdmaen), ADC1->osq1 (oclen = ordinary sequence
 *     length), TMR1->cctrl (c4en et al), TMR1->cm2 (c4octrl/mode),
 *     TMR1->c4dt (CCR4), plus the running tmr1_ch4_event_count and
 *     dma_fdt_count IRQ counters at that instant.
 *
 * (2) stage_e23_regs_after_first_dma_irq: the SAME register set,
 *     captured inside DMA1_Channel1_IRQHandler at the first FDT IRQ
 *     that occurs after entering closed loop -- directly shows
 *     whether that IRQ's own hardware auto-reload (loop_mode_enable=
 *     TRUE) actually restored dtcnt to a fresh transfer count, whether
 *     DMA1_CHANNEL1->ctrl.chen is still set, and whether ADC1->ctrl2's
 *     octen is still 1 -- i.e. whether anything LOOKS disabled right
 *     after that one successful transfer, versus the registers reading
 *     healthy while conversions still mysteriously stop.
 *
 * stage_e23_first_timeout_diag (retained from Stage E16B) still
 * reports the running tmr1_ch4_event_count/dma_fdt_count deltas across
 * the WHOLE first closed-loop sector up to timeout -- the pass
 * criterion this file targets is ch4_event_count_delta >> 1 and
 * dma_fdt_count_delta >> 1 (and therefore adc_sample_count >> 1) by
 * the time that sector times out, instead of the single-sample dead
 * stop Stage E16B observed.
 *
 * STAGE E16D ADDITION (this file; control logic is NOT changed at all
 * -- CH4 config/Mode B/CCR4/ADC/DMA setup untouched): Stage E16C's
 * result narrowed the problem to TMR1 itself: DMA ctrl/dtcnt, ADC
 * ctrl2, and TMR1's own cctrl/cm2/c4dt all read back unchanged and
 * healthy across the transition, YET ch4_event_count_delta=0 while
 * dma_irq_count_delta=1 for the first closed-loop sector -- i.e. CH4's
 * compare-match event (which requires TMR1's counter to actually keep
 * running past CCR4 every cycle) never fired again after the first
 * closed-loop commutation, even though the ONE DMA transfer that did
 * happen was most likely a pipelined conversion already in flight
 * from before that commutation, not new evidence TMR1 kept running.
 * cctrl/cm2/c4dt describe CH4's CONFIGURATION, not whether TMR1's
 * counter itself is still enabled and counting -- that lives in
 * ctrl1's tmren bit (counter enable) and cval (the live count), which
 * Stage E16C never captured. This file adds exactly that:
 *
 * (1) A full TMR1 register dump -- ctrl1 (tmren, the counter-enable
 *     bit, first and foremost), cval, pr, iden (interrupt/DMA enable,
 *     particularly c4ien/c4den), ists (status, particularly c4if),
 *     cctrl, cm2, c4dt, swevt (software event trigger, read as a
 *     snapshot only -- this file never WRITES it) -- captured at TWO
 *     points: stage_e23_tmr1_regs_after_commutation (immediately
 *     after the first closed-loop apply_step()) and
 *     stage_e23_tmr1_regs_at_timeout (at the first closed-loop
 *     timeout, right where stage_e23_first_timeout_diag is already
 *     captured). If ctrl1.tmren reads 0 at either point, TMR1's
 *     counter itself has been disabled -- direct proof, not inferred
 *     from CH4-specific registers that don't carry that bit.
 *
 * (2) A non-blocking TMR1->cval progression check: cval_0 (+ its own
 *     TMR2 timestamp) captured at the same instant as
 *     stage_e23_tmr1_regs_after_commutation, and cval_1 (+ its TMR2
 *     timestamp) captured at the first DMA1_Channel1_IRQHandler FDT
 *     IRQ that fires afterward (the SAME natural IRQ Stage E16C's
 *     regs_after_first_dma_irq already uses) -- NOT a busy-wait: this
 *     reuses an interrupt that E16C's own data shows already fires
 *     shortly after the commutation, so no new blocking delay is
 *     introduced anywhere in the control path. If TMR1's counter is
 *     still running, cval_1 should differ from cval_0 by roughly
 *     (TMR2 timestamp delta * 96) ticks (96MHz/1MHz); if TMR1 is
 *     frozen, cval_1 == cval_0 regardless of how much real time
 *     (per the TMR2 timestamps) actually elapsed between the two
 *     captures.
 *
 * Per instruction, a source diff of every tmr_counter_enable(TMR1,...)
 * / tmr_output_enable(TMR1,...) call site between Stage E15 and Stage
 * E16 was performed before writing any of this: both files call
 * tmr_counter_enable(TMR1, FALSE)/tmr_output_enable(TMR1, FALSE) only
 * in stop_and_force_off(), and tmr_counter_enable(TMR1, TRUE)/
 * tmr_output_enable(TMR1, TRUE) only once in main(), identically in
 * both files -- Stage E16's closed-loop additions (schedule_next_
 * commutation(), arm_timeout_watchdog(), tmr3_arm_oneshot()) call
 * tmr_counter_enable(), tmr_counter_value_set(), tmr_event_sw_trigger()
 * exclusively on TMR3, never on TMR1, and TMR3/TMR1 are distinct
 * peripheral base pointers so there is no possibility of one call
 * silently hitting the other's registers. No source-level call that
 * pauses/disables TMR1 was found outside stop_and_force_off() in
 * either file -- if TMR1 is actually stopping, it is not through any
 * of these documented driver calls, which is exactly why this file
 * captures ctrl1.tmren and cval directly instead of continuing to
 * reason about it from the call sites alone.
 *
 * STAGE E16E ADDITION (diagnostics only -- ZC/duty/ADC/CH4/handover
 * condition/delay coefficients and the actual pause->cnt=0->pr=delay->
 * sw_trigger->resume re-arm SEQUENCE are all UNCHANGED in this file):
 * Stage E16D's result ruled out a frozen TMR1 -- ctrl1.tmren stayed 1,
 * cval visibly advanced (418@27675us -> 644@27677us, i.e. ~226 ticks
 * in ~2us, right around the expected ~96 ticks/us), pr/cctrl/cm2/c4dt
 * were all unchanged. So TMR1 itself is fine; the investigation moves
 * to TMR3's one-shot re-arm sequence (tmr3_arm_oneshot()) instead --
 * specifically whether tmr_event_sw_trigger(TMR3, TMR_OVERFLOW_SWTRIG)
 * sets TMR3's own overflow flag immediately (a documented side effect
 * of software UG-style events on this timer family), which could make
 * the very next TMR3_GLOBAL_IRQHandler entry fire immediately on a
 * stale/spurious flag instead of waiting for the real programmed
 * delay.
 *
 * This file adds logging ONLY, at exactly the points requested,
 * without changing the re-arm sequence itself yet:
 *
 * (1) Inside tmr3_arm_oneshot() (now taking an explicit purpose
 *     parameter so it can log with full context in one place): arm
 *     timestamp (TMR2->cval), purpose (TIMEOUT/COMMUTATION),
 *     requested_delay_us, and TMR3->ists read at three points --
 *     immediately BEFORE the software trigger, immediately AFTER it,
 *     and immediately BEFORE tmr_counter_enable(TMR3, TRUE) (the
 *     "resume" step) -- stage_e23_last_arm_log (overwritten every
 *     call) plus separately latched stage_e23_first_commutation_arm_
 *     log and stage_e23_first_timeout_arm_log (each captured once,
 *     the first time that purpose occurs).
 *
 * (2) At the very top of TMR3_GLOBAL_IRQHandler, BEFORE the existing
 *     flag-check/early-return (so a spurious entry is captured too,
 *     not filtered out first): ISR entry timestamp, the CURRENT
 *     tmr3_purpose, TMR3->ists/cval/pr as found at entry, and
 *     elapsed_us = entry timestamp - the matching arm log's
 *     timestamp. Same latch-once-per-purpose pattern as (1), but ALSO
 *     gated on stage_e23_mode==MODE_CLOSED_LOOP (tmr3_purpose is only
 *     meaningful once closed-loop starts; without this gate, every
 *     ordinary MODE_RAMP/MODE_HOLD overflow would spuriously satisfy
 *     "purpose==TMR3_PURPOSE_TIMEOUT", its zero-initialized default,
 *     and get latched long before the real closed-loop timeout this
 *     file actually needs to see).
 *
 * Pass/fail is read directly off stage_e23_first_commutation_arm_log
 * vs stage_e23_first_commutation_isr_entry: elapsed_us there should
 * land close to the ~3089us-class requested_delay_us if the re-arm is
 * honest; a near-zero elapsed_us (or ists already showing the overflow
 * flag set at "before_resume") is direct evidence of the SW-trigger-
 * sets-the-flag-immediately hypothesis. The same comparison for
 * stage_e23_first_timeout_arm_log/_isr_entry should show elapsed_us
 * close to that watchdog's requested_delay_us (last known delay x3).
 *
 * STAGE E16F: Stage E16E's real-hardware log CONFIRMED the hypothesis
 * -- ists_before_sw_trigger=0x1e, ists_after_sw_trigger=0x1f (bit0/
 * ovfif newly set) for BOTH the first COMMUTATION arm (requested
 * 3340us, actual ISR elapsed 3us) and the first TIMEOUT arm (requested
 * 10020us, actual ISR elapsed 5us): tmr_event_sw_trigger(TMR3,
 * TMR_OVERFLOW_SWTRIG) sets TMR3's own OVF flag immediately, and
 * because that flag was never cleared afterward, the already-enabled
 * TMR3 overflow interrupt fired within microseconds on the stale flag
 * instead of waiting for the real programmed delay -- root cause of
 * every closed-loop stall back through Stage E16.
 *
 * tmr3_arm_oneshot() is now fixed to the safe sequence per instruction
 * (using the same vendor APIs already used elsewhere in this project --
 * tmr_counter_enable(), tmr_interrupt_enable(TMR3, TMR_OVF_INT, ...),
 * tmr_flag_clear(TMR3, TMR_OVF_FLAG), tmr_counter_value_set(),
 * tmr_event_sw_trigger(), and CMSIS's NVIC_ClearPendingIRQ() for the
 * NVIC-level pending bit -- no ad hoc register pokes):
 *   1. tmr_counter_enable(TMR3, FALSE)            -- stop
 *   2. tmr_interrupt_enable(TMR3, TMR_OVF_INT, FALSE) -- IRQ disable
 *   3. tmr_flag_clear(TMR3, TMR_OVF_FLAG)          -- OVF flag clear
 *   4. tmr_counter_value_set(TMR3, 0)              -- CNT=0
 *   5. TMR3->pr = ticks - 1                        -- PR=delay_ticks-1
 *   6. tmr_event_sw_trigger(TMR3, TMR_OVERFLOW_SWTRIG) -- SW update event
 *   7. tmr_flag_clear(TMR3, TMR_OVF_FLAG)          -- clear the flag the
 *      SW trigger just set (THE required step -- everything above was
 *      already present in Stage E16-E16E; this line is the actual fix)
 *   8. NVIC_ClearPendingIRQ(TMR3_GLOBAL_IRQn)      -- clear any latched
 *      NVIC-level pending bit from step 6, belt-and-suspenders
 *   9. stage_e23_tmr3_purpose = purpose           -- set purpose
 *  10. tmr_interrupt_enable(TMR3, TMR_OVF_INT, TRUE)  -- IRQ re-enable
 *  11. tmr_counter_enable(TMR3, TRUE)              -- resume
 *
 * Verification logging: ists_after_sw_trigger (step 6/7 boundary),
 * ists_after_clear (right after step 7), requested_delay_us, and the
 * actual ISR elapsed_us are captured for the first COMMUTATION and
 * first TIMEOUT arm, same latch-once pattern as Stage E16E. Pass
 * criteria: ists_after_sw_trigger may show OVF=1 (expected, per the
 * confirmed hardware behavior), ists_after_clear must show OVF=0, and
 * actual elapsed_us must land close to requested_delay_us (~3340us-
 * class for COMMUTATION, ~10020us-class for TIMEOUT) instead of the
 * previous 3-5us stalls. ZC, handover condition, duty, ADC, CH4, and
 * the delay calculation itself are all unchanged.
 *
 * STAGE E17: Stage E16F's TMR3 flag-clear fix WORKED on real hardware
 * -- handover_success=1, closed_loop_step_count=22, zc_count=21,
 * polarity_error_count=0 (21/22 ZC capture rate), and
 * scheduled_delay ~= sector_period/2 exactly as the 30-degree-delay
 * design intends. But sector_period ranged 1770-22852us and 7 timeouts
 * occurred before the run stopped on 3 consecutive faults -- this file
 * is a STABILITY diagnostic for that variation, not a fix: duty,
 * acceleration control, ZC threshold, 15% duty, handover condition,
 * and the 30-degree delay calculation are all UNCHANGED, and so is
 * Stage E16F's TMR3 re-arm sequence.
 *
 * Two small ring buffers, sized to stay well within SRAM budget:
 *
 * (1) stage_e23_sector_log[64]: one entry per CLOSED-LOOP commutation
 *     (closed_loop_step_count 1-64), describing the sector that just
 *     ENDED (previous commutation -> this one) -- step_index,
 *     commutation_timestamp_us (this sector's start), zc_timestamp_us
 *     (the confirming ZC), measured sector_period_us, scheduled_
 *     delay_us (the delay that produced this commutation),
 *     expected_dir, zc_confirmed (always 1 for a logged entry -- a
 *     sector only reaches this log by way of a direction-correct
 *     confirm), timeout_occurred (whether >=1 timeout watchdog fired
 *     during this sector before its eventual confirm),
 *     consecutive_fault_count_at_confirm (the fault streak right
 *     before this confirm reset it to 0 -- 0 means clean, no
 *     preceding timeout/mismatch), and floating_adc/positive_adc as
 *     read at the confirming ZC. Captured across two IRQs without a
 *     race: the DMA IRQ (ZC confirm) stashes the per-sector values
 *     that only it can see (zc_timestamp, ADC readings, this sector's
 *     timeout count) into small "pending_*" statics BEFORE calling
 *     schedule_next_commutation() (which only re-arms TMR3 -- the
 *     actual commutation is still at least one tick away); the TMR3
 *     IRQ's existing COMMUTATION branch, which already computes
 *     period_us, reads those pending_* values once apply_step() has
 *     run and writes the completed log entry.
 *
 * (2) stage_e23_timeout_log[32]: one entry per zc_timeout_count event
 *     -- current_step, time_since_commutation_us (elapsed since this
 *     sector's apply_step, computed the same TMR2-mod-2^16-safe way
 *     as everywhere else in this project), expected_timeout_us (the
 *     watchdog's own requested_delay_us, from stage_e23_last_arm_log
 *     -- unchanged E16E/F instrumentation, reused not duplicated),
 *     last_valid_sector_period_us / last_valid_scheduled_delay_us (the
 *     most recent SUCCESSFUL sector's values, so a timeout can be
 *     compared against what "normal" looked like most recently),
 *     zc_state, and diff_min/diff_max observed since this sector's
 *     commutation (live_diff_min/live_diff_max, already tracked).
 *
 * What this data should answer directly: whether sector_period grows
 * gradually across the 64-entry log or jumps in one step; whether a
 * timeout's time_since_commutation lands just past its own expected_
 * timeout_us (a real late ZC) or something else; whether
 * zc_timeout_count increments more than once per stage_e23_timeout_log
 * entry's surrounding sector (visible directly from timeout_occurred
 * plus how many stage_e23_timeout_log entries share the same
 * current_step in a row); and whether particular step_index values
 * dominate the timeout_log (a sector-dependent pattern) versus being
 * evenly spread.
 *
 * STAGE E18: Stage E17's ring buffer showed a strong SECTOR-dependent
 * pattern, not a common "falling-direction" problem -- step1
 * (pos=A/neg=C/floating=B) stayed a healthy ~5.5-7.8ms, while step3
 * (pos=B/neg=A/floating=C) degraded to 18-21.5ms and step5
 * (pos=C/neg=B/floating=A) to 16.3-23.9ms, plus one anomalous 690us
 * early ZC on step4. Since step1's negative leg is C while step3/step5's
 * negative legs are A and B respectively, the user's instruction is to
 * investigate the negative-leg-specific electrical difference between
 * A/B and C FIRST, using the phase ADC itself as the observed output
 * state (Stage E15C's GPIO-idt-based AF readback was already shown
 * unreliable and is not reused here).
 *
 * No closed-loop logic, timeout, duty, ZC threshold, handover, or the
 * 30-degree delay calculation change in this file -- purely additional
 * per-sector ADC aggregation on top of the SAME 3-channel-per-PWM-
 * cycle scan this project has used since Stage E14:
 *
 * (1) stage_e23_sector_log[64] gains positive_adc_min/max/sum,
 *     negative_adc_min/max/sum, floating_adc_min/max, diff_min/max,
 *     and sample_count -- aggregated over every post-blanking scan
 *     from this sector's commutation to its ZC confirm (mean =
 *     sum/sample_count) -- plus positive_adc_at_zc/negative_adc_at_zc/
 *     floating_adc_at_zc/diff_at_zc, the exact instantaneous values at
 *     the confirming scan itself (negative_adc_at_zc is new; the other
 *     three already existed in Stage E17's log under different field
 *     names and are kept, now alongside diff_at_zc). Since the
 *     negative phase is low-side (should sit near GND regardless of
 *     PWM state), comparing negative_adc_min/max/mean across step1 vs
 *     step3 vs step5 is the direct test: if step3/5's negative_adc
 *     reads meaningfully higher, or swings much more across the PWM
 *     cycle, than step1's, that is electrical evidence pointing at
 *     CH1C/CH2C's actual output behavior (not just their GPIO/MUX
 *     routing, already fixed in Stage E15D) -- to be chased next only
 *     if this data shows it.
 *
 * (2) stage_e23_step_trace[3][16]: a short raw trace (positive_adc/
 *     negative_adc/floating_adc per sample, 16 consecutive post-
 *     blanking scans) captured ONCE for the FIRST closed-loop
 *     occurrence of each of step1/step3/step5 (index 0/1/2
 *     respectively) -- started the instant apply_step() enters that
 *     step (while MODE_CLOSED_LOOP), filled by the next 16 ADC scans
 *     regardless of ZC outcome. This is the most direct look at
 *     whether negative_adc is stable near GND or visibly moving scan-
 *     to-scan within a single sector.
 *
 * The final ~690us early-ZC sector from Stage E17 is still recorded
 * (it flows through the same sector_log/timeout_log mechanism as any
 * other sector) but per instruction NO plausibility filter is added
 * for it yet -- it is data to look at, not something this file reacts
 * to.
 *
 * STAGE E19: Stage E18's real-hardware result ruled out a negative-leg
 * A/B-vs-C electrical difference (negative_adc means ~19-34 across
 * step1/3/5, all near GND) but turned up something else: step5's first
 * 16-sample trace showed ALL THREE phases (positive/negative/floating)
 * simultaneously dropping to ~10-60 counts, coinciding with step5's
 * commutation-to-ZC time of ~8.9-11.1ms -- consistent with the
 * MP6540HA's documented OCP behavior (all six MOSFETs disabled/Hi-Z,
 * nFAULT low, ~10ms retry). nFAULT itself is confirmed NOT wired to
 * this MCU on this board, so this file verifies the OCP/Hi-Z
 * hypothesis using ONLY the existing 3-channel ADC scan already
 * running every PWM cycle -- no new pins, no control changes. Closed-
 * loop control, 15% duty, ZC, handover, timeout, the 30-degree delay
 * calculation, and Stage E16F's TMR3 fix are all unchanged.
 *
 * Diagnostic-only definition (per instruction, NEVER used for control
 * decisions in this file): a scan is "all-low" when
 * adc_a<100 && adc_b<100 && adc_c<100 (ALLLOW_THRESHOLD). This runs on
 * EVERY DMA FDT sample while MODE_CLOSED_LOOP && running && !aligning
 * -- deliberately NOT gated by blank_scans_remaining or zc_locked
 * (unlike the ZC classification logic), since the point is to observe
 * the driver's actual output state independent of what the ZC state
 * machine is doing with it.
 *
 * A simple start/end edge detector tracks entry into and exit from
 * the all-low condition:
 *   - on entry: start_timestamp_us (TMR2->cval), step_index (the
 *     sector active when it started), time_since_commutation_us
 *     (start_timestamp_us - that sector's own commutation timestamp,
 *     mod-2^16-safe as everywhere else in this project).
 *   - while active: running min/max of A/B/C, scan count.
 *   - on exit: end_timestamp_us, duration_us, number_of_adc_scans, and
 *     the min/max A/B/C observed during the whole event.
 * The first ALLLOW_LOG_SIZE=8 completed events get a full detail
 * record (stage_e23_alllow_log[8]) to keep RAM bounded; EVERY event
 * still updates the per-step aggregates stage_e23_alllow_event_count[6]
 * and stage_e23_alllow_longest_duration_us[6] regardless of whether
 * detail logging is still available, so step-dependence remains
 * visible even past the first 8 events. An event still active when the
 * run stops is closed out at that point too, so nothing is left
 * dangling.
 *
 * Pass/fail per instruction: if step5 shows an all-low event starting
 * right after its commutation and lasting on the order of several ms
 * up to ~10ms before the phases recover, that is close to conclusive
 * for MP6540HA OCP. threshold=100 is diagnostic-only, per instruction,
 * and is not read by any control-path code in this file.
 *
 * STAGE E20: Stage E19 essentially confirmed OCP on real hardware --
 * all-low events ONLY on step5 (event_count={0,0,0,0,0,5}, 5/5),
 * always starting ~48us after commutation, duration 7.666-9.875ms
 * (matches MP6540HA's typ 10ms OCP retry), all three phases pinned
 * near 0 throughout. This file tests ONE specific alternate
 * explanation before accepting step5's actual applied phase current as
 * the cause: whether apply_step()'s per-channel reconfiguration
 * (looping CH1/CH2/CH3 one at a time, each a separate tmr_output_
 * channel_config() call -- see the very same channel-order question
 * Stage E15D already tested and found NOT to matter for steady-state
 * gate levels) causes a brief cross-conduction/overlap TRANSIENT
 * during the step4->step5 commutation edge specifically, which could
 * itself look like or trigger OCP.
 *
 * Diagnostic-only break-before-make, added to apply_step() and applied
 * IDENTICALLY to every sector regardless of mode (open-loop ramp/hold
 * included, per instruction "全sectorで同じように行う"), BEFORE the
 * existing per-phase pos/neg/floating configuration loop:
 *   1. all three phases forced to the SAME "floating" configuration
 *      this file's apply_step() already uses for the floating role
 *      every sector (oc_mode=FORCE_LOW, oc_output_state=FALSE,
 *      occ_output_state=FALSE for CH1/CH2/CH3 main+complementary) --
 *      the closest already-verified-safe all-off pattern in this
 *      codebase, reused rather than inventing a new one,
 *   2. tmr_event_sw_trigger() to commit it,
 *   3. a bounded, deliberately short 2us busy-wait (TMR2, mod-2^16-
 *      safe, same pattern used everywhere else in this project) --
 *      diagnostic-only, NOT proposed as the final design; it does
 *      briefly block inside the TMR3 ISR for every commutation while
 *      this file is in use,
 *   4. THEN the existing per-phase pos/neg/floating loop runs exactly
 *      as before, applying the new sector's real gate state.
 *
 * Closed-loop control, 15% duty, ZC, timeout, handover, the 30-degree
 * delay calculation, and Stage E16F's TMR3 fix are all unchanged.
 * Stage E19's all-low event detector (event_count[6]/duration log) is
 * kept exactly as-is -- the 2us intentional all-off window will itself
 * show up as very short (~us-scale) all-low blips at every commutation
 * now, but those are trivially distinguishable by duration from a
 * genuine multi-millisecond OCP event, so no filtering was added.
 *
 * Verdict per instruction: if step5's OCP-scale (multi-ms) events
 * disappear with this break-before-make in place, the step4->step5
 * commutation transient was the cause; if step5 still shows the same
 * ~8-10ms events, the applied step5 phase current itself is reaching
 * the OCP threshold, independent of any commutation-edge transient.
 *
 * STAGE E20B: Stage E20's break-before-make made the motor spin
 * noticeably faster and more stably, with no more fault-driven stops
 * -- consistent with the step5 OCP event having been a step4->step5
 * commutation-transient artifact, not the true step5 phase current.
 * Motor temperature was rising during that open-ended run, so this
 * file adds a short, SAFE, bounded test window -- nothing about the
 * control path changes: 2us break-before-make, 15% duty, ZC, handover,
 * timeout, and the 30-degree delay calculation are all identical to
 * Stage E20.
 *
 * The only addition: once MODE_CLOSED_LOOP is entered, the run stops
 * itself -- via the SAME stop_and_force_off() an actual fault would
 * use, guaranteeing all six gates end up forced low the same safe way
 * -- at whichever of these comes first:
 *   - 60 closed-loop commutations (closed_loop_step_count reaches 60), or
 *   - 500ms elapsed since the handover instant (TMR2 timestamp taken
 *     at handover, checked both at each commutation and at each
 *     timeout-watchdog firing, mod-2^16-safe -- so the time cap is
 *     enforced even if the run happens to be stuck waiting through a
 *     long sector rather than actively commutating).
 * stage_e23_stop_reason records which of NONE/FAULT/HANDOVER_TIMEOUT
 * (pre-existing stop paths)/MAX_COMMUTATIONS/MAX_TIME actually ended
 * the run.
 *
 * All of Stage E19/E20's diagnostics are kept: alllow_event_count[6],
 * alllow_longest_duration_us[6], alllow_log[8], closed_loop_step_count,
 * zc_count, zc_timeout_count, polarity_error_count. Purpose is
 * strictly to confirm -- briefly and safely -- whether step5's
 * 7-10ms all-low/OCP event is gone with break-before-make in place, not
 * to raise speed or duty (both unchanged from Stage E20).
 *
 * STAGE E21: Stage E20B completed 60/60 closed-loop commutations with
 * zc_timeout_count=0 and polarity_error_count=0 -- break-before-make
 * resolved the step5 OCP-scale event. Two things happen in this file,
 * per instruction:
 *
 * (1) break-before-make is now the STANDARD commutation sequence, not
 *     a bolted-on diagnostic -- apply_step() always does old-sector-
 *     off -> break interval -> new-sector-enable, in that order, with
 *     NO code path that goes back to configuring the new phase's
 *     outputs before the old ones are off (i.e. no window where old
 *     and new phases could ever be simultaneously ON). The break
 *     interval itself stays at Stage E20's BREAK_BEFORE_MAKE_US=2us
 *     baseline -- NOT re-tuned/optimized in this file.
 *
 * (2) Per-step timing characterization for evaluating heating/current/
 *     commutation-timing behavior at 15% duty:
 *       - stage_e23_sector_period_by_step[6]: min/max/sum/count of
 *         sector_period_us (commutation-to-commutation), filed under
 *         the step that just ENDED (same step_index convention the
 *         sector log already uses).
 *       - stage_e23_zc_delay_by_step[6]: min/max/sum/count of the
 *         measured ZC-to-next-commutation delay (the 30-degree value),
 *         filed under the step whose ZC produced it.
 *       - stage_e23_sector_period_overall / stage_e23_zc_delay_overall:
 *         the same min/max/sum/count structure, summed across all 6
 *         steps, so the average closed-loop sector period is directly
 *         computable (sum/count) without re-deriving it from the
 *         per-step tables.
 *       - stage_e23_alllow_ms_event_count: count of all-low events
 *         (from Stage E19's detector, kept unchanged) whose duration
 *         is >=1000us -- i.e. genuine millisecond-scale events,
 *         distinct from the ~2us break-before-make blips the detector
 *         also now sees on every commutation.
 *
 * Control conditions unchanged from Stage E20/E20B: 15% duty, handover
 * condition, ZC classification, the 30-degree ZC-to-commutation delay
 * calculation, and the 60-commutation/500ms safe-stop window. No
 * acceleration or duty ramp is added in this file.
 *
 * STAGE E22: Stage E21 completed 60/60 commutations with zc_timeout=0,
 * polarity_error=0, 0 ms-scale all-low events, overall sector period
 * avg 3573us, overall ZC-to-commutation delay avg 1778us (=49.8% of
 * sector -- confirms the 30-degree-delay symmetry assumption). Two
 * changes, per instruction:
 *
 * (1) LOGGER FIX ONLY, no control-path change: Stage E21's
 *     zc_delay_by_step[5] sum/count came back corrupted (min/max were
 *     fine) while every other step's entry was clean. Per instruction
 *     this is fixed defensively rather than chased through another
 *     hardware round-trip: stage_e23_sector_period_by_step[6] and
 *     stage_e23_zc_delay_by_step[6] (plus the _overall accumulators)
 *     are now explicitly zero-initialized at the very top of main(),
 *     before anything else runs (not left to an assumption that BSS
 *     is zeroed at reset), AND every stat_update() call site for both
 *     arrays now bounds-checks its step index (<6) before indexing,
 *     instead of trusting stage_e23_step_index's range implicitly.
 *     Neither change touches ZC classification, duty, timing, or any
 *     other control-path logic -- it only hardens the diagnostic
 *     write path itself.
 *
 * (2) Duty ramp, gated on sustained closed-loop success: startup and
 *     handover are UNCHANGED (still 15%, same handover condition).
 *     Once MODE_CLOSED_LOOP has completed 12 consecutive commutations
 *     at the current duty (every TMR3_PURPOSE_COMMUTATION firing is by
 *     construction a "good" commutation -- it only happens after a
 *     direction-correct confirmed ZC rescheduled it, so no separate
 *     success/failure bookkeeping is needed beyond counting them),
 *     duty steps up by +5%: 15 -> 20 -> 25 -> 30, holding each level
 *     for 12 commutations before advancing to the next. g_duty_ccr is
 *     recomputed from the new percentage and used by apply_step() the
 *     same way it always has been -- no other part of apply_step()
 *     changes. break-before-make (2us), ZC threshold, the 30-degree
 *     delay calculation, and the timeout condition are all unchanged
 *     at every duty level. After 12 commutations AT 30% (the top
 *     level), the run stops itself safely (does not attempt to go
 *     past 30% or toward 100k rpm in this file).
 *
 *     Per instruction, this file does NOT tolerate any fault at all
 *     during the ramp: the first zc_timeout OR polarity-error event,
 *     at any duty level, stops the run immediately (does not wait for
 *     the pre-existing FAULT_STOP_THRESHOLD=3 consecutive-fault
 *     counter, which remains in place as an unreachable backstop) --
 *     duty is never increased after a fault, per instruction.
 *
 *     Per-duty-level log, stage_e23_duty_log[] (Stage E22 had 4 levels
 *     0-3 for 15/20/25/30%; this file reduces it to 2, see the Stage
 *     E23 note below): sector_period_us {min,max,sum,count}, zc_count,
 *     timeout_count, polarity_error_count, ms-scale all-low event
 *     count, and commutations completed at that level.
 *
 * STAGE E23: Stage E22's real-hardware result showed the fault is
 * duty-dependent, not a one-time step5 artifact: 15% ran 12/12 clean
 * (timeout=0, polarity=0, ms-scale all-low=0), but 20% faulted after
 * only 3 commutations (1 polarity error, 2 ms-scale all-low events,
 * one sector as long as 12284us) -- driver protection recurring at
 * 20%. This file isolates ONE variable: the break-before-make
 * interval, doubled from Stage E20-E22's 2us baseline to 4us, with
 * EVERYTHING else held fixed -- startup/handover still 15%, ZC
 * threshold, the 30-degree delay calculation, and the timeout
 * condition are all unchanged.
 *
 * The duty ramp machinery from Stage E22 is reused but simplified to
 * exactly two levels this time (duty_levels_percent={15,20},
 * NUM_DUTY_LEVELS=2) with DIFFERENT hold counts per level
 * (duty_hold_commutations={12,24} -- 12 commutations to confirm 15%
 * is stable post-handover, matching Stage E22's own hold count, then
 * 24 commutations at 20% to give the fault every reasonable chance to
 * recur if it still exists at the doubled break interval). Since there
 * are only two levels, the existing "advance if a next level exists,
 * else stop" logic in the ramp-advance code naturally stops the run
 * once the 20% hold count is reached -- duty never goes to 25% or
 * beyond in this file, per instruction. A dedicated 300ms cap,
 * measured from the moment 20% is entered (not from handover), is
 * layered on top so a run that stalls mid-sector at 20% without
 * completing 24 commutations still stops on time; the pre-existing
 * handover-based 500ms/60-commutation window from Stage E20B remains
 * as an outer backstop, unchanged.
 *
 * Any timeout or polarity-error event still stops the run immediately
 * (Stage E22's zero-fault-tolerance policy, unchanged).
 *
 * Diagnostics recorded: closed_loop_step_count, zc_count,
 * timeout_count, polarity_error_count, per-step ms-scale all-low
 * event_count[6]/longest_duration_us[6] (Stage E19's detector,
 * unchanged), and overall sector_period {min,max,sum,count}
 * (stage_e23_sector_period_overall, already tracked since Stage E21).
 *
 * Verdict per instruction: if all-low events disappear at 4us, the
 * 2us break-before-make was insufficient specifically at 20%; if
 * ~7-10ms-class all-low events still occur at 4us, that points away
 * from a commutation-overlap transient and toward the actual 20%-duty
 * phase current reaching the OCP threshold.
 */

#include "clock_config.h"

#define PHASE_A_CHANNEL ADC_CHANNEL_0 /* PA0 */
#define PHASE_B_CHANNEL ADC_CHANNEL_4 /* PA4 */
#define PHASE_C_CHANNEL ADC_CHANNEL_5 /* PA5 */

#define PWM_ARR (96000000u / 24000u - 1u) /* 24kHz, unchanged */
#define DEAD_TIME_COUNT 53u /* unverified placeholder, see porting-plan Section 18-C/H */

#define OPEN_LOOP_DUTY_PERCENT 15u /* fixed for this whole run -- no closed-loop accel control yet */
#define ALIGN_DURATION_MS 300u
#define TIM1_TICKS_PER_US 96u
#define ADC_TRIGGER_OFFSET_TICKS 115u /* ~1.2us -- unchanged from Stage E10/E14/E15 */

#define COMMUTATION_BLANK_SCANS 2u
#define ZC_CONFIRM_COUNT 3
#define ZC_DEADBAND 8

#define TMR3_TICK_US 10u
/* Open-loop ramp: 15,13,11,9,7 ms/step, 40 steps each -- identical
 * mechanism/values to Stage E2/E6/E14/E15's ramp floor. */
static const uint32_t ramp_period_ticks[] = {1500, 1300, 1100, 900, 700};
#define NUM_RAMP_BUCKETS (sizeof(ramp_period_ticks) / sizeof(ramp_period_ticks[0]))
#define STEPS_PER_RAMP_BUCKET 40u

#define HOLD_PERIOD_TICKS 600u /* 6ms/step -- fastest step Stage E15 validated without real-motor stall */
#define MAX_STEPS_AT_TARGET 200u /* safety cap while waiting for handover at the hold rate */
#define HANDOVER_CONSECUTIVE_REQUIRED 6u

#define FAULT_STOP_THRESHOLD 3u /* consecutive zc_timeout/polarity_error events before all-low stop */

typedef enum { PH_A = 0, PH_B = 1, PH_C = 2 } phase_t;

typedef struct {
  phase_t pos, neg, floating;
} step_t;

/* Unchanged drive pattern, Stage E2 onward. */
static const step_t steps[6] = {
  {PH_A, PH_B, PH_C},
  {PH_A, PH_C, PH_B},
  {PH_B, PH_C, PH_A},
  {PH_B, PH_A, PH_C},
  {PH_C, PH_A, PH_B},
  {PH_C, PH_B, PH_A},
};

/* Adopted per Stage E15's MUX-fixed, 9-bucket (15-3ms) confirmation:
 * speed-independent, RISING/FALLING alternating exactly as listed.
 * 1=RISING, 2=FALLING (matches zc_filter_update()'s return convention). */
static const int expected_dir[6] = {1, 2, 1, 2, 1, 2};

static const tmr_channel_select_type phase_channel[3] = {
  TMR_SELECT_CHANNEL_1, TMR_SELECT_CHANNEL_2, TMR_SELECT_CHANNEL_3
};

static volatile uint16_t adc_buf[3];

typedef enum { MODE_RAMP = 0, MODE_HOLD = 1, MODE_CLOSED_LOOP = 2, MODE_STOPPED = 3 } run_mode_t;

/* Stage E23: why the run ended. */
typedef enum {
  STOP_REASON_NONE = 0,
  STOP_REASON_FAULT = 1,             /* consecutive_fault_count threshold (pre-existing, unreachable backstop) */
  STOP_REASON_HANDOVER_TIMEOUT = 2,  /* MAX_STEPS_AT_TARGET reached without handover (pre-existing) */
  STOP_REASON_MAX_COMMUTATIONS = 3,  /* 60 closed-loop commutations reached (outer safety net) */
  STOP_REASON_MAX_TIME = 4,          /* 500ms since handover reached (outer safety net) */
  STOP_REASON_FAULT_DURING_RAMP = 5, /* new: first timeout/polarity-error during the duty ramp -- immediate stop */
  STOP_REASON_RAMP_COMPLETE = 6,     /* 24 commutations completed at 20% (Stage E23's top level) -- test done */
  STOP_REASON_DUTY20_TIME_LIMIT = 7  /* new (Stage E23): 300ms since entering 20% reached without completing 24 commutations */
} stop_reason_t;
volatile stop_reason_t stage_e23_stop_reason;

#define TEST_MAX_CLOSED_LOOP_COMMUTATIONS 60u
#define TEST_MAX_TIME_SINCE_HANDOVER_US 500000u

volatile uint32_t stage_e23_handover_timestamp_us;

/* Stage E23: duty ramp constants -- struct/globals defined further
 * below, once stage_e23_stat_t itself is declared (see
 * stage_e23_duty_log[]). */
/* Stage E23: two levels only, 15% (12 commutations, confirm stable
 * post-handover) then 20% (24 commutations -- give the fault every
 * reasonable chance to recur). Duty never advances past 20% here. */
#define NUM_DUTY_LEVELS 2u
static const uint32_t duty_levels_percent[NUM_DUTY_LEVELS] = {15, 20};
static const uint32_t duty_hold_commutations[NUM_DUTY_LEVELS] = {12, 24};

#define DUTY20_TIME_LIMIT_US 300000u /* measured from the moment 20% is entered, not from handover */

/*
 * TMR2's physical counter wraps at 16 bits (established in Stage E15/
 * E16 -- see delay_us()'s chunking below and the mod-2^16 zc_delay_us
 * arithmetic elsewhere in this file). 500ms is far beyond that 65.536ms
 * window, so the 500ms cap is NOT a single subtraction -- it is
 * accumulated incrementally, one 16-bit-safe delta at a time, at every
 * commutation and every timeout-watchdog firing (both happen far more
 * often than every 65ms in every case this project has observed).
 */
static uint32_t elapsed_since_handover_us;
static uint32_t last_time_check_us;

static void update_elapsed_since_handover(void)
{
  uint32_t now = TMR2->cval;
  uint32_t delta = (uint32_t)(uint16_t)(now - last_time_check_us);
  elapsed_since_handover_us += delta;
  last_time_check_us = now;
}

/* Stage E23: dedicated 300ms clock measured from the instant 20% is
 * entered (not from handover) -- same incremental, mod-2^16-safe
 * accumulation pattern as update_elapsed_since_handover(). */
static uint32_t elapsed_since_duty20_us;
static uint32_t last_time_check_duty20_us;

static void update_elapsed_since_duty20(void)
{
  uint32_t now = TMR2->cval;
  uint32_t delta = (uint32_t)(uint16_t)(now - last_time_check_duty20_us);
  elapsed_since_duty20_us += delta;
  last_time_check_duty20_us = now;
}

/*
 * While MODE_CLOSED_LOOP, TMR3 always has exactly one of these two
 * purposes armed -- never both, never ambiguous:
 *   TMR3_PURPOSE_TIMEOUT: armed immediately after every apply_step(),
 *     waiting for the NEXT sector's ZC. If TMR3 overflows in this
 *     state, no ZC arrived in time -- that is purely a timeout event
 *     (zc_timeout_count++); it NEVER calls apply_step()/advances
 *     commutation by itself.
 *   TMR3_PURPOSE_COMMUTATION: armed only by a direction-correct
 *     confirmed ZC (schedule_next_commutation()), for exactly the
 *     measured 30-degree-equivalent delay. If TMR3 overflows in this
 *     state, that IS the scheduled commutation -- apply_step() runs,
 *     then TMR3 is immediately re-armed back to TMR3_PURPOSE_TIMEOUT
 *     for the new sector.
 * This is the only thing that changed from the first E16 draft: a
 * timeout no longer silently drives an ordinary commutation on stale
 * timing -- it is counted and re-armed as a timeout, nothing else.
 */
typedef enum { TMR3_PURPOSE_TIMEOUT = 0, TMR3_PURPOSE_COMMUTATION = 1 } tmr3_purpose_t;
volatile tmr3_purpose_t stage_e23_tmr3_purpose;

volatile run_mode_t stage_e23_mode;
volatile uint32_t stage_e23_step_index;
volatile uint32_t stage_e23_step_count;
volatile uint32_t stage_e23_consecutive_correct; /* live, during MODE_HOLD, toward handover */
volatile uint32_t stage_e23_at_target_step_count;

volatile int stage_e23_floating_phase, stage_e23_positive_phase, stage_e23_negative_phase;
volatile uint16_t stage_e23_adc_a, stage_e23_adc_b, stage_e23_adc_c;
volatile uint16_t stage_e23_floating_adc, stage_e23_positive_adc, stage_e23_negative_adc;
volatile int32_t stage_e23_floating_diff;
volatile uint32_t stage_e23_step_start_us;
volatile uint32_t stage_e23_current_step_period_us;

/* --- diagnostics requested for this stage --- */
volatile int stage_e23_handover_success;
volatile uint32_t stage_e23_handover_step_index;
volatile uint32_t stage_e23_closed_loop_step_count;
volatile uint32_t stage_e23_zc_count;
volatile uint32_t stage_e23_zc_timeout_count;
volatile uint32_t stage_e23_polarity_error_count;
volatile uint32_t stage_e23_sector_period_us_min, stage_e23_sector_period_us_max, stage_e23_sector_period_us_sum;
volatile uint32_t stage_e23_scheduled_delay_us_min, stage_e23_scheduled_delay_us_max, stage_e23_scheduled_delay_us_sum;
volatile uint32_t stage_e23_final_step;

/* --- Stage E23: per-step timing characterization --- */

typedef struct {
  uint32_t min, max, sum, count;
} stage_e23_stat_t;

stage_e23_stat_t stage_e23_sector_period_by_step[6];
stage_e23_stat_t stage_e23_zc_delay_by_step[6];

/* --- Stage E23: duty ramp per-level log --- */
typedef struct {
  stage_e23_stat_t sector_period;
  uint32_t zc_count;
  uint32_t timeout_count;
  uint32_t polarity_error_count;
  uint32_t alllow_ms_event_count;
  uint32_t commutation_count;
} stage_e23_duty_level_log_t;

stage_e23_duty_level_log_t stage_e23_duty_log[NUM_DUTY_LEVELS];
volatile uint32_t stage_e23_current_duty_level_index;
volatile uint32_t stage_e23_current_duty_percent;

static uint32_t commutations_since_duty_change;
stage_e23_stat_t stage_e23_sector_period_overall; /* sum across all 6 steps -- mean = sum/count */
stage_e23_stat_t stage_e23_zc_delay_overall;

volatile uint32_t stage_e23_alllow_ms_event_count; /* all-low events (Stage E19 detector) with duration_us >= 1000 */

static void stat_update(stage_e23_stat_t *s, uint32_t value)
{
  if (s->count == 0u || value < s->min) s->min = value;
  if (s->count == 0u || value > s->max) s->max = value;
  s->sum += value;
  s->count++;
}

/* Stage E23 logger fix: bounds-checked indexing into a 6-entry
 * per-step stat array -- diagnostic-only hardening, no control-path
 * effect (an out-of-range index is simply dropped instead of
 * indexing/writing out of bounds). */
static void stat_update_by_step(stage_e23_stat_t *arr6, uint32_t step_index, uint32_t value)
{
  if (step_index < 6u) {
    stat_update(&arr6[step_index], value);
  }
}

volatile uint32_t stage_e23_heartbeat;
volatile int stage_e23_running;
volatile int stage_e23_aligning;
volatile uint32_t stage_e23_dma_error;
volatile uint32_t stage_e23_dma_fdt_count;
volatile uint32_t stage_e23_adc_conversion_count;
volatile uint32_t stage_e23_tmr1_ch4_event_count;

static uint32_t consecutive_fault_count;
static uint32_t last_scheduled_delay_us; /* most recent good ZC-measured delay, seeds the timeout window */

#define ZC_TIMEOUT_MULTIPLIER 3u /* watchdog window = this many times the last known good delay */

/* --- Stage E23 instrumentation --- */

typedef struct {
  uint32_t step_index;
  int expected_dir;
  int pos_phase, neg_phase, floating_phase;
  int sector_decided;   /* zc_locked at capture time */
  int zc_state;         /* zc_arm_state_t at capture time */
  int zc_confirmed_sign, zc_confirm_run; /* zc_active fields at capture time */
  uint32_t blank_scans_remaining;
  uint32_t dma_fdt_count;
} stage_e23_commutation_snapshot_t;

stage_e23_commutation_snapshot_t stage_e23_first_closed_loop_snapshot;
volatile int stage_e23_first_closed_loop_snapshot_valid;

typedef struct {
  uint32_t adc_sample_count; /* post-blanking scans processed this sector */
  int32_t diff_min, diff_max;
  uint32_t armed_count;      /* 0 or 1 -- did this sector ever leave UNARMED */
  uint32_t rising_confirm_count, falling_confirm_count;
  uint16_t last_adc_a, last_adc_b, last_adc_c;
  uint16_t last_floating_adc, last_positive_adc, last_negative_adc;
  uint32_t dma_irq_count_delta; /* dma_fdt_count at timeout minus at sector start */
  uint32_t ch4_event_count_delta; /* tmr1_ch4_event_count at timeout minus at sector start -- pass criterion: >>1 */
  int zc_state_at_timeout;      /* zc_arm_state_t at the moment of timeout */
} stage_e23_first_timeout_diag_t;

stage_e23_first_timeout_diag_t stage_e23_first_timeout_diag;
volatile int stage_e23_first_timeout_diag_valid;

/* Live, per-sector accumulators -- reset every apply_step(), read out
 * into stage_e23_first_timeout_diag the first time a timeout occurs. */
static uint32_t live_adc_sample_count;
static int32_t live_diff_min, live_diff_max;
static uint32_t live_armed_count;
static uint32_t live_rising_confirm_count, live_falling_confirm_count;
static uint32_t live_sector_dma_fdt_start;
static uint32_t live_sector_ch4_event_start;

/* --- Stage E23: raw ADC/DMA/CH4 register snapshots --- */

typedef struct {
  uint32_t dma_ch1_ctrl;   /* DMA1_CHANNEL1->ctrl -- bit0 = chen */
  uint32_t dma_ch1_dtcnt;  /* DMA1_CHANNEL1->dtcnt -- remaining transfer count */
  uint32_t dma1_sts;       /* DMA1->sts -- bit1 = fdtf1 */
  uint32_t adc1_ctrl2;     /* ADC1->ctrl2 -- bit20 = octen, bit8 = ocdmaen */
  uint32_t adc1_osq1;      /* ADC1->osq1 -- bits[23:20] = oclen (sequence length-1) */
  uint32_t tmr1_cctrl;     /* TMR1->cctrl -- bit12 = c4en */
  uint32_t tmr1_cm2;       /* TMR1->cm2 -- c4octrl (CH4 mode) */
  uint32_t tmr1_c4dt;      /* TMR1->c4dt -- CCR4 */
  uint32_t ch4_event_count;
  uint32_t dma_fdt_count;
} stage_e23_reg_snapshot_t;

stage_e23_reg_snapshot_t stage_e23_regs_after_commutation;
volatile int stage_e23_regs_after_commutation_valid;

stage_e23_reg_snapshot_t stage_e23_regs_after_first_dma_irq;
volatile int stage_e23_regs_after_first_dma_irq_valid;

static void capture_reg_snapshot(stage_e23_reg_snapshot_t *s)
{
  s->dma_ch1_ctrl = DMA1_CHANNEL1->ctrl;
  s->dma_ch1_dtcnt = DMA1_CHANNEL1->dtcnt;
  s->dma1_sts = DMA1->sts;
  s->adc1_ctrl2 = ADC1->ctrl2;
  s->adc1_osq1 = ADC1->osq1;
  s->tmr1_cctrl = TMR1->cctrl;
  s->tmr1_cm2 = TMR1->cm2;
  s->tmr1_c4dt = TMR1->c4dt;
  s->ch4_event_count = stage_e23_tmr1_ch4_event_count;
  s->dma_fdt_count = stage_e23_dma_fdt_count;
}

/* --- Stage E23: TMR1-specific register dump (ctrl1.tmren, cval, pr,
 * iden, ists -- none of which Stage E16C captured) --- */

typedef struct {
  uint32_t ctrl1;  /* bit0 = tmren, counter enable */
  uint32_t cval;   /* live counter value */
  uint32_t pr;     /* period register (ARR) */
  uint32_t iden;   /* interrupt/DMA enable -- bit4 = c4ien, bit12 = c4den */
  uint32_t ists;   /* status -- bit4 = c4if */
  uint32_t cctrl;  /* bit12 = c4en (repeated here for convenience alongside the rest) */
  uint32_t cm2;    /* c4octrl (CH4 mode) */
  uint32_t c4dt;   /* CCR4 */
  uint32_t swevt;  /* software event trigger -- read only, never written by this file */
} stage_e23_tmr1_regs_t;

stage_e23_tmr1_regs_t stage_e23_tmr1_regs_after_commutation;
volatile int stage_e23_tmr1_regs_after_commutation_valid;

stage_e23_tmr1_regs_t stage_e23_tmr1_regs_at_timeout;
volatile int stage_e23_tmr1_regs_at_timeout_valid;

static void capture_tmr1_regs(stage_e23_tmr1_regs_t *s)
{
  s->ctrl1 = TMR1->ctrl1;
  s->cval = TMR1->cval;
  s->pr = TMR1->pr;
  s->iden = TMR1->iden;
  s->ists = TMR1->ists;
  s->cctrl = TMR1->cctrl;
  s->cm2 = TMR1->cm2;
  s->c4dt = TMR1->c4dt;
  s->swevt = TMR1->swevt;
}

/*
 * Non-blocking TMR1->cval progression check: cval_0 captured at the
 * same instant as stage_e23_tmr1_regs_after_commutation; cval_1
 * captured at the first DMA FDT IRQ afterward (the same naturally-
 * occurring IRQ Stage E16C's regs_after_first_dma_irq already uses --
 * no new busy-wait). Both carry their own TMR2 timestamp so the real
 * elapsed time (and therefore the expected tick delta, ~96 ticks/us)
 * can be computed instead of assumed.
 */
typedef struct {
  uint32_t cval_0;
  uint32_t cval_0_timestamp_us;
  uint32_t cval_1;
  uint32_t cval_1_timestamp_us;
  int valid; /* both points captured */
} stage_e23_cval_progression_t;

stage_e23_cval_progression_t stage_e23_cval_progression;

/* --- Stage E23: TMR3 one-shot re-arm sequence logging --- */

typedef struct {
  uint32_t arm_timestamp_us; /* TMR2->cval at the start of tmr3_arm_oneshot() */
  int purpose;                /* tmr3_purpose_t being armed */
  uint32_t requested_delay_us;
  uint32_t ists_before_sw_trigger;
  uint32_t ists_after_sw_trigger;
  uint32_t ists_after_clear; /* Stage E23: right after the fix's required re-clear (step 7) */
  uint32_t ists_before_resume;
} stage_e23_arm_log_t;

stage_e23_arm_log_t stage_e23_last_arm_log; /* overwritten every tmr3_arm_oneshot() call */

stage_e23_arm_log_t stage_e23_first_commutation_arm_log;
volatile int stage_e23_first_commutation_arm_log_valid;
stage_e23_arm_log_t stage_e23_first_timeout_arm_log;
volatile int stage_e23_first_timeout_arm_log_valid;

typedef struct {
  uint32_t isr_entry_timestamp_us;
  int purpose;
  uint32_t elapsed_us; /* isr_entry_timestamp_us - the matching arm log's arm_timestamp_us */
  uint32_t ists;
  uint32_t cval;
  uint32_t pr;
} stage_e23_isr_entry_log_t;

stage_e23_isr_entry_log_t stage_e23_last_isr_entry_log; /* overwritten every TMR3 IRQ entry */

stage_e23_isr_entry_log_t stage_e23_first_commutation_isr_entry;
volatile int stage_e23_first_commutation_isr_entry_valid;
stage_e23_isr_entry_log_t stage_e23_first_timeout_isr_entry;
volatile int stage_e23_first_timeout_isr_entry_valid;

/* --- Stage E23: closed-loop stability ring buffers --- */

#define SECTOR_LOG_SIZE 64u
#define TIMEOUT_LOG_SIZE 32u

typedef struct {
  uint32_t step_index;
  uint32_t commutation_timestamp_us;
  uint32_t zc_timestamp_us;
  uint32_t sector_period_us;
  uint32_t scheduled_delay_us;
  int expected_dir;
  int zc_confirmed;
  int timeout_occurred;
  uint32_t consecutive_fault_count_at_confirm;
  uint16_t floating_adc_at_zc;
  uint16_t positive_adc_at_zc;
  /* --- Stage E23: per-sector ADC aggregation, post-blanking-to-ZC --- */
  uint16_t positive_adc_min, positive_adc_max; uint32_t positive_adc_sum;
  uint16_t negative_adc_min, negative_adc_max; uint32_t negative_adc_sum;
  uint16_t floating_adc_min, floating_adc_max;
  int32_t diff_min, diff_max;
  uint32_t sample_count;
  uint16_t negative_adc_at_zc;
  int32_t diff_at_zc;
} stage_e23_sector_entry_t;

stage_e23_sector_entry_t stage_e23_sector_log[SECTOR_LOG_SIZE];
volatile uint32_t stage_e23_sector_log_count;

typedef struct {
  uint32_t current_step;
  uint32_t time_since_commutation_us;
  uint32_t expected_timeout_us;
  uint32_t last_valid_sector_period_us;
  uint32_t last_valid_scheduled_delay_us;
  int zc_state;
  int32_t diff_min, diff_max;
} stage_e23_timeout_entry_t;

stage_e23_timeout_entry_t stage_e23_timeout_log[TIMEOUT_LOG_SIZE];
volatile uint32_t stage_e23_timeout_log_count;

static uint32_t last_valid_sector_period_us;

/* Stashed by the DMA IRQ (ZC confirm) for the TMR3 IRQ's COMMUTATION
 * branch to read once apply_step() has run -- see file header. */
static uint32_t pending_zc_timestamp_us;
static uint16_t pending_floating_adc_at_zc, pending_positive_adc_at_zc;
static uint32_t pending_timeout_count_this_sector;
static uint32_t pending_consecutive_fault_count;
static uint16_t pending_negative_adc_at_zc; /* Stage E23 */
static int32_t pending_diff_at_zc;           /* Stage E23 */
static uint16_t pending_positive_adc_min, pending_positive_adc_max; static uint32_t pending_positive_adc_sum;
static uint16_t pending_negative_adc_min, pending_negative_adc_max; static uint32_t pending_negative_adc_sum;
static uint16_t pending_floating_adc_min, pending_floating_adc_max;
static int32_t pending_agg_diff_min, pending_agg_diff_max;
static uint32_t pending_agg_sample_count;

/* Live, per-sector: how many timeouts happened before this sector's
 * eventual confirm. Reset in apply_step(), incremented in the TIMEOUT
 * branch of TMR3_GLOBAL_IRQHandler. */
static uint32_t live_timeout_count_this_sector;

/* Live, per-sector ADC aggregation (Stage E23) -- reset in apply_step(),
 * updated every post-blanking scan, stashed into pending_* at ZC
 * confirm the same way the rest of this sector's data already is. */
static uint16_t live_pos_adc_min, live_pos_adc_max; static uint32_t live_pos_adc_sum;
static uint16_t live_neg_adc_min, live_neg_adc_max; static uint32_t live_neg_adc_sum;
static uint16_t live_flo_adc_min, live_flo_adc_max;

/* --- Stage E23: short raw trace, first closed-loop occurrence of
 * step1/step3/step5 only (index 0/1/2). --- */
#define STEP_TRACE_GROUPS 3u
#define STEP_TRACE_LEN 16u

typedef struct {
  uint16_t positive_adc;
  uint16_t negative_adc;
  uint16_t floating_adc;
} stage_e23_trace_sample_t;

stage_e23_trace_sample_t stage_e23_step_trace[STEP_TRACE_GROUPS][STEP_TRACE_LEN];
volatile uint32_t stage_e23_step_trace_fill_count[STEP_TRACE_GROUPS];
volatile int stage_e23_step_trace_captured[STEP_TRACE_GROUPS];

static int trace_active_group = -1; /* -1 = not currently capturing */

static int step_trace_group_for(uint32_t step_index)
{
  if (step_index == 1u) return 0;
  if (step_index == 3u) return 1;
  if (step_index == 5u) return 2;
  return -1;
}

/* --- Stage E23: 3-phase all-low (suspected OCP/Hi-Z) event detector.
 * DIAGNOSTIC ONLY -- ALLLOW_THRESHOLD is never read by any control-path
 * code in this file. --- */

#define ALLLOW_THRESHOLD 100u
#define ALLLOW_LOG_SIZE 8u

typedef struct {
  uint32_t start_timestamp_us;
  uint32_t step_index;
  uint32_t time_since_commutation_us;
  uint32_t end_timestamp_us;
  uint32_t duration_us;
  uint32_t number_of_adc_scans;
  uint16_t adc_a_min, adc_a_max;
  uint16_t adc_b_min, adc_b_max;
  uint16_t adc_c_min, adc_c_max;
} stage_e23_alllow_event_t;

stage_e23_alllow_event_t stage_e23_alllow_log[ALLLOW_LOG_SIZE];
volatile uint32_t stage_e23_alllow_log_count;

volatile uint32_t stage_e23_alllow_event_count[6];
volatile uint32_t stage_e23_alllow_longest_duration_us[6];

static int alllow_active;
static uint32_t alllow_start_us;
static uint32_t alllow_step_index;
static uint32_t alllow_commutation_us;
static uint32_t alllow_scan_count;
static uint16_t alllow_a_min, alllow_a_max, alllow_b_min, alllow_b_max, alllow_c_min, alllow_c_max;

static void alllow_finish_event(uint32_t end_us)
{
  uint32_t duration_us = (uint32_t)(uint16_t)(end_us - alllow_start_us);
  uint32_t step = alllow_step_index;

  stage_e23_alllow_event_count[step]++;
  if (duration_us > stage_e23_alllow_longest_duration_us[step]) {
    stage_e23_alllow_longest_duration_us[step] = duration_us;
  }
  /* Stage E23: distinguish genuine ms-scale events from the ~2us
   * break-before-make blip the detector now also sees every commutation. */
  if (duration_us >= 1000u) {
    stage_e23_alllow_ms_event_count++;
    if (stage_e23_current_duty_level_index < NUM_DUTY_LEVELS) {
      stage_e23_duty_log[stage_e23_current_duty_level_index].alllow_ms_event_count++;
    }
  }

  if (stage_e23_alllow_log_count < ALLLOW_LOG_SIZE) {
    stage_e23_alllow_event_t *ev = &stage_e23_alllow_log[stage_e23_alllow_log_count];
    ev->start_timestamp_us = alllow_start_us;
    ev->step_index = step;
    ev->time_since_commutation_us = (uint32_t)(uint16_t)(alllow_start_us - alllow_commutation_us);
    ev->end_timestamp_us = end_us;
    ev->duration_us = duration_us;
    ev->number_of_adc_scans = alllow_scan_count;
    ev->adc_a_min = alllow_a_min; ev->adc_a_max = alllow_a_max;
    ev->adc_b_min = alllow_b_min; ev->adc_b_max = alllow_b_max;
    ev->adc_c_min = alllow_c_min; ev->adc_c_max = alllow_c_max;
    stage_e23_alllow_log_count++;
  }

  alllow_active = 0;
}

/* Called on EVERY closed-loop DMA sample, independent of blanking/
 * zc_locked -- observes the driver's actual output state, not the ZC
 * state machine. */
static void alllow_update(uint16_t a, uint16_t b, uint16_t c)
{
  int low = (a < ALLLOW_THRESHOLD) && (b < ALLLOW_THRESHOLD) && (c < ALLLOW_THRESHOLD);

  if (low) {
    if (!alllow_active) {
      alllow_active = 1;
      alllow_start_us = TMR2->cval;
      alllow_step_index = stage_e23_step_index;
      alllow_commutation_us = stage_e23_step_start_us;
      alllow_scan_count = 0;
      alllow_a_min = alllow_a_max = a;
      alllow_b_min = alllow_b_max = b;
      alllow_c_min = alllow_c_max = c;
    }
    alllow_scan_count++;
    if (a < alllow_a_min) alllow_a_min = a;
    if (a > alllow_a_max) alllow_a_max = a;
    if (b < alllow_b_min) alllow_b_min = b;
    if (b > alllow_b_max) alllow_b_max = b;
    if (c < alllow_c_min) alllow_c_min = c;
    if (c > alllow_c_max) alllow_c_max = c;
  } else {
    if (alllow_active) {
      alllow_finish_event(TMR2->cval);
    }
  }
}

void _init(void) {}
void _fini(void) {}

typedef struct {
  int confirmed_sign;
  int confirm_run;
} zc_filter_t;

typedef enum { ZC_UNARMED = 0, ZC_ARMED_RISING = 1, ZC_ARMED_FALLING = 2 } zc_arm_state_t;

static volatile zc_filter_t zc_active;
static volatile zc_arm_state_t zc_arm_state;
static volatile int zc_locked;
static volatile uint32_t blank_scans_remaining;

/* Identical to Stage E15's zc_filter_update() -- not touched. */
static int zc_filter_update(volatile zc_filter_t *f, int32_t diff)
{
  int qualifies, new_sign;

  if (f->confirmed_sign <= 0) {
    new_sign = 1;
    qualifies = (diff >= ZC_DEADBAND);
  } else {
    new_sign = -1;
    qualifies = (diff <= -ZC_DEADBAND);
  }

  if (qualifies) {
    if (f->confirm_run < ZC_CONFIRM_COUNT) f->confirm_run++;
  } else {
    f->confirm_run = 0;
  }

  if (f->confirm_run >= ZC_CONFIRM_COUNT) {
    f->confirm_run = 0;
    int prev = f->confirmed_sign;
    f->confirmed_sign = new_sign;
    if (prev != new_sign) return new_sign > 0 ? 1 : 2;
  }
  return 0;
}

static void gate_pins_force_off(void)
{
  gpio_init_type g;

  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

  gpio_bits_reset(GPIOA, GPIO_PINS_7 | GPIO_PINS_8 | GPIO_PINS_9 | GPIO_PINS_10);
  gpio_bits_reset(GPIOB, GPIO_PINS_0 | GPIO_PINS_1);

  gpio_default_para_init(&g);
  g.gpio_mode = GPIO_MODE_OUTPUT;
  g.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  g.gpio_pull = GPIO_PULL_NONE;
  g.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;

  g.gpio_pins = GPIO_PINS_7 | GPIO_PINS_8 | GPIO_PINS_9 | GPIO_PINS_10;
  gpio_init(GPIOA, &g);
  g.gpio_pins = GPIO_PINS_0 | GPIO_PINS_1;
  gpio_init(GPIOB, &g);

  gpio_bits_reset(GPIOA, GPIO_PINS_7 | GPIO_PINS_8 | GPIO_PINS_9 | GPIO_PINS_10);
  gpio_bits_reset(GPIOB, GPIO_PINS_0 | GPIO_PINS_1);
}

static void tim1_pins_to_af(void)
{
  gpio_init_type g;

  gpio_default_para_init(&g);
  g.gpio_mode = GPIO_MODE_MUX;
  g.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  g.gpio_pull = GPIO_PULL_NONE;
  g.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;

  g.gpio_pins = GPIO_PINS_7 | GPIO_PINS_8 | GPIO_PINS_9 | GPIO_PINS_10;
  gpio_init(GPIOA, &g);
  g.gpio_pins = GPIO_PINS_0 | GPIO_PINS_1;
  gpio_init(GPIOB, &g);

  /* MUX_2 for all six pins -- Stage E15D confirmed fix. */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE8, GPIO_MUX_2);  /* CH1  */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE7, GPIO_MUX_2);  /* CH1C */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE9, GPIO_MUX_2);  /* CH2  */
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE0, GPIO_MUX_2);  /* CH2C */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE10, GPIO_MUX_2); /* CH3  */
  gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE1, GPIO_MUX_2);  /* CH3C */
}

static void tim1_init(void)
{
  tmr_brkdt_config_type brkdt;

  crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);

  tmr_base_init(TMR1, (uint16_t)PWM_ARR, 0);
  tmr_cnt_dir_set(TMR1, TMR_COUNT_UP);

  tmr_brkdt_default_para_init(&brkdt);
  brkdt.brk_enable = FALSE;
  brkdt.auto_output_enable = TRUE;
  brkdt.deadtime = DEAD_TIME_COUNT;
  brkdt.fcsodis_state = TRUE;
  brkdt.fcsoen_state = TRUE;
  brkdt.brk_polarity = TMR_BRK_INPUT_ACTIVE_HIGH;
  brkdt.wp_level = TMR_WP_OFF;
  tmr_brkdt_config(TMR1, &brkdt);

  tmr_channel_buffer_enable(TMR1, TRUE);
}

static uint16_t g_duty_ccr;

/* Verbatim from Stage E10/E14/E15 -- CH4 trigger mechanism untouched. */
static void tim1_adc_trigger_config(void)
{
  tmr_output_config_type oc;

  tmr_output_default_para_init(&oc);
  oc.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_B;
  oc.oc_output_state = TRUE;
  oc.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  oc.oc_idle_state = FALSE;
  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_4, &oc);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_4, ADC_TRIGGER_OFFSET_TICKS);
  tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_4, TRUE);
  tmr_event_sw_trigger(TMR1, TMR_OVERFLOW_SWTRIG | TMR_HALL_SWTRIG);

  nvic_irq_enable(TMR1_CH_IRQn, 2, 0);
  tmr_interrupt_enable(TMR1, TMR_C4_INT, TRUE);
}

void TMR1_CH_IRQHandler(void)
{
  if (tmr_flag_get(TMR1, TMR_C4_FLAG) != RESET) {
    tmr_flag_clear(TMR1, TMR_C4_FLAG);
    stage_e23_tmr1_ch4_event_count++;
  }
}

/* Drive pattern (steps[] pos/neg/floating table) unchanged since
 * Stage E15. */
static void delay_us(uint32_t us); /* forward decl -- definition is below, break-before-make needs it here */

#define BREAK_BEFORE_MAKE_US 4u /* Stage E23: doubled from Stage E20-E22's 2us baseline -- the ONE variable under test */

/*
 * Standard commutation step 1/3: force ALL THREE phases off (the SAME
 * "floating" all-off configuration this function already applies to
 * the floating role every sector) and commit it. Called by
 * apply_step() BEFORE any new-sector output is configured, so the old
 * sector's drive is fully off before the break interval even starts --
 * there is no path back to configuring a new phase's outputs first.
 */
static void force_all_gates_off_via_tmr1(void)
{
  tmr_output_config_type oc;

  for (int p = 0; p < 3; p++) {
    tmr_output_default_para_init(&oc);
    oc.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    oc.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    oc.oc_mode = TMR_OUTPUT_CONTROL_FORCE_LOW;
    oc.oc_output_state = FALSE;
    oc.oc_idle_state = FALSE;
    oc.occ_output_state = FALSE;
    oc.occ_idle_state = FALSE;
    tmr_output_channel_config(TMR1, phase_channel[p], &oc);
  }

  tmr_event_sw_trigger(TMR1, TMR_OVERFLOW_SWTRIG | TMR_HALL_SWTRIG);
}

static void apply_step(int idx, uint16_t duty_ccr)
{
  const step_t *s = &steps[idx];
  tmr_output_config_type oc;

  /*
   * Standard commutation sequence (Stage E23): old sector off -> break
   * interval -> new sector enable, in that fixed order, every time --
   * step 1/3 (old-sector-off) and step 2/3 (break) here; step 3/3
   * (new-sector-enable) is the per-phase loop below. No old/new phase
   * is ever simultaneously ON.
   */
  force_all_gates_off_via_tmr1();
  delay_us(BREAK_BEFORE_MAKE_US);

  for (int p = 0; p < 3; p++) {
    tmr_output_default_para_init(&oc);
    oc.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    oc.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;

    if (p == s->pos) {
      oc.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
      oc.oc_output_state = TRUE;
      oc.oc_idle_state = FALSE;
      oc.occ_output_state = TRUE;
      oc.occ_idle_state = FALSE;
      tmr_output_channel_config(TMR1, phase_channel[p], &oc);
      tmr_channel_value_set(TMR1, phase_channel[p], duty_ccr);
    } else if (p == s->neg) {
      oc.oc_mode = TMR_OUTPUT_CONTROL_FORCE_LOW;
      oc.oc_output_state = TRUE;
      oc.oc_idle_state = FALSE;
      oc.occ_output_state = TRUE;
      oc.occ_idle_state = FALSE;
      tmr_output_channel_config(TMR1, phase_channel[p], &oc);
    } else {
      oc.oc_mode = TMR_OUTPUT_CONTROL_FORCE_LOW;
      oc.oc_output_state = FALSE;
      oc.oc_idle_state = FALSE;
      oc.occ_output_state = FALSE;
      oc.occ_idle_state = FALSE;
      tmr_output_channel_config(TMR1, phase_channel[p], &oc);
    }
  }

  tmr_event_sw_trigger(TMR1, TMR_OVERFLOW_SWTRIG | TMR_HALL_SWTRIG);

  stage_e23_step_index = (uint32_t)idx;
  stage_e23_floating_phase = (int)s->floating;
  stage_e23_positive_phase = (int)s->pos;
  stage_e23_negative_phase = (int)s->neg;

  zc_arm_state = ZC_UNARMED;
  zc_locked = 0;
  blank_scans_remaining = COMMUTATION_BLANK_SCANS;
  stage_e23_step_start_us = TMR2->cval;

  /* Stage E23 instrumentation: fresh per-sector accumulators. */
  live_adc_sample_count = 0;
  live_diff_min = 0x7fffffff;
  live_diff_max = -0x7fffffff - 1;
  live_armed_count = 0;
  live_rising_confirm_count = 0;
  live_falling_confirm_count = 0;
  live_sector_dma_fdt_start = stage_e23_dma_fdt_count;
  live_sector_ch4_event_start = stage_e23_tmr1_ch4_event_count;
  live_timeout_count_this_sector = 0;

  /* Stage E23: fresh per-sector ADC aggregation. */
  live_pos_adc_min = 0xffffu; live_pos_adc_max = 0; live_pos_adc_sum = 0;
  live_neg_adc_min = 0xffffu; live_neg_adc_max = 0; live_neg_adc_sum = 0;
  live_flo_adc_min = 0xffffu; live_flo_adc_max = 0;

  /* Stage E23: start a trace only for the FIRST closed-loop occurrence
   * of step1/step3/step5, one-shot per group. */
  if (stage_e23_mode == MODE_CLOSED_LOOP) {
    int g = step_trace_group_for((uint32_t)idx);
    if (g >= 0 && !stage_e23_step_trace_captured[g]) {
      trace_active_group = g;
      stage_e23_step_trace_fill_count[g] = 0;
    } else {
      trace_active_group = -1;
    }
  } else {
    trace_active_group = -1;
  }
}

static void stop_and_force_off(stop_reason_t reason)
{
  /* Stage E23: close out any all-low event still active so nothing is
   * left dangling. */
  if (alllow_active) {
    alllow_finish_event(TMR2->cval);
  }

  tmr_output_enable(TMR1, FALSE);
  tmr_counter_enable(TMR1, FALSE);
  tmr_counter_enable(TMR3, FALSE);
  gate_pins_force_off();
  stage_e23_running = 0;
  stage_e23_mode = MODE_STOPPED;
  stage_e23_final_step = stage_e23_step_index;
  stage_e23_stop_reason = reason;
}

/*
 * Stage E23 FIX: Stage E16E's real hardware log confirmed
 * tmr_event_sw_trigger(TMR3, TMR_OVERFLOW_SWTRIG) sets TMR3's own OVF
 * flag immediately (ists_before=0x1e -> ists_after=0x1f), and because
 * that flag was never cleared afterward, the already-enabled overflow
 * interrupt fired within microseconds on the stale flag instead of
 * waiting for the programmed delay -- root cause of every closed-loop
 * stall since Stage E16. Sequence below per instruction; step 7 (the
 * required re-clear after the software trigger) is the actual fix --
 * everything else here was already present in some form.
 */
static void tmr3_arm_oneshot(uint32_t us, tmr3_purpose_t purpose)
{
  uint32_t requested_delay_us = us;
  uint32_t arm_timestamp_us = TMR2->cval;

  uint32_t ticks = us / TMR3_TICK_US;
  if (ticks < 1u) ticks = 1u;

  tmr_counter_enable(TMR3, FALSE);                    /* 1. stop */
  tmr_interrupt_enable(TMR3, TMR_OVF_INT, FALSE);      /* 2. IRQ disable */
  tmr_flag_clear(TMR3, TMR_OVF_FLAG);                  /* 3. OVF flag clear */
  tmr_counter_value_set(TMR3, 0);                      /* 4. CNT=0 */
  TMR3->pr = ticks - 1u;                               /* 5. PR=delay_ticks-1 */

  uint32_t ists_before_sw_trigger = TMR3->ists;
  tmr_event_sw_trigger(TMR3, TMR_OVERFLOW_SWTRIG);     /* 6. SW update event */
  uint32_t ists_after_sw_trigger = TMR3->ists;

  tmr_flag_clear(TMR3, TMR_OVF_FLAG);                  /* 7. REQUIRED: clear the flag the SW trigger just set */
  uint32_t ists_after_clear = TMR3->ists;

  NVIC_ClearPendingIRQ(TMR3_GLOBAL_IRQn);              /* 8. clear any latched NVIC pending bit, belt-and-suspenders */

  stage_e23_tmr3_purpose = purpose;                   /* 9. set purpose */

  uint32_t ists_before_resume = TMR3->ists;

  tmr_interrupt_enable(TMR3, TMR_OVF_INT, TRUE);       /* 10. IRQ re-enable */
  tmr_counter_enable(TMR3, TRUE);                      /* 11. resume */

  stage_e23_current_step_period_us = ticks * TMR3_TICK_US;

  stage_e23_last_arm_log.arm_timestamp_us = arm_timestamp_us;
  stage_e23_last_arm_log.purpose = (int)purpose;
  stage_e23_last_arm_log.requested_delay_us = requested_delay_us;
  stage_e23_last_arm_log.ists_before_sw_trigger = ists_before_sw_trigger;
  stage_e23_last_arm_log.ists_after_sw_trigger = ists_after_sw_trigger;
  stage_e23_last_arm_log.ists_after_clear = ists_after_clear;
  stage_e23_last_arm_log.ists_before_resume = ists_before_resume;

  if (purpose == TMR3_PURPOSE_COMMUTATION && !stage_e23_first_commutation_arm_log_valid) {
    stage_e23_first_commutation_arm_log = stage_e23_last_arm_log;
    stage_e23_first_commutation_arm_log_valid = 1;
  }
  if (purpose == TMR3_PURPOSE_TIMEOUT && !stage_e23_first_timeout_arm_log_valid) {
    stage_e23_first_timeout_arm_log = stage_e23_last_arm_log;
    stage_e23_first_timeout_arm_log_valid = 1;
  }
}

/*
 * Armed ONLY by a direction-correct confirmed ZC, for exactly the
 * measured delay -- the next TMR3 overflow IS the real, scheduled
 * commutation (see tmr3_purpose_t comment above).
 */
static void schedule_next_commutation(uint32_t delay_us)
{
  tmr3_arm_oneshot(delay_us, TMR3_PURPOSE_COMMUTATION);
  last_scheduled_delay_us = delay_us;
}

/*
 * Armed right after every apply_step() while MODE_CLOSED_LOOP, as a
 * pure watchdog for "no ZC arrived in time" -- its overflow NEVER
 * calls apply_step() (see tmr3_purpose_t comment above). Window is a
 * generous multiple of the last known-good delay so normal jitter
 * doesn't false-trigger it.
 */
static void arm_timeout_watchdog(void)
{
  uint32_t base_us = last_scheduled_delay_us ? last_scheduled_delay_us
                                              : (HOLD_PERIOD_TICKS * TMR3_TICK_US);
  tmr3_arm_oneshot(base_us * ZC_TIMEOUT_MULTIPLIER, TMR3_PURPOSE_TIMEOUT);
}

static void check_fault_stop(void)
{
  if (consecutive_fault_count >= FAULT_STOP_THRESHOLD) {
    stop_and_force_off(STOP_REASON_FAULT);
  }
}

static uint32_t bucket_step_count;

static void start_ramp_bucket(uint32_t idx)
{
  uint32_t period_ticks = ramp_period_ticks[idx];
  stage_e23_current_step_period_us = period_ticks * TMR3_TICK_US;
  bucket_step_count = 0;
  TMR3->pr = period_ticks - 1u;
}

static void enter_hold(void)
{
  stage_e23_mode = MODE_HOLD;
  stage_e23_current_step_period_us = HOLD_PERIOD_TICKS * TMR3_TICK_US;
  stage_e23_at_target_step_count = 0;
  stage_e23_consecutive_correct = 0;
  TMR3->pr = HOLD_PERIOD_TICKS - 1u;
}

static uint32_t ramp_bucket_index;

void TMR3_GLOBAL_IRQHandler(void)
{
  /*
   * Stage E23 instrumentation: logged BEFORE the flag check/early
   * return below, so a spurious entry (e.g. the SW-trigger-sets-the-
   * flag-immediately hypothesis) is captured too, not filtered out
   * first. Purely reads -- ists/cval/pr are not written here.
   */
  {
    uint32_t entry_ts = TMR2->cval;
    uint32_t ists_now = TMR3->ists;
    uint32_t cval_now = TMR3->cval;
    uint32_t pr_now = TMR3->pr;
    tmr3_purpose_t purpose_now = stage_e23_tmr3_purpose;
    uint32_t elapsed = (uint32_t)(uint16_t)(entry_ts - stage_e23_last_arm_log.arm_timestamp_us);

    stage_e23_last_isr_entry_log.isr_entry_timestamp_us = entry_ts;
    stage_e23_last_isr_entry_log.purpose = (int)purpose_now;
    stage_e23_last_isr_entry_log.elapsed_us = elapsed;
    stage_e23_last_isr_entry_log.ists = ists_now;
    stage_e23_last_isr_entry_log.cval = cval_now;
    stage_e23_last_isr_entry_log.pr = pr_now;

    /* Gated on MODE_CLOSED_LOOP: tmr3_purpose is only meaningful once
     * closed-loop starts (it defaults to TMR3_PURPOSE_TIMEOUT=0 at
     * boot), so without this gate every ordinary MODE_RAMP/MODE_HOLD
     * overflow would spuriously latch as the "first timeout" entry. */
    if (stage_e23_mode == MODE_CLOSED_LOOP) {
      if (purpose_now == TMR3_PURPOSE_COMMUTATION && !stage_e23_first_commutation_isr_entry_valid) {
        stage_e23_first_commutation_isr_entry = stage_e23_last_isr_entry_log;
        stage_e23_first_commutation_isr_entry_valid = 1;
      }
      if (purpose_now == TMR3_PURPOSE_TIMEOUT && !stage_e23_first_timeout_isr_entry_valid) {
        stage_e23_first_timeout_isr_entry = stage_e23_last_isr_entry_log;
        stage_e23_first_timeout_isr_entry_valid = 1;
      }
    }
  }

  if (tmr_flag_get(TMR3, TMR_OVF_FLAG) == RESET) return;
  tmr_flag_clear(TMR3, TMR_OVF_FLAG);

  if (!stage_e23_running) return;

  if (stage_e23_mode == MODE_RAMP) {
    int next = (int)((stage_e23_step_index + 1u) % 6u);
    apply_step(next, g_duty_ccr);
    stage_e23_step_count++;
    bucket_step_count++;

    if (bucket_step_count >= STEPS_PER_RAMP_BUCKET) {
      ramp_bucket_index++;
      if (ramp_bucket_index >= NUM_RAMP_BUCKETS) {
        enter_hold();
      } else {
        start_ramp_bucket(ramp_bucket_index);
      }
    }
    return;
  }

  if (stage_e23_mode == MODE_HOLD) {
    int next = (int)((stage_e23_step_index + 1u) % 6u);
    apply_step(next, g_duty_ccr);
    stage_e23_step_count++;
    stage_e23_at_target_step_count++;

    if (stage_e23_at_target_step_count >= MAX_STEPS_AT_TARGET) {
      /* Handover never achieved within the safety cap -- stop, leave
       * handover_success at 0. */
      stop_and_force_off(STOP_REASON_HANDOVER_TIMEOUT);
    }
    return;
  }

  if (stage_e23_mode == MODE_CLOSED_LOOP) {
    if (stage_e23_tmr3_purpose == TMR3_PURPOSE_TIMEOUT) {
      /*
       * Pure watchdog firing: no ZC arrived in time. NOT a
       * commutation -- apply_step() is NOT called, step_index/
       * step_count do not advance. Count the fault and keep waiting
       * (re-arm another timeout window) unless the fault threshold
       * stops the motor.
       */
      stage_e23_zc_timeout_count++;
      live_timeout_count_this_sector++;
      if (stage_e23_current_duty_level_index < NUM_DUTY_LEVELS) {
        stage_e23_duty_log[stage_e23_current_duty_level_index].timeout_count++;
      }

      /* Stage E23: append to the timeout ring buffer (capped, not
       * wrapping -- stops recording detail past TIMEOUT_LOG_SIZE
       * entries, zc_timeout_count itself keeps counting normally). */
      if (stage_e23_timeout_log_count < TIMEOUT_LOG_SIZE) {
        stage_e23_timeout_entry_t *te = &stage_e23_timeout_log[stage_e23_timeout_log_count];
        uint32_t now_us = TMR2->cval;
        te->current_step = stage_e23_step_index;
        te->time_since_commutation_us = (uint32_t)(uint16_t)(now_us - stage_e23_step_start_us);
        te->expected_timeout_us = stage_e23_last_arm_log.requested_delay_us;
        te->last_valid_sector_period_us = last_valid_sector_period_us;
        te->last_valid_scheduled_delay_us = last_scheduled_delay_us;
        te->zc_state = (int)zc_arm_state;
        te->diff_min = live_diff_min;
        te->diff_max = live_diff_max;
        stage_e23_timeout_log_count++;
      }

      /* Stage E23 instrumentation: capture the FIRST closed-loop
       * timeout's full sector picture, before anything below re-arms
       * or resets it. */
      if (!stage_e23_first_timeout_diag_valid) {
        stage_e23_first_timeout_diag_t *d = &stage_e23_first_timeout_diag;
        d->adc_sample_count = live_adc_sample_count;
        d->diff_min = live_diff_min;
        d->diff_max = live_diff_max;
        d->armed_count = live_armed_count;
        d->rising_confirm_count = live_rising_confirm_count;
        d->falling_confirm_count = live_falling_confirm_count;
        d->last_adc_a = stage_e23_adc_a;
        d->last_adc_b = stage_e23_adc_b;
        d->last_adc_c = stage_e23_adc_c;
        d->last_floating_adc = stage_e23_floating_adc;
        d->last_positive_adc = stage_e23_positive_adc;
        d->last_negative_adc = stage_e23_negative_adc;
        d->dma_irq_count_delta = stage_e23_dma_fdt_count - live_sector_dma_fdt_start;
        d->ch4_event_count_delta = stage_e23_tmr1_ch4_event_count - live_sector_ch4_event_start;
        d->zc_state_at_timeout = (int)zc_arm_state;
        stage_e23_first_timeout_diag_valid = 1;

        capture_tmr1_regs(&stage_e23_tmr1_regs_at_timeout);
        stage_e23_tmr1_regs_at_timeout_valid = 1;
      }

      consecutive_fault_count++;
      check_fault_stop();

      /* Stage E23: per instruction, the ramp tolerates NO fault --
       * stop immediately on the first timeout, rather than waiting
       * for the 3-strike threshold above. */
      if (stage_e23_running) {
        stop_and_force_off(STOP_REASON_FAULT_DURING_RAMP);
      }

      /* Stage E23: dedicated 300ms-since-entering-20% cap, checked here
       * too so a run stuck waiting through a long sector at 20% still
       * stops on time. */
      if (stage_e23_running && stage_e23_current_duty_level_index == 1u) {
        update_elapsed_since_duty20();
        if (elapsed_since_duty20_us >= DUTY20_TIME_LIMIT_US) {
          stop_and_force_off(STOP_REASON_DUTY20_TIME_LIMIT);
        }
      }

      /* Stage E23: 500ms-since-handover cap (outer backstop), checked
       * here too so a run stuck waiting through a long sector still
       * stops on time. */
      if (stage_e23_running) {
        update_elapsed_since_handover();
        if (elapsed_since_handover_us >= TEST_MAX_TIME_SINCE_HANDOVER_US) {
          stop_and_force_off(STOP_REASON_MAX_TIME);
        }
      }

      if (stage_e23_running) arm_timeout_watchdog();
      return;
    }

    /* TMR3_PURPOSE_COMMUTATION: this overflow IS the ZC-scheduled
     * commutation. */
    uint32_t prev_step_start = stage_e23_step_start_us;
    uint32_t prev_step_index = stage_e23_step_index; /* the sector that is ENDING -- apply_step() below overwrites stage_e23_step_index */
    int next = (int)((stage_e23_step_index + 1u) % 6u);

    apply_step(next, g_duty_ccr);
    stage_e23_step_count++;
    stage_e23_closed_loop_step_count++;

    /* Stage E23 instrumentation: capture the FIRST closed-loop
     * commutation's state, immediately after apply_step() -- proof of
     * what apply_step() actually reset for the new sector. */
    if (stage_e23_closed_loop_step_count == 1u && !stage_e23_first_closed_loop_snapshot_valid) {
      stage_e23_commutation_snapshot_t *snap = &stage_e23_first_closed_loop_snapshot;
      snap->step_index = stage_e23_step_index;
      snap->expected_dir = expected_dir[stage_e23_step_index];
      snap->pos_phase = stage_e23_positive_phase;
      snap->neg_phase = stage_e23_negative_phase;
      snap->floating_phase = stage_e23_floating_phase;
      snap->sector_decided = zc_locked;
      snap->zc_state = (int)zc_arm_state;
      snap->zc_confirmed_sign = zc_active.confirmed_sign;
      snap->zc_confirm_run = zc_active.confirm_run;
      snap->blank_scans_remaining = blank_scans_remaining;
      snap->dma_fdt_count = stage_e23_dma_fdt_count;
      stage_e23_first_closed_loop_snapshot_valid = 1;

      capture_reg_snapshot(&stage_e23_regs_after_commutation);
      stage_e23_regs_after_commutation_valid = 1;

      capture_tmr1_regs(&stage_e23_tmr1_regs_after_commutation);
      stage_e23_tmr1_regs_after_commutation_valid = 1;

      stage_e23_cval_progression.cval_0 = TMR1->cval;
      stage_e23_cval_progression.cval_0_timestamp_us = TMR2->cval;
    }

    uint32_t now_us = stage_e23_step_start_us; /* just set by apply_step() */
    uint32_t period_us = (uint32_t)(uint16_t)(now_us - prev_step_start);
    if (period_us < stage_e23_sector_period_us_min) stage_e23_sector_period_us_min = period_us;
    if (period_us > stage_e23_sector_period_us_max) stage_e23_sector_period_us_max = period_us;
    stage_e23_sector_period_us_sum += period_us;
    last_valid_sector_period_us = period_us;

    /* Stage E23: per-step + overall sector-period stats, filed under
     * the step that just ended. */
    stat_update_by_step(stage_e23_sector_period_by_step, prev_step_index, period_us);
    stat_update(&stage_e23_sector_period_overall, period_us);

    /* Stage E23: complete the sector-log entry for the sector that
     * just ended, using the pending_* values the DMA IRQ stashed at
     * the confirming ZC (before this commutation was even armed). */
    if (stage_e23_sector_log_count < SECTOR_LOG_SIZE) {
      stage_e23_sector_entry_t *se = &stage_e23_sector_log[stage_e23_sector_log_count];
      se->step_index = prev_step_index;
      se->commutation_timestamp_us = prev_step_start;
      se->zc_timestamp_us = pending_zc_timestamp_us;
      se->sector_period_us = period_us;
      se->scheduled_delay_us = stage_e23_last_arm_log.requested_delay_us;
      se->expected_dir = expected_dir[prev_step_index];
      se->zc_confirmed = 1;
      se->timeout_occurred = (pending_timeout_count_this_sector > 0) ? 1 : 0;
      se->consecutive_fault_count_at_confirm = pending_consecutive_fault_count;
      se->floating_adc_at_zc = pending_floating_adc_at_zc;
      se->positive_adc_at_zc = pending_positive_adc_at_zc;
      se->negative_adc_at_zc = pending_negative_adc_at_zc;
      se->diff_at_zc = pending_diff_at_zc;
      se->positive_adc_min = pending_positive_adc_min; se->positive_adc_max = pending_positive_adc_max; se->positive_adc_sum = pending_positive_adc_sum;
      se->negative_adc_min = pending_negative_adc_min; se->negative_adc_max = pending_negative_adc_max; se->negative_adc_sum = pending_negative_adc_sum;
      se->floating_adc_min = pending_floating_adc_min; se->floating_adc_max = pending_floating_adc_max;
      se->diff_min = pending_agg_diff_min; se->diff_max = pending_agg_diff_max;
      se->sample_count = pending_agg_sample_count;
      stage_e23_sector_log_count++;
    }

    /* Stage E23: per-duty-level log for the sector that just ended
     * (filed under the level that WAS active during it, i.e. before
     * any ramp-advance below). */
    if (stage_e23_current_duty_level_index < NUM_DUTY_LEVELS) {
      stage_e23_duty_level_log_t *dl = &stage_e23_duty_log[stage_e23_current_duty_level_index];
      stat_update(&dl->sector_period, period_us);
      dl->commutation_count++;
    }

    /* Stage E23: two-level hold -- 12 commutations at 15% then 24 at
     * 20%, per-level hold counts (duty_hold_commutations[]), not a
     * single constant. Every commutation reaching this point is by
     * construction "good" (a timeout never calls apply_step(), and a
     * mismatch never reschedules it), so no separate success/failure
     * bookkeeping is needed here. */
    commutations_since_duty_change++;
    if (commutations_since_duty_change >= duty_hold_commutations[stage_e23_current_duty_level_index]) {
      if (stage_e23_current_duty_level_index + 1u < NUM_DUTY_LEVELS) {
        stage_e23_current_duty_level_index++;
        stage_e23_current_duty_percent = duty_levels_percent[stage_e23_current_duty_level_index];
        g_duty_ccr = (uint16_t)((PWM_ARR + 1u) * stage_e23_current_duty_percent / 100u);
        commutations_since_duty_change = 0;

        /* Stage E23: start the dedicated 20%-specific 300ms clock the
         * instant 20% is entered. */
        last_time_check_duty20_us = TMR2->cval;
        elapsed_since_duty20_us = 0;
      } else {
        /* 24 commutations completed AT 20% (top level in this file) --
         * test done, per instruction duty never goes past 20% here. */
        stop_and_force_off(STOP_REASON_RAMP_COMPLETE);
        return;
      }
    }

    /* Stage E23: dedicated 300ms-since-entering-20% cap. */
    if (stage_e23_running && stage_e23_current_duty_level_index == 1u) {
      update_elapsed_since_duty20();
      if (elapsed_since_duty20_us >= DUTY20_TIME_LIMIT_US) {
        stop_and_force_off(STOP_REASON_DUTY20_TIME_LIMIT);
        return;
      }
    }

    /* Stage E23: bounded test window -- 60 closed-loop commutations
     * or 500ms since handover, whichever first (outer safety net),
     * stopping the same safe way a real fault would. */
    update_elapsed_since_handover();
    if (stage_e23_closed_loop_step_count >= TEST_MAX_CLOSED_LOOP_COMMUTATIONS) {
      stop_and_force_off(STOP_REASON_MAX_COMMUTATIONS);
      return;
    }
    if (elapsed_since_handover_us >= TEST_MAX_TIME_SINCE_HANDOVER_US) {
      stop_and_force_off(STOP_REASON_MAX_TIME);
      return;
    }

    /* Immediately go back to waiting for THIS new sector's ZC -- TMR3
     * must not carry over its commutation purpose. */
    arm_timeout_watchdog();
    return;
  }
}

static void step_timer_init(uint32_t first_period_ticks)
{
  crm_periph_clock_enable(CRM_TMR3_PERIPH_CLOCK, TRUE);
  nvic_irq_enable(TMR3_GLOBAL_IRQn, 1, 0);

  tmr_cnt_dir_set(TMR3, TMR_COUNT_UP);
  tmr_base_init(TMR3, first_period_ticks - 1u, 960u - 1u); /* 10us tick, unchanged from Stage E15 */
  tmr_interrupt_enable(TMR3, TMR_OVF_INT, TRUE);
}

static void adc_gpio_config(void)
{
  gpio_init_type g;

  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  gpio_default_para_init(&g);
  g.gpio_mode = GPIO_MODE_ANALOG;
  g.gpio_pins = GPIO_PINS_0 | GPIO_PINS_4 | GPIO_PINS_5;
  gpio_init(GPIOA, &g);
}

static void dma_config(void)
{
  dma_init_type d;

  crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
  nvic_irq_enable(DMA1_Channel1_IRQn, 0, 0);
  dma_reset(DMA1_CHANNEL1);

  dma_flexible_config(DMA1, FLEX_CHANNEL1, DMA_FLEXIBLE_ADC1);

  dma_default_para_init(&d);
  d.buffer_size = 3;
  d.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
  d.memory_base_addr = (uint32_t)adc_buf;
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
}

static void adc_config(void)
{
  adc_base_config_type b;

  crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK, TRUE);
  adc_reset(ADC1);
  crm_adc_clock_div_set(CRM_ADC_DIV_4);

  adc_base_default_para_init(&b);
  b.sequence_mode = TRUE;
  b.repeat_mode = FALSE;
  b.data_align = ADC_RIGHT_ALIGNMENT;
  b.ordinary_channel_length = 3;
  adc_base_config(ADC1, &b);

  adc_ordinary_channel_set(ADC1, PHASE_A_CHANNEL, 1, ADC_SAMPLETIME_13_5);
  adc_ordinary_channel_set(ADC1, PHASE_B_CHANNEL, 2, ADC_SAMPLETIME_13_5);
  adc_ordinary_channel_set(ADC1, PHASE_C_CHANNEL, 3, ADC_SAMPLETIME_13_5);

  adc_ordinary_conversion_trigger_set(ADC1, ADC12_ORDINARY_TRIG_TMR1CH4, TRUE);
  adc_dma_mode_enable(ADC1, TRUE);

  nvic_irq_enable(ADC1_IRQn, 2, 0);
  adc_interrupt_enable(ADC1, ADC_CCE_INT, TRUE);

  adc_enable(ADC1, TRUE);

  adc_calibration_init(ADC1);
  while (adc_calibration_init_status_get(ADC1));
  adc_calibration_start(ADC1);
  while (adc_calibration_status_get(ADC1));
}

void ADC1_IRQHandler(void)
{
  if (adc_interrupt_flag_get(ADC1, ADC_CCE_FLAG) != RESET) {
    adc_flag_clear(ADC1, ADC_CCE_FLAG);
    stage_e23_adc_conversion_count++;
  }
}

void DMA1_Channel1_IRQHandler(void)
{
  if (dma_interrupt_flag_get(DMA1_FDT1_FLAG) != RESET) {
    dma_flag_clear(DMA1_FDT1_FLAG);
    stage_e23_dma_fdt_count++;

    /* Stage E23 instrumentation: the first FDT IRQ after entering
     * closed loop -- does the hardware auto-reload/re-arm look healthy
     * right after this one successful transfer? */
    if (stage_e23_mode == MODE_CLOSED_LOOP &&
        stage_e23_regs_after_commutation_valid &&
        !stage_e23_regs_after_first_dma_irq_valid) {
      capture_reg_snapshot(&stage_e23_regs_after_first_dma_irq);
      stage_e23_regs_after_first_dma_irq_valid = 1;

      /* Stage E23: cval_1, the second point of the non-blocking
       * progression check -- same IRQ, no busy-wait added. */
      stage_e23_cval_progression.cval_1 = TMR1->cval;
      stage_e23_cval_progression.cval_1_timestamp_us = TMR2->cval;
      stage_e23_cval_progression.valid = 1;
    }

    uint16_t a = adc_buf[0], b = adc_buf[1], c = adc_buf[2];
    uint16_t vals[3];
    vals[0] = a; vals[1] = b; vals[2] = c;

    stage_e23_adc_a = a;
    stage_e23_adc_b = b;
    stage_e23_adc_c = c;

    /* Stage E23: runs on EVERY closed-loop sample, independent of
     * blanking/zc_locked -- see file header. */
    if (stage_e23_running && !stage_e23_aligning && stage_e23_mode == MODE_CLOSED_LOOP) {
      alllow_update(a, b, c);
    }

    if (stage_e23_running && !stage_e23_aligning) {
      uint16_t v = vals[stage_e23_floating_phase];
      uint16_t pos_v = vals[stage_e23_positive_phase];
      uint16_t neg_v = vals[stage_e23_negative_phase];

      stage_e23_floating_adc = v;
      stage_e23_positive_adc = pos_v;
      stage_e23_negative_adc = neg_v;

      int32_t diff = (int32_t)v - (int32_t)pos_v / 2;
      stage_e23_floating_diff = diff;

      if (blank_scans_remaining) {
        blank_scans_remaining--;
      } else if (!zc_locked) {
        int confirmed_dir = 0;

        /* Stage E23 instrumentation: live per-sector accumulators. */
        live_adc_sample_count++;
        if (diff < live_diff_min) live_diff_min = diff;
        if (diff > live_diff_max) live_diff_max = diff;

        if (pos_v < live_pos_adc_min) live_pos_adc_min = pos_v;
        if (pos_v > live_pos_adc_max) live_pos_adc_max = pos_v;
        live_pos_adc_sum += pos_v;
        if (neg_v < live_neg_adc_min) live_neg_adc_min = neg_v;
        if (neg_v > live_neg_adc_max) live_neg_adc_max = neg_v;
        live_neg_adc_sum += neg_v;
        if (v < live_flo_adc_min) live_flo_adc_min = v;
        if (v > live_flo_adc_max) live_flo_adc_max = v;

        if (trace_active_group >= 0 &&
            stage_e23_step_trace_fill_count[trace_active_group] < STEP_TRACE_LEN) {
          uint32_t idx = stage_e23_step_trace_fill_count[trace_active_group];
          stage_e23_step_trace[trace_active_group][idx].positive_adc = pos_v;
          stage_e23_step_trace[trace_active_group][idx].negative_adc = neg_v;
          stage_e23_step_trace[trace_active_group][idx].floating_adc = v;
          stage_e23_step_trace_fill_count[trace_active_group] = idx + 1u;
          if (idx + 1u >= STEP_TRACE_LEN) {
            stage_e23_step_trace_captured[trace_active_group] = 1;
            trace_active_group = -1;
          }
        }

        if (zc_arm_state == ZC_UNARMED) {
          if (diff <= -ZC_DEADBAND) {
            zc_arm_state = ZC_ARMED_RISING;
            zc_active.confirmed_sign = -1;
            zc_active.confirm_run = 0;
            live_armed_count++;
          } else if (diff >= ZC_DEADBAND) {
            zc_arm_state = ZC_ARMED_FALLING;
            zc_active.confirmed_sign = 1;
            zc_active.confirm_run = 0;
            live_armed_count++;
          }
        } else {
          int r = zc_filter_update(&zc_active, diff);
          if (zc_arm_state == ZC_ARMED_RISING && r == 1) confirmed_dir = 1;
          else if (zc_arm_state == ZC_ARMED_FALLING && r == 2) confirmed_dir = 2;
        }

        if (confirmed_dir == 1) live_rising_confirm_count++;
        else if (confirmed_dir == 2) live_falling_confirm_count++;

        if (confirmed_dir) {
          zc_locked = 1;

          uint32_t now_us = TMR2->cval;
          uint32_t delay_us = (uint32_t)(uint16_t)(now_us - stage_e23_step_start_us);
          int matched = (confirmed_dir == expected_dir[stage_e23_step_index]);

          if (stage_e23_mode == MODE_RAMP || stage_e23_mode == MODE_HOLD) {
            if (matched) {
              stage_e23_consecutive_correct++;
            } else {
              stage_e23_consecutive_correct = 0;
            }

            if (stage_e23_mode == MODE_HOLD &&
                stage_e23_consecutive_correct >= HANDOVER_CONSECUTIVE_REQUIRED) {
              /* HANDOVER -- schedule_next_commutation() below arms
               * TMR3_PURPOSE_COMMUTATION for the first closed-loop
               * step; TMR3_GLOBAL_IRQHandler re-arms the timeout
               * watchdog immediately after that step applies. */
              stage_e23_mode = MODE_CLOSED_LOOP;
              stage_e23_handover_success = 1;
              stage_e23_handover_step_index = stage_e23_step_index;

              /* Stage E23: start the bounded-test-window clock. */
              stage_e23_handover_timestamp_us = now_us;
              last_time_check_us = now_us;
              elapsed_since_handover_us = 0;

              /* Stage E23: duty ramp starts at 15% (handover duty,
               * unchanged) -- g_duty_ccr is already set to that value
               * from main(), not touched here. */
              stage_e23_current_duty_level_index = 0;
              stage_e23_current_duty_percent = duty_levels_percent[0];
              commutations_since_duty_change = 0;

              /* Stage E23: stash this (handover) sector's log values
               * for the TMR3 IRQ's COMMUTATION branch to pick up. */
              pending_zc_timestamp_us = now_us;
              pending_floating_adc_at_zc = v;
              pending_positive_adc_at_zc = pos_v;
              pending_timeout_count_this_sector = live_timeout_count_this_sector;
              pending_consecutive_fault_count = consecutive_fault_count;
              pending_negative_adc_at_zc = neg_v;
              pending_diff_at_zc = diff;
              pending_positive_adc_min = live_pos_adc_min; pending_positive_adc_max = live_pos_adc_max; pending_positive_adc_sum = live_pos_adc_sum;
              pending_negative_adc_min = live_neg_adc_min; pending_negative_adc_max = live_neg_adc_max; pending_negative_adc_sum = live_neg_adc_sum;
              pending_floating_adc_min = live_flo_adc_min; pending_floating_adc_max = live_flo_adc_max;
              pending_agg_diff_min = live_diff_min; pending_agg_diff_max = live_diff_max;
              pending_agg_sample_count = live_adc_sample_count;

              consecutive_fault_count = 0;

              stage_e23_sector_period_us_min = 0xffffffffu;
              stage_e23_sector_period_us_max = 0;
              stage_e23_sector_period_us_sum = 0;
              stage_e23_scheduled_delay_us_min = 0xffffffffu;
              stage_e23_scheduled_delay_us_max = 0;
              stage_e23_scheduled_delay_us_sum = 0;

              schedule_next_commutation(delay_us);
              stage_e23_scheduled_delay_us_min = delay_us;
              stage_e23_scheduled_delay_us_max = delay_us;
              stage_e23_scheduled_delay_us_sum = delay_us;

              /* Stage E23: the handover ZC's delay counts too. */
              stat_update_by_step(stage_e23_zc_delay_by_step, stage_e23_step_index, delay_us);
              stat_update(&stage_e23_zc_delay_overall, delay_us);
            }
          } else if (stage_e23_mode == MODE_CLOSED_LOOP) {
            stage_e23_zc_count++;
            if (stage_e23_current_duty_level_index < NUM_DUTY_LEVELS) {
              stage_e23_duty_log[stage_e23_current_duty_level_index].zc_count++;
            }

            if (matched) {
              /* Stage E23: stash this sector's log values for the
               * TMR3 IRQ's COMMUTATION branch to pick up, BEFORE the
               * fault streak is reset. */
              pending_zc_timestamp_us = now_us;
              pending_floating_adc_at_zc = v;
              pending_positive_adc_at_zc = pos_v;
              pending_timeout_count_this_sector = live_timeout_count_this_sector;
              pending_consecutive_fault_count = consecutive_fault_count;
              pending_negative_adc_at_zc = neg_v;
              pending_diff_at_zc = diff;
              pending_positive_adc_min = live_pos_adc_min; pending_positive_adc_max = live_pos_adc_max; pending_positive_adc_sum = live_pos_adc_sum;
              pending_negative_adc_min = live_neg_adc_min; pending_negative_adc_max = live_neg_adc_max; pending_negative_adc_sum = live_neg_adc_sum;
              pending_floating_adc_min = live_flo_adc_min; pending_floating_adc_max = live_flo_adc_max;
              pending_agg_diff_min = live_diff_min; pending_agg_diff_max = live_diff_max;
              pending_agg_sample_count = live_adc_sample_count;

              consecutive_fault_count = 0;

              if (delay_us < stage_e23_scheduled_delay_us_min) stage_e23_scheduled_delay_us_min = delay_us;
              if (delay_us > stage_e23_scheduled_delay_us_max) stage_e23_scheduled_delay_us_max = delay_us;
              stage_e23_scheduled_delay_us_sum += delay_us;

              /* Stage E23: per-step + overall ZC-to-commutation (30-
               * degree) delay stats, filed under the step whose ZC
               * produced this delay. */
              stat_update_by_step(stage_e23_zc_delay_by_step, stage_e23_step_index, delay_us);
              stat_update(&stage_e23_zc_delay_overall, delay_us);

              /* Supersede the timeout watchdog: this real ZC arms the
               * actual scheduled commutation instead. */
              schedule_next_commutation(delay_us);
            } else {
              stage_e23_polarity_error_count++;
              if (stage_e23_current_duty_level_index < NUM_DUTY_LEVELS) {
                stage_e23_duty_log[stage_e23_current_duty_level_index].polarity_error_count++;
              }
              consecutive_fault_count++;
              check_fault_stop();
              /* leave the running timeout watchdog untouched -- we do
               * not act on a suspect (wrong-direction) reading */

              /* Stage E23: per instruction, the ramp tolerates NO
               * fault -- stop immediately on the first polarity error,
               * rather than waiting for the 3-strike threshold above. */
              if (stage_e23_running) {
                stop_and_force_off(STOP_REASON_FAULT_DURING_RAMP);
              }
            }
          }
        }
      }
    }
  }
  if (dma_interrupt_flag_get(DMA1_DTERR1_FLAG) != RESET) {
    dma_flag_clear(DMA1_DTERR1_FLAG);
    stage_e23_dma_error++;
  }
}

/*
 * TMR2 wraps at 16 bits (see zc_delay_us's own mod-2^16 handling in
 * DMA1_Channel1_IRQHandler). A single mod-2^16 comparison is only
 * valid for waits under 65536us -- ALIGN_DURATION_MS*1000 = 300000us
 * exceeds that, so a bare (uint16_t)(now-start) < us busy-wait can
 * never terminate (the wrapped difference maxes out at 65535, which
 * is always < 300000). Fixed by splitting any wait into <=60000us
 * chunks, each safely inside one 16-bit window.
 */
static void delay_us(uint32_t us)
{
  while (us) {
    uint16_t chunk = (us > 60000u) ? 60000u : (uint16_t)us;
    uint16_t start = (uint16_t)TMR2->cval;
    while ((uint16_t)((uint16_t)TMR2->cval - start) < chunk);
    us -= chunk;
  }
}

static void timestamp_timer_config(void)
{
  crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);
  tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);
  tmr_base_init(TMR2, 0xFFFFFFFFu, (uint32_t)(system_core_clock / 1000000u) - 1u);
  tmr_counter_enable(TMR2, TRUE);
}

int main(void)
{
  /* Stage E23 logger fix: explicit zero-init, not left to an assumption
   * that BSS is zeroed at reset. Diagnostic-only, no control-path
   * effect. */
  for (int i = 0; i < 6; i++) {
    stage_e23_sector_period_by_step[i].min = 0;
    stage_e23_sector_period_by_step[i].max = 0;
    stage_e23_sector_period_by_step[i].sum = 0;
    stage_e23_sector_period_by_step[i].count = 0;
    stage_e23_zc_delay_by_step[i].min = 0;
    stage_e23_zc_delay_by_step[i].max = 0;
    stage_e23_zc_delay_by_step[i].sum = 0;
    stage_e23_zc_delay_by_step[i].count = 0;
  }
  stage_e23_sector_period_overall.min = 0;
  stage_e23_sector_period_overall.max = 0;
  stage_e23_sector_period_overall.sum = 0;
  stage_e23_sector_period_overall.count = 0;
  stage_e23_zc_delay_overall.min = 0;
  stage_e23_zc_delay_overall.max = 0;
  stage_e23_zc_delay_overall.sum = 0;
  stage_e23_zc_delay_overall.count = 0;

  for (int i = 0; i < (int)NUM_DUTY_LEVELS; i++) {
    stage_e23_duty_log[i].sector_period.min = 0;
    stage_e23_duty_log[i].sector_period.max = 0;
    stage_e23_duty_log[i].sector_period.sum = 0;
    stage_e23_duty_log[i].sector_period.count = 0;
    stage_e23_duty_log[i].zc_count = 0;
    stage_e23_duty_log[i].timeout_count = 0;
    stage_e23_duty_log[i].polarity_error_count = 0;
    stage_e23_duty_log[i].alllow_ms_event_count = 0;
    stage_e23_duty_log[i].commutation_count = 0;
  }

  clock_config_96mhz();

  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

  gate_pins_force_off();

  timestamp_timer_config();
  adc_gpio_config();
  dma_config();

  tim1_init();
  g_duty_ccr = (uint16_t)((PWM_ARR + 1u) * OPEN_LOOP_DUTY_PERCENT / 100u);
  tim1_adc_trigger_config();

  adc_config();
  dma_channel_enable(DMA1_CHANNEL1, TRUE);

  tim1_pins_to_af();
  stage_e23_aligning = 1;
  stage_e23_running = 1;
  stage_e23_mode = MODE_RAMP;
  stage_e23_step_index = 0;
  apply_step(0, g_duty_ccr);
  tmr_output_enable(TMR1, TRUE);
  tmr_counter_enable(TMR1, TRUE);
  delay_us(ALIGN_DURATION_MS * 1000u);
  stage_e23_aligning = 0;

  ramp_bucket_index = 0;
  start_ramp_bucket(0);
  stage_e23_step_count = 1;
  step_timer_init(ramp_period_ticks[0]);
  tmr_counter_enable(TMR3, TRUE);

  for (;;) {
    ++stage_e23_heartbeat;
  }
}
