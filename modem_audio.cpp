/// modem_audio.cpp
/// - - - - - - - - - - - -
/// I2S double-buffered EasyDMA chime playback for the MAX98357A amp.
/// See modem_audio.h for the design rationale (HAL, not driver) and the
/// sound-enabled gate contract.

#include <Arduino.h>
#include <nrfx.h>
#include "nrf_i2s.h"   // nrfx I2S HAL (on the core include path: nordic/nrfx/hal)

#include "modem_audio.h"
#include "modem_chimes.h"
#include "puck_settings.h"

/// I2S pin assignments  ---------------------------------------------------------
/// Raw nRF52840 GPIO numbers (P0.xx = xx, P1.xx = 32 + xx). The MAX98357A needs
/// three signals; it derives its own clock, so MCLK and SDIN are NOT connected.
///   BCLK  -> amp BCLK   (I2S SCK)
///   LRCK  -> amp LRC    (I2S word-select / WS)
///   DIN   -> amp DIN    (I2S SDOUT, puck -> amp)
///
/// Pins per the board wiring (raw nRF52840 GPIO numbers, P0.xx). Confirmed by
/// the hardware owner 2026-06-23. Distinct from LIGHTS_PIN (P0.31).
static constexpr uint32_t MODEM_I2S_PIN_BCLK = 20;   // P0.20 -> amp BCLK
static constexpr uint32_t MODEM_I2S_PIN_LRCK = 22;   // P0.22 -> amp LRC (word-select)
static constexpr uint32_t MODEM_I2S_PIN_DIN  = 17;   // P0.17 -> amp DIN (SDOUT)

/// Clock / format  -------------------------------------------------------------
/// 16-bit stereo frames (32 bits/frame). The mono PCM sample is duplicated into
/// both channels so a true-mono amp (MAX98357A, LR tied) gets full level.
///
/// Sample rate: LRCK = MCK / RATIO. The nRF MCK is 32 MHz / N, and 16000 Hz is
/// not exactly reachable. MCK = 32M/21 = 1.523810 MHz with RATIO 96X gives
/// LRCK = 15873 Hz — ~0.8 % flat vs the chimes' 16000 Hz authoring rate, an
/// inaudible pitch error. (No integer N/RATIO pair lands on exactly 16000.)
static constexpr nrf_i2s_mck_t   I2S_MCK   = NRF_I2S_MCK_32MDIV21;  // 1.523810 MHz
static constexpr nrf_i2s_ratio_t I2S_RATIO = NRF_I2S_RATIO_96X;     // -> 15873 Hz LRCK

/// IRQ priority  ---------------------------------------------------------------
/// The SoftDevice reserves interrupt priorities 0, 1 and 4. The I2S IRQ MUST sit
/// strictly below it so BLE timing is never starved: we use 7 (the lowest /
/// least-urgent priority), comfortably below the SoftDevice. The nrfx_i2s driver
/// is not used, so there is no NRFX_I2S_CONFIG_IRQ_PRIORITY to set — the priority
/// is applied here directly via NVIC_SetPriority(I2S_IRQn, 7).
static constexpr uint32_t I2S_IRQ_PRIORITY = 7;

/// Double buffers  -------------------------------------------------------------
/// Two EasyDMA TX buffers ping-pong. Each 32-bit word is one stereo frame
/// (left in one half-word, right in the other); since L == R for our duplicated
/// mono sample, the half-word order is irrelevant. 256 frames/buffer = ~16 ms at
/// 15873 Hz, long enough that the loop()/IRQ refill is never tight, small enough
/// to keep RAM (2 KB total) and restart latency low.
static constexpr uint16_t BUF_FRAMES = 256;
static uint32_t s_buf[2][BUF_FRAMES];

// Playback state. Touched by both playChime() (loop context) and the I2S IRQ;
// the IRQ is disabled while playChime() reconfigures, so no further locking is
// needed. g_pos counts frames already QUEUED into a DMA buffer (incl. the silent
// guard tail), not frames physically played.
static volatile const int16_t* s_data    = nullptr;
static volatile uint32_t       s_len      = 0;   // source sample count
static volatile uint32_t       s_pos      = 0;   // next source frame to queue
static volatile uint32_t       s_total    = 0;   // s_len + silent guard frames
static volatile uint8_t        s_nextIdx  = 0;   // buffer to hand the DMA next IRQ
static volatile bool           s_playing  = false;
static bool                    s_ready    = false;  // init succeeded

// A silent tail guarantees the last real samples fully clock out before STOP
// (STOP only cuts silence). Two buffers of margin is ample and costs ~32 ms.
static constexpr uint32_t GUARD_FRAMES = 2 * BUF_FRAMES;

// Pack one mono int16 sample into a 32-bit stereo frame (both channels equal).
// VOLUME HOOK: the single place to apply amplitude scaling in a future pass.
// Leave the sample untouched here for now (no scaling — out of scope).
static inline uint32_t packFrame(int16_t s) {
  uint16_t u = (uint16_t)s;                 // <-- volume scaling would go here
  return (uint32_t)u | ((uint32_t)u << 16);
}

// Fill buffer `idx` with the next BUF_FRAMES frames from the source, zero-padding
// (silence) once the source is exhausted. Advances s_pos, capped at s_total.
static void fillBuffer(uint8_t idx) {
  uint32_t* dst = s_buf[idx];
  for (uint16_t i = 0; i < BUF_FRAMES; i++) {
    int16_t sample = 0;
    if (s_pos < s_len && s_data) sample = s_data[s_pos];
    dst[i] = packFrame(sample);
    if (s_pos < s_total) s_pos++;
  }
}

// Stop the transfer and park the engine idle. Called from the IRQ at end-of-chime
// and from playChime() before a restart.
static void audioStop() {
  nrf_i2s_int_disable(NRF_I2S, NRF_I2S_INT_TXPTRUPD_MASK);
  nrf_i2s_task_trigger(NRF_I2S, NRF_I2S_TASK_STOP);
  // Wait (bounded) for the peripheral to acknowledge the stop so a back-to-back
  // restart doesn't race a still-running transfer. ~1 buffer max; bail out hard.
  for (uint32_t spin = 0; spin < 200000; spin++) {
    if (nrf_i2s_event_check(NRF_I2S, NRF_I2S_EVENT_STOPPED)) break;
  }
  nrf_i2s_event_clear(NRF_I2S, NRF_I2S_EVENT_STOPPED);
  nrf_i2s_disable(NRF_I2S);
  s_playing = false;
}

void modemAudioBegin() {
  // Configure master mode, I2S format, 16-bit, stereo frames at ~16 kHz.
  bool ok = nrf_i2s_configure(NRF_I2S,
                              NRF_I2S_MODE_MASTER,
                              NRF_I2S_FORMAT_I2S,
                              NRF_I2S_ALIGN_LEFT,
                              NRF_I2S_SWIDTH_16BIT,
                              NRF_I2S_CHANNELS_STEREO,
                              I2S_MCK,
                              I2S_RATIO);
  if (!ok) {
    s_ready = false;
    Serial.println("MODEM_AUDIO init FAILED (bad I2S clock/format config)");
    return;
  }

  // MCK and SDIN are unused by the MAX98357A path -> not connected.
  nrf_i2s_pins_set(NRF_I2S,
                   MODEM_I2S_PIN_BCLK,
                   MODEM_I2S_PIN_LRCK,
                   NRF_I2S_PIN_NOT_CONNECTED,   // MCK: amp self-clocks
                   MODEM_I2S_PIN_DIN,           // SDOUT: puck -> amp DIN
                   NRF_I2S_PIN_NOT_CONNECTED);  // SDIN: TX only

  // I2S IRQ strictly below the SoftDevice (priority 7). Enabled now; the
  // per-event TXPTRUPD interrupt source is gated on/off per playback.
  NVIC_SetPriority(I2S_IRQn, I2S_IRQ_PRIORITY);
  NVIC_ClearPendingIRQ(I2S_IRQn);
  NVIC_EnableIRQ(I2S_IRQn);

  s_ready = true;
  Serial.print("MODEM_AUDIO init OK: I2S master 16-bit stereo, LRCK~15873Hz, "
                "BCLK=P0."); Serial.print(MODEM_I2S_PIN_BCLK);
  Serial.print(" LRCK=P0."); Serial.print(MODEM_I2S_PIN_LRCK);
  Serial.print(" DIN=P0.");  Serial.print(MODEM_I2S_PIN_DIN);
  Serial.print(" IRQprio="); Serial.println(I2S_IRQ_PRIORITY);
}

void playChime(const int16_t* data, uint16_t len) {
  if (!s_ready) {
    Serial.println("MODEM_AUDIO playChime ignored: audio not initialized");
    return;
  }
  // LIVE sound gate: read the persisted setting at trigger time (RAM-resident
  // record, no flash read) so a runtime BLE toggle applies to the very next
  // chime. No cached boot-time copy.
  if (!puckSettings().soundEnabled) {
    Serial.print("MODEM_AUDIO chime SUPPRESSED (sound disabled) len=");
    Serial.println(len);
    return;
  }
  if (!data || len == 0) return;

  // Latest wins: stop any in-flight chime before reloading state.
  if (s_playing) audioStop();

  s_data    = data;
  s_len     = len;
  s_pos     = 0;
  s_total   = (uint32_t)len + GUARD_FRAMES;
  s_nextIdx = 1;                 // buf0 is the initial transfer; buf1 handed first IRQ

  // Prime both buffers: buf0 plays first, buf1 is queued at the first TXPTRUPD.
  fillBuffer(0);
  fillBuffer(1);

  nrf_i2s_event_clear(NRF_I2S, NRF_I2S_EVENT_TXPTRUPD);
  nrf_i2s_event_clear(NRF_I2S, NRF_I2S_EVENT_STOPPED);
  // size is in 32-bit words; one frame == one word here. RX disabled (TX only).
  nrf_i2s_transfer_set(NRF_I2S, BUF_FRAMES, nullptr, s_buf[0]);
  nrf_i2s_enable(NRF_I2S);
  nrf_i2s_int_enable(NRF_I2S, NRF_I2S_INT_TXPTRUPD_MASK);
  s_playing = true;
  nrf_i2s_task_trigger(NRF_I2S, NRF_I2S_TASK_START);

  Serial.print("MODEM_AUDIO chime START len="); Serial.print(len);
  Serial.print(" (~"); Serial.print((uint32_t)len * 1000 / MODEM_PCM_SAMPLE_RATE);
  Serial.println("ms)");
}

// I2S interrupt: a TXPTRUPD means the DMA just latched the buffer we handed it
// last time and now wants the next one. Hand the pre-filled buffer, then refill
// the freed one for the following IRQ. When everything (incl. the silent guard)
// has been queued and clocked, stop cleanly.
extern "C" void I2S_IRQHandler(void) {
  if (!nrf_i2s_event_check(NRF_I2S, NRF_I2S_EVENT_TXPTRUPD)) return;
  nrf_i2s_event_clear(NRF_I2S, NRF_I2S_EVENT_TXPTRUPD);

  if (s_pos >= s_total) {        // all real samples + guard silence have played
    audioStop();
    return;
  }

  uint8_t hand = s_nextIdx;                 // filled on the previous IRQ / at start
  nrf_i2s_tx_buffer_set(NRF_I2S, s_buf[hand]);

  uint8_t freeIdx = hand ^ 1;               // just finished playing -> refill it
  fillBuffer(freeIdx);
  s_nextIdx = freeIdx;
}

/// Chime wrappers  -------------------------------------------------------------
/// Each plays a distinct PCM clip; the sound gate lives in playChime so every
/// wrapper inherits it.
void chimeJoin()   { playChime(MODEM_CHIME_JOIN,   MODEM_CHIME_JOIN_LEN);   }
void chimeLeave()  { playChime(MODEM_CHIME_LEAVE,  MODEM_CHIME_LEAVE_LEN);  }
void chimeNotify() { playChime(MODEM_CHIME_NOTIFY, MODEM_CHIME_NOTIFY_LEN); }
void chimeStart()  { playChime(MODEM_CHIME_START,  MODEM_CHIME_START_LEN);  }
void chimeClaim()  { playChime(MODEM_CHIME_CLAIM,  MODEM_CHIME_CLAIM_LEN);  }
