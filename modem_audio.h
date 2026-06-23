/// modem_audio.h
/// - - - - - - - - - - - -
/// Speaker chime playback for the Modem Puck.
///
/// Drives a MAX98357A I2S mono amplifier from the nRF52840's I2S peripheral
/// using double-buffered EasyDMA. PCM source data lives in modem_chimes.h
/// (16-bit signed mono @ MODEM_PCM_SAMPLE_RATE). Playback is non-blocking: a
/// chime is kicked off and serviced entirely from the I2S interrupt; the caller
/// returns immediately.
///
/// IMPORTANT: this is implemented directly on the nrfx I2S *HAL* (nrf_i2s.h),
/// NOT the nrfx_i2s *driver* — the Adafruit nRF52 core ships the HAL and the
/// driver header but does NOT compile nrfx_i2s.c, so nrfx_i2s_init() would not
/// link. We therefore own the EasyDMA buffer flip and the I2S IRQ handler here.
///
/// Every chime is gated on the persisted `soundEnabled` setting, read LIVE at
/// trigger time (puckSettings().soundEnabled) so a runtime BLE toggle takes
/// effect on the very next chime with no reboot and no stale boot-time copy.

#pragma once
#include <stdint.h>

// Initialize the I2S peripheral + DMA engine for the MAX98357A. Call once from
// setup() (after puckSettingsBegin so the sound gate can be read). Safe to call
// even if audio init fails — playChime() then becomes a no-op.
void modemAudioBegin();

// Start (or restart) playback of a 16-bit signed mono PCM chime. Non-blocking.
// Early-returns and plays NOTHING when sound is disabled (logged as suppressed)
// or when audio init failed. Starting a chime while one is playing stops the
// in-flight chime and starts this one (latest wins).
void playChime(const int16_t* data, uint16_t len);

// Convenience wrappers for the session/ownership events. Each honors the sound gate.
void chimeJoin();    // a (non-first) phone joined the session
void chimeLeave();   // a phone left the session
void chimeNotify();  // an allowlisted notification arrived for a member
void chimeStart();   // the first phone joined -> session started (host)
void chimeClaim();   // the puck was claimed / taken over (ownership change)
