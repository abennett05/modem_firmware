/// puck_settings.h
/// - - - - - - - - - - - -
/// Persistent, versioned device settings for the Modem Puck.
///
/// A single small fixed-size record (owner key + device name + sound + light
/// brightness) stored in the nRF52840's internal flash via Adafruit_LittleFS
/// (InternalFS) so it survives power cycles. This is NOT a general filesystem
/// use — it is one record file, mirroring a reserved-flash "small record"
/// approach but leaning on the core's bundled LittleFS for the page/erase/wear
/// handling instead of hand-rolling raw flash writes.
///
/// The record carries a magic sentinel + schema version so blank flash (a fresh
/// board) or an older/newer layout is detected on boot and re-initialized to
/// DEFAULTS rather than read as garbage.

#pragma once
#include <stdint.h>
#include "modem_types.h"

// Sentinel + schema version written into every record. Bump the version when
// the struct layout changes so old records are discarded as incompatible.
constexpr uint32_t PUCK_SETTINGS_MAGIC   = 0x4D444D31;  // 'MDM1'
constexpr uint16_t PUCK_SETTINGS_VERSION = 1;
constexpr uint8_t  PUCK_OWNER_KEY_LEN    = 16;          // phone-generated claim key

// Defaults applied on first boot and on every ownership change.
#define PUCK_DEFAULT_NAME "Modem Puck"

/// The persisted record. Packed + fixed-size: the on-flash size must stay stable
/// so a read can validate the stored length against sizeof(PuckSettings).
struct __attribute__((packed)) PuckSettings {
  uint32_t magic;                        // sentinel to detect uninitialized/old flash
  uint16_t version;                      // schema version, start at 1
  uint8_t  claimed;                      // 0 = unclaimed, 1 = claimed
  uint8_t  ownerKey[PUCK_OWNER_KEY_LEN]; // phone-generated claim key
  char     name[MAX_DEVICE_NAME_LEN];    // null-terminated, default PUCK_DEFAULT_NAME
  uint8_t  soundEnabled;                 // 0/1, default 1
  uint8_t  brightness;                   // 0..LIGHT_BRIGHTNESS_MAX, default = LIGHT_BRIGHTNESS_MAX
};

// Mount InternalFS and load the record into RAM. If the stored record is missing
// or its magic/version/size doesn't match, initialize to DEFAULTS (unclaimed,
// name "Modem Puck", sound ON, brightness MAX) and persist it. Call once in setup().
void puckSettingsBegin();

// The live, in-RAM settings. Mutate these, then call puckSettingsSave().
PuckSettings& puckSettings();

// Persist the in-RAM settings to flash ONLY if they differ from what is already
// stored (debounced against identical values to limit flash wear). Returns true
// iff a write actually happened.
bool puckSettingsSave();

// Reset the content fields (name / sound / brightness) to DEFAULTS while KEEPING
// the current ownerKey and claimed flag. Used on any ownership change. Does not
// persist on its own — the caller saves once after composing the new state.
void puckSettingsResetToDefaults();
