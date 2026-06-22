/// puck_settings.cpp
/// - - - - - - - - - - - -
/// InternalFS (LittleFS)-backed implementation of the persistent PuckSettings
/// record. See puck_settings.h for the contract and the rationale.

#include <Arduino.h>
#include "puck_settings.h"
#include <string.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

using namespace Adafruit_LittleFS_Namespace;

// Single fixed record file. The leading '/' keeps it at the FS root.
static const char* kSettingsPath = "/puck.cfg";

// The live settings (mutated by the firmware) and a shadow copy of what is
// currently on flash (so puckSettingsSave can skip no-op writes / wear).
static PuckSettings g_settings;
static PuckSettings g_persisted;

// Compose the DEFAULTS into `s` (unclaimed, default name, sound ON, max brightness).
static void fillDefaults(PuckSettings& s) {
  memset(&s, 0, sizeof(s));
  s.magic        = PUCK_SETTINGS_MAGIC;
  s.version      = PUCK_SETTINGS_VERSION;
  s.claimed      = 0;
  memset(s.ownerKey, 0, PUCK_OWNER_KEY_LEN);
  strncpy(s.name, PUCK_DEFAULT_NAME, MAX_DEVICE_NAME_LEN);
  s.name[MAX_DEVICE_NAME_LEN - 1] = '\0';
  s.soundEnabled = 1;
  s.brightness   = LIGHT_BRIGHTNESS_MAX;
}

// Overwrite the on-flash record with `s`. LittleFS's FILE_O_WRITE appends, so we
// remove the old file first to keep the record exactly one struct long. Updates
// the persisted shadow on success.
static bool writeRecord(const PuckSettings& s) {
  InternalFS.remove(kSettingsPath);
  File f(InternalFS);
  if (!f.open(kSettingsPath, FILE_O_WRITE)) {
    Serial.println("puck_settings: open for write FAILED");
    return false;
  }
  const size_t wrote = f.write((const uint8_t*)&s, sizeof(s));
  f.close();
  if (wrote != sizeof(s)) {
    Serial.print("puck_settings: short write "); Serial.println(wrote);
    return false;
  }
  g_persisted = s;
  return true;
}

void puckSettingsBegin() {
  InternalFS.begin();

  bool valid = false;
  File f(InternalFS);
  if (f.open(kSettingsPath, FILE_O_READ)) {
    if (f.size() == sizeof(PuckSettings)) {
      const size_t got = f.read((uint8_t*)&g_settings, sizeof(g_settings));
      valid = (got == sizeof(g_settings) &&
               g_settings.magic == PUCK_SETTINGS_MAGIC &&
               g_settings.version == PUCK_SETTINGS_VERSION);
    }
    f.close();
  }

  if (valid) {
    g_persisted = g_settings;
    Serial.print("puck_settings: loaded (claimed="); Serial.print(g_settings.claimed);
    Serial.print(" name=\""); Serial.print(g_settings.name);
    Serial.print("\" sound="); Serial.print(g_settings.soundEnabled);
    Serial.print(" bright="); Serial.print(g_settings.brightness);
    Serial.println(")");
  } else {
    fillDefaults(g_settings);
    writeRecord(g_settings);  // also seeds g_persisted
    Serial.println("puck_settings: blank/invalid record -> wrote DEFAULTS");
  }
}

PuckSettings& puckSettings() { return g_settings; }

bool puckSettingsSave() {
  if (memcmp(&g_settings, &g_persisted, sizeof(PuckSettings)) == 0) {
    return false;  // nothing changed — skip the flash write
  }
  return writeRecord(g_settings);
}

void puckSettingsResetToDefaults() {
  // Preserve ownership across the reset.
  const uint8_t claimed = g_settings.claimed;
  uint8_t key[PUCK_OWNER_KEY_LEN];
  memcpy(key, g_settings.ownerKey, PUCK_OWNER_KEY_LEN);

  fillDefaults(g_settings);

  g_settings.claimed = claimed;
  memcpy(g_settings.ownerKey, key, PUCK_OWNER_KEY_LEN);
}
