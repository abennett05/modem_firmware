/// fs_format.ino
/// - - - - - - - - - - -
/// One-shot LittleFS (InternalFS) recovery/erase utility for the Modem Puck
/// nRF52840 boards. Flash THIS sketch to a board whose internal filesystem has
/// gone corrupt — the failure mode is a LittleFS assertion
///   assertion "head >= 2 && head <= lfs->cfg->block_count" failed
/// (a bad on-flash metadata/CTZ block pointer), which hangs the real firmware in
/// puckSettingsBegin()/bond load. A corrupt FS bricks the board AT BOOT, so it
/// stops enumerating over USB until you force the UF2 bootloader (double-tap reset
/// / RST-pin trick). Enter the bootloader, drag/flash THIS, let it run once, then
/// reflash modem_firmware.
///
/// What it does: reformats the entire InternalFS, wiping EVERYTHING —
///   /puck.cfg                (owner/name/sound/brightness settings)
///   /adafruit/bond_prph/...  (peripheral bonds — phones you've paired)
///   /adafruit/bond_cntr/...
/// then verifies the fresh FS mounts and is read/writable.
///
/// AFTER FORMATTING: the puck has no bonds, but the iPhone still thinks it does.
/// On the phone go Settings -> Bluetooth -> (the puck) -> "Forget This Device"
/// before re-pairing — iOS has no API to drop its half of an asymmetric bond.
///
/// READ THE RESULT (serial @115200, or the LED):
///   "FS FORMAT: OK"     -> recovered. Reflash modem_firmware. If it then boots
///                          clean and survives churn, it was one-off corruption.
///   "FS FORMAT: FAILED" -> the flash would not format/verify: that board's flash
///                          is likely physically worn. Retire it to a test rig.
/// LED (if wired): slow ~1 Hz blink = OK/done; fast ~5 Hz blink = FAILED.
///
/// Build:  arduino-cli compile --fqbn adafruit:nrf52:feather52840 tools/fs_format
/// Upload: arduino-cli upload  --fqbn adafruit:nrf52:feather52840 -p <port> tools/fs_format

#include <Arduino.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

using namespace Adafruit_LittleFS_Namespace;

// Safety window: a flashed formatter runs immediately on boot, so give a few
// seconds to ABORT (send any serial char) in case this was flashed to a good
// board by mistake. On a truly bricked board there's nothing to abort — just wait.
static const uint32_t ABORT_WINDOW_MS = 5000;
static const char*    kVerifyPath     = "/.fmt_selftest";

static bool g_ok = false;     // result, drives the loop() LED signal

// Confirm the freshly-formatted FS actually mounts and round-trips a file, so a
// silent format that left the flash unusable is reported as FAILED, not OK.
static bool verifyReadWrite() {
  if (!InternalFS.begin()) {
    Serial.println("[fs] verify: re-mount after format FAILED");
    return false;
  }
  const uint8_t  pattern[] = {0xA5, 0x5A, 0xC3, 0x3C};
  uint8_t        readback[sizeof(pattern)] = {0};

  if (InternalFS.exists(kVerifyPath)) InternalFS.remove(kVerifyPath);

  {
    File f(kVerifyPath, FILE_O_WRITE, InternalFS);
    if (!f) { Serial.println("[fs] verify: open-for-write FAILED"); return false; }
    const size_t wrote = f.write(pattern, sizeof(pattern));
    f.close();
    if (wrote != sizeof(pattern)) {
      Serial.print("[fs] verify: short write "); Serial.println(wrote);
      return false;
    }
  }
  {
    File f(kVerifyPath, FILE_O_READ, InternalFS);
    if (!f) { Serial.println("[fs] verify: open-for-read FAILED"); return false; }
    const int got = f.read(readback, sizeof(readback));
    f.close();
    if (got != (int)sizeof(readback) || memcmp(pattern, readback, sizeof(pattern)) != 0) {
      Serial.println("[fs] verify: readback MISMATCH");
      return false;
    }
  }
  InternalFS.remove(kVerifyPath);     // leave the FS clean for the real firmware
  Serial.println("[fs] verify: read/write round-trip OK");
  return true;
}

void setup() {
  Serial.begin(115200);
#ifdef LED_BUILTIN
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
#endif

  // Wait briefly for a USB serial host, but don't block forever on a headless
  // board (it must still format if no terminal is attached).
  const uint32_t serialDeadline = millis() + 3000;
  while (!Serial && (int32_t)(millis() - serialDeadline) < 0) delay(10);

  Serial.println();
  Serial.println("=== Modem Puck — InternalFS FORMAT / RECOVERY TOOL ===");
  Serial.println("This ERASES all puck settings AND all BLE bonds on this board.");
  Serial.print  ("Send any character within ");
  Serial.print  (ABORT_WINDOW_MS / 1000);
  Serial.println("s to ABORT.");

  const uint32_t abortDeadline = millis() + ABORT_WINDOW_MS;
  while ((int32_t)(millis() - abortDeadline) < 0) {
    if (Serial.available()) {
      while (Serial.available()) Serial.read();
      Serial.println(">>> ABORTED by user. No changes made. Power-cycle to retry.");
      while (true) {                 // park: do nothing, leave the FS untouched
#ifdef LED_BUILTIN
        digitalWrite(LED_BUILTIN, HIGH); delay(120);
        digitalWrite(LED_BUILTIN, LOW);  delay(120);
#else
        delay(240);
#endif
      }
    }
    delay(20);
  }

  Serial.println("[fs] mounting InternalFS...");
  InternalFS.begin();                // mount (or auto-init) before formatting

  Serial.println("[fs] formatting (wipes /puck.cfg + /adafruit/bond_*)...");
  const bool formatted = InternalFS.format();
  Serial.print("[fs] format() returned: "); Serial.println(formatted ? "true" : "false");

  g_ok = formatted && verifyReadWrite();

  Serial.println();
  if (g_ok) {
    Serial.println("FS FORMAT: OK");
    Serial.println(">>> Reflash modem_firmware now.");
    Serial.println(">>> On the iPhone: Settings > Bluetooth > Forget This Device, then re-pair.");
  } else {
    Serial.println("FS FORMAT: FAILED");
    Serial.println(">>> Flash is likely worn/bad. Retire this board to a test rig.");
  }
  Serial.println("(LED: slow blink = OK, fast blink = FAILED. Safe to power off.)");
}

void loop() {
  // Persisted visual status so the result is readable without a serial terminal.
#ifdef LED_BUILTIN
  const uint16_t period = g_ok ? 500 : 100;   // slow = OK, fast = FAILED
  digitalWrite(LED_BUILTIN, HIGH); delay(period);
  digitalWrite(LED_BUILTIN, LOW);  delay(period);
#else
  delay(1000);
#endif
}
