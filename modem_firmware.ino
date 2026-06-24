/// modem_firmware.ino
/// - - - - - - - - - - -
/// Where the functionality
/// of the embedded software
/// lies. Many enter sane,
/// few leave that way.
/// Beware of spaghetti
/// code journeyman.
///
/// nRF52840 (Adafruit Bluefruit core) firmware for the Modem Puck.
/// Ported from modem_hw/modem_prototype, keeping its behavior but
/// trading the Arduino-isms for an event-driven design:
///   - ArduinoBLE polling -> Bluefruit write callback (no BLE.poll()).
/// Audio drives a speaker (not the prototype's buzzer) via an I2S MAX98357A amp;
/// chime playback lives in modem_audio.* and is gated on the sound setting.

#include <bluefruit.h>
#include <Adafruit_NeoPixel.h>
#include "modem_types.h"
#include "ancs_client.h"
#include "puck_settings.h"
#include "modem_audio.h"

/// Pins
/// - - - - - - - - - - - -
/// Set this to the GPIO actually wired on the nRF52840 carrier.
constexpr uint8_t LIGHTS_PIN = 31;

/// Lights
/// - - - - - - - - - - - -
Adafruit_NeoPixel strip(NUM_LIGHTS, LIGHTS_PIN, NEO_GRB + NEO_KHZ800);

static const char* DEVICE_NAME = "Modem Puck";

/// BLE service + characteristics
/// - - - - - - - - - - - -
/// 128-bit UUIDs in LSB-first byte order (Bluefruit convention).
/// MSB string: fb59xxxx-ec62-4ba6-baf9-c02429a2a3ac
/// Only byte [12] (the "xxxx" selector) changes per characteristic.
static uint8_t const UUID_SVC[16]    = {0xac,0xa3,0xa2,0x29,0x24,0xc0,0xf9,0xba,0xa6,0x4b,0x62,0xec,0x00,0x00,0x59,0xfb};
static uint8_t const UUID_OWNER[16]  = {0xac,0xa3,0xa2,0x29,0x24,0xc0,0xf9,0xba,0xa6,0x4b,0x62,0xec,0x01,0x00,0x59,0xfb};
static uint8_t const UUID_NAME[16]   = {0xac,0xa3,0xa2,0x29,0x24,0xc0,0xf9,0xba,0xa6,0x4b,0x62,0xec,0x02,0x00,0x59,0xfb};
static uint8_t const UUID_SOUND[16]  = {0xac,0xa3,0xa2,0x29,0x24,0xc0,0xf9,0xba,0xa6,0x4b,0x62,0xec,0x03,0x00,0x59,0xfb};
static uint8_t const UUID_UNAME[16]  = {0xac,0xa3,0xa2,0x29,0x24,0xc0,0xf9,0xba,0xa6,0x4b,0x62,0xec,0x04,0x00,0x59,0xfb};
static uint8_t const UUID_COLOR[16]  = {0xac,0xa3,0xa2,0x29,0x24,0xc0,0xf9,0xba,0xa6,0x4b,0x62,0xec,0x05,0x00,0x59,0xfb};
static uint8_t const UUID_COUNT[16]  = {0xac,0xa3,0xa2,0x29,0x24,0xc0,0xf9,0xba,0xa6,0x4b,0x62,0xec,0x06,0x00,0x59,0xfb};
static uint8_t const UUID_STIME[16]  = {0xac,0xa3,0xa2,0x29,0x24,0xc0,0xf9,0xba,0xa6,0x4b,0x62,0xec,0x07,0x00,0x59,0xfb};
static uint8_t const UUID_SESS[16]   = {0xac,0xa3,0xa2,0x29,0x24,0xc0,0xf9,0xba,0xa6,0x4b,0x62,0xec,0x08,0x00,0x59,0xfb};
static uint8_t const UUID_ROSTER[16] = {0xac,0xa3,0xa2,0x29,0x24,0xc0,0xf9,0xba,0xa6,0x4b,0x62,0xec,0x09,0x00,0x59,0xfb};
// allowlist: per-connection NOTIFICATION allowlist (Phase B). Selector 0x0A —
// NOT 0x05: 0x05 is already the color characteristic. The phone writes ITS
// bundle-ID list here; the puck resolves the writing connHandle to its ANCS
// slot and feeds that slot's allowlist. See onAllowlistWrite.
static uint8_t const UUID_ALLOW[16]  = {0xac,0xa3,0xa2,0x29,0x24,0xc0,0xf9,0xba,0xa6,0x4b,0x62,0xec,0x0a,0x00,0x59,0xfb};
// disconnect: the phone writes here (any byte) to ask the puck to sever ITS side
// of the link. Selector 0x0B. iOS keeps a bonded ANCS connection alive when only
// the phone calls cancelPeripheralConnection, so the puck (the peripheral) must
// initiate the GAP disconnect for the link to actually drop. See onDisconnectRequest.
static uint8_t const UUID_DISCONN[16] = {0xac,0xa3,0xa2,0x29,0x24,0xc0,0xf9,0xba,0xa6,0x4b,0x62,0xec,0x0b,0x00,0x59,0xfb};
// brightness: NEW persisted light-brightness control. Selector 0x0C — NOT 0x06:
// 0x06 is already UUID_COUNT (the session userCount characteristic). uint8 in
// 0..LIGHT_BRIGHTNESS_MAX. Read/write, persisted, applied to the strip on write.
static uint8_t const UUID_BRIGHT[16] = {0xac,0xa3,0xa2,0x29,0x24,0xc0,0xf9,0xba,0xa6,0x4b,0x62,0xec,0x0c,0x00,0x59,0xfb};
// pickup: PRODUCTION per-phone unlock-event sink. Selector 0x0D — the next free
// selector after brightness (0x0C). Each phone reports its own "pickup" (an
// unlock, used as a proxy for picking the phone up) here while it is in a session
// and connected, including while backgrounded+locked over the iOS background
// CoreBluetooth path. The puck attributes the event to a user BY COLOR (the
// reporter rides its OWN background connection, not the app's session link, so
// the writing connHandle is NOT the session user's) and logs it for puck-side
// aggregation. WRITE / WRITE_WO_RESP. See onPickupWrite. (Replaces the removed
// Phase-0 throwaway test sink that lived at selector 0x00FE.)
static uint8_t const UUID_PICKUP[16] = {0xac,0xa3,0xa2,0x29,0x24,0xc0,0xf9,0xba,0xa6,0x4b,0x62,0xec,0x0d,0x00,0x59,0xfb};
// spike: group-level pickup-SPIKE trigger. Selector 0x0E — next free after
// pickup (0x0D). READ + NOTIFY: the puck aggregates pickup events across ALL
// connected phones and, when the group's pickup rate spikes (the conversation
// has drifted), notifies here so the next layer (question selection) can react.
// Payload (4 bytes, little-endian): [0..1] spikeSeq uint16 (increments once per
// fired spike — the edge to act on), [2] pickupCount in window, [3] distinct
// slots involved. See detectPickupSpike / notifySpike. Aggregate only; the puck
// never exposes a per-user pickup metric.
static uint8_t const UUID_SPIKE[16]  = {0xac,0xa3,0xa2,0x29,0x24,0xc0,0xf9,0xba,0xa6,0x4b,0x62,0xec,0x0e,0x00,0x59,0xfb};
// charging: USB/charge-state flag. Selector 0x0F — next free after spike (0x0E).
// READ + NOTIFY, uint8 (0 = on battery, 1 = USB present / charging). The standard
// Battery Service (0x180F) only carries a 0..100 percent, and while charging VDDH
// is inflated so the puck SUPPRESSES the percent update — leaving the app to show
// a stale/zero level. This flag lets the app render a charging icon instead. See
// sampleBattery / isUsbPresent.
static uint8_t const UUID_CHARGE[16] = {0xac,0xa3,0xa2,0x29,0x24,0xc0,0xf9,0xba,0xa6,0x4b,0x62,0xec,0x0f,0x00,0x59,0xfb};

BLEService        modemSvc(UUID_SVC);
BLECharacteristic ownerChar(UUID_OWNER);
BLECharacteristic nameChar(UUID_NAME);
BLECharacteristic soundChar(UUID_SOUND);
BLECharacteristic colorChar(UUID_COLOR);
// userName: each phone writes its own chosen display name here right after
// connecting; the firmware stores it on that user so it can appear in the
// roster broadcast (the BLE peer name is unreliable, especially on iOS).
BLECharacteristic userNameChar(UUID_UNAME);
// userCount: pushed to every connected phone whenever a phone joins/leaves so
// the app can render the live "N / MAX_USERS" session roster size.
BLECharacteristic countChar(UUID_COUNT);
// screenTime: each phone writes its own accrued screen time (uint16 seconds,
// little-endian) here every 10s; the firmware stores it on that user.
BLECharacteristic stimeChar(UUID_STIME);
// sessionTime: the puck owns the canonical session clock (seconds since the
// first user joined) and pushes it to every phone once a minute so all devices
// show the same session time. uint16 seconds, little-endian. READ + NOTIFY.
BLECharacteristic sessChar(UUID_SESS);
// roster: the full connected-user table (count + each user's color, screen
// time and name), pushed to every phone whenever it changes so each phone can
// render the complete "Total Screentime" leaderboard. READ + NOTIFY.
BLECharacteristic rosterChar(UUID_ROSTER);
// allowlist: the connected phone writes its per-user notification allowlist here
// (null-/newline-separated UTF-8 bundle IDs, capped at ALLOW_PAYLOAD_MAX bytes).
// WRITE / WRITE_WO_RESP. The write is associated with the writing connHandle ->
// resolved to that phone's ANCS slot -> fed to ancsSetAllowlist. Per-slot only.
BLECharacteristic allowChar(UUID_ALLOW);
constexpr uint16_t ALLOW_PAYLOAD_MAX = 256;
// disconnect: the connected phone writes here to request that the puck drop its
// link (peripheral-initiated GAP disconnect — see UUID_DISCONN). WRITE / WRITE_WO_RESP.
BLECharacteristic disconnChar(UUID_DISCONN);
// brightness: persisted light brightness (uint8, 0..LIGHT_BRIGHTNESS_MAX). READ +
// WRITE: the owning phone reads the current value and writes a new one; the puck
// applies it to the strip and persists it. See onBrightnessWrite.
BLECharacteristic brightChar(UUID_BRIGHT);
// pickup: production per-phone unlock-event sink (UUID_PICKUP). WRITE /
// WRITE_WO_RESP only — the phone never reads it back. See onPickupWrite.
BLECharacteristic pickupChar(UUID_PICKUP);
// spike: group pickup-spike trigger (UUID_SPIKE). READ + NOTIFY. See notifySpike.
BLECharacteristic spikeChar(UUID_SPIKE);
// charging: USB/charge-state flag (UUID_CHARGE). READ + NOTIFY, uint8 0/1. See sampleBattery.
BLECharacteristic chargeChar(UUID_CHARGE);

// Cadence + bookkeeping for the once-a-minute session-time broadcast.
constexpr uint32_t SESSION_NOTIFY_MS = 60000;  // push session time every minute
static uint32_t    g_lastSessMS      = 0;       // last session-time push

// Roster is delivered as ONE notification per user (see notifyRoster). Sending
// several notifications to the SAME connection back-to-back blocks on that link's
// HVN TX semaphore (getHvnPacket), which would deadlock if done from a GATT write
// callback (the task that frees the semaphore is the one we'd be blocking). So the
// callbacks only RAISE this flag; loop() does the multi-packet send in task
// context where blocking is safe. Coalesces multiple changes into one push.
static volatile bool g_rosterDirty = false;

/// Battery level (standard BLE Battery Service 0x180F / 0x2A19)
/// - - - - - - - - - - - -
/// The puck reports its battery as a 0..100% level over the standard Battery
/// Service so the app (and any generic BLE tool) can read/subscribe it.
///
/// This board is a SuperMini nRF52840 (Nice!Nano clone). Its on-board battery
/// divider is miswired to P0.24, which is NOT an ADC-capable pin, so the old
/// external-pin read was invalid. We instead read the SAADC's internal
/// VDDHDIV5 input: the battery feeds VDDH on this board, and VDDHDIV5 measures
/// VDDH/5 with no external pin and no external divider.
BLEBas blebas;                                   // Bluefruit's Battery Service helper
constexpr uint32_t BATTERY_SAMPLE_MS = 30000;    // re-measure + push every 30s
static uint32_t    g_lastBatteryMS   = 0;
static volatile bool g_forceBattery   = false;  // request an immediate re-sample (set on connect)
static uint8_t     g_batteryPercent  = 0;

// VDDHDIV5 conversion. With the internal 0.6V reference and gain 1/6
// (AR_INTERNAL) the SAADC's full-scale input is 3.6V at 12-bit (0..4095). The
// internal input presents VDDH/5, so:
//   ADC input mV = raw * (3600.0 / 4096.0)     // = VDDH / 5
//   battery  mV  = ADC input mV * 5            // undo the internal /5 divider
// A 3.7-4.2V cell -> VDDH/5 = 0.74-0.84V, comfortably inside the 3.6V span.
#define VDDH_FULLSCALE_MV (3600.0F)              // 0.6V ref x6 (AR_INTERNAL), 12-bit
#define VDDH_ADC_MAX      (4096.0F)
#define VDDHDIV5_RATIO    (5.0F)                 // VDDHDIV5 = VDDH / 5

// Raw ADC count from the most recent battery read, kept for the serial log.
static uint16_t g_lastVbatRaw = 0;

// True while USB/VBUS is present. While charging/USB-powered VDDH rises above
// the true cell voltage, so the reading is NOT a valid state-of-charge.
static bool g_usbPresent = false;

// VBUS present? The SoftDevice is enabled (BLE) and guards the POWER
// peripheral, so query USB regulator status through the SD rather than reading
// NRF_POWER directly.
static bool isUsbPresent() {
  uint32_t usbreg = 0;
  if (sd_power_usbregstatus_get(&usbreg) != NRF_SUCCESS) return false;
  return (usbreg & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;
}

// Read the battery voltage in millivolts via the internal VDDHDIV5 input.
// This core (1.6.x) has no analogReadVDDHDIV5() wrapper and its analogRead
// helper is private, so we drive the SAADC directly: single-ended, 12-bit,
// internal 0.6V reference, gain 1/6 -> 3.6V full scale. We average 8 samples
// (after a throwaway settling read) to reject noise, then disable the SAADC.
// The core's analogRead() reconfigures the SAADC fully on every call, so no
// global ADC state needs saving/restoring here.
static float readBatteryMv() {
  NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_12bit;
  NRF_SAADC->ENABLE     = (SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos);
  for (int i = 0; i < 8; i++) {
    NRF_SAADC->CH[i].PSELN = SAADC_CH_PSELP_PSELP_NC;
    NRF_SAADC->CH[i].PSELP = SAADC_CH_PSELP_PSELP_NC;
  }
  NRF_SAADC->CH[0].CONFIG =
      ((SAADC_CH_CONFIG_RESP_Bypass       << SAADC_CH_CONFIG_RESP_Pos)   & SAADC_CH_CONFIG_RESP_Msk)
    | ((SAADC_CH_CONFIG_RESP_Bypass       << SAADC_CH_CONFIG_RESN_Pos)   & SAADC_CH_CONFIG_RESN_Msk)
    | ((SAADC_CH_CONFIG_GAIN_Gain1_6      << SAADC_CH_CONFIG_GAIN_Pos)   & SAADC_CH_CONFIG_GAIN_Msk)
    | ((SAADC_CH_CONFIG_REFSEL_Internal   << SAADC_CH_CONFIG_REFSEL_Pos) & SAADC_CH_CONFIG_REFSEL_Msk)
    | ((SAADC_CH_CONFIG_TACQ_10us         << SAADC_CH_CONFIG_TACQ_Pos)   & SAADC_CH_CONFIG_TACQ_Msk)
    | ((SAADC_CH_CONFIG_MODE_SE           << SAADC_CH_CONFIG_MODE_Pos)   & SAADC_CH_CONFIG_MODE_Msk);
  NRF_SAADC->CH[0].PSELP = SAADC_CH_PSELP_PSELP_VDDHDIV5;
  NRF_SAADC->CH[0].PSELN = SAADC_CH_PSELP_PSELP_VDDHDIV5;

  uint32_t acc = 0;
  constexpr uint8_t N = 8;
  for (uint8_t i = 0; i < N + 1; i++) {     // i == 0 is a throwaway settling read
    volatile int16_t value = 0;
    NRF_SAADC->RESULT.PTR    = (uint32_t)&value;
    NRF_SAADC->RESULT.MAXCNT = 1;

    NRF_SAADC->TASKS_START = 0x01UL;
    while (!NRF_SAADC->EVENTS_STARTED);
    NRF_SAADC->EVENTS_STARTED = 0x00UL;

    NRF_SAADC->TASKS_SAMPLE = 0x01UL;
    while (!NRF_SAADC->EVENTS_END);
    NRF_SAADC->EVENTS_END = 0x00UL;

    NRF_SAADC->TASKS_STOP = 0x01UL;
    while (!NRF_SAADC->EVENTS_STOPPED);
    NRF_SAADC->EVENTS_STOPPED = 0x00UL;

    if (value < 0) value = 0;
    if (i > 0) acc += (uint16_t)value;
  }
  NRF_SAADC->ENABLE = (SAADC_ENABLE_ENABLE_Disabled << SAADC_ENABLE_ENABLE_Pos);

  g_lastVbatRaw = (uint16_t)(acc / N);
  return g_lastVbatRaw * (VDDH_FULLSCALE_MV / VDDH_ADC_MAX) * VDDHDIV5_RATIO;
}

// Map a LiPo cell voltage (mV) to a rough 0..100% charge (Adafruit's curve).
static uint8_t mvToPercent(float mvolts) {
  if (mvolts < 3300) return 0;
  if (mvolts < 3600) { mvolts -= 3300; return (uint8_t)(mvolts / 30); }
  mvolts -= 3600;
  uint16_t pct = 10 + (uint16_t)(mvolts * 0.15F);   // 3.6V..4.2V -> 10..100%
  return pct > 100 ? 100 : (uint8_t)pct;
}

// Sample the battery and publish it on the Battery Service (value + notify).
// Logs the raw ADC count, the computed battery millivolts, and the percent so
// the reading can be verified / calibrated on the serial monitor.
static void sampleBattery() {
  const float mv = readBatteryMv();
  g_usbPresent   = isUsbPresent();

  // Publish the charge state so the app can show a charging icon (the suppressed
  // percent below would otherwise read as a stale/zero level while plugged in).
  chargeChar.write8(g_usbPresent ? 1 : 0);
  chargeChar.notify8(g_usbPresent ? 1 : 0);

  // While charging/USB-powered VDDH is inflated by the charger, so skip the
  // percent update (keep the last battery-power level) instead of reporting a
  // misleadingly high state-of-charge.
  if (g_usbPresent) {
    Serial.print("MODEM_BATT raw="); Serial.print(g_lastVbatRaw);
    Serial.print(" mv=");            Serial.print(mv, 1);
    Serial.println(" CHARGING (VDDH inflated; percent update suppressed)");
    return;
  }

  g_batteryPercent = mvToPercent(mv);
  blebas.write(g_batteryPercent);
  blebas.notify(g_batteryPercent);
  Serial.print("MODEM_BATT raw="); Serial.print(g_lastVbatRaw);
  Serial.print(" mv=");            Serial.print(mv, 1);
  Serial.print(" pct=");           Serial.println(g_batteryPercent);
}

// Diagnostic heartbeat cadence: how often loop() dumps full session + ANCS slot
// state to Serial. The MVP test surfaced multi-connection bugs that only a live
// log can pin down, so this stays on; it's a few cheap prints every few seconds.
constexpr uint32_t HEARTBEAT_MS = 5000;

/// Light-strip FX
/// - - - - - - - - - - - -
/// The BLE callbacks only raise a flag + the color; loop() runs the whole
/// sequence so strip.show() never runs in the SoftDevice callback context.
/// Effects are colored by the triggering user (green fallback if unset),
/// and never run until a color exists.
///   color write   -> spin color around board -> hold lit ~1s -> gauge -> off
///   disconnect    -> color fades in fast -> flickers out -> gauge -> off
///   notification  -> pulse the user's color a few times -> off
enum class FxPhase : uint8_t { Idle, Spin, Hold, FadeIn, Flicker, Gauge, Pulse };
static volatile bool     g_connectFx    = false;
static volatile bool     g_disconnectFx = false;
// Notification FX is requested from the BLE event task (triggerNotifyLight) with
// its own color so a match never clobbers a connect/disconnect FX mid-run; loop()
// latches it into the shared g_fxColor when it actually starts the pulse.
static volatile bool     g_notifyFx     = false;
static volatile uint32_t g_notifyColor  = 0;    // color for a pending notify pulse
static volatile uint32_t g_fxColor      = 0;    // color for the running FX
static FxPhase           g_fxPhase      = FxPhase::Idle;
static uint32_t          g_fxNextMS     = 0;     // next phase/frame time
static uint32_t          g_fxStartMS    = 0;     // start of a time-based phase
static uint16_t          g_fxStep       = 0;     // step within a phase
constexpr uint32_t SPIN_STEP_MS  = 45;    // per-LED spin speed
constexpr uint32_t HOLD_MS       = 1000;  // board lit after the spin
constexpr uint32_t FADEIN_MS     = 150;   // fast fade-in on disconnect
constexpr uint32_t FX_FRAME_MS   = 20;    // fade-in frame interval
constexpr uint8_t  FLICKER_STEPS = 6;     // on/off toggles when flickering out
constexpr uint32_t FLICKER_MS    = 55;    // per flicker toggle
constexpr uint8_t  NOTIFY_PULSES  = 3;    // ramp up+down cycles on a notification
constexpr uint32_t NOTIFY_HALF_MS = 220;  // duration of one ramp (up OR down)
constexpr uint32_t GAUGE_MS      = 600;   // user-count display duration

/// State + Session
/// - - - - - - - - - - - -
/// CURR_STATE is the device power mode. g_session holds the live set of
/// connected users; it exists (Active) whenever userCount > 0 and is torn
/// down (Idle) once the last user leaves.
State   CURR_STATE = State::Idle;
Session g_session;

// Audio chime requests. Raised from the BLE event task (startSession / addUser /
// onColorWrite-reclaim / finalizeLeave / triggerNotifyLight) and consumed in
// loop(), mirroring the visual-FX flag pattern (g_connectFx etc.) so I2S/DMA
// playback is never kicked off from inside a GATT write callback. The sound gate
// itself lives in the audio module (playChime reads puckSettings().soundEnabled).
static volatile bool g_chimeStart  = false;   // first user joined -> session start
static volatile bool g_chimeJoin   = false;   // a subsequent user joined
static volatile bool g_chimeLeave  = false;   // a user left
static volatile bool g_chimeNotify = false;   // allowlisted notification for a member
static volatile bool g_claimFx     = false;   // puck claimed/taken over -> green pulse + claim chime

/// Phone-requested disconnect queue
/// - - - - - - - - - - - -
/// A phone asks the puck to sever its link by writing the disconnect
/// characteristic (UUID_DISCONN). The write callback only ENQUEUES the conn
/// handle here; loop() drains the queue and issues the GAP disconnect, so we
/// never call into the SoftDevice from deep inside a GATT write callback. SPSC
/// ring (producer: write callback / BLE task; consumer: loop()); sized one past
/// MAX_USERS so a full session can request teardown without overrun.
constexpr uint8_t DISC_Q_SIZE = MAX_USERS + 1;
static volatile uint16_t g_discQueue[DISC_Q_SIZE];
static volatile uint8_t  g_discHead = 0;   // consumer (loop)
static volatile uint8_t  g_discTail = 0;   // producer (callback)


/// Leave grace + intentional-leave tracking
/// - - - - - - - - - - - -
/// On iOS the app link and the system ANCS link are the SAME GAP connection,
/// which iOS drops and silently reopens on its own. So a spontaneous disconnect
/// is usually transient: instead of evicting the member immediately (which blinks
/// them off every other phone's leaderboard and kills their notifications), we
/// hold them for LEAVE_GRACE_MS. A quick reconnect + identity re-write reclaims
/// the slot (reclaimPendingLeave); if the grace expires, loop() finalizes the
/// leave. An APP-REQUESTED leave (the fb59000B disconnect) is intentional and
/// must finalize at once — those handles are recorded here so disconnectCallback
/// can tell the two apart.
constexpr uint32_t LEAVE_GRACE_MS = 6000;
static uint16_t g_intentionalLeave[MAX_USERS];
static uint8_t  g_intentionalLeaveCount = 0;

// Record that conn_handle's upcoming disconnect was requested by the app (so it
// finalizes immediately, not after the grace window). Deduplicated; bounded.
static void markIntentionalLeave(uint16_t conn_handle) {
  for (uint8_t i = 0; i < g_intentionalLeaveCount; i++)
    if (g_intentionalLeave[i] == conn_handle) return;
  if (g_intentionalLeaveCount < MAX_USERS)
    g_intentionalLeave[g_intentionalLeaveCount++] = conn_handle;
}

// Was conn_handle's disconnect app-requested? Removes it from the set if so
// (swap-with-last), so a later reused handle doesn't inherit the flag.
static bool takeIntentionalLeave(uint16_t conn_handle) {
  for (uint8_t i = 0; i < g_intentionalLeaveCount; i++) {
    if (g_intentionalLeave[i] == conn_handle) {
      g_intentionalLeave[i] = g_intentionalLeave[--g_intentionalLeaveCount];
      return true;
    }
  }
  return false;
}


/// Helpers
/// - - - - - - - - - - - -
static uint32_t parseRGB(const char* s) {
  int r = 0, g = 0, b = 0;
  sscanf(s, "%d,%d,%d", &r, &g, &b);
  r = constrain(r, 0, 255);
  g = constrain(g, 0, 255);
  b = constrain(b, 0, 255);
  Serial.print("color rx: \""); Serial.print(s);
  Serial.print("\" -> "); Serial.print(r); Serial.print(',');
  Serial.print(g); Serial.print(','); Serial.println(b);
  return strip.Color((uint8_t)r, (uint8_t)g, (uint8_t)b);
}

// Fill the whole strip with one color. Gamma-correct here so every fill path
// (incl. showColorScaled's fades, which call through this) lands on the LED's
// perceptual curve rather than the raw linear ramp that washes colors out.
static void showColor(uint32_t c) {
  c = strip.gamma32(c);
  for (uint16_t i = 0; i < NUM_LIGHTS; i++) strip.setPixelColor(i, c);
  strip.show();
}

// Light a fraction of the strip proportional to connected users
// (userCount / MAX_USERS), rounded to the nearest LED.
static void showUserGauge() {
  uint16_t lit = (uint16_t)(((uint32_t)g_session.userCount * NUM_LIGHTS
                             + MAX_USERS / 2) / MAX_USERS);
  strip.clear();
  for (uint16_t i = 0; i < lit; i++)
    strip.setPixelColor(i, strip.Color(255, 255, 255));   // white gauge
  strip.show();
}

// Fill the strip with a color dimmed to scale/255 (used for the fades).
static void showColorScaled(uint32_t c, uint8_t scale) {
  uint8_t r = (uint8_t)((((c >> 16) & 0xFF) * scale) / 255);
  uint8_t g = (uint8_t)((((c >>  8) & 0xFF) * scale) / 255);
  uint8_t b = (uint8_t)((((c      ) & 0xFF) * scale) / 255);
  showColor(strip.Color(r, g, b));
}

// Effects need a visible color; fall back to green when a user has none.
static uint32_t colorOrFallback(uint32_t c) {
  return c ? c : strip.Color(0, 150, 0);   // green
}

// Spin frame: light LEDs 0..head so the lit arc grows as the head travels
// around, leaving the whole board lit by the end of the lap.
static void showSpin(uint32_t c, uint16_t head) {
  c = strip.gamma32(c);                     // match showColor's perceptual curve
  strip.clear();
  for (uint16_t i = 0; i <= head && i < NUM_LIGHTS; i++)
    strip.setPixelColor(i, c);
  strip.show();
}

// Speaker audio now lives in modem_audio.* (I2S/DMA chime playback for the
// MAX98357A amp). Chimes are requested via the g_chime* flags and played in
// loop(); the sound-enabled gate is enforced inside playChime().

/// ANCS notify-light trigger
/// - - - - - - - - - - - -
/// Called by ancs_client when an Added notification's app bundle ID matches the
/// allowlist for `slotIndex` (an ANCS slot). Pulses the strip in THAT user's
/// color. Only joined session members animate: a notification on a connection
/// that never joined (e.g. a system ANCS reconnect that re-subscribed on its own)
/// is logged and ignored, so it can't flash the green fallback.
void triggerNotifyLight(uint8_t slotIndex) {
  uint16_t conn = ancsConnHandle(slotIndex);   // ANCS slot -> conn handle
  User*    u    = findUser(conn);              // conn handle -> user (color)
  Serial.print("MODEM_ANCS triggerNotifyLight slot="); Serial.print(slotIndex);
  Serial.print(" conn=0x"); Serial.print(conn, HEX);
  Serial.print(" color=0x"); Serial.println(u ? u->color : 0, HEX);
  if (!u) {   // not a session member -> no light
    Serial.println("MODEM_ANCS notify on non-member conn, ignoring light");
    return;
  }
  // Runs in the BLE event task: only raise a flag + remember the color (green
  // fallback if the member somehow has no color). loop() runs the pulse so
  // strip.show() never executes in callback context (this file's cardinal rule).
  g_notifyColor = colorOrFallback(u->color);
  g_notifyFx    = true;
  g_chimeNotify = true;          // allowlisted notification for a member -> notify chime
}

/// Session / Users
/// - - - - - - - - - - - -
/// Users are stored by value inside g_session and looked up by their BLE
/// connHandle. The session is created on the first connect and destroyed
/// when the last user disconnects, carrying the device State with it.

// Push the current user count to every connected phone. The characteristic
// value is also kept current so a late reader (or a fresh connect) sees the
// right number without waiting for the next join/leave.
static void notifyUserCount() {
  const uint8_t n = g_session.userCount;
  countChar.write8(n);
  for (uint8_t i = 0; i < g_session.userCount; i++)
    countChar.notify(g_session.connectedUsers[i].connHandle, &n, 1);
}

// Push the canonical session time (seconds since the first user joined) to
// every connected phone so all devices share one synced clock. Zero while Idle.
static void notifySessionTime() {
  uint16_t secs = 0;
  if (CURR_STATE == State::Active && g_session.startTimeMS) {
    uint32_t elapsed = (millis() - g_session.startTimeMS) / 1000;
    secs = elapsed > 0xFFFF ? 0xFFFF : (uint16_t)elapsed;
  }
  sessChar.write16(secs);
  for (uint8_t i = 0; i < g_session.userCount; i++)
    sessChar.notify16(g_session.connectedUsers[i].connHandle, secs);
}

// Serialize the connected-user table and push it to every phone so each can
// render the full leaderboard. Layout (little-endian):
//   [0]    userCount
//   per user: R, G, B, pickups(uint16), nameLen, name[nameLen]
// The per-user uint16 carries this user's PICKUP count (phone unlocks) so each
// phone can render the pickups leaderboard. The characteristic value is kept
// current so a fresh reader sees the roster without waiting for the next change.
//
// PER-USER PACKET layout (one BLE notification per user, little-endian):
//   [0] userCount   total members (lets the phone know how many to expect)
//   [1] index       this user's 0-based position in the table
//   [2] R [3] G [4] B
//   [5] pickups lo  [6] pickups hi
//   [7] nameLen     (capped to fit one notification)
//   [8..] name
// The phone collects packets by index until it has all `userCount` and then
// renders the leaderboard. Each packet fits the 20-byte payload of a 23-byte MTU
// (header 8 + up to 12 name bytes), so nothing is ever truncated — the reason we
// page instead of sending one big roster (which doesn't fit, and a bigger MTU
// overflows SoftDevice RAM). The readable characteristic value is kept as the full
// UNPAGED table for a one-shot debug/blob read; live delivery is the packets.
constexpr uint8_t ROSTER_PKT_HEADER = 8;   // count,index,R,G,B,pkLo,pkHi,nameLen

// Update the readable roster value and request a paged push. Callable from GATT
// write/CCCD callbacks: it never notifies here (that can block) — loop() does.
static void notifyRoster() {
  uint8_t buf[1 + MAX_USERS * (3 + 2 + 1 + MAX_USER_NAME_LEN)];
  uint16_t n = 0;
  buf[n++] = g_session.userCount;
  for (uint8_t i = 0; i < g_session.userCount; i++) {
    const User& u = g_session.connectedUsers[i];
    buf[n++] = (uint8_t)((u.color >> 16) & 0xFF);   // R
    buf[n++] = (uint8_t)((u.color >>  8) & 0xFF);   // G
    buf[n++] = (uint8_t)( u.color        & 0xFF);   // B
    buf[n++] = (uint8_t)( u.pickups        & 0xFF);
    buf[n++] = (uint8_t)((u.pickups >>  8) & 0xFF);
    uint8_t len = (uint8_t)strnlen(u.name, MAX_USER_NAME_LEN);
    buf[n++] = len;
    memcpy(&buf[n], u.name, len);
    n += len;
  }
  rosterChar.write(buf, n);        // readable (unpaged) value for blob/debug reads
  g_rosterDirty = true;            // loop() sends the per-user notifications
}

// Send the roster as one notification per user to every member. MUST run in task
// context (loop()), never a callback: notifying the same connection repeatedly
// blocks on its HVN TX semaphore until each packet transmits. Drains g_rosterDirty.
static void pushRosterPaged() {
  for (uint8_t m = 0; m < g_session.userCount; m++) {
    const uint16_t conn = g_session.connectedUsers[m].connHandle;
    BLEConnection* c = Bluefruit.Connection(conn);
    const uint16_t usable = (c && c->getMtu() > 3) ? (uint16_t)(c->getMtu() - 3) : 20;
    const uint8_t  maxName = usable > ROSTER_PKT_HEADER
                               ? (uint8_t)(usable - ROSTER_PKT_HEADER) : 0;
    for (uint8_t i = 0; i < g_session.userCount; i++) {
      const User& u = g_session.connectedUsers[i];
      uint8_t pkt[ROSTER_PKT_HEADER + MAX_USER_NAME_LEN];
      uint16_t n = 0;
      pkt[n++] = g_session.userCount;
      pkt[n++] = i;
      pkt[n++] = (uint8_t)((u.color >> 16) & 0xFF);
      pkt[n++] = (uint8_t)((u.color >>  8) & 0xFF);
      pkt[n++] = (uint8_t)( u.color        & 0xFF);
      pkt[n++] = (uint8_t)( u.pickups        & 0xFF);
      pkt[n++] = (uint8_t)((u.pickups >>  8) & 0xFF);
      uint8_t len = (uint8_t)strnlen(u.name, MAX_USER_NAME_LEN);
      if (len > maxName) len = maxName;          // keep the packet within one MTU
      pkt[n++] = len;
      memcpy(&pkt[n], u.name, len);
      n += len;
      rosterChar.notify(conn, pkt, n);
    }
  }
}

// CCCD-subscribe backstop for the join-time race. On the nRF52 the SoftDevice
// auto-serves characteristic READS from the stored value immediately, but the
// color write's callback (onColorWrite -> joinUser, which sets the count and fills
// the roster) is dispatched LATER from Bluefruit's event task. So a phone that
// writes its color and then reads count/roster during connect can be served the
// STALE pre-join values, and the join-time notify is dropped because the phone
// hasn't finished subscribing yet — leaving "0/8" and an empty leaderboard.
//
// Fix: when a phone ENABLES notifications (writes CCCD=1) on a state
// characteristic, push that characteristic's CURRENT value to it right then. The
// CCCD write is processed AFTER the earlier color write on the same Bluefruit
// event task, so the join is already applied AND the phone is, by definition, now
// subscribed — no timing guess. Replaces the old g_rosterRepushMS delay, which
// fired too early (timed from the join, not from the later roster subscribe).
static void onCountSubscribe(uint16_t conn_handle, BLECharacteristic*, uint16_t value) {
  if (!(value & 0x0001)) return;                 // only on enable-notify
  const uint8_t n = g_session.userCount;
  countChar.notify(conn_handle, &n, 1);
}
static void onSessionSubscribe(uint16_t conn_handle, BLECharacteristic*, uint16_t value) {
  if (!(value & 0x0001)) return;
  uint16_t secs = 0;
  if (CURR_STATE == State::Active && g_session.startTimeMS) {
    uint32_t elapsed = (millis() - g_session.startTimeMS) / 1000;
    secs = elapsed > 0xFFFF ? 0xFFFF : (uint16_t)elapsed;
  }
  sessChar.notify16(conn_handle, secs);
}
static void onRosterSubscribe(uint16_t conn_handle, BLECharacteristic*, uint16_t value) {
  if (!(value & 0x0001)) return;
  // Re-push the full table; the just-subscribed phone now receives current state.
  notifyRoster();
}

// Find the User that owns a connection (nullptr if none).
static User* findUser(uint16_t conn_handle) {
  for (uint8_t i = 0; i < g_session.userCount; i++)
    if (g_session.connectedUsers[i].connHandle == conn_handle)
      return &g_session.connectedUsers[i];
  return nullptr;
}

// First user in -> spin up a session and go Active.
static void startSession() {
  g_session.ID++;
  g_session.startTimeMS = millis();
  CURR_STATE = State::Active;
  g_chimeStart = true;            // host: first user joined -> session-start chime
  Serial.print("session "); Serial.print(g_session.ID);
  Serial.println(" started -> Active");
}

// Defined later (with the spike-detection block); declared here so endSession can
// clear the rolling window + cooldown when the last user leaves.
static void resetPickupSpikeState();

// Last user out -> tear the session down and go Idle.
static void endSession() {
  g_session.startTimeMS = 0;
  CURR_STATE = State::Idle;
  resetPickupSpikeState();        // fresh session starts quiet + responsive
  Serial.println("session ended -> Idle");
}

// Register a freshly-connected central as a User, creating the session if
// it is the first one. Returns nullptr if we are already full.
static User* addUser(uint16_t conn_handle) {
  if (g_session.userCount >= MAX_USERS) return nullptr;
  bool firstUser = (g_session.userCount == 0);

  User& u = g_session.connectedUsers[g_session.userCount];
  u = User{};                    // zero-init (screenTime left for later)
  u.ID         = g_session.userCount;
  u.connHandle = conn_handle;
  BLEConnection* c = Bluefruit.Connection(conn_handle);
  if (c) c->getPeerName(u.name, sizeof(u.name));   // phone's name, if any
  g_session.userCount++;

  if (firstUser) startSession();   // raises g_chimeStart (session-start chime)
  else           g_chimeJoin = true;   // a subsequent member -> join chime
  return &u;
}

// Promote a BLE connection to a session member on its first identity write
// (color or userName). A bare BLE connection is NOT a join: iOS keeps/reopens a
// bonded ANCS link on its own (the "phantom reconnect"), and that link has no app
// behind it, so it never writes an identity and never joins here. Idempotent —
// returns the existing user if this conn already joined. Pushes the join-time
// broadcasts (count / roster / synced clock) exactly once, when the user is added.
static User* joinUser(uint16_t conn_handle) {
  User* u = findUser(conn_handle);
  if (u) return u;                 // already a member (later identity writes)

  u = addUser(conn_handle);
  if (!u) return nullptr;          // session full

  // Now that this connection is a real session member, subscribe its ANCS feed.
  // ANCS subscription is gated on the join so a phantom reconnect (iOS reopening
  // the bonded link with no app behind it) never subscribes and never steals
  // notification delivery from the real, joined connection.
  ancsOnJoin(conn_handle);

  // Tell every phone the new roster size, the freshly joined table, and the
  // current session clock (so this joiner sees the synced time immediately).
  notifyUserCount();
  notifyRoster();
  notifySessionTime();
  g_lastSessMS = millis();
  // NOTE: the join-time count/roster/session pushes above can race the joining
  // phone's own subscribe (it may not have written CCCD=1 yet). The CCCD-write
  // callbacks (onCountSubscribe / onRosterSubscribe / onSessionSubscribe) re-push
  // current state the instant that phone subscribes, which is the reliable fix.

  Serial.print("join: users="); Serial.println(g_session.userCount);
  return u;
}

// Drop a disconnected central, keeping the array packed. Ends the session
// when the last user leaves.
static void removeUser(uint16_t conn_handle) {
  for (uint8_t i = 0; i < g_session.userCount; i++) {
    if (g_session.connectedUsers[i].connHandle == conn_handle) {
      for (uint8_t j = i; j + 1 < g_session.userCount; j++)
        g_session.connectedUsers[j] = g_session.connectedUsers[j + 1];
      g_session.userCount--;
      break;
    }
  }
  if (g_session.userCount == 0) endSession();
}

// Finalize a member's departure: leave FX in their color, drop them from the
// roster, and tell the remaining phones. Used both for an app-requested leave
// (immediately, from disconnectCallback) and for a spontaneous drop whose grace
// window expired without a reconnect (from loop()).
static void finalizeLeave(uint16_t conn_handle) {
  User* u = findUser(conn_handle);
  if (!u) return;
  g_fxColor      = colorOrFallback(u->color);
  g_disconnectFx = true;
  g_chimeLeave   = true;          // leave chime (endSession below may also fire if last out)
  removeUser(conn_handle);
  notifyUserCount();
  notifyRoster();
  Serial.print("leave finalized: users="); Serial.println(g_session.userCount);
}

// Re-adopt a member that dropped spontaneously and is now reconnecting. A drop
// keeps the user in the roster (pendingLeaveMS set) but frees their old ANCS
// slot; the phone then reconnects on a NEW connHandle and re-writes its color.
// Colors are unique within a session (join-time collision resolution), so a
// pending-leave user with this exact color IS the same person reclaiming their
// seat: rebind them to the new handle, clear the grace flag, and keep their
// accrued screen time. Returns the reclaimed user, or nullptr if none matched.
static User* reclaimPendingLeave(uint16_t newHandle, uint32_t color) {
  for (uint8_t i = 0; i < g_session.userCount; i++) {
    User& u = g_session.connectedUsers[i];
    if (u.pendingLeaveMS != 0 && u.color == color) {
      Serial.print("reclaim: conn 0x"); Serial.print(u.connHandle, HEX);
      Serial.print(" -> 0x"); Serial.println(newHandle, HEX);
      u.connHandle     = newHandle;
      u.pendingLeaveMS = 0;
      return &u;
    }
  }
  return nullptr;
}

// Periodic diagnostic dump (driven by the heartbeat in loop()): device state,
// live link count, and each member's color / screen time / grace status,
// followed by the per-slot ANCS state. Read-only — safe to call from loop().
static void dumpState() {
  Serial.print("MODEM_state t="); Serial.print(millis());
  Serial.print(" state=");
  Serial.print(CURR_STATE == State::Active ? "Active" : "Idle");
  Serial.print(" links="); Serial.print(Bluefruit.connected());
  Serial.print(" users="); Serial.println(g_session.userCount);
  for (uint8_t i = 0; i < g_session.userCount; i++) {
    const User& u = g_session.connectedUsers[i];
    BLEConnection* c = Bluefruit.Connection(u.connHandle);
    Serial.print("  user["); Serial.print(i); Serial.print("] conn=0x");
    Serial.print(u.connHandle, HEX);
    Serial.print(" mtu="); Serial.print(c ? c->getMtu() : 0);
    Serial.print(" color=0x"); Serial.print(u.color, HEX);
    Serial.print(" stime="); Serial.print(u.screenTime);
    Serial.print(" pickups="); Serial.print(u.pickups);
    Serial.print(" name=\""); Serial.print(u.name); Serial.print("\"");
    if (u.pendingLeaveMS != 0) {
      Serial.print(" [leaving ");
      Serial.print(millis() - u.pendingLeaveMS);
      Serial.print("ms]");
    }
    Serial.println();
  }
  ancsDumpState();
}

// Called whenever a central connects to the puck. A connection is NOT yet a
// session join: we start ANCS (so notifications work) but defer addUser/roster/FX
// until the phone sends its identity (see joinUser / onColorWrite). This is what
// makes a system ANCS reconnect — which has no app behind it and never writes an
// identity — invisible to the session instead of a "phantom user".
static void connectCallback(uint16_t conn_handle) {
  // Start this slot's per-connection ANCS client (requests bonding; discovery +
  // subscribe happen once the link is secured). ancs_client keys by conn_handle
  // and owns its own slot; userHint is log-only and unknown until a join, so 0xFF.
  ancsOnConnect(conn_handle, 0xFF);

  // Write the device name back to the phone: push it as a notification
  // (the characteristic value is also readable as a fallback). Sourced from the
  // persisted settings so a renamed puck reports its current name on connect.
  const char* puckName = puckSettings().name;
  nameChar.notify(conn_handle, puckName, strlen(puckName));

  // Log the link count and this link's MTU. The MTU exchange may not have
  // completed yet at connect time, so this is the lower bound; the heartbeat
  // (dumpState) logs the settled per-user MTU for verifying the roster fix.
  BLEConnection* c = Bluefruit.Connection(conn_handle);
  Serial.print("BLE connect: links="); Serial.print(Bluefruit.connected());
  Serial.print(" mtu="); Serial.println(c ? c->getMtu() : 0);

  // Force a battery+charge re-sample on the next loop pass so the app's quick
  // connect-and-read sees a current charge state (not one up to 30s stale).
  // Flagged here; the ADC read itself runs in loop(), not callback context.
  g_forceBattery = true;

  // Keep advertising while SESSION-MEMBER slots remain — gate on userCount, NOT
  // the raw physical link count. iOS background/phantom links inflate
  // Bluefruit.connected() without being session members; gating on links stopped
  // advertising early and hid the puck from later phones while seats were still
  // open. (Bounded by the physical ceiling regardless: begin(MAX_USERS) is what
  // ultimately refuses a connection once all physical links are in use.)
  if (g_session.userCount < MAX_USERS)
    Bluefruit.Advertising.start(0);
}

// Called whenever a central disconnects.
static void disconnectCallback(uint16_t conn_handle, uint8_t reason) {
  // Tear down this slot's ANCS client first (halt fetches, drop allowlist,
  // reset buffers). Always runs — even for a connection that never joined the
  // session (e.g. a phantom ANCS reconnect). Idempotent + matches the exact
  // conn_handle; safe whether or not the user slot is reclaimed below.
  ancsOnDisconnect(conn_handle);

  // Only a connection that actually JOINED the session affects the roster/FX. A
  // non-member drop (phantom reconnect, or a link that never sent its identity)
  // changes nothing visible — no leave FX, no count/roster churn.
  User* u = findUser(conn_handle);
  if (!u) {
    Serial.print("BLE disconnect (non-member): links="); Serial.println(Bluefruit.connected());
    return;
  }

  if (takeIntentionalLeave(conn_handle)) {
    // The app explicitly asked to leave (fb59000B) — finalize now, no grace.
    Serial.print("BLE disconnect (intentional) conn=0x"); Serial.println(conn_handle, HEX);
    finalizeLeave(conn_handle);
  } else {
    // Spontaneous drop (iOS churn / brief out-of-range). Hold the member through
    // the grace window: leave the roster untouched so the OTHER phones don't see
    // them blink out, and let a fast reconnect + color re-write reclaim the slot
    // (reclaimPendingLeave). loop() finalizes the leave only if grace expires.
    if (u->pendingLeaveMS == 0) u->pendingLeaveMS = millis();
    Serial.print("BLE disconnect (spontaneous) conn=0x"); Serial.print(conn_handle, HEX);
    Serial.print(" reason=0x"); Serial.print(reason, HEX);
    Serial.print(" -> holding "); Serial.print(LEAVE_GRACE_MS);
    Serial.println("ms for grace");
  }
  // restartOnDisconnect(true) re-advertises automatically.
}

// Bluefruit write-callback for the screen-time characteristic. Each phone
// reports its own accrued screen time as a little-endian uint16 (seconds); we
// store it against the user that owns the writing connection.
static void onScreenTimeWrite(uint16_t conn_handle, BLECharacteristic* /*chr*/,
                              uint8_t* data, uint16_t len) {
  if (len < 2) return;
  uint16_t secs = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
  User* u = findUser(conn_handle);
  if (u) u->screenTime = secs;
  Serial.print("screenTime rx conn="); Serial.print(conn_handle);
  Serial.print(" -> "); Serial.println(secs);
  // Screen time feeds the leaderboard — refresh every phone's roster.
  notifyRoster();
}

// Bluefruit write-callback for the per-user name characteristic. Each phone
// uploads its chosen display name right after connecting; we store it on the
// user that owns the writing connection so it can appear in the roster.
static void onUserNameWrite(uint16_t conn_handle, BLECharacteristic* /*chr*/,
                            uint8_t* data, uint16_t len) {
  // First identity write joins this connection to the session (no-op if already).
  User* u = joinUser(conn_handle);
  if (!u) return;                  // session full
  uint16_t n = len < MAX_USER_NAME_LEN - 1 ? len : MAX_USER_NAME_LEN - 1;
  memcpy(u->name, data, n);
  u->name[n] = '\0';
  Serial.print("userName rx conn="); Serial.print(conn_handle);
  Serial.print(" -> "); Serial.println(u->name);
  notifyRoster();
}

// Canonical color palette, mirroring the Flutter app's kPalette (lib/settings.dart)
// in palette order. Stored as 0x00RRGGBB to match strip.Color() / User.color. The
// app already prevents two phones from picking the same color (it greys out taken
// swatches and runs its own collision modal), so this table exists as a FIRMWARE
// BACKSTOP: it makes the "colors are unique per session" invariant that
// reclaimPendingLeave / findUserByColor depend on true in code, covering the race
// where two phones commit the same color before the roster propagates.
//   FUTURE: identity should key on a stable per-user ID (User.ID already exists)
//   echoed by the app, retiring color-as-identity entirely. Out of scope here.
static const uint32_t COLOR_PALETTE[MAX_USERS] = {
  0xEC7D3E,  // Coral
  0x6F86F2,  // Indigo
  0xF4E07A,  // Lemon
  0x74E36B,  // Lime
  0xE070E0,  // Orchid
  0xFF4D4D,  // Crimson
  0x4FC3F7,  // Sky
  0x9B6BF0,  // Violet
};

// True if any CURRENT member other than `exceptHandle` already owns `color`.
static bool colorTakenByOther(uint32_t color, uint16_t exceptHandle) {
  for (uint8_t i = 0; i < g_session.userCount; i++) {
    const User& u = g_session.connectedUsers[i];
    if (u.connHandle != exceptHandle && u.color == color) return true;
  }
  return false;
}

// Resolve a requested color to a unique one. If no other member holds it, the
// request stands. On a collision, nudge to the first FREE palette entry so the
// uniqueness invariant holds; if the whole palette is somehow occupied (a full
// 8-user session), keep the request unchanged (joinUser will reject the write
// anyway when the session is full).
static uint32_t resolveUniqueColor(uint32_t requested, uint16_t conn_handle) {
  if (!colorTakenByOther(requested, conn_handle)) return requested;
  for (uint8_t i = 0; i < MAX_USERS; i++) {
    if (!colorTakenByOther(COLOR_PALETTE[i], conn_handle)) {
      Serial.print("color collision 0x"); Serial.print(requested, HEX);
      Serial.print(" -> nudged to palette[0x"); Serial.print(COLOR_PALETTE[i], HEX);
      Serial.println("]");
      return COLOR_PALETTE[i];
    }
  }
  Serial.print("color collision 0x"); Serial.print(requested, HEX);
  Serial.println(" but palette full -> kept");
  return requested;
}

// Bluefruit write-callback signature.
static void onColorWrite(uint16_t conn_handle, BLECharacteristic* /*chr*/,
                         uint8_t* data, uint16_t len) {
  char buf[33];
  uint16_t n = len < sizeof(buf) - 1 ? len : sizeof(buf) - 1;
  memcpy(buf, data, n);
  buf[n] = '\0';

  uint32_t color = parseRGB(buf);

  // Reconnect fast-path: if this color belongs to a member who just dropped and
  // is still inside their leave grace, this write is them reclaiming their seat
  // — rebind to the new connHandle (keeping screen time) and re-subscribe ANCS,
  // instead of joining a duplicate that would later be removed by the grace
  // sweep. Colors are unique per session, so a match here is unambiguous.
  User* reclaimed = reclaimPendingLeave(conn_handle, color);
  if (reclaimed) {
    reclaimed->color = color;
    ancsOnJoin(conn_handle);     // re-enable this user's notifications on the new slot
    notifyRoster();              // resync any reader; the table itself is unchanged
    g_fxColor   = colorOrFallback(color);
    g_connectFx = true;
    g_chimeJoin = true;          // reconnect within grace -> treat as a join chime
    return;
  }

  // First identity write joins this connection to the session (no-op if already).
  // A phantom ANCS reconnect never sends a color, so it never joins or animates.
  User* u = joinUser(conn_handle);
  if (!u) return;                  // session full — ignore the write
  // Enforce per-session color uniqueness so reclaimPendingLeave / findUserByColor
  // (which key identity on color) can never alias two members (backstop; the app
  // already prevents collisions). On a clash this nudges to a free palette color.
  color    = resolveUniqueColor(color, conn_handle);
  u->color = color;

  // Color shows in each leaderboard row — refresh every phone's roster.
  notifyRoster();

  // A color write drives the connect FX (spin -> hold -> gauge -> off),
  // colored by this user (green fallback if somehow unset).
  g_fxColor   = colorOrFallback(color);
  g_connectFx = true;
}

// Bluefruit write-callback for the per-connection notification allowlist.
// The connected phone writes ITS bundle-ID list (null-/newline-separated UTF-8).
// We resolve the WRITING connHandle to its ANCS slot (NOT to a fixed index —
// handles are opaque/reused) and hand the raw payload to that slot's parser.
// Allowlists are strictly per-slot; nothing is shared or merged across slots.
static void onAllowlistWrite(uint16_t conn_handle, BLECharacteristic* /*chr*/,
                             uint8_t* data, uint16_t len) {
  int slot = ancsSlotForHandle(conn_handle);   // connHandle -> ANCS slot
  Serial.print("MODEM_ANCS allowlist write conn=0x"); Serial.print(conn_handle, HEX);
  Serial.print(" len="); Serial.print(len);
  Serial.print(" -> slot="); Serial.println(slot);
  if (slot < 0) return;                          // no ANCS slot for this conn
  if (len > ALLOW_PAYLOAD_MAX) len = ALLOW_PAYLOAD_MAX;   // bounded
  ancsSetAllowlist((uint8_t)slot, data, len);
}

// Owner (claim) write-callback. The phone writes a 16-byte key to claim an
// unclaimed puck or to take over a claimed one. Claim semantics:
//   - UNCLAIMED            -> adopt the key, claim, RESET settings to DEFAULTS.
//   - CLAIMED, same key    -> NO-OP (guards reconnect re-sends of the same key).
//   - CLAIMED, diff key    -> takeover: overwrite the key, RESET to DEFAULTS.
// ANY ownership change resets name/sound/brightness to DEFAULTS (keeping the new
// key). The readable Owner value is a 0/1 owned-flag (the raw key is never read
// back). Firmware trusts the write; the app gates takeover behind a confirmation.
static void onOwnerWrite(uint16_t conn_handle, BLECharacteristic* /*chr*/,
                         uint8_t* data, uint16_t len) {
  if (len != PUCK_OWNER_KEY_LEN) {
    Serial.print("owner write ignored: len="); Serial.println(len);
    return;
  }
  PuckSettings& s = puckSettings();
  if (s.claimed && memcmp(s.ownerKey, data, PUCK_OWNER_KEY_LEN) == 0) {
    Serial.println("owner write: same key -> no-op");
    return;                        // reconnect re-send: never reset, never rewrite flash
  }

  // Unclaimed -> claim, or a different key -> takeover. Adopt the new key, mark
  // claimed, and reset the content settings to DEFAULTS (reset keeps the key).
  memcpy(s.ownerKey, data, PUCK_OWNER_KEY_LEN);
  s.claimed = 1;
  puckSettingsResetToDefaults();
  puckSettingsSave();

  // Apply the freshly-reset state to the hardware + readable characteristics.
  // (The sound gate reads puckSettings().soundEnabled live, so nothing to mirror.)
  strip.setBrightness(s.brightness);
  ownerChar.write8(s.claimed);     // owned-flag now reads 1
  nameChar.write(s.name);
  soundChar.write8(s.soundEnabled);
  brightChar.write8(s.brightness);
  // Echo the (default) name back to the claiming phone so it reads defaults.
  nameChar.notify(conn_handle, s.name, strlen(s.name));
  // Confirm the ownership change with a brief green pulse + claim chime. Deferred
  // to loop() (this is a GATT write callback — no strip.show()/I2S here).
  g_claimFx = true;
  Serial.println("owner write: claimed/took over -> settings reset to DEFAULTS");
}

// Name write-callback: rename the device. Stored in the persisted settings
// (debounced) and reflected in the readable Name characteristic value.
static void onNameWrite(uint16_t conn_handle, BLECharacteristic* /*chr*/,
                        uint8_t* data, uint16_t len) {
  PuckSettings& s = puckSettings();
  uint16_t n = len < MAX_DEVICE_NAME_LEN - 1 ? len : MAX_DEVICE_NAME_LEN - 1;
  memcpy(s.name, data, n);
  s.name[n] = '\0';
  puckSettingsSave();
  nameChar.write(s.name);
  nameChar.notify(conn_handle, s.name, strlen(s.name));
  Serial.print("name write -> "); Serial.println(s.name);
}

// Sound write-callback: enable/disable interaction sound. Persisted to the
// settings record (the live source of truth the chime gate reads in playChime)
// and echoed to the readable Sound value. Takes effect on the very next chime.
static void onSoundWrite(uint16_t /*conn_handle*/, BLECharacteristic* /*chr*/,
                         uint8_t* data, uint16_t len) {
  if (len < 1) return;
  PuckSettings& s = puckSettings();
  s.soundEnabled = data[0] ? 1 : 0;
  puckSettingsSave();
  soundChar.write8(s.soundEnabled);
  Serial.print("sound write -> "); Serial.println(s.soundEnabled);
}

// Brightness write-callback: set the light brightness (clamped to
// 0..LIGHT_BRIGHTNESS_MAX). Persisted, applied to the strip, and mirrored to the
// readable Brightness value. strip.show() re-renders whatever is currently lit
// so a change is visible immediately when an effect is on screen.
static void onBrightnessWrite(uint16_t /*conn_handle*/, BLECharacteristic* /*chr*/,
                              uint8_t* data, uint16_t len) {
  if (len < 1) return;
  PuckSettings& s = puckSettings();
  uint8_t b = data[0] > LIGHT_BRIGHTNESS_MAX ? LIGHT_BRIGHTNESS_MAX : data[0];
  s.brightness = b;
  puckSettingsSave();
  strip.setBrightness(b);
  strip.show();
  brightChar.write8(s.brightness);
  Serial.print("brightness write -> "); Serial.println(s.brightness);
}

// Bluefruit write-callback for the disconnect characteristic. The connected
// phone writes here (the payload is ignored — any write is a request) to ask the
// puck to sever ITS side of the link. We only ENQUEUE the writing connHandle;
// loop() issues Bluefruit.disconnect(), which then fires disconnectCallback for
// the normal graceful teardown (ANCS slot reset, user removal, leave FX).
static void onDisconnectRequest(uint16_t conn_handle, BLECharacteristic* /*chr*/,
                                uint8_t* /*data*/, uint16_t /*len*/) {
  Serial.print("disconnect request conn=0x"); Serial.println(conn_handle, HEX);
  uint8_t nextTail = (uint8_t)((g_discTail + 1) % DISC_Q_SIZE);
  if (nextTail == g_discHead) return;     // queue full — loop() will catch up
  g_discQueue[g_discTail] = conn_handle;
  g_discTail = nextTail;
}

// Find a CURRENT session member by their packed color (0x00RRGGBB), or nullptr.
// Colors are unique per session (collisions are resolved at join), so a match is
// unambiguous. This is how a pickup write is attributed to a user: the reporter
// rides its own background BLE connection, so its connHandle is NOT the session
// user's — color is the stable per-user handle the app already owns.
static User* findUserByColor(uint32_t color) {
  for (uint8_t i = 0; i < g_session.userCount; i++)
    if (g_session.connectedUsers[i].color == color)
      return &g_session.connectedUsers[i];
  return nullptr;
}

// ===== Group pickup-SPIKE detection =====
// Aggregate pickup events from ALL connected phones and fire a single group-level
// SPIKE when the group's pickup rate jumps — the signal that the conversation has
// drifted and a question should be offered. Aggregate, NOT per-user: a spike
// requires multiple DISTINCT users picking up in a short window, so one person
// fidgeting can never trigger it. The next layer (question selection) consumes
// the spike via the UUID_SPIKE characteristic; we only detect and emit here.
//
// These thresholds are deliberately simple and WILL need empirical tuning on real
// sessions — keep them here as named constants. Pickup reporting is lossy
// (background BLE gaps), so this detects a TREND and is robust to missing events.
constexpr uint32_t PICKUP_WINDOW_MS           = 20000;   // sliding aggregate window
constexpr uint8_t  PICKUP_SPIKE_THRESHOLD     = 3;       // pickups in window to fire
constexpr uint8_t  PICKUP_SPIKE_MIN_SLOTS     = 2;       // distinct users required (>1)
constexpr uint32_t PICKUP_COOLDOWN_MS         = 120000;  // base min gap between spikes
constexpr uint32_t PICKUP_COOLDOWN_BACKOFF_MS = 60000;   // added when drift persists
constexpr uint32_t PICKUP_COOLDOWN_MAX_MS     = 300000;  // cap on cooldown growth
constexpr uint32_t PICKUP_DRIFT_GRACE_MS      = 30000;   // re-fire within cooldown+this => still drifting
constexpr uint8_t  PICKUP_EVENT_CAP           = 32;      // recent-event buffer size

// Receipt-time windowing: events carry phone-side monotonic timestamps from
// DIFFERENT devices with no shared clock, so we window on the puck's own millis()
// receipt time and treat the phone timestamp as an ordering hint only.
struct PickupEvent { uint32_t recvMS; uint8_t slot; };
static PickupEvent g_pickupWin[PICKUP_EVENT_CAP];
static uint8_t  g_pickupWinCount   = 0;
static uint32_t g_lastSpikeMS      = 0;                  // 0 = none fired yet
static uint32_t g_spikeCooldownMS  = PICKUP_COOLDOWN_MS; // current (escalating) cooldown
static uint16_t g_spikeSeq         = 0;                  // increments once per fired spike

// Drop events that have aged out of the sliding window (compacting in place).
// (uint32_t) subtraction handles millis() wrap as long as the window << ~49 days.
static void prunePickupWindow(uint32_t now) {
  uint8_t w = 0;
  for (uint8_t i = 0; i < g_pickupWinCount; i++)
    if ((uint32_t)(now - g_pickupWin[i].recvMS) <= PICKUP_WINDOW_MS)
      g_pickupWin[w++] = g_pickupWin[i];
  g_pickupWinCount = w;
}

// Reset all spike state — called when a session ends so a fresh session starts
// quiet and responsive (no stale window or escalated cooldown carried over).
static void resetPickupSpikeState() {
  g_pickupWinCount  = 0;
  g_lastSpikeMS     = 0;
  g_spikeCooldownMS = PICKUP_COOLDOWN_MS;
}

// Push a fired spike to every connected phone (and keep the readable value
// current for a late reader). Payload: [0..1] seq LE, [2] count, [3] distinct.
static void notifySpike(uint8_t count, uint8_t distinct) {
  uint8_t buf[4];
  buf[0] = (uint8_t)(g_spikeSeq & 0xFF);
  buf[1] = (uint8_t)((g_spikeSeq >> 8) & 0xFF);
  buf[2] = count;
  buf[3] = distinct;
  spikeChar.write(buf, sizeof(buf));
  for (uint8_t i = 0; i < g_session.userCount; i++)
    spikeChar.notify(g_session.connectedUsers[i].connHandle, buf, sizeof(buf));
}

// Record one attributed pickup (resolved slot + puck receipt time) and decide
// whether the group just spiked. Sparse beats frequent: a fire is gated by a
// cooldown, and if the group keeps drifting (re-fires right as the cooldown lifts)
// the cooldown LENGTHENS rather than the system getting louder.
static void recordPickupAndDetectSpike(uint8_t slot, uint32_t now) {
  prunePickupWindow(now);
  if (g_pickupWinCount < PICKUP_EVENT_CAP) {
    g_pickupWin[g_pickupWinCount++] = {now, slot};
  } else {
    // Window saturated (very busy) — drop the oldest to make room.
    for (uint8_t i = 1; i < PICKUP_EVENT_CAP; i++) g_pickupWin[i - 1] = g_pickupWin[i];
    g_pickupWin[PICKUP_EVENT_CAP - 1] = {now, slot};
  }

  // Count distinct slots in the window (slot is a User.ID, 0..MAX_USERS-1).
  uint16_t slotMask = 0;
  uint8_t  distinct = 0;
  for (uint8_t i = 0; i < g_pickupWinCount; i++) {
    uint8_t s = g_pickupWin[i].slot;
    if (s < 16 && !(slotMask & (uint16_t)(1u << s))) {
      slotMask |= (uint16_t)(1u << s);
      distinct++;
    }
  }

  const bool crossed   = g_pickupWinCount >= PICKUP_SPIKE_THRESHOLD &&
                         distinct >= PICKUP_SPIKE_MIN_SLOTS;
  const bool inCooldown = g_lastSpikeMS != 0 &&
                          (uint32_t)(now - g_lastSpikeMS) < g_spikeCooldownMS;

  Serial.print("MODEM_SPIKE window count="); Serial.print(g_pickupWinCount);
  Serial.print(" distinct=");                Serial.print(distinct);
  Serial.print(" thresh=");                  Serial.print(PICKUP_SPIKE_THRESHOLD);
  Serial.print(" crossed=");                 Serial.print(crossed ? 1 : 0);
  Serial.print(" cooldown=");                Serial.print(inCooldown ? 1 : 0);
  Serial.print(" cooldownMS=");              Serial.println(g_spikeCooldownMS);

  if (!crossed) return;
  if (inCooldown) {
    // Threshold met but we recently fired — stay quiet. Continued crossings here
    // are exactly the "ignored spark" the backoff below will make quieter.
    Serial.print("MODEM_SPIKE suppressed (cooldown, ");
    Serial.print(g_spikeCooldownMS - (uint32_t)(now - g_lastSpikeMS));
    Serial.println("ms left)");
    return;
  }

  // Fire. Adjust the NEXT cooldown based on how soon this fired after the last:
  //   - re-fired within (cooldown + grace) => group kept drifting => LENGTHEN.
  //   - long calm since the last spike      => drift resolved     => RESET to base.
  if (g_lastSpikeMS != 0 &&
      (uint32_t)(now - g_lastSpikeMS) < g_spikeCooldownMS + PICKUP_DRIFT_GRACE_MS) {
    uint32_t grown = g_spikeCooldownMS + PICKUP_COOLDOWN_BACKOFF_MS;
    g_spikeCooldownMS = grown > PICKUP_COOLDOWN_MAX_MS ? PICKUP_COOLDOWN_MAX_MS : grown;
    Serial.print("MODEM_SPIKE backoff -> cooldown grows to "); Serial.println(g_spikeCooldownMS);
  } else {
    g_spikeCooldownMS = PICKUP_COOLDOWN_MS;
  }
  g_lastSpikeMS = now;
  g_spikeSeq++;
  Serial.print("MODEM_SPIKE FIRE seq="); Serial.print(g_spikeSeq);
  Serial.print(" count=");               Serial.print(g_pickupWinCount);
  Serial.print(" distinct=");            Serial.println(distinct);
  notifySpike(g_pickupWinCount, distinct);
}
// ===== END group pickup-spike detection =====

// Write-callback for the production pickup characteristic (UUID_PICKUP). Each
// phone writes one event per UNLOCK (protectedDataDidBecomeAvailable; lock is NOT
// a pickup) while it is in a session and connected, including backgrounded+locked.
// Payload (11 bytes):
//   [0]    R        uint8     — writing user's color, the per-user handle
//   [1]    G        uint8
//   [2]    B        uint8
//   [3..10] phoneMs uint64 LE — phone monotonic (systemUptime) ms of the unlock
// We attribute the raw event to its user by color, log it (MODEM_PICKUP tag) with
// the resolved slot + phone timestamp + receive-side delta, then feed it to the
// group spike detector (recordPickupAndDetectSpike) which decides whether the
// group just spiked. The phone timestamp is logged for diagnostics only; spike
// windowing uses the puck's receipt time (no shared cross-device clock).
static uint32_t g_pickupLastRecvMS = 0;     // millis() of the previous pickup write
static void onPickupWrite(uint16_t conn_handle, BLECharacteristic* /*chr*/,
                          uint8_t* data, uint16_t len) {
  const uint32_t now = millis();
  uint8_t r = len > 0 ? data[0] : 0;
  uint8_t g = len > 1 ? data[1] : 0;
  uint8_t b = len > 2 ? data[2] : 0;
  uint64_t phoneMs = 0;
  if (len >= 11) {
    for (int i = 0; i < 8; i++) phoneMs |= (uint64_t)data[3 + i] << (8 * i);
  }
  // Pack the same way the strip/roster does so the compare matches u.color.
  const uint32_t color = strip.Color(r, g, b);
  User* u = findUserByColor(color);
  const int slot = u ? (int)u->ID : -1;     // -1 = no current member with this color
  const int32_t deltaMS = g_pickupLastRecvMS == 0 ? -1 : (int32_t)(now - g_pickupLastRecvMS);
  Serial.print("MODEM_PICKUP recvMS="); Serial.print(now);
  Serial.print(" conn=0x");             Serial.print(conn_handle, HEX);
  Serial.print(" slot=");               Serial.print(slot);
  Serial.print(" color=0x");            Serial.print(color, HEX);
  Serial.print(" name=");               Serial.print(u ? u->name : "?");
  Serial.print(" phoneMs=");            Serial.print((unsigned long)phoneMs);
  Serial.print(" deltaMS=");            Serial.print(deltaMS);
  Serial.print(" len=");                Serial.println(len);
  g_pickupLastRecvMS = now;

  // Count this pickup against its user and push the refreshed roster so every
  // phone's pickups leaderboard updates live. Only attributed events (resolved to
  // a current member) count — an unattributable color isn't a session user.
  if (u) {
    if (u->pickups < 0xFFFF) u->pickups++;
    notifyRoster();
  }

  // Feed the group spike detector. Only attributed events (resolved to a current
  // member) count toward the aggregate — an unattributable color can't be a
  // distinct user. Windowing uses the puck's receipt time, not phoneMs.
  if (slot >= 0) recordPickupAndDetectSpike((uint8_t)slot, now);
}

void setup() {
  Serial.begin(115200);

  // Persistent settings: mount flash + load the record (or seed DEFAULTS on a
  // blank/old board) BEFORE the lights so the strip comes up at the saved
  // brightness and the sound flag is in effect from the first interaction.
  puckSettingsBegin();

  // Speaker: init the I2S chime engine after settings (so the sound gate is
  // readable). Safe even if init fails — chimes then no-op.
  modemAudioBegin();

  // Lights: init first, then sit at the idle (cleared) color.
  strip.begin();
  strip.setBrightness(puckSettings().brightness);
  strip.clear();
  strip.show();

  // BLE: Bluefruit peripheral + GATT server.
  // Give the GATT attribute table extra headroom so the full characteristic set
  // (incl. the brightness + claim additions and the large roster/allowlist value
  // buffers) always registers — an overflow here silently drops characteristics.
  // Must be configured BEFORE Bluefruit.begin().
  Bluefruit.configAttrTableSize(0x1000);
  // ATT MTU stays at the core default (23). The SoftDevice RAM reservation (24 KB,
  // app RAM ORIGIN 0x20006000 in nrf52840_s140_v6.ld) is nearly full at 8 links +
  // this attr table, so ANY MTU bump (tried 247, then 128) overflows it,
  // sd_ble_enable() returns NO_MEM, begin() fails, and the puck goes unconnectable.
  // Instead of a bigger MTU, the roster is sent ONE USER PER NOTIFICATION (each
  // packet fits the 20-byte payload of a 23-byte MTU) and reassembled on the
  // phone — see notifyRoster / pushRosterPaged. No configPrphConn here on purpose.
  // Allow up to MAX_USERS concurrent peripheral connections. NOTE: this is the
  // PHYSICAL link ceiling; on iOS a single phone can hold a second (background
  // pickup / phantom ANCS) link, so this 8 can be consumed by fewer than 8
  // session members. Raising it to give each phone two links is deferred: it
  // also requires resizing the ANCS slot pool (ancs_client g_slots[MAX_USERS],
  // allocated per-connection) and re-validating SoftDevice RAM on hardware. The
  // links-vs-users heartbeat below is the instrument to confirm whether that
  // ceiling is actually being hit in the field before taking that change.
  // Check the return: begin() fails (returns false) if the SoftDevice config
  // (notably the MTU above) needs more RAM than the linker reserved. Make that
  // LOUD — a silent failure here leaves BLE down and the puck unconnectable, which
  // is hard to tell apart from a hardware fault. If this ever fires, lower the MTU
  // in configPrphConn above.
  if (!Bluefruit.begin(MAX_USERS)) {
    Serial.println("MODEM_FATAL Bluefruit.begin() FAILED — SoftDevice RAM/config "
                   "overflow. BLE is DOWN. Lower the configPrphConn MTU.");
  }
  Bluefruit.setTxPower(4);
  Bluefruit.setName(DEVICE_NAME);
  Bluefruit.Periph.setConnectCallback(connectCallback);
  Bluefruit.Periph.setDisconnectCallback(disconnectCallback);
  // Parent service must begin() before the characteristics it owns.
  modemSvc.begin();

  // Owner: claim/takeover target. READ returns a 0/1 owned-flag (the raw key is
  // never read back); WRITE takes the phone's 16-byte claim key (see onOwnerWrite).
  ownerChar.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  ownerChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  ownerChar.setMaxLen(PUCK_OWNER_KEY_LEN);
  ownerChar.setWriteCallback(onOwnerWrite);
  ownerChar.begin();
  ownerChar.write8(puckSettings().claimed);   // owned-flag

  // Name: device name. READ + NOTIFY (pushed on connect) + WRITE to rename. The
  // value is sourced from the persisted settings (see onNameWrite).
  nameChar.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE | CHR_PROPS_NOTIFY);
  nameChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  nameChar.setMaxLen(MAX_DEVICE_NAME_LEN);
  nameChar.setWriteCallback(onNameWrite);
  nameChar.begin();
  nameChar.write(puckSettings().name);

  // Sound: enable/disable interaction sound. READ + WRITE, persisted.
  soundChar.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  soundChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  soundChar.setMaxLen(1);
  soundChar.setWriteCallback(onSoundWrite);
  soundChar.begin();
  soundChar.write8(puckSettings().soundEnabled);

  // Brightness: light brightness (uint8, 0..LIGHT_BRIGHTNESS_MAX). READ + WRITE,
  // persisted, applied to the strip on write (see onBrightnessWrite).
  brightChar.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  brightChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  brightChar.setMaxLen(1);
  brightChar.setWriteCallback(onBrightnessWrite);
  brightChar.begin();
  brightChar.write8(puckSettings().brightness);

  colorChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  colorChar.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  colorChar.setMaxLen(32);
  colorChar.setWriteCallback(onColorWrite);
  colorChar.begin();

  // READ + NOTIFY: live count of connected phones (0..MAX_USERS). The app
  // reads it once and subscribes for join/leave updates.
  countChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  countChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  countChar.setMaxLen(1);
  // Push the current count the instant a phone subscribes (join-time race fix).
  countChar.setCccdWriteCallback(onCountSubscribe);
  countChar.begin();
  countChar.write8(g_session.userCount);

  // WRITE: per-phone chosen display name (UTF-8). Stored on the writing user
  // and echoed back to everyone in the roster broadcast.
  userNameChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  userNameChar.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  userNameChar.setMaxLen(MAX_USER_NAME_LEN);
  userNameChar.setWriteCallback(onUserNameWrite);
  userNameChar.begin();

  // WRITE: per-phone screen-time telemetry (uint16 seconds, little-endian).
  stimeChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  stimeChar.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  stimeChar.setMaxLen(2);
  stimeChar.setWriteCallback(onScreenTimeWrite);
  stimeChar.begin();

  // READ + NOTIFY: canonical session clock (uint16 seconds). Pushed once a
  // minute (and on every join) so all phones share one synced session time.
  sessChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  sessChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  sessChar.setMaxLen(2);
  // Push the current session clock the instant a phone subscribes.
  sessChar.setCccdWriteCallback(onSessionSubscribe);
  sessChar.begin();
  sessChar.write16(0);

  // READ + NOTIFY: full connected-user roster (see notifyRoster for layout).
  // Pushed whenever the table changes so every phone shows the same leaderboard.
  rosterChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  rosterChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  rosterChar.setMaxLen(1 + MAX_USERS * (3 + 2 + 1 + MAX_USER_NAME_LEN));
  // Push the full roster the instant a phone subscribes (fixes the empty
  // leaderboard: the connect-time read races the join and is served the empty
  // table, and the join-time notify lands before the phone has subscribed).
  rosterChar.setCccdWriteCallback(onRosterSubscribe);
  rosterChar.begin();
  { const uint8_t empty = 0; rosterChar.write(&empty, 1); }

  // WRITE: per-connection notification allowlist (Phase B). The phone writes its
  // bundle-ID list; onAllowlistWrite routes it to the writer's ANCS slot.
  allowChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  allowChar.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  allowChar.setMaxLen(ALLOW_PAYLOAD_MAX);
  allowChar.setWriteCallback(onAllowlistWrite);
  allowChar.begin();

  // WRITE: phone-requested disconnect. The app writes here (any value) to ask
  // the puck to drop ITS link; onDisconnectRequest enqueues the writer's conn
  // handle and loop() initiates the GAP disconnect so iOS actually severs the
  // bonded ANCS connection (a phone-side cancel alone leaves the link up).
  disconnChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  disconnChar.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  disconnChar.setMaxLen(1);
  disconnChar.setWriteCallback(onDisconnectRequest);
  disconnChar.begin();

  // WRITE-only: production per-phone pickup (unlock) events. onPickupWrite
  // attributes each event to a user by color and logs it for puck-side
  // aggregation; the puck never reads it back.
  pickupChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  pickupChar.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  pickupChar.setMaxLen(16);
  pickupChar.setWriteCallback(onPickupWrite);
  pickupChar.begin();

  // READ + NOTIFY: group pickup-spike trigger. The puck notifies here when the
  // aggregate pickup rate spikes; phones observe it to surface a question.
  spikeChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  spikeChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  spikeChar.setMaxLen(4);
  spikeChar.begin();
  { uint8_t z[4] = {0, 0, 0, 0}; spikeChar.write(z, sizeof(z)); }  // seed readable value

  // Charging flag: READ + NOTIFY uint8 (0/1). Seeded 0; sampleBattery keeps it
  // current. Lets the app show a charging icon while the percent is suppressed.
  chargeChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  chargeChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  chargeChar.setMaxLen(1);
  chargeChar.begin();
  chargeChar.write8(0);

  // Standard Battery Service (0x180F): exposes the puck's 0..100% level for the
  // app's home-screen indicator. Begun after Bluefruit.begin(); seed it with an
  // initial reading so a phone that connects/reads early gets a real value.
  blebas.begin();
  sampleBattery();
  g_lastBatteryMS = millis();

  // Per-slot ANCS consumer: registers the per-connection ANCS client services
  // and the secured-link callback. Must come after Bluefruit.begin().
  ancsBegin();

  // Advertise the service; the stack re-advertises itself after a drop.
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(modemSvc);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);  // 20 ms .. 152.5 ms
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);              // 0 = advertise forever

  // Start Idle with an empty session; the first connection creates one.
  g_session  = Session{};
  CURR_STATE = State::Idle;
}

void loop() {
  const uint32_t now = millis();

  // Pump the per-slot ANCS clients: issue any pending Control Point attribute
  // fetches (non-blocking). Callbacks only parse/enqueue; the work happens here.
  ancsService();

  // Honor any phone-requested disconnects: the peripheral must initiate the GAP
  // disconnect so iOS actually drops the link (a phone-side cancel alone leaves
  // it up). disconnectCallback then runs the normal teardown (ANCS slot reset +,
  // if the conn was a joined session member, user removal + leave FX).
  //
  // The BOND IS KEPT intact. We must NOT delete the puck's bond: iOS has no API
  // to forget its half, so a puck-only bond wipe leaves an asymmetric bond that
  // blocks every future reconnect until the user manually "Forget"s the device.
  // The "phantom ANCS reconnect" is instead made harmless two ways: (B) we drop
  // the ANCS subscription here so iOS has less reason to reopen the link, and (A)
  // a bare BLE connection is no longer treated as a session join (see joinUser),
  // so even if iOS does reopen the link it never appears as a user.
  while (g_discHead != g_discTail) {
    uint16_t h = g_discQueue[g_discHead];
    g_discHead = (uint8_t)((g_discHead + 1) % DISC_Q_SIZE);
    markIntentionalLeave(h);                 // app asked: finalize on its disconnect (no grace)
    ancsUnsubscribe(h);                      // (B) stop consuming ANCS, keep bond
    Bluefruit.disconnect(h);
    Serial.print("disconnect issued conn=0x"); Serial.println(h, HEX);
  }

  // Roster: send the per-user packets here (task context) when a change raised the
  // dirty flag. Done in loop() — not the callbacks that set it — because notifying
  // one connection repeatedly blocks on its HVN TX semaphore until each transmits.
  if (g_rosterDirty) {
    g_rosterDirty = false;
    pushRosterPaged();
  }

  // The puck owns the session clock: push it to every phone once a minute so
  // all devices stay in sync. (A baseline is also pushed on each join.)
  if (CURR_STATE == State::Active &&
      (int32_t)(now - g_lastSessMS) >= (int32_t)SESSION_NOTIFY_MS) {
    g_lastSessMS = now;
    notifySessionTime();
  }

  // Leave-grace sweep: finalize any spontaneously-dropped member whose grace
  // window expired with no reconnect. One per pass — finalizeLeave repacks the
  // user array, so the rest are re-checked on the next loop.
  if (CURR_STATE == State::Active) {
    for (uint8_t i = 0; i < g_session.userCount; i++) {
      const User& u = g_session.connectedUsers[i];
      if (u.pendingLeaveMS != 0 &&
          (int32_t)(now - u.pendingLeaveMS) >= (int32_t)LEAVE_GRACE_MS) {
        Serial.print("leave grace expired conn=0x");
        Serial.println(u.connHandle, HEX);
        finalizeLeave(u.connHandle);
        break;
      }
    }
  }

  // Battery: re-measure periodically, or immediately when a connect requested it
  // (so a freshly-connected app reads a current charge state), and push to subs.
  if (g_forceBattery ||
      (int32_t)(now - g_lastBatteryMS) >= (int32_t)BATTERY_SAMPLE_MS) {
    g_forceBattery = false;
    g_lastBatteryMS = now;
    sampleBattery();
  }

  // Diagnostic heartbeat: dump full session + ANCS slot state every few seconds.
  static uint32_t s_lastHeartbeatMS = 0;
  if ((int32_t)(now - s_lastHeartbeatMS) >= (int32_t)HEARTBEAT_MS) {
    s_lastHeartbeatMS = now;
    dumpState();
  }

  // A color write starts the connect FX: spin -> hold lit -> gauge -> off.
  if (g_connectFx) {
    g_connectFx = false;
    g_fxPhase  = FxPhase::Spin;
    g_fxStep   = 0;
    g_fxNextMS = now;                        // first spin frame now
  }

  // A disconnect starts the leave FX: fade in fast -> flicker out -> gauge.
  if (g_disconnectFx) {
    g_disconnectFx = false;
    g_fxPhase   = FxPhase::FadeIn;
    g_fxStartMS = now;
    g_fxStep    = 0;
    g_fxNextMS  = now;
  }

  // An allowlist-matched notification pulses the puck in that user's color. We
  // only start it when no other FX is running so a join/leave animation isn't
  // interrupted (a notification mid-FX is simply dropped — they arrive often).
  if (g_notifyFx) {
    g_notifyFx = false;
    if (g_fxPhase == FxPhase::Idle) {
      g_fxColor   = g_notifyColor;
      g_fxPhase   = FxPhase::Pulse;
      g_fxStartMS = now;
      g_fxStep    = 0;                        // ramp index: even=up, odd=down
      g_fxNextMS  = now;
    }
  }

  // A claim/takeover confirms with a brief green pulse (reusing the notify Pulse
  // ramp, forced green) plus the claim chime below. Claim is a deliberate, rare
  // onboarding action, so it takes over any running FX rather than being dropped.
  if (g_claimFx) {
    g_claimFx   = false;
    g_fxColor   = strip.Color(0, 150, 0);    // green
    g_fxPhase   = FxPhase::Pulse;
    g_fxStartMS = now;
    g_fxStep    = 0;                          // ramp index: even=up, odd=down
    g_fxNextMS  = now;
    chimeClaim();
  }

  // Audio chimes: play the requested chime for each session event. Done here
  // (not in the BLE callbacks that raise the flags) so I2S/DMA is never started
  // from GATT-callback context, matching the deferred-FX rule above. The sound
  // gate is enforced inside the wrappers (playChime). Distinct events are
  // mutually exclusive per pass except notify, which can ride alongside a join.
  if (g_chimeStart)  { g_chimeStart  = false; chimeStart();  }
  if (g_chimeJoin)   { g_chimeJoin   = false; chimeJoin();   }
  if (g_chimeLeave)  { g_chimeLeave  = false; chimeLeave();  }
  if (g_chimeNotify) { g_chimeNotify = false; chimeNotify(); }

  // Advance whichever FX is running once its frame/phase timer elapses.
  if (g_fxPhase != FxPhase::Idle && (int32_t)(now - g_fxNextMS) >= 0) {
    switch (g_fxPhase) {
      case FxPhase::Spin:
        showSpin(g_fxColor, g_fxStep);
        if (g_fxStep + 1 >= NUM_LIGHTS) {    // full lap -> hold the board lit
          g_fxPhase  = FxPhase::Hold;
          g_fxNextMS = now + HOLD_MS;
        } else {
          g_fxStep++;
          g_fxNextMS = now + SPIN_STEP_MS;
        }
        break;

      case FxPhase::Hold:                    // held lit -> show capacity
        showUserGauge();
        g_fxPhase  = FxPhase::Gauge;
        g_fxNextMS = now + GAUGE_MS;
        break;

      case FxPhase::FadeIn: {
        uint32_t elapsed = now - g_fxStartMS;
        if (elapsed >= FADEIN_MS) {          // fully in -> start flickering
          showColor(g_fxColor);
          g_fxPhase  = FxPhase::Flicker;
          g_fxStep   = 0;
          g_fxNextMS = now + FLICKER_MS;
        } else {
          uint8_t scale = (uint8_t)(255UL * elapsed / FADEIN_MS);
          showColorScaled(g_fxColor, scale);
          g_fxNextMS = now + FX_FRAME_MS;
        }
        break;
      }

      case FxPhase::Flicker:
        if (g_fxStep >= FLICKER_STEPS) {     // ended off -> show capacity
          showUserGauge();
          g_fxPhase  = FxPhase::Gauge;
          g_fxNextMS = now + GAUGE_MS;
        } else {
          if (g_fxStep & 1) { strip.clear(); strip.show(); }  // off
          else              showColor(g_fxColor);             // on
          g_fxStep++;
          g_fxNextMS = now + FLICKER_MS;
        }
        break;

      case FxPhase::Gauge:                   // capacity held -> all off
        strip.clear();
        strip.show();
        g_fxPhase = FxPhase::Idle;
        break;

      case FxPhase::Pulse: {                 // notification: ramp up/down N times
        const uint32_t elapsed = now - g_fxStartMS;
        if (elapsed >= NOTIFY_HALF_MS) {     // this ramp done -> next half-cycle
          g_fxStep++;
          g_fxStartMS = now;
          if (g_fxStep >= (uint16_t)NOTIFY_PULSES * 2) {  // all pulses done -> off
            strip.clear();
            strip.show();
            g_fxPhase = FxPhase::Idle;
          } else {
            g_fxNextMS = now;                // start the next ramp immediately
          }
        } else {                             // within a ramp: even=up, odd=down
          const uint32_t frac = 255UL * elapsed / NOTIFY_HALF_MS;
          const uint8_t  scale = (g_fxStep & 1) ? (uint8_t)(255 - frac)
                                                : (uint8_t)frac;
          showColorScaled(g_fxColor, scale);
          g_fxNextMS = now + FX_FRAME_MS;
        }
        break;
      }

      case FxPhase::Idle:
        break;
    }
  }
}
