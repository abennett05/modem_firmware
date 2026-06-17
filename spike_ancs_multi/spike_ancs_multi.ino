/// spike_ancs_multi.ino  —  MODEM Phase 0 de-risk spike
/// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
/// PURPOSE (and ONLY purpose): prove the load-bearing assumption before any of
/// Phase A/B/C is built — that this nRF52840 (Bluefruit) puck can hold MULTIPLE
/// SIMULTANEOUS BONDED ANCS links and attribute notifications to the correct
/// connection with NO cross-talk.
///
/// This is a throwaway test sketch. It is deliberately separate from
/// modem_firmware.ino and shares no state with it. It does NOT touch the MODEM
/// GATT service, NFC, session logic, or LEDs. Flash it standalone, bond 2–3
/// iPhones, watch the serial log, then report the numbers requested at the
/// bottom of this file.
///
/// WHAT IT DEMONSTRATES
///   1. Bluefruit.begin(NUM_SLOTS, 0) — NUM_SLOTS *peripheral* links, 0 central.
///      iPhones connect to us as centrals; we are the GATT *client* for ANCS on
///      each peripheral link. (Library fact: Bluefruit configures BLE_CONN_CFG_GATTC
///      on the *peripheral* connection config, so GATT-client works over peripheral
///      links — no central role required.)
///   2. One BLEAncs instance PER SLOT (g_ancs[NUM_SLOTS]). Each BLEClientService
///      tracks exactly one conn_handle (BLEClientService::_conn_hdl), so one
///      instance == one phone. The stock single-global BLEAncs CANNOT do this.
///   3. Per-slot Added-notification logging tagged with connHandle + slot, plus a
///      per-slot AppIdentifier (bundle id) fetch — proving the Control Point write
///      and Data Source reassembly work independently per connection.
///
/// HARDWARE: Adafruit Feather nRF52840 Express (adafruit:nrf52:feather52840),
/// Bluefruit core. Build:
///   arduino-cli compile --fqbn adafruit:nrf52:feather52840 spike_ancs_multi
///   arduino-cli upload  --fqbn adafruit:nrf52:feather52840 -p <port> spike_ancs_multi
///
/// LIMITS THAT MAKE 8 SLOTS FIT (verified against core 1.7.0 / S140 6.1.1):
///   - CFG_GATT_MAX_CLIENT_CHARS = 40  -> 8 slots * 3 client chars = 24  (OK)
///   - CFG_GATT_MAX_CLIENT_SERVICE = 20 -> 8 client services           (OK)
///   - BLE_MAX_CONNECTION = 20 (SoftDevice); we ask for 8 peripheral links.
///   - Keep DEFAULT bandwidth (MTU 23). Do NOT call configPrphBandwidth(MAX):
///     the stock ANCS example does that for ONE link; at 8 links the SoftDevice
///     RAM request will overrun the app region and sd_ble_enable() will fail.

#include <bluefruit.h>

// Test up to the production cap of 8. Bonding 2–3 phones already answers the
// cross-talk question; the array is sized for the full target so RAM/registry
// pressure is exercised honestly.
#define NUM_SLOTS 8

#define LOG  Serial.print
#define LOGLN Serial.println
static const char* PFX = "MODEM_ANCS";

// ---- Per-slot state -------------------------------------------------------
// connHandle is the ONLY key. Do NOT assume slot index == connHandle; handles
// are opaque and reused by the stack. This mirrors the production slot model.
struct Slot {
  bool     inUse;
  uint16_t connHandle;
};
static Slot    g_slot[NUM_SLOTS];
static BLEAncs g_ancs[NUM_SLOTS];   // one independent ANCS client per slot

const char* EVENT_STR[] = { "Added", "Modified", "Removed" };

// ---- Slot helpers (connHandle <-> slot index) -----------------------------
static int slotForHandle(uint16_t h) {
  for (int i = 0; i < NUM_SLOTS; i++)
    if (g_slot[i].inUse && g_slot[i].connHandle == h) return i;
  return -1;
}
static int allocSlot(uint16_t h) {
  for (int i = 0; i < NUM_SLOTS; i++)
    if (!g_slot[i].inUse) { g_slot[i] = { true, h }; return i; }
  return -1;
}

// ---- Shared notification handler ------------------------------------------
// Called from per-slot trampolines below. We already know the slot, so we can
// prove attribution end-to-end: log the connHandle the instance is bound to,
// the Added UID, then fetch the AppIdentifier on THIS slot's Control Point and
// reassemble THIS slot's Data Source reply.
static void onAncsNotification(int slot, AncsNotification_t* n) {
  uint16_t bound = g_ancs[slot].connHandle();   // conn this instance is on
  LOG(PFX); LOG(" slot="); LOG(slot);
  LOG(" conn=0x"); Serial.print(bound, HEX);
  LOG(" evt="); LOG(EVENT_STR[n->eventID]);
  LOG(" cat="); LOG(n->categoryID);
  LOG(" uid="); Serial.println(n->uid);

  if (n->eventID != ANCS_EVT_NOTIFICATION_ADDED) return;  // light trigger = Added only

  // Per-slot bundle-id fetch (Control Point write + Data Source reassembly).
  // Stock getAppID blocks on this slot's _adamsg until its Data Source reply
  // completes; that is fine here (runs in the Bluefruit event task, not an ISR).
  char appid[64] = { 0 };
  uint16_t len = g_ancs[slot].getAppID(n->uid, appid, sizeof(appid));
  LOG(PFX); LOG(" slot="); LOG(slot); LOG(" appid=\"");
  LOG(len ? appid : "(empty)"); LOGLN("\"");
}

// ---- 8 thin trampolines ---------------------------------------------------
// The stock BLEAncs notification callback signature carries no conn handle, so
// each instance gets its own trampoline that knows its slot. (Production will
// instead read chr->connHandle() in a custom notify cb — see ancs_client.)
#define TRAMP(i) static void ancsCb##i(AncsNotification_t* n){ onAncsNotification(i, n); }
TRAMP(0) TRAMP(1) TRAMP(2) TRAMP(3) TRAMP(4) TRAMP(5) TRAMP(6) TRAMP(7)
static BLEAncs::notification_callback_t TRAMPS[NUM_SLOTS] = {
  ancsCb0, ancsCb1, ancsCb2, ancsCb3, ancsCb4, ancsCb5, ancsCb6, ancsCb7
};

// ---- BLE callbacks --------------------------------------------------------
static void connectCb(uint16_t conn_handle) {
  int slot = allocSlot(conn_handle);
  LOG(PFX); LOG(" CONNECT conn=0x"); Serial.print(conn_handle, HEX);
  LOG(" -> slot="); LOGLN(slot);
  if (slot < 0) { LOG(PFX); LOGLN(" no free slot, dropping");
    Bluefruit.disconnect(conn_handle); return; }

  // ANCS needs a bonded, encrypted link. Ask the phone to pair; iOS shows the
  // PAIR prompt. Discovery + notify enable happen once the link is secured.
  Bluefruit.Connection(conn_handle)->requestPairing();

  // Keep advertising while slots remain so more phones can join concurrently.
  if (Bluefruit.connected() < NUM_SLOTS) Bluefruit.Advertising.start(0);
}

static void securedCb(uint16_t conn_handle) {
  BLEConnection* c = Bluefruit.Connection(conn_handle);
  if (!c->secured()) { c->requestPairing(); return; }

  int slot = slotForHandle(conn_handle);
  if (slot < 0) return;

  LOG(PFX); LOG(" SECURED slot="); LOG(slot);
  LOG(" conn=0x"); Serial.println(conn_handle, HEX);

  if (g_ancs[slot].discover(conn_handle)) {
    LOG(PFX); LOG(" ANCS discovered slot="); LOGLN(slot);
    g_ancs[slot].setNotificationCallback(TRAMPS[slot]);
    g_ancs[slot].enableNotification();   // CCCD on Notification Source + Data Source
    LOG(PFX); LOG(" notifications enabled slot="); LOGLN(slot);
  } else {
    LOG(PFX); LOG(" ANCS NOT found slot="); LOGLN(slot);
  }
}

static void disconnectCb(uint16_t conn_handle, uint8_t reason) {
  int slot = slotForHandle(conn_handle);
  LOG(PFX); LOG(" DISCONNECT conn=0x"); Serial.print(conn_handle, HEX);
  LOG(" slot="); LOG(slot); LOG(" reason=0x"); Serial.println(reason, HEX);
  if (slot >= 0) g_slot[slot] = { false, 0 };
  // The Bluefruit GATT layer auto-tears-down client chars/services whose
  // connHandle matches the disconnected link, so g_ancs[slot] resets itself.
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}   // brief wait for USB serial

  LOG(PFX); LOGLN(" Phase 0 spike: multi simultaneous bonded ANCS");

  // Persisted bonds make reconnects silent; clearing here gives a clean test.
  // Comment out to test bonded-reconnect resumption instead.
  Bluefruit.begin(NUM_SLOTS, 0);   // NUM_SLOTS peripheral links, 0 central
  Bluefruit.setTxPower(4);
  Bluefruit.setName("Modem Spike");

  Bluefruit.Periph.setConnectCallback(connectCb);
  Bluefruit.Periph.setDisconnectCallback(disconnectCb);
  Bluefruit.Security.setSecuredCallback(securedCb);

  for (int i = 0; i < NUM_SLOTS; i++) {
    g_slot[i] = { false, 0 };
    g_ancs[i].begin();
  }

  // Advertise the ANCS solicitation so iOS offers to pair/expose ANCS.
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(g_ancs[0]);   // advertise ANCS solicited UUID
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);

  LOG(PFX); LOG(" advertising; NUM_SLOTS="); LOGLN(NUM_SLOTS);
}

static uint32_t g_lastReport = 0;
void loop() {
  // Once a second, report how many links are live so headroom is visible.
  uint32_t now = millis();
  if (now - g_lastReport >= 5000) {
    g_lastReport = now;
    uint8_t n = Bluefruit.connected();
    if (n) { LOG(PFX); LOG(" live links="); LOGLN(n); }
  }
}

/// ============================================================================
/// REPORT THESE (paste serial output):
///   - Max simultaneous bonded ANCS links you achieved (aim 2–3, array holds 8).
///   - For each phone: "SECURED" + "ANCS discovered" + "notifications enabled".
///   - That Added notifications from phone A log slot/conn for A only, and from
///     phone B log slot/conn for B only (no cross-talk).
///   - Any sd_ble_enable / RAM error at boot (there should be NONE at default
///     bandwidth; begin(8) already runs in production firmware).
///   - Anything that fails to discover, pair, or notify.
/// STOP here and report before Phase A/B/C.
/// ============================================================================
