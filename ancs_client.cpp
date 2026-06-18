/// ancs_client.cpp
/// - - - - - - - - - - - -
/// Per-slot ANCS consumer. See ancs_client.h for the slot model (connHandle is
/// the key; this module owns its own stable slot space).
///
/// THREADING (Bluefruit task model — important for the "no blocking in
/// callbacks" rule):
///   - ancsOnConnect / secured callback / discovery / CCCD enable run in the
///     Bluefruit *callback* task (deferred). Blocking SoftDevice calls there are
///     safe and are the library norm; we only block during one-time setup.
///   - The Notification Source and Data Source notify callbacks are installed
///     with useAdaCallback=false, so they run in the BLE *event* task. They must
///     NOT block: they only parse + enqueue / reassemble.
///   - ancsService() runs in the main loop() task and issues Control Point
///     writes. Single producer/consumer split keeps the pending ring and the
///     in-flight flag race-free without locks.

#include <Arduino.h>
#include <bluefruit.h>
#include <string.h>
#include "ancs_client.h"

// ---- Logging --------------------------------------------------------------
#define ANCS_LOG_PFX "MODEM_ANCS"
#define LOGP(...)  do { Serial.print("" ANCS_LOG_PFX " "); Serial.print(__VA_ARGS__); } while (0)

// ---- ANCS UUIDs -----------------------------------------------------------
// Reuse the Bluefruit core's verified little-endian UUID byte arrays (defined
// in clients/BLEAncs.cpp, external linkage via BLEAncs.h) so there is no risk of
// a hand-transcribed byte-order bug. Standard (big-endian) forms:
//   Service             7905F431-B5CE-4E99-A40F-4B1E122D00D0
//   Notification Source 9FBF120D-6301-42D9-8C58-25E699A21DBD (notify)
//   Control Point       69D1D8F3-45E1-49A8-9821-9BBDFDAAD9D9 (write w/ response)
//   Data Source         22EAC6E9-24D6-4BB5-BE44-B36ACE7C7BFB (notify)
extern const uint8_t BLEANCS_UUID_SERVICE[];
extern const uint8_t BLEANCS_UUID_CHR_CONTROL[];
extern const uint8_t BLEANCS_UUID_CHR_NOTIFICATION[];
extern const uint8_t BLEANCS_UUID_CHR_DATA[];

// ANCS enum values come from the core's BLEAncs.h (pulled in by bluefruit.h):
//   ANCS_EVT_NOTIFICATION_ADDED     = 0  (Notification Source EventID)
//   ANCS_CMD_GET_NOTIFICATION_ATTR  = 0  (Control Point CommandID)
//   ANCS_ATTR_APP_IDENTIFIER        = 0  (AttributeID; variable len, NO len field in request)
// We reference them directly rather than redefining (which would conflict).

// ---- Per-slot buffers (fixed; no dynamic allocation) ----------------------
constexpr uint8_t  ANCS_PEND_SIZE = 8;    // pending Added-UID ring (power of two)
constexpr uint8_t  ANCS_PEND_MASK = ANCS_PEND_SIZE - 1;
constexpr uint16_t ANCS_DS_BUF    = 96;   // Data Source reassembly buffer / slot
constexpr uint16_t ANCS_APPID_MAX = 64;   // extracted bundle-id cap (for log/match)
// Max time a Control Point fetch may stay in flight before we give up on its
// reply and free the slot. The Data Source reply normally follows the write by a
// few connection intervals; this only fires when a reply is genuinely lost or the
// write was rejected (e.g. link congested during the pairing-time flood). Without
// it, one lost reply wedges the slot forever (inFlight never clears -> the pending
// ring fills -> every later notification is dropped).
constexpr uint32_t ANCS_FETCH_TIMEOUT_MS = 3000;

/// One independent ANCS client + its per-slot state. One instance == one phone.
class AncsSlot {
 public:
  AncsSlot()
      : svc(BLEANCS_UUID_SERVICE),
        ns(BLEANCS_UUID_CHR_NOTIFICATION),
        cp(BLEANCS_UUID_CHR_CONTROL),
        ds(BLEANCS_UUID_CHR_DATA) {}

  // Register the client service + characteristics once (at ancsBegin). The
  // parent service MUST be passed explicitly: the no-arg begin() binds to a
  // global "last service", which is wrong when 8 instances exist.
  void begin() {
    svc.begin();
    cp.begin(&svc);
    ns.begin(&svc);
    ds.begin(&svc);
    reset();
  }

  // Clear all runtime state for this slot (connect alloc + disconnect teardown).
  // Does not unregister the BLE objects (begin() is once-only).
  void reset() {
    active      = false;
    chrsFound   = false;
    joined      = false;
    ready       = false;
    connHandle  = BLE_CONN_HANDLE_INVALID;
    userHint    = 0xFF;
    allowCount  = 0;
    pendHead    = 0;
    pendTail    = 0;
    inFlight        = false;
    inFlightUid     = 0;
    inFlightSinceMS = 0;
    rxLen           = 0;
  }

  // BLE client objects (one ANCS service + its 3 characteristics).
  BLEClientService        svc;
  BLEClientCharacteristic ns;   // Notification Source (notify)
  BLEClientCharacteristic cp;   // Control Point       (write w/ response)
  BLEClientCharacteristic ds;   // Data Source         (notify)

  // Identity / lifecycle.
  volatile bool active;         // slot allocated to a connection
  volatile bool chrsFound;      // ANCS service + characteristics discovered
  volatile bool joined;         // owning connection joined the session (gates subscribe)
  volatile bool ready;          // ANCS notifications enabled (CCCD written) — pump gate
  uint16_t      connHandle;     // the durable key for this slot
  uint8_t       userHint;       // orchestrator array index at connect (log only)

  // Per-slot allowlist (Phase B). Strictly per-slot — never shared/merged.
  char    allow[ANCS_ALLOW_MAX][ANCS_ALLOW_ENTRY_LEN];
  uint8_t allowCount;

  // Pending Added-UID ring. Producer: NS notify cb (BLE event task) -> pendTail.
  // Consumer: ancsService (loop task) -> pendHead. SPSC, lock-free.
  uint32_t         pend[ANCS_PEND_SIZE];
  volatile uint8_t pendHead;
  volatile uint8_t pendTail;

  // One in-flight attribute fetch per slot. Set true by ancsService when the
  // Control Point write is issued; set false by the Data Source cb on
  // completion/abort. Ordering is causal (response always follows the write),
  // so no concurrent write to this flag occurs in practice.
  volatile bool inFlight;
  uint32_t      inFlightUid;
  uint32_t      inFlightSinceMS;   // millis() when the fetch was issued (timeout)

  // Per-slot Data Source reassembly buffer (ANCS replies span ~20-byte packets
  // at the default MTU). Buffers are NEVER shared across slots.
  uint8_t  rx[ANCS_DS_BUF];
  uint16_t rxLen;
};

static AncsSlot g_slots[MAX_USERS];

// ---- Slot lookup ----------------------------------------------------------
static AncsSlot* slotByHandle(uint16_t h) {
  if (h == BLE_CONN_HANDLE_INVALID) return nullptr;
  for (uint8_t i = 0; i < MAX_USERS; i++)
    if (g_slots[i].active && g_slots[i].connHandle == h) return &g_slots[i];
  return nullptr;
}

static int indexOf(const AncsSlot* s) {
  for (uint8_t i = 0; i < MAX_USERS; i++)
    if (&g_slots[i] == s) return i;
  return -1;
}

int ancsSlotForHandle(uint16_t connHandle) {
  AncsSlot* s = slotByHandle(connHandle);
  return s ? indexOf(s) : -1;
}

uint16_t ancsConnHandle(uint8_t slotIndex) {
  if (slotIndex >= MAX_USERS || !g_slots[slotIndex].active)
    return BLE_CONN_HANDLE_INVALID;
  return g_slots[slotIndex].connHandle;
}

// ---- Allowlist ------------------------------------------------------------
void ancsSetAllowlist(uint8_t slotIndex, const uint8_t* payload, uint16_t len) {
  if (slotIndex >= MAX_USERS) return;
  AncsSlot& s = g_slots[slotIndex];

  s.allowCount = 0;
  uint16_t i = 0;
  while (i < len && s.allowCount < ANCS_ALLOW_MAX) {
    // Skip separators ('\n', '\r', NUL).
    while (i < len && (payload[i] == '\n' || payload[i] == '\r' || payload[i] == '\0')) i++;
    if (i >= len) break;

    uint16_t start = i;
    while (i < len && payload[i] != '\n' && payload[i] != '\r' && payload[i] != '\0') i++;
    uint16_t entryLen = i - start;
    if (entryLen == 0) continue;

    uint16_t n = (entryLen < ANCS_ALLOW_ENTRY_LEN - 1) ? entryLen : (ANCS_ALLOW_ENTRY_LEN - 1);
    memcpy(s.allow[s.allowCount], &payload[start], n);
    s.allow[s.allowCount][n] = '\0';
    s.allowCount++;
  }

  LOGP("allowlist slot="); Serial.print(slotIndex);
  Serial.print(" entries="); Serial.println(s.allowCount);
}

// Case-sensitive exact match of a bundle id against the slot's allowlist.
static bool allowlistMatch(const AncsSlot& s, const char* bundleId) {
  for (uint8_t i = 0; i < s.allowCount; i++)
    if (strcmp(bundleId, s.allow[i]) == 0) return true;
  return false;
}

// ---- Control Point: request the AppIdentifier for one notification --------
// ANCS "Get Notification Attributes" request layout (historically fragile):
//   [0]      CommandID            = 0x00 (GetNotificationAttributes)
//   [1..4]   NotificationUID      = 4 bytes, LITTLE-ENDIAN, exactly as received
//   [5]      AttributeID          = 0x00 (AppIdentifier)
//            AppIdentifier is variable-length and takes NO 2-byte length field
//            in the request (Title/Subtitle/Message would; we never ask for them).
// Sent as a Write-With-Response (ANCS Control Point has no Write-Without-Response
// property). We issue it non-blocking via sd_ble_gattc_write and let the reply
// arrive asynchronously on Data Source — we do NOT wait here.
static bool cpRequestAppId(AncsSlot& s, uint32_t uid) {
  uint8_t cmd[6];
  cmd[0] = (uint8_t)ANCS_CMD_GET_NOTIFICATION_ATTR;
  cmd[1] = (uint8_t)(uid       & 0xFF);
  cmd[2] = (uint8_t)((uid >> 8)  & 0xFF);
  cmd[3] = (uint8_t)((uid >> 16) & 0xFF);
  cmd[4] = (uint8_t)((uid >> 24) & 0xFF);
  cmd[5] = (uint8_t)ANCS_ATTR_APP_IDENTIFIER;

  ble_gattc_write_params_t p;
  memset(&p, 0, sizeof(p));
  p.write_op = BLE_GATT_OP_WRITE_REQ;   // write with response
  p.flags    = 0;
  p.handle   = s.cp.valueHandle();
  p.offset   = 0;
  p.len      = sizeof(cmd);
  p.p_value  = cmd;

  uint32_t err = sd_ble_gattc_write(s.connHandle, &p);
  return err == NRF_SUCCESS;   // BUSY/RESOURCES -> caller retries next tick
}

// ---- Data Source: reassemble the reply and extract the bundle id ----------
// Response layout (may span multiple notify packets — reassembled PER SLOT):
//   CommandID(1) | NotificationUID(4, LE) | then repeating attribute tuples:
//     AttributeID(1) | AttributeLength(2, LE) | AttributeValue(len bytes)
// We requested only AppIdentifier, so the single tuple is at offset 5.
static void dataHandle(AncsSlot& s, const uint8_t* data, uint16_t len) {
  const int slot = indexOf(&s);

  // A Data Source packet with no fetch outstanding is stale/unexpected — drop.
  if (!s.inFlight) return;

  // Append, guarding against overrun of the fixed per-slot buffer.
  if ((uint32_t)s.rxLen + len > ANCS_DS_BUF) {
    LOGP("slot="); Serial.print(slot); Serial.println(" DS overrun, abort fetch");
    s.rxLen = 0;
    s.inFlight = false;
    return;
  }
  memcpy(&s.rx[s.rxLen], data, len);
  s.rxLen += len;

  // Header (CommandID + UID + AttributeID + AttributeLength) = 8 bytes.
  if (s.rxLen < 8) return;   // wait for more

  if (s.rx[0] != (uint8_t)ANCS_CMD_GET_NOTIFICATION_ATTR ||
      s.rx[5] != (uint8_t)ANCS_ATTR_APP_IDENTIFIER) {
    LOGP("slot="); Serial.print(slot); Serial.println(" DS unexpected header, abort fetch");
    s.rxLen = 0;
    s.inFlight = false;
    return;
  }

  uint16_t attrLen = (uint16_t)s.rx[6] | ((uint16_t)s.rx[7] << 8);
  if ((uint32_t)8 + attrLen > ANCS_DS_BUF) {
    LOGP("slot="); Serial.print(slot); Serial.println(" DS attr too long, abort fetch");
    s.rxLen = 0;
    s.inFlight = false;
    return;
  }
  if (s.rxLen < 8 + attrLen) return;   // attribute value still incomplete

  // Full AppIdentifier received. Extract as a bounded ASCII string.
  char bundleId[ANCS_APPID_MAX];
  uint16_t n = (attrLen < ANCS_APPID_MAX - 1) ? attrLen : (ANCS_APPID_MAX - 1);
  memcpy(bundleId, &s.rx[8], n);
  bundleId[n] = '\0';

  // Fetch complete — free the slot for the next pending UID.
  uint32_t uid = s.inFlightUid;
  s.rxLen    = 0;
  s.inFlight = false;

  LOGP("slot="); Serial.print(slot);
  Serial.print(" conn=0x"); Serial.print(s.connHandle, HEX);
  Serial.print(" uid="); Serial.print(uid);
  Serial.print(" bundle=\""); Serial.print(n ? bundleId : "(empty)"); Serial.println("\"");

  // Missing/empty AppIdentifier can never match — log and stop.
  if (n == 0) {
    LOGP("slot="); Serial.print(slot); Serial.println(" no-match (empty bundle)");
    return;
  }

  if (allowlistMatch(s, bundleId)) {
    LOGP("slot="); Serial.print(slot);
    Serial.print(" MATCH bundle=\""); Serial.print(bundleId); Serial.println("\" -> trigger");
    triggerNotifyLight((uint8_t)slot);
  } else {
    LOGP("slot="); Serial.print(slot);
    Serial.print(" no-match bundle=\""); Serial.print(bundleId); Serial.println("\"");
  }
}

// ---- Notification Source: detect Added, enqueue its UID -------------------
// Packet (8 bytes): EventID(1) | EventFlags(1) | CategoryID(1) | CategoryCount(1)
//                 | NotificationUID(4, LE). Only EventID==Added drives the light.
static void notifHandle(AncsSlot& s, const uint8_t* data, uint16_t len) {
  if (len < 8) return;

  const uint8_t eventID = data[0];
  if (eventID != (uint8_t)ANCS_EVT_NOTIFICATION_ADDED) return;  // ignore Modified/Removed

  const uint32_t uid = (uint32_t)data[4] | ((uint32_t)data[5] << 8) |
                       ((uint32_t)data[6] << 16) | ((uint32_t)data[7] << 24);

  LOGP("slot="); Serial.print(indexOf(&s));
  Serial.print(" conn=0x"); Serial.print(s.connHandle, HEX);
  Serial.print(" Added uid="); Serial.println(uid);

  // Enqueue for ancsService() to fetch its AppIdentifier. Drop if ring full.
  uint8_t nextTail = (uint8_t)((s.pendTail + 1) & ANCS_PEND_MASK);
  if (nextTail == s.pendHead) {
    LOGP("slot="); Serial.print(indexOf(&s)); Serial.println(" pending full, drop uid");
    return;
  }
  s.pend[s.pendTail] = uid;
  s.pendTail = nextTail;
}

// ---- Notify callbacks (BLE event task; resolve slot by connHandle) --------
// One shared function per characteristic kind; chr->connHandle() identifies the
// slot, so notifications route per-connection with no cross-talk.
static void onNotificationSource(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len) {
  AncsSlot* s = slotByHandle(chr->connHandle());
  if (s) notifHandle(*s, data, len);
}
static void onDataSource(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len) {
  AncsSlot* s = slotByHandle(chr->connHandle());
  if (s) dataHandle(*s, data, len);
}

// ---- Subscribe gate -------------------------------------------------------
// Write the CCCDs (the actual ANCS subscription) only once BOTH halves are true:
// the characteristics are discovered AND the owning connection has joined the
// session. Discovery happens on secure; the join arrives later (the first
// identity write). Either order is fine — whichever completes second calls this
// and flips the subscription on. Idempotent via the `ready` flag.
//
// Why gate on join: iOS keeps the bond, so after a disconnect it reopens the ANCS
// link on its own (a "phantom" with no app behind it). A phantom secures and
// discovers but NEVER joins, so it never subscribes here — which means iOS
// delivers ANCS over the real, joined connection (the one holding the allowlist)
// instead of a stale phantom that would silently swallow every notification.
static void enableNotifyIfReady(AncsSlot& s) {
  if (!s.chrsFound || !s.joined || s.ready) return;
  // Enable Data Source FIRST: it carries the attribute-fetch replies, and iOS
  // begins the notification flood the moment the Notification Source CCCD is
  // written — so Data Source must already be subscribed when those fetches reply.
  s.ds.enableNotify();   // CCCD on Data Source
  s.ns.enableNotify();   // CCCD on Notification Source (starts the flood)
  s.ready = true;

  LOGP("slot="); Serial.print(indexOf(&s));
  Serial.print(" conn=0x"); Serial.print(s.connHandle, HEX);
  Serial.println(" ANCS notifications enabled (joined)");
}

// ---- Secured-link callback: discover ANCS (subscribe deferred to join) -----
// ANCS requires a bonded, encrypted link. This fires once the link is secured
// (after the phone accepts pairing). Runs in the Bluefruit callback task, so the
// blocking discovery here is safe. The CCCD writes are deferred to the join gate.
static void securedCallback(uint16_t conn_handle) {
  BLEConnection* conn = Bluefruit.Connection(conn_handle);
  if (!conn) return;

  if (!conn->secured()) {
    // Encryption not established yet (e.g. peer rejected stored keys) — re-ask.
    conn->requestPairing();
    return;
  }

  AncsSlot* s = slotByHandle(conn_handle);
  if (!s) return;                       // not ours
  const int slot = indexOf(s);

  // Discovery is one-time. If we've already discovered, a re-fired secure event
  // just means we should (re)evaluate the subscribe gate (e.g. join raced ahead).
  if (s->chrsFound) { enableNotifyIfReady(*s); return; }

  if (!s->svc.discover(conn_handle)) {
    LOGP("slot="); Serial.print(slot);
    Serial.print(" conn=0x"); Serial.print(conn_handle, HEX);
    Serial.println(" ANCS service NOT found");
    return;
  }

  BLEClientCharacteristic* chrs[] = { &s->cp, &s->ns, &s->ds };
  uint8_t found = Bluefruit.Discovery.discoverCharacteristic(conn_handle, chrs, 3);
  if (found < 3 || !s->cp.discovered() || !s->ns.discovered() || !s->ds.discovered()) {
    LOGP("slot="); Serial.print(slot);
    Serial.print(" ANCS chars incomplete (found="); Serial.print(found);
    Serial.println("), abort");
    return;
  }

  // Register the notify handlers (BLE event task; non-blocking) now — harmless
  // without the CCCD; the actual subscription waits for the join gate.
  s->ns.setNotifyCallback(onNotificationSource, false);
  s->ds.setNotifyCallback(onDataSource, false);
  s->chrsFound = true;

  LOGP("slot="); Serial.print(slot);
  Serial.print(" conn=0x"); Serial.print(conn_handle, HEX);
  Serial.println(" ANCS discovered");

  // Subscribe now if this connection has already joined; otherwise the join will.
  enableNotifyIfReady(*s);
}

// ---- Public lifecycle -----------------------------------------------------
void ancsBegin(void) {
  Bluefruit.Security.setSecuredCallback(securedCallback);
  for (uint8_t i = 0; i < MAX_USERS; i++) g_slots[i].begin();
  LOGP("begin, slots="); Serial.println(MAX_USERS);
}

void ancsOnConnect(uint16_t connHandle, uint8_t userHint) {
  // Idempotent: if this connHandle already owns a slot, do nothing (avoid
  // double-allocating two slots for one link).
  if (slotByHandle(connHandle)) {
    LOGP("connect conn=0x"); Serial.print(connHandle, HEX);
    Serial.println(" already has a slot, ignoring");
    return;
  }

  // Allocate the first free ANCS slot, keyed by connHandle (stable for life).
  AncsSlot* s = nullptr;
  for (uint8_t i = 0; i < MAX_USERS; i++) {
    if (!g_slots[i].active) { s = &g_slots[i]; break; }
  }
  if (!s) {
    LOGP("conn=0x"); Serial.print(connHandle, HEX);
    Serial.println(" no free ANCS slot");
    return;
  }

  s->reset();
  s->active     = true;
  s->connHandle = connHandle;
  s->userHint   = userHint;

  LOGP("connect conn=0x"); Serial.print(connHandle, HEX);
  Serial.print(" -> slot="); Serial.print(indexOf(s));
  Serial.print(" (userHint="); Serial.print(userHint); Serial.println(")");

  // ANCS needs a bonded, encrypted link: ask the phone to pair. iOS shows the
  // PAIR prompt; discovery + subscribe happen in securedCallback once encrypted.
  BLEConnection* conn = Bluefruit.Connection(connHandle);
  if (conn) conn->requestPairing();
}

void ancsOnJoin(uint16_t connHandle) {
  // The owning connection just joined the session (first identity write). Flip
  // the join half of the subscribe gate and subscribe if discovery is already
  // done (otherwise securedCallback will, once it finishes). A no-op for a
  // connection with no ANCS slot, or one that already subscribed.
  AncsSlot* s = slotByHandle(connHandle);
  if (!s) return;
  s->joined = true;
  enableNotifyIfReady(*s);
}

void ancsUnsubscribe(uint16_t connHandle) {
  AncsSlot* s = slotByHandle(connHandle);
  if (!s || !s->ready) return;     // not ours / never finished setup -> nothing to do

  // CCCD=0 on both notify characteristics. We're in the loop() task here, so the
  // blocking GATT-client writes are fine. Mark not-ready so a stale Data Source
  // packet in flight is ignored; ancsOnDisconnect's reset() runs right after.
  s->ns.disableNotify();
  s->ds.disableNotify();
  s->ready = false;

  LOGP("unsubscribe conn=0x"); Serial.println(connHandle, HEX);
}

void ancsOnDisconnect(uint16_t connHandle) {
  // Idempotent: unknown/already-freed handle is a safe no-op. Match the EXACT
  // connHandle; never touch another slot.
  AncsSlot* s = slotByHandle(connHandle);
  if (!s) return;

  LOGP("disconnect conn=0x"); Serial.print(connHandle, HEX);
  Serial.print(" -> slot="); Serial.println(indexOf(s));

  // Stop this slot's ANCS client: halt the fetch state machine, drop the
  // allowlist, reset parse buffers. (The Bluefruit GATT layer already
  // auto-disconnects the matching client chars/service on GAP disconnect; this
  // clears OUR per-slot state and frees the slot for reuse.)
  s->reset();
}

void ancsService(void) {
  const uint32_t now = millis();

  // Issue at most one Control Point fetch per slot per call; non-blocking.
  for (uint8_t i = 0; i < MAX_USERS; i++) {
    AncsSlot& s = g_slots[i];
    if (!s.active || !s.ready) continue;

    // Recover a wedged slot: a fetch that has been in flight too long means its
    // Data Source reply was lost or the Control Point write was rejected. Free the
    // slot so the next pending UID can be fetched — otherwise inFlight never clears,
    // the pending ring fills, and every later notification is dropped (the
    // "pending full, drop uid" deadlock). A late reply that arrives afterwards is
    // ignored by dataHandle's `!inFlight` guard.
    if (s.inFlight && (uint32_t)(now - s.inFlightSinceMS) >= ANCS_FETCH_TIMEOUT_MS) {
      LOGP("slot="); Serial.print(i);
      Serial.print(" fetch timeout uid="); Serial.print(s.inFlightUid);
      Serial.println(" -> recovering slot");
      s.inFlight = false;
      s.rxLen    = 0;
    }

    if (s.inFlight) continue;
    if (s.pendHead == s.pendTail) continue;   // nothing pending

    uint32_t uid = s.pend[s.pendHead];
    if (cpRequestAppId(s, uid)) {
      // Mark in-flight BEFORE the reply can arrive (response follows the write
      // by >=1 connection interval), then advance the ring.
      s.inFlightUid     = uid;
      s.rxLen           = 0;
      s.inFlight        = true;
      s.inFlightSinceMS = now;
      s.pendHead        = (uint8_t)((s.pendHead + 1) & ANCS_PEND_MASK);

      LOGP("slot="); Serial.print(i);
      Serial.print(" conn=0x"); Serial.print(s.connHandle, HEX);
      Serial.print(" CP GetNotificationAttributes uid="); Serial.println(uid);
    }
    // else: BUSY/RESOURCES — leave UID at pendHead; retried next ancsService().
  }
}

void ancsDumpState(void) {
  for (uint8_t i = 0; i < MAX_USERS; i++) {
    const AncsSlot& s = g_slots[i];
    if (!s.active) continue;
    const uint8_t pend = (uint8_t)((s.pendTail - s.pendHead) & ANCS_PEND_MASK);
    LOGP("slot="); Serial.print(i);
    Serial.print(" conn=0x"); Serial.print(s.connHandle, HEX);
    Serial.print(" chrs="); Serial.print(s.chrsFound);
    Serial.print(" joined="); Serial.print(s.joined);
    Serial.print(" ready="); Serial.print(s.ready);
    Serial.print(" allow="); Serial.print(s.allowCount);
    Serial.print(" pend="); Serial.print(pend);
    Serial.print(" inFlight="); Serial.println(s.inFlight);
  }
}
