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
/// Audio is being reworked for a speaker (not the prototype's buzzer);
/// playSound() is a stub pending that hardware.

#include <bluefruit.h>
#include <Adafruit_NeoPixel.h>
#include "modem_types.h"
#include "ancs_client.h"

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

// Cadence + bookkeeping for the once-a-minute session-time broadcast.
constexpr uint32_t SESSION_NOTIFY_MS = 60000;  // push session time every minute
static uint32_t    g_lastSessMS      = 0;       // last session-time push

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

// TODO: speaker audio playback (to be implemented).
// Hardware will drive a speaker rather than the prototype's buzzer; the
// audio path (I2S / PWM / DAC + pin assignment) is still pending.
static void playSound() {
  // intentionally empty - to be implemented
}

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
//   per user: R, G, B, screenTime(uint16), nameLen, name[nameLen]
// The characteristic value is kept current so a fresh reader sees the roster
// without waiting for the next change.
static void notifyRoster() {
  uint8_t buf[1 + MAX_USERS * (3 + 2 + 1 + MAX_USER_NAME_LEN)];
  uint16_t n = 0;
  buf[n++] = g_session.userCount;
  for (uint8_t i = 0; i < g_session.userCount; i++) {
    const User& u = g_session.connectedUsers[i];
    buf[n++] = (uint8_t)((u.color >> 16) & 0xFF);   // R
    buf[n++] = (uint8_t)((u.color >>  8) & 0xFF);   // G
    buf[n++] = (uint8_t)( u.color        & 0xFF);   // B
    buf[n++] = (uint8_t)( u.screenTime        & 0xFF);
    buf[n++] = (uint8_t)((u.screenTime >>  8) & 0xFF);
    uint8_t len = (uint8_t)strnlen(u.name, MAX_USER_NAME_LEN);
    buf[n++] = len;
    memcpy(&buf[n], u.name, len);
    n += len;
  }
  rosterChar.write(buf, n);
  for (uint8_t i = 0; i < g_session.userCount; i++)
    rosterChar.notify(g_session.connectedUsers[i].connHandle, buf, n);
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
  Serial.print("session "); Serial.print(g_session.ID);
  Serial.println(" started -> Active");
}

// Last user out -> tear the session down and go Idle.
static void endSession() {
  g_session.startTimeMS = 0;
  CURR_STATE = State::Idle;
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

  if (firstUser) startSession();
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
  // (the characteristic value is also readable as a fallback).
  nameChar.notify(conn_handle, DEVICE_NAME, strlen(DEVICE_NAME));

  Serial.print("BLE connect: links="); Serial.println(Bluefruit.connected());

  // Keep advertising while physical connection slots remain.
  if (Bluefruit.connected() < MAX_USERS)
    Bluefruit.Advertising.start(0);
}

// Called whenever a central disconnects.
static void disconnectCallback(uint16_t conn_handle, uint8_t /*reason*/) {
  // Tear down this slot's ANCS client first (halt fetches, drop allowlist,
  // reset buffers). Always runs — even for a connection that never joined the
  // session (e.g. a phantom ANCS reconnect). Idempotent + matches the exact
  // conn_handle; safe before the user slot is reclaimed.
  ancsOnDisconnect(conn_handle);

  // Only a connection that actually JOINED the session affects the roster/FX. A
  // non-member drop (phantom reconnect, or a link that never sent its identity)
  // changes nothing visible — no leave FX, no count/roster churn.
  User* u = findUser(conn_handle);
  if (!u) {
    Serial.print("BLE disconnect (non-member): links="); Serial.println(Bluefruit.connected());
    return;
  }

  // Capture the leaving member's color before their slot is reclaimed so loop()
  // can flicker it out and then refresh the user gauge.
  g_fxColor      = colorOrFallback(u->color);
  g_disconnectFx = true;

  removeUser(conn_handle);

  // Tell the phones that remain the new roster size and user table.
  notifyUserCount();
  notifyRoster();

  Serial.print("disconnect: users="); Serial.println(g_session.userCount);
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

// Bluefruit write-callback signature.
static void onColorWrite(uint16_t conn_handle, BLECharacteristic* /*chr*/,
                         uint8_t* data, uint16_t len) {
  char buf[33];
  uint16_t n = len < sizeof(buf) - 1 ? len : sizeof(buf) - 1;
  memcpy(buf, data, n);
  buf[n] = '\0';

  uint32_t color = parseRGB(buf);

  // First identity write joins this connection to the session (no-op if already).
  // A phantom ANCS reconnect never sends a color, so it never joins or animates.
  User* u = joinUser(conn_handle);
  if (!u) return;                  // session full — ignore the write
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

void setup() {
  Serial.begin(115200);

  // Lights: init first, then sit at the idle (cleared) color.
  strip.begin();
  strip.setBrightness(150);
  strip.clear();
  strip.show();

  // BLE: Bluefruit peripheral + GATT server.
  // Allow up to MAX_USERS concurrent peripheral connections.
  Bluefruit.begin(MAX_USERS);
  Bluefruit.setTxPower(4);
  Bluefruit.setName(DEVICE_NAME);
  Bluefruit.Periph.setConnectCallback(connectCallback);
  Bluefruit.Periph.setDisconnectCallback(disconnectCallback);
  // Parent service must begin() before the characteristics it owns.
  modemSvc.begin();

  ownerChar.setProperties(CHR_PROPS_READ);
  ownerChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  ownerChar.setMaxLen(MAX_USER_NAME_LEN);
  ownerChar.begin();
  ownerChar.write("Owner");

  // READ + NOTIFY so the puck can push its name back to connected phones.
  nameChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  nameChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  nameChar.setMaxLen(MAX_DEVICE_NAME_LEN);
  nameChar.begin();
  nameChar.write(DEVICE_NAME);

  // Sound write target retained for parity with the prototype's GATT table.
  soundChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  soundChar.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  soundChar.setMaxLen(1);
  soundChar.begin();

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
  sessChar.begin();
  sessChar.write16(0);

  // READ + NOTIFY: full connected-user roster (see notifyRoster for layout).
  // Pushed whenever the table changes so every phone shows the same leaderboard.
  rosterChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  rosterChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  rosterChar.setMaxLen(1 + MAX_USERS * (3 + 2 + 1 + MAX_USER_NAME_LEN));
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
    ancsUnsubscribe(h);                      // (B) stop consuming ANCS, keep bond
    Bluefruit.disconnect(h);
    Serial.print("disconnect issued conn=0x"); Serial.println(h, HEX);
  }

  // The puck owns the session clock: push it to every phone once a minute so
  // all devices stay in sync. (A baseline is also pushed on each join.)
  if (CURR_STATE == State::Active &&
      (int32_t)(now - g_lastSessMS) >= (int32_t)SESSION_NOTIFY_MS) {
    g_lastSessMS = now;
    notifySessionTime();
  }

  // A color write starts the connect FX: spin -> hold lit -> gauge -> off.
  if (g_connectFx) {
    g_connectFx = false;
    g_fxPhase  = FxPhase::Spin;
    g_fxStep   = 0;
    g_fxNextMS = now;                        // first spin frame now
    playSound();
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
      playSound();
    }
  }

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
