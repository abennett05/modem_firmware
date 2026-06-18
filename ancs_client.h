/// ancs_client.h
/// - - - - - - - - - - - -
/// Per-connection-slot ANCS (Apple Notification Center Service) consumer for the
/// Modem Puck. iOS exposes notifications only to a BONDED BLE peer, and a phone
/// cannot see its own notifications — so the puck is the ANCS *client* (GATT
/// client) on each phone's link. ANCS is a per-bonded-pair relationship, so the
/// puck runs up to MAX_USERS INDEPENDENT ANCS client instances, one per slot.
/// Each instance serves exactly one phone, holds that phone's allowlist, and
/// lights that user's color. There is NO cross-user reconciliation.
///
/// SLOT MODEL — read this before touching the code
/// - - - - - - - - - - - -
/// connHandle is the ONLY durable key. Do NOT assume connHandle == slotIndex:
/// handles are opaque and reused by the stack. The orchestrator's
/// connectedUsers[] array is *packed* (removeUser() shifts entries down), so a
/// user's index there is NOT stable across another user's disconnect. Therefore
/// this module owns its OWN stable slot space (0..MAX_USERS-1), allocated on
/// connect and held for the connection's lifetime, keyed by connHandle. Resolve
/// connHandle -> ANCS slot with ancsSlotForHandle() (used by Phase B allowlist
/// writes and Phase C teardown). The "slotIndex" everywhere below is THIS
/// module's stable slot, not the orchestrator's array index.

#pragma once
#include <stdint.h>
#include "modem_types.h"   // MAX_USERS

// Per-slot allowlist sizing (fixed; no dynamic allocation). Bundle IDs are
// ASCII like "com.spotify.client"; 31 chars + NUL covers all common ones.
constexpr uint8_t ANCS_ALLOW_MAX       = 16;   // entries per slot
constexpr uint8_t ANCS_ALLOW_ENTRY_LEN = 32;   // bytes per entry (incl NUL)

/// Lifecycle (wired into the orchestrator's connect/disconnect callbacks).
/// ancsBegin            — once at setup(); registers the per-slot client
///                        services and installs the secured-link callback.
/// ancsOnConnect        — a central connected. userHint is the orchestrator's
///                        current array index (logging only); the module keys by
///                        connHandle and allocates its own stable slot, then
///                        requests pairing (ANCS needs a bonded, encrypted link).
/// ancsOnDisconnect     — tear down the slot owning connHandle. Idempotent: a
///                        disconnect for an unknown/already-freed handle is a
///                        safe no-op. Never touches any other slot.
/// ancsService          — pump from loop(); issues the pending Control Point
///                        attribute fetches. Non-blocking.
void ancsBegin(void);
void ancsOnConnect(uint16_t connHandle, uint8_t userHint);
void ancsOnDisconnect(uint16_t connHandle);
void ancsService(void);

/// ancsOnJoin — the connection owning connHandle just joined the session (its
/// first identity write). ANCS is only subscribed for joined connections, so the
/// orchestrator must call this from joinUser(). Subscribing is gated on BOTH the
/// join and ANCS discovery completing; this flips the join half and subscribes if
/// discovery is already done. Idempotent; a no-op for a connection with no slot.
/// This is what keeps a phantom ANCS reconnect (iOS reopening the bonded link
/// with no app behind it) from subscribing and silently swallowing notifications.
void ancsOnJoin(uint16_t connHandle);

/// Disable this slot's ANCS notifications (writes CCCD=0 on Notification Source +
/// Data Source) WITHOUT tearing down the slot or the bond. Called just before an
/// explicit user-requested disconnect so iOS sees the puck stop consuming ANCS —
/// relieving its pressure to immediately reopen the bonded link. A no-op if the
/// handle has no ready slot. The bond is intentionally left intact (so the next
/// reconnect re-secures instantly); securedCallback re-enables notifications then.
void ancsUnsubscribe(uint16_t connHandle);

/// Per-slot allowlist (Phase B feeds this from the fb590005 write handler).
/// payload is the raw UTF-8 list as written by the phone: bundle IDs separated
/// by '\n', '\r', or NUL. Parsed into the slot's fixed array (extra entries /
/// over-length entries are truncated/dropped). Replaces the slot's prior list.
void ancsSetAllowlist(uint8_t slotIndex, const uint8_t* payload, uint16_t len);

/// connHandle <-> ANCS slot resolution (for Phase B write routing & Phase C).
/// Returns -1 if no active slot owns connHandle.
int      ancsSlotForHandle(uint16_t connHandle);
/// Returns the connHandle bound to an ANCS slot (BLE_CONN_HANDLE_INVALID if free).
uint16_t ancsConnHandle(uint8_t slotIndex);

/// Diagnostic: log every active ANCS slot's lifecycle flags (chrs discovered /
/// joined / ready), bound connHandle, allowlist size, queued-notification depth
/// and in-flight-fetch state. Read-only; called from the orchestrator's
/// heartbeat to make multi-connection issues visible in a field log.
void ancsDumpState(void);

/// Light trigger — IMPLEMENTED ELSEWHERE (modem_firmware.ino) as a stub.
/// Called when an Added notification's app bundle ID matches slotIndex's
/// allowlist. LED/animation actuation is out of scope here; the stub just logs.
void triggerNotifyLight(uint8_t slotIndex);
