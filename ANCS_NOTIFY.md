# MODEM — Per-user notification signaling (ANCS)

The puck lights a specific user's color when an app on **that user's** allowlist
sends them a notification. Each connected phone gets an independent ANCS client
on the puck (one per connection slot); there is no cross-user reconciliation.

## ⚠️ Characteristic UUID deviation from the brief

The brief specified `fb590005` for the allowlist characteristic. **That selector
was already taken** — the live GATT table uses `…0005` for the **color**
characteristic. The allowlist therefore lives at the next free selector,
**`fb59000A`**. Nothing in `0001–0009` was altered.

Current GATT table (service `fb590000-ec62-4ba6-baf9-c02429a2a3ac`):

| UUID | Meaning | Props |
|------|---------|-------|
| `…0001` | Owner | R |
| `…0002` | Device name | R / N |
| `…0003` | Sound | W / WnR |
| `…0004` | User display name | W / WnR |
| `…0005` | User color | W / WnR |
| `…0006` | User count | R / N |
| `…0007` | Screen time | W / WnR |
| `…0008` | Session time | R / N |
| `…0009` | Roster | R / N |
| **`…000A`** | **Notification allowlist (NEW)** | **W / WnR** |
| **`…000B`** | **Disconnect request (NEW)** | **W / WnR** |

## `fb59000A` — allowlist wire format

- **Direction:** phone → puck. **Write WITH response, using long/queued writes**
  (`allowLongWrite`). The payload routinely exceeds one ATT MTU (`mtu - 3`); a
  write-without-response throws in that case (flutter_blue_plus won't fragment
  it) and the list silently never arrives, so the app must use a long write. The
  SoftDevice reassembles the queued write and fires `onAllowlistWrite` once with
  the full payload (`allowChar` is sized to `ALLOW_PAYLOAD_MAX`).
- **Payload:** the writing phone's chosen app **bundle IDs** as a UTF-8 list,
  separated by `\n` (newline). `\r` and NUL are also accepted as separators, so a
  trailing NUL is harmless. The app sends `\n`-joined IDs plus a trailing NUL.
- **Cap:** `ALLOW_PAYLOAD_MAX = 256` bytes (firmware) — the app drops whole
  entries that would overflow (never splits a bundle ID).
- **Per-slot storage:** parsed into a fixed `16 × 32` array on the writing
  connection's ANCS slot (`ANCS_ALLOW_MAX × ANCS_ALLOW_ENTRY_LEN`). Each write
  **replaces** that slot's list. Strictly per-slot — never shared/merged.
- **Routing:** the firmware resolves the **writing `connHandle`** to its ANCS
  slot via `ancsSlotForHandle()` (handles are opaque/reused — never assumed equal
  to a slot index), then calls `ancsSetAllowlist(slot, payload, len)`.

Example payload (18 + 1 + 12 + 1 bytes):

```
com.spotify.client\ncom.apple.MobileSMS\0
```

## Supported-app bundle-ID map

Curated in **one** place: `modem_app/lib/notify_apps.dart` → `kNotifyApps`
(`display name → iOS bundle ID`). Add apps there. Bundle IDs must match exactly
what ANCS reports (the puck matches **case-sensitively**). Seeded set includes:

| App | Bundle ID |
|-----|-----------|
| Spotify | `com.spotify.client` |
| WhatsApp | `net.whatsapp.WhatsApp` |
| Messages | `com.apple.MobileSMS` |
| Instagram | `com.burbn.instagram` |
| … | (see `kNotifyApps`) |

This list is **separate** from the Screen Time picker. Screen Time's
`FamilyActivitySelection` yields opaque `ApplicationToken`s, not bundle IDs, so
it cannot drive ANCS matching. Both pickers coexist; the Screen Time shield is
**not** a notification filter (ANCS reports shielded apps too — the per-slot
bundle-ID match is the sole filter).

## Per-slot lifecycle

```
NFC tap → BLE connect
   └─ connectCallback → ancsOnConnect(connHandle, 0xFF)
        └─ allocate ANCS slot (keyed by connHandle), requestPairing()
        NB: connecting is NOT joining — no user/roster/FX yet (see below)
   iOS PAIR accepted → link secured
        └─ securedCallback: discover ANCS, enable CCCD on
           Notification Source + Data Source
   app: write color/userName → joinUser(): first identity write adds the user,
        starts the session if first, broadcasts count/roster/session-clock
   app: write fb59000A  → onAllowlistWrite → ancsSlotForHandle → ancsSetAllowlist
        └─ slot's allowlist populated

Notification arrives on phone:
   Notification Source notify (Added) → enqueue UID (per slot)
   loop()/ancsService(): Control Point "GetNotificationAttributes"
        (CommandID 0x00 | UID 4B LE | AttributeID 0x00 AppIdentifier, no length)
   Data Source notify(s) → reassemble (per-slot buffer) → extract bundle ID
   allowlist match (case-sensitive exact) → triggerNotifyLight(slot)
        └─ resolve slot→conn→user color, raise g_notifyFx (BLE task only sets a
           flag); loop() pulses the strip in that user's color N times → off

Second tap / out-of-range / app "leave" → BLE disconnect
   app writes fb59000B → onDisconnectRequest enqueues connHandle
        └─ loop() drains: ancsUnsubscribe(connHandle) → Bluefruit.disconnect()
   (also: out-of-range / power-off drops the link directly)
   └─ disconnectCallback → ancsOnDisconnect(connHandle)  [always]
        └─ idempotent: find slot by exact connHandle, halt fetch state machine,
           drop allowlist, reset buffers, free slot.
        └─ if the conn was a joined member: removeUser + leave FX + roster push;
           a non-member drop (phantom reconnect) changes nothing visible.
   bond: ALWAYS kept. iOS exposes no API to forget its half, so a puck-only bond
        wipe would block every future reconnect until a manual "Forget Device".
```

## `fb59000B` — disconnect request

The phone cannot reliably drop a bonded ANCS link from its own side: on iOS,
`cancelPeripheralConnection` only releases the app's interest, but the system
keeps the connection up because of the bond / active ANCS consumer. So the puck
(the **peripheral**) must initiate the GAP disconnect. The app writes a single
byte here (value ignored); `onDisconnectRequest` enqueues the writing
`connHandle` and `loop()` issues `Bluefruit.disconnect(connHandle)`. That fires
the normal `disconnectCallback`, so the existing graceful teardown (ANCS slot
reset, user removal, leave FX) runs unchanged. The write callback never calls
into the SoftDevice itself — it only enqueues — so the disconnect happens in the
`loop()` task, not deep inside a GATT write callback.

**The phantom-reconnect problem, and why the bond is kept.** A phone's app link
and its system ANCS link are the *same* GAP connection. While the bond persists,
iOS reopens that link on its own after the puck drops it — purely to keep ANCS
alive (a bonded ANCS accessory is *supposed* to stay connected). It is tempting to
delete the puck's bond to stop this, but **iOS exposes no API to forget its half
of a bond** — neither CoreBluetooth nor the app — so a puck-only wipe leaves an
asymmetric bond that fails encryption on every future reconnect until the user
manually "Forget"s the device in Settings. So the bond is **always kept**, and the
phantom reconnect is instead made harmless two ways:

- **(B) Unsubscribe before disconnect.** The fb59000B drain calls
  `ancsUnsubscribe(connHandle)` (CCCD=0 on Notification Source + Data Source)
  right before `Bluefruit.disconnect()`, so iOS sees the puck stop consuming ANCS
  and has less reason to immediately reopen the link.
- **(A) Connecting ≠ joining.** A bare BLE connection no longer creates a session
  user; `joinUser()` runs only on the first identity write (color / userName),
  which only the app sends. A phantom reconnect has no app behind it, never writes
  an identity, and so never becomes a user, never animates, and (if it manages to
  re-subscribe ANCS on its own) its notifications are ignored by
  `triggerNotifyLight` because the conn has no member. Leaving the app and
  reconnecting later are both clean, with no "Forget Device" step.

## Firmware modules

- `ancs_client.{h,cpp}` — per-slot ANCS consumer (one `AncsSlot` per connection;
  `AncsSlot g_slots[MAX_USERS]`). Public API: `ancsBegin`, `ancsOnConnect`,
  `ancsOnDisconnect`, `ancsService`, `ancsUnsubscribe`, `ancsSetAllowlist`,
  `ancsSlotForHandle`, `ancsConnHandle`. `triggerNotifyLight(slot)` lives in
  `modem_firmware.ino` and pulses the matched member's color.
- `spike_ancs_multi/` — Phase 0 de-risk spike + `PHASE0_REPORT.md`.

All ANCS state/buffers/allowlists are strictly per-slot; no dynamic allocation in
hot paths; notify callbacks never block (the Control Point write is pumped from
`loop()`).
