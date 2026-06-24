# Multi-Connection Leaderboard Inconsistency — Diagnosis

**Scope:** discovery/diagnosis only, no firmware changes. All refs are
`modem_firmware/modem_firmware.ino` unless noted.
**Symptom:** with several phones connected, rosters/counts differ between devices
or some users never appear.

---

## 1. SoftDevice connection capacity

**What the code does**
- `Bluefruit.begin(MAX_USERS)` → `Bluefruit.begin(8)` (line 1149). `MAX_USERS = 8`
  (`modem_types.h:21`). The second (central) arg defaults to 0, so the build allows
  **8 simultaneous peripheral links** — confirmed bring-up-tested in
  `spike_ancs_multi/PHASE0_REPORT.md:71` ("`Bluefruit.begin(8)` successfully").
- `configAttrTableSize(0x1000)` is raised (line 1147) so the GATT table fits.
- Advertising is stopped once `Bluefruit.connected() >= MAX_USERS`
  (connectCallback line 704; setup line 1299).

**Verdict: capacity is nominally 8, but it is the wrong 8.** The cap counts
**physical GAP links**, while `userCount` counts **session members**, and on iOS
these are not 1:1:
- A phone reports pickups over a **separate background connection** — the firmware
  itself documents this: onPickupWrite, lines 64–73 and 1085 ("the reporter rides
  its OWN background connection, not the app's session link, so the writing
  connHandle is NOT the session user's").
- iOS also opens **phantom ANCS reconnects** on the bonded link with no app behind
  them (lines 342–352, 682–683).

So one engaged phone can hold **2 physical links** (app/session link + background
pickup/ANCS link). With an 8-link ceiling, the budget can be exhausted at **~4
phones**, at which point `connectCallback` stops advertising (line 704) and the
**5th phone can no longer connect even though `userCount` is only 4** — exactly the
"users fail to appear at all" symptom. This is listed as an open "dual-connection
risk" in the project memory (pickup-reporting note).

**Gaps in config**
- No `Bluefruit.configPrphConn(...)` / bandwidth / GAP event-length tuning anywhere
  (grep-confirmed). With 8 links on default `BANDWIDTH_NORMAL`, per-link throughput
  drops; not a disconnect cause by itself, but it slows the roster fan-out under
  load.

**Fix (future pass)**
- Decouple the advertising gate from raw link count: keep advertising until
  `userCount >= MAX_USERS` (or budget links explicitly as members + reserved
  background links). Consider raising the peripheral link count above 8 if each
  phone legitimately needs 2 links, and size SoftDevice RAM accordingly.
- The existing heartbeat already logs `links=` vs `users=` (dumpState, lines
  658–660). **Verify in a field log whether `links > users`** — that single line
  confirms or kills this hypothesis immediately.

---

## 2. Connected-user tracking

**What the code does**
- Structure: `Session g_session` with `User connectedUsers[MAX_USERS]`, `userCount`
  (`modem_types.h:84–89`). Each `User` carries `connHandle`, `color`, `pickups`,
  `pendingLeaveMS`, etc. (`modem_types.h:55–74`).
- Lookup is a **linear connHandle search** — `findUser` (lines 523–528),
  `removeUser` (604–611) — correctly NOT assuming `handle == index`. ANCS keeps its
  own stable slot space keyed by handle (`ancs_client.h:11–21`). This is correct.
- Populate on join: `addUser` appends at `connectedUsers[userCount]` (558–564).
  Clear on leave: `removeUser` shift-packs and decrements (604–611).
- Disconnect path is split (disconnectCallback 709–741): app-requested leaves
  finalize immediately; spontaneous drops are held `LEAVE_GRACE_MS` (6 s) as
  ghosts and finalized by the loop sweep (1345–1356) if not reclaimed.

**Verdict: mostly correct, but it rests on an UNENFORCED invariant.**
- The reclaim and pickup-attribution paths match users **by color** and three
  comments assert "colors are unique per session (collisions resolved at join)"
  (lines 634, 788, 932). **There is no such resolution anywhere in the firmware.**
  `onColorWrite` simply does `u->color = color` (line 804) with no collision check
  (grep for uniqueness/collision logic returns only the comments). If two phones
  pick the same color:
  - `reclaimPendingLeave` (638–650) can rebind a reconnecting phone onto the
    **wrong** user's slot.
  - `findUserByColor` (936–941) attributes pickups to the **wrong** user and the
    spike detector's distinct-slot count is wrong.
  This directly yields divergent counts. The invariant may currently hold only
  because the (out-of-scope) app assigns distinct colors — fragile and unverified
  on the firmware side.
- Ghost-slot handling itself is sound: spontaneous-drop slots are intentionally
  retained, and the grace sweep finalizes them; the intentional-leave set
  (markIntentionalLeave/takeIntentionalLeave, 359–376) is deduped and swap-removed
  so a reused handle can't inherit a stale flag. No leaked-ghost path found.

**Fix (future pass)**
- Either enforce color uniqueness at join (reject/auto-nudge a duplicate, or carry
  a real per-user ID), or stop keying identity on color. The `User.ID` field
  already exists (`modem_types.h:56`) and would be a stable handle if the app
  echoed it. At minimum, make the three "colors are unique" comments true in code.

---

## 3. Pickup-count data model

**What the code does**
- Canonical source: the puck owns the count array. Each `User.pickups` is
  incremented puck-side in `onPickupWrite` (line 1114) and the **full** table is
  serialized in `notifyRoster` (501–520) on the **roster** characteristic
  **UUID_ROSTER**, selector `0x09` (line 49). Phones render from this array; they
  do **not** compute standings locally. This is the correct (single-canonical)
  design.
- Roster value layout (501–516): `[0]=userCount`, then per user
  `R,G,B, pickups(uint16 LE), nameLen, name[nameLen]`.
- `notifyRoster` keeps the readable value current (`rosterChar.write`, 517) **and**
  fans out (518–519). Char max len is the full 241 bytes (setMaxLen, line 1233).

**Verdict: model is correct, but the payload TRUNCATES over notifications.**
- Worst-case size = `1 + 8*(3+2+1+24)` = **241 bytes**.
- A BLE notification can carry only **(ATT_MTU − 3)** bytes; Bluefruit silently
  truncates to fit. **The firmware never negotiates/sets MTU** (no
  `configPrphConn`/`requestMtu`, grep-confirmed), so the effective cap is whatever
  the link negotiates:

  | Negotiated MTU | notify usable | full-name users that fit | truncates at |
  |---|---|---|---|
  | 23 (default, no exchange) | 20 B | 0 | 1+ |
  | 185 (typical iOS) | 182 B | 6 | 7+ |
  | 247 (max) | 244 B | 8 | none |

  On a typical iOS MTU of 185, the **7th and 8th users' rows are cut off** the
  notified roster. A phone that parses `userCount` but receives a short buffer
  either mis-parses or shows a partial list → different phones show different
  rosters. (Long *reads* can fetch the full 241 via ATT Read Blob, so a phone that
  re-reads sees more than one that relies on notify — another divergence axis.)

**Fix (future pass)**
- Explicitly `Bluefruit.configPrphConn(247, ...)` before `begin()` and verify the
  negotiated MTU per link (log `Bluefruit.Connection(h)->getMtu()`); and/or chunk
  the roster (paged notifications / per-row notify with a sequence header) so it
  never depends on MTU. Do not rely on names being short.

---

## 4. Notification fan-out

**What the code does**
- Every broadcaster iterates **all session members** and notifies each handle:
  - `notifyUserCount` (474–479)
  - `notifySessionTime` (483–492)
  - `notifyRoster` (501–520)
  - `notifySpike` (993–1002)
- `onPickupWrite` increments the count and calls `notifyRoster()` (1115), which
  pushes the full table to **everyone**, not just the triggering link.

**Verdict: CORRECT.** The classic "only notify the triggering connHandle" bug is
**not present** — fan-out walks `connectedUsers[]` and notifies all members on
every change. This is the right pattern.

**Minor notes (not the root cause)**
- Fan-out targets `connectedUsers[i].connHandle` for `i < userCount`, which
  includes users in the 6 s leave-grace window holding a **stale** handle; those
  notifies just fail harmlessly (Bluefruit no-ops a dead/un-subscribed handle).
- A pickup arrives on a background link whose handle is **not** a member, so the
  triggering link is never in the fan-out set anyway — reinforcing that fan-out
  must (and does) target the member table, not the writer.

---

## 5. CCCD / subscription state

**What the code does**
- All NOTIFY characteristics are declared `CHR_PROPS_READ | CHR_PROPS_NOTIFY`
  (roster 1231, count 1200, session 1223, spike 1266, charge 1274). CCCD is
  per-connection and managed by Bluefruit; `notify(handle,...)` only emits to
  handles that subscribed — correct and per-connection.
- **Back-fill of late joiners is handled:** `joinUser` (577–599) calls
  `notifyUserCount` + `notifyRoster` + `notifySessionTime` at join, and each
  broadcaster also keeps the **readable characteristic value current**
  (`rosterChar.write` 517, `countChar.write8` 476, `sessChar.write16` 489). So a
  phone that joins later gets the existing users' full counts via the next push
  **or** a one-time read — it is not a deltas-only model. Good.
- The puck always pushes the **FULL** array on change (501–520), so join order
  doesn't change the converged state — except for the truncation in §3.

**Verdict: design is correct; convergence is defeated only by §3 truncation and a
join-ordering race.**
- **Race:** `joinUser` fires `notifyRoster()` the instant the first identity write
  arrives (color/name, 802/764). If the phone has **not yet written CCCD=1** on the
  roster characteristic, that join-time notification is dropped and the phone must
  fall back to a manual read of the (current) value. If the app does not read the
  roster after subscribing, that phone can sit with a stale/empty roster until the
  *next* unrelated change. This is app-timing-dependent but the firmware makes it
  worse by not re-pushing after a short settle.

**Fix (future pass)**
- After a join, also re-push the roster on a short delay (or on the CCCD-write
  event) so a phone that subscribes just after its identity write still gets a
  notification, not only a value it must remember to read. Pair with the §3 MTU/
  chunking fix so the pushed value is complete.

---

## Ranked root causes (most → least likely)

1. **MTU truncation of the roster notification (§3).** No MTU negotiation; a
   241-byte worst-case roster is silently cut to ~182 B on iOS, dropping the 7th–
   8th users' rows. Best explanation for "different rosters / users missing" once
   the session grows. **Highest confidence, fully in firmware.**
2. **Physical-link budget consumed by non-member connections (§1).** iOS
   background pickup links + phantom ANCS reconnects can exhaust the 8-link ceiling
   at ~4 phones, stopping advertising while `userCount` is still low → later phones
   can't connect at all. Verify via the existing `links=` vs `users=` heartbeat.
3. **Unenforced "unique color" invariant (§2).** Reclaim and pickup attribution key
   on color with no uniqueness guarantee in firmware; a color clash mis-attributes
   counts and can rebind the wrong slot, producing divergent counts.
4. **Join-time CCCD race (§5).** Roster pushed before the late joiner subscribes;
   recovery depends on the app issuing a read. Causes a transient stale roster on
   freshly joined phones.

Items 4 is real but transient; 1–3 are the substantive multi-connection
inconsistencies. None require redesigning the leaderboard or changing the metric —
each has a contained fix listed above for the follow-up pass.

---

# Implementation pass — fixes applied

All changes are in `modem_firmware.ino`. Build verified:
`arduino-cli compile --profile nrf52840` → **OK**, 22% flash, 10% RAM globals.
**Confirmed the before-state:** the core default is **MTU 23** (`bluefruit.cpp`
`_sd_cfg.prph.mtu_max = BLE_GATT_ATT_MTU_DEFAULT`) and the firmware never raised it,
so roster notifications were capped at **20 bytes** — worse than the §3 table's
iOS-185 row; effectively only `userCount` + a partial first row was reaching phones.

### FIX 1 — roster truncation (root cause #1)
- **1a. (MTU bump tried, then REVERTED — RAM regression.)** Raising the ATT MTU to
  247 via `configPrphConn(247, …)` across MAX_USERS links overflowed the
  SoftDevice's RAM reservation: `sd_ble_enable()` returns `NO_MEM`,
  `Bluefruit.begin()` returns false (the sketch never checks it), BLE never comes
  up, and **the puck stops being connectable**. The compile-time "RAM 10%" figure
  does not capture this (it's app RAM, separate from the SoftDevice's runtime
  reservation). Left the MTU at the core default (23). The bump was only an
  optimization — see 1b: the roster is already correct at any MTU via the app's
  read-fallback. Per-link MTU is still logged in `connectCallback` / `dumpState`
  for visibility. A future MTU increase must be stepped up with on-device
  confirmation that `begin()` still succeeds (and/or a larger linker RAM reserve).
- **1b.** Roster framing left **unpaged** (single `[0]=userCount` + rows buffer for
  BOTH the readable value and the notification). At MTU 247 the whole 241-byte
  worst-case table rides one notification. If a link settles on a smaller MTU
  (iOS ~185), the notification truncates, but the **app already detects the short
  buffer and falls back to a full GATT read** (`_rosterComplete` →
  `_refreshRosterByRead` in `puck_connection.dart`); an ATT Read Blob returns the
  whole value regardless of MTU, so every phone still converges.
  - **No app change required.** (An earlier draft paged the notification, which
    would have forced a Flutter parser change and conflicted with the app's
    shared notify/read stream — reverted in favor of the app's existing
    read-fallback, which keeps notify and read on one unambiguous framing.)

### FIX 2 — advertising gate decoupled from physical links (root cause #2)
- `connectCallback` now keeps advertising while `g_session.userCount < MAX_USERS`
  instead of `Bluefruit.connected() < MAX_USERS`, so transient iOS background/
  phantom links no longer hide the puck while session seats remain open.
- **Deliberately NOT raised** the physical link ceiling (`Bluefruit.begin(8)`):
  doing so would also require resizing the ANCS slot pool
  (`ancs_client g_slots[MAX_USERS]`, allocated per-connection) and re-validating
  SoftDevice RAM on hardware. The dual-connection hypothesis is still unconfirmed;
  the `links=` vs `users=` + per-user `mtu=` heartbeat is now the instrument to
  confirm it before taking that larger, hardware-gated change. Commented in setup.

### FIX 3 — color-uniqueness backstop (root cause #3)
- Added `COLOR_PALETTE[MAX_USERS]` mirroring the app's `kPalette`
  (`lib/settings.dart`) and `resolveUniqueColor()`. `onColorWrite`'s **new-join**
  path now nudges a colliding color to the first free palette entry, so
  `reclaimPendingLeave` / `findUserByColor` can never alias two members. The
  reclaim path still matches on the exact (already-unique) color. The app already
  prevents collisions in its UI, so this is defense-in-depth for the commit race.
  Left an in-code note that ID-based identity (`User.ID`) is the preferred future
  direction (needs app cooperation; out of scope).

### FIX 4 — join-time CCCD race (root cause #4)
- `joinUser` schedules a single delayed roster re-push (`g_rosterRepushMS`,
  `ROSTER_REPUSH_MS = 600 ms`); `loop()` fires it once. A phone that subscribed to
  the roster characteristic just after its identity write now gets a notification
  rather than relying on a manual read. One-shot deadline — no polling loop added.

## How to verify on hardware (existing instrumentation only)
- **FIX 1a (MTU):** on connect, serial shows `BLE connect: links=N mtu=…`; the
  heartbeat shows `user[i] … mtu=…`. With the MTU bump reverted this reads **23**
  (the SoftDevice caps negotiation at its `mtu_max`); the value is logged so a
  future bump can be confirmed. Critically: the puck must still **advertise and
  accept connections** — a failed `begin()` (the symptom of the reverted bump)
  shows as `links=0` heartbeats forever and no `BLE connect` line.
- **FIX 1b (roster completeness):** with **7–8 phones** in a session, every phone's
  leaderboard lists **all** members. At MTU 23 every multi-user push truncates and
  the app's `MODEM_roster` read-fallback fetches the full value (one extra read per
  push — chattier but correct). Before this pass the firmware also ran at MTU 23,
  so this matches the known-good connectivity profile.
- **FIX 2:** heartbeat `links=` vs `users=` — if `links > users` appears, the
  dual-connection capacity hypothesis is confirmed and the ceiling-raise follow-up
  is warranted. Advertising now persists until `users == 8`.
- **FIX 3:** if two phones ever commit the same color, serial logs
  `color collision 0x… -> nudged to palette[0x…]` and the roster shows distinct
  colors; pickup counts stay attributed to the right user.
- **FIX 4:** a freshly joined phone shows the complete roster within ~0.6 s without
  needing to background/foreground or re-read.


Generate a Claude Code prompt to fix the issues addressed.
