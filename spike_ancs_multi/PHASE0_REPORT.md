# Phase 0 — De-risk spike report: multiple simultaneous bonded ANCS links

**Verdict (analysis): FEASIBLE.** Every load-bearing assumption checks out against
the installed core (`adafruit:nrf52 1.7.0`, SoftDevice **S140 6.1.1**). The one
thing I cannot do from here is physically bond iPhones — the spike sketch
(`spike_ancs_multi.ino`) produces the empirical numbers; flash it and report the
serial log. Sections below mark **[verified in source]** vs **[confirm on HW]**.

> Note: `sketch.yaml` pins core **1.6.1**, but the machine has **1.7.0** installed.
> Findings are from 1.7.0. Either bump the pin or install 1.6.1 before building.

---

## 1. Can a peripheral run an ANCS *client*? — YES [verified in source]

The puck advertises; iPhones connect to it, so the puck is the **GAP peripheral**
and the iPhone is the central. ANCS makes the puck the **GATT client** of the
phone's ANCS service. GATT-client and GAP-peripheral roles are independent.

`bluefruit.cpp` `begin()` sets `BLE_CONN_CFG_GATTC` (the GATT-client write-command
queue) inside the **peripheral** connection-config block (`CONN_CFG_PERIPHERAL`).
So GATT-client operations are provisioned on every peripheral link.
**No central role is required** — `Bluefruit.begin(8, 0)` (8 peripheral, 0 central,
exactly today's `Bluefruit.begin(MAX_USERS)`) already supports ANCS-as-client on
all 8 links. The stock `Peripheral/ancs` example confirms the pattern (it runs as
a peripheral).

## 2. One ANCS client per phone — requires N instances [verified in source]

`BLEClientService` stores a **single** `_conn_hdl`; `BLEClientCharacteristic` keeps
a single `_chr` and derives its connection from `_service->connHandle()`. So one
`BLEAncs` instance binds to exactly **one** connection. The single-global `BLEAncs`
in Adafruit's example therefore **cannot** serve 8 phones.

→ Design: an array `BLEAncs g_ancs[NUM_SLOTS]` (or, in production, a custom
per-slot client). Multi-instance is safe: the notification trampoline recovers its
owner via `chr->parentService()`, so each instance dispatches to its own state.

## 3. No cross-talk — guaranteed by the dispatcher [verified in source]

`BLEGatt::_eventHandler` routes every client event (`HVX`, `WRITE_RSP`, `READ_RSP`)
**only** to the client characteristic whose `connHandle()` equals the event's
`conn_handle` *and* whose value handle matches:

```c
for (i ...) if (evt_conn_hdl == _client.chr_list[i]->connHandle()) { ... }
```

Notifications from phone A can only reach phone A's instance. On
`BLE_GAP_EVT_DISCONNECTED` the same loop auto-calls `disconnect()` on the matching
client chars/services — a free assist for Phase C teardown.

## 4. Static registry capacity — 8 slots fit with headroom [verified in source]

`BLEGatt.h`:

| Registry | Limit | 8-slot ANCS usage | Spare |
|---|---|---|---|
| `CFG_GATT_MAX_CLIENT_CHARS`   | 40 | 8 × 3 = **24** | 16 |
| `CFG_GATT_MAX_CLIENT_SERVICE` | 20 | **8**          | 12 |
| `CFG_GATT_MAX_SERVER_CHARS`   | 40 | 10 (MODEM svc + Phase B fb5905) | 30 |
| `CFG_GATT_MAX_SERVER_SERVICE` | 20 | 1 | 19 |

Client and server registries are separate. No overflow at 8.
`BLE_MAX_CONNECTION = 20` (S140 ceiling); we use 8.

## 5. RAM — already proven; keep DEFAULT bandwidth [verified + confirm on HW]

The SoftDevice RAM requirement is fixed at `sd_ble_enable()` by role/conn counts +
bandwidth, **not** by GATT-client usage. Today's firmware already calls
`Bluefruit.begin(8)` successfully, and the peripheral GATTC conn-cfg is set
regardless — so **enabling ANCS adds no SoftDevice RAM**.

Added RAM is purely application-side per-slot state:
- allowlist 16 × 32 B = 512 B/slot
- Data Source reassembly buffer ≈ 256 B/slot
- bookkeeping ≈ tens of B
→ **≈ <1 KB/slot × 8 ≈ <8 KB** application RAM. Trivial on the nRF52840's 256 KB.

**Hard rule:** do **NOT** call `configPrphBandwidth(BANDWIDTH_MAX)`. The stock ANCS
example does it for *one* link (MTU 247, event-len 100); at 8 links the SoftDevice
RAM request overruns the app region and `sd_ble_enable()` fails. Default bandwidth
(MTU 23) is correct — it also forces the multi-packet Data Source reassembly that
Phase A must implement anyway.

## 6. Bonding on a peripheral link — works [verified in source / confirm on HW]

`conn->requestPairing()` from the peripheral sends an SMP Security Request; iOS
shows the PAIR prompt and initiates pairing. `central_sec_count = 0` is irrelevant
(that governs the device acting as central). Bluefruit persists bond keys in flash,
so reconnects resume ANCS silently — which is exactly what Phase C wants (we never
remove iOS bonding).

---

## What the spike empirically confirms — **[confirm on HW]**

Flash `spike_ancs_multi.ino`, bond 2–3 iPhones, capture serial (115200). Expect:

1. Boot: `MODEM_ANCS advertising; NUM_SLOTS=8`, **no** `sd_ble_enable`/RAM error.
2. Per phone: `CONNECT … -> slot=N`, then after PAIR: `SECURED`, `ANCS discovered`,
   `notifications enabled`.
3. Trigger a notification on phone A → exactly one `slot=A conn=… evt=Added uid=…`
   line followed by `slot=A appid="com.…"`. Phone B's notifications log only
   `slot=B`. **No line attributes A's notification to B's slot.**
4. `live links=` reflects the number actually held simultaneously.

### Numbers to report back (the Phase 0 gate)
- **Max simultaneous bonded ANCS links achieved** (target 2–3; array holds 8).
- **RAM headroom** — any boot error, or clean enable at default bandwidth.
- **SoftDevice limits hit** — pairing failures, discovery failures, dropped links.

**Do not proceed to Phase A/B/C until #3 (no cross-talk) is observed on hardware.**

---

## Implications carried into Phase A (so the design isn't re-derived later)
- Production `ancs_client` will **not** reuse stock `BLEAncs`'s blocking
  `getAttribute()` or its conn-handle-less callback. It will use
  `BLEClientService`/`BLEClientCharacteristic` directly with a notify callback that
  reads `chr->connHandle()` → resolves the slot, and a **non-blocking** per-slot
  attribute-fetch state machine with fixed reassembly buffers (spec: no blocking in
  callbacks, no dynamic allocation in hot paths).
- connHandle is the only key; never assume `connHandle == slotIndex`.
- Keep default bandwidth; reassembly across ~20-byte Data Source packets is
  mandatory, not optional.
