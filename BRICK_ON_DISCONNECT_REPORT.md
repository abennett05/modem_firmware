# Brick-on-disconnect — investigation findings

**Mode:** investigation only. No firmware was changed. All remediations are proposals.
**Repo state:** `modem_firmware/` is a git repo (history below); the parent
`ENGR6610` workspace is not. **Core:** Adafruit nRF52 **1.6.1** (pinned by
`sketch.yaml`), resolved at
`~/Library/Arduino15/internal/adafruit_nrf52_1.6.1_eb8f495d60cc4aad`. FreeRTOS is
in use (Adafruit core default).

Paths to core files below are abbreviated `CORE/` =
`…/adafruit_nrf52_1.6.1_…/cores/nRF5` and `BSP/` =
`…/libraries/Bluefruit52Lib/src`.

---

## Phase 0 — Codebase reconnaissance

### 0.1 Disconnect path entry points

| Item | Location |
|---|---|
| Registration | `modem_firmware.ino:1334` `Bluefruit.Periph.setDisconnectCallback(disconnectCallback)` |
| Handler | `modem_firmware.ino:815` `disconnectCallback(uint16_t conn_handle, uint8_t reason)` |
| Dispatch task | `BSP/bluefruit.cpp:849` — `BLE_GAP_EVT_DISCONNECTED` does `ada_callback(NULL, 0, Periph._disconnect_cb, conn_hdl, reason)`. **The callback does NOT run in the SoftDevice/BLE event task — it is queued onto the "Callback" (Ada-callback) task.** Same for connect (`bluefruit.cpp:829`). |

Functions called transitively from `disconnectCallback` (one level + key deeper calls):

- `ancsOnDisconnect(conn_handle)` → `slotByHandle` → `AncsSlot::reset()` (`ancs_client.cpp:484,138,81`). Pure memory clear, no SoftDevice/flash/blocking.
- `findUser(conn_handle)` (`modem_firmware.ino:613`).
- `takeIntentionalLeave(conn_handle)` (`:376`).
- **Intentional-leave branch** → `finalizeLeave(conn_handle)` (`:713`) → `colorOrFallback`, sets `g_disconnectFx`/`g_chimeLeave` flags, `removeUser` (`:697`, may call `endSession` `:635` → `resetPickupSpikeState`), **`notifyUserCount` (`:482`)**, **`notifyRoster` (`:527`)**, multiple `Serial.print*`.
- **Spontaneous-leave branch** → only sets `u->pendingLeaveMS = millis()` + `Serial.print*`. The heavy `finalizeLeave` runs later from `loop()` (task context), not here.

Notify behavior in-callback: `notifyUserCount`/`notifyRoster` call `…Char.write()`/`…Char.notify()` for *count* (1 byte) and write the readable roster value, then set `g_rosterDirty` so the **multi-packet** roster send happens in `loop()` (`:550 pushRosterPaged`, `:1526`). The deliberate design keeps blocking HVN sends out of callbacks — that part is sound.

### 0.2 Per-connection / per-slot state (all static, all bounded)

| Structure | Where | Size | Freed/zeroed on disconnect |
|---|---|---|---|
| `Session g_session` (`User connectedUsers[MAX_USERS]`) | `:323`, type `modem_types.h:84` | `User` ≈ 44 B × 8 | `removeUser` packs array, `--userCount` (`:697`). Only on **finalize**, not on the spontaneous-drop callback. |
| `AncsSlot g_slots[MAX_USERS]` | `ancs_client.cpp:135` | per slot ≈ 96 B rx + 16×32 B allow + rings ≈ 700 B; ×8 | `ancsOnDisconnect`→`reset()` zeroes flags/buffers, keyed by exact `connHandle`, idempotent (`:484`). **This is the per-slot ANCS client; it has landed.** |
| `g_intentionalLeave[MAX_USERS]` | `:362` | 8×2 B | swap-with-last removal in `takeIntentionalLeave` (`:376`). |
| `g_discQueue[DISC_Q_SIZE]` (SPSC ring) | `:345`, size `MAX_USERS+1` | 9×2 B | head/tail; drained in `loop()` (`:1514`). |
| `g_pickupWin[PICKUP_EVENT_CAP]` | `:1126` | 32×8 B | reset on `endSession`. |

No array is indexed by raw `connHandle`; all lookups are linear scans bounded by `userCount`/`MAX_USERS`. **No out-of-bounds candidate found in the firmware's own disconnect path.** ANCS teardown is wired and exact-handle matched.

### 0.3 Roster / leaderboard chunking buffers

- `notifyRoster` (`:527`): local **stack** buffer `uint8_t buf[1 + MAX_USERS*(3+2+1+MAX_USER_NAME_LEN)]` = `1 + 8*30` = **241 B on the stack**. Runs inside callbacks (incl. `disconnectCallback`→`finalizeLeave`).
- `pushRosterPaged` (`:550`): local **stack** `uint8_t pkt[ROSTER_PKT_HEADER + MAX_USER_NAME_LEN]` = 32 B. Runs only in `loop()`.
- Neither is heap; neither persists; both bounded. Lifecycle on disconnect: re-emitted, nothing to free.

### 0.4 Heap / stack / SoftDevice RAM configuration

Linker `CORE/linker/nrf52840_s140_v6.ld` + `nrf52_common.ld`:

- `RAM : ORIGIN = 0x20006000, LENGTH = 0x20040000 - 0x20006000` → **app RAM = 232 KB**; **SoftDevice reserves 0x20000000–0x20006000 = 24 KB** (`APP_RAM_BASE = 0x20006000`).
- `__StackTop = ORIGIN(RAM)+LENGTH(RAM) = 0x20040000`; `__StackSize = 1024*2 = 2048` → **`__StackLimit = 0x2003F800`** (this is the **MSP / main stack**, used by ISRs + SoftDevice event entry only).
- `__HeapBase` = end of `.bss`; heap grows **up** (newlib `sbrk`) toward `__StackLimit`. Link-time guard: `ASSERT(__StackLimit >= __HeapLimit …)`.
- `configTOTAL_HEAP_SIZE = 4096` **but** `FreeRTOSConfig.h:58` comments *"not used since we use malloc"* — FreeRTOS dynamic allocation is routed to newlib `malloc`/`rtos_malloc`, i.e. the **single C-library heap** between `__HeapBase` and `__StackLimit`. (`configSUPPORT_DYNAMIC_ALLOCATION=1`, `configUSE_MALLOC_FAILED_HOOK=1`, `configCHECK_FOR_STACK_OVERFLOW=1`, `FreeRTOSConfig.h:73,78,79`.)

**FreeRTOS tasks (stack depth in *words*; ×4 = bytes; all allocated *from the heap*):**

| Task | Stack | Created | Runs |
|---|---|---|---|
| `loop` | `LOOP_STACK_SZ = 256*4` = 1024 w = **4 KB** | `CORE/main.cpp:88` | `loop()` |
| `BLE` | `CFG_BLE_TASK_STACKSIZE = 256*5` = 1280 w = **5 KB** | `BSP/bluefruit.cpp:473` | `_ble_handler` (`bluefruit.cpp:778`) + ANCS notify cbs (registered `useAdaCallback=false`, `ancs_client.cpp:405-406`) |
| **`Callback`** | **`CALLBACK_STACK_SZ = 256*3` = 768 w = 3 KB** | `CORE/main.cpp:91` `ada_callback_init` | **connect/disconnect cbs, all GATT write cbs, secured cb, bond-flash saves** |
| `SOC` | `200` w = 0.8 KB | `bluefruit.cpp:480` | power/soc events |
| idle / timer | `configMINIMAL_STACK_SIZE=100` w / `configTIMER_TASK_STACK_DEPTH` | `rtos.cpp:105,129` | — |

### 0.5 Dynamic-allocation surface

- **Firmware code (`modem_firmware.ino`, `ancs_client.cpp`, `modem_audio.cpp`, `puck_settings.cpp`): zero `malloc`/`calloc`/`new`/`realloc`/`strdup`, zero Arduino `String`.** All state is static or stack. `parseRGB` uses a stack `char buf` + `sscanf` (`:389,932`).
- **Hidden allocators on the disconnect path, inside the core:**
  - **`ada_callback` mallocs one `ada_callback_t` per dispatched callback** and frees it after the call (`CORE/utility/AdaCallback.c:101-138, adafruit_callback_task` `:44-78`). Every connect, disconnect, GATT write, secured event = one heap alloc/free on the Callback task.
  - **`ada_callback_queue` *grows the queue by doubling* when full** (`AdaCallback.c:80-99` → `ada_callback_queue_resize(2*_cb_qdepth)` `:155`). Initial depth `INITIAL_QUEUE_DEPTH = 64` (`:39`); each resize `xQueueCreate`s a larger queue from the heap. **There is no shrink and no upper bound.**
  - **Bonding/system-attribute saves to flash are deferred via `ada_callback`** → `bond_save_keys_dfr` / `bond_save_cccd_dfr` (`BSP/utility/bonding.cpp:138,262`), executed on the **same 3 KB Callback task**, calling `Adafruit_LittleFS` (`puck_settings.cpp` uses the same `InternalFS`).
  - `puckSettingsSave()` → `InternalFS` LittleFS write (`puck_settings.cpp:38-53`), reached from owner/name/sound/brightness GATT write callbacks — also on the Callback task.

### 0.6 Logging in the disconnect path

`disconnectCallback` + `finalizeLeave` + `ancsOnDisconnect` emit ~6–10 `Serial.print*` calls (`:827,833,841-844,722,490`). All use fixed format args / bounded strings (`u->name` is NUL-terminated, capped `MAX_USER_NAME_LEN`). **No format-string or buffer-overrun risk.** `Serial` is `begin()`-d in `setup()` (`:1284`). Serial/TinyUSB CDC TX uses its own buffering; a print flood is a latency cost on the Callback task, not an overflow — but see Phase 1 H1 (it *slows the drain*).

### 0.7 Audio coupling

**The leave chime is NOT played from the disconnect handler.** `finalizeLeave` only raises `g_chimeLeave` (`:721`); `loop()` calls `chimeLeave()`→`playChime` in task context (`:1622`). `playChime`/`audioStop` touch EasyDMA only from `loop()` and the I2S IRQ (`modem_audio.cpp:94-105,145-185`). **The "EasyDMA repointed mid-disconnect" hypothesis is ruled out** — no audio call is reachable from the disconnect callback. (Separately, `audioStop` has a bounded 200 000-iteration spin `:99`, on the loop task, that bails out — not disconnect-related.)

### 0.8 Recent changes (`git log`, `modem_firmware/.git`)

```
0481cf8 Fixed leaderboard system   (ino +213 — roster paging / chunking)
ff6a806 Speaker Integration        (modem_audio.* + chimes; ino +112)
ebe2b83 Pickup Detection           (ino +251)
2abcb41 Ownership System Added      (ino +295; puck_settings.* NEW — adds flash writes)
16a40ac ANCS fixes                 (ino +174; ancs_client.*)
001833c ANCS connections
```

All six touch areas in scope and all land within the last ~2 weeks. The ones that
**add load to the 3 KB Callback task / the shared heap** are the prime suspects:
`2abcb41` (introduced `puckSettingsSave` flash writes on the Callback task),
`16a40ac`/`001833c` (per-connection ANCS + pairing on every connect), and
`0481cf8` (more `notifyRoster` calls → more in-callback work).

---

## Phase 1 — Failure-mode characterization

### Characterizing "bricked" first (acceptance criterion)

`vApplicationStackOverflowHook` and `vApplicationMallocFailedHook` are
**`while(CFG_DEBUG) yield();`** (`CORE/rtos.cpp:84-95`), and the production build
sets **`-DCFG_DEBUG=0`** (`platform.txt:58`). So on the shipped build **the hooks
return immediately** — a stack overflow or a failed `malloc` does **not** halt
cleanly; execution continues over corrupted memory → almost certain **HardFault**.
The HardFault handler resolves to a `while(1)` spin (`CORE/hooks.c:56` family),
i.e. the device **hangs until power-cycle/reset.**

This is a **soft hard-hang**, *not* an APPROTECT lockout: nothing here writes
`UICR.APPROTECT` or the bootloader. **SWD/double-tap-reset DFU still work; the
board is fully re-flashable.** "Bricked" = dead-until-reset / reflash, recoverable.
Remediations should target the hang, not unlock procedures.

### Ranked candidates

**H1 — Ada-callback heap exhaustion (queue auto-resize) → `MallocFailedHook` → hang. — HIGH.**
Grounded in 0.4/0.5: the disconnect callback, connect callback, every GATT write
callback, the secured callback, **and the bonding flash save** all execute on the
single 3 KB **Callback** task, and each is enqueued via a per-item `rtos_malloc`.
When that task stalls — a LittleFS bond/`puckSettings` flash write (0.5), or the
in-`loop` blocking `disableNotify`, or a Serial flood (0.6) — and iOS produces a
**burst** of connect/disconnect/secured/phantom-reconnect callbacks (the exact
multi-connection churn already documented in `MULTICONN_REPORT.md` / memory
`modem-ios-multiconnection-bugs`), `ada_callback_queue` fills and **doubles the
queue (64→128→256…) with no bound and no shrink** (`AdaCallback.c:155`). Repeated
bursts ratchet the heap up until `rtos_malloc` fails. The disconnect event itself
*triggers a bond CCCD/system-attribute flash save on the very task that must stay
drained*, so a disconnect is precisely the trigger.
*Evidence to confirm:* `util_heap_get_free_size()` / `mallinfo().fordblks`
snapshot each heartbeat and on connect/disconnect; watch it ratchet down across
connect/disconnect cycles and never recover; log `_cb_qdepth` growth.
*Matches symptom?* **Yes** — terminal, dead-until-reset, and "memory overflow on
disconnect" is a literal description.

**H2 — `Callback` task (3 KB) stack overflow on the disconnect/connect path. — MEDIUM-HIGH.**
The 3 KB Callback stack is the smallest task stack carrying real work, and it runs
the deepest call chains in the system: `disconnectCallback`→`finalizeLeave`→
`notifyRoster` (**241 B stack buffer**, 0.3) → SoftDevice GATTS write, *and*
(separately, same task) the **LittleFS bond/`puckSettings` write**, whose
LittleFS read/prog/cache path is itself stack-hungry. `configCHECK_FOR_STACK_OVERFLOW=1`
exists, but its hook returns on `CFG_DEBUG=0` (above) → corruption → HardFault.
*Evidence to confirm:* `uxTaskGetStackHighWaterMark` for the `"Callback"` task
logged each heartbeat; a value approaching 0 after a disconnect+flash event
confirms. *Matches symptom?* **Yes**, same dead-until-reset signature. Ranked
just under H1 because the per-call frames *measured* (≈0.6–1 KB) leave some margin
unless a LittleFS write coincides.

**H3 — SoftDevice RAM / event exhaustion (`NRF_ERROR_NO_MEM` swallowed). — MEDIUM.**
`setup()` itself documents that app-RAM base `0x20006000` (24 KB SD reservation)
is "**nearly full at 8 links + this attr table**" (`:1306-1330`). Under churn,
`Bluefruit.Advertising.start(0)` on every connect (`:810`) + `requestPairing` on
every connect (`ancs_client.cpp:456`) + 8 links' HVN/Write queues can hit
`NO_MEM`/`NO_TX_PACKETS` from `sd_ble_*`. Most return codes here are **not
checked** (`notify`/`write` results ignored). *Evidence:* assert/log every
`sd_ble_*` and `Bluefruit.*` return on the disconnect/advertise/notify paths.
*Matches symptom?* **Partially** — usually "BLE silently stops / unconnectable"
rather than a CPU hang; could co-trigger H1 by backing callbacks up.

**H4 — Advertising-restart / pairing storm after disconnect. — LOW-MEDIUM.**
`restartOnDisconnect(true)` (`:1485`) + the explicit `Advertising.start(0)` in
`connectCallback` + per-connect `requestPairing` create a tight reconnect loop
under iOS phantom churn. Not itself terminal, but it is the **burst generator**
that feeds H1/H3. *Evidence:* count connect/disconnect/secured events per minute
in the heartbeat.

**H5 — Heap leak from a per-connection allocation never freed. — LOW.**
The firmware allocates nothing per connection (0.5). Any leak would be inside the
core/LittleFS, which is H1's mechanism rather than a separate firmware bug.

**H6 — Bond/flash write colliding with another flash op. — LOW.**
Bond saves and `puckSettingsSave` both run on the Callback task and are therefore
**serialized, not concurrent** (0.5), so direct reentrancy corruption is unlikely.
The risk they pose is *latency* (stalling the drain → feeding H1/H2), not a race.

**H7 — Use-after-free on a chunking/notification buffer freed mid-flight. — LOW.**
Roster buffers are stack-local and re-derived each send (0.3); ANCS Data-Source
reassembly is guarded by `!s.inFlight` after `reset()` (`ancs_client.cpp:236,497`).
No freed-buffer reference survives disconnect in firmware code.

**H8 — Watchdog starvation. — LOW.** No WDT is started in `setup()`; no
`feed`/`NRF_WDT` use anywhere in firmware. Not applicable.

**H9 — OOB write on a per-slot array via reused/stale `connHandle`. — LOW.**
All slot/user access is exact-handle-matched and `userCount`/`MAX_USERS`-bounded
(0.2). No index derived from a raw handle.

**H10 — Serial/log buffer overrun. — LOW (but contributes to H1/H2).**
Bounded args (0.6); the only real effect is slowing the Callback-task drain.

---

## Phase 2 — Reproduction & instrumentation plan (no code changes)

### 2.1 Minimal repro

Most likely to trigger, in order:
1. **Sustained connect/disconnect churn with bonding active** — pair a phone, then
   force repeated drops (toggle iPhone Bluetooth, or walk in/out of range)
   **dozens of times**, watching for a *cumulative* failure rather than a single
   event. H1 is cumulative; a one-shot disconnect won't show it.
2. **App-requested ("intentional") leave at 7–8 users**, because that path is the
   one that runs `finalizeLeave`→`notifyRoster` (the 241 B buffer + most
   SoftDevice work) *inside* the disconnect callback (H2), and at full roster the
   buffer/SD-queue pressure is maximal.
3. **A claim/rename/brightness write (flash) immediately followed by a disconnect**
   — co-schedules a `puckSettings` flash write and the bond save on the Callback
   task, maximizing both stall (H1) and stack depth (H2).
4. **Abrupt vs. graceful:** abrupt range-out drops generate more phantom
   reconnect + re-pair callbacks per drop than a clean app leave, so prefer abrupt.

### 2.2 Instrumentation (described, not applied) — confirm H1 then H2

Add to the existing `dumpState`/heartbeat (`:749`, every 5 s) and to
connect/disconnect entry/exit:
1. **Heap free trend (H1, cheapest, do first):** log `util_heap_get_free_size()`
   (core helper, `common_func.h`) — or `mallinfo().fordblks` — every heartbeat and
   at connect+disconnect. Also log the Ada queue depth `_cb_qdepth` (expose/read
   via a tiny accessor). **Confirmation:** free heap ratchets down across cycles
   and never recovers; `_cb_qdepth` steps 64→128→256….
2. **Per-task stack high-water (H2):** `uxTaskGetStackHighWaterMark(NULL)` for
   `"Callback"`, `"BLE"`, `"loop"` (handles via `xTaskGetHandle`). **Confirmation:**
   the `"Callback"` watermark trends toward 0 right after a disconnect that
   coincides with a flash write.
3. **GPIO scope markers:** toggle a spare GPIO high at `disconnectCallback` entry,
   low at exit, and a second pin around the LittleFS write, to measure how long the
   Callback task is held off the queue (the stall window that feeds H1).
4. **SoftDevice return-code asserts (H3):** wrap `Bluefruit.disconnect`,
   `Advertising.start`, and the `notify`/`write` calls; log any non-`NRF_SUCCESS`.
5. **Event-rate counters (H4):** per-minute connect/disconnect/secured tallies in
   the heartbeat.
6. Optional: temporarily build with `-DCFG_DEBUG=1` so the overflow/malloc hooks
   **stop and stay put** (instead of returning into corruption), making the failing
   task name printable rather than a blind HardFault.

### 2.3 What success looks like

- **H1 confirmed** if free heap declines monotonically across connect/disconnect
  cycles (and `_cb_qdepth` grows), terminating in a `MallocFailedHook`/HardFault at
  the moment of a disconnect.
- **H2 confirmed** if the `"Callback"` stack high-water hits ~0 (and, under
  `CFG_DEBUG=1`, `StackOverflowHook` names `"Callback"`) on a disconnect+flash
  coincidence.
- Either gives a single, reproducible "it's hypothesis N" with a named task and a
  trending counter — not an inference.

---

## Phase 3 — Remediation options (sketch only; no patches)

Ranked to match the hypotheses. Several touch areas **outside** the disconnect
handler — flagged so scope can be debated before any implementation prompt.

**R1 — Stop the disconnect path from stalling the Callback task (targets H1/H2 root, in-scope).**
Move the work the disconnect callback shares the Callback task with *off* that
task: defer **all** flash writes (`puckSettingsSave`, and ideally bond saves) and
any potentially-blocking client-CCCD work to `loop()` via the existing flag/queue
pattern this file already uses (`g_rosterDirty`, `g_discQueue`). Concretely, set a
"settings dirty" flag in the owner/name/sound/brightness write callbacks and call
`puckSettingsSave()` once from `loop()`. *Change X:* `onOwnerWrite`/`onNameWrite`/
`onSoundWrite`/`onBrightnessWrite` (`ino:998-1074`) raise a flag; `loop()` drains
it. *Risk:* a rename/claim that races a power-loss before the next `loop()` isn't
persisted (small, acceptable); must debounce. *Note:* does **not** touch the
disconnect handler itself — it relieves the *shared task*, which is the actual
coupling. Bond-save deferral is a core behavior and may not be overridable without
deeper changes — investigate before promising.

**R2 — Cap / pre-size the Ada-callback queue, or throttle the churn that floods it (targets H1, partially out-of-scope).**
Two sub-options: (a) reduce the *rate* of enqueued callbacks on the firmware side —
e.g. don't `requestPairing` on every phantom reconnect, and gate
`Advertising.start`/`restartOnDisconnect` so iOS churn doesn't loop (touches
`connectCallback`/`setup`, **outside** the disconnect path); (b) raise
`INITIAL_QUEUE_DEPTH` / bound the resize in the *core* (out-of-scope: edits a
vendored core file — list explicitly, likely rejected). Prefer (a). *Risk:*
changing advertising/pairing cadence can affect reconnect latency and the
documented iOS phantom-link handling — must be validated against the existing
multi-connection fixes, not done blind.

**R3 — Enlarge the Callback task stack (targets H2, mitigation not fix, out-of-scope).**
`CALLBACK_STACK_SZ` is `CORE/main.cpp` (vendored core). Bumping 3 KB→6 KB buys
margin for the `notifyRoster`+LittleFS depth but does not stop H1's heap growth and
edits a core file. Treat as a stop-gap only if H2 is confirmed dominant.

**R4 — Shrink in-callback work on the intentional-leave path (targets H2, in-scope).**
`finalizeLeave` is the only heavy thing the disconnect callback does
(`notifyRoster`'s 241 B buffer + SoftDevice writes). Option: have the intentional
branch, like the spontaneous branch, only *flag* the leave and let `loop()` run
`finalizeLeave` (the machinery already exists for the grace sweep at `:1542`). This
removes the 241 B buffer and all SoftDevice writes from the 3 KB Callback stack.
*Risk:* a few ms extra latency before other phones see the leave — cosmetic. Stays
within the disconnect contract.

**R5 — Check/handle `sd_ble_*` return codes on the disconnect/advertise/notify paths (targets H3, in-scope-ish, defensive).**
Surface `NO_MEM`/`NO_TX_PACKETS` instead of swallowing them, so an SD-RAM ceiling
becomes a loud log rather than a silent wedge. Pure observability + graceful
backoff; low risk. (Note: the prompt lists speculative hardening as out of scope —
include only if H3 is implicated by Phase 2 data.)

---

## Bottom line

- **Confirm H1 first** (Ada-callback heap exhaustion via the auto-doubling queue),
  because it is terminal, cumulative, literally "memory overflow on disconnect,"
  and triggered by the disconnect event itself (bond flash save on the same task
  that must drain the callback queue). **Cheapest probe:** log
  `util_heap_get_free_size()` + `_cb_qdepth` every heartbeat and watch them ratchet
  across connect/disconnect cycles — no behavioral change required.
- **H2** (3 KB Callback-task stack overflow) is the close second and is confirmed
  by the same heartbeat with one extra line (`uxTaskGetStackHighWaterMark` for
  `"Callback"`). Add both probes in one instrumentation pass.
- The board is **hung-until-reset, fully re-flashable — not APPROTECT-locked**;
  remediation is about keeping the 3 KB Callback task drained and the heap flat
  (R1/R4 in-scope, R2 partly out-of-scope), not about recovery/unlock.
