# New component code: development progress and logic

> **Living document** — Last reviewed: 2026-10-07<br>
> Update this page whenever a feature is implemented, its behavior changes, or
> a test result changes. The status described here applies to the new,
> identification-focused implementation in `components/toshiba_ab/` and not to
> features advertised by older releases.

## Goal of the rewrite

The new code is rebuilding the Toshiba AB component around a small, reliable
receive and identification pipeline before control and full message decoding
are restored. Its current job is to:

1. read raw traffic without becoming permanently desynchronized;
2. identify the wire protocol from a valid master keepalive;
3. identify or verify the master address;
4. expose useful frame and discovery diagnostics; and
5. retain a valid ESPHome climate entity shape while the functional climate
   behavior is added incrementally.

The implementation is currently **identification-only**. It observes frames and
reports discovery results, but it does not decode climate state and does not
transmit climate commands.

## Progress snapshot

Status meanings:

- **Done**: present in the current implementation.
- **Partial**: present, but intentionally limited or awaiting real-system
  validation.
- **Not started**: no functional implementation exists in the new code yet.

| Area | Status | Current behavior / next work |
| --- | --- | --- |
| ESPHome configuration and code generation | Done | Registers the climate component, UART device, diagnostic text sensor, and reset button. Accepts protocol, system type, and explicit or automatic addresses. |
| TCC frame collection | Done | Uses the length byte and XOR checksum. On an invalid candidate it advances by one byte and tries to synchronize again. |
| A0 frame collection | Done | Synchronizes on `A0:00`, obtains the body length from the header, and validates CRC-16/MCRF4XX. |
| TU2C frame collection | Done | Synchronizes on `F0:F0`, uses the encoded total length, requires the trailing `A0`, and validates the 8-bit additive checksum. |
| Incomplete-frame recovery | Done | Drops a partial frame after a 25 ms inter-byte timeout and records reader resynchronizations. |
| Automatic protocol discovery | Done | Scans TCC, A0, and TU2C for 20 seconds each and confirms a protocol only from a checksum-valid master keepalive. |
| Runtime UART parity selection | Partial | Selects even parity for TCC/A0 and no parity for TU2C. Includes the existing ESP8266 UART0 GPIO13 swap path; broader hardware validation is still required. UART settings reload is guarded to ESP8266/ESP32, matching the ESPHome API and permitting host builds. |
| Master-address discovery and validation | Done | Learns the source from the first valid master keepalive or checks it against an explicitly configured address. |
| Existing remote discovery | Partial | After protocol and master confirmation, known checksum-valid remote pings maintain a live address inventory. Addresses expire after five minutes without a ping and drive automatic ESP address selection. |
| ESP address selection | Done | Auto mode selects the lowest free protocol/system candidate, moves on remote collisions, and reclaims lower expired addresses. Explicit mode retains its address and reports a collision once per component lifetime. Bus registration/transmission remains unimplemented. |
| Frame logging | Done | Logs every complete candidate, highlights addresses and command/type fields, and marks checksum failures. |
| Diagnostic sensor | Done | Publishes the latest discovery event; earlier states remain available through Home Assistant history. |
| Manual rediscovery | Done | The diagnostic reset button clears discovery state, reader state, and counters, then restarts scanning. |
| Climate API capability declaration | Partial | Air systems advertise the intended climate modes, fan modes, swing modes, presets, current temperature, and action. Water systems currently expose only off mode. These are API declarations, not working controls. |
| Climate state decoding | Not started | Valid non-keepalive frames are logged and then discarded. State parsing and publication must be rebuilt. |
| Command generation/transmission | Not started | `control()` deliberately ignores calls. Bus registration, command queues, retries, and bus timing still need implementation. |
| Connection/liveness tracking | Not started | A keepalive confirms discovery, but no ongoing online/offline state or timeout is currently published. |
| Automated parser tests | Partial | Host tests feed synthetic checksum-valid master/remote frames through the real readers and cover address selection, collisions, expiration, reset, checksum rejection, and master mismatch. Captured frames, truncation/noise, and discovery timing still need broader coverage. |
| On-device protocol validation | Partial | The reader logic exists; each protocol and supported hardware path still needs results recorded from representative installations. |

## Runtime logic

### 1. Startup and reset

`setup()` initializes the boot timestamp, copies the configured master address,
sets a safe initial climate state (`off`, target `22 °C`), and selects the first
reader. In automatic mode the first reader is TCC. With a fixed protocol, only
that protocol is selected.

The reset button calls the same discovery initialization intentionally, with
additional cleanup: protocol/master confirmation flags, frame-reader buffers,
and the resynchronization counter are cleared.

### 2. Protocol scan schedule

When `format: auto` is configured, discovery follows this fixed schedule:

| Time after startup/reset | Reader | UART parity |
| --- | --- | --- |
| 0–20 seconds | TCC | Even (8E1 at the configured 2400 baud) |
| 20–40 seconds | A0 | Even (8E1) |
| 40–60 seconds | TU2C | None (8N1) |
| After 60 seconds | Discovery stops | Last selected parity remains active |

Finding a valid master keepalive ends the scan immediately. If no keepalive is
found, the diagnostic sensor records that discovery ended and includes the
number of reader resynchronizations. Discovery does not automatically loop; use
the reset button to start another scan.

### 3. Byte collection and recovery

The main loop first updates discovery, checks for an incomplete-frame timeout,
and then drains all currently available UART bytes into the active reader.

#### TCC

TCC begins immediately with `SRC:DST:OPCODE:LEN`; it has no reserved byte or
byte sequence whose only framing purpose is to announce "a frame starts here."
In other words, source address `0x40` in a captured frame is data, not a sync
marker, and another `0x40` could legally occur elsewhere in a frame. The reader
therefore cannot jump directly to a known boundary after corruption and instead
maintains a sliding candidate buffer. Once four bytes are available, byte 3
gives the payload length and the expected complete buffer size is `length + 5`.
The length must be at least 2 and the complete frame must fit in the 132-byte
buffer. The final byte is compared with the XOR of all prior frame bytes.

- Valid candidate: process it and remove the complete frame from the buffer.
- Invalid length or checksum: discard only the oldest byte, increment the
  resynchronization count, and evaluate the shifted candidate.

When the reader is waiting for the first byte of a new frame, that byte is the
source address. A value above `0xA0` cannot identify a valid TCC participant and
is treated as probable line noise: the byte is discarded immediately, reader
state is reset, and collection remains at the source-byte position. The filter
therefore does not wait for the rest of a noise-derived candidate, and it does
not reject values above `0xA0` while they occupy payload positions in a frame
whose source was already accepted. If checksum recovery shifts a byte into the
source position and that byte is invalid, the accumulated candidate buffer is
reset by the same rule. The filter applies both during automatic TCC discovery
and while TCC is explicitly selected.

Discarding one byte rather than the entire buffer allows the reader to recover
when noise or a truncated frame appears immediately before a valid frame.

#### A0

The A0 reader waits for the unambiguous `A0:00` wrapper. Byte 3 contains the body
length, making the complete size `length + 6` (wrapper, opcode, length, body, and
two CRC bytes). Sizes below 8 or above 132 are rejected. The last two bytes are
read as a big-endian received CRC and compared with CRC-16/MCRF4XX calculated
over every preceding byte. After either a valid or checksum-invalid complete
candidate, the reader discards that whole candidate and waits for the next
`A0:00` wrapper; it does not reinterpret each overlapping suffix as a frame.

#### TU2C

The TU2C reader waits for `F0:F0`. Byte 2 contains the total frame size,
including both `F0` bytes and the final `A0`. A valid size is 7–132 bytes. The
penultimate byte must equal the 8-bit sum of bytes from the length byte through
the byte before the checksum, and the last byte must be `A0`. As with A0, after
a complete candidate the reader resets to searching for the next `F0:F0`
prefix rather than sliding one byte at a time through the failed candidate.

Consequently, the deterministic cascade seen in the TCC log cannot be produced
by the A0 or TU2C recovery paths: one checksum-invalid candidate produces one
failure report. Multiple failures are still possible when multiple frames are
actually damaged, or when corrupted/noisy input contains another apparent
wrapper and forms a second complete candidate. Those are distinct candidates,
not the overlapping suffixes responsible for the TCC cascade.

For every protocol, a partial wrapper or frame is discarded if no next byte is
received for more than 25 ms. Switching scan protocols also resets every reader
so bytes collected under one format cannot leak into the next.

### 4. Frame processing and keepalive identification

Every complete frame candidate is logged. A checksum failure is highlighted and
stops processing. A checksum-valid frame is currently acted upon only when all
three protocol-specific keepalive fields match:

| Protocol | Encoded length | Opcode | Data type | Source byte |
| --- | ---: | ---: | ---: | ---: |
| TCC | `0x02` | `0x10` | `0x8A` | 0 |
| A0 | `0x07` | `0x10` | `0x008A` | 6 |
| TU2C | `0x0A` | `0x00` | `0x3A` | 3 |

These checks deliberately use semantic helper functions rather than scattering
wire offsets and magic values throughout discovery. The exact signature also
prevents an arbitrary TCC frame with opcode `0x10` from being mistaken for the
master keepalive.

Master keepalives and remote pings remain separate frame types: they use
different opcodes and signatures within each protocol and are recognized by
separate predicates. Once both protocol and master are confirmed, the same
logging pipeline identifies these existing remote-controller pings:

| Protocol / system | Remote ping signature | Log description |
| --- | --- | --- |
| TCC | Length `0x07`, opcode `0x15`, payload prefix `08:0C:81` | `remote ping 0xNN` |
| TU2C air | Length `0x0C`, payload prefix `41:5C` | `remote ping 0xNN` |
| TU2C first-generation Estia | Length `0x0C`, payload prefix `E0:41:0C` | `remote ping 0xNN` |
| A0 air and water | Length `0x0C`, opcode `0x15`, data type `0C:81` | `remote ping 0xNN` |

All of these frames are addressed to the master. Classification therefore
requires that the frame destination equal the confirmed master address. The
component keeps a live remote-address inventory internally. A valid ping adds
or refreshes its source address, and an address is removed after five minutes
without another ping. The diagnostic sensor reports `Remote discovered:` and
`Remote removed:` events only when membership changes; routine presence
refreshes do not republish a current-address snapshot. This inventory drives ESP
address selection as described below. As with master keepalive identification, encoded
lengths, opcodes, and data types are held in protocol-value constants and
compared through the common semantic field helpers rather than at raw offsets.

### 5. Confirming protocol and master

When a keepalive is found:

1. If a fixed YAML protocol somehow differs from the received protocol, record
   a diagnostic and reject it.
2. Record and confirm the detected protocol, and stop discovery.
3. If `master_address` is explicit and differs from the source, record the
   mismatch and leave the master unconfirmed.
4. Otherwise, save the source as the master, mark it confirmed, and publish a
   confirmation diagnostic.

Note that the protocol is confirmed before an explicit-address mismatch is
reported. Consequently, the automatic scan stops in that case. This is current
behavior and should be revisited if address mismatch recovery is desired.

### 6. ESP address selection and collisions

After both protocol and master confirmation, `esp_address: auto` selects the
first free candidate in this preference order:

| Protocol | System | Candidate order |
| --- | --- | --- |
| TCC | Air | `0x40, 0x41, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49` |
| TU2C | Air | `0x50, 0x51, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59` |
| TU2C | Water | `0x60` through `0x69` |
| A0 | Water | `0x40` through `0x49`, including `0x41` and `0x42` |

TCC water and A0 air have no established automatic candidate list. They stay
unassigned and publish an unsupported-combination diagnostic; an explicit
address can still be configured. The master address is always excluded from
automatic selection, along with every address in the live remote inventory.

Each recognized, checksum-valid remote ping to the confirmed master updates
that inventory and recalculates the preferred free address. Occupied candidates
are skipped, including addresses above the current selection. When a remote
expires after five minutes without a ping, selection is recalculated after the
whole expiration batch, allowing the ESP to move down to a preferred free address.
Routine pings that do not change the selection do not publish selection events.

If every candidate is occupied, the ESP becomes unassigned (`0xAA` internally)
and publishes one unavailable diagnostic until a candidate becomes free. It
never keeps a known colliding automatic address. Reset clears the inventory and
automatic selection, then waits for a new master confirmation before selecting.

An explicit `esp_address` never changes. A remote ping using that address (or a
confirmed master using it) publishes `ESP address collision:` once per component
lifetime. Repeated pings, expiration/reappearance, and the diagnostic reset button
do not repeat that collision warning; a reboot starts a new component lifetime.

These are local selection decisions only: the component still transmits no
registration, keepalive, or control frames. Maintaining the required first
address on the physical bus needs the future transmit/registration path. Only
recognized pings to the confirmed master populate the remote inventory, so an
unseen or silent participant cannot yet be excluded. Future transmission must
also distinguish locally echoed pings from other participants.

### 7. Climate behavior during this phase

For an air system, the entity advertises the intended modes and controls so API
clients can retain the eventual entity schema while development continues. A
water system creates separate `DHW` and `Zone 1` thermostats by default, with an
optional `Zone 2` thermostat. DHW advertises off/heat and a 45–60 °C range;
both zones advertise off/heat/cool and a 20–65 °C range. All three water
thermostats offer normal, boost, and eco presets and use 0.5 °C steps.

These capabilities must not be interpreted as functional support yet:

- received status frames do not update climate state;
- climate calls are logged at verbose level and ignored; and
- no bytes are transmitted by the new component.

## Configuration surface implemented by the new code

| Option/entity | Default | Purpose |
| --- | --- | --- |
| `format` | `auto` | Select `auto`, `tcc`, `a0`, or `tu2c`. |
| `system_type` | `air` | Select the currently advertised air or water climate traits. |
| `dhw` | `true` | Create the water system's domestic-hot-water thermostat; set to `false` to omit it. |
| `zone_1` | `true` | Create the water system's first heating/cooling thermostat; set to `false` to omit it. |
| `zone_2` | `false` | Create a second heating/cooling thermostat when enabled. |
| `master_address` | `auto` (`0xAA` internally) | Learn the master or require an explicit 8-bit address. |
| `esp_address` | `auto` (`0xAA` internally) | Selects the lowest free candidate in auto mode; explicit addresses stay fixed with a one-time collision warning. Not yet used to transmit. |
| `diagnostic` | `Toshiba AB Diagnostic` | Text sensor containing the latest discovery event. |
| `reset_button` | `Toshiba AB Reset` | Restarts the identification process without rebooting. |

The UART validator requires 2400 baud. It normally requires an RX pin; on
ESP8266, GPIO13 can use the special UART0 swapped-RX path. TX is not required in
this identification-only phase.

## Next development milestones

Work should proceed in small steps that keep receive behavior observable:

1. **Add host-side parser tests** using known captures for all three protocols,
   including noisy and truncated input.
2. **Validate discovery on hardware** and record model, controller, protocol,
   addresses, UART path, and representative redacted frames below.
3. **Add ongoing master liveness**, with an explicit definition of which frames
   refresh it and how disconnect/recovery is reported.
4. **Decode read-only climate state** one protocol at a time and publish only
   fields demonstrated by fixtures and captures.
5. **Validate local-address selection on hardware** and add registration and
   echo handling before enabling any write path.
6. **Build and test command transmission**, including bus-idle timing,
   acknowledgements, retry limits, and failure diagnostics.
7. **Enable `control()` incrementally**, guarding support by protocol and system
   type rather than advertising unverified behavior.
8. **Reconcile public documentation and examples** with the new configuration
   names and only mark a protocol functional after its state and control paths
   are restored.

## Validation log

Append entries instead of replacing older results; this makes regressions and
hardware-specific behavior visible.

| Date | Revision | Environment/system | Protocol/path | Result | Evidence or notes |
| --- | --- | --- | --- | --- | --- |
| 2026-10-07 | ESP address assignment change | Linux host, C++11 with AddressSanitizer/UndefinedBehaviorSanitizer (leak detection disabled for sandbox) | TCC air, TU2C air/water, A0 water | Passed synthetic bus-fixture tests | All candidate ranges, exhaustion/recovery, lower-address reclaim, explicit collision warning once, reset, filtering, master mismatch, and timer wraparound. ESPHome 2026.9.1 generated all four supported combinations and auto/fixed configuration; generated component/main C++ compiled against actual host headers. No live AB-bus validation. |
| 2026-09-06 | Initial tracking document | Source review only | All | Documentation baseline | No new hardware or parser test was performed for this entry. |

## How to update this document

For each development change:

1. change the relevant progress row and its next-work note;
2. update the runtime section if observable logic, constants, byte offsets, or
   sequencing changed;
3. add unresolved behavior to the milestone list rather than implying support;
4. add a validation-log row with the commit/revision and exact test environment;
5. distinguish tests with captured bytes from tests on a live AB bus; and
6. update the **Last reviewed** date at the top.

Keep the description tied to the current source. Protocol background and stable
wire-format reference material belong in `frame_formats.md`; this page should
remain focused on implementation status, decision flow, and verified progress.

