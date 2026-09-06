# New component code: development progress and logic

> **Living document** — Last reviewed: 2026-09-06<br>
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
| Runtime UART parity selection | Partial | Selects even parity for TCC/A0 and no parity for TU2C. Includes the existing ESP8266 UART0 GPIO13 swap path; broader hardware validation is still required. |
| Master-address discovery and validation | Done | Learns the source from the first valid master keepalive or checks it against an explicitly configured address. |
| Frame logging | Done | Logs every complete candidate, highlights addresses and command/type fields, and marks checksum failures. |
| Diagnostic history | Done | Publishes a newline-separated, de-duplicated event history capped at 255 characters. |
| Manual rediscovery | Done | The diagnostic reset button clears discovery state, reader state, counters, and history, then restarts scanning. |
| Climate API capability declaration | Partial | Air systems advertise the intended climate modes, fan modes, swing modes, presets, current temperature, and action. Water systems currently expose only off mode. These are API declarations, not working controls. |
| Climate state decoding | Not started | Valid non-keepalive frames are logged and then discarded. State parsing and publication must be rebuilt. |
| Command generation/transmission | Not started | `control()` deliberately ignores calls. Address assignment, command queues, retries, and bus timing still need implementation. |
| Connection/liveness tracking | Not started | A keepalive confirms discovery, but no ongoing online/offline state or timeout is currently published. |
| Automated parser tests | Not started | Add captured-frame fixtures and tests for valid frames, bad checksums, truncation, noise recovery, discovery timing, and address mismatch behavior. |
| On-device protocol validation | Partial | The reader logic exists; each protocol and supported hardware path still needs results recorded from representative installations. |

## Runtime logic

### 1. Startup and reset

`setup()` initializes the boot timestamp, copies the configured master address,
sets a safe initial climate state (`off`, target `22 °C`), and selects the first
reader. In automatic mode the first reader is TCC. With a fixed protocol, only
that protocol is selected.

The reset button calls the same discovery initialization intentionally, with
additional cleanup: protocol/master confirmation flags, frame-reader buffers,
the resynchronization counter, and diagnostic history are cleared.

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

TCC has no start marker, so the reader maintains a sliding candidate buffer.
Once four bytes are available, byte 3 gives the payload length and the expected
complete buffer size is `length + 5`. The length must be at least 2 and the
complete frame must fit in the 132-byte buffer. The final byte is compared with
the XOR of all prior frame bytes.

- Valid candidate: process it and remove the complete frame from the buffer.
- Invalid length or checksum: discard only the oldest byte, increment the
  resynchronization count, and evaluate the shifted candidate.

Discarding one byte rather than the entire buffer allows the reader to recover
when noise or a truncated frame appears immediately before a valid frame.

#### A0

The A0 reader waits for the unambiguous `A0:00` wrapper. Byte 3 contains the body
length, making the complete size `length + 6` (wrapper, type, length, body, and
two CRC bytes). Sizes below 8 or above 132 are rejected. The last two bytes are
read as a big-endian received CRC and compared with CRC-16/MCRF4XX calculated
over every preceding byte.

#### TU2C

The TU2C reader waits for `F0:F0`. Byte 2 contains the total frame size,
including both `F0` bytes and the final `A0`. A valid size is 7–132 bytes. The
penultimate byte must equal the 8-bit sum of bytes from the length byte through
the byte before the checksum, and the last byte must be `A0`.

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

After confirmation, a matching signature from a different source is not
treated as the master keepalive. This preserves the confirmed master identity
and leaves room for separate remote-controller keepalive handling later.

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

### 6. Climate behavior during this phase

For an air system, the entity advertises the intended modes and controls so API
clients can retain the eventual entity schema while development continues. For
a water system, only off mode is currently advertised. The visual temperature
range is 16–32 °C in 0.5 °C increments for both.

These capabilities must not be interpreted as functional support yet:

- received status frames do not update climate state;
- climate calls are logged at verbose level and ignored; and
- no bytes are transmitted by the new component.

## Configuration surface implemented by the new code

| Option/entity | Default | Purpose |
| --- | --- | --- |
| `format` | `auto` | Select `auto`, `tcc`, `a0`, or `tu2c`. |
| `system_type` | `air` | Select the currently advertised air or water climate traits. |
| `master_address` | `auto` (`0xAA` internally) | Learn the master or require an explicit 8-bit address. |
| `esp_address` | `auto` (`0xAA` internally) | Stores the future local address; it is not yet used to transmit. |
| `diagnostic` | `Toshiba AB Diagnostic` | Text sensor containing recent discovery events. |
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
5. **Implement local-address selection and collision avoidance** before enabling
   any write path.
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
