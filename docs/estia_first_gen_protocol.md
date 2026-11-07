# Estia First-Generation (R410A) Protocol Frame Reference

## Overview

The **Estia first-generation protocol** (also called "Estia TU2C" in this codebase) is used by older Toshiba Estia R410A heat pumps. It uses a wrapped TU2C-style frame format with a simple 8-bit checksum.

**Key Characteristics:**
- **UART**: 2400 baud, 8N1 (NO parity)
- **Frame wrapping**: `F0:F0:[frame]:CRC:A0`
- **Checksum**: Simple 8-bit sum of frame bytes (modulo 256)
- **Master address**: 0x70 (default, auto-detectable)
- **Remote address range**: 0x60–0x69 (10 possible remotes)

---

## Frame Structure

All first-generation Estia frames follow this basic layout:

```
[0]            - Total frame length (includes all bytes from [0] to [3+payload+checksum])
[1]            - Source address
[2]            - Destination address
[3]            - Payload marker (usually 0xE0 for commands/status, 0x80 for sensor queries)
[4]            - Opcode/sub-command identifier
[5]            - Sub-opcode or data
[6..N-1]       - Payload data
[N-1]          - Checksum (8-bit sum of bytes [0..N-2])
```

**Checksum Calculation:**
```
checksum = (sum of all bytes from [0] to [N-2]) & 0xFF
```

---

## Frame Types

Frames are identified by examining specific byte positions: typically **[3]**, **[4]**, and **[5]** contain the frame signature.

### Master-Originated Frames

#### 1. **Master Status (0xE0 marker)**
**Signature**: `raw[3] == 0xE0 && raw[5] == 0x31`  
**Length**: ≥ 12 bytes  
**Meaning**: Master reports current operating state including temperatures and flags

**Frame layout:**
```
[0]         - Frame length (0x11 short announcement or 0x15 full status)
[1]         - Source: master address (0x70, or auto-detected)
[2]         - Destination address (remote address, e.g., 0x60)
[3]         - 0xE0 (status marker)
[4]         - 0xC0 announcement or 0xA0 status reply
[5]         - 0x31 (status signature)
[6]         - Operation flags: bit[0]=Zone1 active, bit[1]=DHW active,
              bit[2]=test mode, bits[5..6]=Cool (0x20) / Heat (0x40), bit[7]=DHW available
[7]         - Feature flags: bit[2]=Auto, bit[3]=frost protection, bit[4]=silent,
              bit[5]=night setback enabled, bit[6]=DHW boost, bit[7]=antibacteria enabled
[8]         - Activity flags: bit[0]=heating resistor, bit[1]=heating/cooling compressor,
              bit[2]=DHW resistor, bit[3]=DHW compressor, bit[4]=water pump,
              bit[7]=heating external booster
[9]         - DHW setpoint temperature (encoded as: °C = value/2 - 16)
[10]        - Zone 1 setpoint temperature
[11]        - Uninterpreted
[12]        - DHW water temperature in full 0x15 status (zero means no DHW sensor)
[13..14]    - Zone 1 water temperature slots in full 0x15 status;
              the decoder uses populated byte 14, otherwise byte 13 (°C = value/2 - 16)
... [N-1]   - Checksum
```

**Auto-detected master**: If `master_address_auto_` is enabled and this frame signature appears, the source address is recorded as the master.

The optional `waterpump_active` binary sensor reports bit 4, including circulation
after heating/cooling is disabled. `heating_cooling_compressor_active` reports
bit 1 in both Heat and Cool. The `dhw_compressor_active` binary sensor
reports bit 3 for the DHW compressor indication. These bits are independent of the
Zone 1 and DHW operation flags in byte 6 and appear in both short `0x11` and full
`0x15` master status frames.

### Optional status indicators

All options below are binary sensors under the `toshiba_ab` climate entry. They
decode reported flags directly from valid master status frames; they do not send
commands or add polling. An omitted option creates no sensor.

| Option | Byte | Mask | Controller indication |
| --- | --- | --- | --- |
| `waterpump_active` | 8 | `0x10` | Internal pump 1 or expansion pump 2 |
| `heating_cooling_compressor_active` | 8 | `0x02` | Heating/cooling compressor |
| `heating_resistor_active` | 8 | `0x01` | Heating resistor inside the hydro unit |
| `dhw_compressor_active` | 8 | `0x08` | DHW compressor |
| `dhw_resistor_active` | 8 | `0x04` | DHW cylinder resistor |
| `heating_external_booster_active` | 8 | `0x80` | Heating external booster, including an auxiliary boiler |
| `test_mode_active` | 6 | `0x04` | Test Mode / Floor Drying wrench indication |

`_active` describes the controller's reported operation/output status, not
independent physical feedback. Feature switches have unsuffixed names. Their
state can mean an enabled setting rather than a function currently being applied:
`night_setback` and `antibacteria` report retained programme settings. The
HWS-P1105XWHM3-E owner's manual, pages 11 and 13, describes Night Setback as a
temperature shift within configured start/end times and Antibacteria as a
recurring programme at a configured time and day interval. Their settings can
remain enabled between operating periods. The `dhw_boost` and `antibacteria`
switches report byte 7 masks `0x40` and `0x80`, respectively, and send commands to
change those states. No duplicate binary sensors are provided for the switchable
features.

Icon visibility also depends on the operating context: Night Setback needs
heating/cooling selected; Antibacteria needs DHW selected. Their switches expose
the underlying flags even when that context is absent and the icon is hidden.

### Feature switches

These options are switches under the same climate entry. Their displayed states
follow valid master status packets rather than optimistic command results.

| Option | On / Off after `E0:01:24` | Status byte 7 |
| --- | --- | --- |
| `frost_protection` | `02:02` / `02:00` | `0x08` |
| `night_setback` | `88:08` / `88:00` | `0x20` |
| `silent_mode` | `04:04` / `04:00` | `0x10` |

Each command pair had matching short/full master statuses and observed icons.
These commands change only their feature: they do not also turn heating On,
change its thermostat mode, or rewrite its setpoint.
Read-only mode suppresses all three commands.

### Silent Mode: active control versus local scheduling

`silent_mode` sends the active-mode On/Off command when the HA switch is toggled;
its reported state follows the master's `0x10` bit. It does not configure the
wall controller's local Silent Mode enable (remote-controller FC09) or start/end
hours (FC0A/0B). Those settings are separate from the bus active state.

In the capture, enabling the controller's menu setting inside an
already-started window sent no On command, even with the compressor running.
At the next configured start boundary it sent `24:04:04`, the master set `0x10`,
and the Zzz icon appeared.

For timed control from HA, schedule `switch.turn_on` and `switch.turn_off` for the
configured Silent Mode switch in an HA automation. The component has no internal
timer and sends the command when called; equipment response is reported by the
master. An enabled wall-controller timer can still issue its own commands, so
choose which scheduler should control the active mode.

### DHW configuration aliases and mapping evidence

The DHW names were changed for consistency with the other status indicators:

| Retained configuration alias | Canonical option |
| --- | --- |
| `hotwater_pump_heating` | `dhw_compressor_active` |
| `hotwater_resistor_heating` | `dhw_resistor_active` |

Both spellings configure the same sensor; existing names, IDs and sensor options
are preserved. Specifying both spellings of one option is rejected instead of
creating duplicate sensors. There is no planned removal of the older aliases.

---

#### 2. **Master Keepalive**
**Signature**: `len == 0x0A && raw[5] == 0x3A`  
**Meaning**: Periodic heartbeat from master confirming it is alive

**Frame layout:**
```
[0]         - Frame length: 0x0A
[1]         - Source: master address (0x70, or auto-detected)
[2]         - Destination (remote address, e.g., 0x60)
[3..4]      - (data)
[5]         - 0x3A (keepalive signature)
... [N-1]   - Checksum
```

This frame resets the "master alive" timeout and confirms connection.

---

#### 3. **Master Sensor/Data Response**
**Signature**: `raw[4] == 0x80 && raw[5] == 0x5C`  
**Meaning**: Response to a sensor query from the remote; contains requested sensor value

**Frame layout:**
```
[0]         - Frame length
[1]         - Source: master address (0x70, or auto-detected)
[2]         - Destination (remote address)
[3]         - (marker)
[4]         - 0x80 (response marker)
[5]         - 0x5C (sensor data signature)
[6]         - High byte of 16-bit sensor value
[7]         - Low byte of 16-bit sensor value
[8]         - (optional additional data)
... [N-1]   - Checksum
```

The 16-bit value at [6:7] is used directly or scaled according to the query type.

---

#### 4. **Master Sensor Unavailable**
**Signature**: `raw[4] == 0x80 && raw[5] == 0xA2`  
**Meaning**: Sensor requested is not available or not supported

Clears the outstanding sensor query flag.

---

### Remote-Originated Frames

#### 5. **Remote Command (Generic)**
**Signature**: `raw[3] == 0xE0 && raw[4] == 0x01 && raw[5] == 0x21`  
**Meaning**: Remote control command

Common sub-commands at [6]:
- `0x03` – Zone 1 on
- `0x0A` – Zone 1 off
- `0x0C` – DHW (domestic hot water) on
- `0x08` – DHW off
- `0x01` – Auto mode on
- `0x00` – Auto mode off

---

#### 6. **Remote Setpoint Change Command**
**Signature**: `raw[3] == 0xE0 && raw[4] == 0x01 && raw[5] == 0x23`  
**Meaning**: Remote requests temperature setpoint change

**Frame layout:**
```
[0]         - Frame length
[1]         - Source: remote address (0x60–0x69)
[2]         - Destination: master (0x70)
[3]         - 0xE0 (command marker)
[4]         - 0x01 (command type)
[5]         - 0x23 (setpoint change signature)
[6]         - (reserved)
[7]         - (reserved)
[8]         - (reserved)
[9]         - 0xA0 (setpoint marker)
[10]        - Target temperature (encoded as: °C = value/2 - 16)
... [N-1]   - Checksum
```

---

#### 7. **Remote Status (Echo)**
**Signature**: `raw[3] == 0xE0 && raw[5] == 0x31`  
**Meaning**: Remote reports its own status (mirrors master status structure)

---

#### 8. **Remote Sensor/Data Query**
**Signature**: `len == 0x0C && raw[4] == 0x41 && raw[5] == 0x5C`  
**Meaning**: Remote requests sensor data from master

**Frame layout:**
```
[0]         - Frame length: 0x0C
[1]         - Source: remote address (0x60–0x69)
[2]         - Destination: master (0x70)
[3]         - (marker)
[4]         - 0x41 (query marker)
[5]         - 0x5C (data request signature)
[6]         - Sensor/data ID to query (e.g., 0xEF for DHW current temperature)
... [N-1]   - Checksum
```

---

#### 9. **Remote Timer Read Query**
**Signature**: `raw[3] == 0xE0 && raw[4] == 0x41 && raw[5] == 0x5C`  
**Meaning**: Remote queries operating hour counters or timer information

---

## Address Space

| Address Range | Meaning | Default | Notes |
|---|---|---|---|
| 0x70 | Master (indoor unit) | 0x70 | Can auto-detect from status frames |
| 0x60–0x69 | Remotes | 0x60 (default for first remote) | Up to 10 remotes supported; collision avoidance increments address |

**Auto-detection Rules:**
- If no master address is set and a status frame (0xE0:0x31) is received, the source becomes the master
- If a remote address collides with outgoing commands, the remote address is auto-incremented up to 0x69
- Once confirmed, addresses persist across frames

---

## Temperature Encoding

Throughout the first-generation Estia protocol, temperatures are encoded as:

```
value = (°C + 16) × 2

Decoding:
°C = value / 2 - 16

Examples:
  20°C  = (20 + 16) × 2 = 0x48 (72)
  22.5°C = (22.5 + 16) × 2 = 0x4D (77)
  0°C   = (0 + 16) × 2 = 0x20 (32)
  -10°C = (-10 + 16) × 2 = 0x0C (12)
```

---

## Checksum Validation

Before processing any frame, always validate the checksum:

```cpp
uint8_t calculated = (sum of bytes [0..N-2]) & 0xFF;
uint8_t received = raw[N-1];

if (calculated != received) {
  // CRC fail — discard frame
  return;
}
```

---

## Protocol Behavior

### Initialization & Auto-Detection

When `master_address_auto_` is enabled:

1. Component watches for incoming frames
2. On receipt of a master-status frame (0xE0:0x31), the source address is recorded as the master
3. Keepalive frames (0xE0:0x3A) with matching keepalive signature confirm master identity
4. **Remote address auto-detect**: If a received remote ping has `src == remote_address_` and `remote_address_auto_` is true, the next address up to `0x69` is chosen (collision avoidance)

### Master Alive Timeout

- On **any** frame from master, timestamp `last_master_alive_millis_` is updated
- After `LAST_ALIVE_TIMEOUT_MILLIS` (~16s) with no master frame, component reports disconnected
- Connected binary sensor is updated accordingly

### Sensor Queries

1. Component sends a query: `raw[4]==0x41, raw[5]==0x5C, raw[6]=sensor_id`
2. Master responds with sensor data (0x80:0x5C) or "unavailable" (0x80:0xA2)
3. Query is marked complete; next polled sensor can be queried
4. If no response within timeout (~2.5s), query is abandoned

---

## Known Sensor Query IDs

| Sensor ID | Description | Notes |
|-----------|-------------|-------|
| 0xEF      | DHW current temperature | Common request for hot water status |
| 0x0A      | (TBD) | Estia first-gen specific |
| (others)  | Zone setpoints, outdoor temp, etc. | Depends on unit configuration |

---

## Common Frame Examples

### Status Update from Master
```
13:70:60:E0:00:31:81:A0:84:4D:48:00:48:48:CC
```
- Length: 13 (19 bytes)
- Source: 70 (master)
- Dest: 60 (remote)
- Status marker: E0, signature: 31
- Byte[6] = 81: Zone1 active (bit 0), DHW active (bit 1)
- Byte[7] = A0: Auto-mode and heating flags
- Byte[8] = 84: Hot water heating flags
- Byte[9] = 4D: DHW setpoint = 4D/2 - 16 = 22.5°C
- Byte[10] = 48: Zone 1 setpoint = 48/2 - 16 = 20°C
- Byte[11] = 00: Padding
- Byte[12] = 48: Zone 1 water temp = 48/2 - 16 = 20°C
- Checksum: CC

---

### DHW On Command from Remote
```
0B:60:70:E0:01:21:0C:00:00:00:E8
```
- Length: 0B (11 bytes)
- Source: 60 (remote)
- Dest: 70 (master)
- Signature: E0:01:21 (remote command)
- Byte[6] = 0C: DHW on command
- Bytes[7-10]: Padding
- Checksum: E8

---

### DHW Off Command from Remote
```
0B:60:70:E0:01:21:08:00:00:00:E4
```
- Length: 0B (11 bytes)
- Source: 60 (remote)
- Dest: 70 (master)
- Signature: E0:01:21 (remote command)
- Byte[6] = 08: DHW off command
- Checksum: E4

---

### Setpoint Change Command (22.5°C)
```
0F:60:70:E0:01:23:00:00:00:A0:4D:00:00:00:C5
```
- Length: 0F (15 bytes)
- Source: 60 (remote)
- Dest: 70 (master)
- Signature: E0:01:23 (setpoint change)
- Byte[6-8]: Reserved (00:00:00)
- Byte[9] = A0: Setpoint marker
- Byte[10] = 4D: Target temp = 4D/2 - 16 = 22.5°C
- Checksum: C5

---

### Sensor Query (DHW Current Temperature)
```
0C:60:70:E0:41:5C:EF:00:00:00:5F
```
- Length: 0C (12 bytes)
- Source: 60 (remote)
- Dest: 70 (master)
- Signature: E0:41:5C (sensor query)
- Byte[6] = EF: Query sensor ID (DHW current temperature)
- Bytes[7-9]: Padding/reserved
- Checksum: 5F

---

### Master Sensor Response
```
0A:70:60:80:80:5C:00:42:00:6C
```
- Length: 0A (10 bytes)
- Source: 70 (master)
- Dest: 60 (remote)
- Signature: 80:80:5C (sensor response)
- Bytes[6-7] = 00:42: 16-bit value = 0x0042 = 66 (encoded temperature ≈ 17°C)
- Byte[8]: Padding
- Checksum: 6C

Decoded temperature: 66/2 - 16 = 33 - 16 = 17°C

---

### Master Keepalive
```
0A:70:60:00:00:3A:00:00:00:F0
```
- Length: 0A (10 bytes)
- Source: 70 (master)
- Dest: 60 (remote)
- Signature: 3A (keepalive)
- Bytes[3-4]: Data (00:00)
- Bytes[7-9]: Padding
- Checksum: F0

---

### Sensor Not Available Response
```
0A:70:60:80:80:A2:00:00:00:8C
```
- Length: 0A (10 bytes)
- Source: 70 (master)
- Dest: 60 (remote)
- Signature: 80:80:A2 (sensor not available)
- Bytes[6-9]: Data/padding
- Checksum: 8C

---

## Error Handling

### Checksum Failures

- Log frame with error detail
- Discard frame
- Continue listening for next frame

### Invalid Lengths

- If `frame->raw[0] < 7` or too large: skip
- If calculated `raw_len` exceeds buffer: return

### Auto-Detection Collisions

- If a received remote ping source matches current remote address and `remote_address_auto_` is true, increment remote address
- Report collision in logs

### Timeouts

- Sensor query timeout (default ~2.5s): clear outstanding flag, allow next query
- Master alive timeout (default ~16s): report disconnected state

---

## Debugging & Logging

Key logging points in the code:

1. **Frame label**: Shows frame type identification (Master Keepalive, Remote Command, etc.)
2. **Raw checksum output**: Helps verify protocol understanding
3. **Sensor query tracking**: Logs sensor ID and raw value for each response
4. **Address auto-detection**: Logs master/remote addresses when discovered

Enable `DEBUG` log level on the `TAG` to see all frame details.

---

## References

- Source: `components/toshiba_ab/toshiba_ab.cpp` — `process_received_data_estia_first_gen_()` (line 3264)
- Frame building: `toshiba_ab.cpp` — `make_estia_first_gen_frame()` (line 3203)
- Commands: `send_estia_first_gen_*()` family of functions
- Header constants: `components/toshiba_ab/toshiba_ab.h` (lines 29–31)

---

## Appendix: Byte-by-Byte Identification Quick Reference

| Byte Position | Value Range | Meaning |
|---|---|---|
| [0] | 07–20 | Frame length |
| [1] | 70 | Master address (fixed, or auto-detected) |
| [1] | 60–69 | Remote address range |
| [3] | E0 | Command/status marker |
| [3] | 80 | Data/response marker |
| [4] | 01 | Command type |
| [4] | 41 | Query/request type |
| [4] | 80 | Data response |
| [5] | 21 | Generic command |
| [5] | 23 | Setpoint change command |
| [5] | 31 | Status frame |
| [5] | 3A | Keepalive signature |
| [5] | 5C | Sensor data/query |
| [5] | A2 | Sensor not available |
