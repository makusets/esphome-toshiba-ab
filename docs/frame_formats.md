# Toshiba AB frame formats

This document describes the three wire protocols understood by the current
component: **TCC**, **TU2C**, and **A0**. `system_type` (`air` or `water`) is not
a fourth protocol: it describes the equipment using a protocol. In particular,
first-generation ESTIA water systems use TU2C, while newer ESTIA systems use A0.

The implementation is currently identification-only. It collects and checks all
frames, but identifies only the two frame types documented below: **master
keepalive** and **remote ping**. Other traffic is logged without being decoded.

All numbers below are hexadecimal bytes. `N` means the value of a length field,
`...` means zero or more bytes, and multi-byte fields are shown most-significant
byte first unless stated otherwise.

## 1. Protocol frame structures

### 1.1 TCC

```text
byte        0     1       2       3     4 .. 4+N-1  4+N
field      SRC   DST    OPCODE    LEN      DATA       CRC
```

The complete frame is `N + 5` bytes. `LEN` counts only `DATA`; the two address
bytes, opcode, length byte, and checksum are excluded. TCC has no sync/preamble
bytes, so the receiver uses `LEN` to locate the end of a candidate frame. `CRC`
is the XOR of every preceding byte, starting at `SRC`.

The component accepts encoded lengths from `0x02` up to the 132-byte receive
buffer limit. It also rejects a candidate source address above `0xA0` as likely
line noise. TCC uses 2400 baud, 8E1 (even parity).

### 1.2 TU2C

```text
byte        0    1       2       3     4       5       6       7     ...   L-2       L-1
field      F0   F0   TOTAL_LEN  SRC   DST   FAMILY   OPCODE  DTYPE  DATA  CHECKSUM    A0
```

`TOTAL_LEN` (`L`) is the size of the **entire frame**, including both `F0`
bytes, the length byte, checksum, and final `A0`. Valid values are `0x07` through
`0x84` (132 bytes). `FAMILY` is commonly `C0` on air systems and `E0` on
first-generation ESTIA water systems; it is part of the payload rather than a
separate wrapper field. `CHECKSUM` is the low byte of the sum from
`TOTAL_LEN` (byte 2) through the last data byte. TU2C uses 2400 baud, 8N1 (no
parity).

### 1.3 A0

```text
byte        0    1     2      3    4      5       6      7       8       9        10    ...  N+4  N+5
field      A0   00   TYPE    LEN   00  SRC_MODE  SRC  DST_MODE  DST  DTYPE_H  DTYPE_L DATA CRC_H CRC_L
```

The complete frame is `N + 6` bytes. `LEN` counts the bytes after itself and
before the two-byte CRC (bytes 4 through `N+3`); it therefore includes the
reserved `00`, both two-byte address fields, the data type, and any data.
`SRC_MODE:SRC` and `DST_MODE:DST` are sometimes called the source and
destination high/low bytes. The component uses the low bytes (`SRC` and `DST`)
as participant addresses.

The received CRC is big-endian (`CRC_H:CRC_L`). Its value is
CRC-16/MCRF4XX (initial value `0xFFFF`, reflected polynomial `0x8408`, no final
XOR) over every byte from the `A0:00` prefix through the last data byte. A0 uses
2400 baud, 8E1 (even parity).

## 2. Typical master addresses

Addresses are learned from a checksum-valid master keepalive when
`master_address: auto` is used. Consequently these values are useful capture
landmarks, not hard-coded requirements.

| Protocol | `system_type` | Typical master address | Notes |
| --- | --- | --- | --- |
| TCC | `air` | `0x00` | Indoor-unit/master address used by classic TCC installations. |
| TCC | `water` | No established value | No distinct TCC water profile has been identified; use automatic discovery rather than assuming `0x00`. |
| TU2C | `air` | `0x90` | Indoor-unit/master address commonly paired with remotes in the `0x50` range. |
| TU2C | `water` | `0x70` | First-generation ESTIA (R410A) master. |
| A0 | `air` | `0x00` | A0 identifies the full source as commonly `08:00`; the component records its low byte, `0x00`. |
| A0 | `water` | `0x00` | ESTIA R32 master, likewise commonly encoded as `08:00`. |

`0xAA` is the component's internal sentinel for an address that is still
automatic/unresolved. It is not a normal bus master address.

## 3. Typical remote addresses

A remote is recorded only after the protocol and master have been confirmed and
a valid remote ping is addressed to that master. These are observed conventions;
the classifier does not restrict a valid remote ping to these ranges.

| Protocol | `system_type` | Typical remote address(es) | Notes |
| --- | --- | --- | --- |
| TCC | `air` | `0x40` | Conventional wall remote. |
| TCC | `water` | No established value | No separate TCC water remote profile is currently identified. |
| TU2C | `air` | `0x50` | Conventional U-series remote. Additional controllers may occupy nearby addresses. |
| TU2C | `water` | `0x60`–`0x69` | First-generation ESTIA supports up to ten remote addresses; `0x60` is the usual first remote. |
| A0 | `air` | `0x40` | Conventional remote, normally represented by the low source byte. |
| A0 | `water` | `0x40`; `0x41` for the optional demand interface | The currently identified A0 remote-ping signature is the `0x41` 0–10 V demand-interface ping. |

The ESP's `esp_address` setting also defaults to automatic. Remote discovery
currently maintains a presence list only; it does not yet select or change the
ESP address.

## 4. Identified frame types and signatures

Classification happens only after the protocol checksum has passed. A signature
is the conjunction of all columns below, not merely an opcode match.

### 4.1 Master keepalive

The first valid master keepalive confirms the protocol and supplies the master
source address. Air and water use the same signature within each wire protocol.

| Protocol | `system_type` | Encoded length | Opcode/type | Data type | Field-level signature |
| --- | --- | ---: | ---: | ---: | --- |
| TCC | `air`, `water` | `0x02` | `OPCODE = 0x10` | `DATA[1] = 0x8A` | `SRC:*:10:02:*:8A:CRC` |
| TU2C | `air`, `water` | `TOTAL_LEN = 0x0A` | byte 6 `OPCODE = 0x00` | byte 7 `DTYPE = 0x3A` | `F0:F0:0A:SRC:DST:*:00:3A:SUM:A0` |
| A0 | `air`, `water` | `LEN = 0x07` | `TYPE = 0x10` | `DTYPE = 00:8A` | `A0:00:10:07:00:SRC_MODE:SRC:DST_MODE:DST:00:8A:CRC16` |

For TCC, the unspecified first data byte is still covered by the XOR. For TU2C,
the family byte and destination are not used to distinguish air from water. For
A0, both complete mode/address pairs remain part of the checksummed frame even
though discovery records the low source byte.

### 4.2 Remote ping

Every remote-ping signature additionally requires `DST` to equal the already
confirmed master and `SRC` to differ from it.

| Protocol | `system_type` | Encoded length | Opcode/type | Data type | Additional signature detail |
| --- | --- | ---: | ---: | ---: | --- |
| TCC | `air` | `0x07` | `OPCODE = 0x15` | `DATA[1] = 0x0C` | Payload begins `08:0C:81`; canonical shape is `SRC:DST:15:07:08:0C:81:...:CRC`. |
| TCC | `water` | `0x07` | `OPCODE = 0x15` | `DATA[1] = 0x0C` | The current classifier is system-type agnostic; no separate water signature is known. |
| TU2C | `air` | `TOTAL_LEN = 0x0C` | byte 6 `OPCODE = 0x41` | byte 7 `DTYPE = 0x5C` | Common family byte `C0`: `F0:F0:0C:SRC:DST:C0:41:5C:...:SUM:A0`. |
| TU2C | `water` | `TOTAL_LEN = 0x0C` | byte 6 `OPCODE = 0x41` | byte 7 `DTYPE = 0x0C` | First-generation ESTIA commonly uses family byte `E0`. |
| A0 | `air` | `LEN = 0x0C` | `TYPE = 0x55` | `DTYPE = 00:9F` | The current classifier is system-type agnostic; this is primarily the demand-interface signature. |
| A0 | `water` | `LEN = 0x0C` | `TYPE = 0x55` | `DTYPE = 00:9F` | Identified for the ESTIA 0–10 V demand interface, normally source `0x41`. |

The TCC implementation's decisive comparison is length/opcode/data type; the
longer `08:0C:81` prefix shown in the table is the canonical observed ping. The
A0 and TU2C classifiers likewise compare the fields shown and do not require
the illustrative family, mode, or trailing data values.

## 5. Examples of every identified frame type

These examples are complete, checksum-valid frames. They illustrate the
current identifiers; bytes marked as otherwise unspecified by a signature are
representative and should not be interpreted as a complete command/status
decode.

### 5.1 Master keepalive examples

#### TCC — air (also the current TCC water signature)

```text
00:40:10:02:00:8A:D8
```

`SRC=00`, `DST=40`, opcode `10`, length `02`, data type `8A`; `D8` is the XOR.
The identical signature is used if `system_type: water`, although no typical
TCC water installation/address profile is established.

#### TU2C — air

```text
F0:F0:0A:90:50:C0:00:3A:E4:A0
```

The master is `0x90`, the representative air remote destination is `0x50`, and
the `C0:00:3A` payload contains opcode `00` and data type `3A`.

#### TU2C — water (first-generation ESTIA)

```text
F0:F0:0A:70:60:E0:00:3A:F4:A0
```

This has the same keepalive identifiers, with typical water addresses and the
`E0` family byte. `F4` is the additive checksum.

#### A0 — air and water

```text
A0:00:10:07:00:08:00:00:40:00:8A:EF:5C
```

The master source is encoded as `08:00`, so the discovered 8-bit address is
`0x00`; the representative destination low byte is `0x40`. A0 air and water use
the same keepalive signature and this example applies to both system types.

### 5.2 Remote ping examples

#### TCC — air (also accepted with `system_type: water`)

```text
40:00:15:07:08:0C:81:00:00:48:00:9F
```

Remote `0x40` pings confirmed master `0x00`; `9F` is the XOR checksum. The
current classifier applies the same signature for water, but a distinct TCC
water ping/address convention has not been established.

#### TU2C — air

```text
F0:F0:0C:50:90:C0:41:5C:90:F3:CC:A0
```

Remote `0x50` pings master `0x90` with opcode/data type `41:5C`; `CC` is the
additive checksum.

#### TU2C — water (first-generation ESTIA)

```text
F0:F0:0C:60:70:E0:41:0C:90:F3:8C:A0
```

Remote `0x60` pings master `0x70`; the water-specific data type is `0x0C` and
`8C` is the additive checksum.

#### A0 — water demand interface (also accepted for A0 air)

```text
A0:00:55:0C:00:00:41:08:00:00:9F:00:00:00:00:00:64:78
```

The demand interface source low byte is `0x41`, the master destination is
encoded as `08:00`, and the type/data-type signature is `55` / `00:9F`.
`64:78` is the CRC-16. Because the current A0 classifier does not branch on
`system_type`, the same complete example is also recognized for an A0 air
configuration.

## Configuration names

The current configuration key is `format`, accepting `auto`, `tcc`, `tu2c`, or
`a0`. In `auto`, the component scans TCC, A0, then TU2C for 20 seconds each and
changes UART parity with the candidate protocol. An explicit format listens
only for that format. Use `system_type: air` or `system_type: water` separately
to describe the attached equipment.
