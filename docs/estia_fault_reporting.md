# First-generation ESTIA fault diagnostics

For `frame_format: estia` (R410A), the component automatically creates two enabled
diagnostic entities. No additional YAML entries are required:

| Entity | Meaning |
| --- | --- |
| Toshiba Fault Active | Problem binary sensor: On for any nonzero master fault code |
| Toshiba Fault Code | Text sensor: decoded code such as `A01`, `E09`, or `F23`; `None` for an explicit clear |

At boot they default to `None`/off without waiting for a report. This is an
assumed no-fault state, not confirmation of master connectivity. A single
`E0:61:29` fault-status query refreshes that assumption after the existing
40-second startup listening period, master confirmation and a safe transmit
slot. It uses the currently selected remote and master addresses, independently
of `ping` and `autonomous` settings.

Both entities become unknown on the existing master-connection timeout. Once
valid AB-master traffic resumes, one further query refreshes the state. Ordinary
operating-status packets and keep-alives establish connectivity but do not imply
an all-clear; only a valid fault or clear report restores these diagnostic
states. This applies to AB-bus recovery, not Home Assistant API reconnections.
The native API propagates the unknown state to Home Assistant.

There is no periodic fault polling or query retry loop. Faults and clears remain
passively decoded between startup/recovery refreshes. Unsolicited `C0:29`
announcements and broadcast `A0:29` replies are both accepted, including replies
to another controller's requests. Read-only mode suppresses the query while
continuing to decode incoming reports; an unsent refresh remains pending until
transmission is allowed. Boot listening, occupied-address exhaustion, bus quiet
time, pre-keepalive hold-off and pending command acknowledgement handling also
apply. The query is built only when it can be sent, so a changed controller
address or a read-only toggle cannot leave a stale refresh in the send queue.

A missing query response does not create an all-clear or trigger repeated
requests. The boot default or last reported state remains until another valid
report or a connection timeout.

The optional existing `remote_error` sensor remains independent: it reports a
remote controller's error indication, whereas these diagnostics report the
master's fault code. No A0/R32, Normal, HM or TU2C fault decoding is added.

## Captured frames and decoding

The captured request from Header `0x60` to master `0x70` is
`F0:F0:0A:60:70:E0:61:29:44:A0` (including its wrapper).

First-generation Series 5 captures reviewed in
[PR #281](https://github.com/makusets/esphome-toshiba-ab/pull/281) include:

```text
0C:70:FF:E0:C0:29:70:01:B5  A01 announcement
0C:70:FF:E0:C0:29:70:77:2B  F23 announcement
0C:70:FF:E0:C0:29:70:00:B4  unsolicited all-clear announcement
0C:70:FF:E0:A0:29:70:00:94  all-clear reply to a controller query
```

Issue [#292](https://github.com/makusets/esphome-toshiba-ab/issues/292), log6,
also contains `0C:70:FF:E0:A0:29:70:49:DD`, reporting E09.

The decoder requires the exact packet length, the selected master's source
address, broadcast destination, `E0:C0/A0:29` signature, and a valid checksum.
The code is byte 7. Byte 6 is retained as uninterpreted payload, not a selector
for the fault-code family.

The family encoding maps hexadecimal ranges A `01–0D`, E `41–52`, F `63–7F`,
H `81–84`, L `C2–DD` and P `E3–FF` to decimal suffixes by subtracting
`00/40/60/80/C0/E0`. A01, F23, E09 and clear have capture evidence; the remaining
mappings are inferred. Unrecognized codes remain visible as `0xNN` and still
activate the problem sensor.

## Optional naming overrides

The entities exist without these entries. Standard ESPHome entity options can
still customize their names and IDs:

```yaml
fault_active:
  name: Heat Pump Fault
fault_code:
  name: Heat Pump Fault Code
```

These options are valid only with `frame_format: estia`.
