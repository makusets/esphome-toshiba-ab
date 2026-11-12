# First-generation ESTIA fault diagnostics

For `frame_format: estia` (R410A), the component automatically creates two enabled
diagnostic entities. No additional YAML entries are required:

| Entity | Meaning |
| --- | --- |
| Toshiba Fault Active | Problem binary sensor: On for any nonzero master fault code |
| Toshiba Fault Code | Text sensor: decoded code such as `A01`, `E09`, or `F23`; `None` for an explicit clear |

At boot they default to `None`/off without waiting for a report. This is an
assumed no-fault state, not confirmation of master connectivity. Valid master
fault reports update their state; normal
operating-status packets, keep-alives and a remote controller's status do not
imply an all-clear. Both become unknown on the existing master-connection timeout
and require another explicit fault or clear report after reconnection. The native
API propagates the unknown state to Home Assistant.

Detection is entirely passive, including in read-only mode. This feature adds no
polling, reset commands or other bus traffic. Unsolicited `C0:29` announcements
report faults and clears. Broadcast `A0:29` replies to another controller's
requests are decoded too. If the ESP starts after an announcement, it retains
the boot default until the next report. A missed clear leaves the last reported
fault active until another explicit clear or a connection timeout; ordinary bus
traffic does not resolve that uncertainty.

The optional existing `remote_error` sensor remains independent: it reports a
remote controller's error indication, whereas these diagnostics report the
master's fault code. No A0/R32, Normal, HM or TU2C fault decoding is added.

## Captured frames and decoding

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
