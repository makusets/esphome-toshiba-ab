# ESTIA fault reporting

Both `frame_format: estia` and `frame_format: a0` (alias `estia_a0`) support
independently optional diagnostic entities inside the `toshiba_ab` climate block:

```yaml
fault_code:
  name: "Heat Pump Fault Code"
fault_active:
  name: "Heat Pump Fault Active"
```

`fault_code` reports the current code, such as `A01` or `F23`, and `None` on an
explicit zero/clear report. `fault_active` is a problem binary sensor: any nonzero
code is on, including an unrecognized code displayed as `0xNN`. Both become unknown
on the existing master-connection timeout and require a fresh fault/clear report
after reconnection. Ordinary status or heartbeat packets do not clear a fault.
Unknown state is sent through the native API to Home Assistant.

With writable `frame_format: estia`, configuring either entity requests current
fault status after the existing 40-second startup send block, once the master is
confirmed, and every 30 seconds thereafter. The captured `E0:61:29` read uses the
component's own remote address. The sensors stay unknown until a real response;
there is no assumed initial OK. Set `fault_poll_interval: 10s` to change the
interval (minimum 1s), or `fault_poll_interval: never` for passive decoding.
Read-only mode always blocks queries. Queries add no reset, mode or setpoint
commands and do not require an ACK. Passive decoding remains available in both
protocols. The entities are separate from `remote_error`, which reports a remote
controller indication. Select an explicit supported frame format; `auto`, normal,
HM and TU2C do not support these entities or fault polling.

Series 5 R410A captures confirm opcode `0x29` announcements and polled responses
for `0x01` = A01, `0x77` = F23, and `0x00` = clear. The formatter retains the
existing A0 family ranges: A `01–0D`, E `41–52`, F `63–7F`, H `81–84`,
L `C2–DD`, P `E3–FF` (hexadecimal), subtracting each family's base
`00/40/60/80/C0/E0` to obtain the decimal suffix. Other mappings are inferred.
A0 extraction follows the existing alarm data type containing `0x49` and the
following error byte from the configured master. Its layout, exact codes and zero
clear are tested with constructed CRC-valid packets, not verified on R32 hardware.

**A0 validation gap:** this fault-reporting change was developed without access
to an A0/R32 pump. Its fault decoder and clear handling need hardware validation,
and its fault-query/probe packet still needs to be identified and verified.
Until then, A0 starts with an assumed
`None`/off state, not a pump-confirmed all-clear; subsequent alarm reports override
that assumption. It remains passive, including in read-only mode. Any upstream
PR for this feature must disclose the missing hardware/probe and unverified A0
behavior, including the assumed initial state.

## Captured first-generation fault query

The physical controller sent `0A:60:70:E0:61:29:44`. After F23 cleared,
the pump answered `0C:70:FF:E0:A0:29:70:00:94`: a polled all-clear.
The component uses its own selected remote address and calculates the checksum,
rather than impersonating the physical controller at `0x60`. Fault payload byte
`70` remains uninterpreted; it is not used to choose the A/F code family.
