# A0/R32 demand emulation startup

With `frame_format: a0` and `demand_enabled: true`, automatic demand heartbeats
wait for a known demand value. Rebooting the ESP no longer causes it to transmit
a fabricated zero before reading the bus. The component learns demand during
the existing startup silence and starts heartbeats afterward using the most
recent decoded value. It does not queue intermediate heartbeats during startup.

Known values can come from a checksum-valid report on the bus or an explicit
`send_estia_demand(value)` call. Zero is a valid decoded or explicitly requested
value; it is distinct from an unknown startup state. If no suitable report or
explicit request arrives, demand heartbeats remain suspended. No new query or
polling is introduced to force initialization.

The optional `demand` sensor reports the decoded AI1 channel. The physical
interface has three channels; AI2 and AI3 are retained from reports and carried
forward in subsequent heartbeats and AI1 commands. A demand command awaiting its
ACK is not replaced by an older demand echo. Read-only mode still permits
decoding but blocks demand commands and heartbeats. Other protocols are unchanged.

## Decoded packets

The following layouts use byte offsets after removing the `A0:00` prefix:

| Report | Required source/destination | AI1 | AI2 | AI3 |
| --- | --- | --- | --- | --- |
| Master `58`, dtype `00:9F` | Selected master to broadcast `00:FE`, this remote, or interface `00:41` | 9 | 13 | 14 |
| Interface `55`, dtype `00:9F` | `00:41` to selected master | 10 | 12 | 13 |
| Interface `11`, dtype `00:5F` | `00:41` to selected master | 9 | 10 | 11 |

CRC, payload length, addresses and channel ranges (0–15) are checked before
accepting state. The master echo must not be decoded with the interface-status
offsets: doing so reports an unrelated zero and can label another byte as a
137°C minimum temperature, as in [issue #291](https://github.com/makusets/esphome-toshiba-ab/issues/291).

These layouts are described in the
[A0 protocol findings in #197](https://github.com/makusets/esphome-toshiba-ab/issues/197#issuecomment-6170709812).
The interface heartbeat's `0x14` byte is retained as an observed constant with
unverified meaning, not interpreted as a minimum temperature. Demand-to-setpoint
conversion depends on DN 680 and other installation settings, so logs report raw
channels instead of assuming a fixed temperature formula.

## Home Assistant slider versus reported demand

An optimistic template number with `restore_value: true` restores its displayed
value. That alone does not call `send_estia_demand()` on the climate component.
Bus synchronization therefore uses the master's reported value rather than
assuming the slider has restored the emulation state. An explicit user or
automation request still sets the desired AI1 value immediately, even if a
startup report has not arrived yet.

For an independent readback, add this under the existing climate entry:

```yaml
demand:
  name: Estia Reported Demand
```
