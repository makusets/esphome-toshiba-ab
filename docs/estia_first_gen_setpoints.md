# First-generation ESTIA Zone 1 setpoints

`frame_format: estia` uses different `E0:01:23` payloads for fixed Heat and
Cool targets. The selected mode comes from the control call, a pending mode
transition, or the last controller report when the climate entity is Off.
Setting a target while Off does not turn Zone 1 on. Auto remains controlled by
the weather curve and ignores fixed targets.

| Mode | Accepted target range | Payload after `23` |
| --- | --- | --- |
| Heat | 20–65°C | `02:T:66:A2:T` |
| Cool | 7–25°C | `01:T:T:A2:T` |

`T` is `round((target °C + 16) × 2)`. These ranges validate writes independently
of visual limits. YAML defaults the visual minimum to 20°C; configure it as 7°C
to expose the full cooling range in Home Assistant:

```yaml
zone1_climate:
  name: Zone 1
  visual:
    min_temperature: 7
    max_temperature: 65
```

Installer settings can impose tighter limits. The existing heating layout is
unchanged. Cooling uses the complete layout captured from an HWS-P1105XWHM3-E
auxiliary wall controller:

```text
0F:60:70:E0:01:23:01:48:48:A2:48:5E
```

That controller command changed the reported cooling target from 15°C to 20°C.
The former ESPHome heating-style payload left it at 15°C. The controller's
7–25°C range was confirmed by its owner. Companion-field semantics and other
first-generation model variants have not been independently established.

Native regression tests exercise the actual climate control path and check the
captured packet, limits, heating compatibility, mode changes, Off, Auto,
read-only behavior and the startup transmission guard. Live results belong to
the deployment record; a matching packet alone does not establish circulation
or compressor operation.
