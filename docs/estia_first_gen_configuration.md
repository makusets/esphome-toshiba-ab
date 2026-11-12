# Estia First-Generation (R410A) Configuration

This guide covers optional indicators, feature switches and physical-controller
interaction for `frame_format: estia`. Start with the
[complete R410A example](../estia_R410A_example.yaml). Packet layouts, status bits
and command bytes are documented in the
[protocol reference](estia_first_gen_protocol.md).

## Optional status indicators

All options below are binary sensors under the `toshiba_ab` climate entry. They
decode reported flags directly from valid master status frames; they do not send
commands or add polling. An omitted option creates no sensor.

| Option | Controller indication |
| --- | --- |
| `waterpump_active` | Internal pump 1 or expansion pump 2 |
| `heating_cooling_compressor_active` | Heating/cooling compressor |
| `heating_resistor_active` | Heating resistor inside the hydro unit |
| `dhw_compressor_active` | DHW compressor |
| `dhw_resistor_active` | DHW cylinder resistor |
| `heating_external_booster_active` | Heating external booster, including an auxiliary boiler |
| `test_mode_active` | Test Mode / Floor Drying wrench indication |

`_active` describes the controller's reported operation/output status, not
independent physical feedback. Feature switches have unsuffixed names. Their
state can mean an enabled setting rather than a function currently being applied:
`night_setback` and `antibacteria` report retained programme settings that can
remain enabled between operating periods. Night Setback shifts the heating or
cooling target within configured start/end times; Antibacteria runs a recurring
programme at a configured time and day interval.

The existing `dhw_boost` and `antibacteria` switches report their states and send
commands to change them. No duplicate binary sensors are provided for switchable
features. Icon visibility also depends on the operating context: Night Setback
needs heating/cooling selected; Antibacteria needs DHW selected. Their switches
expose the underlying flags even when that context is absent and the icon is hidden.

## Feature switches

Configure the desired options under the same `toshiba_ab` climate entry:

| Option | Control |
| --- | --- |
| `dhw_boost` | Hot water boost |
| `antibacteria` | Antibacteria programme enable |
| `frost_protection` | Frost Protection |
| `night_setback` | Night Setback programme enable |
| `silent_mode` | Active Silent Mode |

For example, add these entries to an existing climate configuration:

```yaml
waterpump_active:
  name: Water Pump Active
silent_mode:
  name: Silent Mode
```

The switch states follow valid master status packets rather than optimistic
command results. Frost Protection, Night Setback and Silent Mode commands change
only their feature: they do not also turn heating On, change its thermostat mode,
or rewrite its setpoint.

Feature switches remain writable when ESPHome is in Second mode. That role does
not by itself prevent the master from accepting a command. A physical controller
or its schedule can subsequently change the same setting. The HA switch follows
the resulting master status, so an overridden On command appears as Off rather
than remaining optimistically On. Read-only mode (`read_only: true`) suppresses
ESPHome's feature commands regardless of its remote role.

The Silent Mode conflicts below have been observed. The behavior of Frost
Protection, Night Setback and Antibacteria when another controller changes the
same setting has not been tested.

## Silent Mode: active control versus local scheduling

`silent_mode` sends the active-mode On/Off command when the HA switch is toggled.
It does not configure the wall controller's local Silent Mode enable
(remote-controller FC09) or start/end hours (FC0A/0B). Those settings are separate
from the bus active state.

| Physical remote configuration | Observed behaviour |
| --- | --- |
| Remote in Header mode, FC09 = 0 | Cancels externally activated Silent Mode after the master reports it On. Changing the scheduled hours alone does not prevent this. |
| Remote in Header mode, FC09 = 1 | Allows external On/Off commands, including outside the configured window. Its schedule can still issue commands. |
| Remote in Second mode, with ESPHome in Header mode at `0x60` | External On/Off commands and the compressor-frequency limit worked without cancellation by the physical remote. |

Enabling the menu setting on the physical remote in Header mode inside an
already-started window did not activate Silent Mode, even with the compressor
running. It became active at the next configured start boundary. The exact
scheduled stop-boundary timing
has not been verified. See the
[controller behaviour reference](estia_first_gen_protocol.md#silent-mode-controller-behaviour)
for the packet exchange.

FC09, FC0A and FC0B here belong to the remote controller. The hydro unit has
different settings under the same numbers; ordinary hydro-unit DN reads/writes
do not access the controller's Silent Mode settings.

For timed control from HA, schedule `switch.turn_on` and `switch.turn_off` for the
configured Silent Mode switch in an HA automation. The component has no internal
timer and sends the command when called; equipment response is reported by the
master. With the physical remote in Header mode, FC09 must allow the externally
selected state, and HA scheduling must account for any commands from that
remote's timer.

Using the physical remote in Second mode requires firmware with first-generation
keep-alive support for ESPHome in Header mode; see [Header mode support](https://github.com/makusets/esphome-toshiba-ab/pull/295).
Selecting address `0x60` alone is insufficient and can result in E03. After
changing the physical remote to Second mode, restart ESPHome so automatic address
selection can start again at `0x60`; verify the actual source addresses in the
logs. Leave address selection automatic rather than forcing `0x60` while a
physical remote in Header mode still occupies it.

The owner's manual disables Silent Mode, Schedule timer and Schedule setting on
a physical remote in Second mode. Room-temperature control from a remote in
Second mode is documented through hydro-unit FC40 = 1 and the remote's
Temperature control setting, but that setup has not been verified with ESPHome
in Header mode.

For the models covered by service manual A10-1705, Silent Mode limits compressor
frequency to 40.2 Hz in heating, cooling and DHW, and outdoor fan speed to 500 rpm.
These are maximum values, not fixed operating targets.

## DHW configuration aliases

The DHW names were changed for consistency with the other status indicators:

| Retained configuration alias | Canonical option |
| --- | --- |
| `hotwater_pump_heating` | `dhw_compressor_active` |
| `hotwater_resistor_heating` | `dhw_resistor_active` |

Both spellings configure the same sensor; existing names, IDs and sensor options
are preserved. Specifying both spellings of one option is rejected instead of
creating duplicate sensors. There is no planned removal of the older aliases.

## Manuals

Page numbers below are the printed page numbers:

- [Owner's manual](https://viridusenergy.ie/wp-content/uploads/2023/05/20190925_OM_EF99911201-4_Hydro_Indoor_HWS-_5PXWHMT-E_EN.pdf): Night Setback on p. 11, Antibacteria on p. 13, Silent Mode on pp. 11–12, and restrictions for remotes in Header or Second mode on p. 17.
- [Installation manual](https://www.cdlweb.info/wp-content/uploads/2020/01/20190925_IM_EF99812301-4_Hydro_Indoor_HWS-_5PXWHMT-E_EN.pdf): remote-controller Silent Mode FC table on p. 51; room-temperature control from a remote in Second mode on pp. 30, 40 and 52.
- [Service manual A10-1705](https://www.toshibapro.fi/en/tuotteet/lampopumppujarjestelma/Ilves%20Ilma-Vesilampopumppu/lammonvaihdinyksikot/ilves-huoltomanuaali/at_download/file): Silent Mode frequency and fan limits on p. 54.
