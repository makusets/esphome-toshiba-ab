# Reader regression checks

These checks exercise the component's actual `DataFrameReader` with clean,
zero-prefixed, back-to-back, and invalid-CRC PING frames, plus the option that
allows equal source and destination addresses.

Generate an ESPHome host build using this repository's local component, then
run from the repository root (replace the include path with that build's `src`
directory):

```sh
mkdir -p work
g++ -std=c++20 -DUSE_HOST -DESPHOME_LOG_LEVEL=0 \
  -I. -I/path/to/host-build/src tests/test_reader_resync.cpp \
  -o work/test_reader_resync
work/test_reader_resync
```

Keep assertions enabled (do not define `NDEBUG`). No hardware is needed.

## Estia R32 telemetry

Generate a host source tree and run the integration checks with ESPHome 2026.9.1
and a C++20 compiler:

```sh
esphome compile --only-generate tests/estia_host.yaml
python3 tests/run_estia_telemetry.py work/estia-host/src
```

Checks cover temperature conversions, STATUS and event offsets, missing sensors,
truncated payloads, extra E8 values, zone traits, zone-only setback feedback,
and the CRC-validated receive path. The host GPIO shims replace only the existing board
setup calls; decoding and ESPHome sensor/climate publishing use production code.

Zone control checks verify CRC-valid outgoing frames, full 7–65 °C targets with
20–40 °C visuals, read-only mode, Automatik suppression, late ACK handling and
Zone-2-only mode/power sequencing. The runner uses GCC's `-fno-access-control`
only for tests to inspect the production send queue. The host shim also defines
the ESP8266 Arduino `word(...)` macro to catch naming collisions.

Configuration/default/visual override checks:

```sh
python3 tests/test_estia_config.py
esphome compile tests/estia_esp8266.yaml
```

Pump-fault checks replay the captured R410A A01/F23/clear announcements and
polled responses through checksum validation. Constructed A0 packets cover the
existing alarm layout, both dtype byte positions and explicit zero/clear; these
are software checks, not R32 hardware validation. Shared tests cover family
boundaries, raw unknown codes, startup unknown, duplicate reports, malformed or
wrong-source packets, remote-error independence, connection timeout and recovery.
All four optional-entity combinations run with read-only UART transmission checks.
Configuration generation covers both protocols and the A0 alias, either entity
alone and unsupported formats. The host and both ESP8266 configurations enable
the fault entities.
The A0 ESP8266 fixture includes Wi-Fi and the native API to compile the
text-sensor unknown-state notification; the R410A fixture also verifies a build
without networking.

GitHub Actions runs these checks and the ESP8266 build for pull requests.

Custom preset API compatibility checks (legacy traits API, current entity API,
and transitional releases exposing both):

```sh
g++ -std=c++17 -Wall -Wextra -Werror -I. tests/test_climate_compat.cpp -o /tmp/test_climate_compat
/tmp/test_climate_compat
```

The host suite also checks first-generation DN read/write frames, query
serialization, timeout recovery, reply checksums/addresses/codes, signed curve
shift values and read-only/boot guards. Configuration checks cover the optional
`curve_shift` entity and reject other frame formats. Compile its ESP8266 path with:

```sh
esphome compile tests/estia_curve_esp8266.yaml
```

First-generation status temperature checks replay the 18-byte `0x15` status
layout from issue #242. They cover byte-13 Zone 1 readings when byte 14 is empty,
byte-14 precedence when populated, empty temperature slots, absent DHW sensors,
short status frames and checksum rejection. Byte-14 compatibility is tested at
the same length; the leading `0x15` includes three framing bytes stripped by the reader.
