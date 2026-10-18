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

GitHub Actions runs these checks and the ESP8266 build for pull requests.

Custom preset API compatibility checks (legacy traits API, current entity API,
and transitional releases exposing both):

```sh
g++ -std=c++17 -Wall -Wextra -Werror -I. tests/test_climate_compat.cpp -o /tmp/test_climate_compat
/tmp/test_climate_compat
```
