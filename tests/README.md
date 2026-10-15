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
