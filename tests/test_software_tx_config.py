"""Exercise the real UART final-validation/codegen selection on ESP8266."""
from pathlib import Path
import subprocess
import sys
import tempfile

import yaml

root = Path(__file__).resolve().parents[1]
work = root / "work"
work.mkdir(exist_ok=True)
with tempfile.TemporaryDirectory(prefix="software-tx-config-", dir=work) as temp:
    temp = Path(temp)
    for index, (tx, rx, parity, framing, explicit_rx, expected) in enumerate([
        (12, 13, "EVEN", "normal", None, True),
        (12, 13, "NONE", "tu2c", None, True),
        (12, 13, "ODD", "a0", None, True),
        (15, 13, "EVEN", "normal", None, False),
        (4, 5, "EVEN", "normal", None, False),
        (14, None, "EVEN", "estia", 3, True),
        (10, 13, "EVEN", "normal", None, False),
    ]):
        build = temp / f"build-{index}"
        config = {
            "esphome": {"name": "software-tx-test", "build_path": str(build)},
            "esp8266": {"board": "esp12e"}, "logger": {"baud_rate": 0},
            "external_components": [{"source": {"type": "local", "path": str(root / "components")}}],
            "uart": {"tx_pin": {"number": tx, "inverted": index == 2},
                     "baud_rate": 2400, "parity": parity},
            "climate": [{"platform": "toshiba_ab", "name": "UART test", "frame_format": framing}],
        }
        if rx is not None:
            config["uart"]["rx_pin"] = rx
        if explicit_rx is not None:
            config["climate"][0]["hardware_uart_rx_pin"] = explicit_rx
        path = temp / f"case-{index}.yaml"
        path.write_text(yaml.safe_dump(config))
        subprocess.run([sys.executable, "-m", "esphome", "compile", "--only-generate", str(path)], check=True)
        source = (build / "src/main.cpp").read_text()
        defines = (build / "src/esphome/core/defines.h").read_text()
        ini = (build / "platformio.ini").read_text()
        assert ("set_interruptible_tx_pin(" in source) == expected
        assert ("USE_TOSHIBA_AB_INTERRUPTIBLE_TX" in defines) == expected
        assert ("EspSoftwareSerial" in ini) == expected
        if expected:
            assert f"set_interruptible_tx_config(EspSoftwareSerial::SWSERIAL_8{parity[0]}1)" in source
            assert "set_hardware_uart_rx_pin(" in source
        if index == 2:
            assert "set_inverted(true)" in source
print("Software TX configuration checks passed")
