"""Generate real ESPHome configurations to verify zone defaults and overrides."""
from pathlib import Path
import re
import subprocess
import sys
import tempfile

import yaml

root = Path(__file__).resolve().parents[1]
work = root / "work"
work.mkdir(exist_ok=True)
cases = [
    ({}, [(20, 65)]),
    ({"zone1_climate": False}, []),
    ({"zone2_climate": True}, [(20, 40), (20, 65)]),
    ({"zone1_climate": False, "zone2_climate": True}, [(20, 40)]),
    ({"zone2_climate": {"visual": {"min_temperature": 7, "max_temperature": 65}},
      "zone1_climate": {"name": "Custom Zone 1", "visual": {"min_temperature": 18, "max_temperature": 45}}},
     [(7, 65), (18, 45)]),
    ({"zone1_climate": {"supports_cool": False, "supports_auto": False}}, [(20, 65)]),
    ({"zone2_climate": {"supports_cool": False, "supports_auto": False}}, [(20, 40), (20, 65)]),
]
with tempfile.TemporaryDirectory(prefix="estia-config-", dir=work) as temp:
    temp = Path(temp)
    for index, (zones, expected) in enumerate(cases):
        build = temp / f"build-{index}"
        config = {
            "esphome": {"name": "estia-config-test", "build_path": str(build)},
            "host": {}, "logger": {"baud_rate": 0},
            "external_components": [{"source": {"type": "local", "path": str(root / "components")}}],
            "uart": {"port": "/dev/null", "baud_rate": 2400, "parity": "EVEN"},
            "climate": [{"platform": "toshiba_ab", "name": "DHW", "frame_format": "a0", **zones}],
        }
        path = temp / f"case-{index}.yaml"
        path.write_text(yaml.safe_dump(config))
        subprocess.run([sys.executable, "-m", "esphome", "compile", "--only-generate", str(path)], check=True)
        source = (build / "src/main.cpp").read_text()
        if any(isinstance(value, dict) and value.get("supports_cool") is False for value in zones.values()):
            assert "set_supports_cool(false)" in source
            assert "set_supports_auto(false)" in source
        values = {}
        for entity, bound, value in re.findall(
            r"(\w+)->set_visual_(min|max)_temperature_override\(([\d.]+)f?\);", source
        ):
            if "estiazoneclimate" in entity:
                values.setdefault(entity, {})[bound] = float(value)
        assert [(v["min"], v["max"]) for v in values.values()] == expected, values
    for framing in ["estia", "a0", "normal", "auto"]:
        config["climate"] = [{"platform": "toshiba_ab", "name": "DHW",
                              "frame_format": framing, "curve_shift": {"name": "Curve Shift"}}]
        path = temp / f"curve-{framing}.yaml"
        path.write_text(yaml.safe_dump(config))
        result = subprocess.run([sys.executable, "-m", "esphome", "compile", "--only-generate", str(path)],
                                capture_output=True, text=True)
        if framing == "estia":
            assert result.returncode == 0, result.stderr
            source = (build / "src/main.cpp").read_text()
            assert 'set_curve_shift_select(' in source
            assert '{"-5", "-4", "-3", "-2", "-1", "0", "1", "2", "3", "4", "5"}' in source
        else:
            assert result.returncode != 0
            assert "curve_shift requires frame_format: estia" in result.stdout + result.stderr
print("Estia configuration checks passed")
