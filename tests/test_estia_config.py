"""Generate real ESPHome configurations to verify zones and fault entities."""
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
    for framing in ["estia", "a0", "estia_a0", "normal", "hm", "tu2c", "auto"]:
        for entity_keys in [("fault_code",), ("fault_active",), ("fault_code", "fault_active")]:
            config["climate"] = [{"platform": "toshiba_ab", "name": "DHW", "frame_format": framing,
                                  **{key: {"name": key} for key in entity_keys}}]
            path = temp / f"fault-{framing}-{'-'.join(entity_keys)}.yaml"
            path.write_text(yaml.safe_dump(config))
            result = subprocess.run([sys.executable, "-m", "esphome", "compile", "--only-generate", str(path)],
                                    capture_output=True, text=True)
            if framing in ("estia", "a0", "estia_a0"):
                assert result.returncode == 0, result.stdout + result.stderr
                source = (build / "src/main.cpp").read_text()
                assert ("set_fault_code_text_sensor(" in source) == ("fault_code" in entity_keys)
                assert ("set_fault_active_binary_sensor(" in source) == ("fault_active" in entity_keys)
            else:
                assert result.returncode != 0
                assert "fault_code and fault_active require frame_format: estia or a0" in result.stdout + result.stderr
    poll_cases = [
        ("estia", {"fault_code": {"name": "Fault"}}, "30s", True),
        ("estia", {"fault_active": {"name": "Fault"}}, "never", True),
        ("estia", {}, "30s", False),
        ("a0", {"fault_code": {"name": "Fault"}}, "30s", False),
        ("estia", {"fault_code": {"name": "Fault"}}, "0s", False),
        ("estia", {"fault_code": {"name": "Fault"}}, "500ms", False),
    ]
    for index, (framing, entities, interval, valid) in enumerate(poll_cases):
        config["climate"] = [{"platform": "toshiba_ab", "name": "DHW", "frame_format": framing,
                              "fault_poll_interval": interval, **entities}]
        path = temp / f"fault-poll-{index}.yaml"
        path.write_text(yaml.safe_dump(config))
        result = subprocess.run([sys.executable, "-m", "esphome", "compile", "--only-generate", str(path)],
                                capture_output=True, text=True)
        assert (result.returncode == 0) == valid, result.stdout + result.stderr
        if valid:
            assert "set_fault_poll_interval(" in (build / "src/main.cpp").read_text()
print("Estia configuration checks passed")
