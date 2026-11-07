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
    ({"zone1_climate": {"supports_heat": False}}, [(20, 65)]),
    ({"zone2_climate": {"supports_heat": False, "supports_cool": False, "supports_auto": False}},
     [(20, 40), (20, 65)]),
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
        for option in ("supports_heat", "supports_cool", "supports_auto"):
            configured = [zones.get(name, name == "zone1_climate")
                          for name in ("zone2_climate", "zone1_climate")]
            enabled = [value.get(option, True) if isinstance(value, dict) else True
                       for value in configured if value is not False]
            assert source.count(f"set_{option}(true)") == enabled.count(True), option
            assert source.count(f"set_{option}(false)") == enabled.count(False), option
        values = {}
        for entity, bound, value in re.findall(
            r"(\w+)->set_visual_(min|max)_temperature_override\(([\d.]+)f?\);", source
        ):
            if "estiazoneclimate" in entity:
                values.setdefault(entity, {})[bound] = float(value)
        assert [(v["min"], v["max"]) for v in values.values()] == expected, values
    indicators = (
        "waterpump_active", "heating_cooling_compressor_active", "heating_resistor_active",
        "dhw_compressor_active", "dhw_resistor_active", "heating_external_booster_active",
        "test_mode_active",
    )
    config["climate"] = [{
        "platform": "toshiba_ab", "name": "DHW", "frame_format": "estia",
        **{option: {"name": option} for option in indicators},
        "dhw_boost": {"name": "DHW Boost", "id": "dhw_boost_switch"},
        "antibacteria": {"name": "Antibacteria", "id": "antibacteria_switch"},
        "frost_protection": {"name": "Frost Protection", "id": "frost_protection_switch"},
        "night_setback": {"name": "Night Setback", "id": "night_setback_switch"},
        "silent_mode": {"name": "Silent Mode", "id": "silent_mode_switch"},
    }]
    path = temp / "activity.yaml"
    path.write_text(yaml.safe_dump(config))
    subprocess.run([sys.executable, "-m", "esphome", "compile", "--only-generate", str(path)], check=True)
    source = (build / "src/main.cpp").read_text()
    for option in indicators:
        assert f"set_{option}_binary_sensor(" in source, option
    for option in ("dhw_boost", "antibacteria", "frost_protection", "night_setback", "silent_mode"):
        assert f"set_{option}_switch({option}_switch)" in source, option

    for framing in ("a0", "normal", "auto"):
        config["climate"] = [{"platform": "toshiba_ab", "name": "DHW", "frame_format": framing,
                              "silent_mode": {"name": "Silent Mode"}}]
        path = temp / f"silent-{framing}.yaml"
        path.write_text(yaml.safe_dump(config))
        result = subprocess.run([sys.executable, "-m", "esphome", "compile", "--only-generate", str(path)],
                                capture_output=True, text=True)
        assert result.returncode != 0
        assert "silent_mode requires frame_format: estia" in result.stdout + result.stderr

    for legacy, canonical in (
        ("hotwater_pump_heating", "dhw_compressor_active"),
        ("hotwater_resistor_heating", "dhw_resistor_active"),
    ):
        config["climate"] = [{
            "platform": "toshiba_ab", "name": "DHW", "frame_format": "estia",
            legacy: {"name": "Existing DHW Indicator", "id": "existing_dhw_indicator"},
        }]
        path = temp / f"alias-{legacy}.yaml"
        path.write_text(yaml.safe_dump(config))
        subprocess.run([sys.executable, "-m", "esphome", "compile", "--only-generate", str(path)], check=True)
        source = (build / "src/main.cpp").read_text()
        assert f"set_{canonical}_binary_sensor(existing_dhw_indicator)" in source
        assert source.count(f"set_{canonical}_binary_sensor(") == 1
        config["climate"][0][canonical] = {"name": "Duplicate DHW Indicator"}
        path.write_text(yaml.safe_dump(config))
        result = subprocess.run([sys.executable, "-m", "esphome", "compile", "--only-generate", str(path)],
                                capture_output=True, text=True)
        assert result.returncode != 0
        assert legacy in result.stdout + result.stderr and canonical in result.stdout + result.stderr

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
    for framing, explicit, expected in [
        ("tu2c", None, 5), ("u", None, 5), ("wrapped", None, 5),
        ("tu2c", 3, 3), ("tu2c", 5, 5), ("normal", None, 3),
        ("auto", None, 3), ("hm", None, 3), ("a0", None, 3), ("estia", None, 3),
        ("normal", 3, 3), ("normal", 5, None), ("auto", 5, None),
        ("hm", 5, None), ("a0", 5, None), ("estia", 5, None), ("tu2c", 4, None),
    ]:
        entity = {"platform": "toshiba_ab", "name": "Fan test", "frame_format": framing}
        if explicit is not None:
            entity["fan_levels"] = explicit
        config["climate"] = [entity]
        config["uart"]["parity"] = "NONE" if framing in ("tu2c", "u", "wrapped") else "EVEN"
        path = temp / "fan.yaml"
        path.write_text(yaml.safe_dump(config))
        result = subprocess.run([sys.executable, "-m", "esphome", "compile", "--only-generate", str(path)],
                                capture_output=True, text=True)
        if expected is None:
            assert result.returncode != 0, (framing, explicit)
            assert "fan_levels" in result.stdout + result.stderr
        else:
            assert result.returncode == 0, result.stderr
            source = (build / "src/main.cpp").read_text()
            assert f"set_fan_levels({expected})" in source, (framing, explicit)
print("Estia configuration checks passed")
