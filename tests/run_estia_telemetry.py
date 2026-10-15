"""Compile and run the regression checks against an ESPHome generated host tree."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import argparse
import subprocess
import tempfile

parser = argparse.ArgumentParser()
parser.add_argument("generated_src", type=Path, help="Generated host build's src directory")
args = parser.parse_args()
root = args.generated_src.resolve()
tests = Path(__file__).resolve().parent
sources = [p for p in root.rglob("*.cpp")
           if p != root / "main.cpp" and p != root / "esphome/components/host/core.cpp"]
sources.append(tests / "test_estia_telemetry.cpp")
flags = ["g++", "-std=c++20", "-DUSE_HOST", "-DESPHOME_LOG_LEVEL=0",
         "-include", str(tests / "host_gpio.h"), "-I" + str(root),
         "-ffunction-sections", "-fdata-sections", "-pthread"]
with tempfile.TemporaryDirectory(prefix="estia-tests-") as temp:
    def compile_source(item):
        index, source = item
        obj = str(Path(temp) / f"{index}.o")
        subprocess.run(flags + ["-c", str(source), "-o", obj], check=True)
        return obj

    with ThreadPoolExecutor(max_workers=4) as pool:
        objects = list(pool.map(compile_source, enumerate(sources)))
    binary = str(Path(temp) / "test_estia_telemetry")
    subprocess.run(["g++", "-pthread", "-Wl,--gc-sections", *objects, "-o", binary], check=True)
    subprocess.run([binary], check=True)
