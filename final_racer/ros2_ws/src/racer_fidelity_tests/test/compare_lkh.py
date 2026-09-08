#!/usr/bin/env python3
"""Compare original-baseline and ported LKH tours for one exact ATSP input."""

from pathlib import Path
import re
import subprocess
import sys
import tempfile


MATRIX = (
    (0, 41, 73, 29, 58, 64),
    (34, 0, 18, 67, 45, 31),
    (72, 21, 0, 36, 27, 55),
    (30, 65, 38, 0, 19, 43),
    (57, 46, 26, 17, 0, 22),
    (63, 32, 54, 44, 20, 0),
)


def solve(executable: str, directory: Path):
    problem = directory / "fixture.atsp"
    tour = directory / "fixture.tour"
    parameter = directory / "fixture.par"
    matrix = "\n".join(" ".join(map(str, row)) for row in MATRIX)
    problem.write_text(
        "NAME : fidelity_fixture\nTYPE : ATSP\nDIMENSION : 6\n"
        "EDGE_WEIGHT_TYPE : EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\n"
        f"EDGE_WEIGHT_SECTION\n{matrix}\nEOF\n")
    parameter.write_text(
        "SPECIAL\n"
        f"PROBLEM_FILE = {problem}\nSALESMEN = 1\n"
        "MTSP_OBJECTIVE = MINSUM\nRUNS = 1\nSEED = 0\nTRACE_LEVEL = 0\n"
        f"TOUR_FILE = {tour}\n")
    subprocess.run([executable, str(parameter)], check=True,
                   stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    content = tour.read_text()
    length = re.search(r"COMMENT : Length = (-?\d+)", content)
    section = content.split("TOUR_SECTION\n", 1)[1].split("EOF", 1)[0]
    return int(length.group(1)), tuple(int(value) for value in section.split())


with tempfile.TemporaryDirectory(prefix="racer_lkh_fidelity_") as root:
    root = Path(root)
    baseline_dir = root / "baseline"
    ported_dir = root / "ported"
    baseline_dir.mkdir()
    ported_dir.mkdir()
    baseline = solve(sys.argv[1], baseline_dir)
    ported = solve(sys.argv[2], ported_dir)
if baseline != ported:
    raise SystemExit(f"same-input LKH output differs: baseline={baseline}, port={ported}")
print(f"PASS: ROS1 baseline and ROS2 port LKH output identical: {ported}")
