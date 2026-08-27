#!/usr/bin/env python3
"""Verify that the vendored ROS1 C2 algorithm has only audited port edits."""

from hashlib import sha256
from pathlib import Path
import sys


WORKSPACE = Path(__file__).resolve().parents[1]
BASELINE = WORKSPACE.parent / "C2-Explorer" / "src" / "swarm_exploration"
PORT = WORKSPACE / "src" / "c2_explorer_core" / "upstream"

# Every allowed algorithm-source difference is pinned on both sides.  Updating
# either source requires an explicit review and an update to PORTING_CHANGES.md.
ALLOWED = {
    "active_perception/src/frontier_finder.cpp": (
        "e92dc66e532a7a947eed5ce5674ea7c7f124d35f38153156e613b54ea8a6dbbc",
        "f031c770dc21e2710734a7b2aab06db31f1bb20baf67a7019978c10ec2e7d61b",
    ),
    "plan_env/src/sdf_map.cpp": (
        "38c7b085d2246c2bafea57a567b300d9dd6d94396574375e0794b2a863478812",
        "1cef72aead8d1c1e5254a90df321481aa5a474983708c8653ccab7d018f657f3",
    ),
    "plan_env/src/multi_map_manager.cpp": (
        "34582743e3cb8e108bf1c7ed0a3a8b9bcdae66744920b0de8f00f7fa1b21bd10",
        "bf1d3a86cf20aa4b8375017017d97656854a0a689b1f53c6bf3ef4deaf882a95",
    ),
    "exploration_manager/src/c2_exploration_fsm.cpp": (
        "45f9dfc53da9feb37cdad432dd7878352e8f7aed0fbf1ae8496104cbd6713827",
        "3d95b3ade8d1fbfd0fe524931f2be48a046a90f451f81d0fa6bbcc8d621fe1fd",
    ),
    "exploration_manager/src/c2_exploration_manager.cpp": (
        "99d5bacc1b872c023ac4567cf9fb630d3257734604f41784d115c0772720cac2",
        "ca8cf719e9bb6eecbb4af96c33881c2e5fd36418bb2606f373c32b1592f25874",
    ),
}


def files(root: Path) -> dict[str, Path]:
    return {
        path.relative_to(root).as_posix(): path
        for path in root.rglob("*")
        if path.is_file()
    }


def digest(path: Path) -> str:
    return sha256(path.read_bytes()).hexdigest()


def main() -> int:
    if not BASELINE.is_dir():
        print(f"baseline source missing: {BASELINE}", file=sys.stderr)
        return 2
    baseline_files = files(BASELINE)
    port_files = files(PORT)
    errors: list[str] = []
    if baseline_files.keys() != port_files.keys():
        missing = sorted(baseline_files.keys() - port_files.keys())
        extra = sorted(port_files.keys() - baseline_files.keys())
        errors.extend(f"missing: {name}" for name in missing)
        errors.extend(f"extra: {name}" for name in extra)
    for name in sorted(baseline_files.keys() & port_files.keys()):
        base_hash = digest(baseline_files[name])
        port_hash = digest(port_files[name])
        if name in ALLOWED:
            if (base_hash, port_hash) != ALLOWED[name]:
                errors.append(
                    f"unaudited change: {name}\n"
                    f"  baseline {base_hash}\n  port     {port_hash}"
                )
        elif base_hash != port_hash:
            errors.append(f"unexpected source difference: {name}")
    if errors:
        print("C2 source fidelity check FAILED", file=sys.stderr)
        print("\n".join(errors), file=sys.stderr)
        return 1
    exact = len(port_files) - len(ALLOWED)
    print(
        f"C2 source fidelity check passed: {len(port_files)} files, "
        f"{exact} byte-identical, {len(ALLOWED)} audited boundary/guard edits"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
