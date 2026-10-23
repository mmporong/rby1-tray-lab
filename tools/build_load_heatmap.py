#!/usr/bin/env python3
"""Export recorded torque/contact plots (optional Matplotlib export environment)."""
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import numpy as np


def build(run: Path) -> dict:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from rby1_tray.planning import Robot
    from rby1_tray.scene import build_scene

    trace = run / "trace.jsonl.gz"
    opener = gzip.open if trace.exists() else open
    if not trace.exists():
        trace = run / "trace.jsonl"
    rows = []
    with opener(trace, "rt") as stream:
        for line in stream:
            row = json.loads(line)
            if rows and row["time_s"] < rows[-1]["time_s"]:
                raise ValueError("trace timestamps must be monotonic")
            if not rows or row["time_s"] > rows[-1]["time_s"]:
                rows.append(row)
    if not rows:
        raise ValueError("recorded trace is empty")
    config = json.loads((run / "config.json").read_text())
    model, _ = build_scene(config)
    robot = Robot(model)
    if not np.allclose(model.actuator_gear[robot.act, 0], 1):
        raise ValueError("torque plot requires unit joint actuator gear")
    limits = robot.force_limit
    torques = np.asarray([row["joint_state"]["actuator_force"] for row in rows])
    times = np.asarray([row["time_s"] for row in rows])
    if torques.shape != (len(rows), len(robot.names)) or not np.all(np.isfinite(torques)):
        raise ValueError("recorded joint torques are incomplete or nonfinite")
    utilization = np.abs(torques) / limits
    result = json.loads((run / "result.json").read_text())
    full_step_peak = float(result["metrics"]["max_joint_torque_nm"])
    if not np.isfinite(full_step_peak) or full_step_peak < 0:
        raise ValueError("full-step peak torque must be finite and nonnegative")

    plt.style.use("dark_background")
    figure, axes = plt.subplots(
        2, 1, figsize=(14, 9), layout="constrained",
        gridspec_kw={"height_ratios": [4, 1.1]},
    )
    figure.set_facecolor("#101820")
    image = axes[0].pcolormesh(
        times, np.arange(len(robot.names)) + .5, utilization.T,
        shading="nearest", cmap="turbo", vmin=0, vmax=1, rasterized=True,
    )
    axes[0].set_yticks(
        np.arange(len(robot.names)) + .5,
        [f"{name}  ({limit:g} Nm model limit)" for name, limit in zip(robot.names, limits)],
        fontsize=8,
    )
    axes[0].set_title("RB-Y1 simulated joint torque / model actuator limit", fontsize=14, loc="left")
    axes[0].set_xlabel("Simulation time (s)")
    figure.colorbar(image, ax=axes[0], label="Model limit fraction (not a hardware safety rating)")
    for side, color in (("left", "#43d7e6"), ("right", "#ffb454")):
        axes[1].plot(times, [row["grasp_force_n"][side] for row in rows],
                     label=f"{side.title()} grasp normal force", color=color)
    axes[1].set(
        xlabel="Simulation time (s)", ylabel="Force (N)",
        title="Native finger contact normal force; no contact-pressure inference",
    )
    axes[1].legend(fontsize=8)
    for label, key in (("grasp", "grasped"), ("lift", "lifted"), ("seat", "seated"), ("home", "returned_home")):
        time = result["milestone_times_s"].get(key)
        if time is None:
            continue
        for axis in axes:
            axis.axvline(time, color="#d4dce2", lw=.6, alpha=.5)
        axes[0].annotate(
            label, (time, 1), xycoords=("data", "axes fraction"),
            xytext=(-3, -3), textcoords="offset points", fontsize=8,
            ha="right", va="top", bbox={"facecolor": "#101820", "edgecolor": "none", "alpha": .85},
        )
    figure.savefig(run / "load-heatmap.png", dpi=150)
    figure.savefig(run / "load-heatmap.svg")
    plt.close(figure)

    peak = np.unravel_index(np.argmax(utilization), utilization.shape)
    metadata = {
        "source_commit": json.loads((run / "manifest.json").read_text())["commit"],
        "scope": "rigid-body actuator torque and contact-force visualization; not structural stress or fatigue analysis",
        "units": {"joint_actuator_force": "Nm, verified unit gear revolute actuator", "grasp_contact_normal_force": "N"},
        "joint_names": robot.names,
        "model_torque_limits_nm": limits.tolist(),
        "peak_utilization": float(utilization[peak]),
        "peak_utilization_joint": robot.names[peak[1]],
        "peak_utilization_time_s": float(times[peak[0]]),
        "sampled_peak_abs_joint_torque_nm": float(np.max(np.abs(torques))),
        "full_step_peak_abs_joint_torque_nm": full_step_peak,
        "sampling_note": "50 Hz trace heatmap; full-step peak separately recorded by 500 Hz evaluator",
        "model_limit_provenance": "fixed Menagerie actuator force limits, not measured hardware ratings",
        "input": trace.name,
        "sample_count": len(rows),
        "contact_pressure_available": False,
        "structural_stress_available": False,
        "exporter_script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "matplotlib_version": matplotlib.__version__,
        "svg_heatmap_rasterized": True,
    }
    (run / "load-analysis.json").write_text(json.dumps(metadata, indent=2) + "\n")
    return metadata


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    print(json.dumps(build(parser.parse_args().run), indent=2))
