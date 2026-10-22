import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import sys
import time


def rendering_environment():
    if not os.environ.get("DISPLAY"):
        os.environ.setdefault("MUJOCO_GL", "egl")
        os.environ.setdefault("PYOPENGL_PLATFORM", "egl")
        os.environ.setdefault("EGL_PLATFORM", "surfaceless")
        mesa = "/usr/share/glvnd/egl_vendor.d/50_mesa.json"
        if Path(mesa).exists(): os.environ.setdefault("__EGL_VENDOR_LIBRARY_FILENAMES", mesa)


def save_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def source_hashes():
    root = Path(__file__).parent
    paths = sorted([*root.glob("*.py"), *root.glob("web/*.html")])
    return {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}


_LOADED_SOURCES = source_hashes()


def write_manifest(output, expected_sources):
    import importlib.metadata
    import subprocess
    repository = Path(__file__).resolve().parents[2]
    commit = subprocess.run(["git", "-C", str(repository), "rev-parse", "HEAD"], capture_output=True, text=True)
    if source_hashes() != expected_sources:
        raise RuntimeError("Source changed during the experiment; rerun from a fixed revision")
    manifest = {
        "python": platform.python_version(),
        "versions": {name: importlib.metadata.version(name) for name in ("mujoco", "numpy", "scipy", "Pillow")},
        "commit": commit.stdout.strip() if commit.returncode == 0 else "uncommitted-initial-development",
        "render_backend": os.environ.get("MUJOCO_GL", "none"),
        "egl_vendor": os.environ.get("__EGL_VENDOR_LIBRARY_FILENAMES"),
        "source_sha256": expected_sources,
        "artifact_sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(output.iterdir()) if p.is_file() and p.name != "manifest.json"},
    }
    save_json(output / "manifest.json", manifest)


def run_episode(config, output, render=False, lift_only=False):
    if source_hashes() != _LOADED_SOURCES:
        raise RuntimeError("Source changed since process startup; restart the experiment")
    rendering_environment()
    import mujoco
    import numpy as np
    from PIL import Image
    from .assets import asset_directory, provenance
    from .session import Session
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    expected_sources = _LOADED_SOURCES
    save_json(output / "config.json", config)
    try:
        session = Session(config, lift_only=lift_only)
    except ValueError as error:
        if "No feasible IK candidate" not in str(error):
            raise
        from .scene import build_scene, audit_model
        model, xml = build_scene(config)
        save_json(output / "model-audit.json", audit_model(model))
        save_json(output / "upstream.json", provenance(asset_directory()))
        (output / "scene.xml").write_text(xml.replace(str(asset_directory() / "assets"), "assets"))
        (output / "assets").symlink_to(asset_directory() / "assets", target_is_directory=True)
        result = {"success": False, "status": "planning_infeasible", "failure_reason": str(error),
                  "simulation_time_s": 0.0, "physics_executed": False}
        save_json(output / "result.json", result)
        write_manifest(output, expected_sources)
        return result
    save_json(output / "model-audit.json", session.audit)
    save_json(output / "ik.json", session.ik_errors)
    save_json(output / "upstream.json", provenance(asset_directory()))
    portable_xml = session.xml.replace(str(asset_directory() / "assets"), "assets")
    (output / "scene.xml").write_text(portable_xml)
    (output / "assets").symlink_to(asset_directory() / "assets", target_is_directory=True)
    frames = []
    try:
        while not session.finished:
            session.step(250)
            state = session.snapshot()
            if render:
                frame = session.render(640, 480, 135, -20)
                if float(np.std(frame)) < 1:
                    raise RuntimeError("Renderer produced an empty frame")
                frames.append(Image.fromarray(frame))
            if abs(state["time_s"] % 2) < .01 or session.finished:
                print(json.dumps({"time_s": round(state["time_s"], 2), "phase": state["phase"],
                                  "tray_z_m": state["tray"]["pos_m"][2], "result": state["result"]}, ensure_ascii=False), flush=True)
        if not lift_only:
            session.evaluator.finalize(float(session.data.time))
        final_snapshot = session.snapshot()
        final_snapshot["result"] = session.evaluator.result()
        session.log.append(final_snapshot)
        with (output / "trace.jsonl").open("w") as stream:
            for row in session.log: stream.write(json.dumps(row, allow_nan=False) + "\n")
        evaluation = session.evaluator.result()
        result = dict(evaluation)
        result["evaluation"] = evaluation
        result["run_completed"] = True
        result.update(wall_time_s=time.monotonic() - started, simulation_time_s=session.data.time,
                      metrics=session.snapshot()["metrics"], highest_tray_center_z_m=max(row["tray"]["pos_m"][2] for row in session.log),
                      task="grasp_lift_feasibility" if lift_only else "floor_to_container",
                      modeling_scope="fixed-base simulation only; ground-truth state")
        if lift_only:
            clearance = final_snapshot["tray"]["bottom_z_m"] - session.log[0]["tray"]["bottom_z_m"]
            result["feasibility_pass"] = bool(result["milestones"]["grasped"] and result["milestones"]["lifted"] and result["failure_reason"] is None and clearance >= .05)
            result["lift_clearance_m"] = clearance
            result["assessment"] = {"scope": "grasp_lift_only", "success": result["feasibility_pass"], "full_task_success": False}
        save_json(output / "result.json", result)
        if frames:
            frames[0].save(output / "initial.png")
            frames[-1].save(output / "final.png")
            frames[0].save(output / "rollout.gif", save_all=True, append_images=frames[1:], duration=125, loop=0)
        write_manifest(output, expected_sources)
        return result
    finally:
        session.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description="RB-Y1 loaded-tray physics experiments")
    parser.add_argument("command", choices=["fetch-assets", "audit", "plan", "run", "view", "benchmark"])
    parser.add_argument("--config")
    parser.add_argument("--output")
    parser.add_argument("--render", action="store_true")
    parser.add_argument("--lift-only", action="store_true")
    parser.add_argument("--port", type=int, default=8786)
    parser.add_argument("--heights", default="0.45,0.65,0.85")
    parser.add_argument("--seeds", default="0,1,2")
    args = parser.parse_args(argv)
    from .assets import asset_directory
    if args.command == "fetch-assets":
        directory = asset_directory(fetch=True)
        if args.output:
            destination = Path(args.output) / "assets"
            if destination.is_symlink() and destination.resolve() != directory / "assets":
                destination.unlink()
            if not destination.exists(): destination.symlink_to(directory / "assets", target_is_directory=True)
        print(directory); return 0
    rendering_environment()
    from .scene import load_config, build_scene, audit_model
    config = load_config(args.config)
    if args.command == "audit":
        model, _ = build_scene(config)
        print(json.dumps(audit_model(model), indent=2)); return 0
    if args.command == "plan":
        from .planning import build_plan
        model, _ = build_scene(config)
        _, _, errors = build_plan(model, config, lift_only=args.lift_only)
        print(json.dumps(errors, indent=2)); return 0 if max(e["position_error_m"] for e in errors) < .005 else 1
    if args.command == "view":
        from .session import Session
        from .viewer import serve
        session = Session(config, lift_only=args.lift_only)
        try: serve(session, port=args.port)
        finally: session.close()
        return 0
    if args.command == "run":
        if not args.output: parser.error("--output is required; existing directories are never overwritten")
        result = run_episode(config, args.output, args.render, args.lift_only)
        print(json.dumps(result, indent=2))
        return 0 if result.get("feasibility_pass", result["success"]) else 1
    if args.command == "benchmark":
        import copy
        if not args.output: parser.error("--output is required")
        output = Path(args.output)
        output.mkdir(parents=True, exist_ok=False)
        summaries = []
        save_json(output / "preregistered.json", vars(args))
        for height in map(float, args.heights.split(",")):
            for seed in map(int, args.seeds.split(",")):
                cfg = copy.deepcopy(config)
                cfg["seed"], cfg["target"]["support_z_m"] = seed, height
                destination = output / f"height-{height:.3f}-seed-{seed}"
                result = run_episode(cfg, destination)
                summaries.append({"height_m": height, "seed": seed, **result})
                save_json(output / "summary.json", summaries)
        runs = [f"height-{row['height_m']:.3f}-seed-{row['seed']}" for row in summaries]
        files = ["preregistered.json", "summary.json", *[f"{run}/manifest.json" for run in runs]]
        save_json(output / "dataset-manifest.json", {
            "complete": True, "runs": runs,
            "artifact_sha256": {name: hashlib.sha256((output / name).read_bytes()).hexdigest() for name in files}})
        return 0 if all(row["success"] for row in summaries) else 1


if __name__ == "__main__":
    sys.exit(main())
