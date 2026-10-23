#!/usr/bin/env python3
"""Validate the compact paired-study bundle served by the local viewer."""
import argparse
import json
from rby1_tray.improvement_evidence import ImprovementAssets


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory")
    args = parser.parse_args()
    info = ImprovementAssets(args.directory).info()
    print(json.dumps({"verified": True, "assessment": info["assessment"],
                      "runs": {name: study["completed_runs"] for name, study in info["studies"].items()},
                      "source_commit": info["provenance"]["source_commit"]}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
