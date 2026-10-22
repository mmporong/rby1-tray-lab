"""Verify saved run hashes and trace/result agreement without MuJoCo."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path


def verify(directory):
    directory = Path(directory)
    if (directory / 'dataset-manifest.json').exists():
        return verify_dataset(directory)
    manifest = json.loads((directory / 'manifest.json').read_text())
    for name, expected in manifest['artifact_sha256'].items():
        path = directory / name
        if path.is_file():
            with path.open('rb') as stream:
                actual = hashlib.file_digest(stream, 'sha256').hexdigest()
        elif path.with_suffix(path.suffix + '.gz').is_file():
            with gzip.open(path.with_suffix(path.suffix + '.gz'), 'rb') as stream:
                actual = hashlib.file_digest(stream, 'sha256').hexdigest()
        else:
            raise ValueError(f'Missing artifact: {path}')
        if actual != expected:
            raise ValueError(f'Hash mismatch: {path}')
    root = Path(__file__).resolve().parents[1] / 'src/rby1_tray'
    for name, expected in manifest['source_sha256'].items():
        actual = hashlib.sha256((root / name).read_bytes()).hexdigest()
        if actual != expected:
            raise ValueError(f'Source mismatch: {name}')
    result = json.loads((directory / 'result.json').read_text())
    if result.get('physics_executed') is not False:
        path = directory / 'trace.jsonl'
        opener = open if path.exists() else gzip.open
        path = path if path.exists() else directory / 'trace.jsonl.gz'
        with opener(path, 'rt') as stream:
            last = None
            for line in stream:
                last = json.loads(line)
        if last is None or last['result'] != result['evaluation']:
            raise ValueError('Final trace and evaluator summary differ')
        if result['success'] and not all(result['milestones'].values()):
            raise ValueError('Successful run is missing task milestones')
    print(f'PASS {directory}')


def verify_dataset(directory):
    manifest = json.loads((directory / 'dataset-manifest.json').read_text())
    for name, expected in manifest['artifact_sha256'].items():
        actual = hashlib.sha256((directory / name).read_bytes()).hexdigest()
        if actual != expected:
            raise ValueError(f'Dataset hash mismatch: {name}')
    registered = json.loads((directory / 'preregistered.json').read_text())
    expected_pairs = {(float(height), int(seed))
                      for height in registered['heights'].split(',')
                      for seed in registered['seeds'].split(',')}
    summary = json.loads((directory / 'summary.json').read_text())
    actual_pairs = {(row['height_m'], row['seed']) for row in summary}
    if actual_pairs != expected_pairs or len(summary) != len(expected_pairs):
        raise ValueError('Dataset Cartesian set is incomplete or duplicated')
    expected_runs = {f"height-{height:.3f}-seed-{seed}" for height, seed in expected_pairs}
    if set(manifest['runs']) != expected_runs or not manifest['complete']:
        raise ValueError('Dataset run list is incomplete')
    for row in summary:
        child = directory / f"height-{row['height_m']:.3f}-seed-{row['seed']}"
        result = json.loads((child / 'result.json').read_text())
        if {k: v for k, v in row.items() if k not in ('height_m', 'seed')} != result:
            raise ValueError(f'Summary differs from child result: {child}')
        verify(child)
    print(f'PASS dataset {directory}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('directories', nargs='+')
    args = parser.parse_args()
    for directory in args.directories:
        verify(directory)
