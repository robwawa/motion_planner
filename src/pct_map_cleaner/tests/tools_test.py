#!/usr/bin/env python3
"""Test configuration compatibility, comparison metrics and benchmark helpers."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile

spec = importlib.util.spec_from_file_location('benchmark', Path(__file__).resolve().parents[1] / 'tools/benchmark.py')
benchmark = importlib.util.module_from_spec(spec)
spec.loader.exec_module(benchmark)
cleaner, compare = sys.argv[1:]


def invoke(args, ok=True):
    result = subprocess.run(args, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if (result.returncode == 0) != ok:
        raise AssertionError(result.stdout + result.stderr)
    return result


def write_pcd(path, points):
    path.write_text('VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                    f'WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA ascii\n' +
                    ''.join(f'{x} {y} {z}\n' for x, y, z in points))


with tempfile.TemporaryDirectory(prefix='pct-cleaner-tools-') as temp:
    root = Path(temp)
    config = root / 'config.yaml'
    original = ''.join(section + ':\n  enable: false\n' for section in benchmark.STAGES[1:])
    config.write_text(original)
    input_path = root / 'input.pcd'
    write_pcd(input_path, [(0, 0, 0), (1, 0, 0)])
    output = root / 'output.pcd'
    command = [cleaner, '--config', str(config), '--input', str(input_path), '--output', str(output)]
    report = invoke(command)
    assert 'sor backend: disabled' in report.stdout
    assert benchmark.xyz_info(output)['points'] == 2
    # Missing backend defaults to legacy and disabled detail timers are absent.
    config.write_text(benchmark.set_yaml(original, 'sor', 'enable', 'true'))
    assert 'sor backend: legacy' in invoke(command).stdout
    config.write_text(benchmark.set_yaml(config.read_text(), 'sor', 'search_backend', 'single_exact'))
    assert 'sor backend: single_exact' in invoke(command).stdout
    config.write_text(benchmark.set_yaml(original, 'sor', 'search_backend', 'typo'))
    assert 'sor.search_backend' in invoke(command, ok=False).stderr
    config.write_text(benchmark.set_yaml(original, 'performance', 'threads', '-1'))
    assert 'threads must be >= 0' in invoke(command, ok=False).stderr
    other = root / 'other.pcd'
    write_pcd(other, [(0, 0, 0), (2, 0, 0)])
    prefix = root / 'comparison'
    result = json.loads(invoke([compare, str(output), str(other), str(prefix), '2']).stdout)
    assert result['old_only_points'] == result['new_only_points'] == 1
    assert result['old_to_new']['max_m'] == result['new_to_old']['max_m'] == 1
    assert result['old_to_new']['p50_m'] == 0.5
    assert result['old_to_new']['fraction_gt_0_05m'] == 0.5
    invoke([compare, str(output), str(other), str(prefix)], ok=False)  # No overwriting evidence.
    identical = json.loads(invoke([compare, str(output), str(output), str(root/'same')]).stdout)
    assert identical['old_only_points'] == identical['new_only_points'] == 0
    assert identical['old_to_new']['max_m'] == 0
    empty = root/'empty.pcd'
    write_pcd(empty, [])
    empty_result = json.loads(invoke([compare, str(output), str(empty), str(root/'empty-result')]).stdout)
    assert empty_result['old_to_new']['unmatched_points'] == 2
    assert empty_result['old_to_new']['max_m'] is None
    config.write_text(original)
    for mode in ('quality', 'timing'):
        benchmark_root = root / ('benchmark-' + mode)
        invoke([sys.executable, str(Path(benchmark.__file__)), '--baseline', cleaner,
                '--optimized', cleaner, '--config', str(config), '--input', str(input_path),
                '--output-dir', str(benchmark_root), '--threads', '1', '--repeats', '1', '--mode', mode])
        summary = json.loads((benchmark_root / 'summary.json').read_text())
        if mode == 'quality':
            assert summary['passed']
        else:
            assert len(summary) == 3
            assert json.loads((benchmark_root / 'equality.json').read_text())['passed']
print('CLI and offline tool tests passed')
