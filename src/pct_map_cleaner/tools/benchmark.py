#!/usr/bin/env python3
"""Independent-process benchmark and stage-prefix equality checks (stdlib only)."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import statistics
import subprocess
import time

STAGES = ['finite_filter', 'pre_sampling', 'sor', 'ror', 'floating_cluster',
          'uniform_sampling', 'ground_completion']


def set_yaml(text, section, key, value):
    lines = text.splitlines()
    start = next((i for i, line in enumerate(lines) if line == section + ':'), None)
    if start is None:
        return text + '\n' + section + ':\n  ' + key + ': ' + str(value) + '\n'
    end = next((i for i in range(start + 1, len(lines))
                if re.match(r'^[A-Za-z_]\w*:', lines[i])), len(lines))
    for i in range(start + 1, end):
        if re.match(r'^  ' + re.escape(key) + r'\s*:', lines[i]):
            lines[i] = '  ' + key + ': ' + str(value)
            break
    else:
        lines.insert(start + 1, '  ' + key + ': ' + str(value))
    return '\n'.join(lines) + '\n'


def xyz_info(path):
    # Cleaner output is uncompressed float32 XYZ binary, or empty ASCII.
    with path.open('rb') as stream:
        header = {}
        while True:
            line = stream.readline()
            if not line:
                raise ValueError('PCD header missing DATA: ' + str(path))
            fields = line.decode('ascii').strip().split()
            if not fields or fields[0].startswith('#'):
                continue
            header[fields[0]] = fields[1:]
            if fields[0] == 'DATA':
                break
        data = stream.read()
    count = int(header['POINTS'][0])
    if count == 0 and header['DATA'] == ['ascii']:
        data = b''
    elif (header['FIELDS'] != ['x', 'y', 'z'] or header['SIZE'] != ['4'] * 3 or
          header['TYPE'] != ['F'] * 3 or header.get('COUNT', ['1'] * 3) != ['1'] * 3 or
          header['DATA'] != ['binary'] or len(data) != count * 12):
        raise ValueError('Expected packed binary float32 XYZ: ' + str(path))
    return {'points': count, 'xyz_sha256': hashlib.sha256(data).hexdigest()}


def execute(binary, config, input_path, output, log):
    rss = log.with_suffix('.rss')
    command = ['/usr/bin/time', '-f', '%M', '-o', str(rss), str(binary),
               '--config', str(config), '--input', str(input_path), '--output', str(output)]
    start = time.monotonic()
    result = subprocess.run(command, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    elapsed = time.monotonic() - start
    log.write_text(result.stdout)
    if result.returncode:
        raise RuntimeError('Cleaner failed; see ' + str(log))
    times, counts = {}, {}
    for line in result.stdout.splitlines():
        match = re.match(r'^  ([\w.]+): ([\d.eE+-]+) s$', line)
        if match:
            times[match[1]] = float(match[2])
        match = re.match(r'^  ([\w -]+): (\d+)$', line)
        if match:
            counts[match[1]] = int(match[2])
    return {'times': times, 'counts': counts, 'wall_seconds': elapsed,
            'peak_rss_kib': int(rss.read_text().strip()), **xyz_info(output)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--optimized', type=Path, required=True)
    parser.add_argument('--variant', action='append', default=[], metavar='NAME=BINARY')
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--threads', type=int, nargs='+', default=[1, 4, 8, 16])
    parser.add_argument('--repeats', type=int, default=5)
    parser.add_argument('--mode', choices=['timing', 'quality'], required=True)
    args = parser.parse_args()
    if args.repeats < 1 or any(t < 1 for t in args.threads):
        parser.error('repeats and threads must be positive')
    # Refuse reuse to prevent destroying earlier evidence or map files.
    args.output_dir.mkdir(parents=True, exist_ok=False)
    root = args.output_dir.resolve()
    variants = [('baseline', args.baseline.resolve(), 'legacy')]
    for spec in args.variant:
        name, binary = spec.split('=', 1)
        if not re.fullmatch(r'[A-Za-z0-9_-]+', name):
            parser.error('invalid variant name')
        variants.append((name, Path(binary).resolve(), 'legacy'))
    variants += [('legacy', args.optimized.resolve(), 'legacy'),
                 ('single_exact', args.optimized.resolve(), 'single_exact')]
    if len(set(v[0] for v in variants)) != len(variants):
        parser.error('variant names must be unique')
    original = args.config.read_text()
    text = set_yaml(original, 'debug', 'save_intermediate', 'false')
    manifest = {'input': str(args.input.resolve()),
                'input_sha256': hashlib.sha256(args.input.read_bytes()).hexdigest(),
                'config_sha256': hashlib.sha256(args.config.read_bytes()).hexdigest(),
                'binaries': {name: {'path': str(binary), 'sha256': hashlib.sha256(binary.read_bytes()).hexdigest()}
                             for name, binary, _ in variants},
                'omp_environment': {k: v for k, v in os.environ.items() if k.startswith('OMP_')},
                'threads': args.threads, 'repeats': args.repeats, 'mode': args.mode}
    (root / 'manifest.json').write_text(json.dumps(manifest, indent=2))
    records = []
    mismatches = []
    for threads in args.threads:
        if args.mode == 'timing':
            configs = {}
            for name, binary, backend in variants:
                config = root / f'{name}-t{threads}.yaml'
                config.write_text(set_yaml(set_yaml(text, 'performance', 'threads', threads),
                                           'sor', 'search_backend', backend))
                configs[name] = config
                output = root / f'{name}-t{threads}.pcd'
                execute(binary, config, args.input.resolve(), output, root / f'{name}-t{threads}-warmup.log')
            # Measure every version once per round. Reverse alternate rounds
            # to balance order/thermal drift; do not run redundant baselines
            # for each pair. All candidate comparisons use the same five runs.
            for repeat in range(args.repeats):
                order = variants if repeat % 2 == 0 else list(reversed(variants))
                for name, binary, backend in order:
                    label = f'{name}-t{threads}-r{repeat}'
                    rec = execute(binary, configs[name], args.input.resolve(),
                                  root / f'{name}-t{threads}.pcd', root / (label + '.log'))
                    rec.update(variant=name, threads=threads, repeat=repeat)
                    records.append(rec)
                    print(label, rec['times'].get('total'), flush=True)
                    (root / 'records.json').write_text(json.dumps(records, indent=2))
        else:
            for stage_index, stage in enumerate(STAGES):
                baseline_info = None
                for name, binary, backend in variants:
                    cfg_text = set_yaml(set_yaml(text, 'performance', 'threads', threads),
                                        'sor', 'search_backend', backend)
                    for disabled in STAGES[stage_index + 1:]:
                        cfg_text = set_yaml(cfg_text, disabled, 'enable', 'false')
                    label = f'{name}-t{threads}-{stage}'
                    config = root / (label + '.yaml')
                    config.write_text(cfg_text)
                    rec = execute(binary, config, args.input.resolve(), root / (label + '.pcd'),
                                  root / (label + '.log'))
                    rec.update(variant=name, threads=threads, stage=stage)
                    if name == 'baseline':
                        baseline_info = rec
                    rec['identical_to_baseline'] = rec['xyz_sha256'] == baseline_info['xyz_sha256']
                    if backend == 'legacy' and not rec['identical_to_baseline']:
                        mismatches.append(label)
                    records.append(rec)
                    print(label, rec['points'], rec['identical_to_baseline'], flush=True)
                    (root / 'records.json').write_text(json.dumps(records, indent=2))
            # Each backend should also be independent of thread count.
    if args.mode == 'quality':
        for name, _, _ in variants:
            for stage in STAGES:
                group = [r for r in records if r['variant'] == name and r['stage'] == stage]
                if len({r['xyz_sha256'] for r in group}) > 1:
                    mismatches.append(name + ':' + stage + ':thread-dependence')
        summary = {'mismatches': mismatches, 'passed': not mismatches}
    else:
        summary = []
        for threads in args.threads:
            for name, _, _ in variants:
                group = [r for r in records if r['threads'] == threads and r['variant'] == name]
                times = {key: {'median': statistics.median(r['times'][key] for r in group),
                               'min': min(r['times'][key] for r in group),
                               'max': max(r['times'][key] for r in group)}
                         for key in group[0]['times']}
                summary.append({'variant': name, 'threads': threads,
                                'times': times, 'peak_rss_kib': max(r['peak_rss_kib'] for r in group),
                                'counts': group[-1]['counts']})
    if args.mode == 'timing':
        baseline_hashes = {r['xyz_sha256'] for r in records if r['variant'] == 'baseline'}
        if len(baseline_hashes) != 1:
            mismatches.append('baseline output varies')
        for name, _, backend in variants:
            hashes = {r['xyz_sha256'] for r in records if r['variant'] == name}
            if len(hashes) != 1 or (backend == 'legacy' and hashes != baseline_hashes):
                mismatches.append(name + ':output differs')
        (root / 'equality.json').write_text(json.dumps({'passed': not mismatches,
                                                       'mismatches': mismatches}, indent=2))
    (root / 'summary.json').write_text(json.dumps(summary, indent=2))
    return 1 if mismatches else 0


if __name__ == '__main__':
    raise SystemExit(main())
