#!/usr/bin/env python3

"""Split legacy node-control CSV logs into runs and plot their trajectories."""

import argparse
import csv
from datetime import datetime
import hashlib
from pathlib import Path
import re

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt


REQUIRED_COLUMNS = {'timestamp', 'x', 'y', 'z'}


def parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--input',
        type=Path,
        default=workspace_root() / 'odom_listener_command.csv',
        help='Legacy CSV file to split (default: node_control/odom_listener_command.csv).',
    )
    parser.add_argument(
        '--output-root',
        type=Path,
        default=workspace_root() / 'output',
        help='Directory that receives recovered runs and generated plots.',
    )
    parser.add_argument(
        '--gap-seconds',
        type=float,
        default=5.0,
        help='Timestamp gap that starts a new recovered run (default: 5 seconds).',
    )
    return parser.parse_args()


def workspace_root():
    script_path = Path(__file__).resolve()
    for parent in script_path.parents:
        if (parent / 'src' / 'node_control' / 'package.xml').is_file():
            return parent
    return Path.cwd()


def read_log(path):
    with path.open(newline='', encoding='utf-8') as handle:
        reader = csv.DictReader(handle)
        fieldnames = reader.fieldnames
        if not fieldnames or not REQUIRED_COLUMNS.issubset(fieldnames):
            return None, []

        rows = []
        for row in reader:
            try:
                row['_timestamp'] = float(row['timestamp'])
                row['_x'] = float(row['x'])
                row['_y'] = float(row['y'])
                row['_z'] = float(row['z'])
            except (KeyError, TypeError, ValueError):
                continue
            rows.append(row)
    return fieldnames, rows


def split_runs(rows, gap_seconds):
    if not rows:
        return []

    runs = [[rows[0]]]
    previous_timestamp = rows[0]['_timestamp']
    for row in rows[1:]:
        timestamp = row['_timestamp']
        if timestamp <= previous_timestamp or timestamp - previous_timestamp > gap_seconds:
            runs.append([])
        runs[-1].append(row)
        previous_timestamp = timestamp
    return runs


def unique_run_dir(output_root, name):
    candidate = output_root / name
    suffix = 1
    while candidate.exists():
        candidate = output_root / f'{name}_{suffix:02d}'
        suffix += 1
    candidate.mkdir(parents=True)
    return candidate


def run_name(source, index, start_timestamp):
    source_name = re.sub(r'[^A-Za-z0-9_-]+', '_', source.stem)
    try:
        timestamp = datetime.fromtimestamp(start_timestamp).strftime('%Y%m%d_%H%M%S')
    except (OverflowError, OSError, ValueError):
        timestamp = f'{start_timestamp:.0f}'
    return f'recovered_{source_name}_{timestamp}_{index:02d}'


def write_recovered_run(run_dir, fieldnames, rows, source, recovery_key):
    csv_path = run_dir / 'odometry.csv'
    with csv_path.open('w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({name: row.get(name, '') for name in fieldnames})
    (run_dir / 'source.txt').write_text(f'{source}\n', encoding='utf-8')
    (run_dir / '.recovery_key').write_text(recovery_key, encoding='utf-8')
    return csv_path


def plot_rows(rows, output_path, title):
    if not rows:
        return

    start_timestamp = rows[0]['_timestamp']
    elapsed = [row['_timestamp'] - start_timestamp for row in rows]
    x_values = [row['_x'] for row in rows]
    y_values = [row['_y'] for row in rows]
    z_values = [row['_z'] for row in rows]

    figure, axes = plt.subplots(2, 1, figsize=(9, 8), constrained_layout=True)
    axes[0].plot(elapsed, z_values, color='tab:blue')
    axes[0].set_title(title)
    axes[0].set_xlabel('Time from run start (s)')
    axes[0].set_ylabel('Z (m)')
    axes[0].grid(True)

    axes[1].plot(x_values, y_values, color='tab:orange')
    axes[1].scatter(x_values[0], y_values[0], color='green', label='start')
    axes[1].scatter(x_values[-1], y_values[-1], color='red', label='end')
    axes[1].set_xlabel('X (m)')
    axes[1].set_ylabel('Y (m)')
    axes[1].set_aspect('equal', adjustable='box')
    axes[1].grid(True)
    axes[1].legend()

    figure.savefig(output_path, dpi=160)
    plt.close(figure)


def plot_existing_runs(output_root):
    count = 0
    if not output_root.exists():
        return count
    for csv_path in sorted(output_root.rglob('odometry.csv')):
        _, rows = read_log(csv_path)
        if not rows:
            continue
        plot_rows(rows, csv_path.with_name('trajectory.png'), csv_path.parent.name)
        count += 1
    return count


def existing_recovery_keys(output_root):
    keys = {
        key_file.read_text(encoding='utf-8')
        for key_file in output_root.rglob('.recovery_key')
    }
    for csv_path in output_root.rglob('odometry.csv'):
        fieldnames, rows = read_log(csv_path)
        if rows:
            keys.add(segment_key(fieldnames, rows))
    return keys


def segment_key(fieldnames, rows):
    digest = hashlib.sha256()
    for row in rows:
        digest.update(','.join(row.get(name, '') for name in fieldnames).encode())
        digest.update(b'\n')
    return digest.hexdigest()


def main():
    args = parse_arguments()
    source = args.input.resolve()
    if not source.is_file():
        raise SystemExit(f'log file not found: {source}')

    output_root = args.output_root.resolve()
    output_root.mkdir(parents=True, exist_ok=True)

    plotted = plot_existing_runs(output_root)
    recovered = 0
    recovery_keys = existing_recovery_keys(output_root)

    fieldnames, rows = read_log(source)
    if not rows:
        raise SystemExit(f'no valid odometry rows in: {source}')
    for index, run_rows in enumerate(split_runs(rows, args.gap_seconds), start=1):
        recovery_key = segment_key(fieldnames, run_rows)
        if recovery_key in recovery_keys:
            continue
        directory = unique_run_dir(
            output_root,
            run_name(source, index, run_rows[0]['_timestamp']),
        )
        csv_path = write_recovered_run(
            directory,
            fieldnames,
            run_rows,
            source,
            recovery_key,
        )
        plot_rows(run_rows, directory / 'trajectory.png', directory.name)
        print(f'recovered {csv_path}')
        recovery_keys.add(recovery_key)
        recovered += 1

    print(f'generated {plotted} existing plots and recovered {recovered} runs')


if __name__ == '__main__':
    main()
