"""Run the same fixed-seed experiment in independent local batches."""
import concurrent.futures
import csv
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parent
BINARY = ROOT.parent / 'build' / 'benchmark'
CONDITIONS = ['direct_empty', 'sampled_empty', 'sampled_obstacles_1',
              'sampled_obstacles_2', 'sampled_obstacles_3']
TRIALS = 1000
BATCH = 100
WORKERS = 8


def read(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def write(path, rows):
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def main():
    for name in ['trials.csv', 'summary.csv', 'settings.csv']:
        if (ROOT / name).exists():
            raise RuntimeError(f'Results already exist: {ROOT / name}')
    with tempfile.TemporaryDirectory(prefix='planner-1000-batches-') as temporary:
        batches = Path(temporary)

        def run(task):
            condition, seed = task
            directory = batches / f'{condition}-{seed}'
            count = min(BATCH, TRIALS - seed + 1)
            subprocess.run([str(BINARY), '--condition', condition, '--seed', str(seed),
                            '--trials', str(count), '--output', str(directory)],
                           check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            print(f'Completed {condition}: seeds {seed}-{seed + count - 1}', flush=True)
            return directory

        tasks = [(condition, seed) for seed in range(1, TRIALS + 1, BATCH)
                 for condition in CONDITIONS]
        with concurrent.futures.ThreadPoolExecutor(max_workers=WORKERS) as pool:
            completed = list(pool.map(run, tasks))
        rows = [row for directory in completed for row in read(directory / 'trials.csv')]
        rows.sort(key=lambda row: (CONDITIONS.index(row['condition']), int(row['seed'])))
        for row in rows:
            row['trial'] = row['seed']
        summaries = []
        for condition in CONDITIONS:
            records = [row for row in rows if row['condition'] == condition]
            assert len(records) == TRIALS
            assert [int(row['seed']) for row in records] == list(range(1, TRIALS + 1))
            successes = sum(int(row['success']) for row in records)
            valid = sum(int(row['valid_path_success']) for row in records)
            summaries.append(dict(condition=condition, scene=records[0]['scene'], trials=TRIALS,
                                  successes=successes, success_rate_percent=100 * successes / TRIALS,
                                  valid_paths=valid, valid_path_rate_percent=100 * valid / TRIALS,
                                  mean_planning_seconds=sum(float(row['planning_seconds'])
                                                            for row in records) / TRIALS))
        metadata = read(completed[0] / 'settings.csv')
        for row in metadata:
            if row['setting'] == 'trials_per_condition':
                row['value'] = TRIALS
            if row['setting'] == 'condition':
                row['value'] = 'all'
        metadata.extend([dict(setting='execution_workers', value=WORKERS),
                         dict(setting='batch_size', value=BATCH)])
        write(ROOT / 'trials.csv', rows)
        write(ROOT / 'summary.csv', summaries)
        write(ROOT / 'settings.csv', metadata)
        for row in summaries:
            print(row, flush=True)


if __name__ == '__main__':
    main()
