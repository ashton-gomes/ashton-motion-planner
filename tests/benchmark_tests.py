"""Check paired tasks, retained failures, CSV accounting, and reproducibility."""
import csv
from pathlib import Path
import subprocess
import sys
import tempfile


def read(path):
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    assert all(None not in row for row in rows), 'CSV fields must match the header'
    return rows


def run(binary, directory, *extra):
    result = subprocess.run(
        [binary, '--trials', '1', '--max-expansions', '1', '--output', str(directory), *extra],
        capture_output=True, text=True, timeout=120,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    return read(directory / 'trials.csv'), read(directory / 'summary.csv')


def main():
    binary = str(Path(sys.argv[1]).resolve())
    with tempfile.TemporaryDirectory(prefix='planner-benchmark-test-') as temp:
        first_dir = Path(temp) / 'first'
        trials, summaries = run(binary, first_dir)
        repeated, _ = run(binary, Path(temp) / 'repeated')
        assert len(trials) == len(summaries) == 7
        assert all(row['obstacles'] == 'known' for row in trials + summaries), 'Known geometry is the default'
        assert any(row['success'] == '0' for row in trials), 'Failures must remain in the results'
        goal_columns = ['goal_x', 'goal_y', 'goal_z', 'goal_w', 'goal_qx', 'goal_qy', 'goal_qz']
        assert all(trials[0][key] == trials[1][key] for key in goal_columns), 'Empty tasks must be paired'
        for row, again, summary in zip(trials, repeated, summaries):
            assert {k: v for k, v in row.items() if k != 'planning_seconds'} == {
                k: v for k, v in again.items() if k != 'planning_seconds'
            }, 'Fixed seeds must reproduce every non-timing result'
            assert summary['trials'] == '1'
            assert summary['successes'] == row['success']
            assert float(summary['success_rate_percent']) == 100 * int(row['success'])
            assert summary['valid_paths'] == row['valid_path_success']
            if row['success'] == '0':
                assert row['failure_reason']
                assert row['joint_limits_ok'] == row['collision_free'] == ''
            if row['condition'] == 'direct_empty':
                assert int(row['nodes']) <= 2
        # Sensed obstacles: paired with known geometry (same goal and seed) and
        # validated against the true scene.
        both, both_summaries = run(binary, Path(temp) / 'both', '--condition', 'sampled_random_cylinders',
                                   '--obstacles', 'both')
        assert [row['obstacles'] for row in both] == ['known', 'sensed']
        assert all(both[0][key] == both[1][key] for key in goal_columns), 'Sensed tasks must be paired'
        known_alone = next(row for row in trials if row['condition'] == 'sampled_random_cylinders')
        assert {k: v for k, v in both[0].items() if k != 'planning_seconds'} == {
            k: v for k, v in known_alone.items() if k != 'planning_seconds'
        }, 'Adding the sensed run must not change the known-geometry result'
        known_summary, sensed_summary = both_summaries
        assert known_summary['cloud_points'] == '' and int(sensed_summary['cloud_points']) > 0
        assert int(sensed_summary['obstacle_boxes']) > 0
        before = (first_dir / 'trials.csv').read_bytes()
        rejected = subprocess.run([binary, '--output', str(first_dir)], capture_output=True)
        assert rejected.returncode != 0
        assert before == (first_dir / 'trials.csv').read_bytes(), 'Existing results must be preserved'
        for args in [('--trials', '0'), ('--seed', '-1'), ('--condition', 'unknown'), ('--trials',),
                     ('--obstacles', 'lidar')]:
            rejected = subprocess.run([binary, *args], capture_output=True)
            assert rejected.returncode != 0, args
    print('Benchmark integration checks passed.')


if __name__ == '__main__':
    main()
