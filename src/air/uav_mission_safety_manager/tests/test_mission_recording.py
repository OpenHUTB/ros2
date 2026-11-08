"""Recording integrity and metric boundaries, without ROS or AirSim."""
from contextlib import redirect_stdout, redirect_stderr
import io
import json
from pathlib import Path
import tempfile
import unittest

from uav_mission_safety_manager.mission_recording import StatusLog, read_log, summarize, main


def sample(t, state, completed=0, position=(0., 0., -2.), total=3):
    return dict(kind='status', elapsed_s=t, status=dict(
        adapter_mode='demo', position_ned=position,
        mission_queue=dict(state=state, total=total, completed=completed,
                           remaining=total - completed if state in ('RUNNING', 'PAUSED') else 0)))


class RecordingTests(unittest.TestCase):
    def test_pause_completion_and_distance_exclude_idle_flight(self):
        records = [sample(0, 'IDLE', position=(99, 0, 0)), sample(1, 'RUNNING'),
                   sample(3, 'PAUSED', position=(3, 4, -2)),
                   sample(7, 'RUNNING', position=(3, 4, -2)),
                   sample(9, 'COMPLETED', 3, position=(6, 8, -2)),
                   sample(10, 'COMPLETED', 3, position=(99, 0, 0))]
        report = summarize(records)
        self.assertEqual(report['mission_count'], 1)
        mission = report['missions'][0]
        self.assertEqual(mission['outcome'], 'COMPLETED')
        self.assertEqual(mission['observed_duration_s'], 8)
        self.assertEqual(mission['observed_pause_s'], 4)
        self.assertEqual(mission['sampled_distance_m'], 10)
        self.assertFalse(mission['start_partial'])

    def test_consecutive_missions_and_retained_terminal(self):
        records = [sample(0, 'IDLE'), sample(1, 'RUNNING'), sample(2, 'CANCELED'),
                   sample(3, 'CANCELED'), sample(4, 'RUNNING'), sample(5, 'ABORTED')]
        self.assertEqual([m['outcome'] for m in summarize(records)['missions']],
                         ['CANCELED', 'ABORTED'])

    def test_late_join_and_unfinished_are_not_success(self):
        mission = summarize([sample(3, 'PAUSED', 1), sample(4, 'RUNNING', 1)])['missions'][0]
        self.assertTrue(mission['start_partial'])
        self.assertEqual(mission['outcome'], 'INCOMPLETE')
        terminal = summarize([sample(3, 'COMPLETED', 3), sample(4, 'COMPLETED', 3)])
        self.assertEqual(terminal['mission_count'], 1)
        self.assertTrue(terminal['missions'][0]['start_partial'])
        self.assertEqual(terminal['missions'][0]['observed_duration_s'], 0)

    def test_replacement_without_terminal_keeps_incomplete_episode(self):
        result = summarize([sample(0, 'RUNNING', 2), sample(1, 'RUNNING'),
                            sample(2, 'COMPLETED', 3)])
        self.assertEqual([m['outcome'] for m in result['missions']], ['INCOMPLETE', 'COMPLETED'])
        self.assertTrue(result['missions'][1]['start_partial'])

    def test_missing_position_breaks_distance(self):
        result = summarize([sample(0, 'RUNNING'), sample(1, 'RUNNING', position=None),
                            sample(2, 'CANCELED', position=(10, 0, -2))])
        self.assertEqual(result['missions'][0]['sampled_distance_m'], 0)

    def test_exclusive_logs_round_trip_and_cli_exports(self):
        with tempfile.TemporaryDirectory() as directory:
            clock = iter([0., 1., 2., 3.])
            log = StatusLog(directory, '/status', clock=lambda: next(clock))
            other = StatusLog(directory, '/status')
            self.assertNotEqual(log.path, other.path)
            other.close()
            for record in [sample(0, 'IDLE'), sample(1, 'RUNNING'), sample(2, 'COMPLETED', 3)]:
                log.append(record['status'])
            log.close()
            log.close()
            records = list(read_log(log.path))
            self.assertEqual(len(records), 3)
            self.assertEqual(records[-1]['sequence'], 2)
            export = Path(directory) / 'export'
            with redirect_stdout(io.StringIO()):
                main([str(log.path), '--output-dir', str(export)])
            self.assertEqual(json.loads((export / 'summary.json').read_text())['mission_count'], 1)
            self.assertIn('COMPLETED', (export / 'missions.csv').read_text())
            with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                main([str(log.path), '--output-dir', str(export)])
            self.assertEqual(error.exception.code, 2)

    def test_invalid_input_does_not_write_partial_sample(self):
        with tempfile.TemporaryDirectory() as directory:
            log = StatusLog(directory, '/status')
            for invalid in [[], sample(0, 'RUNNING', position=(float('nan'), 0, 0))['status'],
                            sample(0, 'RUNNING', completed=4)['status']]:
                with self.assertRaises(ValueError):
                    log.append(invalid)
            valid = sample(0, 'IDLE')['status']
            valid['reason'] = float('nan')
            with self.assertRaises(ValueError):
                log.append(valid)
            log.append(sample(0, 'IDLE')['status'])
            log.close()
            self.assertEqual(len(list(read_log(log.path))), 1)

    def test_corrupt_or_unfinished_log_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            log = StatusLog(directory, '/status')
            log.append(sample(0, 'IDLE')['status'])
            log.close()
            lines = log.path.read_text().splitlines()
            bad = Path(directory) / 'bad.jsonl'
            variants = [lines[:-1], lines + [lines[1]], lines[:1] + ['{broken'],
                        [lines[0].replace('"schema_version": 1', '"schema_version": 2')] + lines[1:]]
            for field, value in [('sequence', 7), ('elapsed_s', -1), ('elapsed_s', float('nan'))]:
                record = json.loads(lines[1])
                record[field] = value
                variants.append([lines[0], json.dumps(record), lines[2]])
            for contents in variants:
                bad.write_text('\n'.join(contents) + '\n')
                with self.assertRaises(ValueError):
                    summarize(read_log(bad))

    def test_empty_recording_is_not_a_mission(self):
        with tempfile.TemporaryDirectory() as directory:
            log = StatusLog(directory, '/status')
            log.close()
            self.assertEqual(summarize(read_log(log.path))['mission_count'], 0)


if __name__ == '__main__':
    unittest.main()
