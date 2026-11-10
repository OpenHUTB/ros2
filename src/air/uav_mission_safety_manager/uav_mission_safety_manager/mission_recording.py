"""Append-only status evidence and offline queue metrics; no ROS dependency."""
import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
import uuid


ACTIVE = ('RUNNING', 'PAUSED')
TERMINAL = ('COMPLETED', 'CANCELED', 'ABORTED')


class StatusLog:
    """One exclusively created file per recorder run; flush every sample."""

    def __init__(self, directory, topic, clock=time.monotonic):
        directory = Path(directory).expanduser()
        directory.mkdir(parents=True, exist_ok=True)
        self.path = directory / ('mission_' + uuid.uuid4().hex + '.jsonl')
        self.clock, self.started, self.sequence = clock, clock(), 0
        self.stream = self.path.open('x', encoding='utf-8')
        self._write({'kind': 'header', 'schema_version': 1, 'topic': topic,
                     'started_utc': datetime.now(timezone.utc).isoformat()})

    def _write(self, record):
        # Serialize before writing, so malformed/non-finite data cannot leave a partial line.
        line = json.dumps(record, ensure_ascii=False, allow_nan=False)
        self.stream.write(line + '\n')
        self.stream.flush()

    def append(self, status):
        validate_status(status)
        self._write({'kind': 'status', 'sequence': self.sequence,
                     'elapsed_s': self.clock() - self.started, 'status': status})
        self.sequence += 1

    def close(self):
        if not self.stream.closed:
            try:
                self._write({'kind': 'footer', 'samples': self.sequence})
            finally:
                self.stream.close()


def validate_status(status):
    if not isinstance(status, dict):
        raise ValueError('Expected a status object')
    queue = status.get('mission_queue')
    if not isinstance(queue, dict) or queue.get('state') not in ('IDLE',) + ACTIVE + TERMINAL:
        raise ValueError('Expected a controller mission_queue status')
    for key in ('total', 'completed', 'remaining'):
        value = queue.get(key)
        if type(value) is not int or value < 0:
            raise ValueError('Invalid queue counter: ' + key)
    if not queue['completed'] <= queue['total'] <= 100:
        raise ValueError('Invalid queue progress')
    position = status.get('position_ned')
    if position is not None and (
            not isinstance(position, (list, tuple)) or len(position) != 3 or
            any(type(v) not in (int, float) or not math.isfinite(v) for v in position)):
        raise ValueError('Invalid NED position')


def read_log(path):
    """Strict framing: truncated/corrupt logs produce no misleading report."""
    with Path(path).open(encoding='utf-8') as stream:
        previous_time, count, ended = 0.0, 0, False
        header = stream.readline()
        try:
            meta = json.loads(header)
            if meta.get('kind') != 'header' or meta.get('schema_version') != 1:
                raise ValueError('Unsupported log header')
            for line in stream:
                record = json.loads(line)
                if ended:
                    raise ValueError('Data after footer')
                if record.get('kind') == 'footer':
                    if record.get('samples') != count:
                        raise ValueError('Footer sample count mismatch')
                    ended = True
                    continue
                elapsed = record.get('elapsed_s')
                if (record.get('kind') != 'status' or record.get('sequence') != count or
                        type(elapsed) not in (int, float) or not math.isfinite(elapsed) or
                        elapsed < previous_time):
                    raise ValueError('Invalid sample sequence or monotonic time')
                validate_status(record.get('status'))
                # Reject NaN/Infinity even in optional fields preserved in the snapshot.
                json.dumps(record, allow_nan=False)
                yield record
                previous_time, count = elapsed, count + 1
        except (ValueError, TypeError, AttributeError) as exc:
            raise ValueError('Invalid mission log: ' + str(exc)) from exc
        if not ended:
            raise ValueError('Missing footer: recorder interrupted or log truncated')


def summarize(records):
    """Infer observed queue episodes, never infer an unobserved successful mission."""
    missions, current, previous, last_position = [], None, None, None
    modes, samples = set(), 0
    for record in records:
        samples += 1
        status, elapsed = record['status'], record['elapsed_s']
        queue = status['mission_queue']
        modes.add(status.get('adapter_mode', 'unknown'))
        state = queue['state']
        replaced = current is not None and (
            state == 'IDLE' or queue['total'] != current['total'] or
            queue['completed'] < current['completed'])
        if replaced:
            missions.append(current)
            current, last_position = None, None
        # A terminal state retained on later ticks must not become another mission.
        if current is None and (state in ACTIVE or
                                state in TERMINAL and (previous is None or previous['state'] == 'IDLE')):
            current = dict(mission=len(missions) + 1, outcome='INCOMPLETE',
                           start_observed_s=elapsed, end_observed_s=elapsed,
                           observed_duration_s=0.0, observed_pause_s=0.0,
                           sampled_distance_m=0.0, total=queue['total'],
                           completed=queue['completed'], samples=0,
                           start_partial=(replaced or previous is None or previous['state'] in ACTIVE or
                                          state not in ACTIVE or queue['completed'] > 0),
                           adapter_mode=status.get('adapter_mode', 'unknown'))
            last_position = None
        if current is not None:
            delta = elapsed - current['end_observed_s']
            if previous is not None and previous['state'] == 'PAUSED':
                current['observed_pause_s'] += delta
            position = status.get('position_ned')
            if position is not None and last_position is not None:
                current['sampled_distance_m'] += math.dist(last_position, position)
            # Missing positions break the path, rather than bridging an unknown gap.
            last_position = position
            current.update(end_observed_s=elapsed,
                           observed_duration_s=elapsed - current['start_observed_s'],
                           completed=queue['completed'], samples=current['samples'] + 1)
            if state in TERMINAL:
                current['outcome'] = state
                missions.append(current)
                current, last_position = None, None
        previous = queue
    if current is not None:
        missions.append(current)
    return dict(schema_version=1, samples=samples, adapter_modes=sorted(modes),
                mission_count=len(missions), missions=missions)


def main(args=None):
    parser = argparse.ArgumentParser(description='Offline mission log statistics; sends no ROS commands')
    parser.add_argument('log', type=Path)
    parser.add_argument('--output-dir', type=Path, help='New directory for summary.json and missions.csv')
    options = parser.parse_args(args)
    try:
        report = summarize(read_log(options.log))
        if options.output_dir:
            options.output_dir.mkdir(parents=True, exist_ok=False)
            (options.output_dir / 'summary.json').write_text(
                json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
            fields = ('mission', 'outcome', 'adapter_mode', 'total', 'completed',
                      'start_partial', 'start_observed_s', 'end_observed_s',
                      'observed_duration_s', 'observed_pause_s', 'sampled_distance_m', 'samples')
            with (options.output_dir / 'missions.csv').open('x', newline='', encoding='utf-8') as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                writer.writerows(report['missions'])
        print(json.dumps(report, ensure_ascii=False, indent=2))
    except (OSError, ValueError) as exc:
        parser.exit(2, 'mission_report: ' + str(exc) + '\n')


if __name__ == '__main__':
    main()
