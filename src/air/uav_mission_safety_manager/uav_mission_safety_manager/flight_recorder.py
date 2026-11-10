"""Flight event recorder (black box); independent of ROS and AirSim.

Records state-machine transitions and mission milestones into a bounded ring
buffer, then exports a CSV timeline and a JSON summary for post-flight review.
The module only needs an injected clock, so it is unit-testable offline.
"""
import csv
import json
import time
from collections import deque


class FlightRecorder:
    """Bounded event log with per-state duration statistics."""

    CSV_HEADER = ['seq', 'time_s', 'kind', 'state', 'detail']

    def __init__(self, max_events=500, clock=time.monotonic):
        max_events = int(max_events)
        if max_events <= 0:
            raise ValueError('max_events must be a positive integer')
        self.max_events = max_events
        self.clock = clock
        self.started_at = clock()
        self.events = deque(maxlen=max_events)
        self.dropped = 0
        self.sequence = 0

    def seconds(self, timestamp=None):
        """Elapsed seconds since the recorder was created."""
        now = self.clock() if timestamp is None else timestamp
        return round(now - self.started_at, 3)

    def record(self, kind, detail='', state=None, timestamp=None):
        """Append one event; the oldest event is dropped when full."""
        event = {
            'seq': self.sequence,
            'time_s': self.seconds(timestamp),
            'kind': kind,
            'state': state,
            'detail': detail,
        }
        if len(self.events) == self.max_events:
            self.dropped += 1
        self.events.append(event)
        self.sequence += 1
        return event

    def state_changed(self, previous, current, event='', reason='', timestamp=None):
        """Record a state transition produced by SafetyMachine."""
        previous_name = getattr(previous, 'value', previous)
        current_name = getattr(current, 'value', current)
        detail = 'event={}'.format(event or '-')
        if reason:
            detail += '; reason={}'.format(reason)
        recorded = self.record('state', detail, state=current_name,
                               timestamp=timestamp)
        if event == 'failsafe':
            self.record('failsafe', reason or 'safety condition',
                        state=current_name, timestamp=timestamp)
        return recorded

    def snapshot(self):
        """Copy of the buffered events (oldest first)."""
        return [dict(event) for event in self.events]

    def summary(self, timestamp=None):
        """Aggregate counts, state durations and failsafe reasons."""
        kinds = {}
        failsafe = []
        durations = {}
        state = None
        entered_at = 0.0
        for event in self.events:
            kinds[event['kind']] = kinds.get(event['kind'], 0) + 1
            if event['kind'] == 'failsafe':
                failsafe.append(event['detail'])
            if event['kind'] != 'state':
                continue
            if state is not None:
                durations[state] = round(durations.get(state, 0.0) +
                                        (event['time_s'] - entered_at), 3)
            state = event['state']
            entered_at = event['time_s']
        if state is not None:
            durations[state] = round(durations.get(state, 0.0) +
                                    (self.seconds(timestamp) - entered_at), 3)
        return {
            'events': len(self.events),
            'dropped': self.dropped,
            'kinds': kinds,
            'state_durations_s': durations,
            'final_state': state,
            'failsafe_reasons': failsafe,
        }

    def write_csv(self, path):
        """Write the timeline; returns the path."""
        with open(path, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.writer(handle)
            writer.writerow(self.CSV_HEADER)
            for event in self.events:
                writer.writerow([event['seq'], '%.3f' % event['time_s'],
                                 event['kind'], event['state'] or '',
                                 event['detail']])
        return path

    def write_json(self, path, timestamp=None):
        """Write the summary plus the buffered events; returns the path."""
        payload = {'summary': self.summary(timestamp), 'events': self.snapshot()}
        with open(path, 'w', encoding='utf-8') as handle:
            json.dump(payload, handle, ensure_ascii=False, indent=2)
        return path
