import json
import unittest

from openhutb_yolo_mlp_control.visual_observation import decode_observation


class ObservationTests(unittest.TestCase):
    def payload(self, **changes):
        value = dict(version=1, stamp_ns=123, frame_id='camera', width=640,
                     height=480, boxes=[[480, 240, 80, 100, .9, 'car']])
        value.update(changes)
        return json.dumps(value)

    def test_complete_observation_and_empty_frame(self):
        stamp, frame, width, height, boxes = decode_observation(self.payload())
        self.assertEqual((stamp, frame, width, height), (123, 'camera', 640, 480))
        self.assertEqual(boxes[0].label, 'car')
        self.assertEqual(decode_observation(self.payload(boxes=[]))[-1], [])

    def test_malformed_metadata_and_nonfinite_box_rejected(self):
        for changes in (dict(version=2), dict(stamp_ns=True), dict(width=0),
                        dict(frame_id=''), dict(boxes=[[float('nan'), 0, 1, 1, .9, 'car']]),
                        dict(boxes=[[1, 2]])):
            with self.assertRaises(ValueError):
                decode_observation(self.payload(**changes))
