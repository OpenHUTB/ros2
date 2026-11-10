"""Versioned, self-contained visual observations on std_msgs/String.

Keep the existing vision_msgs output unchanged for other consumers. Dimensions,
capture stamp and boxes here always originate from the same inference input.
"""
import json
import math

from .visual_centering import Box


def encode_observation(detections, width, height):
    boxes = []
    for detection in detections.detections:
        center = getattr(detection.bbox.center, 'position', detection.bbox.center)
        for result in detection.results:
            h = result.hypothesis
            boxes.append([center.x, center.y, detection.bbox.size_x,
                          detection.bbox.size_y, h.score, h.class_id])
    stamp = detections.header.stamp
    return json.dumps(dict(version=1, stamp_ns=stamp.sec*1_000_000_000+stamp.nanosec,
                           frame_id=detections.header.frame_id,
                           width=width, height=height, boxes=boxes), allow_nan=False)


def decode_observation(payload):
    value = json.loads(payload)
    if not isinstance(value, dict) or value.get('version') != 1:
        raise ValueError('unsupported visual observation version')
    stamp = value['stamp_ns']
    width, height = value['width'], value['height']
    if (type(stamp) is not int or stamp <= 0 or
            any(type(v) is not int or not 0 < v <= 16384 for v in (width, height)) or
            not isinstance(value['frame_id'], str) or not value['frame_id']):
        raise ValueError('invalid observation metadata')
    raw_boxes = value['boxes']
    if not isinstance(raw_boxes, list) or len(raw_boxes) > 1000:
        raise ValueError('invalid observation boxes')
    boxes = []
    for raw in raw_boxes:
        if (not isinstance(raw, list) or len(raw) != 6 or
                not isinstance(raw[5], str) or
                any(type(v) not in (int, float) or not math.isfinite(v) for v in raw[:5])):
            raise ValueError('invalid observation box')
        boxes.append(Box(*raw))
    return stamp, value['frame_id'], width, height, boxes
