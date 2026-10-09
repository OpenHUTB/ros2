"""Pure, testable adapters for the legacy Auto_drone policy. Coordinates: NED."""
import numpy as np

ACTIONS = np.array([(1, 0, 0), (0, 1, 0), (0, -1, 0),
                    (0, 0, -1), (0, 0, 1)], dtype=np.float32)


def observation(names, depth, clearance=3.0):
    depth = np.asarray(depth)
    if depth.size == 0 or not np.all(np.isfinite(depth)) or np.any(depth <= 0):
        raise ValueError('Depth must contain finite positive distances')
    obs = np.zeros(10, dtype=np.float32)
    for i, name in enumerate(('building', 'tree', 'road')):
        obs[i] = float(name in names)
    obs[3:6] = (0, 1, 0) if float(depth.min()) < clearance else (1, 0, 0)
    return obs


def velocity(action, speed):
    value = np.asarray(action)
    if value.size != 1 or not np.isfinite(value.item()) or value.item() != int(value.item()):
        raise ValueError('Expected one discrete integer action')
    index = int(value.item())
    if index not in range(len(ACTIONS)):
        raise ValueError('Unsupported policy action')
    return ACTIONS[index] * speed
