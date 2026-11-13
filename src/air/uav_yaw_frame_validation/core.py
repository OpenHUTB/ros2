import math

def transform(point, translation, yaw):
    if len(point) != 2 or len(translation) != 2:
        raise ValueError('two dimensions required')
    if not all(math.isfinite(v) for v in (*point, *translation, yaw)):
        raise ValueError('finite inputs required')
    c, s = math.cos(yaw), math.sin(yaw)
    x, y = point
    return c*x-s*y+translation[0], s*x+c*y+translation[1]

def inverse(point, translation, yaw):
    return transform((point[0]-translation[0],point[1]-translation[1]),(0.,0.),-yaw)
