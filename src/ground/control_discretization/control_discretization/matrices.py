"""最小 2x2 矩阵运算与矩阵指数（只用 Python 标准库）。

只服务本模块的二阶系统，不追求通用性；``expm`` 用「缩放 + 泰勒级数」实现，
对 ``A*Ts`` 范数较大的情况先按 2 的幂缩放再平方回来。
"""


def eye(n=2):
    return [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]


def zeros(n=2, m=2):
    return [[0.0] * m for _ in range(n)]


def add(a, b):
    return [[a[i][j] + b[i][j] for j in range(len(a[0]))] for i in range(len(a))]


def sub(a, b):
    return [[a[i][j] - b[i][j] for j in range(len(a[0]))] for i in range(len(a))]


def scale(a, k):
    return [[a[i][j] * k for j in range(len(a[0]))] for i in range(len(a))]


def mul(a, b):
    rows, inner, cols = len(a), len(b), len(b[0])
    out = [[0.0] * cols for _ in range(rows)]
    for i in range(rows):
        for k in range(inner):
            if a[i][k] == 0.0:
                continue
            for j in range(cols):
                out[i][j] += a[i][k] * b[k][j]
    return out


def mul_vector(a, v):
    return [sum(a[i][j] * v[j] for j in range(len(v))) for i in range(len(a))]


def inverse2(a):
    det = a[0][0] * a[1][1] - a[0][1] * a[1][0]
    if abs(det) < 1e-15:
        raise ValueError('矩阵奇异，无法求逆')
    return [[a[1][1] / det, -a[0][1] / det],
            [-a[1][0] / det, a[0][0] / det]]


def norm_inf(a):
    return max(sum(abs(value) for value in row) for row in a)


def expm(a, terms=24):
    """矩阵指数：先缩放使范数 < 0.5，再泰勒展开，最后平方回原尺度。"""
    n = len(a)
    scaling = 0
    working = [row[:] for row in a]
    while norm_inf(working) > 0.5 and scaling < 60:
        working = scale(working, 0.5)
        scaling += 1

    result = eye(n)
    term = eye(n)
    for index in range(1, terms + 1):
        term = scale(mul(term, working), 1.0 / index)
        result = add(result, term)

    for _ in range(scaling):
        result = mul(result, result)
    return result


def eigenvalues2(a):
    """2x2 矩阵的特征值（返回复数列表）。"""
    trace = a[0][0] + a[1][1]
    det = a[0][0] * a[1][1] - a[0][1] * a[1][0]
    discriminant = trace * trace - 4.0 * det
    if discriminant >= 0.0:
        root = discriminant ** 0.5
        return [complex((trace + root) / 2.0), complex((trace - root) / 2.0)]
    root = (-discriminant) ** 0.5
    return [complex(trace / 2.0, root / 2.0), complex(trace / 2.0, -root / 2.0)]
