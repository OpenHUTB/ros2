"""极简矩阵运算库（仅标准库，用于纯 Python LQR 求解）。

只实现本模块用得到的小规模稠密矩阵运算：乘法、转置、加法、数乘、
求逆（高斯-约当）、单位阵、可控性秩、离散代数 Riccati 迭代。
矩阵用 list[list[float]] 表示（行优先）。
"""


def zeros(r, c):
    return [[0.0 for _ in range(c)] for _ in range(r)]


def identity(n):
    I = zeros(n, n)
    for i in range(n):
        I[i][i] = 1.0
    return I


def matmul(A, B):
    r = len(A)
    k = len(B)
    c = len(B[0])
    out = zeros(r, c)
    for i in range(r):
        Ai = A[i]
        for j in range(c):
            s = 0.0
            for t in range(k):
                s += Ai[t] * B[t][j]
            out[i][j] = s
    return out


def transpose(A):
    return [[A[i][j] for i in range(len(A))] for j in range(len(A[0]))]


def matadd(A, B):
    return [[A[i][j] + B[i][j] for j in range(len(A[0]))] for i in range(len(A))]


def matsub(A, B):
    return [[A[i][j] - B[i][j] for j in range(len(A[0]))] for i in range(len(A))]


def matscale(A, s):
    return [[A[i][j] * s for j in range(len(A[0]))] for i in range(len(A))]


def inverse(A):
    """高斯-约当求逆（A 必须方阵且可逆）。"""
    n = len(A)
    M = [row[:] + [1.0 if i == j else 0.0 for j in range(n)] for i, row in enumerate(A)]
    for col in range(n):
        # 选主元
        piv = max(range(col, n), key=lambda r: abs(M[r][col]))
        if abs(M[piv][col]) < 1e-12:
            raise ValueError("矩阵奇异，无法求逆")
        if piv != col:
            M[col], M[piv] = M[piv], M[col]
        pv = M[col][col]
        for j in range(2 * n):
            M[col][j] /= pv
        for r in range(n):
            if r == col:
                continue
            factor = M[r][col]
            if factor != 0.0:
                for j in range(2 * n):
                    M[r][j] -= factor * M[col][j]
    return [row[n:] for row in M]


def rank(A, tol=1e-9):
    """矩阵秩（高斯消元）。"""
    A = [row[:] for row in A]
    rows = len(A)
    cols = len(A[0])
    r = 0
    for c in range(cols):
        piv = None
        for i in range(r, rows):
            if abs(A[i][c]) > tol:
                piv = i
                break
        if piv is None:
            continue
        A[r], A[piv] = A[piv], A[r]
        pv = A[r][c]
        for j in range(c, cols):
            A[r][j] /= pv
        for i in range(rows):
            if i != r and abs(A[i][c]) > tol:
                f = A[i][c]
                for j in range(c, cols):
                    A[i][j] -= f * A[r][j]
        r += 1
        if r == rows:
            break
    return r


def controllability_matrix(A, B):
    """可控性矩阵 [B, AB, A^2 B, ...]（B 为列向量 n×1）。"""
    n = len(A)
    cols = [B[i][0] for i in range(n)]
    blocks = [[[c] for c in cols]]
    cur = B
    for _ in range(n - 1):
        cur = matmul(A, cur)
        blocks.append(cur)
    # 拼成 n × (n) 矩阵，每列一个块
    C = zeros(n, n)
    for bi, blk in enumerate(blocks):
        for i in range(n):
            C[i][bi] = blk[i][0]
    return C


def solve_dare(A, B, Q, R, iters=5000, tol=1e-10):
    """迭代求解离散代数 Riccati 方程（DARE），返回 P。

    P = Q + A^T P A - A^T P B (R + B^T P B)^{-1} B^T P A
    A: n×n, B: n×m, Q: n×n, R: m×m
    """
    n = len(Q)
    P = [row[:] for row in Q]
    At = transpose(A)
    Bt = transpose(B)
    for _ in range(iters):
        AtP = matmul(At, P)          # n×n
        BtP = matmul(Bt, P)          # m×n
        BtPA = matmul(BtP, A)        # m×n
        BtPB = matmul(BtP, B)        # m×m
        S = matadd(R, BtPB)          # m×m
        Sinv = inverse(S)
        AtPB = matmul(AtP, B)        # n×m
        term = matmul(matmulsafe(AtPB, Sinv), BtPA)  # n×n
        Pn = matsub(matadd(Q, matmul(AtP, A)), term)  # n×n
        diff = max(abs(Pn[i][j] - P[i][j]) for i in range(n) for j in range(n))
        P = Pn
        if diff < tol:
            break
    return P


def matmulsafe(A, B):
    return matmul(A, B)


def lqr_gain(A, B, Q, R):
    """由 DARE 解 P 计算离散 LQR 增益 K = (R + B^T P B)^{-1} B^T P A。"""
    P = solve_dare(A, B, Q, R)
    Bt = transpose(B)
    BtP = matmul(Bt, P)
    BtPB = matmul(BtP, B)
    S = matadd(R, BtPB)
    Sinv = inverse(S)
    BtPA = matmul(BtP, A)
    K = matmul(Sinv, BtPA)
    return K, P
