# -*- coding: utf-8 -*-
"""分层任务优先级全身逆运动学（task-priority whole-body IK）。

为什么还需要一层
----------------
``ik_solver`` 只处理"单个末端任务"：给定位姿，反解关节角。真实人形机器人
上同时有好几个目标：手要到位、腕姿态别歪、关节别贴限位、别掉进奇异。
这些目标常常互相冲突，简单加权求和（``w1·e1 + w2·e2``）调权重很痛苦：
权重大了末端不准，权重小了次级目标形同虚设。

做法
----
按优先级把任务排成栈，**高优先级任务的结果不允许被低优先级破坏**：

    Δq = J₁⁺e₁ + N₁·J₂⁺e₂ + N₁N₂·J₃⁺e₃ + …

其中

    N_i = I − J_i⁺J_i        第 i 个任务的零空间投影，满足 J_i·N_i = 0

实现上不显式求伪逆，而是用阻尼最小二乘（沿用 ``ik_solver.damped_step``），
并递推更新投影矩阵::

    N_i = N_{i−1} − N_{i−1}Jᵢᵀ(JᵢN_{i−1}Jᵢᵀ + λ²I)⁻¹JᵢN_{i−1}

阻尼 λ 的作用和单任务时一样：奇异位形附近不发散，只是会牺牲一点精度。

和已有模块的关系
----------------
* ``kinematics`` 提供正解与雅可比
* ``ik_solver`` 提供阻尼步长与旋转误差
* ``manipulability.joint_limit_cost`` 提供限位代价，以前只在演示里打印，
  现在真正接到零空间里当次级任务用
"""

from __future__ import annotations

from ik_solver import (adaptive_damping, damped_step, default_initial_guess,
                       rotation_error)
from linalg import identity, mat_mul, mat_vec, norm, solve, transpose, zeros
from manipulability import joint_limit_cost

__all__ = [
    "Task",
    "PositionTask",
    "PoseTask",
    "PostureTask",
    "LimitAvoidanceTask",
    "WholeBodyResult",
    "solve_whole_body",
]


# ---------------------------------------------------------------------------
# 矩阵小工具（linalg 只提供单右端项的 solve）
# ---------------------------------------------------------------------------
#: 零空间投影矩阵用的阻尼。步长可以放心用大阻尼求稳，投影不行：λ 大了
#: 零空间就"漏"，次级任务会悄悄改掉主任务。取 1e-6 是数值精度和稳定性
#: 的折中（再小 linalg.solve 会把矩阵判成奇异）。
_PROJ_DAMPING = 1e-6


def _augment(a, lam):
    """给方阵对角线加 λ²。"""
    out = [list(row) for row in a]
    lam2 = lam * lam
    for i in range(len(out)):
        out[i][i] += lam2
    return out


def _mat_solve(a, b):
    """解 A X = B：把 B 按列拆成多个右端项，逐列调用 linalg.solve。"""
    rows = len(b)
    cols = len(b[0])
    out = [[0.0] * cols for _ in range(rows)]
    for c in range(cols):
        rhs = [b[r][c] for r in range(rows)]
        sol = solve(a, rhs, tol=1e-14)
        for r in range(rows):
            out[r][c] = sol[r]
    return out


def _projector_update(n_mat, j, lam):
    """零空间投影矩阵递推：N ← N − N Jᵀ (J N Jᵀ + λ²I)⁻¹ J N。

    新矩阵满足 J·N_new ≈ 0（阻尼项让它只是近似为 0，λ 越小越精确）。
    """
    jn = mat_mul(j, n_mat)
    a = _augment(mat_mul(jn, transpose(j)), lam)
    x = _mat_solve(a, jn)
    njt = mat_mul(n_mat, transpose(j))
    delta = mat_mul(njt, x)
    n = len(n_mat)
    return [[n_mat[i][k] - delta[i][k] for k in range(n)] for i in range(n)]


# ---------------------------------------------------------------------------
# 任务定义
# ---------------------------------------------------------------------------
class Task(object):
    """一个任务：给出当前误差与误差对关节角的雅可比。

    ``error(chain, q)`` 返回长度 m 的误差向量，``jacobian(chain, q)``
    返回 m×n 的雅可比，二者维数必须一致。

    ``joint_space = True`` 表示这个任务直接在关节空间里工作（雅可比是单位阵），
    求解器会跳过阻尼求解、直接把误差投影进零空间——用阻尼公式解单位阵
    会把零空间之外的分量除以 λ²，数值上会爆炸。
    """

    name = "task"
    joint_space = False

    def error(self, chain, q):
        raise NotImplementedError

    def jacobian(self, chain, q):
        raise NotImplementedError

    def __repr__(self):
        return "%s(name=%r)" % (type(self).__name__, self.name)


class PositionTask(Task):
    """末端位置任务（3 维）。"""

    def __init__(self, target, name="position"):
        if len(target) != 3:
            raise ValueError("位置目标必须是三维")
        self.target = [float(x) for x in target]
        self.name = name

    def error(self, chain, q):
        p, _r = chain.fk(q)
        return [self.target[i] - p[i] for i in range(3)]

    def jacobian(self, chain, q):
        return chain.position_jacobian(q)


class PoseTask(Task):
    """末端位姿任务（6 维：位置 + 旋转向量）。

    ``rotation_weight`` 把弧度误差折算到米量级，和 ``solve_pose_ik`` 一致。
    """

    def __init__(self, target_pos, target_rot, rotation_weight=0.3, name="pose"):
        if len(target_pos) != 3:
            raise ValueError("位置目标必须是三维")
        if len(target_rot) != 3 or len(target_rot[0]) != 3:
            raise ValueError("姿态目标必须是 3x3")
        if rotation_weight <= 0:
            raise ValueError("rotation_weight 必须为正")
        self.target_pos = [float(x) for x in target_pos]
        self.target_rot = [list(row) for row in target_rot]
        self.rotation_weight = float(rotation_weight)
        self.name = name

    def error(self, chain, q):
        p, r = chain.fk(q)
        e_pos = [self.target_pos[i] - p[i] for i in range(3)]
        e_rot = rotation_error(self.target_rot, r)
        return e_pos + [self.rotation_weight * x for x in e_rot]

    def jacobian(self, chain, q):
        j = chain.jacobian(q)
        w = self.rotation_weight
        return [list(row) for row in j[:3]] + [[w * x for x in row] for row in j[3:]]


class PostureTask(Task):
    """姿态保持任务：把关节拉向参考位形。

    这一层没有物理意义的上层任务，所以通常放在最低优先级——它只占用
    末端任务用不到的零空间，让冗余的机械臂保持"自然"的关节角。

    权重直接乘在误差上（雅可比保持单位阵），方便和限位任务叠加使用。
    """

    joint_space = True

    def __init__(self, q_ref, weight=0.5, name="posture"):
        self.q_ref = [float(x) for x in q_ref]
        self.weight = float(weight)
        if self.weight <= 0:
            raise ValueError("weight 必须为正")
        self.name = name

    def error(self, chain, q):
        if len(self.q_ref) != chain.dof:
            raise ValueError("参考位形维度 %d 与自由度 %d 不符"
                             % (len(self.q_ref), chain.dof))
        return [self.weight * (self.q_ref[i] - q[i]) for i in range(chain.dof)]

    def jacobian(self, chain, q):
        return identity(chain.dof)


class LimitAvoidanceTask(Task):
    """关节限位规避任务：沿限位代价的负梯度方向运动。

    代价来自 ``manipulability.joint_limit_cost``（越贴近限位越大），
    这里取负梯度当"误差"，雅可比取单位阵——关节空间里的直接运动。
    放到零空间里用，就不会为了躲限位而牺牲末端精度。

    ``max_step`` 限制单步幅度：贴边时代价梯度会很陡，不限幅会一步跳出界。
    """

    def __init__(self, margin=0.1, gain=0.5, max_step=0.2,
                 name="limit_avoidance"):
        if gain <= 0:
            raise ValueError("gain 必须为正")
        if max_step <= 0:
            raise ValueError("max_step 必须为正")
        self.margin = float(margin)
        self.gain = float(gain)
        self.max_step = float(max_step)
        self.name = name
        self.last_cost = 0.0
        self.joint_space = True

    def error(self, chain, q):
        cost, grad = joint_limit_cost(q, chain.joints, margin=self.margin)
        self.last_cost = cost
        e = [-self.gain * g for g in grad]
        e_norm = norm(e)
        if e_norm > self.max_step:
            e = [x * self.max_step / e_norm for x in e]
        return e

    def jacobian(self, chain, q):
        return identity(chain.dof)


# ---------------------------------------------------------------------------
# 结果与求解器
# ---------------------------------------------------------------------------
class WholeBodyResult(object):
    """全身逆解结果。

    success       主任务是否在容差内收敛
    q             关节角
    error         主任务误差范数
    iterations    迭代次数
    history       每次迭代的主任务误差
    task_errors   收敛时各任务的误差范数（按优先级顺序）
    null_step     次级任务单步的最大范数（衡量零空间里动了多少）
    reason        结束原因
    """

    def __init__(self, success, q, error, iterations, history,
                 task_errors=None, null_step=0.0, reason=""):
        self.success = success
        self.q = q
        self.error = error
        self.iterations = iterations
        self.history = history
        self.task_errors = list(task_errors or [])
        self.null_step = null_step
        self.reason = reason

    def __repr__(self):
        return ("WholeBodyResult(%s, err=%.2e, it=%d, tasks=%s)"
                % ("ok" if self.success else "fail", self.error,
                   self.iterations,
                   ",".join("%.1e" % e for e in self.task_errors)))


def _choose_damping(j, damping):
    if damping == "adaptive":
        return adaptive_damping(j)
    if damping == "fixed":
        return 1e-3
    return float(damping)


def _task_errors(chain, q, tasks):
    return [norm(t.error(chain, q)) for t in tasks]


def solve_whole_body(chain, tasks, q0=None, tol=1e-6, max_iter=200,
                     step_size=1.0, damping="adaptive", has_limits=True,
                     step_tol=1e-5):
    """按任务优先级求解全身逆解。

    参数
    ----
    chain       kinematics.Chain
    tasks       任务列表，**顺序即优先级**（tasks[0] 优先级最高）
    q0          初值；缺省取关节限位中点（同 ``ik_solver``）
    tol         主任务收敛容差（与主任务的误差同量纲）
    max_iter    最大迭代次数
    step_size   每步缩放
    damping     "adaptive" / "fixed" / 具体数值
    has_limits  是否把解夹进关节限位（硬约束，最后一步执行）
    step_tol    零空间步长收敛阈值；主任务达标后继续在零空间里迭代，
                直到次级任务"无处可动"或到达 max_iter

    返回 ``WholeBodyResult``。

    注意
    ----
    * 关节限位是硬夹紧，不参与优先级博弈：夹紧后再算雅可比，下一步会
      自动补偿。所以贴边时主任务可能暂时变差，这是物理约束的必然结果。
    * 主任务达标后不会立刻返回——否则零空间只用了头几步，姿态/限位
      这些次级任务还没跑到受限最优。
    """
    if not tasks:
        raise ValueError("任务列表不能为空")
    if tol <= 0:
        raise ValueError("tol 必须为正")
    if step_tol <= 0:
        raise ValueError("step_tol 必须为正")
    if max_iter <= 0:
        raise ValueError("max_iter 必须为正")
    if step_size <= 0:
        raise ValueError("step_size 必须为正")

    n = chain.dof
    if q0 is None:
        q = default_initial_guess(chain)
    else:
        if len(q0) != n:
            raise ValueError("初值维度 %d 与自由度 %d 不符" % (len(q0), n))
        q = [float(x) for x in q0]
    if has_limits:
        q = chain.clamp_all(q)

    primary = tasks[0]
    history = []
    reason = "达到最大迭代次数"
    last_null_step = float("inf")
    best_secondary = float("inf")
    stall = 0

    for it in range(max_iter):
        errs = _task_errors(chain, q, tasks)
        history.append(errs[0])

        # 收敛判据：
        #   主任务达标，并且次级任务已经"无处可动"（零空间步长趋近 0）或者
        #   连续若干次不再改进（受主任务约束的受限最优）。
        #   不能要求次级任务误差趋近 0——姿态/限位这类任务的最优值通常
        #   就是一个非零的受限最优。
        primary_ok = errs[0] < tol
        settled = len(tasks) == 1 or last_null_step < step_tol
        secondary = sum(errs[1:])
        if secondary < best_secondary - 1e-9:
            best_secondary = secondary
            stall = 0
        else:
            stall += 1

        if primary_ok and (settled or stall >= 10):
            return WholeBodyResult(True, q, errs[0], it, history, errs,
                                   last_null_step if last_null_step != float("inf") else 0.0,
                                   "收敛" if settled else "次级任务达到受限最优")

        dq_full = [0.0] * n
        n_mat = identity(n)
        null_step = 0.0
        failed = False

        for idx, task in enumerate(tasks):
            e = task.error(chain, q)
            if not e:                      # 该任务当前无误差，跳过
                continue

            if task.joint_space:
                # 关节空间任务：直接用零空间投影，不走阻尼公式
                step = mat_vec(n_mat, e)
                if idx > 0:
                    null_step = max(null_step, norm(step))
                for i in range(n):
                    dq_full[i] += step[i]
                # 关节空间任务占满剩余自由度，后面没有可用的零空间了
                n_mat = zeros(n, n)
                continue

            j = task.jacobian(chain, q)
            jn = mat_mul(j, n_mat)
            lam = _choose_damping(jn, damping)
            try:
                step = damped_step(jn, e, lam)
            except ValueError:
                reason = "线性求解失败（矩阵奇异）"
                failed = True
                break

            if idx > 0:
                null_step = max(null_step, norm(step))
            for i in range(n):
                dq_full[i] += step[i]
            # 投影矩阵用小阻尼：λ 大了零空间就不"干净"，次级任务会漏到
            # 主任务上。步长的阻尼可以大（求稳），投影的阻尼必须小（求准）。
            n_mat = _projector_update(n_mat, j, _PROJ_DAMPING)

        if failed:
            break

        q = [q[i] + step_size * dq_full[i] for i in range(n)]
        if has_limits:
            q = chain.clamp_all(q)
        last_null_step = null_step

    errs = _task_errors(chain, q, tasks)
    if errs[0] < tol:
        reason = "主任务收敛（次级任务未完全稳定）"
    return WholeBodyResult(False, q, errs[0], len(history), history,
                           errs, last_null_step if last_null_step != float("inf") else 0.0,
                           reason)
