"""
ppo_agent.py — 手写 PPO（Proximal Policy Optimization）。
状态 4 维 [x, y, vx, vy]，动作 2 维 [Fx, Fy]（限幅）。
"""
import numpy as np
import torch
import torch.nn as nn
from . import common


class Actor(nn.Module):
    def __init__(self, s_dim=4, a_dim=2, h=64):
        super().__init__()
        self.net = nn.Sequential(nn.Linear(s_dim, h), nn.Tanh(),
                                 nn.Linear(h, h), nn.Tanh(),
                                 nn.Linear(h, a_dim))
        self.log_std = nn.Parameter(torch.zeros(a_dim))

    def forward(self, s):
        mu = self.net(s)
        std = torch.exp(self.log_std)
        return mu, std


class Critic(nn.Module):
    def __init__(self, s_dim=4, h=64):
        super().__init__()
        self.net = nn.Sequential(nn.Linear(s_dim, h), nn.Tanh(),
                                 nn.Linear(h, h), nn.Tanh(),
                                 nn.Linear(h, 1))

    def forward(self, s):
        return self.net(s)


class PPO:
    def __init__(self, lr=3e-4, gamma=0.99, eps_clip=0.2):
        self.actor = Actor()
        self.critic = Critic()
        self.opt_a = torch.optim.Adam(self.actor.parameters(), lr=lr)
        self.opt_c = torch.optim.Adam(self.critic.parameters(), lr=lr)
        self.gamma = gamma
        self.eps_clip = eps_clip

    def select_action(self, s):
        with torch.no_grad():
            s_t = torch.tensor(s, dtype=torch.float32)
            mu, std = self.actor(s_t)
            dist = torch.distributions.Normal(mu, std)
            a = dist.sample()
            logp = dist.log_prob(a).sum().item()
        return a.numpy(), logp

    def update(self, s_list, a_list, r_list, logp_old_list):
        s = torch.tensor(np.array(s_list), dtype=torch.float32)
        a = torch.tensor(np.array(a_list), dtype=torch.float32)
        logp_old = torch.tensor(logp_old_list, dtype=torch.float32)
        # 回报
        R = 0
        returns = []
        for r in reversed(r_list):
            R = r + self.gamma * R
            returns.insert(0, R)
        returns = torch.tensor(returns, dtype=torch.float32)
        if returns.std() > 1e-6:
            returns = (returns - returns.mean()) / returns.std()
        # 更新 actor
        mu, std = self.actor(s)
        dist = torch.distributions.Normal(mu, std)
        logp = dist.log_prob(a).sum(dim=1)
        adv = returns - self.critic(s).squeeze()
        ratio = torch.exp(logp - logp_old)
        surr1 = ratio * adv.detach()
        surr2 = torch.clamp(ratio, 1-self.eps_clip, 1+self.eps_clip) * adv.detach()
        loss_a = -torch.min(surr1, surr2).mean()
        self.opt_a.zero_grad(); loss_a.backward(); self.opt_a.step()
        # 更新 critic
        loss_c = nn.MSELoss()(self.critic(s).squeeze(), returns)
        self.opt_c.zero_grad(); loss_c.backward(); self.opt_c.step()
        return loss_a.item(), loss_c.item()


def train_ppo(episodes: int = 200):
    """在简化 2D 环境中训练 PPO 跟踪目标点 (2, 0)。
    先用 PID 做行为克隆预热（BC warmup），再 PPO 微调。
    为什么这么做：纯 PPO 在 CPU 200 episode 内很难收敛，
    先用专家数据让 actor 学会"往目标方向走"，再 PPO 优化。
    """
    import mujoco
    common.set_mujoco_gl()
    model = mujoco.MjModel.from_xml_path(str(common.get_package_root() / "models" / "rov_ppo_pool.xml"))
    data = mujoco.MjData(model)

    ppo = PPO(lr=1e-3)
    target = np.array([2.0, 0.0])

    # === 阶段 1：PID 行为克隆预热（30 epoch）===
    print("[BC] 阶段 1: PID 模仿学习预热")
    bc_optimizer = torch.optim.Adam(ppo.actor.parameters(), lr=1e-3)
    for ep in range(30):
        mujoco.mj_resetData(model, data)
        prev_err = np.zeros(2); integral = np.zeros(2)
        total_loss = 0; n = 0
        for step in range(200):
            s = np.array([data.qpos[0], data.qpos[1], data.qvel[0], data.qvel[1]])
            err = target - s[:2]
            F, integral, prev_err = common.pid_control(err, prev_err, integral, kp=2.0, kd=0.5)
            # BC loss: 让 actor 输出接近 PID
            mu, _ = ppo.actor(torch.tensor(s, dtype=torch.float32))
            loss = torch.nn.functional.mse_loss(mu, torch.tensor(F, dtype=torch.float32))
            bc_optimizer.zero_grad(); loss.backward(); bc_optimizer.step()
            total_loss += loss.item(); n += 1
            data.qfrc_applied[0] = F[0] - 2.0*data.qvel[0]
            data.qfrc_applied[1] = F[1] - 2.0*data.qvel[1]
            mujoco.mj_step(model, data)
        if ep % 10 == 0:
            print(f"[BC] ep {ep:3d}  loss={total_loss/n:.4f}")

    # === 阶段 2：PPO 微调 ===
    print("[PPO] 阶段 2: PPO 微调")
    for ep in range(episodes):
        mujoco.mj_resetData(model, data)
        s = np.array([data.qpos[0], data.qpos[1], data.qvel[0], data.qvel[1]])
        s_list, a_list, r_list, lp_list = [], [], [], []
        for step in range(200):
            a, logp = ppo.select_action(s)
            a = np.clip(a, -5.0, 5.0)
            data.qfrc_applied[0] = a[0] - 2.0 * data.qvel[0]
            data.qfrc_applied[1] = a[1] - 2.0 * data.qvel[1]
            mujoco.mj_step(model, data)
            s_next = np.array([data.qpos[0], data.qpos[1], data.qvel[0], data.qvel[1]])
            err = np.linalg.norm(target - s_next[:2])
            r = -err
            s_list.append(s); a_list.append(a); r_list.append(r); lp_list.append(logp)
            s = s_next
        loss_a, loss_c = ppo.update(s_list, a_list, r_list, lp_list)
        if ep % 20 == 0 or ep == episodes-1:
            final_err = np.linalg.norm(target - s[:2])
            print(f"[PPO] ep {ep:3d}/{episodes}  final_err={final_err:.3f}  loss_a={loss_a:.4f}")

    wdir = common.get_package_root() / "weights"
    wdir.mkdir(exist_ok=True)
    torch.save(ppo.actor.state_dict(), common.find_weights_path())
    print(f"[PPO] 权重保存到 {common.find_weights_path()}")
    return ppo


def test_ppo():
    """对比 PPO 与 PID 的轨迹跟踪 MAE。"""
    import mujoco
    common.set_mujoco_gl()
    model = mujoco.MjModel.from_xml_path(str(common.get_package_root() / "models" / "rov_ppo_pool.xml"))
    data = mujoco.MjData(model)
    target = np.array([2.0, 0.0])

    # PID 基线
    mujoco.mj_resetData(model, data)
    prev_err = np.zeros(2); integral = np.zeros(2)
    pid_errors = []
    for step in range(200):
        s = np.array([data.qpos[0], data.qpos[1]])
        err = target - s
        F, integral, prev_err = common.pid_control(err, prev_err, integral, kp=2.0, kd=0.5)
        data.qfrc_applied[0] = F[0] - 2.0*data.qvel[0]
        data.qfrc_applied[1] = F[1] - 2.0*data.qvel[1]
        mujoco.mj_step(model, data)
        pid_errors.append(np.linalg.norm(target - np.array([data.qpos[0], data.qpos[1]])))

    # PPO
    wpath = common.find_weights_path()
    if not wpath.exists():
        raise FileNotFoundError(f"找不到权重 {wpath}，请先运行: python3 main.py train")
    actor = Actor()
    actor.load_state_dict(torch.load(wpath, map_location="cpu"))
    actor.eval()
    mujoco.mj_resetData(model, data)
    ppo_errors = []
    for step in range(200):
        s = np.array([data.qpos[0], data.qpos[1], data.qvel[0], data.qvel[1]])
        with torch.no_grad():
            mu, _ = actor(torch.tensor(s, dtype=torch.float32))
            a = mu.numpy()
        a = np.clip(a, -5.0, 5.0)
        data.qfrc_applied[0] = a[0] - 2.0*data.qvel[0]
        data.qfrc_applied[1] = a[1] - 2.0*data.qvel[1]
        mujoco.mj_step(model, data)
        ppo_errors.append(np.linalg.norm(target - np.array([data.qpos[0], data.qpos[1]])))

    pid_mae = np.mean(pid_errors)
    ppo_mae = np.mean(ppo_errors)
    print(f"[对比] PID  MAE = {pid_mae:.3f} m")
    print(f"[对比] PPO  MAE = {ppo_mae:.3f} m")
    return pid_mae, ppo_mae
