import numpy as np
class MlpPolicy:
    def __init__(self):
        self.w0 = np.random.randn(64, 22) * 0.1
        self.b0 = np.zeros(64)
        self.w1 = np.random.randn(4, 64) * 0.1
        self.b1 = np.zeros(4)
    def forward(self, obs):
        h0 = np.tanh(np.dot(self.w0, obs) + self.b0)
        return np.clip(np.dot(self.w1, h0) + self.b1, -1.0, 1.0)
def action_to_velocity(action):
    return action[0] * 3.0, action[1] * 3.0, action[2] * 1.5, action[3]
