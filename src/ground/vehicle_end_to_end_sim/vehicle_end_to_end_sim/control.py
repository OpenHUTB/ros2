"""控制逻辑 —— 键盘转向 + 平滑回正。

左右方向键控制前轮转向角 steer（范围 [-1, 1]）；松开按键后转向角自动回正，
避免车辆持续偏向一侧。转向角变化速率由 config.STEER_SPEED 控制。
"""

from . import config


def update_steering(steer_left, steer_right, car, steer_speed=config.STEER_SPEED):
    """更新车辆转向角。

    参数
    ----
    steer_left / steer_right : bool
        是否按下左 / 右方向键（由 pygame.key.get_pressed() 提供）。
    car : Car
        车辆对象，读 / 写 steer。
    steer_speed : float
        转向角变化速率。

    返回
    ----
    float : 更新后的 steer（已限制在 [-1, 1]）。
    """
    if steer_left:
        car.steer -= steer_speed
    elif steer_right:
        car.steer += steer_speed
    else:
        # 松开按键自动回正
        if car.steer > 0.0:
            car.steer -= steer_speed
            if car.steer < 0.0:
                car.steer = 0.0
        elif car.steer < 0.0:
            car.steer += steer_speed
            if car.steer > 0.0:
                car.steer = 0.0

    car.steer = max(-config.MAX_STEER, min(config.MAX_STEER, car.steer))
    return car.steer
