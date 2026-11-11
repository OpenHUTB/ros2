"""离线自检：用合成图像把「图像 → 网络 → 边界位置 → 偏移」整条链路跑一遍。

合成图的画法故意简单：两条亮线、位置由横向偏移决定。网络能学会就说明
数据形状、归一化、损失和推理口径没写错；学不会说明代码有 bug，
与 CARLA 无关。
"""
import numpy as np

from . import metrics
from . import model as net


def synthetic_dataset(count=600, seed=0):
    """生成 (图像, 真值) 对。真值是左右边界的横向位置，单位米。"""
    rng = np.random.default_rng(seed)
    images = np.zeros((count, net.INPUT_H, net.INPUT_W), dtype=np.uint8)
    targets = np.zeros((count, 2), dtype=np.float64)

    for index in range(count):
        centre = rng.uniform(-1.5, 1.5)          # 车道中心相对自车的横向位置
        width = rng.uniform(3.0, 3.8)            # 车道宽
        left, right = centre - width / 2.0, centre + width / 2.0
        targets[index] = (left, right)

        image = np.zeros((net.INPUT_H, net.INPUT_W), dtype=np.float64)
        for lane in (left, right):
            # 越远越往画面中间收，模拟透视
            for row in range(6, net.INPUT_H - 2):
                depth = (net.INPUT_H - row) / float(net.INPUT_H)
                column = net.INPUT_W / 2.0 + lane * (8.0 * depth)
                col = int(round(column))
                if 0 <= col < net.INPUT_W:
                    image[row, col] = 235.0
        image += rng.normal(0.0, 6.0, image.shape)
        images[index] = np.clip(image, 0, 255).astype(np.uint8)
    return images, targets


def run(verbose=True):
    """训练一个小网络并评价，返回指标字典。"""
    images, targets = synthetic_dataset()
    cut = int(len(images) * 0.8)
    train_images, train_targets = images[:cut], targets[:cut]
    val_images, val_targets = images[cut:], targets[cut:]

    regressor = net.LaneRegressor()
    if verbose:
        print('参数量 %d，训练 %d / 验证 %d'
              % (net.count_parameters(regressor), cut, len(images) - cut))
    regressor.fit(train_images, train_targets, epochs=20, verbose=verbose)

    prediction = regressor.predict(net.flatten_images(val_images))
    result = metrics.evaluate(prediction, val_targets)
    if verbose:
        print('\n' + metrics.format_report(result))
    return result


if __name__ == '__main__':
    run()