"""相机版车道偏离预警模块的统一入口。

模式：
    --probe       探路，确认路面车道线能被拍到、投影对得上
    --collect     连 CARLA 采图像与真值
    --train       用采好的数据训练并评价
    --demo        连 CARLA 边跑边比，视觉偏移与地图真值对照
    --self-check  合成数据自检，不需要 CARLA
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

DEFAULT_MODEL = os.path.join('models', 'lane_regressor.npz')
DEFAULT_DATA = os.path.join('data', 'lane_dataset.npz')


def build_parser():
    parser = argparse.ArgumentParser(
        description='相机版车道偏离预警（视觉车道线 + 地图真值对照）')
    parser.add_argument('--probe', action='store_true',
                        help='探路：抓几帧图，把地图推出的车道边界投到图上')
    parser.add_argument('--collect', action='store_true', help='采集数据集')
    parser.add_argument('--train', action='store_true', help='训练并评价')
    parser.add_argument('--demo', action='store_true',
                        help='在线演示：视觉偏移与地图真值逐帧对照')
    parser.add_argument('--self-check', action='store_true',
                        help='合成数据自检，不需要 CARLA')

    parser.add_argument('--host', default='127.0.0.1', help='CARLA 服务端地址')
    parser.add_argument('--port', type=int, default=2000, help='CARLA RPC 端口')
    parser.add_argument('--town', default='', help='地图名，留空则用当前地图')
    parser.add_argument('--vehicle', default='vehicle.tesla.model3', help='自车蓝图')
    parser.add_argument('--spawn-index', type=int, default=0, help='出生点索引')
    parser.add_argument('--frames', type=int, default=3000, help='推进多少帧')
    parser.add_argument('--every', type=int, default=1, help='每多少帧采一帧')
    parser.add_argument('--dt', type=float, default=0.05, help='同步步长（秒）')
    parser.add_argument('--width', type=int, default=800, help='图像宽度')
    parser.add_argument('--height', type=int, default=600, help='图像高度')
    parser.add_argument('--fov', type=float, default=90.0, help='水平视场角（度）')
    parser.add_argument('--out-dir', default=os.path.join('out', 'probe'),
                        help='探路图的输出目录')
    parser.add_argument('--no-augment', action='store_true',
                        help='关闭左右镜像扩增')
    parser.add_argument('--max-offset', type=float, default=1.6,
                        help='真值偏移超过它就不计入统计（米）；默认 1.6，'
                             '比 depart_enter 0.9 宽，大偏移样本不丢')
    parser.add_argument('--min-paint', type=float, default=12.0,
                        help='边界处的最小线亮度增益，低于它认为没画线')
    parser.add_argument('--settle', type=int, default=40,
                        help='前多少帧不开采，让车先开出路口')
    parser.add_argument('--top', type=int, default=4,
                        help='保存线亮度得分最高的几帧')

    parser.add_argument('--data', default=DEFAULT_DATA, help='数据集路径')
    parser.add_argument('--checkpoint', default=DEFAULT_MODEL, help='权重路径')
    parser.add_argument('--epochs', type=int, default=120, help='训练轮数')
    parser.add_argument('--lr', type=float, default=0.005, help='学习率')
    parser.add_argument('--batch-size', type=int, default=256, help='批大小')
    parser.add_argument('--record', default='', help='逐帧存 PNG 的目录，用于合动图')
    parser.add_argument('--lane-change', type=int, default=0,
                        help='每多少帧强制变道一次，0 表示不变道')
    parser.add_argument('--sway', type=float, default=0.0,
                        help='演示时的目标横向摆动幅度（米），0 表示用自动驾驶')
    parser.add_argument('--sway-freq', type=float, default=0.25,
                        help='摆动频率（Hz）')
    parser.add_argument('--throttle', type=float, default=0.35,
                        help='摆动模式下的油门')
    parser.add_argument('--smooth', type=float, default=0.4,
                        help='偏移的一阶平滑系数，1 表示不平滑')
    parser.add_argument('--event-tolerance', type=float, default=0.1,
                        help='两路事件对齐的时间容差（秒）')
    parser.add_argument('--save-metrics', default='',
                        help='把演示指标写到这个 json 路径')
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)

    if args.self_check:
        from carla_visual_lane_departure_warning import selfcheck
        result = selfcheck.run()
        print('\n自检结论：合成数据上偏移误差 %.3f m，链路可用'
              % result['mae_offset'])
        return 0

    if args.probe:
        from carla_visual_lane_departure_warning import probe
        return probe.run(args)

    if args.collect:
        from carla_visual_lane_departure_warning import dataset
        dataset.collect(args)
        return 0

    if args.train:
        return train(args)

    if args.demo:
        from carla_visual_lane_departure_warning import demo
        return demo.run(args)

    print('请指定一个模式：--probe / --collect / --train / --demo / --self-check')
    return 2


def train(args):
    """训练、评价、存权重与指标。"""
    from carla_visual_lane_departure_warning import dataset, metrics
    from carla_visual_lane_departure_warning import model as net

    data = dataset.load(args.data)
    if not args.no_augment:
        before = data['images'].shape[0]
        data = dataset.augment_flip(data)
        print('镜像扩增：%d -> %d 帧' % (before, data['images'].shape[0]))
    train_set, val_set = dataset.split(data, seed=0)
    print('训练 %d 帧，验证 %d 帧' % (train_set['images'].shape[0],
                                    val_set['images'].shape[0]))

    regressor = net.LaneRegressor()
    print('参数量 %d' % net.count_parameters(regressor))
    regressor.fit(train_set['images'], train_set['targets'],
                  epochs=args.epochs, lr=args.lr,
                  batch_size=args.batch_size, seed=0)

    prediction = regressor.predict(net.flatten_images(val_set['images']))
    result = metrics.evaluate(prediction, val_set['targets'],
                              val_set.get('speeds'))
    print('\n' + metrics.format_report(result))

    os.makedirs(os.path.dirname(args.checkpoint) or '.', exist_ok=True)
    regressor.save(args.checkpoint)
    metrics_path = os.path.join(os.path.dirname(args.checkpoint), 'metrics.json')
    with open(metrics_path, 'w', encoding='utf-8') as handle:
        json.dump({'source': args.data, **result}, handle,
                  ensure_ascii=False, indent=2)

    size_kb = os.path.getsize(args.checkpoint) / 1024.0
    print('\n权重 %s（%.1f KB）' % (args.checkpoint, size_kb))
    print('指标 %s' % metrics_path)
    return 0


if __name__ == '__main__':
    sys.exit(main())