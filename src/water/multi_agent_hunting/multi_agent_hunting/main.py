"""多 AUV 协同围捕的启动入口。

用法::

    ros2 run multi_agent_hunting multi_agent_hunting
    # 或直接
    python3 -m multi_agent_hunting.main --seed 7 --max-steps 300
"""
import argparse
import os

from multi_agent_hunting.core import HuntingScenario


def _config_path(explicit):
    if explicit:
        return explicit
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(
            get_package_share_directory('multi_agent_hunting'),
            'scenario.json')
    except Exception:
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        return os.path.join(root, 'scenario.json')


def main(argv=None):
    parser = argparse.ArgumentParser(description='多 AUV 协同围捕示例')
    parser.add_argument('--scenario', default=None, help='场景配置文件路径')
    parser.add_argument('--seed', type=int, default=None,
                        help='覆盖配置中的随机种子，用于复现')
    parser.add_argument('--max-steps', type=int, default=None,
                        help='覆盖配置中的最大步数')
    args = parser.parse_args(argv)

    config = HuntingScenario.load_config(_config_path(args.scenario))
    if args.seed is not None:
        config['seed'] = args.seed
    if args.max_steps is not None:
        config['max_steps'] = args.max_steps

    scenario = HuntingScenario(config)
    result = scenario.run()
    print('结果: %s，步数 %d'
          % ('捕获成功' if result['captured'] else '未捕获',
             result['steps']))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
