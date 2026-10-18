"""Command-line episode collector."""

import argparse
from pathlib import Path

from .episode import (
    EpisodeCollector,
    load_episode_config,
    split_episode_ids,
    write_episode,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Collect PD expert episodes / 采集 PD 专家控制 episode 数据"
    )
    parser.add_argument(
        "--model", type=Path, required=True, help="MJCF model / MJCF 模型路径"
    )
    parser.add_argument(
        "--config", type=Path, required=True, help="Episode YAML / Episode 配置"
    )
    parser.add_argument("--output", type=Path, help="Output directory / 输出目录")
    return parser


def main(argv=None) -> int:
    arguments = build_parser().parse_args(argv)
    seed, configured_output, specs = load_episode_config(arguments.config)
    output = arguments.output or configured_output
    collector = EpisodeCollector(arguments.model)
    for spec in specs:
        rows = collector.run(spec)
        write_episode(output, spec, rows, seed)
        print(
            f"Collected {spec.episode_id}: {len(rows)} samples / 已采集 {len(rows)} 条样本"
        )
    split = split_episode_ids((spec.episode_id for spec in specs), seed)
    print(f"Episode split / Episode 划分: {split}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
