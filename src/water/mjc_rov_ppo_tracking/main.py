#!/usr/bin/env python3
"""main.py — 统一入口。"""
import argparse, os, sys
_PKG_DIR = os.path.dirname(os.path.abspath(__file__))
if _PKG_DIR not in sys.path:
    sys.path.insert(0, _PKG_DIR)
from mjc_rov_ppo_tracking import ppo_agent

def main():
    p = argparse.ArgumentParser(prog="mjc_rov_ppo_tracking")
    sub = p.add_subparsers(dest="cmd")
    sub.add_parser("train")
    sub.add_parser("test")
    args = p.parse_args()
    if not args.cmd or args.cmd == "all":
        print("== [1/2] 训练 PPO ==")
        ppo_agent.train_ppo(episodes=200)
        print("== [2/2] PPO vs PID 对比 ==")
        ppo_agent.test_ppo()
    elif args.cmd == "train":
        ppo_agent.train_ppo(episodes=200)
    elif args.cmd == "test":
        ppo_agent.test_ppo()

if __name__ == "__main__":
    main()
