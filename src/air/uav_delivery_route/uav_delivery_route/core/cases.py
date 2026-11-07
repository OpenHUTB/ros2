#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Delivery scenarios: one depot + fixed station layouts (deterministic).

三个 case 全部用固定坐标，保证 README 数字可复现、无随机种子问题。
"""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class Station:
    """One pick-up/drop-off station."""
    sid: str
    x: float
    y: float
    weight_kg: float


@dataclass(frozen=True)
class Case:
    """A delivery scenario: depot at origin + station list."""
    name: str
    description: str
    depot: tuple = (0.0, 0.0)
    stations: tuple = field(default_factory=tuple)


def _s(i, x, y, w):
    return Station("S%02d" % i, x, y, w)


CASE_FARM_DENSE = Case(
    "case_farm_dense",
    "dense orchard belt, 12 close stations, light parcels, single sortie",
    stations=(
        _s(1, -120.0, 80.0, 0.20), _s(2, 60.0, 150.0, 0.15),
        _s(3, 210.0, 90.0, 0.25), _s(4, 150.0, -130.0, 0.18),
        _s(5, -40.0, -170.0, 0.22), _s(6, -200.0, -60.0, 0.12),
        _s(7, 90.0, 300.0, 0.20), _s(8, -150.0, 220.0, 0.16),
        _s(9, 260.0, -60.0, 0.24), _s(10, 30.0, -320.0, 0.19),
        _s(11, -260.0, 120.0, 0.17), _s(12, 180.0, 210.0, 0.22),
    ))

CASE_FARM_SPARSE = Case(
    "case_farm_sparse",
    "scattered farms 0.8-1.6 km out, multi-sortie with battery swaps",
    stations=(
        _s(1, 900.0, 200.0, 0.8), _s(2, 1400.0, 500.0, 1.2),
        _s(3, 700.0, 900.0, 0.6), _s(4, 1200.0, -300.0, 1.0),
        _s(5, 1500.0, -800.0, 0.9), _s(6, -800.0, 600.0, 1.4),
        _s(7, -1300.0, 900.0, 0.7), _s(8, -1100.0, -500.0, 1.1),
        _s(9, -1500.0, -200.0, 0.8), _s(10, 300.0, 1400.0, 1.0),
        _s(11, -400.0, 1500.0, 0.9), _s(12, 600.0, -1400.0, 1.3),
    ))

CASE_FARM_HEAVY = Case(
    "case_farm_heavy",
    "5 near-capacity packages (>=2.2 kg) force one-per-sortie batching",
    stations=(
        _s(1, 400.0, 100.0, 2.3), _s(2, -500.0, 200.0, 2.2),
        _s(3, 200.0, -600.0, 2.4), _s(4, -300.0, -400.0, 2.2),
        _s(5, 700.0, -300.0, 2.3), _s(6, 150.0, 250.0, 0.5),
        _s(7, -200.0, 350.0, 0.8), _s(8, 350.0, 350.0, 0.6),
        _s(9, -450.0, -150.0, 0.7), _s(10, 100.0, -250.0, 0.4),
        _s(11, 550.0, 200.0, 0.9), _s(12, -100.0, 600.0, 0.6),
    ))

CASES = {c.name: c for c in (CASE_FARM_DENSE, CASE_FARM_SPARSE, CASE_FARM_HEAVY)}
CASE_NAMES = tuple(CASES)


def get_case(name: str) -> Case:
    if name not in CASES:
        raise KeyError("unknown case %r, expected one of %s" % (name, ", ".join(CASE_NAMES)))
    return CASES[name]
