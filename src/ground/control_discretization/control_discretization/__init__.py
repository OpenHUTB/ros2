"""连续控制器离散化方法对比（ROS 2 Humble 示例）。"""

from .systems import (METHOD_LABELS, METHODS, SecondOrderSystem,
                      critical_period, discretize, discrete_poles, is_stable,
                      simulate_discrete)

__all__ = ['METHOD_LABELS', 'METHODS', 'SecondOrderSystem', 'critical_period',
           'discrete_poles', 'discretize', 'is_stable', 'simulate_discrete']
