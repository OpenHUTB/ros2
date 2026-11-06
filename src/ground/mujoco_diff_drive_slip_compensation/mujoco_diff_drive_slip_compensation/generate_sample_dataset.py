"""Generate reproducible MuJoCo sample episodes for training and evaluation."""

import argparse
import importlib.metadata
from pathlib import Path
import platform

from .command_profile import ExcitationProfile
from .dataset import DatasetSample, EpisodeCsvWriter, assign_episode_split
from .kinematics import DriveGeometry
from .sensors import SensorNoise, SensorNoiseConfig
from .simulator import MujocoSimulation


def _default_model_path() -> Path:
    """Resolve the installed ROS share model or the source-tree model."""
    try:
        from ament_index_python.packages import get_package_share_directory

        return (
            Path(get_package_share_directory('mujoco_diff_drive_slip_compensation'))
            / 'models'
            / 'diff_drive.xml'
        )
    except ImportError:
        return Path(__file__).parents[1] / 'models' / 'diff_drive.xml'


def generate_dataset(
    output_directory: Path,
    model_path: Path,
    seed: int = 25012,
    episodes_per_split: int = 4,
    duration_seconds: float = 12.0,
    sample_every_steps: int = 10,
) -> list[str]:
    """Create complete episodes with split, friction, and profile isolation."""
    if (episodes_per_split <= 0 or duration_seconds <= 0
            or sample_every_steps <= 0):
        raise ValueError(
            'episode count, duration, and sample interval must be positive'
        )

    simulation = MujocoSimulation(model_path)
    geometry = DriveGeometry(0.08, 0.42, 20.0)
    profile = ExcitationProfile(period_seconds=4.0)
    friction_values = (0.02, 0.08, 0.4, 0.9)
    selected: dict[str, list[str]] = {
        name: [] for name in ('train', 'validation', 'test')
    }
    candidate = 0
    while any(len(ids) < episodes_per_split for ids in selected.values()):
        episode_id = f'demo_{candidate:06d}'
        split = assign_episode_split(episode_id, seed)
        if len(selected[split]) < episodes_per_split:
            selected[split].append(episode_id)
        candidate += 1

    output_directory = Path(output_directory)
    all_ids = [episode_id for ids in selected.values() for episode_id in ids]
    for index, episode_id in enumerate(all_ids):
        split = assign_episode_split(episode_id, seed)
        split_index = selected[split].index(episode_id)
        friction = friction_values[split_index % len(friction_values)]
        simulation.set_ground_friction(friction)
        simulation.reset()
        noise = SensorNoise(
            SensorNoiseConfig(),
            seed + index,
        )
        metadata = {
            'split_seed': seed,
            'random_seed': seed + index,
            'python_version': platform.python_version(),
            'mujoco_version': importlib.metadata.version('mujoco'),
            'package_version': '0.1.0',
            'generator': 'mujoco_diff_drive_slip_compensation.generate_sample_dataset',
            'profile_phase_seconds': float(split_index % 4),
        }
        with EpisodeCsvWriter(
            output_directory,
            episode_id,
            seed,
            metadata=metadata,
            flush_interval=256,
        ) as writer:
            total_steps = int(duration_seconds / simulation.timestep)
            phase = float(split_index % 4)
            for step in range(total_steps):
                elapsed = step * simulation.timestep
                command = profile.command_at(elapsed + phase)
                left, right = geometry.wheel_speeds(
                    command.linear, command.angular
                )
                simulation.apply_wheel_speeds(left, right)
                simulation.step()
                if step % sample_every_steps:
                    continue
                positions, velocities = simulation.wheel_state()
                acceleration, angular_velocity = simulation.imu_state()
                positions, velocities = noise.encoders(positions, velocities)
                acceleration, angular_velocity = noise.imu(
                    acceleration, angular_velocity
                )
                state = simulation.planar_state()
                writer.write(DatasetSample(
                    timestamp_ns=int(round(state.time * 1_000_000_000)),
                    episode_id=episode_id,
                    split=split,
                    friction_coefficient=friction,
                    command_linear_x=command.linear,
                    command_angular_z=command.angular,
                    wheel_position_left_front=positions[0],
                    wheel_position_left_rear=positions[1],
                    wheel_position_right_front=positions[2],
                    wheel_position_right_rear=positions[3],
                    wheel_velocity_left_front=velocities[0],
                    wheel_velocity_left_rear=velocities[1],
                    wheel_velocity_right_front=velocities[2],
                    wheel_velocity_right_rear=velocities[3],
                    imu_acceleration_x=acceleration[0],
                    imu_acceleration_y=acceleration[1],
                    imu_acceleration_z=acceleration[2],
                    imu_angular_velocity_x=angular_velocity[0],
                    imu_angular_velocity_y=angular_velocity[1],
                    imu_angular_velocity_z=angular_velocity[2],
                    ground_truth_x=state.x,
                    ground_truth_y=state.y,
                    ground_truth_yaw=state.yaw,
                    ground_truth_linear_x=state.linear_velocity,
                    ground_truth_angular_z=state.angular_velocity,
                ))
    return all_ids


def main(args: list[str] | None = None) -> None:
    """Generate compact sample episodes without a ROS installation."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('data/demo'))
    parser.add_argument(
        '--model',
        type=Path,
        default=None,
    )
    parser.add_argument('--seed', type=int, default=25012)
    parser.add_argument('--episodes-per-split', type=int, default=4)
    parser.add_argument('--duration', type=float, default=12.0)
    options = parser.parse_args(args)
    episode_ids = generate_dataset(
        options.output,
        options.model or _default_model_path(),
        seed=options.seed,
        episodes_per_split=options.episodes_per_split,
        duration_seconds=options.duration,
    )
    print(f'generated {len(episode_ids)} episodes in {options.output}')


if __name__ == '__main__':
    main()
