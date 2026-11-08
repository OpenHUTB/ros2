"""AirSim circular flight for winter. No RPC is issued without --fly."""
import argparse
import csv
from pathlib import Path
import math
import time


def enu_to_ned(east, north, up):
    return north, east, -up


def check_collision(info, baseline_stamp):
    """AirSim can retain the last hit: only stop for a newer event."""
    if info.has_collided and info.time_stamp > baseline_stamp:
        raise RuntimeError('New collision: object=%s timestamp=%s (baseline=%s)' %
                           (info.object_name, info.time_stamp, baseline_stamp))


def command(x, y, cx, cy, radius, omega, gain, max_speed):
    dx, dy = x - cx, y - cy
    distance = math.hypot(dx, dy)
    if distance < 0.05:
        raise RuntimeError('Too close to circle center')
    ux, uy = dx / distance, dy / distance
    correction = gain * (radius - distance)
    vx = correction * ux - omega * radius * uy
    vy = correction * uy + omega * radius * ux
    scale = min(1.0, max_speed / max(math.hypot(vx, vy), 1e-9))
    return vx * scale, vy * scale


def checked(future, label):
    """get(), unlike join(), propagates msgpack-rpc errors and returns the RPC result."""
    if future.get() is not True:
        raise RuntimeError(label + ' rejected by AirSim')


def wait_deadline(deadline, period=0.1, clock=time.monotonic, sleep=time.sleep):
    """Maintain deadlines; skip missed ticks instead of sending a catch-up burst."""
    deadline += period
    now = clock()
    if deadline < now:
        return now  # Work already consumed this tick; rebase without an extra sleep.
    sleep(max(0.0, deadline - now))
    return deadline


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host', default='127.0.0.1')
    p.add_argument('--port', type=int, default=41451)
    p.add_argument('--vehicle', default='Drone1')
    p.add_argument('--radius', type=float, default=2.0)
    p.add_argument('--height', type=float, default=3.0,
                   help='meters above initial vehicle position, not terrain clearance')
    p.add_argument('--speed', type=float, default=0.5)
    p.add_argument('--laps', type=float, default=1.0)
    p.add_argument('--clockwise', action='store_true', help='viewed from above')
    p.add_argument('--land', action='store_true', help='land after successful completion')
    p.add_argument('--fly', action='store_true')
    p.add_argument('--log', default='winter_circle_trajectory.csv',
                   help='CSV path, created only for --fly; existing files are not overwritten')
    a = p.parse_args()
    if not all(math.isfinite(v) and v > 0 for v in
               (a.radius, a.height, a.speed, a.laps)):
        p.error('radius, height, speed and laps must be finite and positive')
    print('winter / AirSim NED: x=north, y=east, z=down')
    print('radius=%.2fm height=%.2fm speed=%.2fm/s laps=%.2f' %
          (a.radius, a.height, a.speed, a.laps))
    print('Center is radius meters north of initial position; start is on circle edge.')
    if not a.fly:
        print('Preview only. Add --fly after starting winter Play and checking clear space.')
        return
    import airsim
    client = airsim.MultirotorClient(ip=a.host, port=a.port, timeout_value=10)
    vehicle = a.vehicle
    try:
        client.confirmConnection()
    except Exception as exc:
        raise RuntimeError(
            'Cannot connect to AirSim at %s:%s. Start winter in UE4.27, click Play, '
            'and check RpcEnabled, port and firewall. (%s)' %
            (a.host, a.port, exc)) from None
    initial = client.getMultirotorState(vehicle_name=vehicle)
    collision_before = client.simGetCollisionInfo(vehicle_name=vehicle)
    collision_baseline = max(initial.timestamp, collision_before.time_stamp)
    if collision_before.has_collided:
        print('Previous collision record: object=%s timestamp=%s; monitoring new events' %
              (collision_before.object_name, collision_before.time_stamp))
    if initial.landed_state != airsim.LandedState.Landed:
        raise RuntimeError('Start this script with vehicle landed; stop other controllers first')
    pos = initial.kinematics_estimated.position
    cx, cy, target_z = pos.x_val + a.radius, pos.y_val, pos.z_val - a.height
    signed_omega = (1 if a.clockwise else -1) * a.speed / a.radius
    log_path = Path(a.log).expanduser()
    log_path.parent.mkdir(parents=True, exist_ok=True)
    # Exclusive creation preserves previous experimental evidence.
    log_file = log_path.open('x', newline='', encoding='utf-8')
    writer = csv.writer(log_file)
    writer.writerow(['elapsed_s', 'loop_interval_s', 'state_timestamp_ns', 'x_ned_m', 'y_ned_m',
                     'z_ned_m', 'center_x_m', 'center_y_m', 'target_z_m',
                     'radius_error_m', 'height_error_m', 'progress_rad',
                     'command_vx_mps', 'command_vy_mps'])
    active = False
    completed = False
    try:
        client.enableApiControl(True, vehicle_name=vehicle)
        active = True
        if not client.armDisarm(True, vehicle_name=vehicle):
            raise RuntimeError('Arming rejected')
        # Startup operations need a longer RPC deadline than individual control commands.
        startup = airsim.MultirotorClient(ip=a.host, port=a.port, timeout_value=40)
        checked(startup.takeoffAsync(timeout_sec=20, vehicle_name=vehicle), 'Takeoff')
        checked(startup.moveToZAsync(target_z, min(a.speed, 0.5), timeout_sec=30,
                                    vehicle_name=vehicle), 'Climb')
        client = airsim.MultirotorClient(ip=a.host, port=a.port, timeout_value=3)
        state = client.getMultirotorState(vehicle_name=vehicle)
        if state.landed_state == airsim.LandedState.Landed or abs(
                state.kinematics_estimated.position.z_val - target_z) > 0.5:
            raise RuntimeError('Takeoff / altitude check failed')
        check_collision(client.simGetCollisionInfo(vehicle_name=vehicle), collision_baseline)
        previous_angle = None
        progress = 0.0
        start = time.monotonic()
        timeout = 3 * (2 * math.pi * a.radius * a.laps / a.speed) + 30
        last = start
        deadline = start
        while progress < 2 * math.pi * a.laps:
            now = time.monotonic()
            if now - start > timeout or now - last > 2:
                raise RuntimeError('Flight timeout / delayed control loop')
            loop_interval = now - last
            last = now
            state = client.getMultirotorState(vehicle_name=vehicle)
            pos = state.kinematics_estimated.position
            check_collision(client.simGetCollisionInfo(vehicle_name=vehicle), collision_baseline)
            if abs(pos.z_val - target_z) > 1.0:
                raise RuntimeError('Altitude tracking error exceeds 1m')
            if math.hypot(pos.x_val - cx, pos.y_val - cy) > 2 * a.radius + 2:
                raise RuntimeError('Outside circle tracking boundary')
            angle = math.atan2(pos.y_val - cy, pos.x_val - cx)
            if previous_angle is not None:
                delta = math.atan2(math.sin(angle - previous_angle),
                                   math.cos(angle - previous_angle))
                progress += delta * (1 if signed_omega > 0 else -1)
            previous_angle = angle
            ramp = min(1.0, (now - start) / 3.0)
            vx, vy = command(pos.x_val, pos.y_val, cx, cy, a.radius,
                             signed_omega * ramp, 0.5, a.speed)
            writer.writerow([now - start, loop_interval, state.timestamp, pos.x_val, pos.y_val,
                             pos.z_val, cx, cy, target_z,
                             math.hypot(pos.x_val - cx, pos.y_val - cy) - a.radius,
                             pos.z_val - target_z, progress, vx, vy])
            log_file.flush()
            if progress >= 2 * math.pi * a.laps:
                break
            # Short finite segments: check every result before issuing another command.
            # Each segment holds z in AirSim. No outstanding futures accumulate.
            checked(client.moveByVelocityZAsync(vx, vy, target_z, 0.1,
                drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=airsim.YawMode(False, 0), vehicle_name=vehicle), 'Velocity command')
            deadline = wait_deadline(deadline)
        completed = True
        label = 'Partial arc complete' if a.laps < 1 else 'Circle complete'
        print('%s: %.2f/%.2f laps' % (label, progress / (2 * math.pi), a.laps))
    finally:
        log_file.close()
        print('Trajectory CSV:', log_path.resolve())
        if active:
            try:
                # hoverAsync has no timeout_sec argument. Bound its RPC future using
                # a separate short-timeout client, including after a failed startup.
                cleanup = airsim.MultirotorClient(ip=a.host, port=a.port, timeout_value=3)
                checked(cleanup.hoverAsync(vehicle_name=vehicle), 'Hover')
                if completed and a.land:
                    landing = airsim.MultirotorClient(ip=a.host, port=a.port, timeout_value=45)
                    checked(landing.landAsync(timeout_sec=40, vehicle_name=vehicle), 'Landing')
                    if client.getMultirotorState(vehicle_name=vehicle).landed_state == airsim.LandedState.Landed:
                        client.armDisarm(False, vehicle_name=vehicle)
                        client.enableApiControl(False, vehicle_name=vehicle)
            except Exception as exc:
                print('Could not confirm hover/landing:', exc)
                if completed:
                    raise RuntimeError('Requested arc finished, but hover/landing was not confirmed') from exc
        # Keep API control while airborne so hover is not replaced by another mode.


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit('Interrupted; hover was attempted if API control was active.')
    except Exception as exc:
        raise SystemExit('Flight failed: %s' % exc)
