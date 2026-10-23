"""Opt-in simulator flight test. Never run against a real vehicle."""
import argparse
import json
import math
import time

from uav_mission_safety_manager.airsim_adapter import AirSimAdapter
from uav_mission_safety_manager.controller import FlightController
from uav_mission_safety_manager.state_machine import State


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True)
    parser.add_argument('--log', required=True)
    parser.add_argument('--fly', action='store_true', help='authorize simulated flight')
    parser.add_argument('--reset', action='store_true', help='reset simulator vehicle first')
    args = parser.parse_args()
    if not args.fly:
        parser.error('--fly is required; this test moves the simulated vehicle')
    import airsim
    probe = airsim.MultirotorClient(ip=args.host, timeout_value=5)
    if args.reset:
        probe.reset()
    if probe.simIsPause():
        raise RuntimeError('Simulator is paused')
    # reset() may briefly report Landed while the spawned vehicle is falling.
    # Capture home only after continuous stable, landed telemetry.
    deadline, settled, previous = time.monotonic()+30, None, None
    while time.monotonic() < deadline:
        state = probe.getMultirotorState()
        p = state.kinematics_estimated.position
        v = state.kinematics_estimated.linear_velocity
        position = (p.x_val, p.y_val, p.z_val)
        stable = (state.landed_state == airsim.LandedState.Landed and
                  math.sqrt(v.x_val**2+v.y_val**2+v.z_val**2) <= 0.1 and
                  previous is not None and math.dist(position, previous) <= 0.02)
        settled = (settled if settled is not None else time.monotonic()) if stable else None
        if settled is not None and time.monotonic()-settled >= 3:
            break
        previous = position
        time.sleep(0.2)
    else:
        raise RuntimeError('Preflight requires 3s of stable landed telemetry; no flight dispatched')
    adapter = AirSimAdapter(args.host)
    controller = FlightController(adapter)
    with open(args.log, 'w', encoding='utf-8') as log:
        def record(event, **extra):
            row = dict(time=time.monotonic(), event=event,
                       state=controller.machine.state.value,
                       home=controller.home, position=controller.position,
                       target=controller.target, target_speed=controller.target_speed,
                       takeoff_rpc_result=controller.takeoff_rpc_result,
                       last_error=controller.last_error, **extra)
            log.write(json.dumps(row) + '\n')
            log.flush()
            if event != 'sample':
                print(json.dumps(row), flush=True)

        def wait_until(predicate, timeout=120, allow_failsafe=False):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                controller.tick()
                record('sample')
                if controller.machine.state == State.FAILSAFE and not allow_failsafe:
                    raise RuntimeError(controller.machine.reason)
                if predicate():
                    return
                time.sleep(0.2)
            raise TimeoutError('Test stage timed out')

        def request(event, **data):
            wait_until(lambda: controller.pending is None, 5, allow_failsafe=True)
            controller.request(event, **data)
            record(event, request=data)
            wait_until(lambda: controller.pending is None, 5)

        try:
            request('connect')
            request('armed')
            request('airborne')
            wait_until(lambda: controller.machine.state == State.EXECUTING)
            # Avoid cancelling takeoff with the first waypoint: its actual
            # return value must be measured independently of replacement.
            wait_until(lambda: controller.takeoff_rpc_result is not None, 25)
            x, y, z = controller.home
            request('goto', x=x, y=y, z=z-5, speed=1.0)
            wait_until(lambda: controller.target is None)
            request('goto', x=x+3, y=y, z=z-5, speed=0.5)
            wait_until(lambda: controller.position[0]-x >= 0.5, 20)
            original_target = controller.target
            if original_target is None:
                raise AssertionError('Waypoint already completed before pause')
            request('pause')
            paused_at = time.monotonic()
            wait_until(lambda: time.monotonic()-paused_at >= 3, 10)
            state = probe.getMultirotorState()
            velocity = state.kinematics_estimated.linear_velocity
            speed = math.sqrt(sum(v*v for v in
                                  (velocity.x_val, velocity.y_val, velocity.z_val)))
            assert controller.machine.state == State.PAUSED
            assert controller.target == original_target
            assert speed <= 0.1, 'Pause did not stop vehicle'
            record('pause_verified', measured_speed=speed)
            request('resume')
            assert controller.target_speed == 0.5
            assert controller.target == original_target
            record('resume_speed_verified', dispatched_speed=controller.target_speed)
            wait_until(lambda: controller.target is None)
            request('return')
            wait_until(lambda: controller.machine.state == State.LANDED)
            state = probe.getMultirotorState()
            p = state.kinematics_estimated.position
            final_error = math.dist((p.x_val, p.y_val, p.z_val), controller.home)
            assert state.landed_state == airsim.LandedState.Landed
            assert final_error <= 0.15, 'Final position exceeded home tolerance'
            record('PASS', final_home_error=final_error)
        except Exception as exc:
            record('FAIL', error=str(exc))
            # Attempt a normal land on failure, never force-disarm in the air.
            try:
                request('land')
                wait_until(lambda: controller.machine.state == State.LANDED)
                record('recovery_landed')
            except Exception as recovery:
                record('recovery_unconfirmed', error=str(recovery))
            raise
        finally:
            adapter.close()


if __name__ == '__main__':
    main()
