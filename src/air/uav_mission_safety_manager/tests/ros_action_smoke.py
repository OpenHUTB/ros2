"""Real Humble ActionClient/server I/O on a single executor; demo motion only."""
import json
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from std_srvs.srv import Trigger
from uav_mission_interfaces.action import ExecuteMission
from uav_mission_safety_manager.main import MissionNode


def main():
    rclpy.init(args=['--ros-args', '-p', 'use_demo_adapter:=true'])
    server = MissionNode()
    client = rclpy.create_node('mission_action_smoke')
    executor = SingleThreadedExecutor()
    executor.add_node(server)
    executor.add_node(client)
    action = ActionClient(client, ExecuteMission, '/uav_mission_safety_manager/execute_mission')
    publisher = client.create_publisher(String, '/uav_mission_safety_manager/command', 10)
    feedback = []
    c = server.controller

    def wait(predicate, timeout=15.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
            if predicate():
                return
        raise AssertionError('Timeout: ' + str(c.mission_queue.status()))

    def resolve(future):
        wait(future.done)
        return future.result()

    def goal(xs=(4., 8., 12.), speed=4.):
        return ExecuteMission.Goal(
            waypoints=[Point(x=x, y=0., z=-2.) for x in xs],
            speeds=[speed] * len(xs))

    def send(request):
        return resolve(action.send_goal_async(
            request, feedback_callback=lambda msg: feedback.append(msg.feedback)))

    def service(name):
        srv = client.create_client(Trigger, '/uav_mission_safety_manager/' + name)
        try:
            assert srv.wait_for_service(timeout_sec=3.)
            response = resolve(srv.call_async(Trigger.Request()))
            assert response.success, response.message
        finally:
            client.destroy_client(srv)

    def command(event, **data):
        publisher.publish(String(data=json.dumps(dict(event=event, **data))))

    def idle_dispatch():
        wait(lambda: c.pending is None)

    def record(check, result=None):
        data = {'check': check, 'queue': c.mission_queue.status()}
        if result:
            data.update(action_status=result.status, message=result.result.message)
        print(json.dumps(data), flush=True)

    try:
        assert action.wait_for_server(timeout_sec=5.)
        wait(lambda: publisher.get_subscription_count() > 0)
        assert not send(goal()).accepted
        record('reject_before_takeoff')
        for event, state in [('connect', 'ARMING'), ('armed', 'TAKEOFF'), ('airborne', 'EXECUTING')]:
            command(event)
            wait(lambda: c.machine.state.value == state and c.pending is None)
        for invalid in [goal(()), ExecuteMission.Goal(waypoints=[Point(z=-2.)]),
                        goal(speed=0.), goal(speed=float('nan')),
                        ExecuteMission.Goal(waypoints=[Point(z=1.)], speeds=[1.])]:
            assert not send(invalid).accepted
            assert c.mission_queue.state == 'IDLE'
        record('invalid_goals_leave_queue_unchanged')
        handle = send(goal())
        assert handle.accepted
        result_future = handle.get_result_async()
        assert not send(goal()).accepted
        wait(lambda: len(feedback) > 0 and c.pending is None)
        command('mission', waypoints=[dict(x=99., y=0., z=-2.)])
        command('goto', x=99., y=0., z=-2.)
        service('pause')
        wait(lambda: any(f.queue_state == 'PAUSED' for f in feedback) and c.pending is None)
        completed = c.mission_queue.completed
        until = time.monotonic() + 0.6
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=0.02)
            assert c.mission_queue.completed == completed
        assert c.mission_queue.waypoints[0][0][0] == 4.
        record('busy_rejection_and_pause_feedback')
        service('resume')
        result = resolve(result_future)
        assert result.status == GoalStatus.STATUS_SUCCEEDED
        assert (result.result.queue_state, result.result.total, result.result.completed) == ('COMPLETED', 3, 3)
        assert any(f.current_index == -1 and f.remaining == 0 for f in feedback)
        record('three_waypoints_succeeded', result)
        idle_dispatch()
        handle = send(goal((30.,), speed=0.5))
        assert handle.accepted
        idle_dispatch()
        service('pause')
        wait(lambda: c.mission_queue.state == 'PAUSED' and c.pending is None)
        assert resolve(handle.cancel_goal_async()).goals_canceling
        result = resolve(handle.get_result_async())
        assert result.status == GoalStatus.STATUS_CANCELED
        assert result.result.queue_state == 'CANCELED' and c.target is None
        assert c.mission_queue.completed == 0
        record('action_cancel_while_paused', result)
        idle_dispatch()
        service('resume')
        idle_dispatch()
        handle = send(goal((30.,), speed=0.5))
        assert handle.accepted
        idle_dispatch()
        assert resolve(handle.cancel_goal_async()).goals_canceling
        result = resolve(handle.get_result_async())
        assert result.status == GoalStatus.STATUS_CANCELED
        assert c.target is None
        record('action_cancel_while_running', result)
        idle_dispatch()
        command('mission', waypoints=[dict(x=30., y=0., z=-2., speed=0.5)])
        wait(lambda: c.mission_queue.active and c.pending is None)
        assert not send(goal()).accepted
        service('cancel')
        idle_dispatch()
        record('legacy_queue_rejects_action')
        # An external cancellation has no Action cancel request: abort the goal.
        handle = send(goal((30.,), speed=0.5))
        assert handle.accepted
        idle_dispatch()
        service('cancel')
        result = resolve(handle.get_result_async())
        assert result.status == GoalStatus.STATUS_ABORTED
        assert result.result.queue_state == 'CANCELED'
        record('external_cancel_aborts_action', result)
        idle_dispatch()
        handle = send(goal((30.,), speed=0.5))
        assert handle.accepted
        # Inject an operation deadline, not an AirSim/network failure.
        c.operation_timeout = 0.001
        result = resolve(handle.get_result_async())
        assert result.status == GoalStatus.STATUS_ABORTED
        assert result.result.queue_state == 'ABORTED'
        assert 'timed out' in result.result.message
        record('injected_timeout_aborts_action', result)
        c.operation_timeout = 120.
        idle_dispatch()
        service('land')
        wait(lambda: c.machine.state.value == 'LANDED' and c.pending is None)
        command('reset')
        wait(lambda: c.machine.state.value == 'IDLE')
        for event, state in [('connect', 'ARMING'), ('armed', 'TAKEOFF'), ('airborne', 'EXECUTING')]:
            command(event)
            wait(lambda: c.machine.state.value == state and c.pending is None)
        handle = send(goal((30.,), speed=0.5))
        assert handle.accepted
        idle_dispatch()
        service('return_home')
        result = resolve(handle.get_result_async())
        assert result.status == GoalStatus.STATUS_ABORTED
        assert result.result.queue_state == 'CANCELED'
        record('return_home_aborts_action', result)
        wait(lambda: c.machine.state.value == 'LANDED', timeout=25.)
        assert c.mission_queue.state == 'CANCELED'
        record('return_finishes_landed_without_replaying_queue')
        print('PASS: typed Action goals, rejection, feedback, pause/resume, success, '
              'cancel, external interruption, injected timeout, return/land', flush=True)
    finally:
        action.destroy()
        server.mission_action.server.destroy()
        server.adapter.close()
        executor.shutdown()
        server.destroy_node()
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
