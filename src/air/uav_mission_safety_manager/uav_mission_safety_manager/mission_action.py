"""Timer-driven Action bridge. All controller access stays on the ROS executor."""
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from uav_mission_interfaces.action import ExecuteMission

from .mission_queue import MissionQueue
from .state_machine import State


class MissionAction:
    def __init__(self, node):
        self.node = node
        self.busy = False
        self.handle = self.finished = None
        self.server = ActionServer(
            node, ExecuteMission, '~/execute_mission', self.execute,
            goal_callback=self.goal, cancel_callback=self.cancel,
            handle_accepted_callback=self.accepted,
            # An awaiting execute callback must not hold the timer's group.
            callback_group=ReentrantCallbackGroup())

    @staticmethod
    def waypoints(request):
        if len(request.waypoints) != len(request.speeds):
            raise ValueError('Provide exactly one speed per waypoint')
        return [dict(x=p.x, y=p.y, z=p.z, speed=s)
                for p, s in zip(request.waypoints, request.speeds)]

    def goal(self, request):
        c = self.node.controller
        if (self.busy or c is None or c.machine.state != State.EXECUTING or
                c.home is None or c.target is not None or c.mission_queue.active or
                c.pending is not None and not c.pending[1].done()):
            return GoalResponse.REJECT
        try:
            MissionQueue().load(self.waypoints(request), c.home[2], c.machine.max_altitude)
        except (ValueError, TypeError, OverflowError):
            return GoalResponse.REJECT
        # Reserve before accepting so simultaneous goals cannot replace a queue.
        self.busy = True
        return GoalResponse.ACCEPT

    def accepted(self, handle):
        self.handle, self.finished = handle, Future()
        try:
            self.node.controller.request('mission', waypoints=self.waypoints(handle.request))
        except (ValueError, RuntimeError, KeyError, TypeError) as exc:
            self.finished.set_result(('abort', ExecuteMission.Result(
                queue_state='ABORTED', message=str(exc))))
        handle.execute()

    def cancel(self, handle):
        c = self.node.controller
        if handle is self.handle and c.mission_queue.active:
            return CancelResponse.ACCEPT
        return CancelResponse.REJECT

    async def execute(self, handle):
        outcome, result = await self.finished
        if outcome == 'succeed':
            handle.succeed()
        elif outcome == 'cancel':
            handle.canceled()
        else:
            handle.abort()
        self.handle = self.finished = None
        self.busy = False
        return result

    def tick(self):
        if self.handle is None or self.finished.done():
            return
        c, handle = self.node.controller, self.handle
        if handle.is_cancel_requested and c.mission_queue.active:
            try:
                c.request('cancel')
            except ValueError:
                # A dispatch can still be pending. Retry on the next timer tick.
                if c.pending is None:
                    raise
        status = c.mission_queue.status()
        handle.publish_feedback(ExecuteMission.Feedback(
            flight_state=c.machine.state.value, queue_state=status['state'],
            total=status['total'], completed=status['completed'],
            current_index=-1 if status['current_index'] is None else status['current_index'],
            remaining=status['remaining']))
        state = status['state']
        if state in ('COMPLETED', 'CANCELED', 'ABORTED'):
            outcome = ('succeed' if state == 'COMPLETED' else
                       'cancel' if state == 'CANCELED' and handle.is_cancel_requested else 'abort')
            message = ('Waypoint mission completed' if outcome == 'succeed' else
                       'Canceled by action client' if outcome == 'cancel' else
                       c.machine.reason or 'Mission interrupted by external control')
            self.finished.set_result((outcome, ExecuteMission.Result(
                queue_state=state, total=status['total'],
                completed=status['completed'], message=message)))
