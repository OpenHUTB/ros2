"""Small rospy facade for shared node logic, not a ROS 1/ROS 2 wire bridge.

Subscriber/service callbacks only enqueue work. All application code, including
msgpack-rpc calls and their Tornado event loop, runs on the main thread.
Sensor subscriptions coalesce to their newest message to bound memory usage.
"""
from collections import OrderedDict, deque
import threading
import time
from types import SimpleNamespace

import rospy


class Node:
    def __init__(self, name):
        self.defaults = {}
        self.lock = threading.Lock()
        self.pending = OrderedDict()
        self.services = deque()
        self.handles = []
        self.timers = []

    def declare_parameter(self, name, value):
        self.defaults[name] = value

    def get_parameter(self, name):
        return SimpleNamespace(value=rospy.get_param('~' + name, self.defaults[name]))

    def get_logger(self):
        return SimpleNamespace(info=rospy.loginfo, warning=rospy.logwarn, error=rospy.logerr)

    def create_publisher(self, typ, topic, depth):
        pub = rospy.Publisher(topic, typ, queue_size=depth)
        self.handles.append(pub)
        return pub

    def create_subscription(self, typ, topic, callback, depth):
        def receive(message):
            with self.lock:
                self.pending[topic] = (callback, message, time.monotonic())
        sub = rospy.Subscriber(topic, typ, receive, queue_size=depth, buff_size=16 * 1024 * 1024)
        self.handles.append(sub)
        return sub

    def create_service(self, typ, topic, callback):
        def handle(request):
            job = dict(callback=callback, request=request, response=typ._response_class(),
                       done=threading.Event(), cancelled=False)
            with self.lock:
                if len(self.services) >= 8:
                    raise rospy.ServiceException('Service queue is full')
                self.services.append(job)
            deadline = time.monotonic() + 60
            while not job['done'].wait(0.1):
                if rospy.is_shutdown() or time.monotonic() > deadline:
                    with self.lock:
                        job['cancelled'] = True
                    raise rospy.ServiceException('Service timeout; check vehicle state before retrying')
            if 'error' in job:
                raise rospy.ServiceException(str(job['error']))
            return job['response']
        srv = rospy.Service(topic, typ, handle)
        self.handles.append(srv)
        return srv

    def create_timer(self, period, callback):
        self.timers.append([period, time.monotonic() + period, callback])

    def spin(self):
        while not rospy.is_shutdown():
            with self.lock:
                job = self.services.popleft() if self.services else None
                messages = list(self.pending.values())
                self.pending.clear()
            if job and not job['cancelled']:
                try:
                    job['response'] = job['callback'](job['request'], job['response'])
                except Exception as exc:
                    job['error'] = exc
                finally:
                    job['done'].set()
            for callback, message, received_at in messages:
                # Do not rejuvenate data queued while a long RPC/inference ran.
                if time.monotonic() - received_at < 0.5:
                    callback(message)
            for timer in self.timers:
                if time.monotonic() >= timer[1]:
                    timer[2]()
                    timer[1] = time.monotonic() + timer[0]
            time.sleep(0.01)

    def destroy_node(self):
        for handle in self.handles:
            if hasattr(handle, 'unregister'):
                handle.unregister()
            elif hasattr(handle, 'shutdown'):
                handle.shutdown()
