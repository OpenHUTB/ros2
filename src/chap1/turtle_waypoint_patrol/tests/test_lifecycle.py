"""Offline lifecycle regression with ROS test doubles; NOT a ROS integration test."""
import csv
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import textwrap
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
# Child subprocesses get these explicit stand-ins, never installed into ROS.
STUB = '''
import json, os, sys, threading, time, types
class Twist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0.0)
        self.angular = types.SimpleNamespace(z=0.0)
class Response:
    def __init__(self, success, message):
        self.success, self.message = success, message
for pkg, kind, values in [
    ('geometry_msgs','msg', {'Twist':Twist}),
    ('std_msgs','msg', {'String':str}),
    ('std_srvs','srv', {'Trigger':object,'TriggerResponse':Response}),
    ('turtlesim','msg', {'Pose':object})]:
    sys.modules[pkg] = types.ModuleType(pkg)
    mod = types.ModuleType(pkg+'.'+kind)
    mod.__dict__.update(values)
    sys.modules[mod.__name__] = mod
params=json.loads(os.environ.get('FAKE_PARAMS','{}'))
shutdown=False
subs=[]
class Publisher:
    def __init__(self, topic, *a, **kw): self.topic=topic
    def publish(self, value):
        if isinstance(value, Twist):
            value={'v':value.linear.x,'w':value.angular.z}
        with open(os.environ['FAKE_LOG'],'a') as f:
            f.write(json.dumps([self.topic,value])+'\\n')
class Subscriber:
    def __init__(self, topic, cls, callback, **kw):
        self.callback=callback; self.live=True; subs.append(self)
        def feed():
            while self.live:
                callback(types.SimpleNamespace(x=5.5,y=5.5,theta=0))
                time.sleep(.01)
        self.thread=threading.Thread(target=feed, daemon=True); self.thread.start()
    def unregister(self):
        self.live=False; self.thread.join(1)
def Service(name, cls, callback):
    if name.endswith('pause') and os.environ.get('FAKE_PAUSE'):
        callback(None)
def init_node(*a, **kw):
    assert kw.get('disable_signals') is True
    time.sleep(float(os.environ.get('FAKE_INIT_DELAY','0')))
def signal_shutdown(reason):
    global shutdown
    shutdown=True
    for sub in subs: assert not sub.thread.is_alive(), 'subscriber still running'
def get_param(name, default=None): return params.get(name,default)
def is_shutdown(): return shutdown
'''


class Lifecycle(unittest.TestCase):
    def child(self, sig, completed=False, paused=False, during_init=False):
        with tempfile.TemporaryDirectory() as directory:
            d = Path(directory)
            (d/'rospy.py').write_text(STUB)
            log, trace = d/'published.jsonl', d/'trace.csv'
            env = dict(os.environ, PYTHONPATH=str(d), FAKE_LOG=str(log),
                       FAKE_PARAMS=json.dumps({'~trace_path':str(trace), '~waypoints':
                           [[5.5,5.5]] if completed else [[9,9]]}))
            if paused: env['FAKE_PAUSE']='1'
            if during_init: env['FAKE_INIT_DELAY']='0.3'
            p = subprocess.Popen([sys.executable,str(ROOT/'main.py')], env=env,
                                 stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
            try:
                deadline=time.monotonic()+3
                # During initialization wait for START; otherwise wait for status.
                if during_init:
                    self.assertIn('START pid=',p.stderr.readline())
                else:
                    wanted='COMPLETED' if completed else ('PAUSED' if paused else 'RUNNING')
                    while time.monotonic()<deadline:
                        if log.exists() and wanted in log.read_text(): break
                        if p.poll() is not None: self.fail(p.communicate()[1])
                        time.sleep(.01)
                    else: self.fail('no status')
                t=time.monotonic(); p.send_signal(sig)
                out,err=p.communicate(timeout=3)
                self.assertEqual(p.returncode,0,err)
                self.assertLess(time.monotonic()-t,2)
                self.assertIn('EXIT ros_shutdown_done',err)
                self.assertIn('EXIT trace_closed',err)
                messages=[json.loads(row) for row in log.read_text().splitlines()]
                cmds=[m[1] for m in messages if m[0]=='/turtle1/cmd_vel']
                self.assertEqual(cmds[-1],{'v':0.0,'w':0.0})
                with trace.open() as f:
                    rows=list(csv.DictReader(f))
                self.assertTrue(rows)
                return err
            finally:
                if p.poll() is None: p.kill(); p.communicate()

    def test_sigterm_running(self): self.child(signal.SIGTERM)
    def test_sigint_running(self): self.child(signal.SIGINT)
    def test_sigterm_paused(self): self.child(signal.SIGTERM,paused=True)
    def test_sigterm_completed(self): self.child(signal.SIGTERM,completed=True)
    def test_signal_during_init(self): self.child(signal.SIGTERM,during_init=True)

    def test_invalid_goal_creates_no_trace(self):
        with tempfile.TemporaryDirectory() as directory:
            d=Path(directory); (d/'rospy.py').write_text(STUB)
            trace=d/'trace.csv'
            env=dict(os.environ,PYTHONPATH=str(d),FAKE_LOG=str(d/'log'),
                     FAKE_PARAMS=json.dumps({'~trace_path':str(trace),'~waypoints':[[100,2]]}))
            result=subprocess.run([sys.executable,str(ROOT/'main.py')],env=env,
                                  capture_output=True,text=True,timeout=3)
            self.assertNotEqual(result.returncode,0)
            self.assertIn('ValueError',result.stderr)
            self.assertFalse(trace.exists())

    def test_diagnostic_signal_preserves_process(self):
        # SIGUSR1 stack collection is exercised on a live Python worker.
        with tempfile.TemporaryDirectory() as directory:
            d=Path(directory); (d/'rospy.py').write_text(STUB)
            env=dict(os.environ,PYTHONPATH=str(d),FAKE_LOG=str(d/'log'))
            p=subprocess.Popen([sys.executable,str(ROOT/'main.py')],env=env,
                               stderr=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
            try:
                deadline=time.monotonic()+3
                while not (d/'log').exists() and time.monotonic()<deadline: time.sleep(.01)
                p.send_signal(signal.SIGUSR1); time.sleep(.05)
                self.assertIsNone(p.poll())
                p.send_signal(signal.SIGTERM)
                _,err=p.communicate(timeout=3)
                self.assertIn('Current thread',err)
                self.assertIn('main.py',err)
                self.assertEqual(p.returncode,0,err)
            finally:
                if p.poll() is None: p.kill(); p.communicate()


if __name__=='__main__': unittest.main(verbosity=2)
