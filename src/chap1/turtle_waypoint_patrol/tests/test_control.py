"""Deterministic tests of the production control step, with message-only stubs."""
import importlib.util
import math
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]

class Twist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0.)
        self.angular = types.SimpleNamespace(z=0.)

def load_controller():
    modules = {'rospy': types.ModuleType('rospy')}
    for pkg, sub, symbols in (
        ('geometry_msgs','msg',{'Twist':Twist}),
        ('std_msgs','msg',{'String':str}),
        ('std_srvs','srv',{'Trigger':object,'TriggerResponse':object}),
        ('turtlesim','msg',{'Pose':object})):
        modules[pkg] = types.ModuleType(pkg)
        modules[pkg+'.'+sub] = types.ModuleType(pkg+'.'+sub)
        modules[pkg+'.'+sub].__dict__.update(symbols)
    spec = importlib.util.spec_from_file_location('patrol_control_under_test',ROOT/'main.py')
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules,modules): spec.loader.exec_module(module)
    return module

MODULE=load_controller()

class Control(unittest.TestCase):
    def node(self,goal=(1.,0.),theta=0.,state='RUNNING'):
        p=MODULE.Patrol.__new__(MODULE.Patrol)
        p.pose=types.SimpleNamespace(x=0.,y=0.,theta=theta)
        p.goals=[list(goal)];p.index=0;p.state=state;p.started=0.;p.received=1.
        return p

    def assert_zero(self,cmd):
        self.assertEqual((cmd.linear.x,cmd.angular.z),(0.,0.))

    def test_exact_arrival_threshold_completes(self):
        p=self.node((.08,0.));cmd=p.control_step(1.)
        self.assertEqual((p.state,p.index),('COMPLETED',1));self.assert_zero(cmd)

    def test_just_outside_threshold_keeps_moving(self):
        p=self.node((.080001,0.));cmd=p.control_step(1.)
        self.assertEqual((p.state,p.index),('RUNNING',0));self.assertGreater(cmd.linear.x,0.)

    def test_arrival_advances_one_waypoint(self):
        p=self.node((.08,0.));p.goals.append([1.,0.]);cmd=p.control_step(1.)
        self.assertEqual((p.state,p.index),('RUNNING',1));self.assert_zero(cmd)

    def test_heading_proportional_gain(self):
        p=self.node((2*math.cos(.2),2*math.sin(.2)));cmd=p.control_step(1.)
        self.assertAlmostEqual(cmd.angular.z,.8)
        self.assertAlmostEqual(cmd.linear.x,1.5*math.cos(.2))

    def test_heading_both_signs_saturate_and_stop_translation(self):
        for y,expected in ((1.,2.),(-1.,-2.)):
            with self.subTest(y=y):
                cmd=self.node((0.,y)).control_step(1.)
                self.assertEqual(cmd.angular.z,expected);self.assertEqual(cmd.linear.x,0.)

    def test_heading_wraps_across_pi(self):
        cmd=self.node((-1.,0.),theta=-math.pi+.1).control_step(1.)
        self.assertAlmostEqual(cmd.angular.z,-.4)
        self.assertGreater(cmd.linear.x,0.)

    def test_turn_in_place_boundary(self):
        for angle,moving in ((.599999,True),(.600001,False)):
            with self.subTest(angle=angle):
                cmd=self.node((math.cos(angle),math.sin(angle))).control_step(1.)
                self.assertEqual(cmd.linear.x>0.,moving)

    def test_linear_velocity_limit_and_distance_scaling(self):
        self.assertAlmostEqual(self.node((.5,0.)).control_step(1.).linear.x,.5)
        self.assertAlmostEqual(self.node((5.,0.)).control_step(1.).linear.x,1.5)

    def test_pose_timeout_boundary(self):
        p=self.node();cmd=p.control_step(1.5)
        self.assertEqual(p.state,'RUNNING');self.assertGreater(cmd.linear.x,0.)
        cmd=p.control_step(1.500001)
        self.assertEqual(p.state,'POSE_TIMEOUT');self.assert_zero(cmd)

    def test_initial_pose_timeout_boundary(self):
        p=self.node();p.pose=None;p.received=None
        self.assert_zero(p.control_step(3.));self.assertEqual(p.state,'RUNNING')
        self.assert_zero(p.control_step(3.000001));self.assertEqual(p.state,'POSE_TIMEOUT')

    def test_task_timeout_boundary_with_fresh_pose(self):
        p=self.node();p.received=120.
        self.assertGreater(p.control_step(120.).linear.x,0.);self.assertEqual(p.state,'RUNNING')
        self.assert_zero(p.control_step(120.000001));self.assertEqual(p.state,'TASK_TIMEOUT')

    def test_task_timeout_includes_paused_duration(self):
        p=self.node(state='PAUSED');p.received=120.
        self.assert_zero(p.control_step(120.000001));self.assertEqual(p.state,'TASK_TIMEOUT')

    def test_paused_and_terminal_states_do_not_navigate(self):
        for state in ('PAUSED','CANCELED','COMPLETED','POSE_TIMEOUT','TASK_TIMEOUT'):
            with self.subTest(state=state):
                p=self.node((.01,0.),state=state)
                self.assert_zero(p.control_step(1.));self.assertEqual(p.index,0)
                self.assertEqual(p.state,state)

if __name__=='__main__': unittest.main(verbosity=2)
