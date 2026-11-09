"""Execute actual service methods with fake RPC clients; no ROS or flight."""
import ast
from pathlib import Path
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock
import numpy as np

class OperationResults(unittest.TestCase):
    def bridge(self, result, landed):
        path = Path(__file__).parents[1] / 'src/airsim_rl_planner_ros1/bridge_node.py'
        tree = ast.parse(path.read_text())
        definition = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'AirSimBridge')
        operation = Mock()
        operation.takeoffAsync.return_value.get.return_value = result
        operation.landAsync.return_value.get.return_value = result
        fake = NS(MultirotorClient=Mock(return_value=operation),LandedState=NS(Flying=1,Landed=0))
        scope = dict(Node=object, airsim=fake, np=np, time=time, DepthGuard=Mock())
        exec(compile(ast.Module(body=[definition],type_ignores=[]),str(path),'exec'),scope)
        bridge = scope['AirSimBridge'].__new__(scope['AirSimBridge'])
        bridge.client = Mock()
        bridge.client.armDisarm.return_value = True
        bridge.client.getMultirotorState.return_value = NS(landed_state=landed)
        bridge.vehicle = 'Drone1'
        bridge.param = lambda key: {'host':'192.0.2.1','port':41451}[key]
        bridge.send = Mock()
        return bridge, operation, fake

    def test_takeoff_rejection_not_success(self):
        bridge, op, fake = self.bridge(False, 1)
        response = bridge.takeoff(None,NS(success=False,message=''))
        self.assertFalse(response.success)
        self.assertFalse(bridge.active)
        self.assertIn('rejected',response.message)
        bridge.send.assert_not_called()
        self.assertEqual(fake.MultirotorClient.call_args.kwargs['timeout_value'],20)
        op.takeoffAsync.return_value.get.assert_called_once()

    def test_takeoff_result_and_flying_state(self):
        bridge, op, fake = self.bridge(True, 1)
        self.assertTrue(bridge.takeoff(None,NS(success=False,message='')).success)
        bridge.send.assert_called_once_with((0,0,0))

    def test_land_rejection_does_not_disarm(self):
        bridge, op, fake = self.bridge(False, 0)
        response = bridge.land(None,NS(success=False,message=''))
        self.assertFalse(response.success)
        bridge.client.armDisarm.assert_not_called()
        self.assertEqual(fake.MultirotorClient.call_args.kwargs['timeout_value'],35)

    def test_land_requires_landed_confirmation(self):
        bridge, op, fake = self.bridge(True, 1)
        self.assertFalse(bridge.land(None,NS(success=False,message='')).success)
        bridge.client.armDisarm.assert_not_called()
        bridge, op, fake = self.bridge(True, 0)
        self.assertTrue(bridge.land(None,NS(success=False,message='')).success)
        bridge.client.armDisarm.assert_called_once_with(False,vehicle_name='Drone1')

if __name__ == '__main__': unittest.main()
