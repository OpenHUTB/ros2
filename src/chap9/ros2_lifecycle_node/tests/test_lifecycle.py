"""生命周期状态机单元测试。

覆盖：初始状态、全部合法转移路径、非法转移拒绝、回调失败处理、
shutdown 的「任意状态」语义、发布门控，以及演示脚本的统计数字。
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "scripts"))

import lifecycle as lc
import simulate as sim


class TestInitial(unittest.TestCase):
    def test_starts_unconfigured(self):
        m = lc.LifecycleStateMachine()
        self.assertEqual(m.state, lc.STATE_UNCONFIGURED)
        self.assertFalse(m.is_active)
        self.assertEqual(m.actions(), ['configure', 'shutdown'])

    def test_history_records_init(self):
        m = lc.LifecycleStateMachine()
        self.assertEqual(m.history, [(lc.STATE_UNCONFIGURED, 'init')])


class TestLegalSequences(unittest.TestCase):
    def test_configure_then_activate(self):
        m = lc.LifecycleStateMachine()
        m.configure()
        self.assertEqual(m.state, lc.STATE_INACTIVE)
        m.activate()
        self.assertEqual(m.state, lc.STATE_ACTIVE)
        self.assertTrue(m.is_active)

    def test_deactivate_and_cleanup(self):
        m = lc.LifecycleStateMachine()
        m.configure()
        m.activate()
        m.deactivate()
        self.assertEqual(m.state, lc.STATE_INACTIVE)
        m.cleanup()
        self.assertEqual(m.state, lc.STATE_UNCONFIGURED)

    def test_reactivation_counts(self):
        m = lc.LifecycleStateMachine()
        for _ in range(3):
            m.configure()
            m.activate()
            m.deactivate()
            m.cleanup()
        self.assertEqual(m.active_count, 3)

    def test_shutdown_from_any_state(self):
        for prep in ([], ['configure'], ['configure', 'activate']):
            m = lc.LifecycleStateMachine()
            for act in prep:
                m.trigger(act)
            m.shutdown()
            self.assertEqual(m.state, lc.STATE_FINALIZED)

    def test_finalized_is_terminal(self):
        m = lc.LifecycleStateMachine()
        m.shutdown()
        for act in lc.TRANSITIONS:
            self.assertFalse(m.can(act))
            with self.assertRaises(lc.InvalidTransition):
                m.trigger(act)


class TestIllegalSequences(unittest.TestCase):
    def test_activate_without_configure(self):
        m = lc.LifecycleStateMachine()
        with self.assertRaises(lc.InvalidTransition):
            m.activate()
        self.assertEqual(m.state, lc.STATE_UNCONFIGURED)

    def test_double_configure(self):
        m = lc.LifecycleStateMachine()
        m.configure()
        with self.assertRaises(lc.InvalidTransition):
            m.configure()

    def test_cleanup_while_active(self):
        m = lc.LifecycleStateMachine()
        m.configure()
        m.activate()
        with self.assertRaises(lc.InvalidTransition):
            m.cleanup()
        self.assertEqual(m.state, lc.STATE_ACTIVE)

    def test_unknown_action(self):
        m = lc.LifecycleStateMachine()
        with self.assertRaises(lc.InvalidTransition):
            m.trigger('explode')

    def test_state_unchanged_after_reject(self):
        m = lc.LifecycleStateMachine()
        before = m.state
        try:
            m.activate()
        except lc.InvalidTransition:
            pass
        self.assertEqual(m.state, before)
        self.assertEqual(len(m.history), 1)


class TestCallbacks(unittest.TestCase):
    def test_success_callback_runs(self):
        calls = []
        m = lc.LifecycleStateMachine({
            'on_configure': lambda: calls.append('cfg') or True,
            'on_activate': lambda: calls.append('act') or True,
        })
        m.configure()
        m.activate()
        self.assertEqual(calls, ['cfg', 'act'])

    def test_failure_keeps_state(self):
        m = lc.LifecycleStateMachine({'on_configure': lambda: False})
        with self.assertRaises(lc.CallbackFailed):
            m.configure()
        self.assertEqual(m.state, lc.STATE_UNCONFIGURED)
        self.assertEqual(len(m.errors), 1)

    def test_exception_is_failure(self):
        def boom():
            raise RuntimeError('sensor not found')
        m = lc.LifecycleStateMachine({'on_configure': boom})
        with self.assertRaises(lc.CallbackFailed):
            m.configure()
        self.assertEqual(m.state, lc.STATE_UNCONFIGURED)
        self.assertIn('sensor not found', m.errors[0][1])

    def test_retry_after_failure(self):
        flag = {'ok': False}
        m = lc.LifecycleStateMachine({'on_configure': lambda: flag['ok']})
        with self.assertRaises(lc.CallbackFailed):
            m.configure()
        flag['ok'] = True
        m.configure()               # 修好之后还能再试
        self.assertEqual(m.state, lc.STATE_INACTIVE)


class TestPublishGating(unittest.TestCase):
    def test_publish_dropped_before_activate(self):
        m = lc.LifecycleStateMachine()
        pub = lc.ManagedPublisher(m)
        for i in range(5):
            self.assertFalse(pub.publish(i))
        self.assertEqual(pub.published, 0)
        self.assertEqual(pub.dropped, 5)

    def test_publish_after_activate(self):
        m = lc.LifecycleStateMachine()
        pub = lc.ManagedPublisher(m)
        m.configure()
        m.activate()
        self.assertTrue(pub.publish(1.0))
        self.assertEqual(pub.published, 1)

    def test_publish_stops_after_deactivate(self):
        m = lc.LifecycleStateMachine()
        pub = lc.ManagedPublisher(m)
        m.configure()
        m.activate()
        pub.publish(1.0)
        m.deactivate()
        self.assertFalse(pub.publish(2.0))
        self.assertEqual(pub.stats(), {'published': 1, 'dropped': 1})

    def test_messages_preserved_in_order(self):
        m = lc.LifecycleStateMachine()
        pub = lc.ManagedPublisher(m)
        m.configure()
        m.activate()
        for v in (3.0, 4.0, 5.0):
            pub.publish(v)
        self.assertEqual(pub.sent, [3.0, 4.0, 5.0])


class TestScript(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.result = sim.run()
        cls.stats = sim.summary(cls.result)

    def test_tick_count(self):
        self.assertEqual(self.stats['ticks'],
                         int(round(sim.DURATION / sim.TICK)) + 1)

    def test_exactly_two_rejections(self):
        self.assertEqual(self.stats['rejected'], 2)
        times = [round(t, 1) for t, _a, _w in self.result['rejected']]
        self.assertEqual(times, [6.5, 12.0])

    def test_active_twice(self):
        self.assertEqual(self.stats['active_count'], 2)

    def test_accepted_plus_dropped_equals_ticks(self):
        self.assertEqual(self.stats['published'] + self.stats['dropped'],
                         self.stats['ticks'])

    def test_final_state(self):
        self.assertEqual(self.stats['final_state'], lc.STATE_FINALIZED)

    def test_transitions_demo_length(self):
        steps = sim.transitions_demo()
        self.assertEqual(len(steps), 9)
        self.assertEqual(steps[0], ('configure', lc.STATE_INACTIVE))
        self.assertEqual(steps[-1][1], lc.STATE_FINALIZED)


if __name__ == "__main__":
    unittest.main(verbosity=2)
