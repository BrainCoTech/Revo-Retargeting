"""Camera-free checks for recording boundaries and incomplete-session labels."""
import unittest

from guide import ACTIONS, Guide


class GuideChecks(unittest.TestCase):
    def test_restart_resets_timer_and_preserves_superseded_frames(self):
        guide = Guide()
        guide.record(guide.label(10), 0, 10, 'tracking')
        guide.request_start()
        old_label = guide.label(11)
        guide.record(old_label, 1, 11, 'tracking')
        guide.record(guide.label(12), 2, 12, 'missing_hand:hold')
        self.assertTrue(guide.request_restart())
        new_label = guide.label(12.5)
        guide.record(new_label, 3, 12.5, 'tracking')
        self.assertEqual(new_label['action_id'], old_label['action_id'])
        self.assertEqual((old_label['attempt'], new_label['attempt']), (1, 2))
        self.assertEqual(new_label['remaining_s'], ACTIONS[0][1])
        timeline = guide.timeline(12.5, 'interrupted')
        old = timeline['superseded_attempts'][0]
        latest = timeline['segments'][1]
        self.assertEqual(old['status'], 'superseded')
        self.assertEqual((old['frame_start'], old['frame_end_exclusive']), (1, 3))
        self.assertEqual((old['frames'], old['tracking_frames']), (2, 1))
        self.assertEqual(latest['status'], 'partial')
        self.assertEqual((latest['frame_start'], latest['frame_end_exclusive']), (3, 4))
        self.assertEqual(latest['planned_start_timestamp_s'], 12.5)
        self.assertEqual(latest['attempt'], 2)

    def test_restart_at_deadline_keeps_current_action(self):
        guide = Guide()
        guide.label(0)
        guide.request_start()
        guide.label(1)
        guide.request_restart()
        label = guide.label(1 + ACTIONS[0][1] + .1)
        self.assertEqual(label['phase'], 'record')
        self.assertEqual(label['action_id'], 'open_hold')
        self.assertAlmostEqual(label['remaining_s'], ACTIONS[0][1])

    def test_multiple_restarts_have_distinct_attempts(self):
        guide = Guide()
        guide.label(0)
        guide.request_start()
        for stamp in [1, 1.5, 2]:
            label = guide.label(stamp)
            guide.record(label, label['attempt'], stamp, 'tracking')
            if stamp != 2:
                guide.request_restart()
        timeline = guide.timeline(2, 'partial')
        self.assertEqual([a['attempt'] for a in timeline['superseded_attempts']], [1, 2])
        self.assertEqual(timeline['segments'][1]['attempt'], 3)
        self.assertEqual([a['frames'] for a in timeline['superseded_attempts']], [1, 1])

    def test_restart_before_first_recording_is_ignored(self):
        guide = Guide()
        self.assertFalse(guide.request_restart())
        guide.label(0)
        self.assertFalse(guide.request_restart())
        self.assertEqual(guide.label(1)['phase'], 'prepare')

    def test_restart_in_prepare_retries_previous_and_enter_advances(self):
        guide = Guide()
        guide.label(0)
        guide.request_start()
        guide.record(guide.label(1), 0, 1, 'tracking')
        self.assertEqual(guide.label(3)['action_id'], 'open_close')
        # Retry takes precedence over an Enter not yet applied to a frame.
        guide.request_start()
        self.assertTrue(guide.request_restart())
        retry = guide.label(4)
        self.assertEqual((retry['action_id'], retry['attempt']), ('open_hold', 2))
        self.assertEqual(retry['remaining_s'], 2)
        guide.record(retry, 1, 4, 'tracking')
        waiting = guide.label(6)
        self.assertEqual((waiting['action_id'], waiting['phase']), ('open_close', 'prepare'))
        self.assertEqual(guide.timeline(6, 'partial')['superseded_attempts'][0]['frames'], 1)
        guide.request_start()
        next_action = guide.label(7)
        self.assertEqual((next_action['action_id'], next_action['attempt']), ('open_close', 1))

    def test_waits_indefinitely_until_enter(self):
        guide = Guide()
        self.assertEqual(guide.label(42)['phase'], 'prepare')
        waiting = guide.label(1000)
        self.assertEqual(waiting['action_id'], 'open_hold')
        self.assertIsNone(waiting['remaining_s'])
        self.assertTrue(guide.request_start())
        started = guide.label(1001)
        self.assertEqual(started['phase'], 'record')
        self.assertEqual(started['remaining_s'], ACTIONS[0][1])
        self.assertEqual(guide.label(1001 + ACTIONS[0][1] - .001)['phase'], 'record')
        next_action = guide.label(1001 + ACTIONS[0][1])
        self.assertEqual(next_action['action_id'], 'open_close')
        self.assertEqual(next_action['phase'], 'prepare')
        self.assertEqual(guide.label(2000)['phase'], 'prepare')

    def test_enter_during_recording_is_not_queued(self):
        guide = Guide()
        guide.label(0)
        guide.request_start()
        guide.label(1)
        self.assertFalse(guide.request_start())
        self.assertEqual(guide.label(1 + ACTIONS[0][1])['phase'], 'prepare')
        self.assertEqual(guide.label(100)['phase'], 'prepare')

    def test_interruption_preserves_failed_tracking_frames(self):
        guide = Guide()
        guide.record(guide.label(10), 0, 10, 'tracking')
        guide.request_start()
        for frame, stamp, status in [(1, 16., 'tracking'), (2, 17., 'missing_hand:hold')]:
            guide.record(guide.label(stamp), frame, stamp, status)
        timeline = guide.timeline(17., 'interrupted')
        self.assertFalse(timeline['completed'])
        self.assertEqual(timeline['segments'][0]['status'], 'complete')
        action = timeline['segments'][1]
        self.assertEqual(action['status'], 'partial')
        self.assertEqual((action['frame_start'], action['frame_end_exclusive']), (1, 3))
        self.assertEqual((action['frames'], action['tracking_frames']), (2, 1))
        self.assertEqual(action['planned_start_timestamp_s'], 16)
        self.assertEqual(action['planned_end_timestamp_s'], 16 + ACTIONS[0][1])
        self.assertEqual(timeline['segments'][2]['status'], 'not_started')
        self.assertIsNone(timeline['segments'][2]['planned_start_timestamp_s'])

    def test_large_time_gap_cannot_skip_next_action(self):
        guide = Guide()
        guide.label(0)
        guide.request_start()
        guide.record(guide.label(1), 0, 1, 'tracking')
        label = guide.label(10000)
        self.assertEqual(label['action_id'], 'open_close')
        self.assertEqual(label['phase'], 'prepare')
        self.assertEqual(sum(s['frames'] for s in guide.timeline(10000, 'partial')['segments']), 1)

    def test_empty_failed_session(self):
        timeline = Guide().timeline(-1, 'failed')
        self.assertIsNone(timeline['origin_timestamp_s'])
        self.assertFalse(timeline['completed'])
        self.assertTrue(all(s['status'] == 'not_started' for s in timeline['segments']))

    def test_all_actions_require_separate_enter_and_finish(self):
        guide = Guide()
        stamp = 0.
        guide.label(stamp)
        for action in ACTIONS:
            self.assertEqual(guide.label(stamp)['phase'], 'prepare')
            guide.request_start()
            stamp += 2
            self.assertEqual(guide.label(stamp)['action_id'], action[0])
            self.assertEqual(guide.label(stamp)['phase'], 'record')
            stamp += action[1]
            guide.label(stamp)
        self.assertEqual(guide.label(stamp)['phase'], 'review')
        self.assertTrue(guide.request_restart())
        stamp += 1
        retry = guide.label(stamp)
        self.assertEqual((retry['action_id'], retry['attempt']), (ACTIONS[-1][0], 2))
        stamp += ACTIONS[-1][1]
        self.assertEqual(guide.label(stamp)['phase'], 'review')
        self.assertTrue(guide.request_start())
        self.assertEqual(guide.label(stamp)['phase'], 'done')
        self.assertFalse(guide.request_restart())
        self.assertTrue(guide.timeline(stamp, 'complete')['completed'])
        self.assertFalse(guide.request_start())

    def test_protocol_separates_touch_from_noncontact_sliding(self):
        guide = Guide()
        self.assertEqual(len({a[0] for a in ACTIONS}), len(ACTIONS))
        self.assertEqual([s['action_id'] for s in guide.phases if s['phase'] == 'record'][-2:],
                         ['contact_rubbing', 'gap_sliding'])
        self.assertEqual(len(guide.phases), 2 * len(ACTIONS))


if __name__ == '__main__':
    unittest.main(verbosity=2)
