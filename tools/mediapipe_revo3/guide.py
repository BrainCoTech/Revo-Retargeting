"""Timed recording prompts and source-aligned labels; no mapping dependencies."""
import shutil
import subprocess
import sys


# Each label describes the requested action, not a detector-verified event.
ACTIONS = [
    ('open_hold', 2, '张手保持', '掌心朝镜头，五指自然张开并保持，手腕尽量不动。',
     'Hold your hand open, palm facing camera.', 'Keep the wrist still; spread fingers naturally.'),
    ('open_close', 4, '握拳与张开', '自然握拳再张开，连续做两次。',
     'Close and open your hand twice.', 'Keep your whole hand in view.'),
    ('individual_flexion', 8, '单指屈伸', '从拇指到小指，依次单指弯曲再伸直，其他手指尽量保持。',
     'Bend and extend one finger at a time.', 'Order: thumb, index, middle, ring, little.'),
    ('spread_close', 2, '四指张开并拢', '四指伸直，张开再并拢，完成一次即可。',
     'Spread and bring together the four straight fingers.', 'One spread-and-close cycle.'),
    ('opposition_index', 2, '拇指对食指', '拇指接近食指，轻触后立即分开，完成一次即可。',
     'Thumb to INDEX: approach, briefly touch, release.', 'One cycle; move naturally.'),
    ('opposition_middle', 2, '拇指对中指', '拇指接近中指，轻触后立即分开，完成一次即可。',
     'Thumb to MIDDLE: approach, briefly touch, release.', 'One cycle; move naturally.'),
    ('opposition_ring', 2, '拇指对无名指', '拇指接近无名指，轻触后立即分开，完成一次即可。',
     'Thumb to RING: approach, briefly touch, release.', 'One cycle; move naturally.'),
    ('opposition_little', 2, '拇指对小指', '拇指接近小指，轻触后立即分开，完成一次即可。',
     'Thumb to LITTLE: approach, briefly touch, release.', 'One cycle; move naturally.'),
    ('contact_rubbing', 3, '拇指食指轻触揉搓', '拇指与食指指腹轻触，小幅往返揉搓，手腕尽量不动。',
     'Gently rub thumb and index pads back and forth.', 'Light contact; keep the wrist mostly still.'),
    ('gap_sliding', 3, '隔空滑动模拟拧瓶盖', '拇指与食指不接触，保持约一到两厘米间距，像拧瓶盖一样小幅往返相对滑动，尽量保持间距。',
     'Bottle-cap motion: slide thumb and index relative to each other.',
     'NO contact or object; keep an approximate 1-2 cm gap.'),
]


class Guide:
    def __init__(self, voice=False):
        self.origin = None
        self.current = 0
        self.start_requested = False
        self.restart_requested = False
        self.finished = False
        self.superseded_attempts = []
        self.phases = []
        for index, (name, duration, title, instruction, line1, line2) in enumerate(ACTIONS):
            for phase in ['prepare', 'record']:
                self.phases.append(dict(action_id=name, action_index=index + 1, phase=phase,
                    attempt=1 if phase == 'record' else 0,
                    title=title, instruction=instruction, display_lines=[line1, line2],
                    duration_s=duration if phase == 'record' else None,
                    start_offset_s=None, end_offset_s=None,
                    frame_start=None, frame_end_exclusive=None, first_timestamp_s=None,
                    last_timestamp_s=None, frames=0, tracking_frames=0))
        self.duration_s = sum(a[1] for a in ACTIONS)
        self.last_prompt = None
        self.speech = None
        self.say = shutil.which('say') if voice and sys.platform == 'darwin' else None
        if voice and self.say is None:
            print('语音提示不可用；请看窗口与终端提示。', flush=True)

    def request_start(self):
        """Accept Enter only while waiting; never queue a future action."""
        if self.origin is None or self.finished:
            return False
        if self.current >= len(self.phases):
            self.finished = True
            return True
        if self.phases[self.current]['phase'] != 'prepare':
            return False
        self.start_requested = True
        return True

    def request_restart(self):
        """Retry the last recording while waiting, or the active recording."""
        if self.origin is None or self.finished or self.current == 0:
            return False
        self.start_requested = False
        self.restart_requested = True
        return True

    def label(self, stamp):
        if self.origin is None:
            self.origin = stamp
            self.phases[0]['start_offset_s'] = 0.
        elapsed = stamp - self.origin
        # Process the key before the deadline transition: the user pressed R
        # while this action was still displayed, even if capture crosses its end.
        if self.restart_requested:
            if self.current == len(self.phases) or self.phases[self.current]['phase'] == 'prepare':
                self.current -= 1
            phase = self.phases[self.current]
            self.superseded_attempts.append(dict(phase, phase_index=self.current,
                superseded_at_timestamp_s=stamp, status='superseded'))
            phase.update(attempt=phase['attempt'] + 1, start_offset_s=elapsed,
                end_offset_s=elapsed + phase['duration_s'], frame_start=None,
                frame_end_exclusive=None, first_timestamp_s=None, last_timestamp_s=None,
                frames=0, tracking_frames=0)
            self.restart_requested = False
        if self.current < len(self.phases):
            phase = self.phases[self.current]
            if phase['phase'] == 'record' and elapsed >= phase['end_offset_s']:
                self.current += 1
                if self.current < len(self.phases):
                    waiting = self.phases[self.current]
                    if waiting['start_offset_s'] is None:
                        waiting['start_offset_s'] = elapsed
            if self.current < len(self.phases) and self.start_requested:
                self.phases[self.current]['end_offset_s'] = elapsed
                self.current += 1
                phase = self.phases[self.current]
                phase['start_offset_s'] = elapsed
                phase['end_offset_s'] = elapsed + phase['duration_s']
                self.start_requested = False
        if self.current >= len(self.phases):
            return dict(phase='done' if self.finished else 'review', elapsed_s=elapsed, remaining_s=0.,
                        label_kind='prompted_action_not_verified')
        phase = self.phases[self.current]
        return dict(phase_index=self.current, action_id=phase['action_id'],
            attempt=phase['attempt'],
            action_index=phase['action_index'], action_count=len(ACTIONS),
            phase=phase['phase'], title=phase['title'], instruction=phase['instruction'],
            display_lines=phase['display_lines'], elapsed_s=elapsed,
            phase_elapsed_s=max(0., elapsed - phase['start_offset_s']),
            remaining_s=None if phase['phase'] == 'prepare' else phase['end_offset_s'] - elapsed,
            label_kind='prompted_action_not_verified')

    def record(self, label, frame_index, stamp, status):
        if label['phase'] in ('done', 'review'):
            return
        phase = self.phases[label['phase_index']]
        if phase['frame_start'] is None:
            phase['frame_start'] = frame_index
            phase['first_timestamp_s'] = stamp
        phase['frame_end_exclusive'] = frame_index + 1
        phase['last_timestamp_s'] = stamp
        phase['frames'] += 1
        phase['tracking_frames'] += int(status == 'tracking')

    def prompt(self, label):
        key = (label.get('phase_index', label['phase']), label.get('attempt', 0))
        if key == self.last_prompt:
            return
        self.last_prompt = key
        if label['phase'] == 'done':
            message = '全部动作录制结束。'
        elif label['phase'] == 'review':
            message = '最后一条录制完成。按回车结束保存，按 R 重录最后一条。'
        elif label['phase'] == 'prepare':
            message = (f"准备第 {label['action_index']} 项，共 {len(ACTIONS)} 项。"
                       f"{label['title']}。{label['instruction']} 按回车开始下一条，按 R 重录上一条（首条开始前无效）。")
        else:
            message = f"{'重新开始' if label['attempt'] > 1 else '开始'}：{label['title']}。按 R 可重录当前动作。"
        print(f'\n{message}', flush=True)
        if self.say:
            self.stop_speech()
            try:
                self.speech = subprocess.Popen([self.say, message], stdout=subprocess.DEVNULL,
                                               stderr=subprocess.DEVNULL)
            except OSError:
                self.say = None

    def stop_speech(self):
        if self.speech is not None:
            if self.speech.poll() is None:
                self.speech.terminate()
            self.speech.wait()
            self.speech = None

    def timeline(self, last_stamp, run_status):
        elapsed = 0. if self.origin is None else max(0., last_stamp - self.origin)
        segments = []
        for phase in self.phases:
            item = dict(phase)
            item['planned_start_timestamp_s'] = (None if phase['start_offset_s'] is None
                                                  else self.origin + phase['start_offset_s'])
            item['planned_end_timestamp_s'] = (None if phase['end_offset_s'] is None
                                                else self.origin + phase['end_offset_s'])
            item['status'] = ('not_started' if not phase['frames'] else
                              'complete' if phase['end_offset_s'] is not None and elapsed >= phase['end_offset_s'] else 'partial')
            segments.append(item)
        superseded = []
        for phase in self.superseded_attempts:
            superseded.append(dict(phase,
                planned_start_timestamp_s=self.origin + phase['start_offset_s'],
                planned_end_timestamp_s=self.origin + phase['end_offset_s']))
        return dict(schema_version=3, protocol='hand_baseline_manual_retry_v6', run_status=run_status,
            completed=self.current >= len(self.phases), origin_timestamp_s=self.origin,
            planned_action_duration_s=self.duration_s, start_policy='enter_each_action', label_kind='prompted_action_not_verified',
            interval_convention='start inclusive, end exclusive; frames include tracking failures',
            timestamp_authority='frames.jsonl timestamp_s; raw.avi has fixed playback FPS',
            selection_policy='Use record segments and match action_id AND attempt; superseded attempts are excluded even if the latest is partial',
            superseded_attempts=superseded, segments=segments)


def print_plan():
    guide = Guide()
    print(f'引导录制：{len(ACTIONS)} 段动作，每段等待回车开始；动作计时合计 {guide.duration_s:g} 秒，准备时间不限。')
    for index, (_, duration, title, instruction, _, _) in enumerate(ACTIONS, 1):
        print(f'{index:2}. {title}（{duration} 秒）：{instruction}')
