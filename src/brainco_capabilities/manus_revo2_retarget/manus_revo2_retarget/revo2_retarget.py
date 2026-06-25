"""Backward-compatibility shim for ``HandRetargeting``.

New code should use ``manus_revo2_retarget.retargeters`` directly.
"""

from pathlib import Path

from manus_revo2_retarget.retargeters.dex_retargeter import DexRetargeter


class HandRetargeting(DexRetargeter):
    """Legacy compatibility wrapper – instantiates the dex retargeter with the
    hard-coded default configuration.
    """

    def __init__(self):
        config_file_path = Path(__file__).parent / "brainco_hand" / "brainco.yml"
        super().__init__(config_file_path)


if __name__ == "__main__":
    hand_retargeting = HandRetargeting()
    left_finger_tip_pos = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
    right_finger_tip_pos = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
    [left_target, right_target] = hand_retargeting.retarget_process(
                left_finger_tip_pos, right_finger_tip_pos)
    print(f"left_target: {left_target}", f"right_target: {right_target}")
