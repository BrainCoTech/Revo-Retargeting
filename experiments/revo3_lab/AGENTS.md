# Experiment rendering

## Shared algorithm constraint

- Improve the existing shared retargeting algorithm through parameters and loss terms. Pinch, rubbing, side swing and other motions are evaluation cases, not separate algorithms.
- Do not add task-specific mapping branches, externally prescribed robot slide trajectories or joint trajectories to make a case pass. Robot targets must follow from the input through the same shared solver.
- Compare before/after on identical input and regress other existing motions. Fixture construction and rendering must remain separate from the mapping algorithm; success on a robot excitation test is not retargeting success.

## Video requirements

- Every new rendered video must show the input MANUS skeleton alongside the robot output, using the exact input frame and original topology. Saved mapper points already include the configured reflection; do not reflect them again.
- Label recorded, frozen, and synthetic skeleton inputs explicitly. Never present a synthetic trajectory as a recorded human motion. For generated robot transitions with no human input, show a labeled held endpoint reference and state that no human bridge input exists.
- Render saved `q_actual`, and preserve source timestamps, provenance, and frame alignment. Include a close view when joint articulation or contact is under evaluation.
- Run numerical tests, optimization and video rendering only in the registered DSW environment; do not install a local Python test environment.
