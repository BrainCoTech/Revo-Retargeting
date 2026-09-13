# MANUS → Revo3 CPU lab

## Shared solver iteration

The current optimization work uses one `vector_solver.Solver` across all motion
cases. Opt-in losses add observable lateral references and input-derived local
pad vectors for all four thumb partners, with separate tangential and normal
scales. No motion labels, independent slide command or task-specific joint
trajectory enters this path. `shared_batch_01.json` and `shared_batch_02.json`
are experimental parameter comparisons, not validated defaults.

`check_shared_solver.py` checks Jacobians, input-axis separation, backside
activation and frozen legacy compatibility. `run_shared_evaluation.py` compares
identical inputs across motions; `analyze_shared_evaluation.py` separates raw
input from calibrated surface targets and reachable synthetic inputs.
`render_shared_evaluation.py` displays the exact input skeleton and two saved
dynamic outputs, with optional `--close-view`.
See `docs/revo3_shared_solver_iteration.md` for evidence and remaining limits.

## Articulated rubbing and mandatory input rendering

**Withdrawn as a retargeting result:** the second iteration prescribed robot
sliding independently of its skeletal input. Its results below describe robot
excitation only. Future improvements must modify parameters/losses in the shared
retargeting algorithm and compare identical inputs across multiple motions.

The second rubbing iteration uses bone-preserving synthetic MANUS-topology motion
with coordinated PIP/DIP flexion and extension, plus an explicit ±4 mm slide target.
The two-second candidate maintains 100% contact for three cycles and measures
about 7.18 mm surface-slip excursion per cycle. All four distal joints exceed 13.6°
in every cycle. Absolute posture errors remain substantial; this is contact-first
relative motion mapping, not faithful recorded human rubbing. Reproduction and
limitations are in `docs/revo3_rubbing_iteration.md`.

All new videos must include the input skeleton with source timing and clear
recorded/frozen/synthetic provenance. Rubbing renders saved input points next to
actual dynamics and a pad closeup. Composite opposition transitions display a
labeled held reference when no human bridge input exists. See `AGENTS.md`.

## Thumb-index rubbing (synthetic task excitation)

`run_rubbing.py` adds synthetic longitudinal sliding targets to the frozen MANUS
opposition pose and measures relative surface velocity at each valid contact.
`check_rubbing.py` includes existing pinch regressions and independent sliding
Jacobian/velocity checks; all nine pass. The selected ±2 mm, two-second-period
candidate with 1.8 mm software preload maintains 100% valid pad contact over
three cycles, with about 3.25 mm measured surface-slip excursion per cycle.
Half-timestep and friction ±20% checks retain 99.8–100% contact. Ten dynamics
runs, including four rejected candidates, are retained. This is synthetic task
excitation on a frozen MANUS pose; real rubbing-input tracking and secondary
finger tracking remain unvalidated. See `docs/revo3_rubbing_iteration.md` for
protocol and reproduction; selected run IDs are in `configs/rubbing_selected_runs.json`.

## Four-finger side-swing iteration

`run_side_swing.py` compares legacy projected angles, pi-branch correction,
observability handling, flexion collision optimization and conservative retreat.
`check_side_swing.py` checks distinct finger identities, scale/translation,
per-finger missing observations and limits. `render_side_swing.py` compares saved
dynamic states across runs. Side-swing videos include the exact 25-point input
skeleton alongside robot output in a two-column layout, with source timestamps
and recorded/synthetic provenance. Do not apply the palm reflection again to
saved mapper inputs. All are opt-in experiments; opposition solvers are
unchanged. The known-angle fixture passes with observability plus retreat, but
real stress clips lose substantial motion under retreat and are not qualified
as faithful real-data mappings. See `docs/revo3_side_swing_iteration.md` and
`artifacts/revo3_lab/reports/side_swing_iteration_20260913.md` for the three-round
results and limitations. `trajectory_catalog.py` indexes these runs and prints
their reproduction commands by run ID.

## Trajectory reuse and continuous opposition

See `docs/revo3_trajectory_reuse_and_real_data.md` for the classified trajectory
inventory, exact replay commands, iteration findings and real-data adapter plan.
`trajectory_catalog.py build` creates an immutable catalog snapshot and refreshes
`registry/trajectory_catalog.json`; `list` shows selected aliases, and
`command --case middle_clip_1` prints a frozen-code reproduction command.
Add `--current-code` to evaluate current code against the same frozen fixture.

`run_opposition_sequence.py` re-simulates index → middle → ring → little in one
continuous physics rollout with generated neutral bridges, then renders saved
actual states from two views. The 2026-09-13 run lasts 29.13 seconds; all existing
contact, penetration, command and release gates pass. Index/middle clips are
recorded MANUS; ring/little clips are synthetic human IK. This composite demo
uses cached mapped goals and a predetermined order; online partner detection and
independent real ring/little validation remain future work.

Offline vector experiments run only on registered DSW `dsw-86hev0txafus51z1hp`
(2 CPU / 24 GiB), using `/mnt/workspace/wensheng/revo3-retargeting-lab/envs/core-py312`.
No local Python test environment or ROS production path is used.

Revo3 hardware capability is confirmed by the user. Hardware capability tests are
skipped; MuJoCo dynamics here evaluate mapping and software controller behavior.

## Commands

Run on the DSW, with these task-specific variables:

```sh
LAB_ROOT=/mnt/workspace/wensheng/revo3-retargeting-lab
LAB_PY=$LAB_ROOT/envs/core-py312/bin/python
LAB_CLI=$LAB_ROOT/repos/Revo-Retargeting/experiments/revo3_lab/scripts/lab.py
PYTHONNOUSERSITE=1 "$LAB_PY" "$LAB_CLI" inventory
PYTHONNOUSERSITE=1 "$LAB_PY" "$LAB_CLI" assets
PYTHONNOUSERSITE=1 "$LAB_PY" "$LAB_CLI" ingest
PYTHONNOUSERSITE=1 "$LAB_PY" "$LAB_CLI" check
PYTHONNOUSERSITE=1 "$LAB_PY" "$LAB_CLI" run --config repos/Revo-Retargeting/experiments/revo3_lab/configs/vector_batch_03.json
PYTHONNOUSERSITE=1 "$LAB_PY" "$LAB_CLI" report --run-id RUN_ID
```

`run --max-frames 180` limits each sequence for a timing smoke. Full runs replay
both complete sequences at 30 Hz. Each run gets a new directory and saves a code
snapshot, manifest, config, logs, metrics, q_target/q_actual and contact records.
One advisory process lock enforces serial experiments. `initialize_lab.py` and
`bootstrap_cpu.py` are the existing P0/environment setup commands; no environment
bootstrap is needed for ordinary runs. Archive/cleanup subcommands are not yet
implemented.

## Scope and results

- `prepare_assets.py`: fixed official right-hand XML and every referenced mesh;
  pinned ufbx v0.17.1 C evaluator, instead of a full Blender installation.
- `ingest_manus.py`: evaluates actual FBX key times and full transforms; stores
  units, parents, raw world transforms, palm landmarks, masks and quality metrics.
  Re-ingestion verifies identical data without changing a frozen normalized set.
- `revo3_model.py`: unchanged official 21-axis kinematics and torque/joint limits,
  explicit software PD. v2 restores direct parent collision filtering for the
  fixed root. This is a comparison controller, not a hardware-identified one.
- `vector_solver.py`: newly built thumb-IK/finger-angle reference, fingertip and
  relative-vector objectives, temporal/posture regularization, proximity
  hysteresis, and analytic contact-separation penalties. All candidates share
  filtering and a 4 rad/s command rate limit.
- `check_vector.py`: seven meaningful numerical regression checks.
- `summarize_iteration.py`: comparisons, latency tails and time-separated failure
  indexes from completed runs. `render_vector.py` optionally renders q_actual
  from saved dynamics with EGL; image rendering itself is pose playback.

Three initial batches completed; see
`artifacts/revo3_lab/reports/vector_iteration_20260913.md`. Local artifacts are
archived only under `/Users/woltim/code/Revo-Retargeting/artifacts/revo3_lab`;
remote outputs remain authoritative. The two `/mnt/workspace` and `/mnt/data_nas`
paths alias one NFS storage and are not separate backups.

The official examples have no verified side/person/contact/sliding annotations.
The source palm reflection is explicit in config; it is not a claim of side
identification. The two files are integration/tuning and validation inputs, not a
final test set. OpenGraph ingestion is pending. Distal mesh contacts and summed
contact-point slip are diagnostics, not verified fingerpad contact or task slip.
No rubbing success or qualified teacher labels are claimed. See
`docs/revo3_manus_execution_plan.md` for remaining acceptance work.

## Pad-opposition follow-up

`pinch_geometry.py` now defines thumb/index pad regions and normals by ray
intersection with the original distal collision STL surfaces. It adds no collision
geometry. `run_pinch.py` compares the old vector and pad-vector mappings of the
real finger_agility pose at 21.3 s, explicitly freezing that input for a two-second
hold after a common approach. The hold reached 100% valid pad contact, with
maximum rollout penetration 0.131 mm. Three numerical perturbation cases passed.

`run_pinch_clip.py` separately replays real chronological short clips. The selected
controller uses a discrete-braking command limiter and two frames of causal
closing-gap prediction. With identical acceptance thresholds, finger_agility
20.8–21.8 s reached 97.7% contact coverage and hand_mobility 25.8–26.7 s reached
99.0%. Both clip task gates passed; these already-inspected public examples are
not an independent final test. Other three fingers stay at identical neutral
commands in both baselines. Full-sequence contact/release and rubbing remain open.

Reproduce inside the registered DSW environment:

```sh
LAB_SCRIPTS=$LAB_ROOT/repos/Revo-Retargeting/experiments/revo3_lab/scripts
"$LAB_PY" "$LAB_SCRIPTS/check_pinch.py"
"$LAB_PY" "$LAB_SCRIPTS/run_pinch.py"
"$LAB_PY" "$LAB_SCRIPTS/run_pinch_clip.py" --lead-s 0.06666666666666667
"$LAB_PY" "$LAB_SCRIPTS/run_pinch_clip.py" --lead-s 0.06666666666666667 --sequence hand_mobility --start-s 25.8 --end-s 26.7 --pose-s 26.2
"$LAB_PY" "$LAB_SCRIPTS/check_pinch_dynamics.py"
"$LAB_PY" "$LAB_SCRIPTS/render_pinch.py" --run-id RUN_ID
```

Five pad/contact/command numerical checks pass. Complete artifacts, rejected
candidates, videos and exact selected configs are in each immutable run and in
`artifacts/revo3_lab/reports/pinch_iteration_20260913.md`. Source force truth,
full force/sliding acceptance and teacher-data qualification remain unavailable.

## Middle, ring and little opposition

All three isolated pair subtasks now pass their selected software simulation
cases. Middle uses recorded MANUS clips (96.2% and 98.8% valid contact coverage).
Ring and little use explicitly labeled, bone-preserving human IK synthetic
close-hold-release inputs (98.1% and 96.4%). These synthetic inputs do not establish
real MANUS generalization or anatomical validity. Each pair has a two-second
100% contact hold; middle's hold freezes a recorded pose. Both synthetic clips
release at the end.

`synthesize_partner_input.py` creates the missing inputs without moving inactive
human landmarks or changing bone lengths. `pinch_geometry.py` selects the correct
partner DOFs and original mesh pad regions. The little-finger configuration uses
surface targets shifted 4 mm across each pad, selected from nine candidates; no
geometry, pad-region limits, acceptance thresholds or collision exclusions were
changed. Inactive robot fingers receive neutral commands in these pair tests.

Exact selected configs, runs and playback IDs are in
`configs/partner_selected_runs.json`. Reproduce a selected clip with its frozen
`runs/RUN_ID/config.json` and the recorded `--lead-s` argument. Existing index
defaults remain available. Nine regression checks, three synthetic-input checks
and nine fixed-goal numerical perturbation cases pass. The clips are solved
offline and replayed at 30 Hz; online end-to-end latency is not validated.

`report_partners.py` audits saved contacts, including longest contact breaks and
synthetic release. `archive_partners.py` preserves all candidates and records a
hash inventory. Local report and videos are linked from
`artifacts/revo3_lab/reports/partner_iteration_20260913.md`. Full-sequence switching,
simultaneous five-finger motion, rubbing and independent real ring/little input
validation remain outside this completed pair stage. Hardware capability tests
remain skipped as requested.
