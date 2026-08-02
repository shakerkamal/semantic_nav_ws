# Up-Front Recovery Evaluation — Execution Plan

**Status:** proposed, not started
**Goal:** raise the up-front lane in Chapter 7 from *demonstrated* to *simulation-validated*, and move the open-set A1/A2 comparison off the TurtleBot3 onto the depth-only UGV Rover.
**Budget:** ~6–8 h total; ~2–3 h harness work, ~4 h supervised running.
**Constraint:** code freeze holds. This plan adds an evaluation harness only — no new recovery capability, no new action class, no change to `up_front_policy.py` or the orchestrator.

---

## 1. Why this is worth doing

Chapter 7 currently reports the up-front lane at demonstration level: one end-to-end rover run, plus a three-model open-set comparison collected on the TurtleBot3. Two consequences follow, and both are stated openly in the chapter as written:

- the thesis cannot claim the **complete dual-lane architecture** was simulation-validated;
- the strongest up-front evidence — that a deterministic affordance table cannot make the correct directive *eligible* — rests on a platform with a 360° sensor, not the one the thesis is about.

This plan removes both caveats. It does **not** attempt to make the up-front lane a second principal contribution; the en-route grid remains the centre of gravity. The target is a defensible existence result with N=5, matching the en-route grid's statistical posture.

**If time runs out, do not start this.** The chapter is complete and defensible without it, and §7.5 is written so the results drop in without restructuring.

---

## 2. What already exists (no work needed)

The three arms are already selectable from launch parameters. This is the single biggest reason the plan is cheap.

| Arm | Launch configuration | Isolates |
|---|---|---|
| **U0** | `up_front_recovery_enabled:=false` | geometric baseline — pre-flight fails, nothing recovers |
| **U1** | `up_front_recovery_enabled:=true`, `open_set_inference_enabled:=false` | deterministic affordance table |
| **U2** | both `true` | LLM open-set affordance inference |

U1 and U2 share the same geometric diagnosis, the same responsible-object matcher, the same eligible-action construction, the same standoff geometry, the same Nav2 revalidation, and the same operator gate. **Only the affordance source changes.** That is what makes the comparison interpretable, and it is the up-front analogue of the en-route Tier-1 byte-parity argument.

Also already present:

- `eval/open_set_scenario.md` — the scenario is fully specified (goal `bed:120`, blocker `room partition:121`, the tag deliberately absent from `object_action_attributes.json`).
- `close_partition.sh` / `open_partition.sh` — blocker spawn and removal.
- `eval/open_set_A1.txt`, `eval/open_set_A2_<model>.txt` — four reference traces with exact `[UP_FRONT]` log lines. **These are the parser's test fixtures**, which removes most of the risk from step 3.1.
- `eval/run_enroute_trial.sh` — pre-flight gates (`/cmd_vel` publisher set, `/operator_decision` server, `/map` residual check) that apply unchanged.
- `eval/map_residual_check.py`, `eval/auto_map.sh` — reset tooling.

---

## 3. Work items

### 3.1 Log parser — `eval/upfront_ablation.py` (~1.5 h)

Model it on `eval/enroute_ablation.py::parse_trial`. It is **simpler** than the en-route parser: there is no mid-drive blockage trigger to correlate, because the blocker is present from the start.

Events to extract, all of which appear verbatim in the four existing traces:

| Field | Log anchor |
|---|---|
| `initial_path_valid` | absence of `[UP_FRONT] Pre-flight validation failed` |
| `upfront_recovery_triggered` | `[UP_FRONT] Pre-flight validation failed; entering up-front blockage recovery.` |
| `attempt`, `diagnosis`, `barrier_centroid_x/y` | `[UP_FRONT] attempt=N diagnosis=... centroid=(x, y)` |
| `affordance_inferred`, `inf_openable`, `inf_clearable`, `inf_safety`, `affordance_confidence` | `[AFFORDANCE] tag=... -> openable=... clearable=... safety=... confidence=...` |
| `eligible_actions` | `[UP_FRONT] eligible=[...]` |
| `raw_directive` | `llm='...'` on the same line |
| `accepted_directive`, `proposal_overridden`, `override_reason` | `-> directive=... (overridden=... reason=...)` |
| `responsible_tag`, `responsible_match_type` | `responsible_tag='...' match_type=...` |
| `dist_to_barrier_m`, `within_verify_range` | same line |
| `llm_latency_s`, `llm_calls` | `[UP_FRONT] LLM recovery response in N.Ns` |
| `standoff_reached` | `[EXECUTION] ... object_key='__standoff__'` with `SUCCEEDED` |
| `recheck_polls`, `plan_ok_final` | `[UP_FRONT] Recheck poll=N: barrier=... plan_ok=... barrier_ok=...` |
| `operator_action_completed` | `[UP_FRONT] Operator decision for '...': acknowledged=True` |
| `original_goal_revalidated` | `[UP_FRONT] Goal reachable ...; dispatching.` |
| `escalated` | `[UP_FRONT] Directive '...' needs operator action; escalating.` |
| `navigation_success`, `terminal_outcome` | final `[EXECUTION] Executor finished ... object_key='bed:120'` |
| `original_target_preserved` | final `object_key` equals the resolved original |

**Two fields must stay separate**, exactly as in the en-route parser: `raw_directive` (what the model proposed) and `accepted_directive` (what the filter admitted). Collapsing them makes the containment claim unmeasurable.

**Critically: do not emit a `barrier_clear_succeeded` column.** The up-front barrier-clear gate is a known near-no-op — a one-cell-thick barrier cannot reach the lethal-fraction threshold, so it always reports `cleared`. Recording it invites someone to cite it later. The real up-front gate is `plan_ok`, and that is what `original_goal_revalidated` captures.

Write it against the four existing traces first; they cover the escalation path (A1, 8B) and the success path (14B, 32B).

### 3.2 Scenario definition — `eval/upfront_scenarios.yaml` (~0.5 h)

Mirror the structure of `enroute_scenarios.yaml`.

| ID | Scenario | Expected U2 behaviour | Expected U1 behaviour |
|---|---|---|---|
| **U-A** | Reachable control: no blocker, goal `bed:120` | recovery must **not** trigger | identical |
| **U-B** | Open-set barrier: `room partition:121` sealing the bedroom doorway | affordance inferred → `approach_and_recheck` → `open_door_then_replan` → operator opens → goal revalidates → reached | `open_door_then_replan` never eligible → escalation |

U-A is the true-negative control and is cheap. U-B is the substantive cell.

**Do not add an unknown-barrier scenario.** It is the one the requirements document argues for hardest, and it is genuinely valuable — a diagnosis architecture should be judged on rejecting unsupported attribution, not only on correct positive attribution. But it needs a new blocker asset, a new expected-outcome class in the parser, and a further 15 runs. Under the current deadline it is the first thing to cut, and its absence is a stated limitation rather than a silent gap.

### 3.3 Trial runner — `eval/run_upfront_trial.sh` (~0.5 h)

Copy `run_enroute_trial.sh` and:

- **keep** all three pre-flight gates unchanged;
- **drop** the blockage-trigger invocation — the partition is spawned before the run by `close_partition.sh`, not injected mid-drive;
- add the arm parameter, which selects the two launch flags in §2;
- issue the natural-language command (`tired`) rather than a direct goal, so the resolver and ranker are exercised as they are in the real system.

---

## 4. Run matrix and protocol

| Scenario | Arms | Reps | Runs |
|---|---|---|---|
| U-A control | U0, U1, U2 | 5 | 15 |
| U-B open-set barrier | U0, U1, U2 | 5 | 15 |
| | | **Total** | **30** |

At ~160 s per trial plus a ~10 min reset cycle, that is roughly **5 h supervised**. To fit ~4 h, run U-A at 3 reps (control cells are low-variance and a false trigger would show immediately), giving **24 runs**.

Protocol, identical to the en-route campaign so the two are comparable:

1. Fresh SLAM session per repetition; no saved map.
2. `map_residual_check.py` gate before each rep — the partition's coordinate must read free.
3. `/cmd_vel` publisher-set gate before each rep.
4. `navigation_terminal` running in its own TTY for the operator confirm.
5. One LLM warm-up call after any relaunch.
6. Qwen3-14B via the Zenoh bridge, matching the en-route grid.
7. Operator responds to the open-door prompt within a fixed interval; record the interval, since it enters the resolution time.
8. Discard and re-run only on a pre-declared infrastructure fault (absent operator service, stray `/cmd_vel` publisher, residual-map gate). Record every discard.

### Ordering

Run **U-B/U2 first**. It is the cell that must work for the campaign to be worth finishing; if it does not reproduce the demonstration run, stop and reassess rather than burning 4 h on the remaining cells.

---

## 5. What this will and will not support

**Will support** (upgrading §7.5 to simulation-validated):

- The up-front lane triggers correctly and does not fire on a reachable goal (U-A, all arms).
- U0 cannot recover a pre-flight-blocked goal — the up-front analogue of the en-route B-GEO result.
- U1 escalates because the correct directive is never *eligible*, not because it was proposed and rejected. This is the structural claim, and with N=5 on the rover it stops being a single-trace anecdote.
- U2 reaches the original target with the goal preserved, at a measured cost.

**Will not support**, and must remain stated as such:

- Any claim beyond one blocker class. This is an existence result, exactly as the en-route grid is.
- Rejection of unsupported attribution, since the unknown-barrier scenario is cut (§3.2).
- Significance testing. N=5 descriptive counts, same posture as §7.11.
- Anything about the up-front clearance gate, which remains a near-no-op.

---

## 6. Chapter edits once the data lands (~20 min)

1. Uncomment and fill `tab:eval-upfront-grid` in `experimentalEvaluation.tex` §7.5 (the scaffold is already in place with the arm definitions).
2. Replace the opening framing of §7.5 — "reported at demonstration level" — with the gridded result.
3. Change the up-front rows of `tab:eval-traceability` and `tab:eval-answers` from *demonstrated* to *simulation-validated*.
4. Remove the TB3 platform caveat from §7.5 and §7.11 external validity, replacing it with the retained single-blocker-class limitation.
5. Add an up-front outcomes figure alongside `fig:eval-enroute-outcomes`; `eval/plot_thesis_figures.py` can take a `fig_upfront_outcomes` in the same style.

---

## 7. Open decision for the author

The requirements document also asks for a **deterministic en-route semantic baseline** (`BDet`: same Tier-3 diagnosis and actions, deterministic directive selection). That would isolate model selection from the semantic mechanism, which the current en-route grid explicitly cannot do.

It is **not** in this plan. It is a separate campaign of comparable size, and the chapter already states the limitation conservatively in §7.6.2 and §7.12. If a choice must be made between the two, the up-front campaign is the better investment: it closes a claimed-but-unevaluated gap in the architecture, whereas `BDet` would sharpen an already-honest claim.
