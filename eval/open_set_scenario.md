# Open-Set Affordance Scenario — "room partition" (spec §21.4/§21.5)

The one scenario where the LLM **provably** beats the deterministic affordance
table under a **fixed goal**: the blocker's tag is *not* in
`object_action_attributes.json`, so the table can only fall back to its
restrictive default (`openable=false, clearable=false`). The LLM, reading the
object's caption, infers the correct affordance and unlocks a same-goal
recovery. This isolates open-set generalisation from every geometric factor.

> **Fixed goal (professor's constraint).** The goal — reach the **bed** — never
> changes between A1 and A2. Only the *recovery strategy* differs. The LLM does
> not pick a different destination; it recovers the *same* goal better.

## Objects (map_v001.json)

| key | tag | caption (abridged) | bbox_center | role |
|---|---|---|---|---|
| `object_120` | `bed` | "A double bed in the far bedroom, past the folding room partition." | `(-6.165, 2.031, 0.3)` | **goal** |
| `object_121` | `room partition` | "A folding room partition / privacy screen standing across the bedroom doorway; it can be slid or folded aside to pass through." | `(-2.461, 1.844, 1.0)` | **blocker** |

`room partition` is deliberately a **novel, non-door tag** (verified absent from
`object_action_attributes.json` `by_tag`, and not caught by the `"door"`
substring rule) — the point is to generalise beyond doors, not to special-case
one. Its Gazebo counterpart is the closed `FoldingDoor_01_001` panel at the
same pose (perception is simulated: the map is authored, the panel spawned).

## The gate

The orchestrator's up-front loop, per blocker:

```
tag_is_classifiable("room partition", table_tags) -> False
  -> (A1) open_set_inference_enabled = false  -> table default
                openable=false, clearable=false, safety_class=none
  -> (A2) open_set_inference_enabled = true   -> /infer_affordance(tag, caption)
                LLM reads caption -> openable=true (foldable/slidable aside)
```

The inferred affordance then feeds the **existing** M4 pipeline unchanged
(`eligible_directives → select_and_override_directive`). Nothing about the
selection logic is open-set-specific; only the affordance source changes.

## A1 — deterministic table only (baseline)

Launch with `open_set_inference_enabled:=false`.

Expected `[UP_FRONT]` trace:
- `open_set_inference_enabled=false` → no `/infer_affordance` call.
- `aff` = table default: `openable=false clearable=false safety=none`.
- `eligible_directives` for a non-openable, non-clearable blocker yields no
  `open_door_then_replan` / `clear_object_then_replan` → the eligible set
  collapses to `approach_and_recheck` (if a standoff exists) then, once
  exhausted, `give_up`.
- Terminal outcome: escalates to the operator menu (autonomous **FAIL** — the
  system cannot, on its own, decide the partition is passable).

## A2 — open-set inference enabled (contribution)

Launch with `open_set_inference_enabled:=true` (default) and
`navigator_node` + `llama_ros` up.

Expected `[UP_FRONT]` trace:
- `open-set affordance inferred for tag='room partition': openable=true ...`
- `aff.openable=true` → `eligible_directives` now includes
  `open_door_then_replan`.
- `directive=open_door_then_replan reason=llm_selected` → operator is asked to
  fold the partition aside → rescan → barrier clear → validate → **SUCCESS**
  (same goal: the bed).

## Difference that matters

| | A1 (table only) | A2 (open-set inference) |
|---|---|---|
| goal | bed | bed (**unchanged**) |
| inferred openable | false (default) | true (from caption) |
| eligible recovery | approach → give_up | open_door_then_replan |
| autonomous outcome | FAIL / operator escalation | SUCCESS |

A1 ≈ A2 would be a refutation of the LLM's value. Here they **diverge** because
the deciding fact (the partition is foldable) lives only in the caption, which
only the LLM reads — the deterministic table cannot represent a tag it never
enumerated.

## Run steps (Task 7)

See `2026-07-08-open-set-affordance-inference.md` Task 7. Bring up the stack +
`navigator_node` + `llama_ros`; confirm `ros2 service list | grep
infer_affordance`; spawn the closed panel with `close_partition.sh` at
`(-2.461, 1.844)`; issue the bed goal (`bed:120`); capture `[UP_FRONT]` logs to
`eval/open_set_A1.txt` and `eval/open_set_A2.txt`.

The ablation is exposed as a launch arg
(`open_set_inference_enabled:=false|true`) **and** read live at use-time, so you
can run A1 then A2 on the *same* SLAM map without a relaunch:

```bash
ros2 param set /navigation_orchestrator open_set_inference_enabled false   # A1
# ... issue bed:120, capture ...
ros2 param set /navigation_orchestrator open_set_inference_enabled true    # A2
# ... open the partition, re-issue bed:120, capture ...
```

The sibling M4 switch `up_front_llm_enabled` is live-togglable the same way.
