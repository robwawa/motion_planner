# PCT-SCAN Navigation

The navigation supervisor is the single owner of mission execution. It accepts
goals from `/goal_pose_3d` and `/move_base_simple/goal`, calls the raw PCT
planner, processes the returned route, and sends the processed route to SCAN
through `/scan/follow_reference_path`.

The route pipeline is:

```text
goal_pose_3d / RViz goal
        -> navigation_supervisor
        -> /pct/plan_path                 (raw global path)
        -> ReferencePathProcessor         (start validation and truncation)
        -> /navigation/reference_path     (visualization)
        -> /scan/follow_reference_path    (SCAN Action)
```

`global_pct_planner` no longer owns route validation or publishes
`/pct/global_path`. `navigation_supervisor` owns mission and route IDs, global
replanning, cancellation, and recovery. SCAN keeps local trajectory
generation, local replanning, and braking; when its local recovery budget is
exhausted it returns `FAILED` with the reason `local_replan_exhausted`; the
navigation behavior tree decides whether another global planning attempt is
allowed.

Launch the integrated stack with:

```bash
roslaunch pct_scan_navigation pct_scan_demo.launch
```

The standalone supervisor launch is:

```bash
roslaunch navigation_supervisor navigation_supervisor.launch
```
