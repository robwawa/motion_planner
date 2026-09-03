# PCT-SCAN Navigation

`navigation_manager.py` submits the current body pose and the user's original
3D goal to `/pct/plan_path`, validates the result, and publishes the remaining
route on `/navigation/reference_path` for SCAN.

## Global replan handoff

PCT computes global multi-layer routes and SCAN handles local collision
avoidance. SCAN is the sole source of global-replan requests.

When SCAN exhausts `fsm/max_replan_fail_count` local replan attempts in
reference-path mode, it first completes its emergency stop and then publishes
one `/scan/global_replan_request` event.  `navigation_manager` checks whether
the robot is already within `goal_reached_distance` of PCT's snapped endpoint.
Otherwise it sends PCT a new Action request from the latest odometry.

For each user goal, SCAN can hand off to PCT at most
`global_replan_max_cycles` times (default: 3). This total includes handoffs
whose PCT Action succeeds, so a route that repeatedly fails local planning
cannot cycle indefinitely. Once exhausted, the navigation manager publishes
`REPLAN_EXHAUSTED` and ignores further SCAN replan requests until a new user
goal arrives.

Within each accepted handoff, failed PCT global replans retry at most
`replan_max_attempts` times in total (default: 3), with 2 s and 4 s delays.
This Action retry budget is independent of the per-goal handoff budget.
