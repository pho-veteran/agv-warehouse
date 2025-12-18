# Exploration Log Tags - Quick Reference

## Goal Selection Flow
```
[FILTER] → [SELECT] → [GOAL_SELECT] → [PATH_CHECK] → [NAV2]
```

## Log Tags

| Tag | Meaning | Common Causes |
|-----|---------|---------------|
| `[FILTER]` | Frontier rejected - too close to obstacle | `obstacle_clearance` too large |
| `[SELECT]` | Frontier rejected - too close to robot OR blacklisted | `min_goal_distance` too large, or goal failed before |
| `[GOAL_SELECT]` | Goal selection process | Shows how many frontiers checked |
| `[PATH_CHECK] REJECTED` | Nav2 planner can't find path | Costmap inflation blocking, goal in obstacle |
| `[PATH_CHECK] TIMEOUT` | IsPathValid service timeout | Nav2 overloaded or crashed |
| `[NAV2] Goal REJECTED` | Nav2 action server refused goal | Goal pose invalid |
| `[NAV2] Navigation FAILED` | Navigation aborted mid-way | Controller failed, robot stuck |
| `[BLACKLIST]` | Goal near previously failed goal | Too many failures in area |

## Status Codes (NAV2)
- `4 = SUCCEEDED` - Navigation completed
- `5 = CANCELED` - Goal was canceled
- `6 = ABORTED` - Planner/controller failed

## Key Parameters to Tune

| Issue | Parameter | File |
|-------|-----------|------|
| Too many `[FILTER]` | `obstacle_clearance` | exploration_params.yaml |
| Too many `[SELECT]` too close | `min_goal_distance` | exploration_params.yaml |
| Too many `[PATH_CHECK] REJECTED` | `inflation_radius` | tugbot_nav2_params.yaml |
| Too many `[BLACKLIST]` | `blacklist_radius` | exploration_params.yaml |
