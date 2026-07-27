# Flight plan risk report — demo-ao

**Recommendation: REVIEW**

_Generated 2026-07-27T23:36:01.634625+00:00_

## Summary

| Metric | Value |
| --- | ---: |
| Route index | 0 |
| Total route length | 5298.9 m |
| Fiber used | 5298.9 m |
| Fiber available | 8000.0 m |
| Exposed cable length | 287.2 m |
| Exposed flight length (drone) | 610.9 m |
| Flown below canopy | 2653.6 m |
| Corridor half-width (median / min) | 50 / 0 m |
| Non-navigable segments | 0 |
| Marginal segments | 0 |
| Crossings needing manipulation | 0 |

## Kinematic feasibility

**Trajectory feasible**

| Metric | Value |
| --- | ---: |
| Min turn radius | 8.0 m |
| Tightest turn achieved | 8.0 m |
| Turn-radius violations | 0 |
| Reversals | 0 |
| Max climb rate | 1.36 m/s |
| Max descent rate | 1.47 m/s |
| Climb-rate violations | 0 |

## Objective scores

| Objective | Score |
| --- | ---: |
| corridor_half_width_median_m | 50.0000 |
| corridor_half_width_min_m | 0.0000 |
| exposed_fraction | 0.1235 |
| length_m | 5298.9457 |
| max_exposure | 1.0000 |
| max_exposure_corridor | 1.0000 |
| max_exposure_flown | 1.0000 |
| mean_exposure | 0.1500 |
| mean_exposure_cable | 0.2706 |
| mean_exposure_corridor | 0.1642 |
| mean_exposure_flown | 0.1294 |
| path_cost | 263.7881 |

## Findings

- ⚠️ **warning** (exposed_cable): 287 m of cable is visually/fire exposed
- ⚠️ **warning** (below_canopy_flight): 2654 m of the route is flown below the canopy on canopy-top (DSM) evidence alone — trunks, branches and understory are unmodelled. The route is a corridor to follow approximately; clearing obstacles is the flight stack's job
- ⚠️ **warning** (exposed_drone): 611 m of the route is flown in view of threat observers at the planned flight altitude — the aircraft is detectable there even where the cable beneath it is concealed
- **info** (corridor_pinched): 32/282 waypoints (11%) grant no lateral freedom — the route runs at the edge of its cover there, so a deviation leaves the concealment the plan was scored on. Elsewhere the corridor is 50 m (median half-width)
- ⚠️ **warning** (flyability_not_analyzed): concealment/defilade analyzed, but forest flyability was not (no lidar/voxel model); route assumes open flight — confirm before execution

