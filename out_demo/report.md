# Flight plan risk report — demo-ao

**Recommendation: REVIEW**

_Generated 2026-07-27T00:35:00.511984+00:00_

## Summary

| Metric | Value |
| --- | ---: |
| Route index | 0 |
| Total route length | 5135.6 m |
| Fiber used | 5135.6 m |
| Fiber available | 8000.0 m |
| Exposed length | 154.1 m |
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
| Max climb rate | 0.00 m/s |
| Max descent rate | 0.00 m/s |
| Climb-rate violations | 0 |

## Objective scores

| Objective | Score |
| --- | ---: |
| exposed_fraction | 0.0359 |
| length_m | 5135.5671 |
| max_exposure | 1.0000 |
| mean_exposure | 0.2186 |
| path_cost | 264.0169 |

## Findings

- ⚠️ **warning** (exposed_cable): 154 m of cable is visually/fire exposed
- **info** (descent_densified): 2 vertex/vertices inserted to split big-drop legs so the autopilot cannot descend ahead of the terrain (min clearance now 11.9 m AGL)
- ⚠️ **warning** (flyability_not_analyzed): concealment/defilade analyzed, but forest flyability was not (no lidar/voxel model); route assumes open flight — confirm before execution

