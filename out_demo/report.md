# Flight plan risk report — demo-ao

**Recommendation: NO-GO**

_Generated 2026-08-03T17:52:41.350578+00:00_

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

## Endurance budget

**INFEASIBLE — outruns the battery**

| Metric | Value |
| --- | ---: |
| Flight time | 57.1 min |
| — cruise | 56.0 min |
| — vertical-rate bound | 0.0 min |
| — waypoint overhead | 0.8 min |
| — route ends | 0.3 min |
| Energy-equivalent time | 57.7 min |
| Usable budget | 24.0 min |
| Budget consumed | 240% |
| Endurance required (incl. reserve) | 72.1 min |
| Min sustainable cruise | 3.89 m/s |
| Vertical travel commanded | 284 m |

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
- ⛔ **critical** (endurance_exceeded): route needs 57.7 min of flight but the airframe has 30.0 min of endurance — it cannot be flown at 3.89 m/s or slower. Shorten the route, raise cruise speed, or fit a bigger battery
- ⚠️ **warning** (stem_speed_capped): 191 leg(s) are slowed below cruise by modelled stem spacing, giving an effective 1.58 m/s against a 6.00 m/s airframe cruise. Stem density is derived from canopy height by a self-thinning relation (fields/stem_density.py), NOT measured — no stem inventory has been ingested. The ordering (taller stands are sparser) is more trustworthy than the absolute density
- ⚠️ **warning** (flyability_not_analyzed): concealment/defilade analyzed, but forest flyability was not (no lidar/voxel model); route assumes open flight — confirm before execution

