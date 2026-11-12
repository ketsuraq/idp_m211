# How the robot navigates

The robot does not count turns from a script. It holds a **map of the field**, knows **where it is and which way it faces**, and works out each turn from that.

## 1. The map: junctions and compass directions

Stand at the start box facing the ramp: **N** = towards the ramp/racks, **E** = right, **S** = towards you, **W** = left.

Every junction on the field is a node in `GRAPH` (`state_machine.py`). Each node lists the lines leaving it and where each one leads:

```python
"right_corner":   {"W": "right_box_edge", "N": "right_depot_1"},
"right_depot_1":  {"S": "right_corner", "N": "right_depot_2", "W": "right_depot_1_end"},
"right_box_edge": {"W": "start", "E": "right_corner", "N": None, "S": None},
```

- `None` means there is a line there, but the robot never takes it (e.g. the start box edge crossing the main line).
- A node with only one line is a **dead end**, i.e. the end of a branch (`..._end`).
- Only the right side is written out. The left side is its mirror image, with E/W and `right_`/`left_` swapped automatically.
- Numbering: depot 1 / pickup 1 are nearest the corner. The "upper" depot is `depot_2`.

```
            right_end
                |
  pickup_6_end--+ right_pickup_6
       ...      |   ...
  pickup_1_end--+ right_pickup_1
  depot_2_end---+ right_depot_2      (upper green)
  depot_1_end---+ right_depot_1
                |
 start ----+----+ right_corner
     right_box_edge
```

## 2. Routes

The field has no loops, so there is exactly one way between any two places. To get from A to B, the robot heads back towards `start` until it reaches a junction that is also on B's way to `start`, then follows B's way out. For example, pickup 3 → upper green depot never goes near the start, but pickup 3 → red depot goes through the start.

To see a route in plain words, run `python3 idp/tests/route_test.py` on a laptop:

```
right_pickup_3_end   facing W: reverse
right_pickup_3       facing W: left
right_pickup_2       facing S: straight
right_pickup_1       facing S: straight
right_depot_2        facing S: right
right_depot_2_end    arrive
```

## 3. Turns

The robot always knows which way it faces. At each junction, it compares that with the compass direction of the next line:

| Next line is… | Action |
|---|---|
| same direction as facing | `straight` |
| 90° clockwise | `right` |
| 90° anticlockwise | `left` |
| behind (only at a dead end) | `reverse` out, still facing the same way |

That's why the same junction can be a left turn on the way up and a right turn on the way down.

## 4. Checking the map against the sensors

The map also says what each junction **should** look like. Arriving at `right_depot_1` facing N, there is a line to the W, so the left outer sensor should fire. If the sensors disagree, the robot stops and prints what it expected and what it saw (`STOP_ON_MISMATCH` in `state_machine.py`). This makes miscounted junctions obvious during testing. Later it can be switched to a warning and the robot can resync on distinctive junctions.

A junction only counts if an outer sensor sees it on **2 reads in a row**, to ignore one-off flickers.

## 5. The state machine

`Mission` is event-driven. The driving code reports what happened, and the mission answers what to do:

| Driving code reports | Mission answers |
|---|---|
| `on_junction(seen)` | `straight` / `left` / `right` / `stop` |
| `on_line_end()` | `collect` / `drop` |
| `on_job_done(polarity)` | `reverse` |

States: `TO_PICKUP → TO_DEPOT → TO_PICKUP …` until there are no boxes left or under 60 s remain, then `TO_HOME → DONE`. Any problem → `ERROR` and the motors stop.

North-pole boxes go to the red (left) upper depot, south-pole to the green (right) upper depot. If polarity is unknown (no hall sensor yet), the box goes to the green one.

## 6. Who does what

| File | Job |
|---|---|
| `state_machine.py` | Map, routes, decisions. No hardware, so it runs on a laptop. |
| `agv.py` | Motors and sensors: `follow_line`, `reverse_follow_line`, turning. |
| `line_following_test.py` | Drives the first full test (start → right pickup 3 → upper green depot → start) by asking `Mission` what to do. |
| `main_test.py` | The tested junction turn (`turn_at_junction`), reused above. |
| `pickup3_test.py` | Same route, but starting on the main line just past `right_depot_2` facing N. |
| `tests/route_test.py` | Laptop check of the map and routes. |

## 7. Leaving a branch: turn around, don't reverse

Reversing out of a branch didn't work (the sensors trail behind the wheels when reversing, so corrections made it worse). Instead the robot backs up briefly, spins round and drives out forwards. Full details, tuning and caveats: [turnaround_test.md](turnaround_test.md).

## 8. Adding the ramp

Add the ramp and upper-deck junctions to `GRAPH` in the same format, starting from `ramp_bottom`. Routes, turns and checks then work automatically.
