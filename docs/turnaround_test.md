# Pickup 3 → depot → home: how we got the test working

This covers the changes that made `pickup3_test.py` run end to end: right pickup 3, then the upper green depot, then back to the start box. It explains what we changed, why, and what still needs work.

## 1. Reversing out of a branch didn't work

The original plan was to collect the box, then **reverse back out of the branch** with line following until the sensors reached the main line, and pivot onto it.

This failed. The line sensors are at the front of the robot and the wheels are at the back. Going forwards, the sensors lead the wheels, so a correction steers the sensors back onto the line. In reverse, the sensors are effectively **behind the wheels**, like reversing a trailer:

- A correction moves the sensors towards the line for a moment.
- It also changes the direction of travel, which carries them further off the line straight after.

Every correction made the drift worse, and the weaker left wheel made the robot start off-course anyway. Once the sensors lost the line, the robot couldn't recover. Tuning didn't fix it.

## 2. What we do instead: turn around and drive out forwards

At the end of a branch the robot now turns around and leaves **forwards**, so it can use the normal line following and the normal junction turn that already worked. This is `turn_around()` in `line_following_test.py`.

### At a pickup
1. **Back up for 1 s** (`TURN_AROUND_BACK_MS`) with reverse line following. This gives room to spin, and over a short distance the reverse correction problem doesn't have time to build up.
2. **Spin left** at double turn speed. The first part is blind (`TURN_AROUND_BLIND_TIME`) to get clear of the line it was on.
3. **Keep spinning until the middle sensors leave the line, then until both are back on it.** The robot now faces back down the branch.
4. **Drive forwards** to the main line. The sensor bar is now parallel to the main line, so **all 4 sensors light up**. That's the signal to **turn right** towards home, using the normal `turn_at_junction`.
5. **Skip one junction:** the turn out of the pickup branch carries the robot past the next junction, `right_pickup_2`. So the code marks that junction as already passed, and the next one it looks for is `right_pickup_1`. We found this in testing.

### At a depot
- **No backing up.** Depot branch lines are much shorter, so it spins straight away.
- **The spin lands it straight on the main line, facing S.** The code counts the depot junction and its right turn as already done, and the next junction it looks for is `right_depot_1`.

The mission's compass heading is flipped after each turn-around, for example from W to E, so the map checks keep working.

## 3. Turn speed doubled, so the blind time is halved

The turn-around was very slow at the normal `TURN_SPEED` (40), so **turn-arounds spin at double speed** (`TURN_AROUND_SPEED = TURN_SPEED * 2`). Normal junction turns are unchanged.

Spinning twice as fast covers twice the angle in the same time. So the **blind part of the spin**, before the sensors' readings count, is **halved to match**: `TURN_AROUND_BLIND_TIME = TURN_TIME * 1.5 / 2`. Without that, the blind spin would overshoot past the line and the robot would end up facing the wrong way.

## 4. The sensors are read with no delay during the spin

Everywhere else, the sensors are read every 10 ms. At the double spin speed, both middle sensors are only on the line for a few milliseconds. With a 10 ms gap between reads, the robot **missed the line altogether**: it would almost always see one middle sensor white and one black, and spin straight past.

During the turn-around's sensor loops, the sensors are now read **as fast as the board can loop**, with no sleep. Other loops still use 10 ms.

## 5. Other changes made along the way

| Change | Why |
|---|---|
| `LEFT_SPEED_MULTIPLIER = 1.2` in `hardware.py` | The left wheel is weaker. Every left-wheel speed is multiplied by 1.2, capped at 100%. |
| `CORNER_LATENCY` 1.67 → **1.5** | The robot was overshooting junction turns. |
| `DEPOT_CORNER_LATENCY = 1.4` | Depot junctions needed a shorter forward walk than the other corners. |
| Line following during the corner walk (`turn_at_junction`) | Keeps the robot straight while it drives forward to put the wheels over the junction. |
| Junction recheck (`RECHECK_MS = 400`) | If a junction doesn't look like the map expects, keep going for up to 400 ms and re-read. Sides seen at different times add up: right then left counts as "both". This handles crossing a line at a slight angle. |
| Start-box follow (`START_FOLLOW_MS = 2000`) | Line follow out of the start box before looking for junctions. |

All the tunable numbers are now in `constants.py`. The old reversing code (`reverse_to_main_line`, `pivot_to_line`, `tests/reverse_test.py` and the `REVERSE_*` constants) has been removed.

## 6. Caveats: what might break

- **Not yet tested with a box.** With a box on board:
  - the robot is heavier, which changes how far it moves in the 1 s back-up and how fast it spins, and makes the weak left wheel lag more;
  - the box makes the robot wider, so it may hit the rack or other boxes while spinning;
  - the blind spin time and corner latencies may need re-tuning.
- **The exits from branches are hard-coded from what we saw on this route.** Skipping `right_pickup_2` after pickup 3, and landing on the main line facing S after the depot, both depend on where the robot happens to end up after these particular turns. Other pickups, the left side of the arena, or a change in speed or battery level may need these exit rules tweaked. If the mission and the robot get one junction out of step, check these two rules first.
- **Battery level matters.** All the timing (back-up, blind spin, corner latency) is time-based, so a weaker battery moves the robot less in the same time.
