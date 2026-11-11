# AGV state-machine plan

## The idea in plain English

The robot needs to remember more than one thing at a time.  For example,
while it is travelling, it might be:

- currently at the left-bottom rack;
- travelling to the left drop-off;
- carrying a north-pole box; and
- expecting to see the next T-junction.

Putting all of that into one state such as `"left-bottom-rack-to-left-drop-off-with-north-box"` would quickly create too many states.  Instead, keep one small `RobotState` object containing separate pieces of information.

The map areas are **locations**, not all behaviour states.  `SCANNING_RACK` and `DELIVERING` are behaviour states: they describe what the robot is doing wherever it happens to be.

```text
RobotState
├── location       Where the robot last recognised itself to be
├── heading        Which way it is facing at that location
├── mode           What the robot is doing now
├── target         The location it is trying to reach
├── cargo_polarity The detected pole of the box, or no box
└── rack_slot      The slot currently being checked in a rack
```

## Names to use

### Map locations

```text
START
LEFT_DROP_OFF
LEFT_BOTTOM_RACK
RIGHT_DROP_OFF
RIGHT_BOTTOM_RACK
RAMP
LEFT_UPPER_RACK
RIGHT_UPPER_RACK
```

### Behaviour modes

```text
NAVIGATING       Follow a planned route to `target`.
SCANNING_RACK    Move through rack slots and check whether a box is present.
PICKING_UP       Perform the physical box collection action.
CHECKING_POLARITY Read the magnetic sensor and choose a depot.
DELIVERING       Perform the physical drop-off action.
WAITING_FOR_IR   Stop before the upper deck until the green-light signal arrives.
RECOVERING       Stop safely after an unexpected or ambiguous marker.
FINISHED         Stop after the required run is complete.
```

## One delivery cycle

This is the intended sequence.  The same sequence works for all four racks.

```text
NAVIGATING to a rack
        │ arrival marker detected
        ▼
SCANNING_RACK
        │ a box is found
        ▼
PICKING_UP
        │ box held
        ▼
CHECKING_POLARITY
        │ north                 │ south
        ▼                       ▼
target = LEFT_DROP_OFF   target = RIGHT_DROP_OFF
        │                       │
        └────── NAVIGATING ─────┘
                    │ depot marker detected
                    ▼
                DELIVERING
                    │ box released
                    ▼
       choose the next rack, then NAVIGATING
```

For upper racks, put `WAITING_FOR_IR` into the route at the finish line before the ramp/upper deck:

```text
arrival at upper-deck finish line → WAITING_FOR_IR
IR green signal                   → continue NAVIGATING
```

## What causes a transition?

A transition is a change in the state object caused by a reliable event.  Do not transition merely because a fixed amount of time has passed.  A turn that takes a little too long or a missed line would then put the robot on the wrong route.

| Sensor observation | Meaning for the state machine |
| --- | --- |
| Colour/line pattern for a branch or T-junction | The next planned route instruction can be carried out. |
| Unique colour marker at a rack | Confirm `location`; change from `NAVIGATING` to `SCANNING_RACK`. |
| Hall/box-presence sensor finds a box | Change from `SCANNING_RACK` to `PICKING_UP`. |
| Hall sensor finds no box | Increment `rack_slot` and inspect the next slot. |
| Magnet sensor returns north/south | Set `cargo_polarity`, choose the corresponding drop-off, and navigate there. |
| Unique colour marker at a depot | Confirm `location`; change from `NAVIGATING` to `DELIVERING`. |
| IR green signal | Leave `WAITING_FOR_IR` and resume the route. |
| Unexpected marker or inconsistent readings | Stop motors and enter `RECOVERING`. |

Sensor readings should be *debounced*: accept a landmark only after the same pattern has been observed for several consecutive reads.  This prevents one noisy colour reading from advancing the route.

## Routes are data, not lots of special-case code

Each route should be a small list of expected landmarks and actions.  For example, the real marker names must be filled in after testing the course.

```text
START → LEFT_BOTTOM_RACK
  1. See START_EXIT marker       → follow line forward
  2. See first T-junction        → turn left
  3. See LEFT_BOTTOM_RACK marker → set location and start scanning
```

The navigation loop only needs to ask:

```text
Did I see the next expected marker?
  yes → perform its action and advance to the next route step
  no  → continue line following
```

If it sees a marker which is not the expected next marker, stop and recover rather than guessing its position.

## Implementation shape

Keep low-level hardware code in `hardware.py`, motor primitives in `agv.py`, and put the decision-making state machine in a new `state_machine.py` when implementation begins.

```text
hardware.py       Pins and raw sensor readings
agv.py            forward / turn / stop / pick-up actions
state_machine.py  RobotState, routes, transition decisions
main.py           Create the AGV and run the state-machine loop
```

The state module uses Python's `Enum` and `dataclass` features.  Enums prevent invalid or misspelled state values, while the dataclass provides the constructor and readable representation of `RobotState`.  Confirm that the Pico firmware includes the `enum` and `dataclasses` modules before uploading; the project now depends on them.

```python
# state_machine.py

from dataclasses import dataclass
from enum import Enum, auto


class Location(Enum):
    START = auto()
    LEFT_DROP_OFF = auto()
    LEFT_BOTTOM_RACK = auto()


class Mode(Enum):
    NAVIGATING = auto()
    SCANNING_RACK = auto()
    PICKING_UP = auto()


class Polarity(Enum):
    NORTH = auto()
    SOUTH = auto()


@dataclass
class RobotState:
    location: Location = Location.START
    mode: Mode = Mode.NAVIGATING
    target: Location = Location.LEFT_BOTTOM_RACK
    cargo_polarity: Polarity | None = None
    rack_slot: int = 0
```

## Build order

1. Define and print `RobotState`; replace the current `state="start"` string.
2. Finish raw sensor methods and calibrate the colour/line patterns for every landmark.
3. Build and test line following independently from map navigation.
4. Encode and test one route: `START → LEFT_BOTTOM_RACK`.
5. Add rack scanning and box/polarity detection.
6. Add both delivery routes, then repeat for the remaining racks.
7. Add IR waiting, debounce checks, recovery behaviour, and safe stopping.

## Course-specific decisions still needed

Before encoding routes, record these for every marker on the physical course:

- the colour/line pattern that identifies it;
- which direction to turn or continue;
- the next expected marker; and
- whether it is distinctive enough to confirm the robot's location.

This turns the physical map into a testable route table instead of scattered turn counters.
