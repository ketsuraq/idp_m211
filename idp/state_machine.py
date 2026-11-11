"""Named robot states and the data remembered by the navigation controller."""

from dataclasses import dataclass
from enum import Enum, auto


class Location(Enum):
    """Known places on the physical course."""

    START = auto()
    LEFT_DROP_OFF = auto()
    LEFT_BOTTOM_RACK = auto()
    RIGHT_DROP_OFF = auto()
    RIGHT_BOTTOM_RACK = auto()
    RAMP = auto()
    LEFT_UPPER_RACK = auto()
    RIGHT_UPPER_RACK = auto()


class Heading(Enum):
    """Directions the AGV can face after it confirms a location."""

    NORTH = auto()
    EAST = auto()
    SOUTH = auto()
    WEST = auto()


class Mode(Enum):
    """High-level activities that the AGV can currently perform."""

    NAVIGATING = auto()
    SCANNING_RACK = auto()
    PICKING_UP = auto()
    CHECKING_POLARITY = auto()
    DELIVERING = auto()
    WAITING_FOR_IR = auto()
    RECOVERING = auto()
    FINISHED = auto()


class Polarity(Enum):
    """Magnetic polarities that determine a box's drop-off location."""

    NORTH = auto()
    SOUTH = auto()


@dataclass
class RobotState:
    """Store the facts the AGV needs to make its next navigation decision."""

    location: Location = Location.START
    heading: Heading = Heading.NORTH
    mode: Mode = Mode.NAVIGATING
    target: Location = Location.LEFT_BOTTOM_RACK
    cargo_polarity: Polarity | None = None
    rack_slot: int = 0
