"""Command QR text -> list of actions. Pure Python, no ROS.

A command QR carries ONE text line:   <id>|<ACTION>|<arg>[|<ACTION>|<arg>...]

  C07|TURN_L|60      turn 60 deg to the left on the spot
  C08|TURN_R|90      turn 90 deg to the right
  C09|MOVE|1.0       drive 1.0 m straight ahead
  C10|WAIT|3         stand still 3 s
  C11|FOLLOW         no action: just a waypoint, go look for the next QR
  C12|STOP           end of the run
  C13|TURN_L|60|MOVE|1.0   (allowed) several actions in one QR, executed in order

The id (C07) must be UNIQUE in the room. The robot remembers the ids it has executed and
ignores them afterwards, so it never reads the same QR twice.
"""
import re
from dataclasses import dataclass, field

ID_RE = re.compile(r'^[A-Za-z]\d+$')

# name -> (takes an argument, min, max)
ACTIONS = {
    'TURN_L': (True, 1.0, 180.0),     # degrees
    'TURN_R': (True, 1.0, 180.0),     # degrees
    'MOVE': (True, 0.05, 5.0),        # metres
    'WAIT': (True, 0.1, 60.0),        # seconds
    'FOLLOW': (False, 0.0, 0.0),
    'STOP': (False, 0.0, 0.0),
}


@dataclass
class Command:
    cid: str
    actions: list = field(default_factory=list)    # [(name, value)]


def command_id(text):
    """First token of the QR text ('C07|TURN_L|60' -> 'C07'), '' if it is not a command id."""
    token = str(text).strip().split('|', 1)[0].strip()
    return token if ID_RE.match(token) else ''


def parse_command(text):
    """Parse the QR text. Raises ValueError with a readable reason when it is not valid."""
    parts = [p.strip() for p in str(text).strip().split('|')]
    cid = parts[0]
    if not ID_RE.match(cid):
        raise ValueError(f'bad command id {cid!r} (expected a letter and digits, e.g. C07)')
    rest = parts[1:]
    if not rest:
        raise ValueError(f'{cid}: no action')
    actions = []
    i = 0
    while i < len(rest):
        name = rest[i].upper()
        if name not in ACTIONS:
            raise ValueError(f'{cid}: unknown action {rest[i]!r} (use {", ".join(ACTIONS)})')
        needs_arg, lo, hi = ACTIONS[name]
        if needs_arg:
            if i + 1 >= len(rest):
                raise ValueError(f'{cid}: {name} needs a value')
            try:
                value = float(rest[i + 1].replace(',', '.'))
            except ValueError:
                raise ValueError(f'{cid}: {name} value {rest[i + 1]!r} is not a number') from None
            if not lo <= value <= hi:
                raise ValueError(f'{cid}: {name} value {value:g} outside {lo:g}..{hi:g}')
            actions.append((name, value))
            i += 2
        else:
            actions.append((name, 0.0))
            i += 1
    return Command(cid, actions)
