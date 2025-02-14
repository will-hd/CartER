"""
### types.py ###

Defines custom types used in networking protocol.
"""

from collections.abc import Callable
from typing import TypeVar

from commander.network.protocol import Packet

PacketT = TypeVar("PacketT", bound=Packet)
PacketSelector = Callable[[PacketT], bool]
